// configure_entry_points_harness.cpp -- ticket 132-007's own acceptance
// test (the-configuration-object.md, sprint 132 "configuration
// discipline"): the six Config::Robot-consuming "subsystems take the
// whole object" entry points, plus Config::Robot's derived-value methods.
//
// Covers, in one lightweight (no I2C bus, no sim plant) harness:
//   - Config::Robot::effectiveTrackWidth()/rotationOffsetPos()/
//     rotationOffsetNeg()/velocityFilterWeight() (config/robot.h) --
//     pure functions of the object's own raw fields, matching
//     boot_calibration.cpp's :25-29/:63-65/:77-81 formulas bit-for-bit.
//   - Drive::configure() -- applies Stage A/B/C via EXISTING setters;
//     verified both directly (controlGains()/adaptationBounds() getters)
//     and behaviorally (tick()'s written duty changes with
//     config.drive.wheel_gain_left_accel, proving setWheelCorrection()
//     actually took effect, not just that the setter was reachable).
//   - App::configurePlanner()/configureMotor()/configureOtos()
//     (boot_calibration.h) -- the three Config::Robot-consuming adapters
//     for the subsystems that cannot take Config::Robot& as a member
//     method themselves (src/firm/motion/planner/'s own narrower dependency
//     rule; the devices isolation invariant) -- see boot_calibration.h's
//     own doc comment for the full rationale, and 132-007's own ticket
//     file for why RobotLoop's own configure() (the sixth entry point)
//     is instead covered by a new scenario in
//     configurator_loadbaked_harness.cpp (it needs the full composition
//     root; these five do not).
//
// Hand-rolled assertions, PASS/FAIL per scenario, nonzero exit on any
// failure -- mirrors every other src/tests/sim/unit harness's own shape.
// Run by test_configure_entry_points.py.
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <string>

#include "app/boot_calibration.h"
#include "app/drive.h"
#include "config/robot.h"
#include "hal/motor.h"
#include "hardware/generic/real_otos.h"
#include "firm/types/robot_state.h"
#include "motion/planner/planner.h"
#include "motion/planner/planner_types.h"

namespace {

int g_failureCount = 0;
std::string g_scenarioName;

void beginScenario(const std::string& name) {
  g_scenarioName = name;
  std::printf("--- %s\n", name.c_str());
}

void fail(const std::string& what) {
  ++g_failureCount;
  std::printf("  FAIL [%s]: %s\n", g_scenarioName.c_str(), what.c_str());
}

void checkTrue(bool condition, const std::string& what) {
  if (!condition) fail(what + " -- expected true, got false");
}

void checkFalse(bool condition, const std::string& what) {
  if (condition) fail(what + " -- expected false, got true");
}

void checkFloatEq(float actual, float expected, const std::string& what, float tol = 1e-4f) {
  if (std::fabs(actual - expected) > tol) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %g, got %g", what.c_str(),
                  static_cast<double>(expected), static_cast<double>(actual));
    fail(buf);
  }
}

// sampleConfig -- a Config::Robot with distinct, easily-recognizable
// values in every field this harness reads, so a mismatched read (wrong
// field, wrong group) fails loudly instead of accidentally matching a
// shared default.
Config::Robot sampleConfig() {
  Config::Robot config;

  config.geometry.trackwidth = 130.0f;          // [mm]
  config.geometry.rotational_slip = 0.9f;
  config.geometry.rotation_gain_pos = 1.05f;
  config.geometry.rotation_offset = -6.0f;       // [deg]
  config.geometry.rotation_gain_neg = 1.06f;
  config.geometry.rotation_offset_neg = -7.0f;   // [deg]

  config.motors.travel_calib_left = 0.71f;   // [mm/deg]
  config.motors.travel_calib_right = 0.70f;  // [mm/deg]
  config.motors.vel_filt_alpha = 0.3f;

  config.drive.wheel_gain_left_accel = 1.0f;
  config.drive.wheel_intercept_left_accel = 0.0f;
  config.drive.wheel_gain_left_decel = 1.0f;
  config.drive.wheel_intercept_left_decel = 0.0f;
  config.drive.wheel_gain_right_accel = 1.0f;
  config.drive.wheel_intercept_right_accel = 0.0f;
  config.drive.wheel_gain_right_decel = 1.0f;
  config.drive.wheel_intercept_right_decel = 0.0f;
  config.drive.crawl_pulse = 0.0f;

  config.wheelControl.pid_kp = 0.11f;
  config.wheelControl.pid_ki = 0.22f;
  config.wheelControl.pid_i_max = 0.33f;
  config.wheelControl.pid_kaff = 0.44f;
  config.wheelControl.pid_max = 0.55f;
  config.wheelControl.v_min = 66.0f;
  config.wheelControl.bias_max = 77.0f;
  config.wheelControl.tau_adapt = 8.0f;
  config.wheelControl.a_steady = 99.0f;
  config.wheelControl.pos_err_max = 12.0f;  // [mm] 133-002
  config.wheelControl.deficit_threshold = 10.0f;
  config.wheelControl.deficit_window = 200.0f;

  // 132-017 split: the six shaper ceilings live on plannerShaper now, not
  // planner (the boot-only remainder) -- see robot_config.proto's
  // PlannerShaper message header comment.
  config.plannerShaper.a_max = 301.0f;
  config.plannerShaper.a_decel = 251.0f;
  config.plannerShaper.alpha_max = 6.1f;
  config.plannerShaper.alpha_decel = 5.1f;
  config.plannerShaper.jerk_max = 1501.0f;
  config.plannerShaper.yaw_jerk_max = 31.0f;

  config.otos.offset_x = -48.0f;    // [mm]
  config.otos.offset_y = 4.0f;      // [mm]
  config.otos.offset_yaw = 0.01f;   // [rad]
  config.otos.linear_scale = 1.03f;
  config.otos.angular_scale = 0.98f;

  return config;
}

// RecordingMotor -- a minimal Hal::Motor test double: records the
// last setDuty()/applyTravelCalib() call, and reports whatever
// velocity()/appliedDuty() the scenario stages, so configureMotor()'s
// "is moving" guard can be exercised in both directions.
class RecordingMotor : public Hal::Motor {
 public:
  void begin() override {}
  void requestSample() override {}
  void setDuty(float duty) override { lastDuty = duty; }
  void setNeutral(Hal::Neutral) override {}
  void applyTravelCalib(float travelCalib) override { lastTravelCalib = travelCalib; }
  [[nodiscard]] bool reconfigure(const Hal::MotorConfig&) override { return true; }
  void tick(uint64_t) override {}

  float position() const override { return 0.0f; }
  float velocity() const override { return stagedVelocity; }
  float appliedDuty() const override { return stagedAppliedDuty; }
  bool connected() const override { return true; }
  uint64_t sampleTime() const override { return 0; }

  void resetPosition() override {}
  void rebaseline() override {}

  // --- scenario-staged inputs ---
  float stagedVelocity = 0.0f;      // [mm/s]
  float stagedAppliedDuty = 0.0f;   // [-1, 1]

  // --- recorded outputs ---
  float lastDuty = 0.0f;
  float lastTravelCalib = -1.0f;  // sentinel: configureMotor() must overwrite this to pass
};

// RecordingOtos -- a minimal Hal::Otos test double: records the
// scalars/offset configureOtos() installs.
class RecordingOtos : public Hal::Otos {
 public:
  void begin() override {}
  void tick(uint64_t) override {}
  Hal::PoseReading pose() const override { return {}; }
  bool poseFresh() const override { return false; }
  bool connected() const override { return true; }
  bool present() const override { return true; }
  uint64_t sampleTime() const override { return 0; }

  void setLinearScalar(float scalar) override { linearScalar = scalar; }
  void setAngularScalar(float scalar) override { angularScalar = scalar; }
  void setOffset(float x, float y, float heading) override {
    offsetX = x;
    offsetY = y;
    offsetHeading = heading;
  }
  void getOffset(float& x, float& y, float& heading) override {
    x = offsetX;
    y = offsetY;
    heading = offsetHeading;
  }
  void init() override {}

  float linearScalar = -1.0f;   // sentinel
  float angularScalar = -1.0f;  // sentinel
  float offsetX = -1.0f;        // [mm] sentinel
  float offsetY = -1.0f;        // [mm] sentinel
  float offsetHeading = -1.0f;  // [rad] sentinel
};

}  // namespace

int main() {
  std::printf("=== 132-007 configure() entry points + derived-value methods ===\n\n");

  const Config::Robot config = sampleConfig();
  constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;

  // --- Config::Robot derived-value methods -------------------------------

  beginScenario("effectiveTrackWidth() divides by rotational_slip when calibrated");
  checkFloatEq(config.effectiveTrackWidth(), 130.0f / 0.9f, "effectiveTrackWidth()");

  beginScenario("effectiveTrackWidth() returns the raw trackwidth when slip == 0 (uncalibrated)");
  {
    Config::Robot uncalibrated = config;
    uncalibrated.geometry.rotational_slip = 0.0f;
    checkFloatEq(uncalibrated.effectiveTrackWidth(), uncalibrated.geometry.trackwidth,
                "effectiveTrackWidth() with slip == 0");
  }

  beginScenario("rotationOffsetPos()/rotationOffsetNeg() convert degrees to radians");
  checkFloatEq(config.rotationOffsetPos(), -6.0f * kDegToRad, "rotationOffsetPos()");
  checkFloatEq(config.rotationOffsetNeg(), -7.0f * kDegToRad, "rotationOffsetNeg()");

  beginScenario("velocityFilterWeight() passes through motors.vel_filt_alpha above the 0.05 floor");
  checkFloatEq(config.velocityFilterWeight(), 0.3f, "velocityFilterWeight() above floor");

  beginScenario("velocityFilterWeight() floors to 1.0 at/below 0.05");
  {
    Config::Robot lowAlpha = config;
    lowAlpha.motors.vel_filt_alpha = 0.05f;
    checkFloatEq(lowAlpha.velocityFilterWeight(), 1.0f, "velocityFilterWeight() at floor (0.05)");
    lowAlpha.motors.vel_filt_alpha = 0.0f;
    checkFloatEq(lowAlpha.velocityFilterWeight(), 1.0f, "velocityFilterWeight() below floor (0.0)");
  }

  // --- App::Drive::configure() -------------------------------------------

  beginScenario("Drive::configure() installs Stage B/C via setControlGains()/setAdaptationBounds()");
  {
    RecordingMotor left, right;
    App::Drive drive(left, right, /*trackWidth=*/128.0f);
    drive.configure(config);

    const App::Drive::ControlGains& gains = drive.controlGains();
    checkFloatEq(gains.kp, config.wheelControl.pid_kp, "controlGains().kp");
    checkFloatEq(gains.ki, config.wheelControl.pid_ki, "controlGains().ki");
    checkFloatEq(gains.iMax, config.wheelControl.pid_i_max, "controlGains().iMax");
    checkFloatEq(gains.kaff, config.wheelControl.pid_kaff, "controlGains().kaff");
    checkFloatEq(gains.pidMax, config.wheelControl.pid_max, "controlGains().pidMax");

    const App::Drive::AdaptationBounds& bounds = drive.adaptationBounds();
    checkFloatEq(bounds.vMin, config.wheelControl.v_min, "adaptationBounds().vMin");
    checkFloatEq(bounds.biasMax, config.wheelControl.bias_max, "adaptationBounds().biasMax");
    checkFloatEq(bounds.tauAdapt, config.wheelControl.tau_adapt, "adaptationBounds().tauAdapt");
    checkFloatEq(bounds.aSteady, config.wheelControl.a_steady, "adaptationBounds().aSteady");
    // 133-002: pos_err_max's BAKE path lands here. Asserted right beside
    // pid_i_max above because the two clamp DIFFERENT domains -- posErrMax
    // is [mm] on Stage B's input, iMax is [mm/s] on its output -- and both
    // must arrive, independently, from the same file.
    checkFloatEq(bounds.posErrMax, config.wheelControl.pos_err_max,
                "adaptationBounds().posErrMax [mm] -- distinct from controlGains().iMax [mm/s]");
    checkFloatEq(bounds.deficitThreshold, config.wheelControl.deficit_threshold,
                "adaptationBounds().deficitThreshold");
    checkFloatEq(bounds.deficitWindow, config.wheelControl.deficit_window,
                "adaptationBounds().deficitWindow");
  }

  beginScenario(
      "Drive::configure() installs Stage A wheel correction via setWheelCorrection() "
      "(behavioral: tick()'s written duty changes with wheel_gain_left_accel)");
  {
    // A ZERO wheelControl group -- Stage B/C are the previous scenario's own
    // job; this one isolates Stage A alone, so tick()'s written duty is
    // exactly correctedCommand() * dutyPerSpeed with no PID/bias/crawl
    // contribution to disentangle.
    Config::Robot isolatedConfig = config;
    isolatedConfig.wheelControl = msg::WheelControl{};
    isolatedConfig.drive.crawl_pulse = 0.0f;

    Types::RobotState state;
    state.wheelLeft.cmdVelocity = 200.0f;   // [mm/s]
    state.wheelRight.cmdVelocity = 0.0f;
    state.time.cyclePeriod = 50000;         // [us]

    RecordingMotor leftA, rightA;
    App::Drive driveA(leftA, rightA, /*trackWidth=*/128.0f);
    Config::Robot configGain1 = isolatedConfig;
    configGain1.drive.wheel_gain_left_accel = 1.0f;
    driveA.configure(configGain1);
    driveA.setDutyPerSpeed(1.0f, 1.0f);  // isolate Stage A: duty == correctedCommand()
    driveA.tick(state);
    checkFloatEq(leftA.lastDuty, 200.0f, "tick() duty with wheel_gain_left_accel == 1.0");

    RecordingMotor leftB, rightB;
    App::Drive driveB(leftB, rightB, /*trackWidth=*/128.0f);
    Config::Robot configGain2 = isolatedConfig;
    configGain2.drive.wheel_gain_left_accel = 2.0f;
    driveB.configure(configGain2);
    driveB.setDutyPerSpeed(1.0f, 1.0f);
    driveB.tick(state);
    checkFloatEq(leftB.lastDuty, 100.0f, "tick() duty with wheel_gain_left_accel == 2.0");
  }

  // --- App::configurePlanner() --------------------------------------------

  beginScenario("configurePlanner() installs shaper ceilings via applyShaperLimits()");
  {
    Motion::PlannerLimits limits;
    Motion::Planner planner(limits);
    checkFalse(planner.shaperConfigured(), "shaperConfigured() before configurePlanner()");

    App::configurePlanner(planner, config);

    checkTrue(planner.shaperConfigured(), "shaperConfigured() after configurePlanner()");
    checkFloatEq(planner.limits().ceilings.aMax, config.plannerShaper.a_max, "limits().ceilings.aMax");
    checkFloatEq(planner.limits().ceilings.aDecel, config.plannerShaper.a_decel,
                "limits().ceilings.aDecel");
    checkFloatEq(planner.limits().ceilings.alphaMax, config.plannerShaper.alpha_max,
                "limits().ceilings.alphaMax");
    checkFloatEq(planner.limits().ceilings.alphaDecel, config.plannerShaper.alpha_decel,
                "limits().ceilings.alphaDecel");
    checkFloatEq(planner.limits().ceilings.jerkMax, config.plannerShaper.jerk_max,
                "limits().ceilings.jerkMax");
    checkFloatEq(planner.limits().ceilings.yawJerkMax, config.plannerShaper.yaw_jerk_max,
                "limits().ceilings.yawJerkMax");
  }

  // --- App::configureMotor() ----------------------------------------------

  beginScenario("configureMotor() applies travel calib and returns true when the motor is at rest");
  {
    RecordingMotor motor;
    motor.stagedVelocity = 0.0f;
    motor.stagedAppliedDuty = 0.0f;
    const bool ok = App::configureMotor(motor, config, /*isLeft=*/true);
    checkTrue(ok, "configureMotor() return value, at rest, left side");
    checkFloatEq(motor.lastTravelCalib, config.motors.travel_calib_left,
                "applyTravelCalib() argument, left side");
  }
  {
    RecordingMotor motor;
    motor.stagedVelocity = 0.0f;
    motor.stagedAppliedDuty = 0.0f;
    const bool ok = App::configureMotor(motor, config, /*isLeft=*/false);
    checkTrue(ok, "configureMotor() return value, at rest, right side");
    checkFloatEq(motor.lastTravelCalib, config.motors.travel_calib_right,
                "applyTravelCalib() argument, right side");
  }

  beginScenario("configureMotor() refuses and applies nothing while the motor is moving");
  {
    RecordingMotor motor;
    motor.stagedVelocity = 100.0f;  // [mm/s], well above the rest threshold
    motor.stagedAppliedDuty = 0.3f;
    const bool ok = App::configureMotor(motor, config, /*isLeft=*/true);
    checkFalse(ok, "configureMotor() return value while moving");
    checkFloatEq(motor.lastTravelCalib, -1.0f,
                "applyTravelCalib() must NOT be called while the motor is moving");
  }

  // --- App::configureOtos() ------------------------------------------------

  beginScenario(
      "configureOtos() applies scalars (REGISTER-domain, via "
      "Hardware::scaleToRegister() -- trap 3 closed, 132-010) and offset "
      "(unconverted) via existing setters");
  {
    RecordingOtos otos;
    App::configureOtos(otos, config);
    checkFloatEq(otos.linearScalar, static_cast<float>(Hardware::scaleToRegister(config.otos.linear_scale)),
                "setLinearScalar() argument is REGISTER-domain, matching begin()'s own conversion");
    checkFloatEq(otos.angularScalar, static_cast<float>(Hardware::scaleToRegister(config.otos.angular_scale)),
                "setAngularScalar() argument is REGISTER-domain, matching begin()'s own conversion");
    checkFloatEq(otos.offsetX, config.otos.offset_x, "setOffset() x argument");
    checkFloatEq(otos.offsetY, config.otos.offset_y, "setOffset() y argument");
    checkFloatEq(otos.offsetHeading, config.otos.offset_yaw, "setOffset() heading argument");
  }

  std::printf("\n");
  if (g_failureCount == 0) {
    std::printf(
        "OK: derived-value methods and five of the six 132-007 configure() entry points "
        "(Drive, configurePlanner, configureMotor, configureOtos) behave as expected -- "
        "RobotLoop::configure() is covered separately by configurator_loadbaked_harness.cpp\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the 132-007 configure() entry point tests\n",
              g_failureCount);
  return 1;
}
