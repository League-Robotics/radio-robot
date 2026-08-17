// configurator_applyfield_harness.cpp -- ticket 132-012's own acceptance
// test (Generic applyField(target, field, value) setter + SetConfigField
// wire command, sprint 132 "configuration discipline"):
// Configurator::applyField() -- the single-field counterpart of
// applyGroup() (132-008), exercised at the SAME Configurator-API level
// configurator_applygroup_harness.cpp already established for the
// whole-group write direction.
//
// Covers, in one lightweight (no I2C bus, no sim plant) harness -- same
// construction shape as configurator_applygroup_harness.cpp/
// configure_entry_points_harness.cpp (132-007), which this harness's
// Configurator construction directly reuses:
//   - Boot-only classification: GEOMETRY returns ERR_NOT_LIVE without ever
//     touching config_ -- applyField() consults the SAME
//     isLiveConfigurable() gate applyGroup() uses, BEFORE any field lookup.
//   - NaN/Inf rejection: applyField() rejects a non-finite `value` via an
//     explicit std::isfinite() check that runs BEFORE
//     msg::wire::setField()/validateBounds() ever sees it -- and reports
//     ERR_BADARG (not ERR_RANGE), matching the-configuration-object.md's
//     own worked design (`if (!std::isfinite(value)) return
//     ErrCode::ERR_BADARG;`, checked ahead of the bounds check) -- distinct
//     from applyGroup()'s own NaN path (wire.cpp's validateBounds() itself
//     catches a NaN that arrived pre-encoded on the wire, reporting
//     ERR_RANGE there instead). Both paths share the SAME underlying
//     "NaN defeats bounds validation" fix (132-008); this ticket adds a
//     SECOND, earlier guard for the single-field path where the value
//     arrives as a plain float argument, not wire bytes.
//   - Unknown field number: an out-of-schema field number on a live target
//     is rejected with ERR_BADARG, config_ untouched.
//   - Out-of-bounds rejection: a value that violates a field's declared
//     (min)/(max)/(abs_max) is rejected with ERR_RANGE, config_ untouched
//     -- reusing the SAME validateBounds() applyGroup()'s decode path
//     uses, via msg::wire::setField() (wire.cpp's 132-012 addition).
//   - Valid single-field push: writes into config_ at the correct offset
//     AND reaches install(target) -- verified behaviorally (Drive::tick()'s
//     written duty for a DRIVE field, controlGains() for a WHEEL_CONTROL
//     field), not just that config_'s own member changed.
//   - install(target) reuse: the SAME per-target effects applyGroup()
//     already established (MOTORS's per-side ERR_BUSY guard, ESTIMATOR's
//     permanent ERR_UNIMPLEMENTED) are reachable through applyField() too,
//     since both call the identical install(target) on success.
//
// Hand-rolled assertions, PASS/FAIL per scenario, nonzero exit on any
// failure -- mirrors every other src/tests/sim/unit harness's own shape.
// Run by test_configurator_applyfield.py.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>

#include "core/boot_calibration.h"
#include "core/configurator.h"
#include "control/differential_drive.h"
#include "config/boot_config.h"
#include "config/robot.h"
#include "hal/clock.h"
#include "hal/motor.h"
#include "hardware/generic/real_otos.h"
#include "firm/types/robot_state.h"
#include "messages/envelope.h"
#include "messages/robot_config.h"

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

void checkEq(msg::ErrCode actual, msg::ErrCode expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected ErrCode %d, got %d", what.c_str(),
                  static_cast<int>(expected), static_cast<int>(actual));
    fail(buf);
  }
}

void checkFloatEq(float actual, float expected, const std::string& what, float tol = 1e-4f) {
  if (std::fabs(actual - expected) > tol) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %g, got %g", what.c_str(),
                  static_cast<double>(expected), static_cast<double>(actual));
    fail(buf);
  }
}

// --- Hal::Motor / Hal::Otos test doubles ----------------------------
// Same shape as configurator_applygroup_harness.cpp's own RecordingMotor/
// RecordingOtos (132-008), which itself mirrors configure_entry_points_
// harness.cpp's originals (132-007) -- duplicated rather than shared,
// matching this project's "each harness compiles ad hoc, no shared
// fixture" convention (src/tests/CLAUDE.md).

class RecordingMotor : public Hal::Motor {
 public:
  void begin() override {}
  void requestSample() override {}
  void setDuty(float duty) override { lastDuty = duty; }
  void setNeutral(Hal::Neutral) override {}
  // Records the emergency zero so a test can assert it actually
  // happened -- a mock that silently swallowed it would make the
  // sentinel's whole point untestable.
  void emergencyStop() override { ++emergencyStopCount; }
  int emergencyStopCount = 0;
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
  float stagedVelocity = 0.0f;     // [mm/s]
  float stagedAppliedDuty = 0.0f;  // [-1, 1]

  // --- recorded outputs ---
  float lastDuty = 0.0f;
  // lastTravelCalib -- DELETED (exploratory-kernel rewrite, 2026-08-15):
  // applyTravelCalib() no longer exists on Hal::Motor.
};

class StubClock : public Hal::Clock {
 public:
  uint64_t nowMicros() const override { return 0; }
};

class StubSleeper : public Hal::Sleeper {
 public:
  void sleepMillis(uint32_t) override {}
  void yield() override {}
};

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
  std::printf("=== 132-012 Configurator::applyField() ===\n\n");

  RecordingMotor motorL, motorR;
  RecordingOtos otos;
  StubClock clock;
  StubSleeper sleeper;
  Control::DifferentialDrive drive(motorL, motorR, clock, sleeper);
  Core::Configurator configurator(drive, motorL, motorR, otos, /*tuningStore=*/nullptr);

  // config_ starts default-constructed (all-zero) -- applyField() is
  // exercised directly, without loadBaked(), so every scenario's
  // before/after comparison is against a known, all-zero baseline rather
  // than whatever the active robot JSON happens to bake.

  // --- Boot-only classification -------------------------------------------

  beginScenario("GEOMETRY is boot-only: applyField() returns ERR_NOT_LIVE without touching config_");
  {
    const msg::ErrCode result =
        configurator.applyField(msg::ConfigGroupTarget::GEOMETRY, /*fieldNumber=*/1, /*value=*/500.0f);
    checkEq(result, msg::ErrCode::ERR_NOT_LIVE, "applyField(GEOMETRY, trackwidth=500) result");
    checkFloatEq(configurator.config().geometry.trackwidth, 0.0f,
                "config().geometry.trackwidth unchanged (still default)");
  }

  beginScenario("PLANNER is boot-only: applyField() returns ERR_NOT_LIVE without touching config_");
  {
    const msg::ErrCode result =
        configurator.applyField(msg::ConfigGroupTarget::PLANNER, /*fieldNumber=*/1, /*value=*/999.0f);
    checkEq(result, msg::ErrCode::ERR_NOT_LIVE, "applyField(PLANNER, v_max=999) result");
    checkFloatEq(configurator.config().planner.v_max, 0.0f,
                "config().planner.v_max unchanged (still default)");
  }

  // --- NaN / Inf rejection -- BEFORE validateBounds(), ERR_BADARG ---------

  beginScenario(
      "applyField() rejects NaN via an explicit isfinite() check that runs "
      "BEFORE validateBounds() -- ERR_BADARG (not ERR_RANGE, the "
      "the-configuration-object.md worked-design distinction), config_ "
      "untouched");
  {
    // Land a known-good value first, so "untouched" below means "still the
    // good push's value", not "still zero" (a weaker claim a bug zeroing
    // the field on failure could accidentally satisfy).
    const msg::ErrCode goodResult = configurator.applyField(
        msg::ConfigGroupTarget::DRIVE, /*fieldNumber=*/4 /*wheel_gain_left_accel*/, 2.0f);
    checkEq(goodResult, msg::ErrCode::ERR_NONE, "the good DRIVE push itself must succeed");

    const float nan = std::nanf("");
    const msg::ErrCode nanResult =
        configurator.applyField(msg::ConfigGroupTarget::DRIVE, /*fieldNumber=*/4, nan);
    checkEq(nanResult, msg::ErrCode::ERR_BADARG, "applyField(DRIVE, wheel_gain_left_accel=NaN) result");
    checkFloatEq(configurator.config().drive.wheel_gain_left_accel, 2.0f,
                "config().drive.wheel_gain_left_accel still the GOOD push's value, not NaN");
  }

  beginScenario(
      "applyField() rejects +/-infinity the SAME way it rejects NaN -- "
      "std::isfinite() catches both, ERR_BADARG, config_ untouched");
  {
    const float posInf = std::numeric_limits<float>::infinity();
    const msg::ErrCode result =
        configurator.applyField(msg::ConfigGroupTarget::DRIVE, /*fieldNumber=*/4, posInf);
    checkEq(result, msg::ErrCode::ERR_BADARG, "applyField(DRIVE, wheel_gain_left_accel=+inf) result");
    checkFloatEq(configurator.config().drive.wheel_gain_left_accel, 2.0f,
                "config().drive.wheel_gain_left_accel still the last GOOD push's value");
  }

  // --- Unknown field number -------------------------------------------------

  beginScenario("applyField() rejects an unknown field number with ERR_BADARG, config_ untouched");
  {
    const msg::ErrCode result =
        configurator.applyField(msg::ConfigGroupTarget::DRIVE, /*fieldNumber=*/999, 1.0f);
    checkEq(result, msg::ErrCode::ERR_BADARG, "applyField(DRIVE, unknown field 999) result");
    checkFloatEq(configurator.config().drive.wheel_gain_left_accel, 2.0f,
                "config().drive.wheel_gain_left_accel still the last GOOD push's value");
  }

  // --- Out-of-bounds rejection ----------------------------------------------

  beginScenario(
      "applyField() rejects an out-of-bounds value with ERR_RANGE (reusing "
      "the SAME validateBounds() applyGroup()'s decode path uses) -- "
      "guards exactly the divide-by-zero/sign-inversion hazard "
      "correctedCommand() (drive.cpp) has if a zero/negative gain ever "
      "landed in config_");
  {
    // wheel_gain_left_accel's own declared bound is (min) = 0.0001 -- 0.0
    // violates it.
    const msg::ErrCode result =
        configurator.applyField(msg::ConfigGroupTarget::DRIVE, /*fieldNumber=*/4, 0.0f);
    checkEq(result, msg::ErrCode::ERR_RANGE, "applyField(DRIVE, wheel_gain_left_accel=0.0) result");
    checkFloatEq(configurator.config().drive.wheel_gain_left_accel, 2.0f,
                "config().drive.wheel_gain_left_accel still the last GOOD push's value, not 0.0");
  }

  // --- Valid single-field push: correct offset + install(target) reached ---

  beginScenario(
      "A valid DRIVE field push writes config_ at the correct offset AND "
      "reaches Drive::configure() via install(target) -- Stage A wheel "
      "correction changes tick()'s written duty, matching "
      "configurator_applygroup_harness.cpp's own DRIVE scenario");
  {
    const msg::ErrCode result =
        configurator.applyField(msg::ConfigGroupTarget::DRIVE, /*fieldNumber=*/4, 4.0f);
    checkEq(result, msg::ErrCode::ERR_NONE, "applyField(DRIVE, wheel_gain_left_accel=4.0) result");
    checkFloatEq(configurator.config().drive.wheel_gain_left_accel, 4.0f,
                "config().drive.wheel_gain_left_accel reflects the push -- CORRECT OFFSET, not a "
                "neighboring field");
    // Every OTHER Drive field is untouched by this single-field push.
    checkFloatEq(configurator.config().drive.duty_per_speed_left, 0.0f,
                "config().drive.duty_per_speed_left untouched by a wheel_gain_left_accel push");
    checkFloatEq(configurator.config().drive.wheel_gain_right_accel, 0.0f,
                "config().drive.wheel_gain_right_accel untouched by a wheel_gain_left_accel push");

    // EXPLORATORY-KERNEL REWRITE (2026-08-15): the deeper tick()-level
    // verification this scenario used to run is GONE -- the kernel has no
    // tick(state)/cmdVelocity/setDutyPerSpeed() surface any more. Verified
    // instead at the kernel's own config() level, the same way
    // configurator_applygroup_harness.cpp's DRIVE scenario now does.
    checkFloatEq(drive.config().wheelGain[0][0], 4.0f,
                "drive.config().wheelGain[0][0] reflects the push -- install(target) was "
                "actually reached, not just config_ written");
  }

  beginScenario(
      "A valid WHEEL_CONTROL field push writes the CORRECT GROUP member "
      "(not DRIVE's) and reaches the kernel's own config() via "
      "installDriveKernelConfig() -- confirms applyField() indexes config_ "
      "by target correctly across more than one group");
  {
    const msg::ErrCode result =
        configurator.applyField(msg::ConfigGroupTarget::WHEEL_CONTROL, /*fieldNumber=*/7 /*pid_kp*/, 0.33f);
    checkEq(result, msg::ErrCode::ERR_NONE, "applyField(WHEEL_CONTROL, pid_kp=0.33) result");
    checkFloatEq(configurator.config().wheelControl.pid_kp, 0.33f,
                "config().wheelControl.pid_kp reflects the push");
    checkFloatEq(configurator.config().drive.wheel_gain_left_accel, 4.0f,
                "config().drive.wheel_gain_left_accel UNCHANGED by a WHEEL_CONTROL push -- "
                "confirms the two groups are not aliased");

    checkFloatEq(drive.config().kp, 0.33f,
                "drive.config().kp reflects the WHEEL_CONTROL push via install(target)");
  }

  // --- install(target) reuse: same per-target effects applyGroup() has ----
  //
  // EXPLORATORY-KERNEL REWRITE (2026-08-15): MOTORS no longer applies
  // travel_calib onto the motor leaf -- it feeds
  // Core::buildDriveKernelConfig() instead, via the SAME
  // installDriveKernelConfig() call DRIVE/WHEEL_CONTROL use, which is
  // exactly as live-safe as those two already were. The per-side "refuses
  // while moving" ERR_BUSY guard (Core::configureMotor(), 132-007) has no
  // motion-safety reason to exist any more -- install(MOTORS) now always
  // returns ERR_NONE, moving or not, through applyField() the same as
  // through applyGroup().

  beginScenario(
      "MOTORS field push applies regardless of motion, ERR_NONE -- "
      "install(target)'s per-side ERR_BUSY guard is GONE, reached the "
      "SAME way through applyField() as it is through applyGroup()");
  {
    motorL.stagedVelocity = 100.0f;  // [mm/s] -- deliberately IN MOTION; no guard reads this any more
    motorL.stagedAppliedDuty = 0.3f;

    const msg::ErrCode result = configurator.applyField(
        msg::ConfigGroupTarget::MOTORS, /*fieldNumber=*/1 /*travel_calib_left*/, 0.81f);
    checkEq(result, msg::ErrCode::ERR_NONE,
           "applyField(MOTORS, travel_calib_left) result, left side in motion -- ERR_NONE, "
           "no guard left to trip");
    checkFloatEq(configurator.config().motors.travel_calib_left, 0.81f,
                "config().motors.travel_calib_left reflects the write");
  }

  beginScenario(
      "ESTIMATOR field push writes config_ (read-back stays honest) but "
      "applyField() itself returns ERR_UNIMPLEMENTED, not ERR_NONE -- "
      "install(ESTIMATOR)'s own PERMANENT dead end (132-010) is reached "
      "the SAME way through applyField() as through applyGroup()");
  {
    const msg::ErrCode result = configurator.applyField(
        msg::ConfigGroupTarget::ESTIMATOR, /*fieldNumber=*/1 /*weight_heading_otos*/, 0.7f);
    checkEq(result, msg::ErrCode::ERR_UNIMPLEMENTED, "applyField(ESTIMATOR, weight_heading_otos) result");
    checkFloatEq(configurator.config().estimator.weight_heading_otos, 0.7f,
                "config().estimator.weight_heading_otos reflects the push despite ERR_UNIMPLEMENTED "
                "-- read-back stays honest even though there is nothing to fan the value out to");
  }

  std::printf("\n");
  if (g_failureCount == 0) {
    std::printf(
        "OK: applyField() classifies boot-only vs. live correctly, rejects "
        "NaN/Inf via isfinite() BEFORE validateBounds() (ERR_BADARG, "
        "distinct from applyGroup()'s own ERR_RANGE NaN path), rejects "
        "unknown fields (ERR_BADARG) and out-of-bounds values (ERR_RANGE), "
        "writes valid pushes at the correct offset, and reaches "
        "install(target) with the SAME per-target effects (MOTORS ERR_BUSY, "
        "ESTIMATOR ERR_UNIMPLEMENTED) applyGroup() already established\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the 132-012 applyField() tests\n", g_failureCount);
  return 1;
}
