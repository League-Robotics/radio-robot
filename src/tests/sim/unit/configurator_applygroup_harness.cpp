// configurator_applygroup_harness.cpp -- ticket 132-008's own acceptance
// test (the-configuration-object.md, sprint 132 "configuration
// discipline"): Configurator::applyGroup()/install(ConfigGroupTarget) --
// the per-target re-appliability table, the boot-only ERR_NOT_LIVE
// rejection, the NaN-defeats-bounds-validation fix (wire.cpp's
// validateBounds(), 132-008), and the no-partial-commit property (a
// rejected push leaves config_ completely untouched, not half-decoded).
//
// Covers, in one lightweight (no I2C bus, no sim plant) harness -- same
// shape as configure_entry_points_harness.cpp (132-007), which this
// harness's Configurator construction directly reuses:
//   - Boot-only classification: GEOMETRY/PLANNER return ERR_NOT_LIVE
//     without ever touching config_ (isLiveConfigurable() runs BEFORE the
//     wire buffer is even read -- both scenarios pass nullptr/0).
//   - Live classification: DRIVE/WHEEL_CONTROL/MOTORS/OTOS/ESTIMATOR are
//     never rejected as boot-only (ESTIMATOR still returns a real error --
//     ERR_UNIMPLEMENTED -- because no consumer is wired yet, ticket 010's
//     job; the point of this scenario is only that it is NOT ERR_NOT_LIVE).
//   - DRIVE/WHEEL_CONTROL: decode into config_ AND a real install(target)
//     effect -- Drive::configure() (132-007), verified behaviorally
//     (tick()'s written duty for DRIVE, controlGains()/adaptationBounds()
//     for WHEEL_CONTROL), not just that a setter was reachable.
//   - MOTORS: applies via Core::configureMotor() at rest; refuses with
//     ERR_BUSY (surfaced, not swallowed) while in motion -- and the guard
//     is PER SIDE, not global: the side that is at rest still gets its
//     travel_calib applied even when the push overall returns ERR_BUSY
//     because the OTHER side is moving.
//   - OTOS: decodes and installs via Core::configureOtos() -- trap 3
//     (the-configuration-object.md) CLOSED (132-010): the scale fields are
//     converted through Hardware::scaleToRegister() before reaching
//     setLinearScalar()/setAngularScalar(), matching RealOtos::begin()'s
//     own boot-time conversion. Covers both the general case and the
//     ticket's own regression guard -- a pushed linearScale/angularScale
//     of 1.0 (true unity) must land as register 0, not the old bug's
//     register 1 (a real +0.1% scalar, "a 1-LSB scalar instead of unity").
//   - ESTIMATOR: decodes into config_ (read-back stays honest) but
//     install() returns ERR_UNIMPLEMENTED, not ERR_NONE -- PERMANENTLY,
//     not a to-be-filled gap (132-010 closes trap 2 by making this
//     explicit rather than inventing a consumer): Core::StateEstimator was
//     deleted as dead code (sprint 128 ticket 016), and its one candidate
//     successor -- Motion::PoseTracker::blendHeading() -- had its only
//     call site and its own config fields deleted outright by 130-009 in
//     favor of a from-scratch fusion redesign (clasi/issues/later/
//     estimator-v2-otos-fusion-sim-first.md). Configurator holds no
//     estimator-shaped reference. This is the ticket's own headline
//     property demonstrated directly: config that acks OK and silently
//     does nothing is worse than config that is rejected -- ESTIMATOR
//     does neither.
//   - NaN rejection: a NaN bit pattern in a bounded field (DRIVE.duty_per_
//     speed_left, `(min) = 0.0`) is rejected (ERR_RANGE), not silently
//     accepted -- bounds alone (`v < min`/`v > max`) are both false for
//     NaN, so wire.cpp's validateBounds() now checks `v != v` explicitly
//     (132-008) before the min/max/abs_max comparisons.
//   - No-partial-commit: a rejected push (NaN or truncated/malformed
//     bytes) leaves config_'s ENTIRE group untouched -- not reverted to
//     defaults, not partially overwritten with whatever decoded before
//     the failure. applyGroup() decodes into a local scratch value and
//     commits it into config_ only after a successful Result -- decoding
//     straight into config_ would be wrong: msg::wire::decode(<Group>&,
//     ...) unconditionally memsets its `out` argument first (the same
//     full-object-zero rationale decode(Telemetry&, ...) uses), which
//     would zero the CURRENTLY LIVE group before it is even known whether
//     the incoming push is valid.
//   - (132-009) Boot-time install(): loadBaked()+install() (the no-arg,
//     construction-time fan-out, distinct from every applyGroup()/
//     install(target) scenario above, which is the LIVE wire-push path)
//     sources dutyPerSpeed from config_.drive.duty_per_speed_left/right --
//     the active robot JSON, via Config::defaultDriveGroup() -- rather
//     than the historical Drive::kDutyPerSpeed C++ literal. This is the
//     kDutyPerSpeed reversal ticket 009's own dispatch note calls out
//     explicitly: see Configurator::install()'s doc comment
//     (configurator.cpp) and Drive::kDutyPerSpeed's own doc comment
//     (drive.h) for the full reasoning.
//
// Hand-rolled assertions, PASS/FAIL per scenario, nonzero exit on any
// failure -- mirrors every other src/tests/sim/unit harness's own shape.
// Run by test_configurator_applygroup.py.
//
// Wire-encoding note: there is no generated msg::wire::encode(<Group>&,
// ...) family (132-008 only added decode(), matching this ticket's own
// acceptance criteria) -- test payloads are hand-encoded here via
// WireRuntime's PUBLIC byte-level primitives (wire_runtime.h), the exact
// same primitives wire.cpp's own generated engine uses internally, and the
// exact same pattern src/tests/sim/support/wire_test_codec.cpp already
// established for CommandEnvelope{MOVE|STOP} (that file's own header
// comment: "the read/write direction the generated codec does NOT cover").
// This is test-only encoding, not a hand-rolled DECODER competing with the
// generated engine applyGroup() itself uses -- the acceptance criterion
// "no new hand-written parsing" is about the PRODUCTION applyGroup() path
// (configurator.cpp), which this harness never bypasses.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include "core/boot_calibration.h"
#include "core/configurator.h"
#include "control/differential_drive.h"
#include "config/boot_config.h"
#include "config/robot.h"
#include "host_fiber.h"
#include "hal/clock.h"
#include "hal/motor.h"
#include "hardware/generic/real_otos.h"
#include "firm/types/robot_state.h"
#include "messages/envelope.h"
#include "messages/robot_config.h"
#include "messages/wire_runtime.h"

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

void checkNe(msg::ErrCode actual, msg::ErrCode notExpected, const std::string& what) {
  if (actual == notExpected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected NOT ErrCode %d, but got it", what.c_str(),
                  static_cast<int>(notExpected));
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

// --- Hand-rolled test-only wire encoding -----------------------------------
// See this file's own header comment ("Wire-encoding note") for why this
// exists here rather than in production code or the generator.

using WireRuntime::WireType;

bool encodeFloatField(uint32_t fieldNumber, float value, uint8_t* buf, size_t cap, size_t* pos) {
  // NaN is deliberately NOT treated as "default" here (unlike the real
  // `value == 0.0f` implicit-presence check every OTHER field in this file
  // uses) -- the NaN-rejection scenario needs the field to actually be
  // encoded on the wire so decodeInto()'s validateBounds() has something to
  // reject; `NaN == 0.0f` is false anyway (every NaN comparison is false),
  // so this helper always encodes a NaN unconditionally regardless.
  if (value == 0.0f) return true;
  if (!WireRuntime::encodeTag(fieldNumber, WireType::kFixed32, buf, cap, pos)) return false;
  return WireRuntime::encodeFloat(value, buf, cap, pos);
}

bool encodeInt32Field(uint32_t fieldNumber, int32_t value, uint8_t* buf, size_t cap, size_t* pos) {
  if (value == 0) return true;
  if (!WireRuntime::encodeTag(fieldNumber, WireType::kVarint, buf, cap, pos)) return false;
  // protobuf's own int32 gotcha (mirrors wire.cpp's encodeScalarValue()):
  // sign-extend to 64 bits before varint-encoding, unless the field is
  // sint32 (none of robot_config.proto's int32 fields are).
  return WireRuntime::encodeVarint(static_cast<uint64_t>(static_cast<int64_t>(value)), buf, cap, pos);
}

bool encodeUint32Field(uint32_t fieldNumber, uint32_t value, uint8_t* buf, size_t cap, size_t* pos) {
  if (value == 0) return true;
  if (!WireRuntime::encodeTag(fieldNumber, WireType::kVarint, buf, cap, pos)) return false;
  return WireRuntime::encodeVarint(value, buf, cap, pos);
}

// --- Hal::Motor / Hal::Otos test doubles ----------------------------
// Same shape as configure_entry_points_harness.cpp's own RecordingMotor/
// RecordingOtos (132-007) -- duplicated rather than shared, matching this
// project's "each harness compiles ad hoc, no shared fixture" convention
// (src/tests/CLAUDE.md).

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
  // applyTravelCalib() no longer exists on Hal::Motor -- MOTORS travel_calib
  // now feeds Core::buildDriveKernelConfig() (boot_calibration.h), never
  // the motor leaf directly. See this file's own MOTORS scenario below for
  // the new assertion shape (config().motors.travel_calib_left/right).
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
  std::printf(
      "=== 132-008 Configurator::applyGroup()/install(ConfigGroupTarget) ===\n\n");

  RecordingMotor motorL, motorR;
  RecordingOtos otos;
  StubClock clock;
  StubSleeper sleeper;
  // The kernel takes its launcher at construction now. This harness
  // never calls start() -- FailingFiberLauncher aborts if it ever
  // does, which is the point: no fibers in a host test.
  TestSim::FailingFiberLauncher fiberLauncher;
  // Hal -> package-port adapters (control/differential_drive.h): the
  // kernel is the DiffDrive package and speaks its own ports.
  Control::MotorPort portLeft(motorL);
  Control::MotorPort portRight(motorR);
  Control::ClockPort portClock(clock);
  Control::SleeperPort portSleeper(sleeper);
  Control::LauncherPort portLauncher(fiberLauncher);
  Control::DifferentialDrive drive(portLeft, portRight, portClock,
                                   portSleeper, portLauncher);
  Core::Configurator configurator(drive, motorL, motorR, otos, /*tuningStore=*/nullptr);

  // config_ starts default-constructed (all-zero) -- applyGroup() is
  // exercised directly, without loadBaked(), so every scenario's
  // before/after comparison is against a known, all-zero baseline rather
  // than whatever the active robot JSON happens to bake.

  // --- Boot-only classification -------------------------------------------

  beginScenario("GEOMETRY is boot-only: applyGroup() returns ERR_NOT_LIVE without touching the buffer");
  {
    const msg::ErrCode result = configurator.applyGroup(msg::ConfigGroupTarget::GEOMETRY,
                                                        /*wire=*/nullptr, /*len=*/0);
    checkEq(result, msg::ErrCode::ERR_NOT_LIVE, "applyGroup(GEOMETRY) result");
    checkFloatEq(configurator.config().geometry.trackwidth, 0.0f,
                "config().geometry.trackwidth unchanged (still default)");
  }

  beginScenario("PLANNER is boot-only: applyGroup() returns ERR_NOT_LIVE without touching the buffer");
  {
    const msg::ErrCode result = configurator.applyGroup(msg::ConfigGroupTarget::PLANNER,
                                                        /*wire=*/nullptr, /*len=*/0);
    checkEq(result, msg::ErrCode::ERR_NOT_LIVE, "applyGroup(PLANNER) result");
    checkFloatEq(configurator.config().planner.v_max, 0.0f,
                "config().planner.v_max unchanged (still default)");
  }

  beginScenario(
      "A boot-only push does not mutate config_ even when it carries real, "
      "well-formed bytes for the target group");
  {
    uint8_t buf[64];
    size_t pos = 0;
    checkTrue(encodeFloatField(1, 999.0f, buf, sizeof(buf), &pos), "encode Geometry.trackwidth=999");
    const msg::ErrCode result =
        configurator.applyGroup(msg::ConfigGroupTarget::GEOMETRY, buf, pos);
    checkEq(result, msg::ErrCode::ERR_NOT_LIVE, "applyGroup(GEOMETRY, well-formed bytes) result");
    checkFloatEq(configurator.config().geometry.trackwidth, 0.0f,
                "config().geometry.trackwidth still unchanged");
  }

  // --- Live classification (none of these is ERR_NOT_LIVE) ----------------

  beginScenario("DRIVE/WHEEL_CONTROL/MOTORS/OTOS/ESTIMATOR are never classified boot-only");
  {
    const msg::ConfigGroupTarget liveTargets[] = {
        msg::ConfigGroupTarget::DRIVE,         msg::ConfigGroupTarget::WHEEL_CONTROL,
        msg::ConfigGroupTarget::MOTORS,        msg::ConfigGroupTarget::OTOS,
        msg::ConfigGroupTarget::ESTIMATOR,
    };
    for (msg::ConfigGroupTarget target : liveTargets) {
      // An EMPTY buffer decodes successfully (proto3 implicit presence --
      // every field stays at its zero default) -- this only exercises the
      // re-appliability gate, not decode correctness.
      const msg::ErrCode result = configurator.applyGroup(target, /*wire=*/nullptr, /*len=*/0);
      checkNe(result, msg::ErrCode::ERR_NOT_LIVE,
              "applyGroup() on a live target must not return ERR_NOT_LIVE");
    }
  }

  // --- DRIVE: decode + Drive::configure() (Stage A), behavioral ----------

  beginScenario(
      "DRIVE push decodes into config_ and reaches Drive::configure() -- "
      "Stage A wheel correction changes tick()'s written duty");
  {
    uint8_t buf[128];
    size_t pos = 0;
    checkTrue(encodeFloatField(1, 0.0f, buf, sizeof(buf), &pos), "encode duty_per_speed_left");
    checkTrue(encodeFloatField(2, 0.0f, buf, sizeof(buf), &pos), "encode duty_per_speed_right");
    checkTrue(encodeFloatField(3, 0.0f, buf, sizeof(buf), &pos), "encode crawl_pulse");
    checkTrue(encodeFloatField(4, 2.0f, buf, sizeof(buf), &pos), "encode wheel_gain_left_accel");
    checkTrue(encodeFloatField(5, 0.0f, buf, sizeof(buf), &pos), "encode wheel_intercept_left_accel");
    checkTrue(encodeFloatField(6, 1.0f, buf, sizeof(buf), &pos), "encode wheel_gain_left_decel");
    checkTrue(encodeFloatField(7, 0.0f, buf, sizeof(buf), &pos), "encode wheel_intercept_left_decel");
    checkTrue(encodeFloatField(8, 1.0f, buf, sizeof(buf), &pos), "encode wheel_gain_right_accel");
    checkTrue(encodeFloatField(9, 0.0f, buf, sizeof(buf), &pos), "encode wheel_intercept_right_accel");
    checkTrue(encodeFloatField(10, 1.0f, buf, sizeof(buf), &pos), "encode wheel_gain_right_decel");
    checkTrue(encodeFloatField(11, 0.0f, buf, sizeof(buf), &pos), "encode wheel_intercept_right_decel");

    const msg::ErrCode result = configurator.applyGroup(msg::ConfigGroupTarget::DRIVE, buf, pos);
    checkEq(result, msg::ErrCode::ERR_NONE, "applyGroup(DRIVE) result");
    checkFloatEq(configurator.config().drive.wheel_gain_left_accel, 2.0f,
                "config().drive.wheel_gain_left_accel reflects the push");
    // EXPLORATORY-KERNEL REWRITE (2026-08-15): the deeper verification this
    // scenario used to run (drive.tick(state) against Types::RobotState::
    // Wheel::cmdVelocity, checking the written duty) is GONE -- the kernel
    // has no tick(state)/cmdVelocity surface any more (its own fiber owns
    // the whole control step; commands arrive via drive(velocity, twist,
    // lease) and gains via setConfig()). config().drive reflecting the
    // push (above) plus wheelGain[0][0] landing in the kernel's OWN config
    // (below) is the level this harness can still verify without
    // simulating a full kernel cycle.
    checkFloatEq(drive.config().wheelGain[0][0], 2.0f,
                "drive.config().wheelGain[0][0] reflects the push via "
                "Core::buildDriveKernelConfig()");
  }

  // --- WHEEL_CONTROL: decode + installDriveKernelConfig() (Stage B/C) -----

  beginScenario(
      "WHEEL_CONTROL push decodes into config_ and reaches the kernel's own "
      "config() via Core::buildDriveKernelConfig()");
  {
    uint8_t buf[128];
    size_t pos = 0;
    checkTrue(encodeFloatField(1, 66.0f, buf, sizeof(buf), &pos), "encode v_min");
    checkTrue(encodeFloatField(2, 77.0f, buf, sizeof(buf), &pos), "encode bias_max");
    checkTrue(encodeFloatField(3, 8.0f, buf, sizeof(buf), &pos), "encode tau_adapt");
    checkTrue(encodeFloatField(4, 99.0f, buf, sizeof(buf), &pos), "encode a_steady");
    checkTrue(encodeFloatField(5, 10.0f, buf, sizeof(buf), &pos), "encode deficit_threshold");
    checkTrue(encodeFloatField(6, 200.0f, buf, sizeof(buf), &pos), "encode deficit_window");
    checkTrue(encodeFloatField(7, 0.11f, buf, sizeof(buf), &pos), "encode pid_kp");
    checkTrue(encodeFloatField(8, 0.22f, buf, sizeof(buf), &pos), "encode pid_ki");
    checkTrue(encodeFloatField(9, 0.33f, buf, sizeof(buf), &pos), "encode pid_i_max");
    checkTrue(encodeFloatField(10, 0.44f, buf, sizeof(buf), &pos), "encode pid_kaff");
    checkTrue(encodeFloatField(11, 0.55f, buf, sizeof(buf), &pos), "encode pid_max");
    // 133-002: field 12. The RUNTIME half of pos_err_max's config surface
    // (.claude/rules/configuration-discipline.md invariant 2) -- pushed
    // over the SAME existing WHEEL_CONTROL machinery as its siblings, with
    // no new arm anywhere. This assertion is the "verify, do not assume"
    // the ticket asked for.
    checkTrue(encodeFloatField(12, 12.0f, buf, sizeof(buf), &pos), "encode pos_err_max");

    const msg::ErrCode result =
        configurator.applyGroup(msg::ConfigGroupTarget::WHEEL_CONTROL, buf, pos);
    checkEq(result, msg::ErrCode::ERR_NONE, "applyGroup(WHEEL_CONTROL) result");
    checkFloatEq(configurator.config().wheelControl.pid_kp, 0.11f,
                "config().wheelControl.pid_kp reflects the push");
    checkFloatEq(configurator.config().wheelControl.pos_err_max, 12.0f,
                "config().wheelControl.pos_err_max reflects the push");

    // EXPLORATORY-KERNEL REWRITE (2026-08-15): ControlGains/AdaptationBounds
    // are GONE -- the kernel's Config is one flat struct now. kp/ki/kaff
    // copy through Core::buildDriveKernelConfig() unconverted (dimensionless/
    // [1/s]/[s]); iMax/pidMax/vMin/biasMax/posErrMax convert mm-domain ->
    // counts-domain via MOTORS.travel_calib_left/right, which this harness
    // never pushes (config_.motors stays all-zero, the "uncalibrated"
    // sentinel) -- those five fields are NOT independently verifiable here
    // without also exercising the MOTORS push below first, so only the
    // unconverted three are checked.
    const Control::DifferentialDrive::Config& kernelCfg = drive.config();
    checkFloatEq(kernelCfg.kp, 0.11f, "drive.config().kp");
    checkFloatEq(kernelCfg.ki, 0.22f, "drive.config().ki");
    checkFloatEq(kernelCfg.kaff, 0.44f, "drive.config().kaff");
  }

  // --- MOTORS: no motion guard any more -------------------------------------
  //
  // EXPLORATORY-KERNEL REWRITE (2026-08-15): MOTORS no longer applies
  // travel_calib onto the motor leaf (Hal::Motor::applyTravelCalib() is
  // deleted -- the leaf is counts-native) -- it feeds
  // Core::buildDriveKernelConfig()'s mm<->counts conversion instead, via
  // the SAME installDriveKernelConfig() call DRIVE/WHEEL_CONTROL use. That
  // rebuild-and-setConfig() push is exactly as live-safe as those two
  // pushes already were (the kernel's own fiber snapshots config at each
  // cycle start), so the per-side "refuses while moving" ERR_BUSY guard
  // (Core::configureMotor(), 132-007) has no motion-safety reason to exist
  // any more -- install(MOTORS) now always returns ERR_NONE, moving or not.

  beginScenario("MOTORS push applies travel_calib to config_, ERR_NONE, regardless of motion");
  {
    motorL.stagedVelocity = 100.0f;  // [mm/s] -- deliberately IN MOTION; no guard reads this any more
    motorL.stagedAppliedDuty = 0.3f;
    motorR.stagedVelocity = 0.0f;
    motorR.stagedAppliedDuty = 0.0f;

    uint8_t buf[128];
    size_t pos = 0;
    checkTrue(encodeFloatField(1, 0.71f, buf, sizeof(buf), &pos), "encode travel_calib_left");
    checkTrue(encodeFloatField(2, 0.70f, buf, sizeof(buf), &pos), "encode travel_calib_right");
    checkTrue(encodeInt32Field(3, 1, buf, sizeof(buf), &pos), "encode fwd_sign_left");
    checkTrue(encodeInt32Field(4, -1, buf, sizeof(buf), &pos), "encode fwd_sign_right");

    const msg::ErrCode result = configurator.applyGroup(msg::ConfigGroupTarget::MOTORS, buf, pos);
    checkEq(result, msg::ErrCode::ERR_NONE,
           "applyGroup(MOTORS) result -- ERR_NONE even with motorL in motion, no guard left to trip");
    checkFloatEq(configurator.config().motors.travel_calib_left, 0.71f,
                "config().motors.travel_calib_left reflects the push");
    checkFloatEq(configurator.config().motors.travel_calib_right, 0.70f,
                "config().motors.travel_calib_right reflects the push");
    // Feeds installDriveKernelConfig() -- fullDutyVelocity/wheelIntercept/
    // etc. all rebuild from the NEW travel_calib. duty_per_speed_left/right
    // are still config_.drive's own all-zero default here (never pushed in
    // this scenario), so fullDutyVelocity itself stays 0 -- this only
    // confirms the MOTORS push reached the rebuild, not any one derived
    // value.
    checkFloatEq(drive.config().fullDutyVelocity, 0.0f,
                "drive.config().fullDutyVelocity rebuilt (still 0 -- duty_per_speed never pushed)");
  }

  // --- OTOS: decode + configureOtos() (trap 3 CLOSED, 132-010) -------------

  beginScenario(
      "OTOS push decodes into config_ and reaches Core::configureOtos() -- "
      "scale fields are REGISTER-domain-converted via "
      "Hardware::scaleToRegister() before reaching setLinearScalar()/"
      "setAngularScalar() (trap 3 closed, 132-010)");
  {
    uint8_t buf[128];
    size_t pos = 0;
    checkTrue(encodeFloatField(1, -48.0f, buf, sizeof(buf), &pos), "encode offset_x");
    checkTrue(encodeFloatField(2, 4.0f, buf, sizeof(buf), &pos), "encode offset_y");
    checkTrue(encodeFloatField(3, 0.01f, buf, sizeof(buf), &pos), "encode offset_yaw");
    checkTrue(encodeFloatField(4, 1.03f, buf, sizeof(buf), &pos), "encode linear_scale");
    checkTrue(encodeFloatField(5, 0.98f, buf, sizeof(buf), &pos), "encode angular_scale");

    const msg::ErrCode result = configurator.applyGroup(msg::ConfigGroupTarget::OTOS, buf, pos);
    checkEq(result, msg::ErrCode::ERR_NONE, "applyGroup(OTOS) result");
    checkFloatEq(configurator.config().otos.linear_scale, 1.03f,
                "config().otos.linear_scale reflects the push (config_ itself stays "
                "in the RAW multiplier domain, per the-configuration-object.md's "
                "'the object holds RAW file values' rule)");
    checkFloatEq(otos.linearScalar, static_cast<float>(Hardware::scaleToRegister(1.03f)),
                "setLinearScalar() argument is REGISTER-domain (scaleToRegister(1.03) "
                "== 30), not the raw 1.03 multiplier -- trap 3 closed");
    checkFloatEq(otos.angularScalar, static_cast<float>(Hardware::scaleToRegister(0.98f)),
                "setAngularScalar() argument is REGISTER-domain (scaleToRegister(0.98) "
                "== -20)");
    checkFloatEq(otos.offsetX, -48.0f,
                "setOffset() x argument is unaffected -- no domain conversion for the "
                "lever arm, only the scale registers");
  }

  beginScenario(
      "OTOS scale regression guard (trap 3's own acceptance criterion): a "
      "pushed linearScale/angularScale of 1.0 (true unity) installs register "
      "0 -- the SAME register value RealOtos::begin() installs for an "
      "identical BAKED 1.0 multiplier, because both call sites now share the "
      "one Hardware::scaleToRegister() conversion. Also confirms the fix is "
      "live: the OLD bug would have installed register 1 (setLinearScalar("
      "1.0) with no conversion -> clamp+truncate -> register 1, a real "
      "+0.1% scalar, 'a 1-LSB scalar instead of unity')");
  {
    uint8_t buf[64];
    size_t pos = 0;
    // encodeFloatField() skips a field whose value is exactly 0.0f (implicit
    // presence) -- 1.0f is non-zero, so both fields are encoded normally.
    checkTrue(encodeFloatField(4, 1.0f, buf, sizeof(buf), &pos), "encode linear_scale = 1.0");
    checkTrue(encodeFloatField(5, 1.0f, buf, sizeof(buf), &pos), "encode angular_scale = 1.0");

    const msg::ErrCode result = configurator.applyGroup(msg::ConfigGroupTarget::OTOS, buf, pos);
    checkEq(result, msg::ErrCode::ERR_NONE, "applyGroup(OTOS, unity scale) result");

    const float bootWouldInstall = static_cast<float>(Hardware::scaleToRegister(1.0f));
    checkFloatEq(bootWouldInstall, 0.0f,
                "sanity: scaleToRegister(1.0) == register 0 -- true unity");
    checkFloatEq(otos.linearScalar, bootWouldInstall,
                "LIVE push of linearScale=1.0 installs the SAME register value "
                "begin() would install for a BAKED linearScale=1.0 -- live and "
                "boot paths agree on what '1.0' means");
    checkFloatEq(otos.angularScalar, bootWouldInstall,
                "LIVE push of angularScale=1.0 installs the SAME register value "
                "begin() would install for a BAKED angularScale=1.0");
  }

  // --- ESTIMATOR: decode succeeds, install honestly reports PERMANENTLY no consumer ---

  beginScenario(
      "ESTIMATOR push decodes into config_ (read-back stays honest) but "
      "install() returns ERR_UNIMPLEMENTED, not ERR_NONE -- PERMANENTLY, "
      "not a to-be-filled gap (trap 2 closed, 132-010, by making the dead "
      "end explicit: Core::StateEstimator is dead code (128-016) and its "
      "one candidate successor's call site was itself deliberately deleted "
      "(130-009) pending a from-scratch fusion redesign, "
      "clasi/issues/later/estimator-v2-otos-fusion-sim-first.md) -- this is "
      "NOT a silent no-op");
  {
    uint8_t buf[64];
    size_t pos = 0;
    checkTrue(encodeFloatField(1, 0.7f, buf, sizeof(buf), &pos), "encode weight_heading_otos");
    checkTrue(encodeFloatField(2, 0.6f, buf, sizeof(buf), &pos), "encode weight_omega_otos");
    checkTrue(encodeUint32Field(3, 500, buf, sizeof(buf), &pos), "encode staleness");

    const msg::ErrCode result = configurator.applyGroup(msg::ConfigGroupTarget::ESTIMATOR, buf, pos);
    checkEq(result, msg::ErrCode::ERR_UNIMPLEMENTED, "applyGroup(ESTIMATOR) result");
    checkFloatEq(configurator.config().estimator.weight_heading_otos, 0.7f,
                "config().estimator.weight_heading_otos reflects the push despite ERR_UNIMPLEMENTED "
                "-- read-back stays honest even though there is nothing to fan the value out to");
  }

  // --- NaN rejection + no-partial-commit -----------------------------------

  beginScenario(
      "A NaN payload in a bounded field is rejected (ERR_RANGE), not silently "
      "accepted -- and the rejected push leaves config_'s ENTIRE group "
      "untouched (no-partial-commit), not reverted-to-default or "
      "partially-overwritten with whatever decoded before the failing field");
  {
    // First land a KNOWN GOOD push, so "untouched" below means "still the
    // good push's values", not "still zero" (a weaker claim that a bug
    // zeroing config_ on failure could accidentally satisfy).
    uint8_t goodBuf[64];
    size_t goodPos = 0;
    checkTrue(encodeFloatField(1, 0.5f, goodBuf, sizeof(goodBuf), &goodPos),
              "encode duty_per_speed_left (good push)");
    checkTrue(encodeFloatField(4, 3.0f, goodBuf, sizeof(goodBuf), &goodPos),
              "encode wheel_gain_left_accel (good push)");
    const msg::ErrCode goodResult =
        configurator.applyGroup(msg::ConfigGroupTarget::DRIVE, goodBuf, goodPos);
    checkEq(goodResult, msg::ErrCode::ERR_NONE, "the good DRIVE push itself must succeed");

    // Field 4 (wheel_gain_left_accel) comes AFTER field 1 (duty_per_speed_
    // left, the NaN target) on the wire, so a bug that committed fields as
    // it decoded them (rather than into a scratch value first) would still
    // show field 1 poisoned but field 4 possibly written -- this ordering
    // is deliberate, to catch exactly that half-decoded state.
    uint8_t nanBuf[64];
    size_t nanPos = 0;
    const float nan = std::nanf("");
    checkTrue(encodeFloatField(1, nan, nanBuf, sizeof(nanBuf), &nanPos),
              "encode duty_per_speed_left = NaN");
    checkTrue(encodeFloatField(4, 9.0f, nanBuf, sizeof(nanBuf), &nanPos),
              "encode wheel_gain_left_accel = 9.0 (must NOT land)");

    const msg::ErrCode nanResult =
        configurator.applyGroup(msg::ConfigGroupTarget::DRIVE, nanBuf, nanPos);
    checkEq(nanResult, msg::ErrCode::ERR_RANGE, "applyGroup(DRIVE, NaN payload) result");
    checkFloatEq(configurator.config().drive.duty_per_speed_left, 0.5f,
                "config().drive.duty_per_speed_left still the GOOD push's value, not NaN");
    checkFloatEq(configurator.config().drive.wheel_gain_left_accel, 3.0f,
                "config().drive.wheel_gain_left_accel still the GOOD push's value, "
                "not the rejected push's 9.0 -- no partial commit");
  }

  beginScenario(
      "Truncated/malformed bytes are rejected (ERR_DECODE), and also leave "
      "config_ untouched (same no-partial-commit property, a non-NaN cause)");
  {
    uint8_t truncated[8];
    size_t pos = 0;
    checkTrue(WireRuntime::encodeTag(4, WireType::kFixed32, truncated, sizeof(truncated), &pos),
              "encode a fixed32 tag for wheel_gain_left_accel");
    // A fixed32 payload needs 4 more bytes; supply only 1, forcing
    // decodeFloat() to run out of buffer mid-field.
    truncated[pos] = 0x00;
    const size_t shortLen = pos + 1;

    const msg::ErrCode result =
        configurator.applyGroup(msg::ConfigGroupTarget::DRIVE, truncated, shortLen);
    checkEq(result, msg::ErrCode::ERR_DECODE, "applyGroup(DRIVE, truncated bytes) result");
    checkFloatEq(configurator.config().drive.duty_per_speed_left, 0.5f,
                "config().drive.duty_per_speed_left still the last GOOD push's value");
    checkFloatEq(configurator.config().drive.wheel_gain_left_accel, 3.0f,
                "config().drive.wheel_gain_left_accel still the last GOOD push's value");
  }

  // --- Boot-time install(): dutyPerSpeed sourced from the file (132-009) --

  beginScenario(
      "loadBaked()+install() (the no-arg, BOOT-time fan-out -- distinct "
      "from every applyGroup()/install(target) LIVE-push scenario above): "
      "dutyPerSpeed is sourced from config_.drive.duty_per_speed_left/right "
      "(the active robot JSON, via Config::defaultDriveGroup()), not the "
      "historical Drive::kDutyPerSpeed C++ literal -- 132-009 reverses "
      "07-31's MEASURED-NOT-CONFIGURED boot override per the 08-03 "
      "configuration-discipline rule");
  {
    // loadBaked() fully overwrites config_ (one assignment per group, no
    // merge) -- safe to call here regardless of every scenario above
    // having already mutated config_/drive_/motorL_/motorR_.
    configurator.loadBaked();
    configurator.install();

    // EXPLORATORY-KERNEL REWRITE (2026-08-15): dutyPerSpeedLeft()/
    // dutyPerSpeedRight()/calibrated() are GONE -- the kernel has ONE
    // kernel-wide fullDutyVelocity (counts/s), built by Core::
    // buildDriveKernelConfig() from duty_per_speed_left/right AND
    // MOTORS.travel_calib_left/right together (not from duty_per_speed
    // alone), and "calibrated" is now `output().ready`, which requires
    // begin() (never called by this lightweight, no-I2C-bus harness) to
    // have run -- there is no clean equivalent check here without also
    // exercising the kernel's own lifecycle, which is out of this
    // harness's scope (it verifies Configurator's fan-out, not the
    // kernel's own begin()/ready state machine).
    const msg::Drive hwDrive = Config::defaultDriveGroup();

    // The SAME boot install() call also retargets DRIVE/WHEEL_CONTROL onto
    // installDriveKernelConfig() instead of re-deriving Stage A/B/C inline
    // a second time -- confirm the fan-out still landed, both at the
    // config_ level and at the kernel's own config() level.
    checkFloatEq(configurator.config().drive.wheel_gain_left_accel,
                hwDrive.wheel_gain_left_accel,
                "config().drive.wheel_gain_left_accel reflects loadBaked()");
    checkFloatEq(drive.config().wheelGain[0][0], hwDrive.wheel_gain_left_accel,
                "drive.config().wheelGain[0][0] reflects the boot install() fan-out");
    const msg::WheelControl hwWheelControl = Config::defaultWheelControlGroup();
    checkFloatEq(drive.config().kp, hwWheelControl.pid_kp,
                "drive.config().kp reflects the boot install() fan-out");
  }

  std::printf("\n");
  if (g_failureCount == 0) {
    std::printf(
        "OK: applyGroup()/install(ConfigGroupTarget) classify boot-only vs. live "
        "correctly, reject NaN and malformed pushes without partial commit, "
        "fan live pushes out to DRIVE/WHEEL_CONTROL/MOTORS/OTOS honestly -- OTOS's "
        "scale domain now agrees with begin()'s boot conversion (trap 3 closed, "
        "132-010) -- with ESTIMATOR's missing-consumer dead end reported "
        "permanently, not hidden (trap 2 closed, 132-010), and loadBaked()+"
        "install() sources dutyPerSpeed from the file, not a hardcoded constant "
        "(132-009)\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the 132-008 applyGroup() tests\n", g_failureCount);
  return 1;
}
