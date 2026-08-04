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
//   - MOTORS: applies via App::configureMotor() at rest; refuses with
//     ERR_BUSY (surfaced, not swallowed) while in motion -- and the guard
//     is PER SIDE, not global: the side that is at rest still gets its
//     travel_calib applied even when the push overall returns ERR_BUSY
//     because the OTHER side is moving.
//   - OTOS: decodes and installs via App::configureOtos() -- known gap
//     (trap 3, the-configuration-object.md): the scale fields pass through
//     with no scaleToRegister() conversion yet (ticket 010's job).
//   - ESTIMATOR: decodes into config_ (read-back stays honest) but
//     install() returns ERR_UNIMPLEMENTED, not ERR_NONE -- there is no
//     live consumer (App::StateEstimator was deleted as dead code, sprint
//     128 ticket 016; ticket 010 adds a replacement reference). This is
//     the ticket's own headline property demonstrated directly: config
//     that acks OK and silently does nothing is worse than config that is
//     rejected -- ESTIMATOR does neither.
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

#include "app/boot_calibration.h"
#include "app/configurator.h"
#include "app/drive.h"
#include "config/robot.h"
#include "devices/motor.h"
#include "devices/otos.h"
#include "firm/types/robot_state.h"
#include "messages/envelope.h"
#include "messages/robot_config.h"
#include "messages/wire_runtime.h"
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

// --- Devices::Motor / Devices::Otos test doubles ----------------------------
// Same shape as configure_entry_points_harness.cpp's own RecordingMotor/
// RecordingOtos (132-007) -- duplicated rather than shared, matching this
// project's "each harness compiles ad hoc, no shared fixture" convention
// (src/tests/CLAUDE.md).

class RecordingMotor : public Devices::Motor {
 public:
  void begin() override {}
  void requestSample() override {}
  void setDuty(float duty) override { lastDuty = duty; }
  void setNeutral(Devices::Neutral) override {}
  void applyTravelCalib(float travelCalib) override { lastTravelCalib = travelCalib; }
  [[nodiscard]] bool reconfigure(const Devices::MotorConfig&) override { return true; }
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
  float lastTravelCalib = -1.0f;  // sentinel: configureMotor() must overwrite this to pass
};

class RecordingOtos : public Devices::Otos {
 public:
  void begin() override {}
  void tick(uint64_t) override {}
  Devices::PoseReading pose() const override { return {}; }
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
  App::Drive drive(motorL, motorR, /*trackWidth=*/128.0f);
  Motion::PlannerLimits limits;
  Motion::Planner planner(limits);
  App::Configurator configurator(drive, motorL, motorR, otos, planner, /*tuningStore=*/nullptr);

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

    // Isolate Stage A: zero wheelControl (Stage B/C inert) and set
    // dutyPerSpeed=1 so tick()'s written duty is exactly
    // correctedCommand() -- Drive::configure() does not touch dutyPerSpeed
    // itself (Drive::kDutyPerSpeed is a MEASURED, boot-baked constant, not
    // a live field -- drive.h's own doc comment; ticket 009's own call on
    // whether that reverses).
    drive.setDutyPerSpeed(1.0f, 1.0f);

    Types::RobotState state;
    state.wheelLeft.cmdVelocity = 200.0f;  // [mm/s]
    state.wheelRight.cmdVelocity = 0.0f;
    state.time.cyclePeriod = 50000;  // [us]
    drive.tick(state);
    checkFloatEq(motorL.lastDuty, 100.0f,
                "tick() duty reflects the pushed wheel_gain_left_accel=2.0 (200/2.0)");
  }

  // --- WHEEL_CONTROL: decode + Drive::configure() (Stage B/C) -------------

  beginScenario(
      "WHEEL_CONTROL push decodes into config_ and reaches Drive::configure() -- "
      "controlGains()/adaptationBounds() reflect the push");
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

    const msg::ErrCode result =
        configurator.applyGroup(msg::ConfigGroupTarget::WHEEL_CONTROL, buf, pos);
    checkEq(result, msg::ErrCode::ERR_NONE, "applyGroup(WHEEL_CONTROL) result");
    checkFloatEq(configurator.config().wheelControl.pid_kp, 0.11f,
                "config().wheelControl.pid_kp reflects the push");

    const App::Drive::ControlGains& gains = drive.controlGains();
    checkFloatEq(gains.kp, 0.11f, "controlGains().kp");
    checkFloatEq(gains.ki, 0.22f, "controlGains().ki");
    checkFloatEq(gains.iMax, 0.33f, "controlGains().iMax");
    checkFloatEq(gains.kaff, 0.44f, "controlGains().kaff");
    checkFloatEq(gains.pidMax, 0.55f, "controlGains().pidMax");

    const App::Drive::AdaptationBounds& bounds = drive.adaptationBounds();
    checkFloatEq(bounds.vMin, 66.0f, "adaptationBounds().vMin");
    checkFloatEq(bounds.biasMax, 77.0f, "adaptationBounds().biasMax");
  }

  // --- MOTORS: guarded, per side -------------------------------------------

  beginScenario("MOTORS push at rest applies travel_calib to both sides, ERR_NONE");
  {
    motorL.stagedVelocity = 0.0f;
    motorL.stagedAppliedDuty = 0.0f;
    motorR.stagedVelocity = 0.0f;
    motorR.stagedAppliedDuty = 0.0f;

    uint8_t buf[128];
    size_t pos = 0;
    checkTrue(encodeFloatField(1, 0.71f, buf, sizeof(buf), &pos), "encode travel_calib_left");
    checkTrue(encodeFloatField(2, 0.70f, buf, sizeof(buf), &pos), "encode travel_calib_right");
    checkTrue(encodeInt32Field(3, 1, buf, sizeof(buf), &pos), "encode fwd_sign_left");
    checkTrue(encodeInt32Field(4, -1, buf, sizeof(buf), &pos), "encode fwd_sign_right");

    const msg::ErrCode result = configurator.applyGroup(msg::ConfigGroupTarget::MOTORS, buf, pos);
    checkEq(result, msg::ErrCode::ERR_NONE, "applyGroup(MOTORS) result, both sides at rest");
    checkFloatEq(configurator.config().motors.travel_calib_left, 0.71f,
                "config().motors.travel_calib_left reflects the push");
    checkFloatEq(motorL.lastTravelCalib, 0.71f, "motorL applyTravelCalib() argument");
    checkFloatEq(motorR.lastTravelCalib, 0.70f, "motorR applyTravelCalib() argument");
  }

  beginScenario(
      "MOTORS push while one side is in motion returns ERR_BUSY (surfaced, not "
      "swallowed) -- the OTHER, at-rest side still applies (per-side guard, "
      "App::configureMotor(), 132-007)");
  {
    motorL.stagedVelocity = 100.0f;  // [mm/s], well above the rest threshold
    motorL.stagedAppliedDuty = 0.3f;
    motorL.lastTravelCalib = -1.0f;  // reset sentinel
    motorR.stagedVelocity = 0.0f;
    motorR.stagedAppliedDuty = 0.0f;
    motorR.lastTravelCalib = -1.0f;  // reset sentinel

    uint8_t buf[128];
    size_t pos = 0;
    checkTrue(encodeFloatField(1, 0.81f, buf, sizeof(buf), &pos), "encode travel_calib_left");
    checkTrue(encodeFloatField(2, 0.80f, buf, sizeof(buf), &pos), "encode travel_calib_right");

    const msg::ErrCode result = configurator.applyGroup(msg::ConfigGroupTarget::MOTORS, buf, pos);
    checkEq(result, msg::ErrCode::ERR_BUSY, "applyGroup(MOTORS) result, left side in motion");
    checkFloatEq(motorL.lastTravelCalib, -1.0f,
                "motorL applyTravelCalib() must NOT be called while moving");
    checkFloatEq(motorR.lastTravelCalib, 0.80f,
                "motorR (at rest) still gets its travel_calib applied");
    // config_ itself still reflects the decode -- only install()'s FAN-OUT
    // is refused for the busy side, not the decode-into-config_ step.
    checkFloatEq(configurator.config().motors.travel_calib_left, 0.81f,
                "config().motors.travel_calib_left still reflects the decode");
  }

  // --- OTOS: decode + configureOtos() (known trap-3 gap) -------------------

  beginScenario(
      "OTOS push decodes into config_ and reaches App::configureOtos() -- "
      "known gap: scale fields pass through with no scaleToRegister() "
      "conversion yet (trap 3, ticket 010's job)");
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
                "config().otos.linear_scale reflects the push");
    checkFloatEq(otos.linearScalar, 1.03f,
                "setLinearScalar() argument -- pass-through, not yet register-converted");
    checkFloatEq(otos.offsetX, -48.0f, "setOffset() x argument");
  }

  // --- ESTIMATOR: decode succeeds, install honestly reports no consumer ---

  beginScenario(
      "ESTIMATOR push decodes into config_ (read-back stays honest) but "
      "install() returns ERR_UNIMPLEMENTED, not ERR_NONE -- no live consumer "
      "exists yet (trap 2, ticket 010's job) -- this is NOT a silent no-op");
  {
    uint8_t buf[64];
    size_t pos = 0;
    checkTrue(encodeFloatField(1, 0.7f, buf, sizeof(buf), &pos), "encode weight_heading_otos");
    checkTrue(encodeFloatField(2, 0.6f, buf, sizeof(buf), &pos), "encode weight_omega_otos");
    checkTrue(encodeUint32Field(3, 500, buf, sizeof(buf), &pos), "encode staleness");

    const msg::ErrCode result = configurator.applyGroup(msg::ConfigGroupTarget::ESTIMATOR, buf, pos);
    checkEq(result, msg::ErrCode::ERR_UNIMPLEMENTED, "applyGroup(ESTIMATOR) result");
    checkFloatEq(configurator.config().estimator.weight_heading_otos, 0.7f,
                "config().estimator.weight_heading_otos reflects the push despite ERR_UNIMPLEMENTED");
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

  std::printf("\n");
  if (g_failureCount == 0) {
    std::printf(
        "OK: applyGroup()/install(ConfigGroupTarget) classify boot-only vs. live "
        "correctly, reject NaN and malformed pushes without partial commit, and "
        "fan live pushes out to DRIVE/WHEEL_CONTROL/MOTORS/OTOS honestly (with "
        "ESTIMATOR's known missing-consumer gap reported, not hidden)\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the 132-008 applyGroup() tests\n", g_failureCount);
  return 1;
}
