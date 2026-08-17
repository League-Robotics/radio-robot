// configurator_getconfig_harness.cpp -- ticket 132-011's own acceptance
// test (GetConfig/ConfigSnapshot wire read-back + host get_config(),
// sprint 132 "configuration discipline"): Configurator::encodeSnapshot() --
// the read-back half of the-configuration-object.md's "GetConfig reads the
// object straight back out" story, exercised at the SAME Configurator-API
// level 132-008's own configurator_applygroup_harness.cpp already
// established for the write direction (applyGroup()/install()).
//
// Covers:
//   - DRIVE (a live-classified, wire-writable target): push a known set of
//     values via applyGroup(), then read them straight back via
//     encodeSnapshot() + msg::wire::decode() on the resulting bytes --
//     "push then get shows the pushed value, not a stale baked one" (this
//     ticket's own acceptance criterion, verbatim).
//   - ESTIMATOR (live-classified but install()'s own PERMANENT
//     ERR_UNIMPLEMENTED, 132-010): confirms read-back stays honest even
//     for a target install() can never actually apply -- config_.estimator
//     still decodes correctly (configurator.h's own doc comment on this
//     exact point).
//   - GEOMETRY (a BOOT-ONLY target): confirms applyGroup(GEOMETRY, ...)
//     itself still returns ERR_NOT_LIVE (the write-side rejection,
//     132-008, re-proven here for this scenario's own before/after
//     contrast) while encodeSnapshot(GEOMETRY, ...) succeeds regardless --
//     "GET still works for boot-only groups even though SET is rejected:
//     read-back is not gated by re-appliability, only writes are" (this
//     ticket's own testing note, verbatim). Populated via loadBaked()
//     (the only way a boot-only group's config_ member ever gets a
//     non-default value) rather than applyGroup(), which the boot-only
//     rejection makes impossible for this target by construction.
//   - CONFIG_GROUP_UNSPECIFIED: encodeSnapshot() rejects with ERR_BADARG,
//     matching applyGroup()'s own "loud rejection, not silence" discipline
//     in the read direction.
//
// Hand-rolled assertions, PASS/FAIL per scenario, nonzero exit on any
// failure -- mirrors every other src/tests/sim/unit harness's own shape
// (configurator_applygroup_harness.cpp, 132-008).
//
// Wire-encoding note: the hand-rolled test-only encodeFloatField()/
// encodeInt32Field()/encodeUint32Field() helpers below are copied from
// configurator_applygroup_harness.cpp verbatim (src/tests/CLAUDE.md's own
// "each harness compiles ad hoc, no shared fixture" convention) -- they
// build the wire bytes this harness PUSHES via applyGroup(); the bytes
// this harness READS BACK come from encodeSnapshot()'s own production
// msg::wire::encode(<Group>&, ...) call (132-011's own generator addition,
// gen_messages.py), decoded back via the production msg::wire::decode()
// this project already trusts (128-008's own decode() family) -- so the
// "push" side is test-only encoding, but the "get" side exercises the
// REAL production encode/decode round trip this ticket's acceptance
// criteria are actually about.
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
#include "hal/clock.h"
#include "hal/motor.h"
#include "hardware/generic/real_otos.h"
#include "firm/types/robot_state.h"
#include "messages/envelope.h"
#include "messages/robot_config.h"
#include "messages/wire.h"
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

void checkFloatEq(float actual, float expected, const std::string& what, float tol = 1e-4f) {
  if (std::fabs(actual - expected) > tol) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %g, got %g", what.c_str(),
                  static_cast<double>(expected), static_cast<double>(actual));
    fail(buf);
  }
}

// --- Hand-rolled test-only wire encoding (PUSH side only -- see file
// header's own "Wire-encoding note") -------------------------------------

using WireRuntime::WireType;

bool encodeFloatField(uint32_t fieldNumber, float value, uint8_t* buf, size_t cap, size_t* pos) {
  if (value == 0.0f) return true;
  if (!WireRuntime::encodeTag(fieldNumber, WireType::kFixed32, buf, cap, pos)) return false;
  return WireRuntime::encodeFloat(value, buf, cap, pos);
}

// --- Hal::Motor / Hal::Otos test doubles (copied verbatim from
// configurator_applygroup_harness.cpp, 132-008 -- this harness never
// exercises MOTORS/OTOS behaviorally, but Configurator's constructor still
// needs real references of these types) -----------------------------------

class RecordingMotor : public Hal::Motor {
 public:
  void begin() override {}
  void requestSample() override {}
  void setDuty(float duty) override { lastDuty = duty; }
  void setNeutral(Hal::Neutral) override {}
  [[nodiscard]] bool reconfigure(const Hal::MotorConfig&) override { return true; }
  void tick(uint64_t) override {}

  float position() const override { return 0.0f; }
  float velocity() const override { return 0.0f; }
  float appliedDuty() const override { return 0.0f; }
  bool connected() const override { return true; }
  uint64_t sampleTime() const override { return 0; }

  void resetPosition() override {}
  void rebaseline() override {}

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

  float linearScalar = -1.0f;
  float angularScalar = -1.0f;
  float offsetX = -1.0f;
  float offsetY = -1.0f;
  float offsetHeading = -1.0f;
};

}  // namespace

int main() {
  std::printf("=== 132-011 Configurator::encodeSnapshot() (GetConfig/ConfigSnapshot read-back) ===\n\n");

  RecordingMotor motorL, motorR;
  RecordingOtos otos;
  StubClock clock;
  StubSleeper sleeper;
  Control::DifferentialDrive drive(motorL, motorR, clock, sleeper);
  Core::Configurator configurator(drive, motorL, motorR, otos, /*tuningStore=*/nullptr);

  // --- DRIVE: push then get -------------------------------------------

  beginScenario(
      "DRIVE: applyGroup() push, then encodeSnapshot()+decode() reads back "
      "the SAME pushed values -- not a stale baked one");
  {
    uint8_t pushBuf[128];
    size_t pushPos = 0;
    checkTrue(encodeFloatField(1, 0.42f, pushBuf, sizeof(pushBuf), &pushPos), "encode duty_per_speed_left");
    checkTrue(encodeFloatField(2, 0.41f, pushBuf, sizeof(pushBuf), &pushPos), "encode duty_per_speed_right");
    checkTrue(encodeFloatField(3, 0.05f, pushBuf, sizeof(pushBuf), &pushPos), "encode crawl_pulse");
    checkTrue(encodeFloatField(4, 2.1f, pushBuf, sizeof(pushBuf), &pushPos), "encode wheel_gain_left_accel");
    checkTrue(encodeFloatField(6, 1.9f, pushBuf, sizeof(pushBuf), &pushPos), "encode wheel_gain_left_decel");
    checkTrue(encodeFloatField(8, 2.2f, pushBuf, sizeof(pushBuf), &pushPos), "encode wheel_gain_right_accel");
    checkTrue(encodeFloatField(10, 1.8f, pushBuf, sizeof(pushBuf), &pushPos), "encode wheel_gain_right_decel");

    const msg::ErrCode pushResult =
        configurator.applyGroup(msg::ConfigGroupTarget::DRIVE, pushBuf, pushPos);
    checkEq(pushResult, msg::ErrCode::ERR_NONE, "applyGroup(DRIVE) push result");

    msg::ConfigSnapshot snapshot;
    const msg::ErrCode getResult = configurator.encodeSnapshot(msg::ConfigGroupTarget::DRIVE, snapshot);
    checkEq(getResult, msg::ErrCode::ERR_NONE, "encodeSnapshot(DRIVE) result");
    checkTrue(snapshot.target == msg::ConfigGroupTarget::DRIVE, "snapshot.target echoes the request");
    checkTrue(snapshot.body_count_val() > 0, "snapshot carries a nonzero-length body");

    msg::Drive decoded;
    const msg::wire::Result decodeResult =
        msg::wire::decode(decoded, snapshot.body(), snapshot.body_count_val());
    checkTrue(decodeResult.ok, "the snapshot's own body decodes cleanly");
    checkFloatEq(decoded.duty_per_speed_left, 0.42f, "read-back duty_per_speed_left == pushed value");
    checkFloatEq(decoded.duty_per_speed_right, 0.41f, "read-back duty_per_speed_right == pushed value");
    checkFloatEq(decoded.crawl_pulse, 0.05f, "read-back crawl_pulse == pushed value");
    checkFloatEq(decoded.wheel_gain_left_accel, 2.1f, "read-back wheel_gain_left_accel == pushed value");
    checkFloatEq(decoded.wheel_gain_left_decel, 1.9f, "read-back wheel_gain_left_decel == pushed value");
    checkFloatEq(decoded.wheel_gain_right_accel, 2.2f, "read-back wheel_gain_right_accel == pushed value");
    checkFloatEq(decoded.wheel_gain_right_decel, 1.8f, "read-back wheel_gain_right_decel == pushed value");

    // A SECOND push with DIFFERENT values must read back as the NEW
    // values, not the first push's -- proving this is live read-back, not
    // a cached snapshot from the first call.
    uint8_t pushBuf2[64];
    size_t pushPos2 = 0;
    checkTrue(encodeFloatField(1, 0.77f, pushBuf2, sizeof(pushBuf2), &pushPos2),
              "encode duty_per_speed_left (second push)");
    checkEq(configurator.applyGroup(msg::ConfigGroupTarget::DRIVE, pushBuf2, pushPos2),
            msg::ErrCode::ERR_NONE, "second applyGroup(DRIVE) push result");

    msg::ConfigSnapshot snapshot2;
    checkEq(configurator.encodeSnapshot(msg::ConfigGroupTarget::DRIVE, snapshot2),
            msg::ErrCode::ERR_NONE, "encodeSnapshot(DRIVE) after second push");
    msg::Drive decoded2;
    checkTrue(msg::wire::decode(decoded2, snapshot2.body(), snapshot2.body_count_val()).ok,
              "second snapshot's body decodes cleanly");
    checkFloatEq(decoded2.duty_per_speed_left, 0.77f,
                "read-back reflects the SECOND push, not the first (not a stale baked value)");
  }

  // --- ESTIMATOR: live-classified but install() is permanently
  //     ERR_UNIMPLEMENTED (132-010) -- read-back must stay honest anyway --

  beginScenario(
      "ESTIMATOR: applyGroup() decodes (install() separately reports "
      "ERR_UNIMPLEMENTED, 132-010 -- not re-tested here), and "
      "encodeSnapshot() reads the decoded value back correctly -- "
      "read-back does not depend on install() ever having a real consumer");
  {
    uint8_t pushBuf[64];
    size_t pushPos = 0;
    checkTrue(encodeFloatField(1, 0.65f, pushBuf, sizeof(pushBuf), &pushPos), "encode weight_heading_otos");
    // applyGroup()'s own return (ERR_UNIMPLEMENTED, from install()) is
    // 132-010's own acceptance criterion, not re-asserted here -- this
    // scenario cares only that config_.estimator was decoded regardless.
    configurator.applyGroup(msg::ConfigGroupTarget::ESTIMATOR, pushBuf, pushPos);

    msg::ConfigSnapshot snapshot;
    const msg::ErrCode getResult =
        configurator.encodeSnapshot(msg::ConfigGroupTarget::ESTIMATOR, snapshot);
    checkEq(getResult, msg::ErrCode::ERR_NONE, "encodeSnapshot(ESTIMATOR) result");

    msg::Estimator decoded;
    checkTrue(msg::wire::decode(decoded, snapshot.body(), snapshot.body_count_val()).ok,
              "the ESTIMATOR snapshot's own body decodes cleanly");
    checkFloatEq(decoded.weight_heading_otos, 0.65f,
                "read-back reflects the pushed value even though install() has no consumer");
  }

  // --- GEOMETRY: boot-only -- SET rejected, GET still works ---------------

  beginScenario(
      "GEOMETRY (boot-only): applyGroup() (the SET path) returns "
      "ERR_NOT_LIVE, but encodeSnapshot() (the GET path) succeeds "
      "regardless -- read-back is not gated by re-appliability, only "
      "writes are (this ticket's own testing note, verbatim)");
  {
    const msg::ErrCode pushResult =
        configurator.applyGroup(msg::ConfigGroupTarget::GEOMETRY, /*wire=*/nullptr, /*len=*/0);
    checkEq(pushResult, msg::ErrCode::ERR_NOT_LIVE, "applyGroup(GEOMETRY) is rejected (SET path)");

    // loadBaked() is the only way a boot-only group's config_ member ever
    // gets a non-default value -- applyGroup() cannot do it for this
    // target by construction (the rejection above). This also
    // demonstrates read-back is HONEST about the boot-baked value, not
    // just "doesn't crash": defaultGeometryGroup() is the SAME generated,
    // robot-JSON-baked source loadBaked() itself reads from (config/
    // boot_config.h, 132-005).
    configurator.loadBaked();
    const msg::Geometry hwGeometry = Config::defaultGeometryGroup();

    msg::ConfigSnapshot snapshot;
    const msg::ErrCode getResult =
        configurator.encodeSnapshot(msg::ConfigGroupTarget::GEOMETRY, snapshot);
    checkEq(getResult, msg::ErrCode::ERR_NONE, "encodeSnapshot(GEOMETRY) succeeds (GET path)");

    msg::Geometry decoded;
    checkTrue(msg::wire::decode(decoded, snapshot.body(), snapshot.body_count_val()).ok,
              "the GEOMETRY snapshot's own body decodes cleanly");
    checkFloatEq(decoded.trackwidth, hwGeometry.trackwidth,
                "read-back trackwidth matches the baked robot JSON value");
    checkFloatEq(decoded.rotation_gain_pos, hwGeometry.rotation_gain_pos,
                "read-back rotation_gain_pos matches the baked robot JSON value");
  }

  // --- CONFIG_GROUP_UNSPECIFIED: loud rejection, not silence -------------

  beginScenario(
      "CONFIG_GROUP_UNSPECIFIED: encodeSnapshot() returns ERR_BADARG -- "
      "the read direction's own version of applyGroup()'s 'reject "
      "loudly, never silently no-op' discipline");
  {
    msg::ConfigSnapshot snapshot;
    const msg::ErrCode result = configurator.encodeSnapshot(
        msg::ConfigGroupTarget::CONFIG_GROUP_UNSPECIFIED, snapshot);
    checkEq(result, msg::ErrCode::ERR_BADARG, "encodeSnapshot(CONFIG_GROUP_UNSPECIFIED) result");
  }

  std::printf("\n");
  if (g_failureCount == 0) {
    std::printf(
        "OK: encodeSnapshot() round-trips a live push (DRIVE), stays honest "
        "for a target with no install() consumer (ESTIMATOR, 132-010), "
        "reads back a boot-only group SET itself refuses (GEOMETRY), and "
        "rejects an unrecognized target loudly (ERR_BADARG)\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the 132-011 encodeSnapshot() tests\n", g_failureCount);
  return 1;
}
