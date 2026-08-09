// configurator_provenance_harness.cpp -- ticket 133-006's own acceptance
// test for PER-GROUP CONFIG PROVENANCE
// (A-live-config-push-is-wiped-by-the-next-reconnect.md, "per-group config
// provenance, reported on the reply").
//
// The property under test: for every config group, the robot can say WHERE
// the values it is currently running came from -- BAKED (the robot-JSON
// values compiled in at boot), LIVE (a wire push landed on that group during
// this power cycle), or PERSISTED (restored at boot out of the flash tuning
// snapshot). Before this ticket a live push could be silently erased by the
// next reconnect and nothing anywhere reported it.
//
// The three design constraints this harness exists to hold in place, each of
// which is a rework if it slips:
//
//   1. STAMPED AT THE MUTATION SITE, NOT AT CALL SITES. loadBaked(),
//      reapplyPersistedTuning(), applyGroup() and applyField() are the only
//      functions that write `config_`, and each stamps as it writes. The
//      scenarios below drive every one of those four paths through the public
//      API and assert the resulting provenance, so a future path that mutates
//      config_ without stamping shows up here as a stale source rather than
//      as a comment nobody read.
//
//   2. PER-GROUP, NEVER ONE GLOBAL FLAG. Scenario 3 is the specific case a
//      single flag gets wrong: DRIVE is pushed LIVE while its untouched
//      siblings must still read BAKED. A global flag would have to lie about
//      one of them.
//
//   3. PROVENANCE IS A PROPERTY OF THE ANSWER, NOT OF THE CONFIG. It rides
//      msg::ConfigSnapshot (the GetConfig reply), never Config::Robot. That
//      one is enforced structurally rather than here -- a `source` field
//      inside a group message would be emitted into the host pydantic model
//      and data/robots/robot_config.schema.json too, and gen_messages.py's
//      _CONFIG_ENVELOPE_MESSAGE_NAMES excludes ConfigSnapshot from both
//      emission modes -- with the host-side guard living in
//      src/tests/unit/test_config_provenance_not_in_json.py. Scenario 8 here
//      covers the wire half: encodeSnapshot() actually carries the source.
//
// Hand-rolled assertions, PASS/FAIL per scenario, nonzero exit on any
// failure -- the same shape every other src/tests/sim/unit harness uses.
// The test-only wire encoder and the Motor/Otos doubles below are copied
// from configurator_getconfig_harness.cpp (132-011) per src/tests/CLAUDE.md's
// "each harness compiles ad hoc, no shared fixture" convention.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include "core/boot_calibration.h"
#include "core/configurator.h"
#include "core/differential_drive.h"
#include "config/boot_config.h"
#include "config/persisted_tuning.h"
#include "config/robot.h"
#include "hal/motor.h"
#include "hardware/generic/real_otos.h"
#include "firm/types/robot_state.h"
#include "messages/envelope.h"
#include "messages/robot_config.h"
#include "messages/wire.h"
#include "messages/wire_runtime.h"
#include "motion/navigator/arc_solver.h"
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

const char* sourceName(msg::ConfigSource source) {
  switch (source) {
    case msg::ConfigSource::CONFIG_SOURCE_UNSPECIFIED: return "UNSPECIFIED";
    case msg::ConfigSource::CONFIG_SOURCE_BAKED: return "BAKED";
    case msg::ConfigSource::CONFIG_SOURCE_LIVE: return "LIVE";
    case msg::ConfigSource::CONFIG_SOURCE_PERSISTED: return "PERSISTED";
  }
  return "?";
}

void checkSource(msg::ConfigSource actual, msg::ConfigSource expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %s, got %s", what.c_str(),
                  sourceName(expected), sourceName(actual));
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

// Every real (non-UNSPECIFIED) ConfigGroupTarget, so a scenario can assert
// "and every OTHER group still reads X" without listing them by hand -- the
// per-group property is only meaningful if the untouched siblings are
// actually checked.
constexpr msg::ConfigGroupTarget kAllGroups[] = {
    msg::ConfigGroupTarget::GEOMETRY,       msg::ConfigGroupTarget::MOTORS,
    msg::ConfigGroupTarget::DRIVE,          msg::ConfigGroupTarget::WHEEL_CONTROL,
    msg::ConfigGroupTarget::PLANNER,        msg::ConfigGroupTarget::OTOS,
    msg::ConfigGroupTarget::ESTIMATOR,      msg::ConfigGroupTarget::PLANNER_SHAPER,
};

const char* groupName(msg::ConfigGroupTarget target) {
  switch (target) {
    case msg::ConfigGroupTarget::GEOMETRY: return "GEOMETRY";
    case msg::ConfigGroupTarget::MOTORS: return "MOTORS";
    case msg::ConfigGroupTarget::DRIVE: return "DRIVE";
    case msg::ConfigGroupTarget::WHEEL_CONTROL: return "WHEEL_CONTROL";
    case msg::ConfigGroupTarget::PLANNER: return "PLANNER";
    case msg::ConfigGroupTarget::OTOS: return "OTOS";
    case msg::ConfigGroupTarget::ESTIMATOR: return "ESTIMATOR";
    case msg::ConfigGroupTarget::PLANNER_SHAPER: return "PLANNER_SHAPER";
    case msg::ConfigGroupTarget::CONFIG_GROUP_UNSPECIFIED: return "UNSPECIFIED";
  }
  return "?";
}

// checkEveryGroupExcept() -- assert every group EXCEPT the named exceptions
// reports `expected`. `exceptions`/`exceptionCount` name the groups a
// scenario deliberately touched.
void checkEveryGroupExcept(const Core::Configurator& configurator, msg::ConfigSource expected,
                           const msg::ConfigGroupTarget* exceptions, size_t exceptionCount,
                           const std::string& what) {
  for (const msg::ConfigGroupTarget target : kAllGroups) {
    bool skip = false;
    for (size_t i = 0; i < exceptionCount; ++i) {
      if (exceptions[i] == target) skip = true;
    }
    if (skip) continue;
    checkSource(configurator.configSource(target), expected,
                what + " [" + groupName(target) + "]");
  }
}

// --- Hand-rolled test-only wire encoding (PUSH side only) ----------------

using WireRuntime::WireType;

bool encodeFloatField(uint32_t fieldNumber, float value, uint8_t* buf, size_t cap, size_t* pos) {
  if (value == 0.0f) return true;
  if (!WireRuntime::encodeTag(fieldNumber, WireType::kFixed32, buf, cap, pos)) return false;
  return WireRuntime::encodeFloat(value, buf, cap, pos);
}

// --- Hal::Motor / Hal::Otos test doubles -------------------------

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
  float velocity() const override { return 0.0f; }
  float appliedDuty() const override { return 0.0f; }
  bool connected() const override { return true; }
  uint64_t sampleTime() const override { return 0; }

  void resetPosition() override {}
  void rebaseline() override {}

  float lastDuty = 0.0f;
  float lastTravelCalib = -1.0f;
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
  std::printf("=== 133-006 Configurator per-group config provenance ===\n\n");

  RecordingMotor motorL, motorR;
  RecordingOtos otos;
  Core::DifferentialDrive drive(motorL, motorR, /*trackWidth=*/128.0f);
  Motion::PlannerLimits limits;
  Motion::Planner planner(limits);
  Motion::NavigatorLimits navigatorLimits;
  Core::Configurator configurator(drive, motorL, motorR, otos, planner, navigatorLimits, /*tuningStore=*/nullptr);

  // --- 1. Before loadBaked(): UNSPECIFIED, which is the honest answer ----

  beginScenario(
      "before loadBaked(): every group reports UNSPECIFIED -- config_ holds "
      "C++ zero-initialization at this point, which is NOT the baked file, "
      "and claiming BAKED here would be the first lie");
  {
    checkEveryGroupExcept(configurator, msg::ConfigSource::CONFIG_SOURCE_UNSPECIFIED,
                          nullptr, 0, "pre-loadBaked() source");
    checkSource(configurator.configSource(msg::ConfigGroupTarget::CONFIG_GROUP_UNSPECIFIED),
                msg::ConfigSource::CONFIG_SOURCE_UNSPECIFIED,
                "CONFIG_GROUP_UNSPECIFIED is never a real group");
  }

  // --- 2. loadBaked() stamps EVERY group BAKED --------------------------

  beginScenario("loadBaked() stamps every group BAKED");
  {
    configurator.loadBaked();
    checkEveryGroupExcept(configurator, msg::ConfigSource::CONFIG_SOURCE_BAKED,
                          nullptr, 0, "post-loadBaked() source");
  }

  // --- 3. THE case a single global flag gets wrong ----------------------

  beginScenario(
      "applyGroup(DRIVE) makes DRIVE read LIVE while every untouched sibling "
      "still reads BAKED -- the specific case a single global flag cannot "
      "represent");
  {
    uint8_t pushBuf[128];
    size_t pushPos = 0;
    checkTrue(encodeFloatField(1, 0.42f, pushBuf, sizeof(pushBuf), &pushPos),
              "encode duty_per_speed_left");
    checkTrue(encodeFloatField(4, 2.1f, pushBuf, sizeof(pushBuf), &pushPos),
              "encode wheel_gain_left_accel");
    checkTrue(configurator.applyGroup(msg::ConfigGroupTarget::DRIVE, pushBuf, pushPos) ==
                  msg::ErrCode::ERR_NONE,
              "applyGroup(DRIVE) succeeds");

    checkSource(configurator.configSource(msg::ConfigGroupTarget::DRIVE),
                msg::ConfigSource::CONFIG_SOURCE_LIVE, "DRIVE reads LIVE after its push");
    const msg::ConfigGroupTarget touched[] = {msg::ConfigGroupTarget::DRIVE};
    checkEveryGroupExcept(configurator, msg::ConfigSource::CONFIG_SOURCE_BAKED, touched, 1,
                          "an untouched sibling still reads BAKED");
  }

  // --- 4. applyField() -- the OTHER mutation path ------------------------

  beginScenario(
      "applyField(WHEEL_CONTROL, pid_kp) stamps LIVE too -- the second of "
      "the two live-push paths, stamped at the same mutation site");
  {
    // WheelControl.pid_kp is field 1 (robot_config.proto).
    checkTrue(configurator.applyField(msg::ConfigGroupTarget::WHEEL_CONTROL,
                                      /*fieldNumber=*/1, 0.37f) == msg::ErrCode::ERR_NONE,
              "applyField(WHEEL_CONTROL, pid_kp) succeeds");
    checkSource(configurator.configSource(msg::ConfigGroupTarget::WHEEL_CONTROL),
                msg::ConfigSource::CONFIG_SOURCE_LIVE, "WHEEL_CONTROL reads LIVE");
    checkSource(configurator.configSource(msg::ConfigGroupTarget::DRIVE),
                msg::ConfigSource::CONFIG_SOURCE_LIVE, "DRIVE's earlier LIVE is not disturbed");
    const msg::ConfigGroupTarget touched[] = {msg::ConfigGroupTarget::DRIVE,
                                              msg::ConfigGroupTarget::WHEEL_CONTROL};
    checkEveryGroupExcept(configurator, msg::ConfigSource::CONFIG_SOURCE_BAKED, touched, 2,
                          "still-untouched siblings still read BAKED");
  }

  // --- 5. A REJECTED push must not claim to be live ---------------------

  beginScenario(
      "every rejection path leaves provenance alone -- a boot-only target, "
      "an unknown field number, and a non-finite value all leave config_ "
      "untouched, so none of them may make a group claim it is LIVE");
  {
    // Boot-only target: refused before any decode.
    checkTrue(configurator.applyGroup(msg::ConfigGroupTarget::GEOMETRY, nullptr, 0) ==
                  msg::ErrCode::ERR_NOT_LIVE,
              "applyGroup(GEOMETRY) is refused (boot-only)");
    checkSource(configurator.configSource(msg::ConfigGroupTarget::GEOMETRY),
                msg::ConfigSource::CONFIG_SOURCE_BAKED,
                "a refused boot-only push leaves GEOMETRY reading BAKED");

    // Unknown field number on a live target: refused before the write.
    checkTrue(configurator.applyField(msg::ConfigGroupTarget::OTOS,
                                      /*fieldNumber=*/9999, 1.0f) != msg::ErrCode::ERR_NONE,
              "applyField(OTOS, bogus field) is refused");
    checkSource(configurator.configSource(msg::ConfigGroupTarget::OTOS),
                msg::ConfigSource::CONFIG_SOURCE_BAKED,
                "an unknown-field push leaves OTOS reading BAKED");

    // Non-finite value: refused before msg::wire::setField() ever runs.
    checkTrue(configurator.applyField(msg::ConfigGroupTarget::OTOS, /*fieldNumber=*/1,
                                      std::nanf("")) == msg::ErrCode::ERR_BADARG,
              "applyField(OTOS, NaN) is refused");
    checkSource(configurator.configSource(msg::ConfigGroupTarget::OTOS),
                msg::ConfigSource::CONFIG_SOURCE_BAKED,
                "a NaN push leaves OTOS reading BAKED");
  }

  // --- 6. ESTIMATOR: config_ WAS mutated, so LIVE is the honest answer ---

  beginScenario(
      "ESTIMATOR reads LIVE even though install() is a permanent "
      "ERR_UNIMPLEMENTED -- config_.estimator holds exactly what was pushed "
      "and read-back returns it, so 'where did what I am reading come from' "
      "is answered by the push, not by whether the fan-out found a consumer");
  {
    uint8_t pushBuf[64];
    size_t pushPos = 0;
    checkTrue(encodeFloatField(1, 0.65f, pushBuf, sizeof(pushBuf), &pushPos),
              "encode weight_heading_otos");
    checkTrue(configurator.applyGroup(msg::ConfigGroupTarget::ESTIMATOR, pushBuf, pushPos) ==
                  msg::ErrCode::ERR_UNIMPLEMENTED,
              "applyGroup(ESTIMATOR) reports ERR_UNIMPLEMENTED (install() has no consumer)");
    checkSource(configurator.configSource(msg::ConfigGroupTarget::ESTIMATOR),
                msg::ConfigSource::CONFIG_SOURCE_LIVE,
                "ESTIMATOR still reads LIVE -- the value did land in config_");
  }

  // --- 7. Reset behaviour falls out of loadBaked() ----------------------

  beginScenario(
      "loadBaked() again (a reset) re-stamps every group BAKED -- including "
      "the ones that were LIVE, with no separate reset handling anywhere");
  {
    configurator.loadBaked();
    checkEveryGroupExcept(configurator, msg::ConfigSource::CONFIG_SOURCE_BAKED,
                          nullptr, 0, "post-reset source");
  }

  // --- 8. encodeSnapshot() carries the source on the REPLY --------------

  beginScenario(
      "encodeSnapshot() reports the source on the ConfigSnapshot reply -- "
      "provenance rides the answer, never the config struct");
  {
    msg::ConfigSnapshot bakedSnapshot;
    checkTrue(configurator.encodeSnapshot(msg::ConfigGroupTarget::DRIVE, bakedSnapshot) ==
                  msg::ErrCode::ERR_NONE,
              "encodeSnapshot(DRIVE) succeeds");
    checkSource(bakedSnapshot.source, msg::ConfigSource::CONFIG_SOURCE_BAKED,
                "a freshly-baked DRIVE reports BAKED on the reply");

    uint8_t pushBuf[64];
    size_t pushPos = 0;
    checkTrue(encodeFloatField(1, 0.77f, pushBuf, sizeof(pushBuf), &pushPos),
              "encode duty_per_speed_left");
    checkTrue(configurator.applyGroup(msg::ConfigGroupTarget::DRIVE, pushBuf, pushPos) ==
                  msg::ErrCode::ERR_NONE,
              "applyGroup(DRIVE) succeeds");

    msg::ConfigSnapshot liveSnapshot;
    checkTrue(configurator.encodeSnapshot(msg::ConfigGroupTarget::DRIVE, liveSnapshot) ==
                  msg::ErrCode::ERR_NONE,
              "encodeSnapshot(DRIVE) succeeds after the push");
    checkSource(liveSnapshot.source, msg::ConfigSource::CONFIG_SOURCE_LIVE,
                "the same group now reports LIVE on the reply");

    // The values and the provenance must agree -- a reply that says LIVE
    // while returning the baked body would be worse than no provenance.
    msg::Drive decoded;
    checkTrue(msg::wire::decode(decoded, liveSnapshot.body(), liveSnapshot.body_count_val()).ok,
              "the LIVE snapshot's own body decodes cleanly");
    checkFloatEq(decoded.duty_per_speed_left, 0.77f,
                "the body carries the pushed value the source claims");

    // A sibling read in the same session still reports BAKED -- per-group
    // provenance survives the trip through the reply, not just the accessor.
    msg::ConfigSnapshot siblingSnapshot;
    checkTrue(configurator.encodeSnapshot(msg::ConfigGroupTarget::PLANNER, siblingSnapshot) ==
                  msg::ErrCode::ERR_NONE,
              "encodeSnapshot(PLANNER) succeeds");
    checkSource(siblingSnapshot.source, msg::ConfigSource::CONFIG_SOURCE_BAKED,
                "an untouched sibling still reports BAKED on its own reply");

    // A rejected read-back reports no source at all rather than a stale one.
    msg::ConfigSnapshot badSnapshot;
    checkTrue(configurator.encodeSnapshot(msg::ConfigGroupTarget::CONFIG_GROUP_UNSPECIFIED,
                                          badSnapshot) == msg::ErrCode::ERR_BADARG,
              "encodeSnapshot(CONFIG_GROUP_UNSPECIFIED) is rejected");
    checkSource(badSnapshot.source, msg::ConfigSource::CONFIG_SOURCE_UNSPECIFIED,
                "a rejected read-back carries no source");
  }

  // --- 9. Flash-restored tuning is PERSISTED, not BAKED -----------------

  beginScenario(
      "reapplyPersistedTuning() stamps the restored groups PERSISTED -- they "
      "came out of flash, not out of the baked file, and reporting them as "
      "BAKED would be a fresh instance of the dishonesty this exists to "
      "remove (a robot running tuned values its read-back denies)");
  {
    Motion::NavigatorLimits freshNavigatorLimits;
    Core::Configurator fresh(drive, motorL, motorR, otos, planner, freshNavigatorLimits, /*tuningStore=*/nullptr);
    fresh.loadBaked();

    Config::TuningSnapshot snapshot;
    snapshot.wheelControlTuned = true;
    snapshot.wheelControlPidKp = 0.21f;
    snapshot.otosTuned = true;
    snapshot.otosLinearScale = 1.0275f;
    // motorsTravelCalibTuned deliberately left FALSE: an untouched persisted
    // group must stay BAKED, proving this is per-group here too and not "any
    // flash content marks everything persisted".
    fresh.reapplyPersistedTuning(snapshot);

    checkSource(fresh.configSource(msg::ConfigGroupTarget::WHEEL_CONTROL),
                msg::ConfigSource::CONFIG_SOURCE_PERSISTED, "restored WHEEL_CONTROL is PERSISTED");
    checkSource(fresh.configSource(msg::ConfigGroupTarget::OTOS),
                msg::ConfigSource::CONFIG_SOURCE_PERSISTED, "restored OTOS is PERSISTED");
    checkSource(fresh.configSource(msg::ConfigGroupTarget::MOTORS),
                msg::ConfigSource::CONFIG_SOURCE_BAKED,
                "MOTORS was not in the snapshot and stays BAKED");
    checkFloatEq(fresh.config().wheelControl.pid_kp, 0.21f,
                "the restored value itself landed, matching its PERSISTED claim");

    // A later live push on a persisted group overtakes PERSISTED.
    checkTrue(fresh.applyField(msg::ConfigGroupTarget::WHEEL_CONTROL, /*fieldNumber=*/1, 0.44f) ==
                  msg::ErrCode::ERR_NONE,
              "applyField(WHEEL_CONTROL, pid_kp) succeeds on a persisted group");
    checkSource(fresh.configSource(msg::ConfigGroupTarget::WHEEL_CONTROL),
                msg::ConfigSource::CONFIG_SOURCE_LIVE,
                "a live push on a PERSISTED group now reads LIVE");
  }

  std::printf("\n");
  if (g_failureCount == 0) {
    std::printf(
        "OK: provenance is per-group (never one global flag), stamped at all "
        "four config_ mutation sites, untouched by every rejection path, "
        "reset by loadBaked(), and carried on the ConfigSnapshot reply\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the 133-006 provenance tests\n", g_failureCount);
  return 1;
}
