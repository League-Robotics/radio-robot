// persisted_tuning_harness.cpp -- off-hardware acceptance harness for
// ticket 114-004 (SUC-003), Config::PersistedTuning's PURE logic
// (src/firm/config/persisted_tuning.{h,cpp}): serializeSnapshot()/
// deserializeSnapshot() round-trip identity and shouldWipe()'s
// version-compare-and-wipe decision. Both are plain functions with NO
// MicroBitStorage/hardware dependency at all -- this harness proves
// exactly that: it links ONLY persisted_tuning.cpp (plus messages/config.h,
// header-only) and never touches TestSim::SimPlant, a RobotLoop graph, or
// any bus/hardware fake, unlike every other src/tests/sim/unit harness.
//
// The ARM-only Config::MicroBitTuningStore adapter this same .cpp also
// defines (behind #ifndef HOST_BUILD) is explicitly NOT exercised here or
// by any other agent-run test -- see persisted_tuning.h's own file header;
// covered only by ticket 006's stakeholder bench checklist. This harness
// compiles with -DHOST_BUILD, so that adapter's own code is compiled out
// entirely (never even parsed).
//
// Hand-rolled assertions -- mirrors measurement_ring_harness.cpp's shape
// exactly (this codebase's established per-harness-file style for a
// header/small-module pure-logic proof).
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include "config/persisted_tuning.h"

namespace {

// --- Hand-rolled assertion plumbing (see measurement_ring_harness.cpp) ---

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

void checkFloatEq(float actual, float expected, const std::string& what, float tol = 1e-6f) {
  if (std::fabs(actual - expected) > tol) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %g, got %g", what.c_str(),
                  static_cast<double>(expected), static_cast<double>(actual));
    fail(buf);
  }
}

void checkUintEq(uint32_t actual, uint32_t expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %u, got %u", what.c_str(),
                  static_cast<unsigned>(expected), static_cast<unsigned>(actual));
    fail(buf);
  }
}

// checkOptFloatEq -- compares an msg::Opt<float> field's (has, val) pair
// as one unit; val is only checked when BOTH sides claim has==true (a
// mismatched-has case already fails on its own via the has comparison,
// and comparing an unset val would be comparing two don't-care defaults).
void checkOptFloatEq(const msg::Opt<float>& actual, const msg::Opt<float>& expected,
                      const std::string& what) {
  if (actual.has != expected.has) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s.has -- expected %d, got %d", what.c_str(),
                  expected.has, actual.has);
    fail(buf);
    return;
  }
  if (expected.has) {
    checkFloatEq(actual.val, expected.val, what + ".val");
  }
}

void checkMotorPatchEq(const msg::MotorConfigPatch& actual, const msg::MotorConfigPatch& expected,
                        const std::string& what) {
  checkOptFloatEq(actual.travel_calib, expected.travel_calib, what + ".travel_calib");
  checkOptFloatEq(actual.kp, expected.kp, what + ".kp");
  checkOptFloatEq(actual.ki, expected.ki, what + ".ki");
  checkOptFloatEq(actual.kff, expected.kff, what + ".kff");
  checkOptFloatEq(actual.i_max, expected.i_max, what + ".i_max");
  checkOptFloatEq(actual.kaw, expected.kaw, what + ".kaw");
}

void checkOtosPatchEq(const msg::OtosConfigPatch& actual, const msg::OtosConfigPatch& expected,
                       const std::string& what) {
  checkOptFloatEq(actual.linear_scale, expected.linear_scale, what + ".linear_scale");
  checkOptFloatEq(actual.angular_scale, expected.angular_scale, what + ".angular_scale");
  checkOptFloatEq(actual.offset_x, expected.offset_x, what + ".offset_x");
  checkOptFloatEq(actual.offset_y, expected.offset_y, what + ".offset_y");
  checkOptFloatEq(actual.offset_yaw, expected.offset_yaw, what + ".offset_yaw");
}

msg::Opt<float> opt(float v) {
  msg::Opt<float> o;
  o.has = true;
  o.val = v;
  return o;
}

// ===========================================================================
// serializeSnapshot()/deserializeSnapshot() round-trip identity -- SUC-003's
// own "version-match round-trip: pushed patch value observed unchanged"
// acceptance criterion, proved here at the pure-function level (no boot, no
// flash).
// ===========================================================================

void scenarioRoundTripFullySetSnapshot() {
  beginScenario("serializeSnapshot()/deserializeSnapshot(): a fully-populated snapshot round-trips exactly");

  Config::TuningSnapshot original;
  original.motorL.side = msg::BoundMotorSide::LEFT;
  original.motorL.travel_calib = opt(1.25f);
  original.motorL.kp = opt(0.02f);
  original.motorL.ki = opt(0.01f);
  original.motorL.kff = opt(0.5f);
  original.motorL.i_max = opt(10.0f);
  original.motorL.kaw = opt(0.1f);

  original.motorR.side = msg::BoundMotorSide::RIGHT;
  original.motorR.travel_calib = opt(-1.30f);  // deliberately DIFFERENT from motorL's --
                                                // proves the two sides don't alias
  original.motorR.kp = opt(0.02f);   // gains mirror in practice (RobotLoop's own merge),
  original.motorR.ki = opt(0.01f);   // but the pure serializer must not assume that --
  original.motorR.kff = opt(0.5f);   // it persists whatever TuningSnapshot actually holds.
  original.motorR.i_max = opt(10.0f);
  original.motorR.kaw = opt(0.1f);

  original.otos.linear_scale = opt(1.067f);
  original.otos.angular_scale = opt(0.987f);
  original.otos.offset_x = opt(-51.5f);
  original.otos.offset_y = opt(0.0f);
  original.otos.offset_yaw = opt(3.14159f);
  original.otos.init = true;  // deliberately set on the INPUT struct -- proves it is
                               // dropped, not merely "usually false"

  Config::Blob blob = Config::serializeSnapshot(original);
  Config::TuningSnapshot roundTripped = Config::deserializeSnapshot(blob);

  checkMotorPatchEq(roundTripped.motorL, original.motorL, "motorL");
  checkMotorPatchEq(roundTripped.motorR, original.motorR, "motorR");
  checkOtosPatchEq(roundTripped.otos, original.otos, "otos");

  checkTrue(roundTripped.motorL.side == msg::BoundMotorSide::LEFT, "deserializeSnapshot() stamps motorL.side == LEFT");
  checkTrue(roundTripped.motorR.side == msg::BoundMotorSide::RIGHT, "deserializeSnapshot() stamps motorR.side == RIGHT");
  checkFalse(roundTripped.otos.init,
             "otos.init is NEVER round-tripped (a one-shot trigger, not a persisted value) "
             "even though the INPUT snapshot had it set");
}

void scenarioRoundTripFreshEmptySnapshot() {
  beginScenario("serializeSnapshot()/deserializeSnapshot(): a fresh (all has=false) snapshot round-trips to itself");

  Config::TuningSnapshot original;  // every Opt<T>{has=false} default -- "nothing live-tuned yet"

  Config::Blob blob = Config::serializeSnapshot(original);
  Config::TuningSnapshot roundTripped = Config::deserializeSnapshot(blob);

  checkMotorPatchEq(roundTripped.motorL, original.motorL, "motorL");
  checkMotorPatchEq(roundTripped.motorR, original.motorR, "motorR");
  checkOtosPatchEq(roundTripped.otos, original.otos, "otos");

  // A fresh snapshot's own blob is the all-zero baseline
  // RobotLoop::lastPersistedBlob_ starts at (robot_loop.h's own doc
  // comment) -- confirm that invariant holds here, at the pure-function
  // level, rather than only asserting it implicitly via RobotLoop's own
  // behavior.
  Config::Blob zeroBlob{};
  checkTrue(blob == zeroBlob, "a fresh TuningSnapshot serializes to the all-zero blob");
}

void scenarioRoundTripPartiallySetSnapshotPreservesAbsentFields() {
  beginScenario("serializeSnapshot()/deserializeSnapshot(): only-some-fields-set stays has=false for the rest");

  Config::TuningSnapshot original;
  original.motorL.kp = opt(0.02f);  // ONLY kp set on motorL; everything else stays has=false
  original.otos.angular_scale = opt(0.987f);  // ONLY angular_scale set

  Config::Blob blob = Config::serializeSnapshot(original);
  Config::TuningSnapshot roundTripped = Config::deserializeSnapshot(blob);

  checkTrue(roundTripped.motorL.kp.has, "motorL.kp.has survives the round trip");
  checkFloatEq(roundTripped.motorL.kp.val, 0.02f, "motorL.kp.val survives the round trip");
  checkFalse(roundTripped.motorL.ki.has, "motorL.ki (never set) stays has=false, not a stray true");
  checkFalse(roundTripped.motorL.travel_calib.has, "motorL.travel_calib (never set) stays has=false");
  checkFalse(roundTripped.motorR.kp.has, "motorR.kp (never touched by this snapshot) stays has=false");

  checkTrue(roundTripped.otos.angular_scale.has, "otos.angular_scale.has survives the round trip");
  checkFloatEq(roundTripped.otos.angular_scale.val, 0.987f, "otos.angular_scale.val survives the round trip");
  checkFalse(roundTripped.otos.linear_scale.has, "otos.linear_scale (never set) stays has=false");
}

// ===========================================================================
// shouldWipe() -- the version-compare-and-wipe DECISION (SUC-003's own
// "version-mismatch: shouldWipe() returns true" acceptance criterion),
// parametrized match/mismatch cases.
// ===========================================================================

void scenarioShouldWipeMatchingVersionsReturnFalse() {
  beginScenario("shouldWipe(): matching versions -> false (reapply, don't wipe)");

  checkFalse(Config::shouldWipe(1, 1), "shouldWipe(1, 1) -- same nonzero version");
  checkFalse(Config::shouldWipe(0, 0), "shouldWipe(0, 0) -- same zero version");
  checkFalse(Config::shouldWipe(42, 42), "shouldWipe(42, 42) -- same arbitrary version");
  checkFalse(Config::shouldWipe(Config::kConfigSchemaVersion, Config::kConfigSchemaVersion),
             "shouldWipe(kConfigSchemaVersion, kConfigSchemaVersion) -- the real compiled constant against itself");
}

void scenarioShouldWipeMismatchedVersionsReturnTrue() {
  beginScenario("shouldWipe(): mismatched versions -> true (wipe the entire store)");

  checkTrue(Config::shouldWipe(1, 2), "shouldWipe(1, 2) -- stored older than current");
  checkTrue(Config::shouldWipe(2, 1), "shouldWipe(2, 1) -- stored newer than current (e.g. a downgrade/rollback)");
  checkTrue(Config::shouldWipe(0, 1), "shouldWipe(0, 1) -- stored at the zero/never-versioned sentinel");
  checkTrue(Config::shouldWipe(1, 0), "shouldWipe(1, 0) -- current somehow reads as the zero sentinel");
}

// ===========================================================================
// kBlobSize sanity -- greppable, explicit budget check (not load-bearing
// for correctness, but catches an accidental field-count/size drift
// immediately rather than only via a byte-offset-shifted round-trip
// failure elsewhere).
// ===========================================================================

void scenarioBlobSizeMatchesFieldBudget() {
  beginScenario("Config::kBlobSize matches the field-count budget persisted_tuning.h itself documents");

  // 2 motor patches * 6 fields + 1 otos patch * 5 fields, each field 5
  // bytes (1 has + 4 float) -- see persisted_tuning.h's own
  // kOptFloatBytes/kMotorPatchFields/kOtosPatchFields constants, which
  // this expression mirrors exactly (not a re-derived magic number). The
  // planner term is GONE (115-004, gut S1) -- kBlobSize is 85, not 110.
  constexpr size_t expected = (2 * Config::kMotorPatchFields * Config::kOptFloatBytes) +
                               (Config::kOtosPatchFields * Config::kOptFloatBytes);
  checkUintEq(static_cast<uint32_t>(Config::kBlobSize), static_cast<uint32_t>(expected),
              "kBlobSize == 2*motorFields*5 + otosFields*5");
}

}  // namespace

int main() {
  scenarioRoundTripFullySetSnapshot();
  scenarioRoundTripFreshEmptySnapshot();
  scenarioRoundTripPartiallySetSnapshotPreservesAbsentFields();
  scenarioShouldWipeMatchingVersionsReturnFalse();
  scenarioShouldWipeMismatchedVersionsReturnTrue();
  scenarioBlobSizeMatchesFieldBudget();

  if (g_failureCount == 0) {
    std::printf("OK: all Config::PersistedTuning pure-logic scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the Config::PersistedTuning scenarios\n", g_failureCount);
  return 1;
}
