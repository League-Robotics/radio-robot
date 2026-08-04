// persisted_tuning_harness.cpp -- off-hardware acceptance harness for
// Config::PersistedTuning's PURE logic (src/firm/config/
// persisted_tuning.{h,cpp}): serializeSnapshot()/deserializeSnapshot()
// round-trip identity and shouldWipe()'s version-compare-and-wipe
// decision. Both are plain functions with NO MicroBitStorage/hardware
// dependency at all -- this harness proves exactly that: it links ONLY
// persisted_tuning.cpp (header-only otherwise) and never touches
// TestSim::SimPlant, a RobotLoop graph, or any bus/hardware fake, unlike
// every other src/tests/sim/unit harness.
//
// RESHAPED, 132-013 (patch-surface retirement): TuningSnapshot is now flat
// plain data (raw floats + 3 per-GROUP "tuned" bools), not the pre-132-013
// curated-live-tuning-message-shaped, per-FIELD Opt<T> presence snapshot
// this harness used to exercise -- see persisted_tuning.h's own doc
// comment for the full reshape rationale. This is a full rewrite of the
// pre-132-013 harness, not an incremental patch: every scenario below
// targets the new shape directly.
//
// The ARM-only Config::MicroBitTuningStore adapter this same .cpp also
// defines (behind #ifndef HOST_BUILD) is explicitly NOT exercised here or
// by any other agent-run test -- see persisted_tuning.h's own file header;
// covered only by a stakeholder bench checklist. This harness compiles
// with -DHOST_BUILD, so that adapter's own code is compiled out entirely
// (never even parsed).
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

// checkSnapshotEq -- compares every TuningSnapshot field (3 group-presence
// bools + 12 floats) as one unit, field by field, so a failure names
// exactly which member diverged rather than a single opaque "not equal".
void checkSnapshotEq(const Config::TuningSnapshot& actual, const Config::TuningSnapshot& expected,
                      const std::string& what) {
  checkTrue(actual.wheelControlTuned == expected.wheelControlTuned, what + ".wheelControlTuned");
  checkTrue(actual.motorsTravelCalibTuned == expected.motorsTravelCalibTuned,
            what + ".motorsTravelCalibTuned");
  checkTrue(actual.otosTuned == expected.otosTuned, what + ".otosTuned");

  checkFloatEq(actual.wheelControlPidKp, expected.wheelControlPidKp, what + ".wheelControlPidKp");
  checkFloatEq(actual.wheelControlPidKi, expected.wheelControlPidKi, what + ".wheelControlPidKi");
  checkFloatEq(actual.wheelControlPidIMax, expected.wheelControlPidIMax,
               what + ".wheelControlPidIMax");
  checkFloatEq(actual.wheelControlPidKaff, expected.wheelControlPidKaff,
               what + ".wheelControlPidKaff");
  checkFloatEq(actual.wheelControlPidMax, expected.wheelControlPidMax, what + ".wheelControlPidMax");

  checkFloatEq(actual.motorsTravelCalibLeft, expected.motorsTravelCalibLeft,
               what + ".motorsTravelCalibLeft");
  checkFloatEq(actual.motorsTravelCalibRight, expected.motorsTravelCalibRight,
               what + ".motorsTravelCalibRight");

  checkFloatEq(actual.otosOffsetX, expected.otosOffsetX, what + ".otosOffsetX");
  checkFloatEq(actual.otosOffsetY, expected.otosOffsetY, what + ".otosOffsetY");
  checkFloatEq(actual.otosOffsetYaw, expected.otosOffsetYaw, what + ".otosOffsetYaw");
  checkFloatEq(actual.otosLinearScale, expected.otosLinearScale, what + ".otosLinearScale");
  checkFloatEq(actual.otosAngularScale, expected.otosAngularScale, what + ".otosAngularScale");
}

// ===========================================================================
// serializeSnapshot()/deserializeSnapshot() round-trip identity -- the
// reshaped surface's own "a live-pushed value survives a save/reload"
// acceptance property, proved here at the pure-function level (no boot, no
// flash).
// ===========================================================================

void scenarioRoundTripFullySetSnapshot() {
  beginScenario("serializeSnapshot()/deserializeSnapshot(): a fully-tuned snapshot round-trips exactly");

  Config::TuningSnapshot original;
  original.wheelControlTuned = true;
  original.wheelControlPidKp = 0.02f;
  original.wheelControlPidKi = 0.01f;
  original.wheelControlPidIMax = 10.0f;
  original.wheelControlPidKaff = 0.5f;
  original.wheelControlPidMax = 0.9f;

  original.motorsTravelCalibTuned = true;
  original.motorsTravelCalibLeft = 1.25f;
  original.motorsTravelCalibRight = -1.30f;  // deliberately DIFFERENT from left --
                                              // proves the two sides don't alias

  original.otosTuned = true;
  original.otosOffsetX = -51.5f;
  original.otosOffsetY = 0.0f;
  original.otosOffsetYaw = 3.14159f;
  original.otosLinearScale = 1.067f;
  original.otosAngularScale = 0.987f;

  Config::Blob blob = Config::serializeSnapshot(original);
  Config::TuningSnapshot roundTripped = Config::deserializeSnapshot(blob);

  checkSnapshotEq(roundTripped, original, "fully-tuned snapshot");
}

void scenarioRoundTripFreshEmptySnapshot() {
  beginScenario("serializeSnapshot()/deserializeSnapshot(): a fresh (all-default) snapshot round-trips to itself");

  Config::TuningSnapshot original;  // every field at its default -- "nothing live-tuned yet"

  Config::Blob blob = Config::serializeSnapshot(original);
  Config::TuningSnapshot roundTripped = Config::deserializeSnapshot(blob);

  checkSnapshotEq(roundTripped, original, "fresh snapshot");
  checkFalse(roundTripped.wheelControlTuned, "fresh snapshot: wheelControlTuned stays false");
  checkFalse(roundTripped.motorsTravelCalibTuned, "fresh snapshot: motorsTravelCalibTuned stays false");
  checkFalse(roundTripped.otosTuned, "fresh snapshot: otosTuned stays false");

  // A fresh snapshot's own blob is the all-zero baseline
  // Configurator::lastPersistedBlob_ starts at (configurator.h's own
  // doc comment) -- confirm that invariant holds here, at the
  // pure-function level, rather than only asserting it implicitly via
  // Configurator's own behavior.
  Config::Blob zeroBlob{};
  checkTrue(blob == zeroBlob, "a fresh TuningSnapshot serializes to the all-zero blob");
}

void scenarioRoundTripOneGroupTunedLeavesOthersUntouched() {
  beginScenario("serializeSnapshot()/deserializeSnapshot(): only-one-group-tuned leaves the other two false");

  Config::TuningSnapshot original;
  original.otosTuned = true;  // ONLY OTOS is tuned; WHEEL_CONTROL/MOTORS stay untouched
  original.otosLinearScale = 1.02f;
  original.otosAngularScale = 0.99f;

  Config::Blob blob = Config::serializeSnapshot(original);
  Config::TuningSnapshot roundTripped = Config::deserializeSnapshot(blob);

  checkTrue(roundTripped.otosTuned, "otosTuned survives the round trip");
  checkFloatEq(roundTripped.otosLinearScale, 1.02f, "otosLinearScale survives the round trip");
  checkFloatEq(roundTripped.otosAngularScale, 0.99f, "otosAngularScale survives the round trip");

  checkFalse(roundTripped.wheelControlTuned,
             "wheelControlTuned (never touched by this snapshot) stays false, not a stray true");
  checkFalse(roundTripped.motorsTravelCalibTuned,
             "motorsTravelCalibTuned (never touched by this snapshot) stays false, not a stray true");
  // The untouched groups' own floats stay at their zero default -- this is
  // exactly why reapplyPersistedTuning() (configurator.cpp) gates each
  // group's config_ write on its own "tuned" flag rather than trusting the
  // float value alone: a zero here does NOT mean "live-tuned to zero," it
  // means "never touched."
  checkFloatEq(roundTripped.wheelControlPidKp, 0.0f, "untouched wheelControlPidKp stays at its zero default");
  checkFloatEq(roundTripped.motorsTravelCalibLeft, 0.0f, "untouched motorsTravelCalibLeft stays at its zero default");
}

// ===========================================================================
// shouldWipe() -- the version-compare-and-wipe DECISION, parametrized
// match/mismatch cases. Unchanged by the 132-013 reshape (the function
// itself is a bare != comparison, no TuningSnapshot dependency) -- kept
// here so this harness stays the one place kConfigSchemaVersion's own
// wipe behavior is exercised.
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
// failure elsewhere). Also the concrete proof for ticket 013's own
// "compute the new blob size" acceptance note: 51 bytes, down from the
// pre-132-013 shape's 85 -- 3 group-presence bytes + 12 floats * 4 bytes,
// no more per-field Opt<T> presence byte.
// ===========================================================================

void scenarioBlobSizeMatchesFieldBudget() {
  beginScenario("Config::kBlobSize matches the field-count budget persisted_tuning.h itself documents");

  constexpr size_t expected =
      Config::kGroupPresenceBytes +
      (Config::kWheelControlFields + Config::kMotorsTravelCalibFields + Config::kOtosFields) *
          Config::kFloatFieldBytes;
  checkUintEq(static_cast<uint32_t>(Config::kBlobSize), static_cast<uint32_t>(expected),
              "kBlobSize == kGroupPresenceBytes + (wheelControlFields+motorsTravelCalibFields+otosFields)*4");
  checkUintEq(static_cast<uint32_t>(Config::kBlobSize), 51u,
              "kBlobSize == 51 (3 presence bytes + 12 floats * 4 bytes) -- 132-013's own reshape figure");
}

}  // namespace

int main() {
  scenarioRoundTripFullySetSnapshot();
  scenarioRoundTripFreshEmptySnapshot();
  scenarioRoundTripOneGroupTunedLeavesOthersUntouched();
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
