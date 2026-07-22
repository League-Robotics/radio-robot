// motion_stop_condition_harness.cpp -- off-hardware acceptance harness for
// ticket 116-002 (SUC-050/SUC-051/SUC-052/SUC-054), Motion::StopCondition
// (src/firm/motion/stop_condition.{h,cpp}).
//
// Unlike app_deadman_harness.cpp (which needs TestSim::SimClock because
// App::Deadman holds a Devices::Clock& and reads nowMicros() itself),
// Motion::StopCondition takes every reading as a plain parameter (see
// stop_condition.h's file header) -- this harness hand-feeds uint64_t
// microsecond timestamps and float readings directly, with no clock/bus
// fake of any kind. That is itself part of what this harness proves: the
// module compiles and runs with zero collaborators.
//
// Mirrors app_deadman_harness.cpp's exact shape otherwise: hand-rolled
// assertion plumbing, PASS/FAIL printf, exit nonzero on failure. Compiled
// by test_motion_stop_condition.py with -DHOST_BUILD against
// stop_condition.cpp only.
#include <cstdint>
#include <cstdio>
#include <string>

#include "motion/stop_condition.h"

namespace {

using Motion::StopCondition;

// --- Hand-rolled assertion plumbing (see app_deadman_harness.cpp) --------

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

void checkOutcome(StopCondition::Outcome actual, StopCondition::Outcome expected,
                   const std::string& what) {
  if (actual != expected) {
    fail(what + " -- outcome mismatch");
  }
}

// --- Scenarios -------------------------------------------------------------

// 1. TIME kind fires AT/AFTER the commanded elapsed time, not before.
void scenarioTimeFiresAtDeadline() {
  beginScenario("Kind::Time: Continue before the deadline, StopConditionMet at/after it");
  StopCondition sc(StopCondition::Kind::Time, /*threshold=*/100.0f /*[ms]*/,
                    /*timeout=*/5000.0f /*[ms]*/, /*now=*/0, /*pathLength=*/0.0f,
                    /*theta=*/0.0f);

  checkOutcome(sc.tick(0, 0.0f, 0.0f), StopCondition::Outcome::Continue,
               "not met immediately at now=0");
  checkOutcome(sc.tick(99000, 0.0f, 0.0f), StopCondition::Outcome::Continue,
               "not met at 99ms (< 100ms threshold)");
  checkOutcome(sc.tick(100000, 0.0f, 0.0f), StopCondition::Outcome::StopConditionMet,
               "met AT the 100ms threshold (>=, not >)");
  checkOutcome(sc.tick(150000, 0.0f, 0.0f), StopCondition::Outcome::StopConditionMet,
               "still met well past the threshold");
}

// 2. DISTANCE kind fires when |pathLength() - baseline| >= threshold, in
//    either direction of travel (fabs).
void scenarioDistanceFiresAtThresholdEitherDirection() {
  beginScenario("Kind::Distance: |pathLength - baseline| >= threshold, forward and reverse");
  StopCondition fwd(StopCondition::Kind::Distance, /*threshold=*/50.0f /*[mm]*/,
                     /*timeout=*/5000.0f, /*now=*/0, /*pathLength=*/1000.0f,
                     /*theta=*/0.0f);
  checkOutcome(fwd.tick(1000, 1049.0f, 0.0f), StopCondition::Outcome::Continue,
               "not met at 49mm traveled forward (< 50mm threshold)");
  checkOutcome(fwd.tick(2000, 1050.0f, 0.0f), StopCondition::Outcome::StopConditionMet,
               "met AT 50mm traveled forward (>=, not >)");

  StopCondition rev(StopCondition::Kind::Distance, /*threshold=*/50.0f,
                     /*timeout=*/5000.0f, /*now=*/0, /*pathLength=*/1000.0f,
                     /*theta=*/0.0f);
  checkOutcome(rev.tick(1000, 951.0f, 0.0f), StopCondition::Outcome::Continue,
               "not met at 49mm traveled in reverse (< 50mm threshold)");
  checkOutcome(rev.tick(2000, 950.0f, 0.0f), StopCondition::Outcome::StopConditionMet,
               "met AT 50mm traveled in reverse -- fabs(), direction-agnostic");
}

// 3. ANGLE kind fires when |theta() - baseline| >= threshold, with NO
//    wrap/modulo applied -- an unwrapped theta well past +-pi still diffs
//    correctly against its own baseline.
void scenarioAngleFiresUnwrappedNoModulo() {
  beginScenario("Kind::Angle: |theta - baseline| >= threshold, unwrapped, no modulo");
  // Baseline theta chosen past +pi (10.0 rad) to prove no wrap handling is
  // applied anywhere in the comparison -- odometry.cpp's own theta_ is
  // documented unwrapped, so a caller-supplied baseline this large is a
  // legitimate, expected input, not an edge case needing special handling.
  StopCondition sc(StopCondition::Kind::Angle, /*threshold=*/1.0f /*[rad]*/,
                    /*timeout=*/5000.0f, /*now=*/0, /*pathLength=*/0.0f,
                    /*theta=*/10.0f);

  checkOutcome(sc.tick(1000, 0.0f, 10.99f), StopCondition::Outcome::Continue,
               "not met at 0.99rad turned (< 1.0rad threshold)");
  checkOutcome(sc.tick(2000, 0.0f, 11.0f), StopCondition::Outcome::StopConditionMet,
               "met AT 1.0rad turned (>=, not >)");

  // Reverse rotation (theta decreasing) fires identically via fabs().
  StopCondition scRev(StopCondition::Kind::Angle, /*threshold=*/1.0f,
                       /*timeout=*/5000.0f, /*now=*/0, /*pathLength=*/0.0f,
                       /*theta=*/10.0f);
  checkOutcome(scRev.tick(1000, 0.0f, 9.0f), StopCondition::Outcome::StopConditionMet,
               "met turning the OTHER way (theta decreasing) -- fabs(), direction-agnostic");
}

// 4. TIMEOUT fires independent of kind, whenever elapsed time reaches
//    `timeout`, even though the kind-specific (Distance) condition never
//    fires at all (the "stalled wheels" scenario the wire spec names).
void scenarioTimeoutFiresIndependentOfKindWhenConditionUnreachable() {
  beginScenario("TIMEOUT: fires at `timeout` even when the Distance condition never progresses");
  StopCondition sc(StopCondition::Kind::Distance, /*threshold=*/500.0f /*[mm], never reached*/,
                    /*timeout=*/200.0f /*[ms]*/, /*now=*/0, /*pathLength=*/0.0f,
                    /*theta=*/0.0f);

  // pathLength never advances past the baseline (0.0f) -- wheels stalled.
  checkOutcome(sc.tick(100000, 0.0f, 0.0f), StopCondition::Outcome::Continue,
               "not timed out yet at 100ms (< 200ms timeout)");
  checkOutcome(sc.tick(199000, 0.0f, 0.0f), StopCondition::Outcome::Continue,
               "not timed out at 199ms (< 200ms timeout)");
  checkOutcome(sc.tick(200000, 0.0f, 0.0f), StopCondition::Outcome::TimedOut,
               "TimedOut AT the 200ms timeout (>=, not >), Distance condition never met");
}

// 4b. TIMEOUT applies to Kind::Time too, when threshold > timeout (a
//     malformed-but-hand-fed case: the commanded TIME target is longer
//     than the safety backstop) -- the backstop still wins.
void scenarioTimeoutOverridesUnreachableTimeThreshold() {
  beginScenario("TIMEOUT: applies to Kind::Time itself when threshold > timeout");
  StopCondition sc(StopCondition::Kind::Time, /*threshold=*/1000.0f /*[ms]*/,
                    /*timeout=*/200.0f /*[ms], shorter than the TIME threshold*/,
                    /*now=*/0, /*pathLength=*/0.0f, /*theta=*/0.0f);

  checkOutcome(sc.tick(199000, 0.0f, 0.0f), StopCondition::Outcome::Continue,
               "neither fired yet at 199ms");
  checkOutcome(sc.tick(200000, 0.0f, 0.0f), StopCondition::Outcome::TimedOut,
               "TimedOut at 200ms -- the 1000ms TIME threshold was never reachable first");
}

// 5. Tie-break: when BOTH the kind-specific condition and timeout are met
//    the SAME cycle, StopConditionMet wins -- never TimedOut.
void scenarioTieBreakKindSpecificWinsOverTimeout() {
  beginScenario("Tie-break: kind-specific StopConditionMet wins over TimedOut on the same cycle");

  // Kind::Time, threshold == timeout exactly: at the shared deadline, both
  // the kind-specific and timeout comparisons are simultaneously true.
  StopCondition scTime(StopCondition::Kind::Time, /*threshold=*/100.0f,
                        /*timeout=*/100.0f, /*now=*/0, /*pathLength=*/0.0f,
                        /*theta=*/0.0f);
  checkOutcome(scTime.tick(100000, 0.0f, 0.0f), StopCondition::Outcome::StopConditionMet,
               "Kind::Time threshold==timeout: StopConditionMet, not TimedOut");

  // Kind::Distance reaching its threshold on the exact same cycle the
  // (unrelated) timeout also elapses.
  StopCondition scDist(StopCondition::Kind::Distance, /*threshold=*/50.0f,
                        /*timeout=*/200.0f, /*now=*/0, /*pathLength=*/1000.0f,
                        /*theta=*/0.0f);
  checkOutcome(scDist.tick(200000, 1050.0f, 0.0f), StopCondition::Outcome::StopConditionMet,
               "Distance threshold met exactly when timeout also elapses: StopConditionMet wins");
}

// 6. Zero/negative/NaN threshold and timeout -- the pinned Open Question 1
//    convention: clamp to 0, fires StopConditionMet on the very FIRST
//    tick() call, uniformly across all three kinds.
void scenarioNonPositiveThresholdFiresImmediately() {
  beginScenario("Zero/negative/NaN threshold clamps to 0 -- StopConditionMet on the first tick()");

  StopCondition zeroTime(StopCondition::Kind::Time, /*threshold=*/0.0f,
                          /*timeout=*/5000.0f, /*now=*/1000, /*pathLength=*/0.0f,
                          /*theta=*/0.0f);
  checkOutcome(zeroTime.tick(1000, 0.0f, 0.0f), StopCondition::Outcome::StopConditionMet,
               "Kind::Time threshold=0: met on the very first tick() at the SAME now as activation");

  StopCondition negTime(StopCondition::Kind::Time, /*threshold=*/-50.0f,
                         /*timeout=*/5000.0f, /*now=*/1000, /*pathLength=*/0.0f,
                         /*theta=*/0.0f);
  checkOutcome(negTime.tick(1000, 0.0f, 0.0f), StopCondition::Outcome::StopConditionMet,
               "Kind::Time threshold=negative: clamps to 0, same immediate-met behavior");

  float nan = 0.0f / 0.0f;
  StopCondition nanTime(StopCondition::Kind::Time, /*threshold=*/nan,
                         /*timeout=*/5000.0f, /*now=*/1000, /*pathLength=*/0.0f,
                         /*theta=*/0.0f);
  checkOutcome(nanTime.tick(1000, 0.0f, 0.0f), StopCondition::Outcome::StopConditionMet,
               "Kind::Time threshold=NaN: clamps to 0 (NaN comparisons are always false), immediate-met");

  StopCondition zeroDist(StopCondition::Kind::Distance, /*threshold=*/0.0f,
                          /*timeout=*/5000.0f, /*now=*/1000, /*pathLength=*/500.0f,
                          /*theta=*/0.0f);
  checkOutcome(zeroDist.tick(1000, 500.0f, 0.0f), StopCondition::Outcome::StopConditionMet,
               "Kind::Distance threshold=0: met on the first tick() with zero travel so far");

  StopCondition zeroAngle(StopCondition::Kind::Angle, /*threshold=*/0.0f,
                           /*timeout=*/5000.0f, /*now=*/1000, /*pathLength=*/0.0f,
                           /*theta=*/2.5f);
  checkOutcome(zeroAngle.tick(1000, 0.0f, 2.5f), StopCondition::Outcome::StopConditionMet,
               "Kind::Angle threshold=0: met on the first tick() with zero turn so far");
}

// 6b. Zero/negative timeout: clamps to 0, fires TimedOut on the very
//     first tick() call -- UNLESS the kind-specific condition also fires
//     that same call (tie-break, scenario 5), which a normal unreached
//     Distance/Angle/Time threshold does not.
void scenarioNonPositiveTimeoutFiresImmediately() {
  beginScenario("Zero/negative timeout clamps to 0 -- TimedOut on the first tick() (no tie)");

  StopCondition zeroTimeout(StopCondition::Kind::Distance, /*threshold=*/500.0f /*not yet reached*/,
                             /*timeout=*/0.0f, /*now=*/1000, /*pathLength=*/0.0f,
                             /*theta=*/0.0f);
  checkOutcome(zeroTimeout.tick(1000, 0.0f, 0.0f), StopCondition::Outcome::TimedOut,
               "timeout=0: TimedOut on the very first tick(), Distance threshold nowhere near met");

  StopCondition negTimeout(StopCondition::Kind::Distance, /*threshold=*/500.0f,
                            /*timeout=*/-10.0f, /*now=*/1000, /*pathLength=*/0.0f,
                            /*theta=*/0.0f);
  checkOutcome(negTimeout.tick(1000, 0.0f, 0.0f), StopCondition::Outcome::TimedOut,
               "timeout=negative: clamps to 0, same immediate-TimedOut behavior");

  // Both threshold and timeout non-positive at once: the tie-break (kind-
  // specific wins) still applies -- StopConditionMet, not TimedOut.
  StopCondition bothZero(StopCondition::Kind::Distance, /*threshold=*/0.0f,
                          /*timeout=*/0.0f, /*now=*/1000, /*pathLength=*/0.0f,
                          /*theta=*/0.0f);
  checkOutcome(bothZero.tick(1000, 0.0f, 0.0f), StopCondition::Outcome::StopConditionMet,
               "threshold=0 AND timeout=0: tie-break still gives StopConditionMet, not TimedOut");
}

// 7. Module has zero dependency on App::MoveQueue/App::Drive/msg::* wire
//    types -- structurally proven by this harness's own #include list
//    (only <cstdint>/<cstdio>/<string>/motion/stop_condition.h) compiling
//    and linking with ONLY stop_condition.cpp, nothing else.
void scenarioZeroDependencyByConstruction() {
  beginScenario("Zero dependency on MoveQueue/Drive/msg::* -- proven by this TU's own link graph");
  // No assertion beyond "this file compiled and linked" -- see doc comment
  // above. A regression that adds a MoveQueue/Drive/messages/ include to
  // stop_condition.h would still compile fine in isolation as long as this
  // harness happens not to trigger a missing symbol, so this scenario is a
  // documentation marker, not a mechanical guarantee; the real guarantee is
  // this .py driver's own compile command listing ONLY stop_condition.cpp.
  StopCondition sc(StopCondition::Kind::Time, 10.0f, 20.0f, 0, 0.0f, 0.0f);
  (void)sc.tick(0, 0.0f, 0.0f);
}

}  // namespace

int main() {
  scenarioTimeFiresAtDeadline();
  scenarioDistanceFiresAtThresholdEitherDirection();
  scenarioAngleFiresUnwrappedNoModulo();
  scenarioTimeoutFiresIndependentOfKindWhenConditionUnreachable();
  scenarioTimeoutOverridesUnreachableTimeThreshold();
  scenarioTieBreakKindSpecificWinsOverTimeout();
  scenarioNonPositiveThresholdFiresImmediately();
  scenarioNonPositiveTimeoutFiresImmediately();
  scenarioZeroDependencyByConstruction();

  if (g_failureCount == 0) {
    std::printf("OK: all Motion::StopCondition scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the Motion::StopCondition scenarios\n",
              g_failureCount);
  return 1;
}
