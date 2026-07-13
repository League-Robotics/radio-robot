// drive_master_profile_harness.cpp -- off-hardware acceptance harness for
// ticket 100-002 (SUC-002/SUC-008): exercises Drive::MasterProfile
// (source/drive/master_profile.{h,cpp}) in isolation, mirroring
// jerk_trajectory_harness.cpp's/ruckig_smoke_harness.cpp's compile-and-run
// pattern -- hand-rolled assertions, no gtest/pytest-native C++ framework,
// run via test_drive_master_profile.py.
//
// Scenarios (the ticket's own "solveToExit boundary tuples"):
//  (a) solveToExit(target, 0.0f, maxVel) is PROVABLY the same solve as
//      solveToRest(target, maxVel): same duration, same trace shape.
//  (b) a nonzero exitVelocity within maxVelocity, same sign as the
//      direction of travel, solves and arrives at the target position AT
//      the requested exit velocity (not zero), never reversing.
//  (c) |exitVelocity| > maxVelocity fails cleanly (returns false) and does
//      NOT corrupt the previously in-flight trajectory -- never UB.
//  (d) an exitVelocity of the WRONG sign (opposite the direction of
//      travel) also fails cleanly, even though |exitVelocity| <=
//      maxVelocity -- the same-sign band.
//  (e) the jerk sentinel (0.0f -> Ruckig's own +infinity) is preserved:
//      same shape as an explicit huge-but-finite jerk.
//  (f) the seeding contract: solveToExit()'s initial sampled state is
//      exactly the seed set by seedCurrent(), never anything else --
//      pinning that this class has no measured-observation parameter
//      anywhere in its public API to leak one through.
//  (g) negative-direction solveToExit with a negative exitVelocity is
//      symmetric to (b).
#include <cmath>
#include <cstdio>
#include <string>

#include "drive/master_profile.h"
#include "drive/types.h"

namespace {

// --- Hand-rolled assertion plumbing (mirrors jerk_trajectory_harness.cpp) ---

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

void checkLe(double actual, double bound, const std::string& what) {
  if (!(actual <= bound)) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected <= %g, got %g", what.c_str(), bound, actual);
    fail(buf);
  }
}

void checkNear(double actual, double expected, double tol, const std::string& what) {
  if (std::fabs(actual - expected) > tol) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected ~%g (tol %g), got %g", what.c_str(), expected,
                  tol, actual);
    fail(buf);
  }
}

Drive::ProfileLimits makeLimits(float accel, float decel, float velocity, float jerk) {
  Drive::ProfileLimits limits;
  limits.accel = accel;
  limits.decel = decel;
  limits.velocity = velocity;
  limits.jerk = jerk;
  return limits;
}

struct Trace {
  double minVel = 1e9;
  double maxVel = -1e9;
  Drive::MasterProfile::State end;
};

Trace traceSpan(Drive::MasterProfile& channel, float tStart, float span, int samples = 400) {
  Trace trace;
  for (int i = 0; i <= samples; ++i) {
    float t = tStart + span * static_cast<float>(i) / static_cast<float>(samples);
    Drive::MasterProfile::State state = channel.sample(t);
    if (state.velocity < trace.minVel) trace.minVel = state.velocity;
    if (state.velocity > trace.maxVel) trace.maxVel = state.velocity;
    trace.end = state;
  }
  return trace;
}

// --- Scenarios ---

// (a) solveToExit(target, 0.0f, maxVel) is provably the same solve as
// solveToRest(target, maxVel): identical duration and end-state.
void scenarioZeroExitMatchesSolveToRest() {
  beginScenario("solveToExit(target, 0, maxVel) matches solveToRest(target, maxVel)");

  Drive::MasterProfile restChannel;
  restChannel.configure(makeLimits(800.0f, 800.0f, 250.0f, 0.0f));
  restChannel.reset();
  checkTrue(restChannel.solveToRest(1000.0f, 250.0f), "solveToRest() succeeds");
  float restDuration = restChannel.duration();
  Trace restTrace = traceSpan(restChannel, 0.0f, restDuration);

  Drive::MasterProfile exitChannel;
  exitChannel.configure(makeLimits(800.0f, 800.0f, 250.0f, 0.0f));
  exitChannel.reset();
  checkTrue(exitChannel.solveToExit(1000.0f, 0.0f, 250.0f), "solveToExit(., 0, .) succeeds");
  float exitDuration = exitChannel.duration();
  Trace exitTrace = traceSpan(exitChannel, 0.0f, exitDuration);

  checkNear(exitDuration, restDuration, 1e-4, "same duration");
  checkNear(exitTrace.end.position, restTrace.end.position, 1e-4, "same end position");
  checkNear(exitTrace.end.velocity, restTrace.end.velocity, 1e-4, "same end velocity (0)");
  checkTrue(restTrace.minVel >= -0.5, "solveToRest never reverses");
  checkTrue(exitTrace.minVel >= -0.5, "solveToExit(., 0, .) never reverses");
}

// (b) A nonzero exitVelocity within maxVelocity, same sign as the
// direction of travel, solves and arrives AT that exit velocity.
void scenarioNonzeroExitWithinMaxVelocitySolves() {
  beginScenario("nonzero exitVelocity within maxVelocity solves, arrives at exit speed");

  Drive::MasterProfile channel;
  channel.configure(makeLimits(800.0f, 800.0f, 250.0f, 0.0f));
  channel.reset();

  checkTrue(channel.solveToExit(1000.0f, 120.0f, 250.0f),
            "solveToExit(1000, exit=120, max=250) succeeds");
  float dur = channel.duration();
  checkTrue(dur > 0.0f, "trajectory duration > 0");

  Trace trace = traceSpan(channel, 0.0f, dur);
  checkTrue(trace.minVel >= -0.5, "velocity never goes negative (no reverse)");
  checkLe(trace.maxVel, 250.0 + 0.5, "velocity respects maxVelocity (250)");
  checkNear(trace.end.position, 1000.0, 1.0, "arrives AT the target position");
  checkNear(trace.end.velocity, 120.0, 0.5, "arrives AT the requested exit velocity (120)");
}

// (c) |exitVelocity| > maxVelocity fails cleanly and does not corrupt the
// previously in-flight trajectory.
void scenarioExitVelocityExceedsMaxVelocityFailsCleanly() {
  beginScenario("|exitVelocity| > maxVelocity fails cleanly, prior trajectory intact");

  Drive::MasterProfile channel;
  channel.configure(makeLimits(800.0f, 800.0f, 250.0f, 0.0f));
  channel.reset();

  checkTrue(channel.solveToExit(1000.0f, 50.0f, 250.0f), "initial valid solve succeeds");
  float priorDuration = channel.duration();
  Drive::MasterProfile::State priorEnd = channel.sample(priorDuration);

  // exitVelocity (300) exceeds maxVelocity (250) -- must fail cleanly.
  checkFalse(channel.solveToExit(2000.0f, 300.0f, 250.0f),
             "solveToExit with |exitVelocity| > maxVelocity returns false");

  // The previous trajectory must still be intact and safely sample-able
  // (the "solve into a temporary" discipline -- a failed solve never
  // corrupts traj_).
  checkNear(channel.duration(), priorDuration, 1e-6, "duration unchanged after failed solve");
  Drive::MasterProfile::State stillEnd = channel.sample(priorDuration);
  checkNear(stillEnd.position, priorEnd.position, 1e-4, "prior trajectory position intact");
  checkNear(stillEnd.velocity, priorEnd.velocity, 1e-4, "prior trajectory velocity intact");
}

// (d) An exitVelocity of the WRONG sign (opposite the direction of travel)
// also fails cleanly, even though its magnitude is within maxVelocity --
// the same-sign band generalization.
void scenarioWrongSignExitVelocityFailsCleanly() {
  beginScenario("exitVelocity of the wrong sign fails cleanly (same-sign band)");

  Drive::MasterProfile channel;
  channel.configure(makeLimits(800.0f, 800.0f, 250.0f, 0.0f));
  channel.reset();

  // Target is AHEAD (positive direction), but exitVelocity is NEGATIVE --
  // physically nonsensical (arriving at a forward target moving backward).
  checkFalse(channel.solveToExit(1000.0f, -50.0f, 250.0f),
             "positive-direction solve with negative exitVelocity fails cleanly");
  checkTrue(channel.duration() == 0.0f, "never-solved channel still reports duration 0");

  // Symmetric: target BEHIND (negative direction), positive exitVelocity.
  Drive::MasterProfile channel2;
  channel2.configure(makeLimits(800.0f, 800.0f, 250.0f, 0.0f));
  channel2.reset();
  checkFalse(channel2.solveToExit(-1000.0f, 50.0f, 250.0f),
             "negative-direction solve with positive exitVelocity fails cleanly");
}

// (e) The jerk sentinel (0.0f -> Ruckig's own +infinity) is preserved:
// same shape as an explicit huge-but-finite jerk.
void scenarioJerkSentinelPreserved() {
  beginScenario("jerk sentinel (0.0f) preserved: same shape as huge-finite jerk");

  Drive::MasterProfile sentinelChannel;
  sentinelChannel.configure(makeLimits(800.0f, 800.0f, 250.0f, 0.0f));
  sentinelChannel.reset();
  checkTrue(sentinelChannel.solveToExit(1000.0f, 0.0f, 250.0f), "sentinel solve succeeds");
  float sentinelDuration = sentinelChannel.duration();

  Drive::MasterProfile hugeFiniteChannel;
  hugeFiniteChannel.configure(makeLimits(800.0f, 800.0f, 250.0f, 1.0e7f));
  hugeFiniteChannel.reset();
  checkTrue(hugeFiniteChannel.solveToExit(1000.0f, 0.0f, 250.0f), "huge-finite jerk solve succeeds");
  float hugeFiniteDuration = hugeFiniteChannel.duration();

  checkNear(hugeFiniteDuration, sentinelDuration, 0.01 * sentinelDuration,
            "huge-finite jerk produces the same shape as the infinite-jerk sentinel");
}

// (f) Seeding contract: solveToExit()'s initial sampled state is exactly
// the seed set by seedCurrent() -- this class's whole public API has no
// measured-observation parameter to leak one through in the first place.
void scenarioSeedingContract() {
  beginScenario("seedCurrent() seed is exactly what solveToExit() starts from");

  Drive::MasterProfile channel;
  channel.configure(makeLimits(800.0f, 800.0f, 250.0f, 4000.0f));
  channel.seedCurrent(/*position=*/50.0f, /*velocity=*/80.0f, /*acceleration=*/10.0f);

  checkTrue(channel.solveToExit(1000.0f, 0.0f, 250.0f), "solve from a nonzero seed succeeds");
  Drive::MasterProfile::State atZero = channel.peek(0.0f);
  checkNear(atZero.position, 50.0, 1e-2, "initial position equals the seeded position");
  checkNear(atZero.velocity, 80.0, 1e-1, "initial velocity equals the seeded velocity");
  checkNear(atZero.acceleration, 10.0, 1e-1, "initial acceleration equals the seeded acceleration");
}

// (g) Negative-direction solveToExit with a negative exitVelocity is
// symmetric to scenario (b).
void scenarioNegativeDirectionNonzeroExit() {
  beginScenario("negative-direction solveToExit with negative exitVelocity (symmetric to b)");

  Drive::MasterProfile channel;
  channel.configure(makeLimits(800.0f, 800.0f, 250.0f, 0.0f));
  channel.reset();

  checkTrue(channel.solveToExit(-1000.0f, -120.0f, 250.0f),
            "solveToExit(-1000, exit=-120, max=250) succeeds");
  Trace trace = traceSpan(channel, 0.0f, channel.duration());
  checkTrue(trace.maxVel <= 0.5, "negative-direction velocity never goes positive (no reverse)");
  checkNear(trace.end.position, -1000.0, 1.0, "arrives AT the target position");
  checkNear(trace.end.velocity, -120.0, 0.5, "arrives AT the requested exit velocity (-120)");
}

}  // namespace

int main() {
  scenarioZeroExitMatchesSolveToRest();
  scenarioNonzeroExitWithinMaxVelocitySolves();
  scenarioExitVelocityExceedsMaxVelocityFailsCleanly();
  scenarioWrongSignExitVelocityFailsCleanly();
  scenarioJerkSentinelPreserved();
  scenarioSeedingContract();
  scenarioNegativeDirectionNonzeroExit();

  if (g_failureCount == 0) {
    std::printf("OK: all Drive::MasterProfile scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the Drive::MasterProfile scenarios\n",
              g_failureCount);
  return 1;
}
