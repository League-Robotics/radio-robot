// jerk_trajectory_harness.cpp -- off-hardware acceptance harness,
// restored 2026-07-17 (sprint 109 ticket 001, originally ticket 089-002,
// SUC-002/SUC-003/SUC-004/SUC-005): exercises Motion::JerkTrajectory
// (src/firm/motion/jerk_trajectory.{h,cpp}) in isolation -- hand-rolled
// assertions, no gtest/pytest-native C++ framework, run via
// test_jerk_trajectory.py.
//
// Scenarios map directly onto the original ticket's testing plan (a)-(h),
// plus (k) added by 109-001 for the new solveToState() entry point:
//  (a) position-control solve-to-rest: no-reverse, arrives at rest at the
//      target.
//  (b) velocity-control solve-to-a-velocity: holds the target velocity past
//      its own duration (Ruckig's own past-duration extrapolation).
//  (c) a stop-triggered-style second solve (velocity-control, target=0)
//      from a mid-cruise seeded state decelerates to rest with no reverse,
//      regardless of when it is triggered.
//  (d) j_max == 0 maps to the same shape (within tolerance) as an explicit
//      huge-but-finite max_jerk; a modest positive j_max measurably
//      lengthens the trajectory (S-curve vs. trapezoid).
//  (e) the per-call max_velocity argument is honored independently of the
//      global config ceiling.
//  (f)/(g) linear-channel direction-mirrored max_acceleration/
//      min_acceleration (Open Question 2), and the rotational channel's
//      symmetric (unmirrored) bound.
//  (h) retarget() continuity (no discontinuity at the reseed) and
//      reanchor()'s accepted discontinuity.
//  (i) retarget()/reanchor() do not validate direction -- a
//      backward-pointing target is defined behavior, not rejected.
//  (j) the class never reads a measured observation to seed a solve (this
//      is also pinned by a static source-text check in
//      test_jerk_trajectory.py).
//  (k) solveToState() (109-001): a position-control solve WITH a nonzero
//      target velocity arrives at the target position CARRYING that
//      velocity (not decelerating to rest), and solveToRest() is
//      equivalent to solveToState(pos, 0, vmax) (same end state, same
//      duration).
#include <cmath>
#include <cstdio>
#include <string>

#include "messages/planner.h"
#include "motion/jerk_trajectory.h"

namespace {

// --- Hand-rolled assertion plumbing (mirrors ruckig_smoke_harness.cpp /
// velocity_pid_harness.cpp) ---

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

// --- Config helpers ---

msg::PlannerConfig makeLinearConfig(float aMax, float aDecel, float vMax, float jMax) {
  msg::PlannerConfig config;
  config.a_max = aMax;
  config.a_decel = aDecel;
  config.v_body_max = vMax;
  config.j_max = jMax;
  return config;
}

msg::PlannerConfig makeRotationalConfig(float yawAccMax, float yawRateMax, float yawJerkMax) {
  msg::PlannerConfig config;
  config.yaw_acc_max = yawAccMax;
  config.yaw_rate_max = yawRateMax;
  config.yaw_jerk_max = yawJerkMax;
  return config;
}

// --- Trace helper: sample a channel's CURRENT trajectory across
// [tStart, tStart + span] and report min/max velocity/acceleration plus the
// end-of-span state. ---

struct Trace {
  double minVel = 1e9;
  double maxVel = -1e9;
  double minAcc = 1e9;
  double maxAcc = -1e9;
  Motion::JerkTrajectory::State end;
};

Trace traceSpan(Motion::JerkTrajectory& channel, float tStart, float span, int samples = 400) {
  Trace trace;
  for (int i = 0; i <= samples; ++i) {
    float t = tStart + span * static_cast<float>(i) / static_cast<float>(samples);
    Motion::JerkTrajectory::State state = channel.sample(t);
    if (state.velocity < trace.minVel) trace.minVel = state.velocity;
    if (state.velocity > trace.maxVel) trace.maxVel = state.velocity;
    if (state.acceleration < trace.minAcc) trace.minAcc = state.acceleration;
    if (state.acceleration > trace.maxAcc) trace.maxAcc = state.acceleration;
    trace.end = state;
  }
  return trace;
}

// --- Scenarios ---

// (a) Position-control solve-to-rest: never reverses, arrives at rest
// exactly at the target -- mirrors test_ruckig_smoke.py's own assertions
// against this wrapper class specifically.
void scenarioPositionControlNoReverseArrivesAtRest() {
  beginScenario("position-control solve-to-rest: no-reverse, arrives at rest at target");

  Motion::JerkTrajectory channel;
  channel.configure(makeLinearConfig(800.0f, 800.0f, 250.0f, 0.0f), /*isRotational=*/false);
  channel.reset();

  checkTrue(channel.solveToRest(1000.0f, 250.0f), "solveToRest() succeeds");
  float dur = channel.duration();
  checkTrue(dur > 0.0f, "trajectory duration > 0");

  Trace trace = traceSpan(channel, 0.0f, dur);
  checkTrue(trace.minVel >= -0.5, "velocity never goes negative (no reverse)");
  checkLe(trace.maxVel, 250.0 + 0.5, "velocity respects max_velocity (250)");
  checkNear(trace.end.velocity, 0.0, 0.5, "arrives at rest (final velocity ~0)");
  checkNear(trace.end.position, 1000.0, 1.0, "arrives AT the target position (1000)");
}

// (b) Velocity-control solve-to-a-velocity: reaches and HOLDS the target
// velocity past the ramp-up's own duration -- Ruckig's own past-duration
// extrapolation, no extra bookkeeping in the wrapper.
void scenarioVelocityControlHoldsPastDuration() {
  beginScenario("velocity-control solve-to-a-velocity: holds past its own duration");

  Motion::JerkTrajectory channel;
  channel.configure(makeLinearConfig(800.0f, 800.0f, 300.0f, 0.0f), /*isRotational=*/false);
  channel.reset();

  checkTrue(channel.solveToVelocity(300.0f, 300.0f), "solveToVelocity() succeeds");
  float dur = channel.duration();
  checkTrue(dur > 0.0f, "ramp-up duration > 0");

  Motion::JerkTrajectory::State atDur = channel.sample(dur);
  checkNear(atDur.velocity, 300.0, 0.5, "reaches the target velocity by its own duration");

  Motion::JerkTrajectory::State wellPast = channel.sample(dur + 5.0f);
  checkNear(wellPast.velocity, 300.0, 0.5, "holds the target velocity 5s past duration");
  checkNear(wellPast.acceleration, 0.0, 0.5, "acceleration is ~0 while holding");
}

// (c) A stop-triggered-style second solve (velocity-control, target=0)
// seeded from a mid-cruise state decelerates to rest with no reverse,
// regardless of when it is triggered relative to the first solve's own
// duration -- test both mid-ramp and post-cruise trigger points.
void scenarioStopTriggeredDecelFromMidCruiseNoReverse() {
  beginScenario("stop-triggered decel from mid-cruise: no reverse, any trigger time");

  // Trigger mid-ramp, before the cruise velocity is even reached.
  {
    Motion::JerkTrajectory channel;
    channel.configure(makeLinearConfig(800.0f, 800.0f, 300.0f, 0.0f), /*isRotational=*/false);
    channel.reset();
    channel.solveToVelocity(300.0f, 300.0f);
    float rampDur = channel.duration();
    channel.sample(rampDur * 0.3f);  // still ramping up; updates last-sample seed

    checkTrue(channel.solveToVelocity(0.0f, 300.0f), "decel-to-zero solve (mid-ramp trigger)");
    Trace trace = traceSpan(channel, 0.0f, channel.duration());
    checkTrue(trace.minVel >= -0.5, "no reverse decelerating from mid-ramp");
    checkNear(trace.end.velocity, 0.0, 0.5, "arrives at rest (mid-ramp trigger)");
  }

  // Trigger well after the cruise plateau (past the ramp-up's own
  // duration) -- exercises the past-duration hold-at-final-state sample.
  {
    Motion::JerkTrajectory channel;
    channel.configure(makeLinearConfig(800.0f, 800.0f, 300.0f, 0.0f), /*isRotational=*/false);
    channel.reset();
    channel.solveToVelocity(300.0f, 300.0f);
    float rampDur = channel.duration();
    channel.sample(rampDur + 2.0f);  // well past duration; cruising at 300

    checkTrue(channel.solveToVelocity(0.0f, 300.0f), "decel-to-zero solve (post-cruise trigger)");
    Trace trace = traceSpan(channel, 0.0f, channel.duration());
    checkTrue(trace.minVel >= -0.5, "no reverse decelerating from post-cruise cruise state");
    checkNear(trace.end.velocity, 0.0, 0.5, "arrives at rest (post-cruise trigger)");
  }
}

// (d) j_max == 0 sentinel maps to the SAME shape (within tolerance) as an
// explicit, huge-but-finite max_jerk; a modest positive j_max produces a
// measurably longer (S-curve) profile.
void scenarioJerkSentinelMapsToInfinity() {
  beginScenario("j_max == 0 sentinel ~= infinite jerk; j_max > 0 is measurably different");

  Motion::JerkTrajectory sentinelChannel;
  sentinelChannel.configure(makeLinearConfig(800.0f, 800.0f, 250.0f, 0.0f), false);
  sentinelChannel.reset();
  checkTrue(sentinelChannel.solveToRest(1000.0f, 250.0f), "sentinel (j_max=0) solve succeeds");
  float sentinelDuration = sentinelChannel.duration();

  Motion::JerkTrajectory hugeFiniteChannel;
  hugeFiniteChannel.configure(makeLinearConfig(800.0f, 800.0f, 250.0f, 1.0e7f), false);
  hugeFiniteChannel.reset();
  checkTrue(hugeFiniteChannel.solveToRest(1000.0f, 250.0f), "huge-finite j_max solve succeeds");
  float hugeFiniteDuration = hugeFiniteChannel.duration();

  checkNear(hugeFiniteDuration, sentinelDuration, 0.01 * sentinelDuration,
            "huge-finite j_max produces the same shape (duration) as the infinite-jerk sentinel");

  Motion::JerkTrajectory modestChannel;
  modestChannel.configure(makeLinearConfig(800.0f, 800.0f, 250.0f, 1500.0f), false);
  modestChannel.reset();
  checkTrue(modestChannel.solveToRest(1000.0f, 250.0f), "modest j_max solve succeeds");
  float modestDuration = modestChannel.duration();

  checkTrue(modestDuration > sentinelDuration * 1.05,
            "a modest positive j_max measurably lengthens the trajectory (S-curve vs trapezoid)");
}

// (e) The per-call max_velocity argument is respected independently of the
// global PlannerConfig ceiling: a low per-call ceiling produces a
// lower-peak trajectory than the global ceiling alone would.
void scenarioPerCallMaxVelocityIndependentOfGlobalCeiling() {
  beginScenario("per-call max_velocity honored independently of the global ceiling");

  // Global ceiling (v_body_max) is deliberately high (1000); this scenario
  // proves the LOW per-call value actually binds.
  Motion::JerkTrajectory lowCeilingChannel;
  lowCeilingChannel.configure(makeLinearConfig(800.0f, 800.0f, 1000.0f, 0.0f), false);
  lowCeilingChannel.reset();
  checkTrue(lowCeilingChannel.solveToRest(1000.0f, 100.0f), "low per-call ceiling solve succeeds");
  Trace lowTrace = traceSpan(lowCeilingChannel, 0.0f, lowCeilingChannel.duration());
  checkLe(lowTrace.maxVel, 100.5, "low per-call max_velocity (100) is respected");

  Motion::JerkTrajectory highCeilingChannel;
  highCeilingChannel.configure(makeLinearConfig(800.0f, 800.0f, 1000.0f, 0.0f), false);
  highCeilingChannel.reset();
  checkTrue(highCeilingChannel.solveToRest(1000.0f, 1000.0f),
            "high per-call ceiling solve succeeds");
  Trace highTrace = traceSpan(highCeilingChannel, 0.0f, highCeilingChannel.duration());

  checkTrue(highTrace.maxVel > lowTrace.maxVel + 50.0,
            "a low per-call ceiling produces a materially lower peak than the global ceiling alone");
}

// (f) Linear-channel max_acceleration/min_acceleration are correctly
// direction-mirrored for a negative-direction solve (Open Question 2).
void scenarioLinearChannelDirectionMirroring() {
  beginScenario("linear channel: direction-mirrored accel bounds (Open Question 2)");

  const float kAMax = 800.0f;    // [mm/s^2] distinct from kADecel, to make mirroring observable
  const float kADecel = 300.0f;  // [mm/s^2]

  Motion::JerkTrajectory positive;
  positive.configure(makeLinearConfig(kAMax, kADecel, 250.0f, 0.0f), false);
  positive.reset();
  checkTrue(positive.solveToRest(1000.0f, 250.0f), "positive-direction solve succeeds");
  Trace positiveTrace = traceSpan(positive, 0.0f, positive.duration());
  checkNear(positiveTrace.maxAcc, kAMax, 5.0,
            "positive direction: speed-up plateau reaches +a_max");
  checkNear(positiveTrace.minAcc, -kADecel, 5.0,
            "positive direction: slow-down plateau reaches -a_decel");

  Motion::JerkTrajectory negative;
  negative.configure(makeLinearConfig(kAMax, kADecel, 250.0f, 0.0f), false);
  negative.reset();
  checkTrue(negative.solveToRest(-1000.0f, 250.0f), "negative-direction solve succeeds");
  Trace negativeTrace = traceSpan(negative, 0.0f, negative.duration());
  checkTrue(negativeTrace.minVel <= 0.5, "negative-direction velocity never goes positive");
  checkNear(negativeTrace.minAcc, -kAMax, 5.0,
            "negative direction: speed-up plateau MIRRORS to -a_max");
  checkNear(negativeTrace.maxAcc, kADecel, 5.0,
            "negative direction: slow-down plateau MIRRORS to +a_decel");
}

// (g) Rotational channel needs no direction-mirroring: yaw_acc_max is the
// SAME bound both ways, confirmed for both a positive- and a
// negative-direction solve.
void scenarioRotationalChannelSymmetric() {
  beginScenario("rotational channel: symmetric accel bound, no mirroring applied");

  const float kYawAccMax = 500.0f;  // [rad/s^2]

  Motion::JerkTrajectory positive;
  positive.configure(makeRotationalConfig(kYawAccMax, 3.0f, 0.0f), /*isRotational=*/true);
  positive.reset();
  checkTrue(positive.solveToRest(2.0f, 3.0f), "positive-direction rotational solve succeeds");
  Trace positiveTrace = traceSpan(positive, 0.0f, positive.duration());
  checkNear(positiveTrace.maxAcc, kYawAccMax, 5.0, "positive direction reaches +yaw_acc_max");
  checkNear(positiveTrace.minAcc, -kYawAccMax, 5.0, "positive direction reaches -yaw_acc_max");

  Motion::JerkTrajectory negative;
  negative.configure(makeRotationalConfig(kYawAccMax, 3.0f, 0.0f), /*isRotational=*/true);
  negative.reset();
  checkTrue(negative.solveToRest(-2.0f, 3.0f), "negative-direction rotational solve succeeds");
  Trace negativeTrace = traceSpan(negative, 0.0f, negative.duration());
  checkNear(negativeTrace.maxAcc, kYawAccMax, 5.0,
            "negative direction ALSO reaches +yaw_acc_max (same magnitude, no mirroring)");
  checkNear(negativeTrace.minAcc, -kYawAccMax, 5.0,
            "negative direction ALSO reaches -yaw_acc_max (same magnitude, no mirroring)");
}

// (f, ticket item) retarget(): from a mid-trajectory seeded state, a new
// solve to a smaller or larger newRemaining produces a trajectory whose
// INITIAL sampled velocity/acceleration equal the seed state exactly (no
// discontinuity), and whose whole trace never reverses.
void scenarioRetargetContinuityAndNoReverse() {
  beginScenario("retarget(): continuity at the reseed, no reverse");

  // A FINITE jerk is deliberately used here (not the jMax=0/infinite-jerk
  // sentinel): with infinite jerk, Ruckig may legitimately jump
  // acceleration instantaneously at a zero-duration segment boundary, so
  // sampling exactly at t=0 does not reliably prove acceleration
  // continuity. With finite jerk, acceleration itself cannot jump, so
  // sample(0) truly pins the solved trajectory's own start state against
  // the pre-reseed seed.
  for (float newRemaining : {300.0f, 2000.0f}) {
    Motion::JerkTrajectory channel;
    channel.configure(makeLinearConfig(800.0f, 800.0f, 250.0f, 4000.0f), false);
    channel.reset();
    channel.solveToRest(1000.0f, 250.0f);

    float midElapsed = channel.duration() * 0.4f;
    Motion::JerkTrajectory::State seed = channel.sample(midElapsed);

    checkTrue(channel.retarget(newRemaining), "retarget() succeeds");
    Motion::JerkTrajectory::State atZero = channel.sample(0.0f);
    checkNear(atZero.position, 0.0, 1e-3, "retarget() re-baselines position to 0");
    checkNear(atZero.velocity, seed.velocity, 1e-2,
              "retarget()'s initial velocity equals the pre-reseed seed exactly (no discontinuity)");
    checkNear(atZero.acceleration, seed.acceleration, 1e-2,
              "retarget()'s initial acceleration equals the pre-reseed seed (no discontinuity)");

    Trace trace = traceSpan(channel, 0.0f, channel.duration());
    checkTrue(trace.minVel >= -0.5, "retarget()'s whole trace never reverses");
    checkNear(trace.end.position, newRemaining, 1.0, "retarget() arrives at the new remaining");
  }
}

// (g, ticket item) reanchor(): a solve seeded from an explicit
// (position, velocity) argument (deliberately DIFFERENT from the channel's
// own last remembered state) produces a well-formed, never-reversing
// trajectory to the given target -- documenting that a velocity
// discontinuity at the seam is EXPECTED and correct here, unlike retarget().
void scenarioReanchorAcceptsDiscontinuityWellFormed() {
  beginScenario("reanchor(): accepts a velocity discontinuity, still well-formed");

  // FINITE jerk, same reasoning as scenarioRetargetContinuityAndNoReverse():
  // proves acceleration is truly forced to 0 at the reseed rather than
  // relying on an infinite-jerk zero-duration segment that could
  // legitimately show a different value at exactly t=0.
  Motion::JerkTrajectory channel;
  channel.configure(makeLinearConfig(800.0f, 800.0f, 250.0f, 4000.0f), false);
  channel.reset();
  channel.solveToRest(1000.0f, 250.0f);  // target_ remembered = 1000

  // Sample early in the plan; the plan's OWN velocity here is modest
  // (well under 100 mm/s at a=800, t=0.1s).
  Motion::JerkTrajectory::State planState = channel.sample(0.1f);
  checkTrue(planState.velocity < 100.0f,
            "sanity: the plan's own velocity at t=0.1s is modest (test setup check)");

  // reanchor() with a position/velocity DELIBERATELY different from the
  // plan's own sampled state above -- proves the seam is a real
  // discontinuity, not silently smoothed to plan continuity.
  const float kReanchorPosition = 100.0f;  // [mm] -- far from the plan's own ~4mm at t=0.1s
  const float kReanchorVelocity = 150.0f;  // [mm/s] -- far from the plan's own ~80mm/s at t=0.1s
  checkTrue(channel.reanchor(kReanchorPosition, kReanchorVelocity), "reanchor() succeeds");

  Motion::JerkTrajectory::State atZero = channel.sample(0.0f);
  checkNear(atZero.position, kReanchorPosition, 1e-3,
            "reanchor()'s initial position is the CALLER-SUPPLIED value (discontinuity accepted)");
  checkNear(atZero.velocity, kReanchorVelocity, 1e-2,
            "reanchor()'s initial velocity is the CALLER-SUPPLIED value (discontinuity accepted)");
  checkNear(atZero.acceleration, 0.0, 1e-2, "reanchor() forces acceleration to 0 at the reseed");

  Trace trace = traceSpan(channel, 0.0f, channel.duration());
  checkTrue(trace.minVel >= -0.5, "reanchor()'s whole trace is still well-formed (no reverse)");
  checkNear(trace.end.position, 1000.0, 1.0,
            "reanchor() still arrives at the SAME remembered target (1000)");
  checkNear(trace.end.velocity, 0.0, 0.5, "reanchor() still arrives at rest");
}

// (h, ticket item) Documentation-pinning: retarget()/reanchor() do NOT
// validate that the new target is ahead of the seed in the commanded
// direction -- calling either with a backward-pointing target is defined
// behavior (it solves backward), which is the CALLER's (Planner's) job to
// never do, per Decision 10. This is NOT a "no-reverse" test -- it is the
// opposite: proving the class does not silently guard against reversal.
void scenarioBackwardTargetIsDefinedButUnguarded() {
  beginScenario("retarget()/reanchor(): a backward-pointing target is defined, unguarded");

  Motion::JerkTrajectory channel;
  channel.configure(makeLinearConfig(800.0f, 800.0f, 250.0f, 0.0f), false);
  channel.reset();
  channel.solveToRest(1000.0f, 250.0f);
  channel.sample(channel.duration() * 0.3f);  // moving forward, nonzero velocity seed

  // A backward-pointing retarget() (newRemaining behind the rebaselined
  // origin, while still moving forward) is accepted, not rejected.
  checkTrue(channel.retarget(-500.0f),
            "retarget() with a backward-pointing target still solves (no guard here)");
  checkTrue(channel.duration() > 0.0f, "the resulting (backward) trajectory is well-formed");
}

// (k, 109-001) solveToState(): a nonzero target velocity is CARRIED at the
// target position rather than decelerated to rest; solveToRest() is
// equivalent to solveToState(pos, 0, vmax) (same end state, same
// duration).
void scenarioSolveToStateCarriesTargetVelocity() {
  beginScenario("solveToState(): arrives at the target CARRYING a nonzero target velocity");

  Motion::JerkTrajectory channel;
  channel.configure(makeLinearConfig(800.0f, 800.0f, 300.0f, 0.0f), /*isRotational=*/false);
  channel.reset();

  checkTrue(channel.solveToState(1000.0f, 150.0f, 300.0f), "solveToState() succeeds");
  float dur = channel.duration();
  checkTrue(dur > 0.0f, "trajectory duration > 0");

  Trace trace = traceSpan(channel, 0.0f, dur);
  checkTrue(trace.minVel >= -0.5, "velocity never goes negative (no reverse)");
  checkNear(trace.end.position, 1000.0, 1.0, "arrives AT the target position (1000)");
  checkNear(trace.end.velocity, 150.0, 0.5,
            "arrives CARRYING the target velocity (150), not decelerated to rest");

  // solveToRest(pos, vmax) === solveToState(pos, 0, vmax): same end state,
  // same duration, for the same inputs otherwise.
  Motion::JerkTrajectory restChannel;
  restChannel.configure(makeLinearConfig(800.0f, 800.0f, 300.0f, 0.0f), false);
  restChannel.reset();
  checkTrue(restChannel.solveToRest(1000.0f, 300.0f), "solveToRest() succeeds");

  Motion::JerkTrajectory stateChannel;
  stateChannel.configure(makeLinearConfig(800.0f, 800.0f, 300.0f, 0.0f), false);
  stateChannel.reset();
  checkTrue(stateChannel.solveToState(1000.0f, 0.0f, 300.0f),
            "solveToState(pos, 0, vmax) succeeds");

  checkNear(stateChannel.duration(), restChannel.duration(), 1e-3,
            "solveToState(pos, 0, vmax) has the SAME duration as solveToRest(pos, vmax)");
  Motion::JerkTrajectory::State restEnd = restChannel.sample(restChannel.duration());
  Motion::JerkTrajectory::State stateEnd = stateChannel.sample(stateChannel.duration());
  checkNear(stateEnd.position, restEnd.position, 1e-2,
            "solveToState(pos, 0, vmax) reaches the SAME end position as solveToRest(pos, vmax)");
  checkNear(stateEnd.velocity, restEnd.velocity, 1e-2,
            "solveToState(pos, 0, vmax) reaches the SAME end velocity (0) as solveToRest(pos, vmax)");
}

}  // namespace

int main() {
  scenarioPositionControlNoReverseArrivesAtRest();
  scenarioVelocityControlHoldsPastDuration();
  scenarioStopTriggeredDecelFromMidCruiseNoReverse();
  scenarioJerkSentinelMapsToInfinity();
  scenarioPerCallMaxVelocityIndependentOfGlobalCeiling();
  scenarioLinearChannelDirectionMirroring();
  scenarioRotationalChannelSymmetric();
  scenarioRetargetContinuityAndNoReverse();
  scenarioReanchorAcceptsDiscontinuityWellFormed();
  scenarioBackwardTargetIsDefinedButUnguarded();
  scenarioSolveToStateCarriesTargetVelocity();

  if (g_failureCount == 0) {
    std::printf("OK: all Motion::JerkTrajectory scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the Motion::JerkTrajectory scenarios\n",
              g_failureCount);
  return 1;
}
