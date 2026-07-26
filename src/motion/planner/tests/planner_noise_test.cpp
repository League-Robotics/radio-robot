// planner_noise_test.cpp -- the imperfect-plant tier (issue §7 items 1, 2
// and 4). The zero-error scenarios in planner_scenarios_test.cpp prove the
// ACCOUNTING is exact; these prove the accounting degrades gracefully once
// the three real-world defects show up:
//
//   * velocity samples are noisy (the encoder's derivative is dirty),
//   * samples are STALE -- a fresh one lands only every other 50 ms cycle.
//     (Degraded-mode emulation: measured 2026-07-26, the register is live
//     at <=16 ms and fresh-every-cycle is the expected regime -- see
//     docs/design/encoder-refresh-characterization.md; this tier proves
//     the planner survives repeats anyway.)
//   * the command takes effect one cycle late (actuationDelay = 50 ms).
//
// Plus the two features that exist for exactly this regime: settle-confirm
// completion (M1) and heading hold on Distance Moves (M3).
#include <algorithm>
#include <cmath>
#include <cstdio>

#include "planner.h"
#include "tests/test_support.h"

using Motion::Move;
using Motion::Planner;
using Motion::PlannerLimits;
using Motion::TickResult;
using TestPlanner::benchLimits;
using TestPlanner::cycle;
using TestPlanner::NoisyPlant;
using TestPlanner::PerfectPlant;

namespace {

constexpr float kPeriod = 50.0f;  // [ms]
constexpr float kCruise = 150.0f;  // [mm/s] the linear cruise every scenario uses
constexpr float kCruiseOmega = 2.0f;  // [rad/s] the angular cruise

Move distanceMove(uint32_t id, float threshold, float v_x) {
  Move m;
  m.id = id;
  m.kind = Move::Kind::Distance;
  m.threshold = threshold;
  m.v_x = v_x;
  m.timeout = 60000.0f;
  return m;
}

Move angleMove(uint32_t id, float threshold, float omega) {
  Move m;
  m.id = id;
  m.kind = Move::Kind::Angle;
  m.threshold = threshold;
  m.omega = omega;
  m.timeout = 60000.0f;
  return m;
}

// The dirty-plant limit set: filtered velocity (the raw sample is noisy)
// and one control period of actuation delay to plan against.
PlannerLimits noisyLimits() {
  PlannerLimits limits = benchLimits();
  limits.velocityFilterWeight = 0.3f;
  // This tier's plant carries a +-40 zig-zag + +-15 white velocity noise;
  // the filtered body-velocity ripple floor is ~8 mm/s, so the rest floor
  // must sit above it (per-robot property -- see PlannerLimits).
  // Sized to THIS profile's physics: the filtered velocity's decay tail
  // (EMA 0.3 over 100 ms fresh samples) reads ~15-20 mm/s for several
  // hundred ms after a physical stop, and the plant's tiny tau (~31 ms)
  // means even a generous floor coasts < 1 mm / < 0.01 rad -- inside the
  // arrival epsilons, which remain the arbiter of arrival truth.
  limits.settleRestVelocity = 25.0f;
  limits.settleRestOmega = 0.15f;
  limits.actuationDelay = kPeriod;  // [ms] command lands one cycle late
  return limits;
}

NoisyPlant dirtyPlant() {
  NoisyPlant plant;
  plant.noiseAmplitude = 40.0f;   // [mm/s] zig-zag, ~27% of cruise
  plant.noiseWhite = 15.0f;       // [mm/s]
  plant.sampleDivisor = 2;        // fresh sample every 100 ms vs the 50 ms loop
  plant.delayedActuation = true;
  // A velocity-PID'd wheel closing 80% of a velocity step per 50 ms
  // interval (time constant ~31 ms). See testTrackingLagSensitivity() for
  // what happens as this degrades.
  plant.trackingLag = 0.8f;
  plant.positionQuantum = 0.5f;  // [mm] encoder resolution
  return plant;
}

// Drive one Move to completion, checking the per-tick invariants that must
// hold no matter how dirty the plant is: the command never exceeds vMax,
// never steps by more than the accel/decel ceiling allows, and -- once the
// Move has completed -- only ever shrinks toward zero (no hunting at the
// goal).
struct Outcome {
  bool completed = false;
  bool settled = false;
  bool timedOut = false;
  int completionTick = -1;
  int ticks = 0;
};

// `angular` picks which acceleration ceiling bounds the per-tick command
// step: an Angle Move's wheels move at +-omega*b/2, so their step ceiling
// is the ANGULAR one scaled by the half-track, not the linear one.
template <typename Plant>
Outcome drive(Planner& planner, Plant& plant, const PlannerLimits& limits,
              int maxTicks, int drainTicks, bool angular = false) {
  Types::RobotState state;
  uint32_t now = 0;
  Outcome outcome;
  const float dt = limits.controlPeriod * 0.001f;  // [s]
  const float stepCeiling =
      (angular ? std::max(limits.alphaMax, limits.alphaDecel) * 0.5f *
                     limits.trackWidth
               : std::max(limits.aMax, limits.aDecel)) *
          dt +
      1e-3f;  // [mm/s]
  float previousLeft = 0.0f;   // [mm/s]
  float previousRight = 0.0f;  // [mm/s]
  float afterGoalPeak = 0.0f;  // [mm/s] monotone envelope once completed

  for (int i = 0; i < maxTicks; ++i) {
    const TickResult r = cycle(planner, state, plant, now, limits.controlPeriod);
    ++outcome.ticks;
    const float left = state.wheelLeft.cmdVelocity;
    const float right = state.wheelRight.cmdVelocity;

    CHECK(std::fabs(left) <= limits.vMax + 1e-3f);
    CHECK(std::fabs(right) <= limits.vMax + 1e-3f);
    CHECK(std::fabs(left - previousLeft) <= stepCeiling);
    CHECK(std::fabs(right - previousRight) <= stepCeiling);
    previousLeft = left;
    previousRight = right;

    if (r.completed) {
      CHECK(!outcome.completed);  // at most one completion per Move
      outcome.completed = true;
      outcome.settled = r.settled;
      outcome.timedOut = r.timedOut;
      outcome.completionTick = i;
      afterGoalPeak = std::max(std::fabs(left), std::fabs(right));
    } else if (outcome.completed) {
      // No oscillation at the goal: the drain envelope only shrinks.
      const float magnitude = std::max(std::fabs(left), std::fabs(right));
      CHECK(magnitude <= afterGoalPeak + 1e-3f);
      afterGoalPeak = magnitude;
      if (i >= outcome.completionTick + drainTicks) break;
    }
  }
  CHECK(outcome.completed);
  return outcome;
}

// ---- item 1: noise / staleness / actuation lag ----

void testDistanceUnderNoiseAndLag() {
  const PlannerLimits limits = noisyLimits();
  Planner planner(limits);
  NoisyPlant plant = dirtyPlant();
  CHECK(planner.move(distanceMove(1, 500.0f, kCruise), false));
  const Outcome outcome = drive(planner, plant, limits, 400, 12);
  CHECK(!outcome.timedOut);

  // Bounded, not exact: one cruise interval's travel is the gate.
  const float traveled = 0.5f * (plant.positionLeft + plant.positionRight);
  const float error = std::fabs(traveled - 500.0f);  // [mm]
  const float gate = kCruise * kPeriod * 0.001f;     // [mm] 7.5
  std::printf("  distance 500 mm, noisy+stale+lagged: error %.3f mm "
              "(gate %.1f mm)\n", error, gate);
  CHECK(error <= gate);
  // The wheels really did stop.
  CHECK_NEAR(planner.commandedLeft(), 0.0f, 1e-6);
  CHECK_NEAR(planner.commandedRight(), 0.0f, 1e-6);
}

void testDistanceBackwardUnderNoiseAndLag() {
  const PlannerLimits limits = noisyLimits();
  Planner planner(limits);
  NoisyPlant plant = dirtyPlant();
  CHECK(planner.move(distanceMove(2, 300.0f, -kCruise), false));
  const Outcome outcome = drive(planner, plant, limits, 400, 12);
  CHECK(!outcome.timedOut);
  const float traveled = 0.5f * (plant.positionLeft + plant.positionRight);
  const float error = std::fabs(traveled + 300.0f);  // [mm]
  std::printf("  distance -300 mm, noisy+stale+lagged: error %.3f mm\n",
              error);
  CHECK(error <= kCruise * kPeriod * 0.001f);
}

void testTurnUnderNoiseAndLag() {
  const PlannerLimits limits = noisyLimits();
  Planner planner(limits);
  NoisyPlant plant = dirtyPlant();
  const float quarterTurn = static_cast<float>(M_PI) * 0.5f;  // [rad]
  CHECK(planner.move(angleMove(3, quarterTurn, kCruiseOmega), false));
  const Outcome outcome = drive(planner, plant, limits, 400, 12, true);
  CHECK(!outcome.timedOut);

  const float heading =
      (plant.positionRight - plant.positionLeft) / limits.trackWidth;  // [rad]
  const float error = std::fabs(heading - quarterTurn);
  const float gate = kCruiseOmega * kPeriod * 0.001f;  // [rad] 0.1
  std::printf("  turn 90 deg, noisy+stale+lagged: error %.5f rad "
              "(gate %.3f rad)\n", error, gate);
  CHECK(error <= gate);
}

void testChainUnderNoiseAndLag() {
  // The carry accounting is what a dirty plant is most likely to break:
  // the boundary is crossed at speed, so the residual debited to the
  // successor is measured from noisy state.
  const PlannerLimits limits = noisyLimits();
  Planner planner(limits);
  NoisyPlant plant = dirtyPlant();
  Types::RobotState state;
  uint32_t now = 0;
  CHECK(planner.move(distanceMove(10, 500.0f, kCruise), false));
  CHECK(planner.move(distanceMove(11, 300.0f, kCruise), false));
  int completions = 0;
  for (int i = 0; i < 800 && completions < 2; ++i) {
    if (cycle(planner, state, plant, now, kPeriod).completed) ++completions;
  }
  CHECK(completions == 2);
  for (int i = 0; i < 12; ++i) cycle(planner, state, plant, now, kPeriod);
  const float traveled = 0.5f * (plant.positionLeft + plant.positionRight);
  const float error = std::fabs(traveled - 800.0f);  // [mm]
  std::printf("  chain 500+300 mm, noisy+stale+lagged: total error %.3f mm\n",
              error);
  // The chain as a whole is still held to ONE interval, not two -- the
  // cumulative baseline means the first leg's residual is not re-spent.
  CHECK(error <= kCruise * kPeriod * 0.001f);
}

void testStandingRobotDoesNotDrift() {
  // Regression: pathLength accumulates |ds|, which RECTIFIES zero-mean
  // jitter into one-way drift. Feeding it the ZOH-extrapolated positions
  // made a standing robot's odometer climb at roughly the velocity noise
  // times the period, every tick, forever -- which in turn made the
  // settle gate impossible to hold and quietly ate into every subsequent
  // Move's distance accounting. The pose is integrated from the MEASURED
  // anchors instead, so a robot that is not moving reads as not moving.
  const PlannerLimits limits = noisyLimits();
  Planner planner(limits);
  NoisyPlant plant = dirtyPlant();
  Types::RobotState state;
  uint32_t now = 0;

  // Park: no Move, wheels commanded to zero, but the encoder keeps
  // publishing its (noisy, stale) opinion of a stationary wheel.
  for (int i = 0; i < 200; ++i) cycle(planner, state, plant, now, kPeriod);
  const float parkedX = state.pose.x;      // [mm]
  const float parkedY = state.pose.y;      // [mm]
  for (int i = 0; i < 400; ++i) cycle(planner, state, plant, now, kPeriod);
  std::printf("  standing robot, 400 idle ticks: pose drift %.6f mm\n",
              std::fabs(state.pose.x - parkedX));
  CHECK_NEAR(state.pose.x, parkedX, 1e-4);
  CHECK_NEAR(state.pose.y, parkedY, 1e-4);

  // And a Move issued afterwards is unaffected by the idle time.
  CHECK(planner.move(distanceMove(5, 200.0f, kCruise), false));
  bool completed = false;
  const float before = 0.5f * (plant.positionLeft + plant.positionRight);
  for (int i = 0; i < 400 && !completed; ++i) {
    completed = cycle(planner, state, plant, now, kPeriod).completed;
  }
  CHECK(completed);
  for (int i = 0; i < 12; ++i) cycle(planner, state, plant, now, kPeriod);
  const float traveled =
      0.5f * (plant.positionLeft + plant.positionRight) - before;
  CHECK(std::fabs(traveled - 200.0f) <= kCruise * kPeriod * 0.001f);
}

void testTrackingLagSensitivity() {
  // WHERE THE REMAINING ERROR LIVES. Of the plant's defects, only one is
  // outside the planner's model: the wheel not reaching the velocity it
  // was told to. Everything else -- noise, staleness, staging latency,
  // encoder quantisation -- the planner reconstructs or anchors away, and
  // at trackingLag = 1 the dirty plant is still landing sub-millimetre.
  //
  // What is left is a pure OVERSHOOT, and it is the coast after the last
  // command: the body is still doing more than the profile's terminal step
  // asked for, and a velocity sink cannot take that back (commanding a
  // reverse velocity to shave off millimetres is not something this
  // planner does, by design). It grows monotonically as tracking degrades
  // and reaches a full cruise interval at trackingLag = 0.5. Closing it
  // needs the plant model that arrives with the duty-plane back end
  // (issue §7.6: the velocity PID moves into the planner's output stage);
  // until then it is characterised here, not tuned away.
  const float lags[] = {1.0f, 0.9f, 0.8f, 0.7f, 0.6f};
  float previousError = -1.0f;  // [mm]
  for (float lag : lags) {
    const PlannerLimits limits = noisyLimits();
    Planner planner(limits);
    NoisyPlant plant = dirtyPlant();
    plant.trackingLag = lag;
    CHECK(planner.move(distanceMove(4, 500.0f, kCruise), false));
    drive(planner, plant, limits, 400, 12);
    const float traveled = 0.5f * (plant.positionLeft + plant.positionRight);
    const float error = traveled - 500.0f;  // [mm] signed
    std::printf("  tracking lag %.1f: %+.3f mm\n", lag, error);
    CHECK(error >= 0.0f);  // always an overshoot, never a stop-short
    CHECK(error <= kCruise * kPeriod * 0.001f);
    CHECK(error >= previousError);  // monotone in the tracking defect
    previousError = error;
  }
}

// ---- item 2: settle-confirm completion (M1) ----

void testSettleCoincidesInZeroErrorSim() {
  // The gate: with a perfect plant, requiring settle must cost nothing --
  // settle-complete and profile-complete land on the SAME tick.
  int completionTick[2] = {-1, -1};
  for (int variant = 0; variant < 2; ++variant) {
    PlannerLimits limits = benchLimits();
    limits.requireSettle = variant == 1;
    limits.settleWindow = 1000.0f;  // [ms]
    Planner planner(limits);
    PerfectPlant plant;
    CHECK(planner.move(distanceMove(20, 500.0f, kCruise), false));
    const Outcome outcome = drive(planner, plant, limits, 400, 6);
    completionTick[variant] = outcome.completionTick;
    // settled is reported truthfully: variant 1 defers completion until
    // it holds; variant 0 completes at profile-complete, one sample
    // BEFORE the v == 0 reading can exist, so settled is honestly false
    // there (measured-velocity-only rest gate).
    if (variant == 1) CHECK(outcome.settled);
    CHECK(!outcome.timedOut);
    CHECK_NEAR(plant.positionLeft, 500.0f, 1e-3);
  }
  std::printf("  settle-confirm, zero-error plant: profile-complete tick %d, "
              "settle-complete tick %d\n", completionTick[0],
              completionTick[1]);
  // Within ONE tick, not zero: the settle gate is measured-velocity-only
  // (the wider "commanded is zero, so at rest next interval" gate admitted
  // real coast on a lagging plant -- see planner.cpp's rest-floor
  // comment), and the sample PROVING v == 0 arrives one cycle after the
  // landing command even on a perfect plant. That one-tick defer is the
  // discrete-sensing bound, not a regression.
  CHECK(completionTick[1] - completionTick[0] <= 1);
}

void testSettleCoincidesOnTurnInZeroErrorSim() {
  PlannerLimits limits = benchLimits();
  limits.requireSettle = true;
  limits.settleWindow = 1000.0f;  // [ms]
  Planner planner(limits);
  PerfectPlant plant;
  const float quarterTurn = static_cast<float>(M_PI) * 0.5f;  // [rad]
  CHECK(planner.move(angleMove(21, quarterTurn, kCruiseOmega), false));
  const Outcome outcome = drive(planner, plant, limits, 400, 6, true);
  CHECK(outcome.settled);
  const float heading =
      (plant.positionRight - plant.positionLeft) / limits.trackWidth;
  CHECK_NEAR(heading, quarterTurn, 1e-5);
}

void testSettleReportsArrivalTruthfullyUnderNoiseAndLag() {
  // The contract settle-confirm actually offers on a dirty plant is not
  // "the Move becomes exact" -- nothing can make a velocity sink take back
  // an overshoot. It is that `settled` MEANS something: settled = true is
  // a promise the body is inside the arrival epsilon; settled = false says
  // the stop condition was met but the target was not hit, which is a
  // different fact from `timedOut` and worth its own bit.
  struct Case {
    float trackingLag;
    const char* label;
    bool expectSettled;
  };
  const Case cases[] = {
      // Overshoot ~0.7 mm, inside the 1 mm epsilon: confirmed arrival.
      {0.95f, "well-tracked", true},
      // The settle-creep now actively walks even the sloppy plant inside
      // the epsilon (the pre-creep behavior was an honest refusal at
      // ~2 mm); truthfulness is enforced by the tolerance check below.
      {0.80f, "sloppily-tracked", true},
  };
  for (const Case& scenario : cases) {
    PlannerLimits limits = noisyLimits();
    limits.requireSettle = true;
    limits.settleWindow = 500.0f;  // [ms]
    Planner planner(limits);
    NoisyPlant plant = dirtyPlant();
    plant.trackingLag = scenario.trackingLag;
    CHECK(planner.move(distanceMove(22, 500.0f, kCruise), false));
    const Outcome outcome = drive(planner, plant, limits, 400, 12);
    CHECK(!outcome.timedOut);  // never a timeout -- the stop condition WAS met
    const float traveled = 0.5f * (plant.positionLeft + plant.positionRight);
    const float error = std::fabs(traveled - 500.0f);  // [mm]
    std::printf("  distance 500 mm, settle-confirm, %s plant: "
                "error %.3f mm, settled=%d\n", scenario.label, error,
                outcome.settled ? 1 : 0);
    CHECK(outcome.settled == scenario.expectSettled);
    // The promise: a settled report is never off by more than the arrival
    // epsilon plus what the sensor CANNOT resolve (one position quantum
    // per wheel) -- the planner's own measured residual is inside the
    // epsilon; truth can differ from it by the quantization floor.
    if (outcome.settled) CHECK(error <= 1.0f + 0.5f);
  }
}

void testSettleReportsTurnArrivalTruthfully() {
  // Same contract on the angular axis. Note the epsilon there (0.005 rad)
  // is FINER than one encoder tick reads on this track width -- a 0.5 mm
  // quantum over a 100 mm track is 0.01 rad of heading -- so a coarse
  // encoder alone can keep an otherwise perfect turn from confirming. That
  // is the honest answer, and it is why item §7.3's bench measurement of
  // the real encoder matters before this gate is trusted on hardware.
  const float quarterTurn = static_cast<float>(M_PI) * 0.5f;  // [rad]
  struct Case {
    float positionQuantum;  // [mm]
    float trackingLag;
    bool expectSettled;
  };
  const Case cases[] = {
      {0.05f, 1.0f, true},  // fine encoder, well-tracked wheel
      // The standard dirty plant used to be unconfirmable (a 0.5 mm
      // quantum reads 0.01 rad of heading -- coarser than the epsilon).
      // The M1 settle-creep changed that: it actively walks the measured
      // residual inside the epsilon from either side (verified 0.0047 rad
      // final error here), so a truthful settled=true is now the expected
      // outcome. The "settled is never a lie" assertion below remains the
      // contract's teeth.
      {0.50f, 0.8f, true},
  };
  for (const Case& scenario : cases) {
    PlannerLimits limits = noisyLimits();
    limits.requireSettle = true;
    limits.settleWindow = 1000.0f;  // [ms]
    Planner planner(limits);
    NoisyPlant plant = dirtyPlant();
    plant.positionQuantum = scenario.positionQuantum;
    plant.trackingLag = scenario.trackingLag;
    CHECK(planner.move(angleMove(24, quarterTurn, kCruiseOmega), false));
    const Outcome outcome = drive(planner, plant, limits, 400, 12, true);
    CHECK(!outcome.timedOut);
    const float heading =
        (plant.positionRight - plant.positionLeft) / limits.trackWidth;
    const float error = std::fabs(heading - quarterTurn);  // [rad]
    std::printf("  turn 90 deg, settle-confirm, quantum %.2f mm lag %.2f: "
                "error %.5f rad, settled=%d\n", scenario.positionQuantum,
                scenario.trackingLag, error, outcome.settled ? 1 : 0);
    CHECK(outcome.settled == scenario.expectSettled);
    if (outcome.settled) CHECK(error <= 0.005f);
  }
}

void testSettleWindowExpiryReportsUnsettled() {
  // A body that never comes to rest (unmodelled creep): the stop condition
  // is met, the physical confirmation never is. Past the window the Move
  // completes anyway, reporting settled = false -- and NOT timedOut, which
  // would mean something else entirely (the safety backstop fired).
  PlannerLimits limits = noisyLimits();
  limits.requireSettle = true;
  limits.settleWindow = 300.0f;  // [ms]
  Planner planner(limits);
  NoisyPlant plant = dirtyPlant();
  plant.creepVelocity = 60.0f;  // [mm/s] never stops
  Planner reference(noisyLimits());
  NoisyPlant referencePlant = dirtyPlant();
  referencePlant.creepVelocity = 60.0f;

  CHECK(planner.move(distanceMove(23, 500.0f, kCruise), false));
  CHECK(reference.move(distanceMove(23, 500.0f, kCruise), false));
  const Outcome settled = drive(planner, plant, limits, 400, 2);
  const Outcome plain = drive(reference, referencePlant, noisyLimits(), 400, 2);

  CHECK(settled.completed);
  CHECK(!settled.settled);
  CHECK(!settled.timedOut);
  // Held back for the whole window and no longer (one tick of slack for
  // the tick the window is first observed to have expired).
  const int windowTicks =
      static_cast<int>(limits.settleWindow / limits.controlPeriod);
  const int deferred = settled.completionTick - plain.completionTick;
  std::printf("  settle window expiry: deferred %d ticks (window %d ticks)\n",
              deferred, windowTicks);
  CHECK(deferred >= windowTicks);
  CHECK(deferred <= windowTicks + 1);
}

void testSettleDoesNotBreakTheCarryChain() {
  // A same-axis chain hands off AT SPEED; there is nothing to settle at the
  // boundary, so settle-confirm must not stall it (and the total stays
  // exact on a perfect plant).
  PlannerLimits limits = benchLimits();
  limits.requireSettle = true;
  limits.settleWindow = 1000.0f;  // [ms]
  Planner planner(limits);
  PerfectPlant plant;
  Types::RobotState state;
  uint32_t now = 0;
  CHECK(planner.move(distanceMove(30, 500.0f, kCruise), false));
  CHECK(planner.move(distanceMove(31, 300.0f, kCruise), false));
  int completions = 0;
  float minSpeedAfterRamp = 1e9f;  // [mm/s]
  bool rampDone = false;
  for (int i = 0; i < 800 && completions < 2; ++i) {
    const TickResult r = cycle(planner, state, plant, now, kPeriod);
    const float speed = std::fabs(state.wheelLeft.cmdVelocity);
    if (speed >= kCruise - 1e-3f) rampDone = true;
    if (rampDone && completions == 0) {
      minSpeedAfterRamp = std::min(minSpeedAfterRamp, speed);
    }
    if (r.completed) ++completions;
  }
  CHECK(completions == 2);
  for (int i = 0; i < 10; ++i) cycle(planner, state, plant, now, kPeriod);
  CHECK_NEAR(plant.positionLeft, 800.0f, 1e-3);
  // Still crossed at speed: the boundary was never held for a settle.
  CHECK(minSpeedAfterRamp >= kCruise - limits.aDecel * 0.05f - 1e-3f);
}

// ---- item 4: heading hold on Distance Moves (M3) ----

void testHeadingHoldRecoversDisturbance() {
  PlannerLimits limits = benchLimits();
  limits.headingHoldGain = 4.0f;  // [1/s]
  Planner planner(limits);
  PerfectPlant plant;
  Types::RobotState state;
  uint32_t now = 0;
  CHECK(planner.move(distanceMove(40, 500.0f, kCruise), false));

  bool completed = false;
  float worstHeading = 0.0f;  // [rad]
  for (int i = 0; i < 400 && !completed; ++i) {
    completed = cycle(planner, state, plant, now, kPeriod).completed;
    if (i == 10) plant.disturbHeading(0.20f, limits.trackWidth);  // [rad] kick
    if (i > 10) {
      worstHeading = std::max(
          worstHeading,
          std::fabs((plant.positionRight - plant.positionLeft) /
                    limits.trackWidth));
    }
  }
  CHECK(completed);
  for (int i = 0; i < 10; ++i) cycle(planner, state, plant, now, kPeriod);

  const float heading =
      (plant.positionRight - plant.positionLeft) / limits.trackWidth;  // [rad]
  const float path = 0.5f * (plant.positionLeft + plant.positionRight);  // [mm]
  std::printf("  heading hold: 0.200 rad kick -> residual %.5f rad, "
              "distance %.6f mm\n", std::fabs(heading), path);
  // Driven back to the activation baseline...
  CHECK(std::fabs(heading) <= 0.01f);
  CHECK(worstHeading <= 0.21f);  // never made it worse
  // ...and the distance is STILL EXACT: the correction is differential, so
  // ds -- the mean of the pair -- is untouched by it.
  CHECK_NEAR(path, 500.0f, 1e-3);
}

void testHeadingHoldOffLeavesDisturbanceStanding() {
  // The control: with the gain at its fail-closed default the same kick is
  // simply carried to the end. Proves the recovery above is the P term and
  // not something else in the pipeline.
  const PlannerLimits limits = benchLimits();
  CHECK(limits.headingHoldGain == 0.0f);
  Planner planner(limits);
  PerfectPlant plant;
  Types::RobotState state;
  uint32_t now = 0;
  CHECK(planner.move(distanceMove(41, 500.0f, kCruise), false));
  bool completed = false;
  for (int i = 0; i < 400 && !completed; ++i) {
    completed = cycle(planner, state, plant, now, kPeriod).completed;
    if (i == 10) plant.disturbHeading(0.20f, limits.trackWidth);  // [rad]
  }
  CHECK(completed);
  const float heading =
      (plant.positionRight - plant.positionLeft) / limits.trackWidth;
  CHECK_NEAR(heading, 0.20f, 1e-4);
  CHECK_NEAR(0.5f * (plant.positionLeft + plant.positionRight), 500.0f, 1e-3);
}

void testHeadingHoldClampedToVelocityCeiling() {
  // A huge gain against a huge error must not push the outer wheel past
  // vMax -- and must still leave the profiled mean exactly where it was.
  PlannerLimits limits = benchLimits();
  limits.headingHoldGain = 200.0f;  // [1/s] absurd on purpose
  Planner planner(limits);
  PerfectPlant plant;
  Types::RobotState state;
  uint32_t now = 0;
  CHECK(planner.move(distanceMove(42, 2000.0f, 500.0f), false));
  for (int i = 0; i < 40; ++i) {
    cycle(planner, state, plant, now, kPeriod);
    if (i == 10) plant.disturbHeading(1.0f, limits.trackWidth);  // [rad]
    CHECK(std::fabs(state.wheelLeft.cmdVelocity) <= limits.vMax + 1e-3f);
    CHECK(std::fabs(state.wheelRight.cmdVelocity) <= limits.vMax + 1e-3f);
  }
}

void testHeadingHoldUnderNoiseAndLag() {
  const PlannerLimits base = noisyLimits();
  PlannerLimits limits = base;
  limits.headingHoldGain = 3.0f;  // [1/s]
  Planner planner(limits);
  NoisyPlant plant = dirtyPlant();
  Types::RobotState state;
  uint32_t now = 0;
  CHECK(planner.move(distanceMove(43, 500.0f, kCruise), false));
  bool completed = false;
  for (int i = 0; i < 400 && !completed; ++i) {
    completed = cycle(planner, state, plant, now, kPeriod).completed;
    if (i == 10) plant.disturbHeading(0.15f, limits.trackWidth);  // [rad]
  }
  CHECK(completed);
  for (int i = 0; i < 12; ++i) cycle(planner, state, plant, now, kPeriod);
  const float heading =
      (plant.positionRight - plant.positionLeft) / limits.trackWidth;  // [rad]
  const float path = 0.5f * (plant.positionLeft + plant.positionRight);  // [mm]
  std::printf("  heading hold on a dirty plant: residual %.5f rad, "
              "distance error %.3f mm\n", std::fabs(heading),
              std::fabs(path - 500.0f));
  CHECK(std::fabs(heading) <= 0.03f);
  // The distance gate is unchanged by the heading correction.
  CHECK(std::fabs(path - 500.0f) <= kCruise * kPeriod * 0.001f);
}

}  // namespace

int main() {
  std::printf("planner_noise_test:\n");
  testDistanceUnderNoiseAndLag();
  testDistanceBackwardUnderNoiseAndLag();
  testTurnUnderNoiseAndLag();
  testChainUnderNoiseAndLag();
  testStandingRobotDoesNotDrift();
  testTrackingLagSensitivity();
  testSettleCoincidesInZeroErrorSim();
  testSettleCoincidesOnTurnInZeroErrorSim();
  testSettleReportsArrivalTruthfullyUnderNoiseAndLag();
  testSettleReportsTurnArrivalTruthfully();
  testSettleWindowExpiryReportsUnsettled();
  testSettleDoesNotBreakTheCarryChain();
  testHeadingHoldRecoversDisturbance();
  testHeadingHoldOffLeavesDisturbanceStanding();
  testHeadingHoldClampedToVelocityCeiling();
  testHeadingHoldUnderNoiseAndLag();
  std::printf("planner_noise_test: all checks passed\n");
  return 0;
}
