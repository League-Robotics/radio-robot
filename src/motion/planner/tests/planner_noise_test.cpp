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
// Plus heading hold on Distance Moves (M3). Settle-confirm completion (M1,
// PlannerLimits::requireSettle) -- the deferred-completion/creep pair this
// tier used to test here -- is DELETED by 130-008 (planner-honesty-pass):
// the `arrived` completion event (Planner::tick()'s own doc comment)
// already tests settleReached() directly and completes the instant it
// holds, so deferring a completion reached some OTHER way bought nothing
// that reporting TickResult::settled truthfully at the same tick does not.
// `requireSettle` is now unread by Motion::Planner (planner_types.h's own
// updated doc comment); the six tests that used to exercise the defer/
// creep behavior here are gone with it.
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

// ---- item 2: settle-confirm completion (M1) -- DELETED by 130-008 ----
//
// The six tests that used to live here (zero-error coincidence on a
// straight and a turn, truthful settled reporting under noise/lag on both
// axes, settle-window expiry, and non-interference with the carry chain)
// all exercised PlannerLimits::requireSettle's deferred-completion path
// and the closed-loop terminal creep that only ever ran inside it. Both
// are deleted (planner.cpp's Planner::tick()/planActive() doc comments);
// `requireSettle` is now unread. The behavior those tests were guarding --
// TickResult::settled reported truthfully at the tick a Move completes --
// is still exercised (with requireSettle at its default false) by the
// `arrived` event covered in planner_scenarios_test.cpp and by the
// noisy/lagging cases directly above (testDistanceUnderNoiseAndLag et al.,
// which never enabled settle-confirm and are therefore unaffected by its
// removal).

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
  testHeadingHoldRecoversDisturbance();
  testHeadingHoldOffLeavesDisturbanceStanding();
  testHeadingHoldClampedToVelocityCeiling();
  testHeadingHoldUnderNoiseAndLag();
  std::printf("planner_noise_test: all checks passed\n");
  return 0;
}
