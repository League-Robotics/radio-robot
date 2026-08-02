// planner_scenarios_test.cpp -- end-to-end Planner + zero-error plant
// scenarios (motion-planner sketch §7 tier 2): the exactness gates. In a
// zero-error simulation the motion must be exact -- a distance Move
// travels exactly its threshold, a rotation turns exactly its angle, and
// chains leak zero total error across boundaries.
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
using TestPlanner::PerfectPlant;

namespace {

constexpr float kPeriod = 50.0f;  // [ms]

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

// Run until `expected` completions arrive (order-checked), then a few
// drain ticks. Returns the plant for final assertions.
struct RunResult {
  PerfectPlant plant;
  Types::RobotState state;
  int ticks = 0;
  float minVelocityBetween = 1e9f;  // [mm/s] min body speed between completions
};

void testDistanceExact() {
  Planner planner(benchLimits());
  PerfectPlant plant;
  Types::RobotState state;
  uint32_t now = 0;
  CHECK(planner.move(distanceMove(1, 500.0f, 150.0f), false));
  bool completed = false;
  for (int i = 0; i < 400 && !completed; ++i) {
    const TickResult r = cycle(planner, state, plant, now, kPeriod);
    if (r.completed) {
      CHECK(r.moveId == 1);
      CHECK(!r.timedOut);
      completed = true;
    }
    // Limits respected throughout.
    CHECK(std::fabs(state.wheelLeft.cmdVelocity) <= 600.0f + 1e-3f);
  }
  CHECK(completed);
  // EXACT: both wheels traveled exactly 500 mm (float-accumulation floor).
  CHECK_NEAR(plant.positionLeft, 500.0f, 1e-3);
  CHECK_NEAR(plant.positionRight, 500.0f, 1e-3);
  // Drain: commands reach zero and stay there.
  for (int i = 0; i < 10; ++i) cycle(planner, state, plant, now, kPeriod);
  CHECK_NEAR(state.wheelLeft.cmdVelocity, 0.0f, 1e-6);
  CHECK_NEAR(plant.positionLeft, 500.0f, 1e-3);
}

void testDistanceExactBackward() {
  Planner planner(benchLimits());
  PerfectPlant plant;
  Types::RobotState state;
  uint32_t now = 0;
  CHECK(planner.move(distanceMove(2, 300.0f, -200.0f), false));
  bool completed = false;
  for (int i = 0; i < 400 && !completed; ++i) {
    completed = cycle(planner, state, plant, now, kPeriod).completed;
  }
  CHECK(completed);
  CHECK_NEAR(plant.positionLeft, -300.0f, 1e-3);
  CHECK_NEAR(plant.positionRight, -300.0f, 1e-3);
}

void testAngleExact() {
  Planner planner(benchLimits());
  PerfectPlant plant;
  Types::RobotState state;
  uint32_t now = 0;
  const float quarterTurn = static_cast<float>(M_PI) * 0.5f;
  CHECK(planner.move(angleMove(3, quarterTurn, 2.0f), false));
  bool completed = false;
  for (int i = 0; i < 400 && !completed; ++i) {
    completed = cycle(planner, state, plant, now, kPeriod).completed;
  }
  CHECK(completed);
  for (int i = 0; i < 10; ++i) cycle(planner, state, plant, now, kPeriod);
  // EXACT heading: (right - left) / track == pi/2; wheels mirrored.
  const float heading =
      (plant.positionRight - plant.positionLeft) / 100.0f;
  CHECK_NEAR(heading, quarterTurn, 1e-5);
  CHECK_NEAR(plant.positionLeft, -plant.positionRight, 1e-4);
}

void testSameAxisChainExactAndCarried() {
  // Two same-direction distance legs: total EXACTLY 800, and the boundary
  // is crossed at speed (no land-at-zero dip between legs).
  Planner planner(benchLimits());
  PerfectPlant plant;
  Types::RobotState state;
  uint32_t now = 0;
  CHECK(planner.move(distanceMove(10, 500.0f, 150.0f), false));
  CHECK(planner.move(distanceMove(11, 300.0f, 150.0f), false));
  int completions = 0;
  float minSpeedAfterRamp = 1e9f;
  bool rampDone = false;
  bool secondDone = false;
  for (int i = 0; i < 800 && !secondDone; ++i) {
    const TickResult r = cycle(planner, state, plant, now, kPeriod);
    const float speed = std::fabs(state.wheelLeft.cmdVelocity);
    if (speed >= 150.0f - 1e-3f) rampDone = true;
    if (rampDone && completions == 0) {
      minSpeedAfterRamp = std::min(minSpeedAfterRamp, speed);
    }
    if (r.completed) {
      ++completions;
      CHECK(r.moveId == (completions == 1 ? 10u : 11u));
      if (completions == 2) secondDone = true;
    }
  }
  CHECK(completions == 2);
  for (int i = 0; i < 10; ++i) cycle(planner, state, plant, now, kPeriod);
  // Chain-exact: total 800.000, zero boundary leak.
  CHECK_NEAR(plant.positionLeft, 800.0f, 1e-3);
  CHECK_NEAR(plant.positionRight, 800.0f, 1e-3);
  // Full carry: once at cruise, speed never dipped more than one decel
  // step below cruise before the first completion.
  CHECK(minSpeedAfterRamp >= 150.0f - 300.0f * 0.05f - 1e-3f);
}

void testOrthogonalChainExact() {
  // Distance leg then rotation: each lands exactly; the rotation leg does
  // not disturb the traveled distance (opposite wheels).
  Planner planner(benchLimits());
  PerfectPlant plant;
  Types::RobotState state;
  uint32_t now = 0;
  const float quarterTurn = static_cast<float>(M_PI) * 0.5f;
  CHECK(planner.move(distanceMove(20, 400.0f, 150.0f), false));
  CHECK(planner.move(angleMove(21, quarterTurn, 2.0f), false));
  int completions = 0;
  for (int i = 0; i < 800 && completions < 2; ++i) {
    if (cycle(planner, state, plant, now, kPeriod).completed) ++completions;
  }
  CHECK(completions == 2);
  for (int i = 0; i < 10; ++i) cycle(planner, state, plant, now, kPeriod);
  const float heading =
      (plant.positionRight - plant.positionLeft) / 100.0f;
  const float path = 0.5f * (plant.positionLeft + plant.positionRight);
  CHECK_NEAR(path, 400.0f, 1e-3);
  CHECK_NEAR(heading, quarterTurn, 1e-5);
}

void testTimeoutReported() {
  // A distance the cruise can never cover before the timeout.
  Planner planner(benchLimits());
  PerfectPlant plant;
  Types::RobotState state;
  uint32_t now = 0;
  Move m = distanceMove(30, 100000.0f, 150.0f);
  m.timeout = 2000.0f;  // [ms]
  CHECK(planner.move(m, false));
  bool completed = false;
  TickResult last{};
  for (int i = 0; i < 100 && !completed; ++i) {
    last = cycle(planner, state, plant, now, kPeriod);
    completed = last.completed;
  }
  CHECK(completed);
  CHECK(last.moveId == 30);
  CHECK(last.timedOut);
}

void testStopFlushes() {
  Planner planner(benchLimits());
  PerfectPlant plant;
  Types::RobotState state;
  uint32_t now = 0;
  CHECK(planner.move(distanceMove(40, 5000.0f, 150.0f), false));
  CHECK(planner.move(distanceMove(41, 5000.0f, 150.0f), false));
  for (int i = 0; i < 20; ++i) cycle(planner, state, plant, now, kPeriod);
  CHECK(planner.active());
  planner.estop();
  CHECK(!planner.active());
  CHECK(planner.pendingCount() == 0);
  cycle(planner, state, plant, now, kPeriod);
  CHECK_NEAR(state.wheelLeft.cmdVelocity, 0.0f, 1e-6);
}

void testReplacePreempts() {
  Planner planner(benchLimits());
  PerfectPlant plant;
  Types::RobotState state;
  uint32_t now = 0;
  CHECK(planner.move(distanceMove(50, 5000.0f, 150.0f), false));
  CHECK(planner.move(distanceMove(51, 5000.0f, 150.0f), false));
  for (int i = 0; i < 20; ++i) cycle(planner, state, plant, now, kPeriod);
  const float positionAtReplace = plant.positionLeft;
  CHECK(planner.move(distanceMove(52, 200.0f, 150.0f), true));
  CHECK(planner.pendingCount() == 1);  // only the replacement
  bool completed = false;
  TickResult last{};
  for (int i = 0; i < 400 && !completed; ++i) {
    last = cycle(planner, state, plant, now, kPeriod);
    completed = last.completed;
  }
  CHECK(completed);
  CHECK(last.moveId == 52);
  for (int i = 0; i < 10; ++i) cycle(planner, state, plant, now, kPeriod);
  // The replacement's 200 mm are measured from its own activation. The
  // one-tick activation latency (move() queues, next tick() activates)
  // travels at most one cruise interval beyond the replace point; the
  // replacement itself is exact from its baseline.
  const float traveledAfter = plant.positionLeft - positionAtReplace;
  CHECK(traveledAfter >= 200.0f - 1e-3f);
  CHECK(traveledAfter <= 200.0f + 150.0f * 0.05f + 1e-3f);
}

void testQueueFullRejected() {
  Planner planner(benchLimits());
  PerfectPlant plant;
  Types::RobotState state;
  uint32_t now = 0;
  // Fill: 5 accepted while nothing is active yet (all pending).
  for (uint32_t i = 0; i < 5; ++i) {
    CHECK(planner.move(distanceMove(60 + i, 100.0f, 150.0f), false));
  }
  CHECK(!planner.move(distanceMove(69, 100.0f, 150.0f), false));  // ERR_FULL
  // Activation alone frees nothing (1 active + 4 pending is still 5);
  // a slot opens only when the first Move COMPLETES.
  cycle(planner, state, plant, now, kPeriod);
  CHECK(!planner.move(distanceMove(70, 100.0f, 150.0f), false));
  bool completed = false;
  for (int i = 0; i < 400 && !completed; ++i) {
    completed = cycle(planner, state, plant, now, kPeriod).completed;
  }
  CHECK(completed);
  CHECK(planner.move(distanceMove(71, 100.0f, 150.0f), false));
  CHECK(!planner.move(distanceMove(72, 100.0f, 150.0f), false));
}

void testInvalidMovesRejected() {
  Planner planner(benchLimits());
  Move noVelocity = distanceMove(80, 100.0f, 0.0f);
  CHECK(!planner.move(noVelocity, false));
  // v_y is carried for a future holonomic drivetrain but not actuated
  // here, so a Move whose only linear velocity is sideways is a motion
  // this robot cannot make -- rejected, not silently driven as nothing.
  Move sideways = distanceMove(82, 100.0f, 0.0f);
  sideways.v_y = 150.0f;
  CHECK(!planner.move(sideways, false));
  // But v_y riding along on an otherwise valid Move is accepted and
  // ignored, exactly as the wire's own MoveTwist.v_y is.
  Move forwardWithStrafe = distanceMove(83, 100.0f, 150.0f);
  forwardWithStrafe.v_y = 150.0f;
  CHECK(planner.move(forwardWithStrafe, false));
  // Wheels Moves accept every stop kind (firmware integration lifted v1's
  // Time-only restriction): the wire protocol's move_wheels arm carries
  // time, distance, and angle stops. The pair ramps and HOLDS; completion
  // is the standard measured-threshold test; the post-completion drain
  // ramps down.
  Move wheelsDistance;
  wheelsDistance.id = 81;
  wheelsDistance.kind = Move::Kind::Distance;
  wheelsDistance.velocityKind = Move::VelocityKind::Wheels;
  wheelsDistance.threshold = 100.0f;
  wheelsDistance.vLeft = 100.0f;
  wheelsDistance.vRight = 100.0f;
  wheelsDistance.timeout = 10000.0f;
  CHECK(planner.move(wheelsDistance, false));
}

void testTimeMoveRuns() {
  Planner planner(benchLimits());
  PerfectPlant plant;
  Types::RobotState state;
  uint32_t now = 0;
  Move m;
  m.id = 90;
  m.kind = Move::Kind::Time;
  m.threshold = 2000.0f;  // [ms]
  m.v_x = 150.0f;
  m.timeout = 10000.0f;
  CHECK(planner.move(m, false));
  bool completed = false;
  int ticks = 0;
  for (int i = 0; i < 100 && !completed; ++i) {
    completed = cycle(planner, state, plant, now, kPeriod).completed;
    ++ticks;
  }
  CHECK(completed);
  // 2000 ms / 50 ms = 40 ticks (+1 for the activation tick).
  CHECK(ticks >= 40 && ticks <= 42);
  // Ramped down toward zero at the end (time-domain taper).
  for (int i = 0; i < 10; ++i) cycle(planner, state, plant, now, kPeriod);
  CHECK_NEAR(state.wheelLeft.cmdVelocity, 0.0f, 1e-6);
}

// Two same-direction Distance legs of DIFFERENT curvature: a straight
// followed by a gentle left arc. testSameAxisChainExactAndCarried() above
// covers the EXACT-ratio hand-off (same shape both legs); this is the NEW
// relaxed case shapesCompatible() alone used to refuse outright, landing
// the straight fully at rest before ever starting the arc. Neither wheel
// reverses direction, so boundaryLambda()'s relaxed path applies: the
// hand-off must still occur at meaningfully-nonzero speed (never a full
// stop), and the DOMINANT wheel's own per-tick step must never exceed its
// configured accel ceiling -- planWheels()'s new per-wheel ceiling rail
// (see its own comment) is what guarantees that regardless of what the
// ratio lock's tie-break alone would have picked.
void testCurvatureChainAtSpeed() {
  Planner planner(benchLimits());
  PerfectPlant plant;
  Types::RobotState state;
  uint32_t now = 0;

  Move straight = distanceMove(40, 500.0f, 300.0f);  // ratio 1:1, cruise 300
  Move arc = distanceMove(41, 400.0f, 300.0f);
  arc.omega = 0.9f;  // wheels (255, 345): ratio 0.7391:1, cruise 345
  CHECK(planner.move(straight, false));
  CHECK(planner.move(arc, false));

  int completions = 0;
  float minRight = 1e9f;   // [mm/s] dominant wheel's minimum after ramp-up
  float previousRight = 0.0f;
  float maxDominantStep = 0.0f;  // [mm/s] largest one-tick |delta right|
  bool rampDone = false;
  bool sawRatioAfterHandoff = false;
  for (int i = 0; i < 800 && completions < 2; ++i) {
    const TickResult r = cycle(planner, state, plant, now, kPeriod);
    const float left = state.wheelLeft.cmdVelocity;
    const float right = state.wheelRight.cmdVelocity;
    CHECK(std::fabs(left) <= 600.0f + 1e-3f);
    CHECK(std::fabs(right) <= 600.0f + 1e-3f);
    if (right >= 300.0f - 1e-2f) rampDone = true;
    // Track the boundary dip only from first reaching the straight's
    // cruise until recovering to near the arc's own cruise -- past that
    // point the arc is running normally and will (correctly) decelerate
    // toward rest as IT completes, which is not what this test is about.
    if (rampDone && !sawRatioAfterHandoff) minRight = std::min(minRight, right);
    if (i > 0) {
      maxDominantStep = std::max(maxDominantStep, std::fabs(right - previousRight));
    }
    if (r.completed) {
      ++completions;
      CHECK(r.moveId == (completions == 1 ? 40u : 41u));
    }
    // Once the arc is active and past its own initial transient, the
    // commanded ratio must sit exactly on 255/345 -- the ratio lock's
    // guarantee holds ACROSS the relaxed hand-off, not just within a move
    // that never changed shape.
    if (completions == 1 && right > 340.0f) {
      CHECK_NEAR(left / right, 255.0f / 345.0f, 1e-3);
      sawRatioAfterHandoff = true;
    }
    previousRight = right;
  }
  CHECK(completions == 2);
  CHECK(sawRatioAfterHandoff);

  // Never lands at rest at the boundary: the dominant wheel stayed well
  // above zero the whole way through (a full land-at-zero hand-off, the
  // pre-existing exact-ratio-only behavior, would have touched 0).
  CHECK(minRight > 100.0f);
  // The dominant (right) wheel's own per-tick step never exceeds a small,
  // generous multiple of what one control cycle's accel ceiling allows --
  // far below the UNBOUNDED per-wheel jump the axisPerLambda rescale
  // defect produced (433 mm/s, clasi/issues/replace-rescales-carried-
  // profile-velocity-by-new-shape.md).
  CHECK(maxDominantStep <= 50.0f);
}

// A curvature change that would REVERSE a wheel's direction (a tight
// left arc into a comparatively straight leg reverses the inner wheel) is
// explicitly excluded from the relaxed path (shapeDirectionsAgree()) --
// this must still land at rest, exactly as shapesCompatible()-only ever
// did.
void testCurvatureReversalStillLandsAtRest() {
  Planner planner(benchLimits());
  PerfectPlant plant;
  Types::RobotState state;
  uint32_t now = 0;

  Move tightArc = distanceMove(50, 400.0f, 300.0f);
  tightArc.omega = 8.0f;  // wheels (-100, 700): left reverses
  Move straight = distanceMove(51, 300.0f, 150.0f);
  CHECK(planner.move(tightArc, false));
  CHECK(planner.move(straight, false));

  int completions = 0;
  bool sawRestAtBoundary = false;
  for (int i = 0; i < 800 && completions < 2; ++i) {
    const TickResult r = cycle(planner, state, plant, now, kPeriod);
    if (completions == 1 &&
        std::fabs(state.wheelLeft.cmdVelocity) < 1.0f &&
        std::fabs(state.wheelRight.cmdVelocity) < 1.0f) {
      sawRestAtBoundary = true;
    }
    if (r.completed) ++completions;
  }
  CHECK(completions == 2);
  CHECK(sawRestAtBoundary);
}

// ===========================================================================
// cmdAccel round-trips through Types::RobotState (130-003,
// wheel-speed-controller-moves-into-drive.md): Planner::update() publishes
// cmdAccel as the finite difference of the staged command across the ONE
// control period that just elapsed, so App::Drive's forthcoming unified
// controller (ticket 004) can read it straight off the same blackboard
// tick() already hands Drive. This is plumbing-only smoke coverage -- it
// proves the wire-up, not any control law (there is none yet).
// ===========================================================================

void testCmdAccelRoundTrips() {
  Planner planner(benchLimits());
  PerfectPlant plant;
  Types::RobotState state;
  uint32_t now = 0;
  CHECK(planner.move(distanceMove(1, 5000.0f, 300.0f), false));

  // Tick 1: ramps up from rest. rollCommandHistory() ages the PRE-tick
  // cmdLeft_ (still 0, the pre-Move default) into cmdLeftPrevious_ before
  // this tick's own cmdLeft_ is computed -- so cmdAccel after this first
  // cycle() describes "leaving rest," not yet a ramp-to-ramp measurement.
  cycle(planner, state, plant, now, kPeriod);
  const float firstLeft = state.wheelLeft.cmdVelocity;
  const float firstRight = state.wheelRight.cmdVelocity;
  CHECK(firstLeft > 0.0f);
  CHECK(firstRight > 0.0f);

  // Tick 2: cmdLeftPrevious_/cmdRightPrevious_ are now tick 1's real,
  // nonzero cmdLeft_/cmdRight_ -- a genuine ramp-to-ramp finite difference,
  // independently computed here from the two OBSERVED cmdVelocity samples
  // rather than from any Planner-internal state.
  cycle(planner, state, plant, now, kPeriod);
  const float secondLeft = state.wheelLeft.cmdVelocity;
  const float secondRight = state.wheelRight.cmdVelocity;
  const float dt = kPeriod * 0.001f;  // [s]
  const float expectedLeftAccel = (secondLeft - firstLeft) / dt;
  const float expectedRightAccel = (secondRight - firstRight) / dt;

  // Genuinely accelerating (still ramping toward the 300 mm/s cruise), so
  // this is a real round-trip check, not a degenerate 0-equals-0 pass.
  CHECK(expectedLeftAccel > 1.0f);
  CHECK(expectedRightAccel > 1.0f);
  CHECK_NEAR(state.wheelLeft.cmdAccel, expectedLeftAccel, 1e-2f);
  CHECK_NEAR(state.wheelRight.cmdAccel, expectedRightAccel, 1e-2f);
  // Straight-line distance Move: both wheels ramp identically.
  CHECK_NEAR(state.wheelLeft.cmdAccel, state.wheelRight.cmdAccel, 1e-3f);
}

}  // namespace

int main() {
  testDistanceExact();
  testDistanceExactBackward();
  testAngleExact();
  testSameAxisChainExactAndCarried();
  testOrthogonalChainExact();
  testTimeoutReported();
  testStopFlushes();
  testReplacePreempts();
  testQueueFullRejected();
  testInvalidMovesRejected();
  testTimeMoveRuns();
  testCurvatureChainAtSpeed();
  testCurvatureReversalStillLandsAtRest();
  testCmdAccelRoundTrips();
  std::printf("planner_scenarios_test: all checks passed\n");
  return 0;
}
