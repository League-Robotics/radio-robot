// ratio_lock_test.cpp -- the invariant the per-wheel profiler exists to
// protect: both wheels are profiled against their OWN remaining distance,
// but one common feasible fraction is applied to both, so the commanded
// left:right ratio -- and therefore the commanded turn radius, and
// therefore the heading the Move sweeps -- is preserved EXACTLY, on every
// tick, in every phase.
//
// Letting each wheel run its own profile independently would let the ratio
// drift mid-move, which is heading error by construction. These tests are
// what stops that regression.
#include <algorithm>
#include <cmath>
#include <cstdio>

#include "planner.h"
#include "shape.h"
#include "tests/test_support.h"

using Motion::Move;
using Motion::MoveShape;
using Motion::Planner;
using Motion::PlannerLimits;
using Motion::shapeOf;
using Motion::TickResult;
using TestPlanner::benchLimits;
using TestPlanner::cycle;
using TestPlanner::NoisyPlant;
using TestPlanner::PerfectPlant;

namespace {

Move twistMove(uint32_t id, Move::Kind kind, float threshold, float v_x,
               float omega) {
  Move m;
  m.id = id;
  m.kind = kind;
  m.threshold = threshold;
  m.timeout = 60000.0f;
  m.velocityKind = Move::VelocityKind::Twist;
  m.v_x = v_x;
  m.omega = omega;
  return m;
}

// The heart of the suite: run a Move to completion and assert, on EVERY
// tick, that the staged pair still lies on the shape's own ray --
// cmdLeft*unitRight == cmdRight*unitLeft. Cross-multiplied so a wheel
// commanded to exactly zero (a legitimate one-wheel arc) needs no special
// case and no division by zero.
struct RatioReport {
  int ticks = 0;
  double worstRatioError = 0.0;  // [mm/s] cross-product residual
  float finalLeft = 0.0f;        // [mm] ground-truth wheel travel
  float finalRight = 0.0f;       // [mm]
  bool completed = false;
};

template <typename Plant>
RatioReport runMove(const Move& move, const PlannerLimits& limits,
                    Plant& plant) {
  Planner planner(limits);
  Types::RobotState state{};
  uint32_t now = 1000;
  RatioReport report;
  const MoveShape shape = shapeOf(move, limits.plant.trackWidth);
  CHECK(shape.valid);

  CHECK(planner.move(move, false));
  for (int i = 0; i < 4000; ++i) {
    const TickResult result =
        cycle(planner, state, plant, now, limits.plant.controlPeriod);
    ++report.ticks;
    // The staged pair must stay on the shape's ray, always.
    const double cross =
        static_cast<double>(planner.commandedLeft()) * shape.unitRight -
        static_cast<double>(planner.commandedRight()) * shape.unitLeft;
    report.worstRatioError =
        std::max(report.worstRatioError, std::fabs(cross));
    if (result.completed) {
      report.completed = true;
      break;
    }
  }
  report.finalLeft = plant.positionLeft;
  report.finalRight = plant.positionRight;
  return report;
}

void testRatioHeldOnPerfectPlant() {
  const PlannerLimits limits = benchLimits();
  struct Case {
    const char* name;
    Move move;
  };
  const Case cases[] = {
      {"straight", twistMove(1, Move::Kind::Distance, 500.0f, 150.0f, 0.0f)},
      {"reverse", twistMove(2, Move::Kind::Distance, 500.0f, -150.0f, 0.0f)},
      {"pivot", twistMove(3, Move::Kind::Angle, 1.5707963f, 0.0f, 2.0f)},
      {"pivot back", twistMove(4, Move::Kind::Angle, 1.5707963f, 0.0f, -2.0f)},
      {"arc 3:1", twistMove(5, Move::Kind::Distance, 500.0f, 200.0f, 1.0f)},
      {"tight arc", twistMove(6, Move::Kind::Distance, 400.0f, 120.0f, 1.8f)},
  };
  for (const Case& c : cases) {
    PerfectPlant plant;
    const RatioReport report = runMove(c.move, limits, plant);
    CHECK(report.completed);
    // Exact on a tracking plant: both wheels' profiles agree algebraically,
    // so the ratio residual is pure float rounding.
    if (report.worstRatioError > 1e-3) {
      std::printf("FAIL %s: ratio residual %g\n", c.name,
                  report.worstRatioError);
      std::exit(1);
    }
    // And both wheels land on their own targets, on the same tick.
    const MoveShape shape = shapeOf(c.move, limits.plant.trackWidth);
    CHECK_NEAR(report.finalLeft, shape.distanceLeft, 1e-2);
    CHECK_NEAR(report.finalRight, shape.distanceRight, 1e-2);
  }
}

void testRatioHeldThroughEveryPhase() {
  // A long arc so the run genuinely visits accel, hold and decel, plus the
  // closing step and the post-completion drain -- the ratio must hold in
  // all of them, not just at cruise.
  const PlannerLimits limits = benchLimits();
  const Move move = twistMove(1, Move::Kind::Distance, 2000.0f, 200.0f, 1.0f);
  const MoveShape shape = shapeOf(move, limits.plant.trackWidth);
  Planner planner(limits);
  Types::RobotState state{};
  PerfectPlant plant;
  uint32_t now = 1000;
  CHECK(planner.move(move, false));

  bool sawClimb = false, sawHold = false, sawFall = false;
  float previous = 0.0f;
  double worst = 0.0;
  bool completed = false;
  // Keep ticking past completion so the drain is covered too.
  for (int i = 0; i < 4000; ++i) {
    const TickResult result =
        cycle(planner, state, plant, now, limits.plant.controlPeriod);
    const float dominant =
        std::max(std::fabs(planner.commandedLeft()),
                 std::fabs(planner.commandedRight()));
    if (dominant > previous + 1e-3f) sawClimb = true;
    else if (std::fabs(dominant - previous) <= 1e-3f && dominant > 1.0f)
      sawHold = true;
    else if (dominant < previous - 1e-3f) sawFall = true;
    previous = dominant;

    const double cross =
        static_cast<double>(planner.commandedLeft()) * shape.unitRight -
        static_cast<double>(planner.commandedRight()) * shape.unitLeft;
    worst = std::max(worst, std::fabs(cross));
    if (result.completed) completed = true;
    if (completed && dominant == 0.0f && i > 5) break;
  }
  CHECK(completed);
  CHECK(sawClimb);
  CHECK(sawHold);
  CHECK(sawFall);
  CHECK(worst <= 1e-3);
}

void testCommandedRatioSurvivesAsymmetricPlant() {
  // An 8%-slow left wheel makes the two wheels' remaining distances
  // genuinely diverge. Without the trim the planner cannot fix what the
  // wheel actually does -- but what it COMMANDS must still lie exactly on
  // the shape's ray. (The trim's job, tested in wheel_trim_test, is to
  // close the gap between commanded and actual.)
  const PlannerLimits limits = benchLimits();
  const Move move = twistMove(1, Move::Kind::Distance, 500.0f, 150.0f, 0.0f);
  NoisyPlant plant;
  plant.gainLeft = 0.92f;
  plant.trackingLag = 0.35f;
  plant.delayedActuation = true;
  const RatioReport report = runMove(move, limits, plant);
  CHECK(report.completed);
  CHECK(report.worstRatioError <= 1e-3);
  // The plant really did diverge -- otherwise this test proves nothing.
  CHECK(std::fabs(report.finalLeft - report.finalRight) > 10.0f);
}

void testBehindWheelBindsRatherThanClipping() {
  // When one wheel's own profile allows less than the other's, the ratio
  // lock must scale BOTH down -- never clip the fast wheel alone, which
  // would break the ratio precisely when it matters most.
  //
  // A tight arc drives the outer wheel to its own vMax while the inner
  // wheel has plenty of headroom: the outer wheel binds, and the inner
  // must come down in proportion rather than holding its own cruise.
  PlannerLimits limits = benchLimits();
  limits.ceilings.vMax = 200.0f;  // [mm/s] force the outer wheel to bind
  const Move move = twistMove(1, Move::Kind::Distance, 800.0f, 200.0f, 1.0f);
  const MoveShape shape = shapeOf(move, limits.plant.trackWidth);
  CHECK(shape.cruise > limits.ceilings.vMax);  // the outer wheel really is over

  Planner planner(limits);
  Types::RobotState state{};
  PerfectPlant plant;
  uint32_t now = 1000;
  CHECK(planner.move(move, false));
  bool sawCruise = false;
  for (int i = 0; i < 4000; ++i) {
    const TickResult result =
        cycle(planner, state, plant, now, limits.plant.controlPeriod);
    const float left = planner.commandedLeft();
    const float right = planner.commandedRight();
    // Neither wheel ever exceeds the ceiling ...
    CHECK(std::fabs(left) <= limits.ceilings.vMax + 1e-3f);
    CHECK(std::fabs(right) <= limits.ceilings.vMax + 1e-3f);
    // ... and the ratio is intact even while the outer wheel is pinned.
    const double cross = static_cast<double>(left) * shape.unitRight -
                         static_cast<double>(right) * shape.unitLeft;
    CHECK(std::fabs(cross) <= 1e-3);
    if (std::fabs(right) >= limits.ceilings.vMax - 1e-2f) {
      sawCruise = true;
      // The inner wheel was scaled down in proportion, NOT left at its
      // own uncapped cruise.
      CHECK(std::fabs(left) < shape.cruise * std::fabs(shape.unitLeft));
    }
    if (result.completed) break;
  }
  CHECK(sawCruise);
}

void testOneWheelArcIsLegal() {
  // v_x == omega*trackWidth/2 stops the inner wheel dead: a legitimate
  // one-wheel arc, not a fault. The still wheel constrains nothing and
  // must not divide by zero or drag the moving wheel to a halt.
  const PlannerLimits limits = benchLimits();
  const float omega = 2.0f;  // [rad/s]
  const float v_x = omega * 0.5f * limits.plant.trackWidth;  // [mm/s] -> left wheel 0
  const Move move = twistMove(1, Move::Kind::Distance, 300.0f, v_x, omega);
  const MoveShape shape = shapeOf(move, limits.plant.trackWidth);
  CHECK_NEAR(shape.unitLeft, 0.0f, 1e-6);
  CHECK_NEAR(shape.unitRight, 1.0f, 1e-6);

  PerfectPlant plant;
  const RatioReport report = runMove(move, limits, plant);
  CHECK(report.completed);
  CHECK(report.worstRatioError <= 1e-3);
  CHECK_NEAR(report.finalLeft, 0.0f, 1e-2);
  CHECK_NEAR(report.finalRight, shape.distanceRight, 1e-2);
}

void testNeverAcceleratesAfterBraking() {
  // The decel latch: once a Move has begun braking, its commanded speed
  // may fall or hold, but never rise again.
  const PlannerLimits limits = benchLimits();
  const Move move = twistMove(1, Move::Kind::Distance, 600.0f, 300.0f, 0.0f);
  Planner planner(limits);
  Types::RobotState state{};
  PerfectPlant plant;
  uint32_t now = 1000;
  CHECK(planner.move(move, false));

  float previous = 0.0f;
  bool braking = false;
  for (int i = 0; i < 4000; ++i) {
    const TickResult result =
        cycle(planner, state, plant, now, limits.plant.controlPeriod);
    const float speed = std::fabs(planner.commandedLeft());
    if (speed < previous - 1e-3f) braking = true;
    if (braking) CHECK(speed <= previous + 1e-3f);
    previous = speed;
    if (result.completed) break;
  }
  CHECK(braking);
}

}  // namespace

int main() {
  testRatioHeldOnPerfectPlant();
  testRatioHeldThroughEveryPhase();
  testCommandedRatioSurvivesAsymmetricPlant();
  testBehindWheelBindsRatherThanClipping();
  testOneWheelArcIsLegal();
  testNeverAcceleratesAfterBraking();
  std::printf("ratio_lock_test: all checks passed\n");
  return 0;
}
