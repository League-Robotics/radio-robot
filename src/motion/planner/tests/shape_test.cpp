// shape_test.cpp -- unit tests for the Move -> per-wheel reduction
// (shape.h): the constant-ratio decomposition, per-wheel distance targets
// for every (Kind x VelocityKind) combination, the degenerate guards, the
// shapeLimits() projection (which must reduce EXACTLY to the hand-written
// per-Kind ceilings it replaces), and lookahead compatibility.
#include <algorithm>
#include <cmath>
#include <cstdio>

#include "shape.h"
#include "tests/test_support.h"

using Motion::AxisLimits;
using Motion::curvatureHandoffLambdaCap;
using Motion::Move;
using Motion::MoveShape;
using Motion::PlannerLimits;
using Motion::shapeDirectionsAgree;
using Motion::shapeLimits;
using Motion::shapeOf;
using Motion::shapesCompatible;

namespace {

constexpr float kTrack = 100.0f;  // [mm]

Move twistMove(Move::Kind kind, float threshold, float v_x, float omega) {
  Move m;
  m.kind = kind;
  m.threshold = threshold;
  m.velocityKind = Move::VelocityKind::Twist;
  m.v_x = v_x;
  m.omega = omega;
  return m;
}

Move wheelsMove(Move::Kind kind, float threshold, float vLeft, float vRight) {
  Move m;
  m.kind = kind;
  m.threshold = threshold;
  m.velocityKind = Move::VelocityKind::Wheels;
  m.vLeft = vLeft;
  m.vRight = vRight;
  return m;
}

void testStraightDistance() {
  // 500 mm at 150 mm/s: both wheels travel 500, ratio 1:1, cruise 150.
  const MoveShape s = shapeOf(twistMove(Move::Kind::Distance, 500.0f, 150.0f,
                                        0.0f),
                              kTrack);
  CHECK(s.valid);
  CHECK(s.hasDistanceTarget);
  CHECK(!s.hasTimeBudget);
  CHECK_NEAR(s.unitLeft, 1.0f, 1e-6);
  CHECK_NEAR(s.unitRight, 1.0f, 1e-6);
  CHECK_NEAR(s.cruise, 150.0f, 1e-4);
  CHECK_NEAR(s.distanceLeft, 500.0f, 1e-3);
  CHECK_NEAR(s.distanceRight, 500.0f, 1e-3);
  // The dominant wheel's unit is a literal 1.0f -- the ratio lock's
  // tie-break depends on it.
  CHECK(s.unitLeft == 1.0f);

  // Reverse: same distances, opposite sign; threshold stays a magnitude.
  const MoveShape r = shapeOf(twistMove(Move::Kind::Distance, 500.0f, -150.0f,
                                        0.0f),
                              kTrack);
  CHECK(r.valid);
  CHECK_NEAR(r.unitLeft, -1.0f, 1e-6);
  CHECK_NEAR(r.unitRight, -1.0f, 1e-6);
  CHECK_NEAR(r.cruise, 150.0f, 1e-4);
  CHECK_NEAR(r.distanceLeft, -500.0f, 1e-3);
  CHECK_NEAR(r.distanceRight, -500.0f, 1e-3);
}

void testPivotAngle() {
  // A pure pivot: wheel arc is +-threshold * trackWidth/2, which the
  // general duration formula must produce without a special case.
  const float theta = 1.5707963f;  // [rad] 90 deg
  const MoveShape s =
      shapeOf(twistMove(Move::Kind::Angle, theta, 0.0f, 2.0f), kTrack);
  CHECK(s.valid);
  CHECK(s.hasDistanceTarget);
  CHECK_NEAR(s.unitLeft, -1.0f, 1e-6);
  CHECK_NEAR(s.unitRight, 1.0f, 1e-6);
  CHECK_NEAR(s.cruise, 100.0f, 1e-4);  // omega * halfTrack = 2 * 50
  const double arc = theta * 0.5 * kTrack;
  CHECK_NEAR(s.distanceLeft, -arc, 1e-3);
  CHECK_NEAR(s.distanceRight, arc, 1e-3);
  // Equal and opposite: a pivot translates the body not at all.
  CHECK_NEAR(s.distanceLeft + s.distanceRight, 0.0, 1e-4);

  // Opposite sense flips both signs.
  const MoveShape n =
      shapeOf(twistMove(Move::Kind::Angle, theta, 0.0f, -2.0f), kTrack);
  CHECK(n.valid);
  CHECK_NEAR(n.distanceLeft, arc, 1e-3);
  CHECK_NEAR(n.distanceRight, -arc, 1e-3);
}

void testArcDistance() {
  // v_x = 200, omega = 1, W = 100 -> wheels (150, 250), ratio 0.6:1,
  // cruise 250 (the dominant, outer wheel). 500 mm of PATH at a mean
  // speed of 200 mm/s is 2.5 s, so the wheels travel 375 and 625, the
  // mean is exactly 500, and the swept heading is omega * T = 2.5 rad.
  const MoveShape s =
      shapeOf(twistMove(Move::Kind::Distance, 500.0f, 200.0f, 1.0f), kTrack);
  CHECK(s.valid);
  CHECK_NEAR(s.unitLeft, 0.6f, 1e-5);
  CHECK_NEAR(s.unitRight, 1.0f, 1e-6);
  CHECK(s.unitRight == 1.0f);  // dominant wheel snapped exactly
  CHECK_NEAR(s.cruise, 250.0f, 1e-4);
  CHECK_NEAR(s.duration, 2.5f, 1e-5);
  CHECK_NEAR(s.distanceLeft, 375.0f, 1e-3);
  CHECK_NEAR(s.distanceRight, 625.0f, 1e-3);
  CHECK_NEAR(0.5 * (s.distanceLeft + s.distanceRight), 500.0, 1e-3);
  const double heading = (s.distanceRight - s.distanceLeft) / kTrack;
  CHECK_NEAR(heading, 2.5, 1e-4);
}

void testTimeMoveHasNoDistanceTarget() {
  const MoveShape s =
      shapeOf(twistMove(Move::Kind::Time, 1500.0f, 120.0f, 0.0f), kTrack);
  CHECK(s.valid);
  CHECK(s.hasTimeBudget);
  CHECK(!s.hasDistanceTarget);
  CHECK_NEAR(s.duration, 1.5f, 1e-6);  // [ms] -> [s]
  CHECK_NEAR(s.cruise, 120.0f, 1e-4);
}

void testWheelsShapes() {
  // Wheels moves still get a shape (the lookahead uses it) even though
  // planActive() deliberately ramps-and-holds them rather than landing.
  const MoveShape d = shapeOf(wheelsMove(Move::Kind::Distance, 300.0f, 300.0f,
                                         100.0f),
                              kTrack);
  CHECK(d.valid);
  CHECK_NEAR(d.unitLeft, 1.0f, 1e-6);
  CHECK_NEAR(d.unitRight, 1.0f / 3.0f, 1e-5);
  CHECK_NEAR(d.cruise, 300.0f, 1e-4);
  // mean speed 200 mm/s, 300 mm -> 1.5 s -> wheels 450 and 150.
  CHECK_NEAR(d.duration, 1.5f, 1e-5);
  CHECK_NEAR(d.distanceLeft, 450.0f, 1e-3);
  CHECK_NEAR(d.distanceRight, 150.0f, 1e-3);

  // Wheels pivot with an Angle stop: diff = 400 mm/s over a 100 mm track
  // is 4 rad/s, so 2 rad takes 0.5 s -> +-100 mm of wheel arc.
  const MoveShape a = shapeOf(wheelsMove(Move::Kind::Angle, 2.0f, -200.0f,
                                         200.0f),
                              kTrack);
  CHECK(a.valid);
  CHECK_NEAR(a.duration, 0.5f, 1e-5);
  CHECK_NEAR(a.distanceLeft, -100.0f, 1e-3);
  CHECK_NEAR(a.distanceRight, 100.0f, 1e-3);
}

void testDegenerateGuards() {
  // A pivot cannot satisfy a distance stop: no net translation.
  CHECK(!shapeOf(twistMove(Move::Kind::Distance, 500.0f, 0.0f, 2.0f),
                 kTrack).valid);
  CHECK(!shapeOf(wheelsMove(Move::Kind::Distance, 500.0f, -200.0f, 200.0f),
                 kTrack).valid);
  // A straight cannot satisfy an angle stop: no net rotation.
  CHECK(!shapeOf(twistMove(Move::Kind::Angle, 1.0f, 150.0f, 0.0f),
                 kTrack).valid);
  CHECK(!shapeOf(wheelsMove(Move::Kind::Angle, 1.0f, 150.0f, 150.0f),
                 kTrack).valid);
  // Commanded to stand still.
  CHECK(!shapeOf(twistMove(Move::Kind::Distance, 500.0f, 0.0f, 0.0f),
                 kTrack).valid);
  // Zero/negative threshold.
  CHECK(!shapeOf(twistMove(Move::Kind::Distance, 0.0f, 150.0f, 0.0f),
                 kTrack).valid);
  // A planned stop has no shape at all -- and is therefore automatically a
  // lookahead barrier.
  Move stop;
  stop.kind = Move::Kind::Stop;
  CHECK(!shapeOf(stop, kTrack).valid);
  // Nonsense geometry fails closed rather than dividing by zero.
  CHECK(!shapeOf(twistMove(Move::Kind::Distance, 500.0f, 150.0f, 0.0f),
                 0.0f).valid);
}

void testShapeLimitsReduceExactly() {
  PlannerLimits limits = TestPlanner::benchLimits();
  limits.ceilings.jerkMax = 5000.0f;   // [mm/s^3]
  limits.ceilings.yawJerkMax = 30.0f;  // [rad/s^3]

  // A straight must reduce to the LINEAR ceilings, untouched -- this is
  // the exact brace-init the per-Kind Distance case used.
  const MoveShape straight =
      shapeOf(twistMove(Move::Kind::Distance, 500.0f, 150.0f, 0.0f),
              limits.plant.trackWidth);
  const AxisLimits lin = shapeLimits(straight, limits);
  CHECK_NEAR(lin.vMax, limits.ceilings.vMax, 1e-4);
  CHECK_NEAR(lin.aMax, limits.ceilings.aMax, 1e-4);
  CHECK_NEAR(lin.aDecel, limits.ceilings.aDecel, 1e-4);
  CHECK_NEAR(lin.jMax, limits.ceilings.jerkMax, 1e-4);

  // A pivot must reduce to the ANGULAR ceilings scaled by half the track --
  // the same scaling the per-Kind Angle case applied AFTER profiling, which
  // is equivalent because profileStep() is homogeneous of degree 1.
  const float halfTrack = 0.5f * limits.plant.trackWidth;
  const MoveShape pivot =
      shapeOf(twistMove(Move::Kind::Angle, 1.5707963f, 0.0f, 2.0f),
              limits.plant.trackWidth);
  const AxisLimits ang = shapeLimits(pivot, limits);
  CHECK_NEAR(ang.aMax, limits.ceilings.alphaMax * halfTrack, 1e-3);
  CHECK_NEAR(ang.aDecel, limits.ceilings.alphaDecel * halfTrack, 1e-3);
  CHECK_NEAR(ang.jMax, limits.ceilings.yawJerkMax * halfTrack, 1e-3);
  // vMax is the tighter of the angular bound and the wheel ceiling.
  CHECK_NEAR(ang.vMax,
             std::min(limits.ceilings.omegaMax * halfTrack, limits.ceilings.vMax), 1e-3);

  // An arc is bounded by whichever axis binds first, and never by more
  // than either.
  const MoveShape arc =
      shapeOf(twistMove(Move::Kind::Distance, 500.0f, 200.0f, 1.0f),
              limits.plant.trackWidth);
  const AxisLimits both = shapeLimits(arc, limits);
  const float mean = std::fabs(0.5f * (arc.unitLeft + arc.unitRight));
  const float diff = std::fabs(arc.unitRight - arc.unitLeft);
  CHECK_NEAR(both.aMax,
             std::min(limits.ceilings.aMax / mean,
                      limits.ceilings.alphaMax * limits.plant.trackWidth / diff), 1e-2);
  CHECK(both.vMax <= limits.ceilings.vMax + 1e-3f);

  // An invalid shape yields all-zero ceilings: fail closed, never a
  // silently unbounded profile.
  const AxisLimits none = shapeLimits(MoveShape{}, limits);
  CHECK(none.vMax == 0.0f);
  CHECK(none.aMax == 0.0f);
  CHECK(none.aDecel == 0.0f);
}

void testCompatibility() {
  auto shape = [](float v_x, float omega) {
    return shapeOf(twistMove(Move::Kind::Distance, 500.0f, v_x, omega),
                   kTrack);
  };
  auto angle = [](float omega) {
    return shapeOf(twistMove(Move::Kind::Angle, 1.0f, 0.0f, omega), kTrack);
  };

  const MoveShape fwd = shape(150.0f, 0.0f);
  const MoveShape fwdFast = shape(400.0f, 0.0f);
  const MoveShape rev = shape(-150.0f, 0.0f);
  const MoveShape turnL = angle(2.0f);
  const MoveShape turnR = angle(-2.0f);

  // Same geometry and direction: hand off at speed, whatever the cruise.
  CHECK(shapesCompatible(fwd, fwdFast));
  CHECK(shapesCompatible(fwdFast, fwd));
  CHECK(shapesCompatible(turnL, angle(5.0f)));
  // Reversal, axis change, opposite turn: must land at zero.
  CHECK(!shapesCompatible(fwd, rev));
  CHECK(!shapesCompatible(fwd, turnL));
  CHECK(!shapesCompatible(turnL, fwd));
  CHECK(!shapesCompatible(turnL, turnR));
  // Arcs: identical radius carries, a different radius does not.
  CHECK(shapesCompatible(shape(200.0f, 1.0f), shape(400.0f, 2.0f)));
  CHECK(!shapesCompatible(shape(200.0f, 1.0f), shape(400.0f, 1.0f)));
  // Wheels pairs of identical ratio carry -- expressible only now.
  CHECK(shapesCompatible(
      shapeOf(wheelsMove(Move::Kind::Distance, 300.0f, 300.0f, 100.0f), kTrack),
      shapeOf(wheelsMove(Move::Kind::Distance, 300.0f, 150.0f, 50.0f), kTrack)));
  // Anything into a planned stop, or an invalid shape, is a barrier.
  Move stop;
  stop.kind = Move::Kind::Stop;
  CHECK(!shapesCompatible(fwd, shapeOf(stop, kTrack)));
  CHECK(!shapesCompatible(MoveShape{}, fwd));
}

void testDirectionsAgree() {
  auto shape = [](float v_x, float omega) {
    return shapeOf(twistMove(Move::Kind::Distance, 500.0f, v_x, omega),
                   kTrack);
  };
  const MoveShape straight = shape(300.0f, 0.0f);          // (1, 1)
  const MoveShape gentleArc = shape(300.0f, 0.9f);         // (0.7391, 1) -- both positive
  const MoveShape sharpArc = shape(300.0f, 2.0f);          // (0.5, 1) -- both positive
  const MoveShape tightReverseArc = shape(300.0f, 8.0f);   // left goes negative
  const MoveShape rev = shape(-300.0f, 0.0f);              // (-1, -1)

  // Identical ratio: trivially agrees (this is what shapesCompatible()
  // already covers, but the direction test must not regress it).
  CHECK(shapeDirectionsAgree(straight, straight));
  // Curvature changes that keep every wheel's sign: agree, regardless of
  // how different the ratios are.
  CHECK(shapeDirectionsAgree(straight, gentleArc));
  CHECK(shapeDirectionsAgree(straight, sharpArc));
  CHECK(shapeDirectionsAgree(gentleArc, sharpArc));
  CHECK(shapeDirectionsAgree(sharpArc, straight));  // symmetric
  // A wheel that must reverse: disagrees.
  CHECK(!shapeDirectionsAgree(straight, tightReverseArc));
  CHECK(!shapeDirectionsAgree(tightReverseArc, straight));
  CHECK(!shapeDirectionsAgree(straight, rev));
  // Invalid shapes never agree.
  CHECK(!shapeDirectionsAgree(MoveShape{}, straight));
  CHECK(!shapeDirectionsAgree(straight, MoveShape{}));
}

void testCurvatureHandoffLambdaCap() {
  auto shape = [](float v_x, float omega) {
    return shapeOf(twistMove(Move::Kind::Distance, 500.0f, v_x, omega),
                   kTrack);
  };
  const MoveShape straight = shape(300.0f, 0.0f);   // (1, 1)
  const MoveShape gentleArc = shape(300.0f, 0.4f);  // small delta
  const MoveShape sharpArc = shape(300.0f, 3.0f);   // large delta

  constexpr float kDt = 0.05f;       // [s]
  constexpr float kCeiling = 300.0f;  // [mm/s^2]

  // Identical ratio: no wheel's unit changes, nothing to bound -- the cap
  // is unbounded (the caller's own cruise/distance caps are what apply).
  const float exact = curvatureHandoffLambdaCap(straight, straight, kCeiling,
                                                kDt, 8);
  CHECK(exact > 1.0e6f);

  // A bigger curvature change yields a TIGHTER (smaller) cap -- the
  // landing speed must fall as the differential the hand-off demands
  // grows, exactly the "slow for that arc, not globally" property the
  // relaxed hand-off is supposed to have.
  const float gentleCap =
      curvatureHandoffLambdaCap(straight, gentleArc, kCeiling, kDt, 8);
  const float sharpCap =
      curvatureHandoffLambdaCap(straight, sharpArc, kCeiling, kDt, 8);
  CHECK(gentleCap > 0.0f);
  CHECK(sharpCap > 0.0f);
  CHECK(sharpCap < gentleCap);

  // More blend cycles (more time to absorb the differential) relaxes the
  // cap; fewer tightens it.
  const float wideBudget =
      curvatureHandoffLambdaCap(straight, gentleArc, kCeiling, kDt, 16);
  const float narrowBudget =
      curvatureHandoffLambdaCap(straight, gentleArc, kCeiling, kDt, 2);
  CHECK(wideBudget > gentleCap);
  CHECK(narrowBudget < gentleCap);

  // Fail closed: no declared decel authority means no cap can be derived,
  // same posture as boundaryLambda()'s own aDecel <= 0 guard.
  CHECK(curvatureHandoffLambdaCap(straight, gentleArc, 0.0f, kDt, 8) == 0.0f);
  // Invalid shapes: no cap can be derived either.
  CHECK(curvatureHandoffLambdaCap(MoveShape{}, gentleArc, kCeiling, kDt, 8) ==
        0.0f);
}

}  // namespace

int main() {
  testStraightDistance();
  testPivotAngle();
  testArcDistance();
  testTimeMoveHasNoDistanceTarget();
  testWheelsShapes();
  testDegenerateGuards();
  testShapeLimitsReduceExactly();
  testCompatibility();
  testDirectionsAgree();
  testCurvatureHandoffLambdaCap();
  std::printf("shape_test: all checks passed\n");
  return 0;
}
