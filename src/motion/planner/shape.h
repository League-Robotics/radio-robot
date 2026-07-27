// shape.h -- reduce a Move to what the per-wheel profiler actually plans on.
//
// Every Move this planner runs is CONSTANT-RATIO: the commanded left:right
// wheel-velocity ratio never changes for the Move's whole duration. A
// straight holds 1:1, a pivot holds -1:1, an arc holds whatever ratio its
// twist implies, a Wheels Move holds the pair it was given. That single
// fact collapses the old four-way per-Kind switch into one derivation:
//
//   1. decompose the Move to a wheel-velocity pair (vLeft, vRight)
//   2. normalize by the DOMINANT wheel -> unit ratio (uLeft, uRight),
//      max(|uLeft|,|uRight|) == 1, and cruise = the dominant wheel's speed
//   3. compute the duration T the Move would take at cruise
//   4. per-wheel distance target = v_w * T
//
// The profiler then plans ONE scalar -- lambda, the dominant wheel's speed
// -- and each wheel's command is lambda * u_w. The ratio is preserved by
// construction, so the commanded turn radius (and therefore the heading
// the Move sweeps) cannot drift no matter what the profile does.
//
// Normalizing by the MAX rather than by, say, the mean is deliberate: it
// makes the dominant wheel's |u| exactly 1.0f, so its arithmetic is a
// no-op and only the sub-dominant wheel eats rounding. That matters for
// the discrete-exact landing guarantee (profile.h) surviving the per-wheel
// split -- see Planner::planActive()'s ratio lock.
#pragma once

#include "planner_types.h"
#include "profile.h"

namespace Motion {

struct MoveShape {
  // Unit wheel ratio, normalized so max(|unitLeft|,|unitRight|) == 1.
  float unitLeft = 0.0f;   // [1]
  float unitRight = 0.0f;  // [1]
  // Signed per-wheel travel this Move must produce. Meaningless unless
  // hasDistanceTarget; a Time Move has no distance to hit.
  float distanceLeft = 0.0f;   // [mm]
  float distanceRight = 0.0f;  // [mm]
  // Shape-space cruise: the DOMINANT wheel's own commanded speed, which is
  // what lambda is measured in.
  float cruise = 0.0f;  // [mm/s]
  float duration = 0.0f;  // [s] the time budget; Time Moves only
  bool hasDistanceTarget = false;
  bool hasTimeBudget = false;
  // False when the Move cannot be shaped at all: a zero commanded velocity,
  // a Distance stop on a motion with no net translation (a pure pivot), an
  // Angle stop on a motion with no net rotation (a straight line), or a
  // Kind::Stop (which has no shape by definition -- it is a forced ramp to
  // zero, handled outside the profiler). Callers must fail closed on it.
  bool valid = false;
};

// Decompose a Move into its constant ratio and per-wheel distance targets.
// Pure; no clamping against PlannerLimits happens here (that is
// shapeLimits()'s job) so the shape stays a property of the Move alone.
MoveShape shapeOf(const Move& move, float trackWidth);  // [mm]

// Project the body-frame ceilings in PlannerLimits onto this shape's single
// scalar axis. Both the linear and the angular ceilings constrain lambda --
// a 3:1 arc is bounded by whichever of the two binds first -- so each is
// converted to a lambda bound and the tighter one wins.
//
// Reduces exactly to the pre-existing hand-written cases: a straight
// (u = 1,1) yields {vMax, aMax, aDecel, jerkMax}; a pivot (u = -1,1) yields
// {omegaMax, alphaMax, alphaDecel, yawJerkMax} scaled by trackWidth/2.
AxisLimits shapeLimits(const MoveShape& shape, const PlannerLimits& limits);

// May a Move of shape `next` be entered at speed directly out of a Move of
// shape `current`? True only when the two ratios are identical -- same
// geometry, same direction. A reversal, a turn following a straight, or a
// change of arc radius all answer false, and the predecessor must land at
// zero. Invalid shapes always answer false.
bool shapesCompatible(const MoveShape& current, const MoveShape& next);

}  // namespace Motion
