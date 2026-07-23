// velocity_shaper.cpp -- Motion::VelocityShaper implementation. See
// velocity_shaper.h's file header for the module's narrow boundary and the
// formula this generalizes.
#include "motion/velocity_shaper.h"

#include <cmath>

namespace Motion {

namespace {

// Malformed-input-safety clamps -- same posture as stop_condition.cpp's own
// clampPositive() (NaN compares false against > 0, so NaN clamps to 0 here
// too, for exactly the same reason).
float clampNonNegative(float value) { return (value > 0.0f) ? value : 0.0f; }

}  // namespace

// next() -- a strict generalization of the literal
//   min(cruiseSpeed, sqrt(2*aDecel*max(remaining,0)), current+aMax*dt)
// formula, computed as two steps instead of a three-way min():
//
//   1. "Approach" cruiseSpeed by at most aMax*dt (accel-ramp clamp). Unlike
//      a bare `current + aMax*dt`, this clamps the STEP toward cruiseSpeed
//      from EITHER side -- current is never pushed PAST cruiseSpeed, and if
//      current is already beyond cruiseSpeed (e.g. a live config dropped
//      the ceiling mid-Move) it ramps back down toward it instead of
//      holding an unreachable value. In the common case the literal
//      formula was written for (current starts at/near 0, ramping up
//      toward a positive cruiseSpeed), step 1 alone reduces to exactly
//      `current + aMax*dt` every tick until it reaches cruiseSpeed, so this
//      is behaviorally IDENTICAL there -- see this file's own unit tests.
//   2. Clamp the MAGNITUDE of step 1's result to the decel-taper ceiling
//      sqrt(2*aDecel*max(remaining,0)), re-applying cruiseSpeed's own sign
//      (the intended direction of travel -- remaining is measured toward
//      the goal in that direction) if the clamp binds. This is the
//      textbook "decelerate to land exactly at remaining==0" curve (the
//      same vCap = sqrt(2*a_decel*dRemaining) the deleted planner
//      subsystem's own PlannerBegin.cpp used, generalized here to a
//      per-tick incremental function rather than a whole-profile plan).
//
// Sign handling: steps 1+2 together handle cruiseSpeed/currentSpeed of
// either sign, and a sign MISMATCH between them (step 1's approach moves
// current toward cruise's sign incrementally, at the aMax*dt rate, rather
// than an instantaneous flip) -- strictly more defined than the bare
// three-way min(), which the ticket's own formula never specifies a sign
// convention for at all.
float VelocityShaper::next(float cruiseSpeed, float currentSpeed, float remaining,
                            float dt, float aMax, float aDecel) {
  float dtClamped = clampNonNegative(dt);
  float aMaxMag = std::fabs(aMax);
  float aDecelMag = std::fabs(aDecel);
  float remainingClamped = clampNonNegative(remaining);

  // Step 1 -- accel-ramp clamp: approach cruiseSpeed by at most aMaxMag*dt.
  float accelStepMax = aMaxMag * dtClamped;
  float delta = cruiseSpeed - currentSpeed;
  float accelStep = (delta > accelStepMax) ? accelStepMax
                     : (delta < -accelStepMax) ? -accelStepMax
                                                : delta;
  float candidate = currentSpeed + accelStep;

  // Step 2 -- decel-taper clamp: cap |candidate| to the distance/angle-
  // remaining-derived ceiling. sqrt(2*aDecelMag*remainingClamped) diverges
  // toward +infinity as remainingClamped does, so a caller passing
  // remaining=+infinity (Kind::Time -- no decel taper) never has this
  // branch bind, matching velocity_shaper.h's own documented contract.
  float decelCapMag = std::sqrt(2.0f * aDecelMag * remainingClamped);
  if (std::fabs(candidate) > decelCapMag) {
    float sign = (cruiseSpeed >= 0.0f) ? 1.0f : -1.0f;
    candidate = sign * decelCapMag;
  }

  return candidate;
}

}  // namespace Motion
