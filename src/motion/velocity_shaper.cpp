// velocity_shaper.cpp -- Motion::VelocityShaper implementation. See
// velocity_shaper.h's file header for the module's narrow boundary and the
// two chained rate clamps this implements (deliberately nothing fancier --
// see that file's own header for the stakeholder's own scope correction).
#include "motion/velocity_shaper.h"

#include <cmath>

namespace Motion {

namespace {

// Malformed-input-safety clamps -- same posture as stop_condition.cpp's own
// clampPositive() (NaN compares false against > 0, so NaN clamps to 0 here
// too, for exactly the same reason).
float clampNonNegative(float value) { return (value > 0.0f) ? value : 0.0f; }

float signOf(float value) {
  if (value > 0.0f) return 1.0f;
  if (value < 0.0f) return -1.0f;
  return 0.0f;
}

}  // namespace

// next() -- two chained rate clamps and an integrator, plus ONE algebraic
// margin term (not a branch, not a phase) folded into clamp 1's own input.
// Nothing else.
//
//   Margin (the "plain conservative margin for the jerk transition" a
//   target computation needs, per this module's own file header): where
//   commandedSpeed_ will END UP if commandedAccel_ decays to 0 under the
//   jerk limit starting THIS tick -- `commandedAccel_^2/(2*jMax)`, signed
//   by commandedAccel_'s own sign. This is the velocity-domain sibling of
//   the classic `v^2/(2*a)` stopping-distance term (accel stands in for
//   velocity, jerk stands in for decel) -- a single algebraic expression,
//   the SAME "one-line formula" shape this file's own header allows for
//   the decel-into-goal target. `predictedSpeed` (commandedSpeed_ plus
//   this margin) feeds clamp 1 below INSTEAD of raw commandedSpeed_, so
//   clamp 1 already "sees" the accel's own momentum and starts easing its
//   own demand early -- without any separate phase decision.
//
//   Clamp 1 (VELOCITY, otherwise unchanged from this module's own first
//   accel-limited pass): approach cruiseSpeed by at most aMax*dt from
//   `predictedSpeed`, then cap the result's magnitude to the decel-taper
//   ceiling sqrt(2*aDecel*max(remaining,0)) -- the textbook "decelerate
//   to land exactly at remaining==0" curve. Produces `candidate`.
//
//   Clamp 2 (ACCEL): `candidate` implies an acceleration this tick --
//   `(candidate - predictedSpeed) / dt` -- slew commandedAccel_ toward
//   THAT implied accel by at most jMax*dt (the jerk clamp itself).
//
//   Integrator: commandedSpeed_ += commandedAccel_ * dt, using the REAL
//   running state and the just-slewed (jerk-clamped) accel -- never
//   `candidate`/`predictedSpeed` directly, both of which are look-ahead
//   quantities for clamp 1/clamp 2's own decisions, not the actual state.
float VelocityShaper::next(float cruiseSpeed, float remaining, float dt, float aMax,
                            float aDecel, float jMax) {
  float dtClamped = clampNonNegative(dt);
  float aMaxMag = std::fabs(aMax);
  float aDecelMag = std::fabs(aDecel);
  float jMaxMag = std::fabs(jMax);
  float remainingClamped = clampNonNegative(remaining);

  // Margin -- see this function's own comment above. jMaxMag <= 0 (the
  // "unlimited jerk" degenerate case) drops the margin to 0, matching
  // this module's own first-pass, margin-free behavior.
  float momentum = (jMaxMag > 0.0f) ? (commandedAccel_ * std::fabs(commandedAccel_)) /
                                           (2.0f * jMaxMag)
                                     : 0.0f;
  float predictedSpeed = commandedSpeed_ + momentum;

  // Clamp 1, step a -- accel-ramp clamp: approach cruiseSpeed by at most
  // aMaxMag*dt, from predictedSpeed (not raw commandedSpeed_).
  float accelStepMax = aMaxMag * dtClamped;
  float delta = cruiseSpeed - predictedSpeed;
  float accelStep = (delta > accelStepMax)    ? accelStepMax
                     : (delta < -accelStepMax) ? -accelStepMax
                                                : delta;
  float candidate = predictedSpeed + accelStep;

  // Clamp 1, step b -- decel-taper clamp: cap |candidate| to the
  // distance/angle-remaining-derived ceiling, `sqrt(2*aDecelMag*
  // effectiveRemaining)` -- Stage 1's own one-line closed form, `remaining`
  // replaced by `effectiveRemaining` (remaining, less a one-line jerk
  // margin: `|commandedSpeed_|*aDecelMag/(2*jMaxMag)`, the extra distance
  // the jerk-limited ramp-down of the deceleration itself consumes -- the
  // "plain conservative margin for the jerk transition" this module's own
  // file header calls for, the SAME two-term-closed-form shape as this
  // module's own jerk-aware-stopping-distance derivation, folded into
  // `remaining` instead of compared against a separate stopping-distance
  // value). Without this margin, the ceiling stays at Stage 1's own
  // undiminished sqrt() value right up until remaining hits exactly 0,
  // then SNAPS to 0 -- too late for the still-in-flight commandedAccel_ to
  // decay in time, so the commanded speed overshoots past 0 (reverses
  // sign) before the jerk clamp can catch up. sqrt(2*aDecelMag*
  // effectiveRemaining) still diverges toward +infinity as remainingClamped
  // does, so remaining=+infinity (Kind::Time -- no decel taper) never
  // binds this branch, matching velocity_shaper.h's own documented
  // contract.
  float jerkMargin = (jMaxMag > 0.0f)
                          ? (std::fabs(commandedSpeed_) * aDecelMag) / (2.0f * jMaxMag)
                          : 0.0f;
  float effectiveRemaining = remainingClamped - jerkMargin;
  if (effectiveRemaining < 0.0f) effectiveRemaining = 0.0f;
  float decelCapMag = std::sqrt(2.0f * aDecelMag * effectiveRemaining);
  if (std::fabs(candidate) > decelCapMag) {
    candidate = signOf(cruiseSpeed) * decelCapMag;
  }

  // Clamp 2 -- jerk clamp: slew commandedAccel_ toward the accel
  // `candidate` implies (relative to predictedSpeed, the SAME basis
  // clamp 1 used), by at most jMaxMag*dt. jMaxMag <= 0 means "unlimited
  // jerk" -- snap directly to the implied accel (this module's own
  // first-pass, accel-limited-only behavior).
  float impliedAccel = (dtClamped > 0.0f) ? (candidate - predictedSpeed) / dtClamped : 0.0f;
  if (jMaxMag <= 0.0f) {
    commandedAccel_ = impliedAccel;
  } else {
    float jerkStepMax = jMaxMag * dtClamped;
    float accelDelta = impliedAccel - commandedAccel_;
    float accelStepJerk = (accelDelta > jerkStepMax)    ? jerkStepMax
                           : (accelDelta < -jerkStepMax) ? -jerkStepMax
                                                          : accelDelta;
    commandedAccel_ += accelStepJerk;
  }

  // Integrator -- advance the REAL commandedSpeed_ from the just-slewed
  // accel.
  commandedSpeed_ += commandedAccel_ * dtClamped;

  return commandedSpeed_;
}

}  // namespace Motion
