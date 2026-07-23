// velocity_shaper.h -- Motion::VelocityShaper: the next tick's commanded
// speed for a Move approaching its own stop condition, decelerating INTO
// the goal instead of running at a constant commanded speed until
// Motion::StopCondition fires and the actuation/momentum tail overshoots
// past it.
//
// Boundary (stakeholder directive, decel-into-the-goal campaign, follow-on
// to clasi/issues/angle-stop-overshoot-61-73-percent-on-hardware.md's own
// "Option 1... remains the path to closing that residual further"): inside
// -- one pure function mapping (cruise target, last-commanded speed,
// predicted remaining distance/angle, dt, accel/decel magnitude limits) to
// the next commanded speed; outside -- WHAT the remaining distance/angle
// is measured against (App::MoveQueue's own predicted-pose reuse of the
// SAME StateEstimator::bodyAt() prediction its stop-condition anticipation
// already computes, ticket's own instruction: "reuse that prediction,
// don't compute twice"), WHICH component of a Move's velocity gets shaped
// (linear v_x vs angular omega, App::MoveQueue's own per-Move-kind
// decision), and what happens once StopCondition itself fires (unchanged
// -- MoveQueue/Drive's job, this module is never the terminal authority).
// No dependency on App::MoveQueue, App::Drive, Motion::StopCondition, or
// any msg::* wire type -- the same "pure computation, host-clean, sibling
// of stop_condition" shape stop_condition.h's own file header established
// for this directory.
//
// Core formula (stakeholder's own words, "a target velocity passed into
// some function that gives you the next maximum speed you can assign to
// the wheels"):
//
//   next = min(cruiseSpeed, sqrt(2*aDecel*max(remaining,0)), current+aMax*dt)
//
// with sign handling generalized as an "approach cruise, then clamp the
// result's MAGNITUDE to the decel-taper ceiling" two-step (see .cpp's own
// comment for why this is a strict generalization of the literal formula
// above in every regime the formula was written for, and additionally
// correct — rather than merely undefined — when currentSpeed and
// cruiseSpeed carry different signs or |current| > |cruise|, neither of
// which the literal formula alone resolves). This mirrors the archived
// BodyVelocityController's own `approach(current, target, maxStep)` idiom
// (source_old/control/BodyVelocityController.cpp,
// `_omega = approach(_omega, omegaTgtClamped, domega_max)`) generalized
// with a second, magnitude-only clamp for the decel taper.
//
// Linear (mm/s, mm/s^2) and angular (rad/s, rad/s^2) both supported by the
// SAME function -- it is unit-agnostic; the caller supplies matching
// units for cruiseSpeed/currentSpeed/dt's own speed axis and
// remaining/aMax/aDecel's own distance axis. App::MoveQueue calls this
// once with linear units for a Distance-kind Move's v_x and once with
// angular units for an Angle-kind Move's omega.
//
// Kind::Time moves (App::MoveQueue's own doc comment): "accel-limited
// ramp-up, no decel taper" -- achieved not by a second code path but by
// the CALLER passing remaining = +infinity, which makes
// sqrt(2*aDecel*remaining) diverge and never bind (see next()'s own doc
// comment below) -- one function, one formula, the "no taper" case is a
// parameter choice, not a branch.
//
// Design/rationale: DESIGN.md (this directory).
#pragma once

namespace Motion {

class VelocityShaper {
 public:
  // next -- computes the speed to command THIS tick.
  //
  // cruiseSpeed  -- [mm/s] or [rad/s], SIGNED steady-state target (the
  //   Move's own commanded magnitude and direction -- e.g. MoveTwist.omega
  //   for an Angle-kind turn, MoveTwist.v_x for a Distance-kind straight
  //   Move). The direction the robot is trying to go; never overshot.
  // currentSpeed -- [mm/s] or [rad/s], SIGNED speed STAGED last tick (the
  //   caller's own running state -- App::MoveQueue holds this across
  //   ticks, per-Move, so the shaper itself stays stateless/pure).
  // remaining    -- [mm] or [rad], predicted remaining distance/angle to
  //   the stop threshold, same sense as Motion::StopCondition's own
  //   threshold (an unsigned magnitude). Negative/NaN clamps to 0
  //   (mirrors stop_condition.cpp's own clampPositive() malformed-input
  //   posture) -- at or past the goal, the decel taper pins the next
  //   speed's magnitude to 0. Pass +infinity (or any value so large
  //   sqrt(2*aDecel*remaining) exceeds every reachable speed) to disable
  //   the decel taper entirely -- Kind::Time moves end on elapsed time,
  //   not position, so there is no "remaining" to taper against; ramp-up
  //   (the accel clamp) still applies.
  // dt           -- [s] elapsed since the previous call. Negative clamps
  //   to 0 (same malformed-input posture).
  // aMax         -- [mm/s^2] or [rad/s^2], nonnegative accel-ramp
  //   magnitude ceiling (how fast currentSpeed may approach cruiseSpeed).
  //   Negative clamps to |aMax| (a magnitude, sign-agnostic input).
  // aDecel       -- [mm/s^2] or [rad/s^2], nonnegative decel-taper
  //   magnitude ceiling (how hard the caller could stop from the current
  //   speed and land exactly at `remaining` == 0). Negative clamps to
  //   |aDecel|.
  //
  // Returns the next tick's SIGNED commanded speed, same units as
  // cruiseSpeed/currentSpeed. Pure: no state, safe to call speculatively;
  // NOT itself Motion::StopCondition -- callers still run the real stop
  // condition unchanged (this only shapes the APPROACH, never decides
  // when the Move ends).
  static float next(float cruiseSpeed, float currentSpeed, float remaining,
                     float dt, float aMax, float aDecel);
};

}  // namespace Motion
