// velocity_shaper.h -- Motion::VelocityShaper: the next tick's commanded
// speed for a Move approaching its own stop condition, decelerating INTO
// the goal instead of running at a constant commanded speed until
// Motion::StopCondition fires and the actuation/momentum tail overshoots
// past it.
//
// Boundary (stakeholder directive, decel-into-the-goal campaign, follow-on
// to clasi/issues/angle-stop-overshoot-61-73-percent-on-hardware.md's own
// "Option 1... remains the path to closing that residual further"): inside
// -- one small stateful object mapping (cruise target, predicted remaining
// distance/angle, dt, accel/decel/jerk magnitude limits) to the next
// commanded speed, carrying its own commanded-speed AND commanded-accel
// state between calls; outside -- WHAT the remaining distance/angle is
// measured against (App::MoveQueue's own predicted-pose reuse of the SAME
// StateEstimator::bodyAt() prediction its stop-condition anticipation
// already computes, ticket's own instruction: "reuse that prediction,
// don't compute twice"), WHICH component of a Move's velocity gets shaped
// (linear v_x vs angular omega, App::MoveQueue's own per-Move-kind
// decision), and what happens once StopCondition itself fires (unchanged
// -- MoveQueue/Drive's job, this module is never the terminal authority).
// No dependency on App::MoveQueue, App::Drive, Motion::StopCondition, or
// any msg::* wire type -- the same "pure computation, host-clean, sibling
// of stop_condition" shape stop_condition.h's own file header established
// for this directory (this module has internal STATE, unlike
// StopCondition's baseline-at-construction shape, but the same zero-I2C/
// zero-globals/zero-heap/host-clean posture).
//
// Jerk-limited slew (stakeholder correction, 2026-07-22, on top of this
// module's own first accel-limited pass -- and DELIBERATELY kept minimal
// per a second stakeholder correction the same day: "I literally just
// wanted acceleration slew rate limiting and velocity slew rate
// limiting," not a Ruckig-shaped profile solver). Exactly two chained
// rate clamps and an integrator, nothing more:
//
//   1. VELOCITY clamp (unchanged from this module's own first pass --
//      see .cpp's own comment): approach cruiseSpeed by at most aMax*dt,
//      then cap the result's magnitude to the decel-taper ceiling
//      sqrt(2*aDecel*max(remaining,0)) (the classic "decelerate to land
//      exactly at remaining==0" curve). This step alone is Stage 1's
//      entire algorithm, byte-for-byte.
//   2. ACCEL clamp (new): the velocity clamp's own result implies an
//      accel this tick (`(candidate-commandedSpeed_)/dt`); slew
//      commandedAccel_ toward THAT implied accel by at most jMax*dt
//      (the jerk clamp), then integrate commandedSpeed_ from the
//      jerk-clamped accel instead of jumping straight to the velocity
//      clamp's own candidate.
//
// No separate "which phase am I in" decision, no roll-off predicate, no
// stopping-distance solve beyond step 1's own one-line sqrt() (already
// exactly what Stage 1 used) -- the outer velocity target is recomputed
// fresh every tick from the CURRENT commanded speed, so it is already
// self-damping as it approaches cruise or zero; the accel clamp on top
// only smooths the RATE of that approach, it does not need its own
// overshoot-avoidance logic. See this file's own "What it is not" note on
// `next()` below and docs/protocol-v4.md §5.2's matching paragraph.
//
// Linear (mm/s, mm/s^2, mm/s^3) and angular (rad/s, rad/s^2, rad/s^3) both
// supported by the SAME class -- it is unit-agnostic; the caller
// instantiates one object per axis (App::MoveQueue holds four: v_x, omega,
// v_left, v_right) and supplies matching units for every call.
//
// Kind::Time moves (App::MoveQueue's own doc comment): "accel-limited
// ramp-up, no decel taper" -- achieved not by a second code path but by
// the CALLER passing remaining = +infinity, which makes the jerk-aware
// stopping-distance comparison never trigger the braking target (see
// next()'s own doc comment below) -- one function, one formula, the "no
// taper" case is a parameter choice, not a branch.
//
// Design/rationale: DESIGN.md (this directory).
#pragma once

namespace Motion {

class VelocityShaper {
 public:
  // next -- computes the speed to command THIS tick, from this object's
  // OWN internal (commandedSpeed_, commandedAccel_) state -- unlike this
  // module's first accel-limited pass, `next()` no longer takes a
  // `currentSpeed` parameter; the object itself IS the running state, so
  // the caller holds one persistent VelocityShaper instance per axis
  // (App::MoveQueue's own shaperVX_/shaperOmega_/shaperVLeft_/
  // shaperVRight_ members) rather than a bare float.
  //
  // cruiseSpeed  -- [mm/s] or [rad/s], SIGNED steady-state target (the
  //   Move's own commanded magnitude and direction -- e.g. MoveTwist.omega
  //   for an Angle-kind turn, MoveTwist.v_x for a Distance-kind straight
  //   Move). The direction the robot is trying to go; the commanded speed
  //   never overshoots it (see the accel-target rule, .cpp's own comment).
  // remaining    -- [mm] or [rad], predicted remaining distance/angle to
  //   the stop threshold, same sense as Motion::StopCondition's own
  //   threshold (an unsigned magnitude). Negative/NaN clamps to 0
  //   (mirrors stop_condition.cpp's own clampPositive() malformed-input
  //   posture). Pass +infinity (or any value large enough the decel-taper
  //   ceiling never binds) to disable the decel taper entirely --
  //   Kind::Time moves end on elapsed time, not position, so there is no
  //   "remaining" to taper against; ramp-up (the accel/jerk clamps) still
  //   applies.
  // dt           -- [s] elapsed since the previous call. Negative clamps
  //   to 0 (same malformed-input posture).
  // aMax         -- [mm/s^2] or [rad/s^2], nonnegative accel-ramp
  //   magnitude ceiling (how fast the velocity clamp may push the
  //   commanded speed toward cruiseSpeed). Negative clamps to |aMax|.
  // aDecel       -- [mm/s^2] or [rad/s^2], nonnegative decel-taper
  //   magnitude ceiling for the velocity clamp's own `sqrt(2*aDecel*
  //   remaining)` cap (step 1, above). Negative clamps to |aDecel|.
  // jMax         -- [mm/s^3] or [rad/s^3], nonnegative jerk magnitude
  //   ceiling -- how fast the COMMANDED ACCEL itself may change per
  //   second (step 2, above). Negative clamps to |jMax|. jMax <= 0
  //   (including NaN) degrades to an UNLIMITED jerk (the commanded accel
  //   snaps directly to the velocity clamp's own implied accel every
  //   tick) -- this module's own first accel-limited-only behavior --
  //   rather than dividing by zero; App::MoveQueue's own ShaperLimits
  //   never actually calls this with jMax <= 0 in practice (its own
  //   "0 == axis disabled" gate keeps the caller from reaching this
  //   function at all on that axis), but this function stays well-defined
  //   for a caller that does.
  //
  // Returns the next tick's SIGNED commanded speed (also cached as
  // commandedSpeed(), below), same units as cruiseSpeed. Also updates
  // commandedAccel() -- the running commanded-acceleration state the next
  // call's jerk slew starts from. NOT itself Motion::StopCondition --
  // callers still run the real stop condition unchanged (this only shapes
  // the approach, never decides when a Move ends).
  //
  // What it is not: a jerk-limited S-curve computed ONCE ahead of time
  // with a known total duration/arrival time (a real, Ruckig-style
  // time-optimal profile) -- this is a per-tick REACTIVE law (two chained
  // rate clamps, deliberately nothing fancier — see this file's own
  // header) with no global time-optimality guarantee and no cross-axis
  // coordination (each axis instance is shaped independently). See
  // DESIGN.md's own "Open Questions" section.
  float next(float cruiseSpeed, float remaining, float dt, float aMax,
             float aDecel, float jMax);

  // reset -- zeroes BOTH internal state values. Used whenever the robot
  // has genuinely stopped (App::MoveQueue's own Drive::stop()/flush()
  // call sites) so the NEXT Move's own ramp starts from a true (0 speed,
  // 0 accel), not a stale nonzero pair left over from a taper that never
  // finished (e.g. the owning Move ended via the timeout backstop
  // mid-taper).
  void reset() {
    commandedSpeed_ = 0.0f;
    commandedAccel_ = 0.0f;
  }

  // syncTo -- sets commandedSpeed() directly (commandedAccel() resets to
  // 0) with NO shaping math -- used by App::MoveQueue::activate() when
  // ShaperLimits disables this axis, keeping this object's own state
  // consistent with whatever raw value was actually staged onto Drive, so
  // a LATER live-enable of shaping on this axis doesn't inherit a stale
  // value from before it was turned off.
  void syncTo(float speed) {
    commandedSpeed_ = speed;
    commandedAccel_ = 0.0f;
  }

  float commandedSpeed() const { return commandedSpeed_; }  // [mm/s] or [rad/s]
  float commandedAccel() const { return commandedAccel_; }  // [mm/s^2] or [rad/s^2]

 private:
  float commandedSpeed_ = 0.0f;
  float commandedAccel_ = 0.0f;
};

}  // namespace Motion
