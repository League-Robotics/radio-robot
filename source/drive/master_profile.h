// master_profile.h -- Drive::MasterProfile: a single-channel, jerk-limited
// motion planner wrapping one vendored ruckig::Ruckig<1>/ruckig::Trajectory
// <1> pair (libraries/ruckig). One instance plans and samples ONE 1-DoF
// master channel -- Drive::MotionPlan (ticket 100-003) holds one, since v2
// solves exactly ONE trajectory per segment (path length for arcs, heading
// for pivots -- the issue's "one trajectory per segment" rule; the second
// channel is always DERIVED, omega = kappa * v, never independently
// solved).
//
// This is a hand-port of motion/jerk_trajectory.h's Motion::JerkTrajectory
// (that file stays untouched -- source/drive/'s "copy, don't include"
// rule), GENERALIZED per architecture-update.md (100) M2: jerk_trajectory's
// two solve modes (solveToRest/solveToVelocity) collapse into one
// position-control primitive with an explicit exit velocity --
// solveToExit(targetPosition, exitVelocity, maxVelocity) -- of which
// solveToRest(targetPosition, maxVelocity) is the exitVelocity == 0.0f
// special case (see solveToRest()'s own doc comment below: it is
// implemented as solveToExit(targetPosition, 0.0f, maxVelocity), so the two
// are provably the same solve). solveToVelocity() (velocity-control,
// open-ended, no target position) is unchanged from jerk_trajectory.h's own
// pattern -- still needed for cruise ramp-up and MOVER teleop
// (Drivetrain::planVelocity(), ticket 100-007).
//
// This class knows nothing about Goal kinds, wire verbs, CODAL, Subsystems,
// or any msg::* type -- source/drive/'s isolation boundary (SUC-008,
// enforced by the grep test alongside this file). ruckig:: types are a
// private implementation detail, declared and used only inside this
// header/.cpp pair (both private members below) -- the PUBLIC API uses only
// float/State, never a ruckig:: type. No heap allocation: Ruckig<1>/
// Trajectory<1> are ordinary, stack-based class members (compile-time DoF,
// std::array-backed), the same viability jerk_trajectory.h's own class
// comment documents for the SAME vendored library under this firmware's
// exact flags (gnu++20 -fno-exceptions -fno-rtti).
//
// -- Two solve modes, one generalized (Decision 2's own precedent, carried
// forward) --
// solveToExit()/solveToRest(): position-control, solve to a known target
// position, arriving at a (possibly nonzero) exit velocity. Used once, at
// segment start, for every Goal (arcs AND pivots share this one call
// shape -- a pivot's "position" is heading, a straight/arc's is path
// length); a pivot's exitSpeed is always 0.0f (Drivetrain::admit()'s
// PIVOT_NONZERO_EXIT verdict enforces this OUTSIDE this class, per the
// class's own "knows nothing about goal kinds" boundary). solveToVelocity():
// velocity-control, solve to a target velocity, open-ended (no target
// position). Used for cruise ramp-up and MOVER's deadman-velocity teleop.
// Both take maxVelocity as a PER-CALL argument: the caller passes its own
// ceiling (Drivetrain::plan()'s folded v_eff); configure()'s velocity
// ceiling (this channel's own ProfileLimits.velocity) still applies
// underneath as an OUTER clamp, never bypassed, but the caller's own
// per-call value is what normally binds.
//
// -- Sampling --
// sample() evaluates Ruckig's own Trajectory::at_time(), including its
// built-in past-duration "hold at final state" extrapolation (jerk_
// trajectory.h's own confirmed behavior, unchanged here) -- this IS "cruise
// sustain" for a velocity-control solve, "stay at rest" for a position-
// control solve with exitVelocity == 0, and "hold at the exit velocity" for
// a position-control solve with a nonzero exitVelocity; no separate
// Drive::-side bookkeeping is added or needed here.
//
// -- Seeding contract (preserved VERBATIM from jerk_trajectory.h's Decision
// 8, load-bearing) --
// Every solveToRest()/solveToExit()/solveToVelocity() call reads current_
// {position,velocity,acceleration} back from this channel's own remembered
// LAST SAMPLE (reset()/seedCurrent()/the previous solve's seed/the previous
// sample() call) -- NEVER from a measured observation. This is a
// deliberate, load-bearing boundary: an earlier version of this codebase's
// own stop-anticipation cap fed measured wheel speed into a similar formula
// and produced a traced limit-cycle oscillation (087-009); seeding
// exclusively from this class's own remembered state (a pure function of
// its own prior output) cannot reproduce that failure mode. Do not add a
// measured-velocity seed path to any solve here. (jerk_trajectory.h's
// retarget()/reanchor() -- two narrow, guarded exceptions to this contract
// for the OLD dead-time-projected divergence replan -- have NO equivalent
// here: v2 has no dead-time compensation in the control law at all
// (kOutputHops/kDeadTime do not exist in v2, enforced by the isolation grep
// test); Drivetrain::replan() (ticket 100-003) re-times the SAME anchored
// path by calling seedCurrent() with the measured state -- an explicit,
// caller-supplied seed, exactly like jerk_trajectory.h's own seedCurrent()
// -- then solveToExit() again to the same goal/exitSpeed, never a
// dedicated retarget/reanchor entry point.)
//
// -- Jerk sentinel (preserved verbatim from jerk_trajectory.h's Decision 6)
// --
// configure()'s ProfileLimits.jerk == 0.0f (the same "off -- trapezoid, no
// S-curve" sentinel PlannerConfig's j_max/yaw_jerk_max use) maps to
// Ruckig's own max_jerk = +infinity -- the library's OWN out-of-the-box
// default, not a new convention. A positive value passes straight through,
// immediately producing a genuinely jerk-limited S-curve.
//
// -- Directional no-reversal band, generalized to a same-sign band
// (architecture-update.md (100) M2) --
// jerk_trajectory.h's solveToRest() sets Ruckig's own asymmetric velocity
// limit (InputParameter::min_velocity, input_parameter.hpp) so a position
// solve can never plan motion AGAINST its own direction of travel:
// min_velocity = 0 for a positive-direction solve (max_velocity = 0 for a
// negative one). Because target_velocity was always 0 there, 0 trivially
// sat inside that band. solveToExit() reuses the EXACT SAME band
// construction -- unchanged -- and simply passes exitVelocity as
// Ruckig's own InputParameter::target_velocity instead of a hardcoded 0.0.
// Ruckig's own input validation (input_parameter.hpp's validate(),
// checked with check_target_state_within_limits = true at every calculate()
// call, confirmed by reading libraries/ruckig/include/ruckig/input_
// parameter.hpp and ruckig.hpp directly) already requires min_velocity <=
// target_velocity <= max_velocity -- so the SAME band that forbade
// reversal for solveToRest's implicit 0 now ALSO enforces "exitVelocity
// must have the same sign as (or be exactly) the direction of travel, and
// |exitVelocity| <= maxVelocity" for solveToExit, entirely as a
// CONSTRAINT OF THE SOLVE: an exitVelocity outside the band or of the
// wrong sign makes calculate() return a non-Working Result (Ruckig<1>'s
// default throw_error = false, confirmed by reading ruckig.hpp -- no
// exception, no UB, just a clean false return), never UB and never a
// silent wrong answer. This IS the "directional no-reversal band
// generalizes to a same-sign band for a nonzero exit speed" the ticket
// describes: no new band-construction code was needed, only routing
// exitVelocity into the field that used to be a hardcoded 0.0.
//
// -- Direction-mirrored linear-channel acceleration bounds (preserved
// verbatim from jerk_trajectory.h's Open Question 2) --
// Ruckig's max_acceleration/min_acceleration are absolute-signed, not
// direction-of-travel-relative the way ProfileLimits.accel (accelerating)/
// .decel (decelerating) are. For a positive-direction solve,
// max_acceleration = accel, min_acceleration = -decel; for a
// negative-direction solve the mapping mirrors: max_acceleration = decel,
// min_acceleration = -accel. "Direction" is computed purely from the
// solve's own target vs. current position (a math fact, not goal-kind
// knowledge) -- never from any caller-supplied "which way am I going"
// flag. A symmetric (rotational) channel needs no such mirroring: the
// caller configures it with accel == decel, which collapses the mirroring
// formula to an identical result regardless of direction -- no separate
// rotational branch anywhere in this class (unlike jerk_trajectory.h,
// which needed an explicit isRotational bool at configure() time; here
// ProfileLimits already carries the right values for whichever channel the
// caller is configuring).
#pragma once

#include "drive/types.h"
#include "ruckig/ruckig.hpp"

namespace Drive {

class MasterProfile {
 public:
  // State -- a sampled kinematic point, returned by sample()/peek().
  struct State {
    float position = 0.0f;      // [mm] or [rad] -- this channel's own frame
    float velocity = 0.0f;      // [mm/s] or [rad/s] signed
    float acceleration = 0.0f;  // [mm/s^2] or [rad/s^2] signed
  };

  // configure -- store this channel's motion limits. Must be called before
  // the first solve. See the class comment for the jerk-sentinel mapping
  // and the symmetric-channel note.
  void configure(const ProfileLimits& limits);

  // reset -- zero the remembered last-sample state (position, velocity,
  // acceleration all 0). Call before the first solve of a fresh channel
  // that starts from rest.
  void reset();

  // seedCurrent -- set the remembered last-sample state directly, without
  // solving. Use instead of reset() when a fresh solve's seed must start
  // from a known nonzero state (e.g. Drivetrain::replan()'s re-timing from
  // the measured state -- see the class comment's seeding-contract note on
  // why this is the caller's explicit, documented mechanism rather than a
  // dedicated retarget()/reanchor() pair).
  void seedCurrent(float position, float velocity, float acceleration);

  // solveToRest -- the exitVelocity == 0.0f special case of solveToExit()
  // below; implemented as exactly that call, so the two are provably the
  // same solve (test: solveToRest(p, v) and solveToExit(p, 0.0f, v) start
  // from the same seed produce bit-identical trajectories).
  bool solveToRest(float targetPosition, float maxVelocity);

  // solveToExit -- position-control solve: (position, velocity,
  // acceleration) -> (targetPosition, exitVelocity, 0). Seeded from this
  // channel's own remembered last sample (class comment's seeding
  // contract) -- never from a measured observation. Valid iff
  // |exitVelocity| <= maxVelocity AND exitVelocity has the same sign as
  // (targetPosition - currentPosition) (the same-sign band, class
  // comment) -- an invalid combination returns false cleanly, never UB.
  // maxVelocity is THIS CALL's own ceiling; configure()'s ProfileLimits.
  // velocity ceiling still applies underneath as an outer clamp.
  bool solveToExit(float targetPosition, float exitVelocity, float maxVelocity);

  // solveToVelocity -- velocity-control solve-to-a-velocity, open-ended
  // (no target position): (velocity, acceleration) -> targetVelocity. Used
  // for cruise ramp-up and, with targetVelocity == 0, an open-ended decel
  // (e.g. MOVER's deadman release). Same seeding/maxVelocity contract as
  // solveToExit().
  bool solveToVelocity(float targetVelocity, float maxVelocity);

  // sample -- evaluate the current trajectory at an elapsed time since it
  // was solved (Ruckig's own Trajectory::at_time(), including its built-in
  // past-duration hold-at-final-state extrapolation -- see the class
  // comment). Updates the remembered last-sample state used to seed the
  // next solve.
  State sample(float elapsed);  // [s]

  // peek -- evaluate the trajectory at an arbitrary elapsed time WITHOUT
  // updating the remembered last-sample seed state (a const, pure read;
  // sample() above is the stateful twin). For consumers that need the plan
  // at a time OTHER than "now" without disturbing the next solve's seed.
  State peek(float elapsed) const;  // [s]

  // duration -- this channel's currently held trajectory's total duration
  // (Ruckig's own Trajectory::get_duration()). 0 before the first
  // successful solve.
  float duration() const;  // [s]

 private:
  // calculated_ -- true once otg_.calculate() has succeeded at least once
  // for this instance. Same defensive UB guard as jerk_trajectory.h's own
  // calculated_ (2026-07-10 fix, class comment there): a default-
  // constructed ruckig::Trajectory<1>'s `profiles` array holds
  // uninitialized std::array members, so reading it via sample()/at_time()
  // before any calculate() call is undefined behavior. sample()/duration()
  // both check this flag and return a safe zero State{}/0.0f instead of
  // touching traj_ when it is still false.
  bool calculated_ = false;

  // accelBoundsForDirection -- the direction-mirrored max_acceleration/
  // min_acceleration mapping (class comment). Positive direction:
  // (accel_, -decel_); negative direction: (decel_, -accel_).
  void accelBoundsForDirection(bool positiveDirection, float* maxAccel, float* minAccel) const;

  // solvePositionControl -- shared implementation behind solveToRest() and
  // solveToExit() -- both are the SAME position-control solve, differing
  // only in what target velocity is passed in (0.0f for solveToRest()).
  bool solvePositionControl(float targetPosition, float currentPosition, float currentVelocity,
                             float currentAcceleration, float exitVelocity, float maxVelocity);

  // ---- Config (configure()) ----
  float accel_ = 0.0f;            // [mm/s^2] or [rad/s^2] accelerating-direction bound
  float decel_ = 0.0f;            // [mm/s^2] or [rad/s^2] decelerating-direction bound (magnitude)
  float maxJerk_ = 0.0f;          // [mm/s^3] or [rad/s^3] mapped jerk (0 sentinel -> +infinity)
  float velocityCeiling_ = 0.0f;  // [mm/s] or [rad/s] outer clamp (ProfileLimits.velocity)

  // ---- Remembered last-sample state (the seeding contract) ----
  float lastPosition_ = 0.0f;      // [mm] or [rad] -- this channel's own frame
  float lastVelocity_ = 0.0f;      // [mm/s] or [rad/s] signed
  float lastAcceleration_ = 0.0f;  // [mm/s^2] or [rad/s^2] signed

  ruckig::Ruckig<1> otg_;
  ruckig::Trajectory<1> traj_;
};

}  // namespace Drive
