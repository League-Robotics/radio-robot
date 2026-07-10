// jerk_trajectory.h -- Motion::JerkTrajectory: a single-channel,
// jerk-limited motion planner wrapping one vendored ruckig::Ruckig<1>/
// ruckig::Trajectory<1> pair (libraries/ruckig). One instance plans and
// samples ONE 1-DoF channel; Subsystems::Planner (tickets 003-005) holds
// two -- linear and rotational (architecture-update.md (089) Decision 1).
//
// Mirrors Motion::VelocityRamp's existing boundary discipline (084): this
// class knows nothing about goal kinds, wire verbs, CODAL, Subsystems, or
// any msg::* type beyond msg::PlannerConfig (configure()'s own input).
// ruckig:: types are a private implementation detail, declared and used
// only inside this header/.cpp pair (both private members below) -- the
// PUBLIC API below (every method signature) uses only float/State, never a
// ruckig:: type, so nothing calling this class ever needs to name one. No
// heap allocation: Ruckig<1>/Trajectory<1> are ordinary, stack-based class
// members (compile-time DoF, std::array-backed) -- the same viability
// ruckig_smoke_harness.cpp already proved under this firmware's exact
// flags (gnu++20 -fno-exceptions -fno-rtti).
//
// -- Two solve modes (architecture-update.md (089) Decision 2) --
// solveToRest(): position-control, solve-to-rest-at-a-known-target. Used
// once, at goal start, for DISTANCE/TURN/ROTATION (and by retarget()/
// reanchor() below, the SAME mode re-invoked). solveToVelocity():
// velocity-control, solve-to-a-target-velocity, open-ended (no target
// position). Used for cruise ramp-up (TIMED/VELOCITY/STREAM) AND, with
// targetVelocity == 0, for every stop-triggered terminal decel across
// every migrated goal kind. Both take maxVelocity as a PER-CALL argument
// (Decision 2's own revision): the caller passes its own ceiling (e.g.
// min(commandedSpeed, v_body_max)) rather than relying on a single static
// value read once by configure() -- configure()'s velocity ceiling
// (v_body_max/yaw_rate_max) is still applied underneath as an OUTER clamp,
// never bypassed, but the caller's own per-call value is what normally
// binds.
//
// -- Sampling (Decision 2) --
// sample() evaluates Ruckig's own Trajectory::at_time(), including its
// built-in past-duration "hold at final state" extrapolation (confirmed in
// trajectory.hpp's state_to_integrate_from()) -- this IS "cruise sustain"
// for a velocity-control solve and "stay at rest" for a position-control
// one; no separate Planner-side bookkeeping is added or needed here.
//
// -- Seeding contract (Decision 8) --
// Every solveToRest()/solveToVelocity() call reads current_{position,
// velocity,acceleration} back from this channel's own remembered LAST
// SAMPLE (reset()/seedCurrent()/the previous solve's seed/the previous
// sample() call) -- NEVER from a measured observation (leftObs/rightObs).
// This is a deliberate, load-bearing boundary: an earlier version of this
// codebase's own stop-anticipation cap fed measured wheel speed into a
// similar formula and produced a traced limit-cycle oscillation
// (087-009); seeding exclusively from this class's own remembered state
// (a pure function of its own prior output) cannot reproduce that failure
// mode. Do not add a measured-velocity seed path to solveToRest()/
// solveToVelocity() -- see retarget()/reanchor() below for the two
// narrow, guarded exceptions Decision 10 introduces instead.
//
// -- retarget()/reanchor() (Decision 10) --
// Two divergence-triggered re-solve entry points added for Planner's
// (tickets 003/005) divergence-triggered replan. Both are still exactly
// the position-control solve-to-rest MODE above, invoked a third time
// (not a third mode): retarget() re-baselines this channel's position
// frame to 0 and solves to an externally supplied new remaining, seeded
// from this channel's OWN remembered velocity/acceleration (never the
// position) -- the NORMAL divergence case. reanchor() re-solves to the
// SAME target this channel was already tracking, seeded from the
// CALLER-SUPPLIED position/velocity with acceleration forced to 0 instead
// of this channel's own remembered state -- the GROSS divergence case, a
// deliberate, narrow exception to the seeding contract above, accepted
// because past the gross-divergence threshold the channel's own
// remembered state is known to be wrong. Both entry points still just
// solve whatever they are told to solve: **the never-solves-backward
// guard, the divergence thresholds, and the replan rate limit are ALL
// enforced by the CALLER (Planner), not here.** retarget()/reanchor() do
// NOT validate that their new target is ahead of the seed in the
// commanded direction -- calling either with a backward-pointing target
// is defined behavior (it solves backward, possibly reversing), and it is
// the caller's job to never do that (architecture-update.md (089)
// Decision 10). Adding such a guard here would require this class to
// track state (commanded sign, live direction) it otherwise has no reason
// to hold, breaking its "knows nothing about goal kinds" boundary.
//
// -- Jerk sentinel (Decision 6) --
// configure()'s j_max/yaw_jerk_max == 0.0f (the existing PlannerConfig
// "off -- trapezoid, no S-curve" sentinel) maps to Ruckig's own
// max_jerk = +infinity -- the library's OWN out-of-the-box default, not a
// new convention. A positive value passes straight through, immediately
// producing a genuinely jerk-limited S-curve.
//
// -- Direction-mirrored linear-channel acceleration bounds (Open Q2) --
// Ruckig's max_acceleration/min_acceleration are absolute-signed, not
// direction-of-travel-relative the way a_max (accelerating)/a_decel
// (decelerating) are. For a positive-direction solve, max_acceleration =
// a_max, min_acceleration = -a_decel; for a negative-direction solve the
// mapping mirrors: max_acceleration = a_decel, min_acceleration = -a_max.
// "Direction" is computed purely from the solve's own target vs. current
// position (a math fact, not goal-kind knowledge) -- never from any
// caller-supplied "which way am I going" flag. The rotational channel
// needs no such mirroring (yaw_acc_max is already symmetric): configure()
// stores the SAME value as both the accelerating and decelerating bound,
// so the shared mirroring formula collapses to an identical result
// regardless of direction with no separate rotational branch.
#pragma once

#include "messages/planner.h"
#include "ruckig/ruckig.hpp"

namespace Motion {

class JerkTrajectory {
 public:
  // State -- a sampled kinematic point, returned by sample().
  struct State {
    float position = 0.0f;      // [mm] or [rad] -- this channel's own frame
    float velocity = 0.0f;      // [mm/s] or [rad/s] signed
    float acceleration = 0.0f;  // [mm/s^2] or [rad/s^2] signed
  };

  // configure -- store this channel's motion limits from PlannerConfig.
  // isRotational selects yaw_rate_max/yaw_acc_max/yaw_jerk_max (rotational
  // channel) instead of a_max/a_decel/v_body_max/j_max (linear channel).
  // Must be called before the first solve. See the class comment for the
  // jerk-sentinel mapping and the rotational channel's symmetric bound.
  void configure(const msg::PlannerConfig& config, bool isRotational);

  // reset -- zero the remembered last-sample state (position, velocity,
  // acceleration all 0). Call before the first solveToRest()/
  // solveToVelocity() of a fresh goal that starts from rest.
  void reset();

  // seedCurrent -- set the remembered last-sample state directly, without
  // solving. Use instead of reset() when a fresh goal's first solve must
  // start from a known nonzero state (mirrors VelocityRamp::seedCurrent()).
  void seedCurrent(float position, float velocity, float acceleration);

  // solveToRest -- position-control solve-to-rest: (position, velocity,
  // acceleration) -> (targetPosition, 0, 0). Seeded from this channel's
  // own remembered last sample (see class comment's seeding contract) --
  // never from a measured observation. maxVelocity is THIS CALL's own
  // ceiling; configure()'s global ceiling still applies underneath as an
  // outer clamp. Returns false if Ruckig could not solve the input.
  bool solveToRest(float targetPosition, float maxVelocity);

  // solveToVelocity -- velocity-control solve-to-a-velocity, open-ended
  // (no target position): (velocity, acceleration) -> targetVelocity. Used
  // for cruise ramp-up and, with targetVelocity == 0, every stop-triggered
  // terminal decel. Same seeding/maxVelocity contract as solveToRest().
  bool solveToVelocity(float targetVelocity, float maxVelocity);

  // retarget -- divergence-triggered re-solve, NORMAL case (Decision 10).
  // Re-baselines this channel's position frame to 0 and re-solves
  // position-control-to-rest to newRemaining (an externally supplied,
  // e.g. dead-time-projected, target), seeded from this channel's OWN
  // remembered velocity/acceleration -- never the position, and never a
  // measured observation. Reuses the maxVelocity ceiling from the most
  // recent solveToRest()/retarget() call. See the class comment: this does
  // NOT validate that newRemaining is ahead of the seed -- that guard is
  // the caller's (Planner's) responsibility.
  bool retarget(float newRemaining);

  // reanchor -- divergence-triggered re-solve, GROSS case (Decision 10).
  // Re-solves position-control-to-rest to the SAME target this channel was
  // already tracking (the target from the most recent solveToRest()/
  // retarget() call), seeded from the CALLER-SUPPLIED position/velocity
  // with acceleration forced to 0 -- a deliberate exception to the normal
  // seeding contract, since past the gross-divergence threshold this
  // channel's own remembered state is known to be wrong. Accepts a
  // velocity discontinuity at the reseed by design. Same
  // caller-responsibility note as retarget() applies.
  bool reanchor(float position, float velocity);

  // sample -- evaluate the current trajectory at an elapsed time since it
  // was solved (Ruckig's own Trajectory::at_time(), including its built-in
  // past-duration hold-at-final-state extrapolation -- see the class
  // comment). Updates the remembered last-sample state used to seed the
  // next solveToRest()/solveToVelocity() call.
  State sample(float elapsed);  // [s]

  // duration -- this channel's currently held trajectory's total duration
  // (Ruckig's own Trajectory::get_duration()). 0 before the first
  // successful solve.
  float duration() const;  // [s]

 private:
  // calculated_ -- true once otg_.calculate() has succeeded at least once
  // for this instance (set in solvePositionControl()/solveToVelocity()).
  // Defensive guard (2026-07-10 UB fix): a default-constructed
  // ruckig::Trajectory<1>'s `duration` field is a real 0.0 (safe), but its
  // `profiles` array holds a default-constructed ruckig::Profile whose
  // std::array<double,...> members (p/v/a/t_sum/brake.duration) are NOT
  // zero-initialized -- reading them via sample()/at_time() before any
  // calculate() call is undefined behavior (observed on hardware/sim as a
  // phantom, heap-residue-dependent velocity spike). sample()/duration()
  // both check this flag and return a safe zero State{}/0.0f instead of
  // touching traj_ when it is still false. This is belt-and-suspenders: the
  // primary fix is that every SegmentExecutor call path that can enter a
  // phase now unconditionally solves EVERY channel it might later sample
  // (segment_executor.cpp's beginStreamFresh()/mergePending()/
  // replaceStream()), so traj_ should always be calculate()'d before this
  // guard would ever trigger in practice.
  bool calculated_ = false;

  // accelBoundsForDirection -- Open Question 2's direction-mirrored
  // max_acceleration/min_acceleration mapping (class comment). Positive
  // direction: (a_max, -a_decel); negative direction: (a_decel, -a_max).
  void accelBoundsForDirection(bool positiveDirection, float* maxAccel, float* minAccel) const;

  // solvePositionControl -- shared implementation behind solveToRest(),
  // retarget(), and reanchor() -- all three are the SAME position-control
  // solve-to-rest mode (class comment), differing only in what current/
  // target state each passes in.
  bool solvePositionControl(float targetPosition, float currentPosition, float currentVelocity,
                             float currentAcceleration, float maxVelocity);

  // ---- Config (configure()) ----
  float aMax_ = 0.0f;             // [mm/s^2] or [rad/s^2] positive-direction accel bound
  float aDecel_ = 0.0f;           // [mm/s^2] or [rad/s^2] negative-direction (decel) bound magnitude
  float maxJerk_ = 0.0f;          // [mm/s^3] or [rad/s^3] mapped jerk (0 sentinel -> +infinity)
  float velocityCeiling_ = 0.0f;  // [mm/s] or [rad/s] global outer clamp (v_body_max/yaw_rate_max)

  // ---- Remembered last-sample / plan state (Decision 8) ----
  float lastPosition_ = 0.0f;      // [mm] or [rad] -- this channel's own frame
  float lastVelocity_ = 0.0f;      // [mm/s] or [rad/s] signed
  float lastAcceleration_ = 0.0f;  // [mm/s^2] or [rad/s^2] signed
  float target_ = 0.0f;            // [mm] or [rad] last position-control target (reanchor() reuse)
  float maxVelocity_ = 0.0f;       // [mm/s] or [rad/s] last per-call ceiling (retarget()/reanchor() reuse)

  ruckig::Ruckig<1> otg_;
  ruckig::Trajectory<1> traj_;
};

}  // namespace Motion
