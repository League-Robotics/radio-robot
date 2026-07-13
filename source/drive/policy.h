// policy.h -- Drive:: pure policy evaluation: replan-envelope monitoring
// (sustain/rate-limit/N-max), the terminal settle machine, the flying-
// handoff envelope check, and pose-fix step absorption/bypass. Responsibility
// 5 from architecture-update.md (100) Step 2/M5 -- "WHEN to replan, WHAT
// Status to report" -- deliberately separated from responsibility 4
// (tracker.{h,cpp}, ticket 100-004: "HOW a reference-vs-measured error
// becomes a wheel-velocity command"). This file owns the ONE explicit
// statelessness residue (StepState's five scalars) and every numeric
// constant transcribed from the driving issue's "Control laws and numbers"
// section -- none of them are re-derived; each is named and commented with
// its exact source.
//
// **THIS IS THE HIGHEST-RISK FILE IN THE SPRINT.** The terminal machine
// governs exactly the kind of small terminal correction this project has
// been burned by before (the reversal-write-train encoder latch --
// docs/knowledge/2026-07-04-encoder-wedge.md: "The reversal write train: an
// immediate H-bridge sign flip written ... while the motor is under way").
// Every branch below that can emit a wheel command during Status::SETTLING
// is structurally non-negative for a forward stop segment -- see "Terminal
// machine: non-pivot" below.
//
// -- CRITICAL sign-convention note (load-bearing; read before touching the
// terminal-walk-in branch) --
// tracker.h's own "Reconciled sign convention" class comment documents that
// TrackerOutput::eAlong/eCross/eTheta report arc_math's NATIVE
// (measured - reference) sign, while the trim LAW internally negates them to
// (reference - measured) -- the sign the issue's whole "Control laws and
// numbers" section is written in ("errors reference-measured"). The
// REPLAN-ENVELOPE and FLYING-HANDOFF checks below compare |e| MAGNITUDES
// (fabsf(tracked.eAlong/eCross/eTheta)) directly against the NATIVE tracker
// output -- sign is moot for a magnitude comparison, so no negation is
// needed there. The TERMINAL WALK-IN's DIRECTION, however, is exactly the
// kind of check the reconciliation matters for: "outside -> drive forward"
// vs. "overshot -> stop, never reverse" is a SIGNED decision. evaluate()
// therefore computes its own `issueEAlong = -tracked.eAlong` (the SAME
// reconciled reference-measured sign tracker.cpp's trim law uses) before
// classifying the walk-in band -- see "Terminal machine: non-pivot" below
// for the derivation of which sign means "short of goal" vs. "overshot".
//
// -- Terminal machine: non-pivot (stop segments, straight or arc) --
// For a stop segment (Goal::exitSpeed == 0.0f), MasterProfile::peek()'s own
// past-duration hold-at-final-state extrapolation (master_profile.h's class
// comment) means RefState is frozen AT THE GOAL for every t >= duration --
// referenceAt(t).v == 0 there, so v_ref's own trim/replan contribution is
// moot; only the STATIC reference position remains. `issueEAlong =
// -tracked.eAlong = reference - measured` (tangent-frame): a POSITIVE value
// means the reference (the frozen goal) is still AHEAD of the measured pose
// along the tangent direction, i.e. the robot has not yet reached the goal
// ("short"); a NEGATIVE value means the robot's measured pose is ahead of
// the (now-stationary) goal, i.e. it has driven PAST it ("overshot"). This
// is exactly the walk-in law's own three bands:
//   |issueEAlong| <= tol      -> "inside": command literal 0.0f, dwell.
//   issueEAlong  >  tol       -> "outside" (short): command
//                                 clamp(k_s*issueEAlong, floor, ceiling),
//                                 POSITIVE by construction (k_s > 0,
//                                 issueEAlong > 0) -- a belt-and-suspenders
//                                 floor(0) still guards it, see policy.cpp.
//   issueEAlong  < -tol       -> "overshot": command literal 0.0f. NEVER a
//                                 negative (backward-correcting) command --
//                                 the issue's own "never negative" rule and
//                                 the wedge hazard above are the same
//                                 constraint from two directions.
// So EVERY command this branch can emit for a forward stop segment is one of
// {0.0f, a value in [floor, ceiling] with floor > 0} -- structurally
// non-negative, never a reversal off a nonzero prior command.
//
// "omega trims off" (the issue's own phrase): for a stop segment, ref.omega
// is already ~0 at t >= duration (omega = kappa * ref.v, and ref.v == 0
// there) and this file never applies an omega TRIM correction during
// SETTLING (unlike tracker.cpp's own arc-mode omegaTrim, which IS still
// computed into TrackRecord's diagnostic fields for observability -- it is
// simply not part of the EMITTED command here); both wheels get the SAME
// walk-in speed (omega == 0 literal), so no differential/IK step is needed
// at all -- see policy.cpp's non-pivot branch.
//
// -- Terminal machine: pivot (NOT a literal reapplication of the along-walk-
// in law -- documented judgment call, ticket 100-005) --
// The issue's terminal-machine numbers (k_s [1/s], the 50/100 mm/s
// floor/ceiling, the 10-15mm/15mm/s completion gate) are unambiguously
// LINEAR (mm, mm/s) and were written under the "Terminal (stop segments)"
// heading for a translating stop. Applying them literally to a PIVOT is not
// possible without a unit mismatch: a pivot's master DOF is HEADING (rad),
// its reference POSITION never moves (motion_plan.cpp's isPivot_ branch
// holds ref.x/ref.y at the anchor for the whole plan), so
// tracked.eAlong/eCross measure incidental TRANSLATIONAL drift during the
// spin, not the heading error the pivot actually needs to converge on --
// and reapplying k_s/floor/ceiling (calibrated in mm/s) to a rad/s omega
// channel produces a nonsensical 50-100 rad/s "walk-in speed". No
// pivot-specific numeric table exists anywhere in the issue's "Control laws
// and numbers" section to transcribe instead.
//
// Given that gap, this file does NOT invent a new banded walk-in law for
// pivots. Instead, per the issue's own "pivot mode (= 098)" framing (the
// control-law table's pivot row is explicitly "same as 098"), a pivot's
// SETTLING phase simply keeps running the SAME proven, UNCLAMPED pivot-mode
// tracker cascade past t >= duration (tracker.cpp's pivotMode branch,
// unmodified, unconditionally exempt from the one-sided forward-arc clamp
// since counter-rotating wheels are the pivot's own normal, safe operating
// mode -- not a "reversal" in the wedge-hazard sense, which is specifically
// about a SINGLE wheel's commanded sign flipping under way, not two wheels
// legitimately spinning opposite directions throughout an intentional
// in-place turn). The completion GATE is generalized structurally (hold the
// SAME dwell for 150ms once within tolerance, time out at duration + 1.5s
// exactly like the linear case) but substitutes eTheta/omega_hat for
// eAlong/v_hat, with tolerance constants (kPivotArriveTolTheta = 0.02 rad,
// kPivotArriveTolOmega = 0.05 rad/s) chosen -- NOT transcribed -- close to
// sprint 098's own proven bench accuracy (100% of turns within +/-1 deg,
// max observed error 0.59 deg -- the same kTheta=6.0 heading loop this
// ticket's pivot branch keeps running unmodified through SETTLING).
// Flagged here for stakeholder/bench-tuning follow-up: no
// closed-loop test in this ticket's own scenario list (a)-(h) exercises a
// pivot's terminal completion specifically (ticket 100-004's tracker harness
// already covers pivot CONVERGENCE; ticket 100-006's tier-0 suite is a
// better home for a bench-informed retune of these two constants once real
// pivot data exists).
//
// -- Replan envelopes / sustain / rate-limit / N-max --
// Rate-scheduled lag ALLOWANCE, not a compensator -- affects only WHEN a
// replan is requested, never what the tracker commands. `trimSaturated`
// (tracker.h's own contract: true exactly when a trim was clamped) AND
// "outside its envelope" must BOTH hold before the trigger even starts
// counting (a transient blip while UNsaturated never counts against
// sustain); once triggered continuously for kSustainHold (200ms), a replan
// is requested, gated by a >=300ms rate limit since the last one and capped
// at 3 total (a 4th attempt aborts -- ABORT_REPLAN_LIMIT). Pivot mode's own
// omegaTrim is UNCLAMPED by construction (tracker.cpp) -- trimSaturated is
// always false there -- so a pivot's RUNNING phase structurally never fires
// this trigger; only a large pose-fix step (below) can request a replan
// during a pivot.
//
// -- Flying handoff (Goal::exitSpeed != 0.0f) --
// Once the reference is exhausted (t >= duration) for a NON-stop segment,
// evaluate() checks the three envelope thresholds (e_cross, e_theta,
// e_along vs. a speed-scaled lag budget) every tick using the SAME magnitude
// comparison as the replan envelopes (sign is moot). Within envelope ->
// Status::DONE_HANDOFF (the NEXT plan()/replan() call -- the caller/adapter,
// ticket 100-007's job -- must seed entrySpeed = THIS plan's exitSpeed, from
// the REFERENCE, and entryAccel = 0: seeding from the measured state would
// inject the tracking lag as phantom deceleration into the next segment;
// this file only emits the Status at the right time, per the issue's own
// "seeding is the caller's job" boundary). Outside envelope -> the SAME pure
// replan mechanism as above (bypassing sustain -- there is no more time left
// in an exhausted plan to wait out a 200ms sustain window), still subject to
// rate-limit + N-max.
//
// -- Pose-fix step absorption/bypass --
// StepInput.poseStep/poseStepTheta (magnitude of an external pose-fix
// correction applied since the last step) <= 30mm/3deg is absorbed by the
// ordinary trim law with no policy action beyond resetting the sustain timer
// (a fresh "one T" grace period so the correction's own transient tracking
// error doesn't itself trip the sustain-hold). > 30mm/3deg bypasses sustain
// and requests a replan immediately (still subject to rate-limit + N-max).
// NEITHER effect applies while the terminal dwell is actively counting
// (StepState::dwellStart >= 0 at the START of the tick the poseStep
// arrives): the segment completes on its PRE-step basis (the dwell timer is
// untouched -- not reset, not extended) and the record still reports the
// TRUE current error honestly (tracker.track() always runs against the
// caller's current `measured` pose; only the DWELL TIMER's own start/reset
// bookkeeping is insulated from the poseStep event specifically).
#pragma once

#include "drive/motion_plan.h"  // RefState, StepState, StepInput, Status
#include "drive/tracker.h"      // TrackerOutput
#include "drive/types.h"        // Limits, WheelVelocities

namespace Drive {

// PolicyResult -- evaluate()'s output: the Status to report this tick and
// the FINAL wheel-velocity command to emit. For RUNNING/REPLAN_DUE (and a
// DONE_HANDOFF tick), command is tracker.track()'s own cascade output,
// unmodified -- policy does not touch the tracked command outside SETTLING.
// During Status::SETTLING, command is the terminal machine's own banded
// walk-in value (non-pivot) or the pivot's own unmodified tracked command
// (pivot) -- see policy.h's class comment. Every ABORT_*/DONE_STOP
// resolution forces a literal {0.0f, 0.0f} (the issue's "emitted setpoint
// snaps to a literal 0.0f" rule, generalized to every terminal exit, not
// only the ordinary dwell-based one).
struct PolicyResult {
  Status status = Status::RUNNING;
  WheelVelocities command;
};

// evaluate -- the pure policy decision described in this file's class
// comment. Reads elapsed time from `in.t`; ALL mutation is confined to
// `*state` (StepState's five scalars -- the subsystem's one statelessness
// residue). Never calls Drivetrain::replan() itself (the issue's explicit
// "never replans itself" rule) -- it only emits Status::REPLAN_DUE; the
// caller/adapter (ticket 100-007) is the one that actually calls replan()
// and swaps the held MotionPlan.
//
// @param duration        [s] the plan's own T_plan (MotionPlan::duration())
// @param exitSpeed        [mm/s] the plan's own Goal::exitSpeed (0 == stop
//                          segment -> the terminal walk-in machine; nonzero
//                          == flying segment -> the handoff envelope check)
// @param isPivot          MotionPlan::isPivot() -- selects the pivot vs.
//                          non-pivot terminal-machine branch
// @param isVelocityMode   MotionPlan::isVelocityMode() -- a MOVER teleop
//                          plan; `duration` is the caller's deadman (see
//                          motion_plan.h's own doc comment on why this same
//                          terminal machine handles the deadman elapsing)
// @param limits           gains snapshot -- ONLY trackKS (k_s) is consumed
//                          here (the walk-in law's own proportional gain);
//                          every other Limits field is the tracker's concern
// @param ref              the already-sampled reference (MotionPlan::
//                          referenceAt(in.t), ticket 100-003)
// @param tracked          tracker.track()'s own cascade output this tick
//                          (ticket 100-004) -- eAlong/eCross/eTheta in
//                          arc_math's NATIVE sign (see this file's own
//                          CRITICAL sign-convention note above)
// @param in               this tick's StepInput (t, measured, poseStep/
//                          poseStepTheta)
// @param state             caller-owned StepState -- read AND mutated here
// @return                 PolicyResult{status, command} -- see the struct's
//                          own doc comment
PolicyResult evaluate(float duration, float exitSpeed, bool isPivot, bool isVelocityMode,
                      const Limits& limits, const RefState& ref, const TrackerOutput& tracked,
                      const StepInput& in, StepState* state);

}  // namespace Drive
