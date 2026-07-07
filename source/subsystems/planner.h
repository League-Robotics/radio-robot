// planner.h -- Subsystems::Planner: the goal-closure engine every motion
// verb (S/T/D/R/TURN/RT/G, tickets 084-002..004) stages a msg::PlannerCommand
// into. Owns one Motion::VelocityRamp and the active goal's stop-condition
// array; advances the ramp and evaluates the stops once per tick(), holding
// its output (a msg::DrivetrainCommand{TWIST}, or a zero twist once idle) via
// hasCommand()/takeCommand() -- the same held/taken discipline Subsystems::
// Drivetrain/PoseEstimator already use.
//
// TWO COEXISTING MOTION-GENERATION MECHANISMS (089-003, architecture-
// update.md (089) Decision 5): this class currently owns BOTH `ramp_`
// (Motion::VelocityRamp, a per-tick "step toward whatever the target
// currently is" chaser) AND `linear_`/`rotational_` (Motion::JerkTrajectory,
// a "solve the whole jerk-limited plan once, sample many times" profiler,
// source/motion/jerk_trajectory.h). Which goal kind uses which is currently
// split as follows -- **KNOWN INTERMEDIATE STATE**, not the sprint's final
// shape:
//   - `DISTANCE` (089-003, this ticket): `linear_` only. `apply()` solves a
//     position-control solve-to-rest once (no live pose/observation
//     dependency -- the whole plan is knowable at apply() time); `tick()`
//     samples it every pass instead of calling `ramp_.advance()`/
//     `applyStopAnticipation()`. `rotational_` is constructed (a class
//     member, like `linear_`) but does no real work for this goal kind
//     (`omega` is always 0 for `DISTANCE`).
//   - `TIMED`/`VELOCITY`/`STREAM`/`TURN`/`ROTATION`/`GOTO_GOAL`: still
//     `ramp_` + `applyStopAnticipation()`, UNCHANGED, until tickets 004
//     (`TIMED`/`VELOCITY`/`STREAM`) and 005 (`TURN`/`ROTATION`, which also
//     retires `applyStopAnticipation()` in full) land.
// `tick()`'s dispatch therefore needs a goal-kind-aware check scoped
// narrowly to `DISTANCE` (`mode_ == msg::DriveMode::DISTANCE`), NOT yet the
// clean `mode_ == GO_TO` vs. not split the architecture doc describes as the
// sprint's END state -- do not generalize this early; ticket 005 is where
// the dispatch collapses to that final two-way split, once every other
// goal kind has migrated too.
//
// Ported (concept, not byte-for-byte) from source_old/superstructure/
// Planner.{h,cpp} + source_old/commands/MotionCommand.{h,cpp}, onto the
// already-generated msg::PlannerCommand/PlannerState/PlannerConfig/
// StopCondition types (source/messages/planner.h). This ticket (084-001)
// lands NO wire verb -- it is built and tested in isolation; tickets
// 084-002..004 register the verbs that construct and stage PlannerCommands.
//
// Like Drivetrain/PoseEstimator, Planner holds NO Hal::Motor/Drivetrain/
// PoseEstimator reference or pointer: tick() takes this tick's observations
// (msg::MotorState for each wheel, msg::PoseEstimate for the fused pose) as
// arguments only. Two consequences that follow directly from this, both
// deliberate (not oversights):
//
//   1. apply(cmd, now) cannot capture a Motion::MotionBaseline -- it has no
//      observations to snapshot from (its locked signature is (const
//      msg::PlannerCommand&, uint32_t) only). Baseline capture instead
//      happens on the FIRST tick() call after a goal is staged (tick() DOES
//      receive observations) -- see planner.cpp's captureBaseline().
//
//   2. apply() cannot resolve a goal that needs the CURRENT pose to compute
//      (e.g. TURN's turn direction: shortest-path to an absolute heading
//      needs the heading AT COMMAND TIME, which apply() has no argument
//      for). For TurnGoal.speed and RotationGoal.speed specifically, this
//      class treats the field as an ALREADY-SIGNED angular rate: the caller
//      (ticket 084-003's wire handler, which DOES have synchronous access to
//      PoseEstimator::fusedPose() when parsing TURN/RT off the wire) resolves
//      the sign -- and the matching HEADING/ROTATION stop condition -- before
//      constructing the msg::PlannerCommand and calling apply(). This is a
//      deliberate division of labor, not a naming accident: the wire-schema
//      field is literally spelled "speed" (a magnitude-shaped name) but this
///     class's contract for it is "signed rate, already resolved."
//
// Stop-condition policy: apply() copies the caller-supplied cmd.stops_[]
// verbatim (Motion::evaluateStopCondition is the only place stop-condition
// MATH lives -- Planner never re-derives it) and additionally synthesizes
// exactly two goal-kind-intrinsic stops that are computable from the goal's
// OWN fields alone, with no PlannerConfig/DrivetrainConfig/pose dependency --
// mirroring what source_old's OWN beginDistance()/beginTimed() did (as
// opposed to their wire-layer callers):
//   - DISTANCE: an implicit STOP_DISTANCE at |distance|, plus a generous
//     STOP_TIME safety net (2x nominal travel time + 2s).
//   - TIMED: an implicit STOP_TIME at `duration`, when duration > 0.
// VELOCITY/STREAM (open-ended by nature) and TURN/ROTATION (their natural
// stop needs the CURRENT fused pose AT COMMAND TIME to resolve a sign/target,
// which apply() cannot read -- the wire-layer caller resolves it instead;
// see the class comment above on TurnGoal.speed/RotationGoal.speed) rely
// entirely on the caller-supplied cmd.stops_[].
//
// GOTO_GOAL (ticket 084-004) is a third case: it owns stops_[] END TO END
// and accepts NO caller-supplied stops at all -- copyCallerStops() is never
// called for this goal kind, matching docs/protocol-v2.md sec 10's G
// contract, which defines no stop= clause for G. It ports the PRE_ROTATE/
// PURSUE state machine from source_old/superstructure/Planner.cpp's
// driveAdvance() G-phase branch + source_old/control/PlannerBegin.cpp's
// beginGoTo(), adapted to this class's "apply() gets no pose" constraint:
//   - apply() decides PRE_ROTATE vs PURSUE from the bearing to the relative
//     (x, y) target ALONE (atan2f(y, x) -- no pose needed, since (x, y) is
//     already expressed in the robot's own frame at command time) against
//     PlannerConfig.turn_in_place_gate (degrees, not radians). PRE_ROTATE's
//     HEADING + TIME-net stops are pose-independent deltas/durations and are
//     staged immediately, right there in apply().
//   - The world-frame anchor for (x, y) -- and, when starting straight into
//     PURSUE, PURSUE's own POSITION + TIME-net stops -- are resolved on the
//     FIRST tick() (captureBaseline(), which DOES receive fusedPose; see
//     enterPursue()), the equivalent "at command time" moment since no
//     movement occurs between apply() and that first tick.
//   - Mid-goal, when PRE_ROTATE's HEADING stop fires, tick() hands off
//     straight to PURSUE (enterPursue()) instead of completing the goal --
//     no event, no ramp-down -- ported from driveAdvance()'s own
//     PRE_ROTATE-terminated branch.
//   - PURSUE re-steers (v, omega) toward the world-frame anchor from the
//     live fusedPose every tick (pursueSteer()) -- terminal decel cap plus a
//     curvature clamp, ported from driveAdvance()'s PURSUE hook -- MINUS the
//     backtrack re-gate counter and D-mode-style stall-forced-completion
//     (out of this ticket's acceptance bar; concept-not-byte-for-byte, the
//     same simplification precedent RT/TURN already established for
//     coast-anticipation/rotational-slip).
//
// "Reply sink" capture: apply()'s locked signature takes no ReplyFn/ctx (a
// wire message cannot carry a function pointer), so the only "reply sink"
// context this class captures is corr_id -- copied verbatim from
// msg::PlannerCommand.corr_id into the held Event a completed goal produces
// (hasEvent()/takeEvent()). Routing that Event to an actual reply channel is
// the DRAINING caller's job (ticket 084-002's dev_loop.cpp step), the same
// division dev_loop.h's own DevLoop::defaultReply documents for the
// watchdog-fire EVT.
#pragma once

#include <stdint.h>

#include "messages/common.h"
#include "messages/drivetrain.h"
#include "messages/motor.h"
#include "messages/planner.h"
#include "motion/jerk_trajectory.h"
#include "motion/motion_baseline.h"
#include "motion/velocity_ramp.h"

namespace Subsystems {

class Planner {
 public:
  // Event -- a held/taken descriptor for one pending "goal completed" signal.
  // reason is one of the tokens Motion::evaluateStopCondition's five
  // supported kinds map to ("time"/"dist"/"heading"/"pos"/"rot"), matching
  // source_old/commands/MotionCommand.cpp's mc_reasonToken() vocabulary for
  // those same five kinds. corrId is copied verbatim from the
  // msg::PlannerCommand.corr_id that staged the completed goal. Formatting
  // the final wire text ("EVT done <verb> reason=<token> #<corrId>") is the
  // draining caller's job -- see the class comment above.
  struct Event {
    char reason[16] = {};
    char corrId[64] = {};
  };

  // apply -- stage the goal (dispatch on goal_kind); captures the reply
  // sink/corr-id context needed for later EVT emission (mirrors
  // source_old/commands/MotionCommand.cpp's setReplySink()). No hardware
  // I/O, no MotionBaseline capture -- see the class comment.
  void apply(const msg::PlannerCommand& cmd, uint32_t now);

  // tick -- advance the owned Motion::VelocityRamp, evaluate the active
  // goal's stop conditions, and hold a msg::DrivetrainCommand{TWIST} (or a
  // zero twist once idle) via hasCommand()/takeCommand(). now/leftObs/
  // rightObs/fusedPose are this tick's OBSERVATIONS ONLY -- never stored
  // beyond this call (see class comment). Runs unconditionally every pass
  // (the caller does not gate this on hasActiveCommand()), mirroring
  // Drivetrain::tick()'s own "sets hasCommand() unconditionally" contract.
  void tick(uint32_t now, const msg::MotorState& leftObs, const msg::MotorState& rightObs,
            const msg::PoseEstimate& fusedPose);

  // Output edge -- unchanged in shape by sprint 087 (ticket 087-003 AC4):
  // Planner's own tick() signature and this hasCommand()/takeCommand() pair
  // are untouched by the blackboard-queue wiring. Per
  // clasi/sprints/087-two-plane-blackboard-synchronous-update-loop-
  // configurator-and-command-queue-transport-greenfield/
  // architecture-update-r1.md Decision 1, whatever DRAINS this edge (the
  // main loop's routeOutputs, ticket 007) is the SECOND producer of
  // Rt::Mailbox<msg::DrivetrainCommand> driveIn -- Subsystems::Drivetrain's
  // own command-plane input (see Drivetrain::tick()'s doc comment,
  // drivetrain.h) -- alongside CommandRouter's `DEV DT` path. Both
  // producers must gate on Drivetrain's currently-published authority state
  // (msg::DrivetrainState::active, 087-003 AC2) before posting to driveIn;
  // that authority GATE itself is ticket 006/007's job, out of this
  // ticket's scope -- Planner posts nothing to driveIn directly.
  bool hasCommand() const;               // true once tick() has run and the output is untaken
  msg::DrivetrainCommand takeCommand();  // clears hasCommand()

  bool hasEvent() const;  // true when a completed goal's Event is untaken
  Event takeEvent();      // clears hasEvent()

  msg::PlannerState state() const;
  void configure(const msg::PlannerConfig& config);

  // hasActiveCommand -- mirrors source_old/commands/MotionCommand.h's
  // active(): true while a goal is running (including a SMOOTH ramp-down),
  // false once fully IDLE.
  bool hasActiveCommand() const;

 private:
  // stageCommon -- 089-003: the bookkeeping shared by EVERY goal_kind case in
  // apply(), factored out of stageGoal() so DISTANCE (which no longer calls
  // ramp_.setTarget(), see class comment) can reuse it directly: captures
  // style/corr_id, resets stopping_/baselineCaptured_, (re)activates the
  // command, and records mode_. Does NOT touch stops_[]/stopsCount_ or
  // stagedV_/stagedOmega_ -- callers set those themselves (stageGoal() below
  // sets the latter; DISTANCE's apply() case sets them directly since it has
  // no single (v, omega) ramp target to derive them from).
  void stageCommon(msg::DriveMode mode, const msg::PlannerCommand& cmd);

  // stageGoal -- shared tail for every ramp_-driven (non-STOP/NONE) goal_kind
  // case in apply(): sets the ramp target, latches the staged (v, omega) for
  // the next baseline capture, then calls stageCommon(). Does NOT touch
  // stops_[]/stopsCount_ -- callers populate those (via copyCallerStops()/
  // appendStop()) before calling this. NOT used by DISTANCE (089-003) --
  // see class comment.
  void stageGoal(float v, float omega, msg::DriveMode mode, const msg::PlannerCommand& cmd);

  // copyCallerStops -- reset stops_[]/stopsCount_ to the caller-supplied
  // cmd.stops_[] verbatim (bounded to the 4-slot cap already enforced by the
  // wire schema). appendStop() may add more, up to the same cap.
  void copyCallerStops(const msg::PlannerCommand& cmd);

  // appendStop -- append one goal-kind-intrinsic stop condition (DISTANCE/
  // TIMED's implicit stops, and GOTO_GOAL's PRE_ROTATE/PURSUE stops -- see
  // class comment). `b`/`ax` default to 0.0f for the single-field kinds
  // (DISTANCE/TIME); GOTO_GOAL's HEADING (a, b) and POSITION (ax, a, b)
  // stops pass them explicitly -- parameter order mirrors msg::
  // StopCondition's own field declaration order. Silently drops the
  // addition if the 4-slot cap is already full (matches source_old/
  // commands/MotionCommand.cpp's addStop() overflow behavior; there is no
  // wire ERR reply path at this layer to signal it).
  void appendStop(msg::StopKind kind, float a, float b = 0.0f, float ax = 0.0f);

  // captureBaseline -- snapshot a Motion::MotionBaseline from this tick's
  // observations. Called once, on the first tick() after a goal is staged
  // (see class comment for why this cannot happen in apply()). For
  // GOTO_GOAL, also resolves the world-frame anchor (gTargetXWorld_/
  // gTargetYWorld_) from this tick's fusedPose -- see class comment.
  void captureBaseline(uint32_t now, const msg::MotorState& leftObs,
                       const msg::MotorState& rightObs, const msg::PoseEstimate& fusedPose);

  // enterPursue -- (re)configure GOTO_GOAL's PURSUE sub-phase: resets
  // stops_[] to PURSUE's own POSITION stop (at gTargetXWorld_/
  // gTargetYWorld_, radius PlannerConfig.arrive_tol) plus a generous TIME
  // safety net, re-baselines baseline_.t0 for that net (ported concept:
  // source_old's MotionCommand::start() re-baselines when PURSUE is
  // (re)configured), and resets the ramp for a fresh ramp-up. Called from
  // captureBaseline() (goal starts straight into PURSUE) and from tick()'s
  // stop-evaluation loop (PRE_ROTATE's HEADING stop handing off to PURSUE
  // mid-goal) -- see class comment.
  void enterPursue(uint32_t now);

  // pursueSteer -- recompute PURSUE's (v, omega) target from the live
  // fusedPose vs. the world-frame goal anchor: a terminal decel cap (v
  // capped so the robot can still stop within arrive_tol) plus a curvature
  // clamp (bounds omega so passing abeam the target cannot orbit tightly).
  // Ported from source_old/superstructure/Planner.cpp's driveAdvance()
  // PURSUE hook, minus the backtrack re-gate counter (see class comment).
  // Called once per tick, BEFORE ramp_.advance(), while gPhase_ == PURSUE.
  void pursueSteer(const msg::PoseEstimate& fusedPose);

  // applyStopAnticipation -- ticket 086-003: the SAME anticipation pattern
  // pursueSteer() already applies for GOTO_GOAL's STOP_POSITION, extended to
  // DISTANCE/TURN/ROTATION via Motion::remainingToStop() (architecture-
  // update.md (086) Decision 2), so those three goal kinds also arrive at
  // their stop condition already near-zero speed/rate instead of handing the
  // motor loop a full-speed "arrest from full speed" problem. For every
  // currently-active stop condition (stops_[0..stopsCount_)) whose kind is
  // STOP_DISTANCE, re-derives a linear speed cap (vCap = sqrt(2 * a_decel *
  // dRemaining)); STOP_ROTATION/STOP_HEADING, the analogous angular-rate cap
  // (omegaCap = sqrt(2 * yaw_acc_max * angleRemaining)). Re-targets the ramp
  // to min(staged, cap) (sign-preserved) every tick, the same way
  // pursueSteer() re-targets every tick regardless of whether the cap
  // currently binds. Deliberately NOT called for GO_TO (mode_ ==
  // msg::DriveMode::GO_TO): PURSUE already owns its own STOP_POSITION
  // anticipation via pursueSteer(), and PRE_ROTATE's own STOP_HEADING is a
  // phase-handoff bearing gate, not a terminal stop -- capping its spin rate
  // as it approached the gate would change GOTO_GOAL's own behavior, which
  // this ticket leaves provably unchanged (see class comment / ticket
  // acceptance). Called once per tick, BEFORE ramp_.advance(), for every
  // other goal kind, while the goal is still running (guarded by the caller
  // on !stopping_ -- once the SMOOTH ramp-down has armed target (0,0), this
  // must not re-target the ramp away from zero).
  //
  // Ticket 087-009: the STOP_DISTANCE/STOP_ROTATION branches additionally
  // fold in a FIXED dead-time compensation (architecture-update-r1.md's
  // Decision 6/2 two-pass Planner->driveIn->Drivetrain->motorIn->Hardware
  // output latency, versus ticket 006's same-pass feed-forward) -- see
  // planner.cpp's own comment on the exact closed-form and ticket 087-009's
  // completion notes for the measured before/after numbers. STOP_HEADING is
  // deliberately left unmodified (TURN's own tests already pass without it).
  void applyStopAnticipation(const msg::MotorState& leftObs,
                             const msg::MotorState& rightObs,
                             const msg::PoseEstimate& fusedPose);

  // linearElapsed -- [s] elapsed time since the linear channel's most recent
  // (re)solve (linearSolveMs_, updated by every solveToRest()/retarget()/
  // reanchor()/solveToVelocity() call this class makes on linear_). The
  // sole argument to linear_.sample() -- see class comment's "KNOWN
  // INTERMEDIATE STATE" note.
  float linearElapsed(uint32_t now) const;

  // maybeReplanDistance -- 089-003, architecture-update.md Decision 10:
  // DISTANCE's divergence-triggered replan. Called once per tick, ONLY
  // while the goal's own stop condition has not fired (guard 1 -- enforced
  // by the caller, tick(), not here), for the CURRENT tick's `now`/
  // observations. Compares linear_'s own remembered plan-remaining (its
  // current target minus its last-sampled position) against
  // Motion::remainingToStop()'s MEASURED remaining for this goal's
  // STOP_DISTANCE condition; when they diverge by at least
  // kDivergenceThreshold, requests a retarget() (normal case) or, past
  // kGrossDivergenceThreshold, a reanchor() (gross case) -- see planner.cpp
  // for the exact dead-time-projected target formula (reuses
  // applyStopAnticipation()'s own kDeadTime, hoisted to file scope so both
  // methods share one definition). Enforces guard 2 (no-reverse-target: a
  // replan is skipped if the dead-time-projected measured remaining is <=
  // 0) and guard 3 (a minimum interval between replans, lastReplanMs_/
  // kMinReplanInterval) itself -- Motion::JerkTrajectory enforces NEITHER
  // guard (ticket 002's boundary decision; see jerk_trajectory.h's own
  // retarget()/reanchor() doc comments).
  void maybeReplanDistance(uint32_t now, const msg::MotorState& leftObs,
                           const msg::MotorState& rightObs,
                           const msg::PoseEstimate& fusedPose);

  // armDistanceStopDecel -- 089-003, ticket item 4: called from tick()'s
  // stop-evaluation loop exactly once, the instant a SMOOTH-style DISTANCE
  // goal's stop condition fires (mirrors the ramp_-driven path's
  // `ramp_.setTarget(0.0f, 0.0f)` call at the same point). If linear_'s
  // OWN plan has not yet naturally converged to rest (elapsed <
  // linear_.duration()), re-solves a fresh velocity-control decel-to-rest
  // (solveToVelocity(0, ...)) seeded from linear_'s own current sampled
  // state (Decision 8 -- never leftObs/rightObs) and resets linearSolveMs_.
  // In the common case (the plan has already converged), this is a no-op --
  // see class comment / ticket description.
  void armDistanceStopDecel(uint32_t now);

  // queueEvent -- hold a completed-goal Event (reason token + the staged
  // goal's corr_id).
  void queueEvent(const char* reason);

  // holdTwistCommand -- pack (v, omega) into a msg::DrivetrainCommand{TWIST}
  // and hold it (hasCommand()/takeCommand()).
  void holdTwistCommand(float v, float omega);

  msg::PlannerConfig config_ = {};
  Motion::VelocityRamp ramp_;

  // 089-003: the linear channel driving DISTANCE (see class comment). The
  // rotational channel is constructed here too (architecture-update.md (089)
  // Decision 1/ticket item 1) but does no real work until ticket 005
  // (TURN/ROTATION) -- DISTANCE's omega is always 0, so nothing in this
  // ticket ever solves or samples it.
  Motion::JerkTrajectory linear_;
  Motion::JerkTrajectory rotational_;

  uint32_t linearSolveMs_ = 0;  // [ms] absolute time of linear_'s most recent (re)solve
  // linear_'s CURRENT target, in whatever frame its most recent (re)solve
  // established (retarget() rebaselines the frame; reanchor() does not
  // change the target). Kept in sync with JerkTrajectory's own internal
  // target_ so maybeReplanDistance() can compute "plan's own remaining"
  // (fabsf(linearTarget_ - <last sampled position>)) without JerkTrajectory
  // needing its own public target() accessor (out of this ticket's scope --
  // Files to modify: planner.h/.cpp only).
  float linearTarget_ = 0.0f;    // [mm]
  float linearCeiling_ = 0.0f;   // [mm/s] per-call max_velocity most recently
                                 // used for a solveToRest() (reused by
                                 // armDistanceStopDecel()'s solveToVelocity()
                                 // call -- retarget()/reanchor() reuse their
                                 // OWN remembered ceiling internally instead).

  // Divergence-replan rate limiting (Decision 10 guard 3) -- shared across
  // whichever channel currently supports it (DISTANCE this ticket; TURN/
  // ROTATION, ticket 005).
  uint32_t lastReplanMs_ = 0;  // [ms] last divergence-triggered replan (or apply()) time
  // Ticket-owned defaults (architecture-update.md Decision 10: "threshold
  // values are ticket-owned, not specified [by the architecture doc]") --
  // characterized on the bench, ticket 007; may be retuned there. Sized
  // empirically against a closed-loop sim scenario (a 15%-lagging plant,
  // planner_harness.cpp's scenarioDistanceGoalDivergenceReplanCorrectsLagging
  // Plant): kMinReplanInterval must be short enough, relative to how fast
  // planSpeed decays during the terminal decel phase, that the LAST
  // correcting replan still lands close enough to the goal's true target --
  // each replan's own dead-time projection intentionally undershoots by
  // planSpeed*kDeadTime (Decision 8's revision), a gap a REAL wheel's
  // continued coast through the output dead time is meant to close, so a
  // replan cadence too coarse to fire again before the plan fully
  // decelerates leaves that gap permanently uncorrected (confirmed by
  // direct measurement: the original, more conservative 8mm/200ms pairing
  // left a goal ~0.6mm short of a 500mm target under that same scenario).
  static constexpr float kDivergenceThreshold = 3.0f;        // [mm]
  static constexpr float kGrossDivergenceThreshold = 40.0f;  // [mm]
  static constexpr uint32_t kMinReplanInterval = 60;         // [ms]

  // currentV_/currentOmega_ -- this tick's held commanded body twist,
  // regardless of which mechanism produced it (ramp_ or a JerkTrajectory
  // channel) -- cached by holdTwistCommand() so state() reports the SAME
  // value handed to takeCommand(), instead of reading ramp_.currentV()/
  // currentOmega() directly (which would go stale for a JerkTrajectory-
  // driven goal kind like DISTANCE, since ramp_ is untouched for it).
  float currentV_ = 0.0f;      // [mm/s] signed
  float currentOmega_ = 0.0f;  // [rad/s] signed

  bool activeCmd_ = false;   // mirrors MotionCommand::_active
  bool stopping_ = false;    // true during a SMOOTH ramp-down to (0,0)
  bool baselineCaptured_ = false;
  uint32_t softDeadline_ = 0;  // [ms] absolute deadline for a SMOOTH ramp-down
  // Matches source_old/commands/MotionCommand.cpp's kSoftDeadline.
  static constexpr uint32_t kSoftDeadlineMs = 3000;  // [ms]

  msg::StopCondition stops_[4] = {};
  uint8_t stopsCount_ = 0;
  msg::StopStyle style_ = msg::StopStyle::SMOOTH;
  char corrId_[64] = {};

  // The (v, omega) staged by the most recent apply() call, latched BEFORE
  // any ramping -- read by captureBaseline() to compute MotionBaseline::
  // vSign/omegaSign (the ramp's own currentV()/currentOmega() would read as
  // the PREVIOUS command's live speed at that point, not the new target).
  float stagedV_ = 0.0f;      // [mm/s]
  float stagedOmega_ = 0.0f;  // [rad/s]

  const char* pendingReason_ = "";  // reason token latched when stopping_ is armed
  Motion::MotionBaseline baseline_ = {};

  msg::DriveMode mode_ = msg::DriveMode::IDLE;

  // GOTO_GOAL (G) state machine (ticket 084-004) -- ported concept from
  // source_old/superstructure/Planner.h's GPhase/_gTargetXWorld/
  // _gTargetYWorld. Only meaningful while mode_ == GO_TO; see class comment.
  enum class GPhase : uint8_t { IDLE, PRE_ROTATE, PURSUE };
  GPhase gPhase_ = GPhase::IDLE;
  float gTargetXWorld_ = 0.0f;  // [mm] world-frame goal X, resolved on the first tick()
  float gTargetYWorld_ = 0.0f;  // [mm] world-frame goal Y, resolved on the first tick()

  // PlannerState reporting fields (mirrors source_old's TargetState-derived
  // fields) -- set by apply(), read back by state().
  float targetX_ = 0.0f;         // [mm]
  float targetY_ = 0.0f;         // [mm]
  float targetSpeed_ = 0.0f;     // [mm/s]
  float distanceTarget_ = 0.0f;  // [mm]

  bool hasCommand_ = false;
  msg::DrivetrainCommand heldCommand_ = {};

  bool hasEvent_ = false;
  Event heldEvent_ = {};

  // dt tracking for VelocityRamp::advance(), independent of goal boundaries
  // (mirrors PoseEstimator's own haveLastTick_/lastTick_ pattern).
  bool haveLastTick_ = false;
  uint32_t lastTickMs_ = 0;  // [ms]
};

}  // namespace Subsystems
