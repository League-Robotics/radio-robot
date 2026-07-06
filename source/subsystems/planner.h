// planner.h -- Subsystems::Planner: the goal-closure engine every motion
// verb (S/T/D/R/TURN/RT/G, tickets 084-002..004) stages a msg::PlannerCommand
// into. Owns one Motion::VelocityRamp and the active goal's stop-condition
// array; advances the ramp and evaluates the stops once per tick(), holding
// its output (a msg::DrivetrainCommand{TWIST}, or a zero twist once idle) via
// hasCommand()/takeCommand() -- the same held/taken discipline Subsystems::
// Drivetrain/PoseEstimator already use.
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
// Every other goal kind (VELOCITY/STREAM: open-ended by nature; TURN/
// ROTATION/GOTO_GOAL: their natural stop needs pose and/or DrivetrainConfig's
// trackwidth, which this class does not have) relies entirely on the
// caller-supplied cmd.stops_[].
//
// GOTO_GOAL is a placeholder this ticket: it holds a straight v=speed,
// omega=0 twist with no pre-rotate/pursue phasing (architecture-update.md:
// "ticket 001's apply()/tick() already has a goal_kind switch with a
// GOTO_GOAL arm reserved from the schema -- ticket 004 is the first to
// implement it meaningfully"). Termination relies entirely on the
// caller-supplied stops_[] (e.g. a POSITION stop) until then.
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
  // stageGoal -- shared tail for every real (non-STOP/NONE) goal_kind case in
  // apply(): sets the ramp target, latches the staged (v, omega) for the
  // next baseline capture, captures style/corr_id, and (re)activates the
  // command. Does NOT touch stops_[]/stopsCount_ -- callers populate those
  // (via copyCallerStops()/appendStop()) before calling this.
  void stageGoal(float v, float omega, msg::DriveMode mode, const msg::PlannerCommand& cmd);

  // copyCallerStops -- reset stops_[]/stopsCount_ to the caller-supplied
  // cmd.stops_[] verbatim (bounded to the 4-slot cap already enforced by the
  // wire schema). appendStop() may add more, up to the same cap.
  void copyCallerStops(const msg::PlannerCommand& cmd);

  // appendStop -- append one goal-kind-intrinsic stop condition (DISTANCE/
  // TIMED's implicit stops -- see class comment). Silently drops the
  // addition if the 4-slot cap is already full (matches source_old/commands/
  // MotionCommand.cpp's addStop() overflow behavior; there is no wire ERR
  // reply path at this layer to signal it).
  void appendStop(msg::StopKind kind, float a);

  // captureBaseline -- snapshot a Motion::MotionBaseline from this tick's
  // observations. Called once, on the first tick() after a goal is staged
  // (see class comment for why this cannot happen in apply()).
  void captureBaseline(uint32_t now, const msg::MotorState& leftObs,
                       const msg::MotorState& rightObs, const msg::PoseEstimate& fusedPose);

  // queueEvent -- hold a completed-goal Event (reason token + the staged
  // goal's corr_id).
  void queueEvent(const char* reason);

  // holdTwistCommand -- pack (v, omega) into a msg::DrivetrainCommand{TWIST}
  // and hold it (hasCommand()/takeCommand()).
  void holdTwistCommand(float v, float omega);

  msg::PlannerConfig config_ = {};
  Motion::VelocityRamp ramp_;

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
