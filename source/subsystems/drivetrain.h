// drivetrain.h -- Subsystems::Drivetrain: THE WAFER ADAPTER (sprint 100
// ticket 100-007, THE CUTOVER -- clasi/sprints/100-motion-stack-v2-self-
// contained-stateless-drive-subsystem/architecture-update.md M7). Bridges
// `source/drive/` (the self-contained, stateless Level-1 motion planner/
// tracker) to the rest of the firmware -- queues, blackboard, HAL staging,
// wire acks -- with ZERO control math anywhere in this file (greppable: no
// Kanayama/IK/saturation math outside source/drive/ after this ticket).
//
// Replaces the sprint 094 rewrite that made this class own a
// Motion::SegmentExecutor (the retired 3-phase Ruckig machine). That class
// (source/motion/segment_executor.{h,cpp}, plus segment.h/motion_baseline.h/
// stop_condition.{h,cpp}) is PARKED, not deleted (stays on disk, ticket
// 100-013 deletes it later, gated on bench/field sign-off) -- this class no
// longer references any of it.
//
// --- What this class HOLDS (architecture-update.md M7's own list) ---
//   - `Hardware& hardware_` (unchanged -- resolves the bound wheel pair and
//     stages output through it, exactly as before this ticket).
//   - `Drive::Drivetrain driveDrivetrain_` -- immutable config (Drive::
//     Limits + trackwidth), rebuilt whenever configure()/configureMotion()
//     supplies a fresh value for either half (source/subsystems/
//     drive_bridge.h's driveLimitsFromConfig() is THE single
//     msg::PlannerConfig -> Drive::Limits conversion point).
//   - `Drive::MotionPlan plan_` -- the CURRENT held plan value. Drive::
//     MotionPlan is copy-CONSTRUCTIBLE but NOT copy-ASSIGNABLE (master_
//     profile.h's own ruckig::Ruckig<1> const members -- documented at
//     tests/_infra/drive/drive_api.cpp's own file header) -- replacePlan()
//     below is the ONE place this class ever "reassigns" it, via placement-
//     new (no heap, matching this project's no-heap-in-hot-path rule,
//     runtime/queue.h's own doc comment).
//   - `Drive::StepState state_` -- the subsystem's one explicit
//     statelessness residue (five scalars -- motion_plan.h's own
//     "Statelessness accounting").
//   - `planStart_` -- [ms] this class's own clock anchor; StepInput.t is
//     computed as `(now - planStart_) * 0.001f` every tick -- the ONLY
//     place this subsystem's elapsed-time clock is derived from firmware
//     time (the issue's own "the adapter is also the ONLY place the
//     subsystem's clock ... is computed from firmware time").
//   - `Rt::WorkQueue<Drive::Goal, 8> ring_` -- ADMITTED, not-yet-planned
//     segments (wire admission -- BinaryChannel's handleSegment()/
//     handleReplace() -- already ran admit() before anything reaches this
//     ring; Drive::Drivetrain::plan()'s real Ruckig solve happens once an
//     entry is POPPED to become the active plan_, in startNextPlan()).
// `ChainTail` is NOT a member of this class -- it is threaded in as a
// `Drive::ChainTail&` (bb.chainTail) every tick(), the SAME "individual
// blackboard-cell reference, never a Blackboard&" convention every other
// parameter here already follows (subsystems never include blackboard.h --
// see runtime/blackboard.h's own file header). BinaryChannel's wire
// admission ALSO reads/advances the SAME bb.chainTail cell synchronously,
// at wire time -- see blackboard.h's own doc comment on chainTail for the
// full two-writer rationale.
//
// --- Boundary conversions (M7) -- all in source/subsystems/drive_bridge.h
// --- msg::MotorState -> Drive::WheelState; bb.bodyState (msg::PoseEstimate)
// -> Drive::BodyState; bb.poseStepped (msg::PoseStep) ->
// StepInput.poseStep/poseStepTheta; Drive::WheelVelocities -> msg::
// MotorCommand velocity staging via hardware_.motor(i).apply() -- the
// UNCHANGED staging path (this class still calls governRatio() on the
// Drive::-produced wheel targets before staging, exactly as it did on the
// retired executor's targets -- M13/ticket 100-013, not this one, is where
// that call retires, once Drive::'s own saturate/clamp cascade is bench/
// field-proven to supersede it).
//
// --- Status reactions (M7) ---
//   REPLAN_DUE -> Drivetrain::replan(), swap plan_ (replacePlan()); a
//     SOLVE_FAILED result leaves plan_/state_ untouched (drivetrain.cpp's
//     own replan() doc comment: "the CALLER keeps the old plan").
//   DONE_STOP/DONE_HANDOFF -> pop the next ring_ entry next tick (seeded
//     from the REFERENCE's own exitSpeed on a HANDOFF, per ticket 100-005's
//     handoff spec -- policy.cpp's own "Seeding contract" comment) or
//     neutral the motors once the ring is empty.
//   ABORT_TIMEOUT/ABORT_REPLAN_LIMIT -> flush the ring, re-anchor
//     ChainTail to the current measured pose, populate lastEvent() (an
//     msg::EventNotify) for MainLoop::commit() to publish onto bb.lastEvent
//     (see blackboard.h's own doc comment -- no loop-originated wire output
//     exists yet post-093 for this ticket to transmit it through; ticket
//     100-009 is the wire-arm ticket).
//
// --- Wire admission (BinaryChannel, commands/binary_channel.cpp -- NOT this
// file) --- runs synchronously at COMMAND time, before anything reaches
// bb.segmentIn/replaceIn: a `primitive=false` segment/replace is REJECTED
// with a typed ERR outright; a `primitive=true` one converts to a
// Drive::Goal and runs admit() (a throwaway Drive::Drivetrain built from
// bb.plannerConfig + bb.drivetrainConfig.trackwidth, mirroring THIS class's
// own rebuildDriveDrivetrain()) against bb.chainTail -- Verdict::OK
// advances bb.chainTail and posts the Goal; any other verdict replies a
// typed ERR and leaves bb.segmentIn/replaceIn (and bb.chainTail) untouched.
// See binary_channel.cpp's handleSegment()/handleReplace() for the full
// implementation -- this class never touches a reply channel.
//
// --- DIRECT/escape-hatch mode -- UNCHANGED (setTwist()/setWheelTargets()/
// setNeutral(), governRatio() for TWIST/WHEELS) --- these methods are
// byte-identical to before this ticket; only tick()'s own ORCHESTRATION of
// WHEN a driveIn command preempts segment mode changed (dispatchEscapeHatch()
// below), because the thing it used to preempt (Motion::SegmentExecutor) no
// longer exists in this class -- see that method's own doc comment for the
// one documented behavior change (an instant stop instead of a graceful
// decel-to-zero when NEUTRAL arrives mid-segment; source/drive/'s own
// graceful-stop equivalent, planVelocity(), is ticket 100-008's MOVER scope,
// not this one's).
#pragma once

#include <stdint.h>

#include "drive/drivetrain.h"
#include "drive/motion_plan.h"
#include "drive/types.h"
#include "messages/drivetrain.h"
#include "messages/envelope.h"
#include "messages/motor.h"
#include "messages/planner.h"
#include "runtime/queue.h"
#include "subsystems/hardware.h"

namespace Subsystems {

// The Drivetrain's bound wheel-motor pair, as 0-based Hardware motor
// indices -- read via ports() (`DEV DT PORTS` -> DrivetrainConfig.
// left_port/right_port, per sprint 079 decision 8; unchanged this ticket).
struct DrivetrainPorts {
  uint32_t left;   // 0-based Hardware motor index
  uint32_t right;  // 0-based Hardware motor index
};

class Drivetrain {
 public:
  // Stores hardware BY REFERENCE (never copied) -- the container this
  // Drivetrain resolves its bound wheel pair through every tick().
  explicit Drivetrain(Hardware& hardware);

  // --- Primitive setters -- the DIRECT (escape-hatch) arms' real
  // implementation. UNCHANGED this ticket -- see the class comment. ---
  void setTwist(float v_x, float v_y, float omega);   // [mm/s] [mm/s] [rad/s]
  void setWheelTargets(float left, float right);      // [mm/s] signed wheel velocities
  void setNeutral(msg::Neutral mode);

  // --- Faceplate verbs. ---

  // configure() -- THE single conversion point (0-based motor indices)
  // where this Drivetrain's bound wheel pair (msg::DrivetrainConfig.
  // left_port/right_port, 1-based brick labels) is converted to 0-based
  // Hardware motor indices EXACTLY ONCE. ALSO caches config.trackwidth and
  // rebuilds driveDrivetrain_ (rebuildDriveDrivetrain()) -- trackwidth is
  // one of driveDrivetrain_'s two construction arguments (the other,
  // Drive::Limits, comes from configureMotion() below); either call
  // rebuilds from the currently-cached pair of the other.
  void configure(const msg::DrivetrainConfig& config);

  // configureMotion() -- (100-007, THE CUTOVER) now populates Drive::Limits
  // via drive_bridge.h's driveLimitsFromConfig() and rebuilds
  // driveDrivetrain_ -- replaces the pre-cutover
  // `executor_.configure(config)` forward. Same call sites, same cadence
  // (boot-only direct call from both composition roots; ticket 098-005's
  // Configurator re-applies it live on a `CONFIG_PLANNER` delta).
  void configureMotion(const msg::PlannerConfig& config);

  // Unpacks the oneof -> the setters above, THEN dispatches the standby
  // side-channel. UNCHANGED this ticket.
  void apply(const msg::DrivetrainCommand& command);

  // tick -- the mandatory per-pass control step (run AFTER hardware.tick()).
  // now: [ms]. segmentIn/replaceIn: ADMITTED Drive::Goal fan-in (wire
  // admission already ran in BinaryChannel -- see the class comment).
  // driveIn: the S/STOP escape-hatch fan-in, drained FIRST per the
  // unchanged precedence rules. bodyState/poseStepped: sprint 099's
  // bb.bodyState/bb.poseStepped cells, converted at the boundary
  // (drive_bridge.h) into StepInput.measured/poseStep/poseStepTheta.
  // chainTail: bb.chainTail, re-anchored here on an ABORT_* (see the class
  // comment's "Status reactions").
  void tick(uint32_t now,
            Rt::WorkQueue<Drive::Goal, 8>& segmentIn,
            Rt::Mailbox<Drive::Goal>& replaceIn,
            Rt::WorkQueue<msg::DrivetrainCommand, 8>& driveIn,
            const msg::PoseEstimate& bodyState,
            const msg::PoseStep& poseStepped,
            Drive::ChainTail& chainTail);

  // Assembled from getters. enc_[]/vel_[]/cmd_[]/acc_[] sourced from
  // hardware_.motorState(port) exactly as before this ticket (MEASURED, not
  // commanded). active/busy/queue/rem now read planActive_/ring_/
  // remainingLinear_ instead of the retired executor_'s own state -- see
  // drivetrain.cpp's state() for the exact mapping.
  msg::DrivetrainState state() const;
  msg::DrivetrainCapabilities capabilities() const;

  void setMotorCapabilities(const msg::MotorCapabilities& left,
                             const msg::MotorCapabilities& right);

  // lastEvent -- (100-007) the most recent msg::EventNotify this class
  // populated on an ABORT_* status -- see blackboard.h's own doc comment on
  // bb.lastEvent (MainLoop::commit() publishes this every pass, mirroring
  // bb.drivetrain's own publish shape).
  msg::EventNotify lastEvent() const { return lastEvent_; }

  // --- Port binding + authority arbitration (unchanged from before this
  // ticket -- sprint 079's design, not touched by 094 or 100). ---
  DrivetrainPorts ports() const;
  bool active() const;
  void standby();

 private:
  enum class Mode : uint8_t { NEUTRAL, TWIST, WHEELS };

  // dispatchEscapeHatch -- tick()'s own helper: decides the ring-clearing/
  // segmentMode_ side effects a driveIn command causes BEFORE calling
  // apply() to unpack the oneof into mode_/targets. Returns true if this
  // command PREEMPTS segmentIn's drain for this tick (WHEELS/TWIST/NEUTRAL
  // -- an actual arm was dispatched), false for POSE/NONE. See the class
  // comment's own note on the one documented behavior change vs. the
  // pre-cutover graceful-NEUTRAL-during-a-segment special case.
  bool dispatchEscapeHatch(const msg::DrivetrainCommand& command);

  // Drains ring_ completely (repeated take()s -- Rt::WorkQueue has no
  // clear()).
  void clearRing();

  // startNextPlan -- pops ring_ entries (skipping any that fail plan()'s
  // real solve -- see this method's own body comment) until one plans
  // successfully (planActive_ becomes true, plan_/state_/planStart_ are
  // set) or the ring is exhausted (planActive_ stays false). start pose is
  // plan_.goal() (the just-completed plan's frozen goal, when haveAnchor_)
  // or the current measured pose (the first plan since boot/an abort).
  void startNextPlan(uint32_t now, const msg::PoseEstimate& bodyState);

  // replacePlan -- placement-new "reassignment" of plan_ (Drive::MotionPlan
  // is not copy-assignable -- see the class comment). No heap.
  void replacePlan(const Drive::MotionPlan& newPlan);

  // abortAndFlush -- ABORT_TIMEOUT/ABORT_REPLAN_LIMIT reaction: flush
  // ring_, re-anchor chainTail to `measured`, populate lastEvent_.
  void abortAndFlush(Drive::ChainTail& chainTail, Drive::Status status,
                      const Drive::BodyState& measured, const Drive::TrackRecord& record);

  // rebuildDriveDrivetrain -- reconstructs driveDrivetrain_ from the
  // currently-cached plannerConfig_/config_.trackwidth pair -- called from
  // both configure() and configureMotion(), whichever last changed.
  void rebuildDriveDrivetrain();

  // Computes this Drivetrain's currently-commanded DIRECT-mode wheel
  // velocity targets, BEFORE the ratio governor. UNCHANGED this ticket.
  void commandedWheelTargets(float* targetLeft, float* targetRight) const;

  // Ratio (sync) governor. UNCHANGED MATH this ticket -- see the class
  // comment.
  void governRatio(float* targetLeft, float* targetRight,
                    const msg::MotorState& leftObs,
                    const msg::MotorState& rightObs) const;

  Hardware& hardware_;

  // Drive:: (Level 1) state -- see the class comment for what each member
  // is and why it exists.
  Drive::Drivetrain driveDrivetrain_{Drive::Limits{}, 0.0f};  // rebuilt by configure()/configureMotion()
  Drive::MotionPlan plan_;              // default-constructed = invalid/empty (motion_plan.h)
  Drive::StepState state_;
  uint32_t planStart_ = 0;              // [ms]
  bool planActive_ = false;             // true: plan_/state_ drive this tick's output
  bool haveAnchor_ = false;             // true: plan_.goal() is a valid start-pose anchor
  float nextEntrySpeed_ = 0.0f;         // [mm/s] seeded into the NEXT startNextPlan() call
  uint32_t segSeq_ = 0;                 // monotonic "which plan generation" counter (lastEvent_.seg_seq)
  uint32_t lateSolveFailures_ = 0;      // diagnostic: admit() passed, the real plan() solve did not
  Rt::WorkQueue<Drive::Goal, 8> ring_;  // admitted, not-yet-planned segments
  msg::EventNotify lastEvent_ = {};

  // true: ring_/plan_ drives this tick's staged output (SEGMENT mode).
  // false: mode_/commandedWheelTargets() drives it (DIRECT/escape-hatch
  // mode). Same shape/defaults as before this ticket.
  bool segmentMode_ = false;

  msg::DrivetrainConfig config_ = {};
  msg::PlannerConfig plannerConfig_ = {};  // cached for rebuildDriveDrivetrain()
  Mode mode_ = Mode::NEUTRAL;

  uint32_t boundLeft_ = 0;
  uint32_t boundRight_ = 1;

  // DIRECT/TWIST-arm state. UNCHANGED.
  float v_x_ = 0.0f;      // [mm/s]
  float v_y_ = 0.0f;      // [mm/s] always ignored -- see setTwist()
  float omega_ = 0.0f;    // [rad/s]

  // DIRECT/WHEELS-arm state. UNCHANGED.
  float wheelTargetLeft_ = 0.0f;    // [mm/s]
  float wheelTargetRight_ = 0.0f;   // [mm/s]

  // Last pass's post-governor commanded wheel velocities -- state()/TLM
  // cmd=. UNCHANGED.
  float cmdVel_[2] = {0.0f, 0.0f};   // [mm/s]

  // Remaining master-DOF distance in the live plan [mm] -- state()/rem=.
  // 0.0f for a pivot (no established v2 convention for a pivot's own
  // "remaining translation" -- see drivetrain.cpp's own tick() comment).
  float remainingLinear_ = 0.0f;   // [mm]

  void updateAccelEma(uint32_t now, int wheel, const msg::MotorState& obs);
  float accelEma_[2] = {0.0f, 0.0f};        // [mm/s^2]
  float lastVelSample_[2] = {0.0f, 0.0f};   // [mm/s] last DISTINCT velocity sample
  uint32_t lastVelSampleMs_[2] = {0, 0};    // [ms]
  bool haveVelSample_[2] = {false, false};

  msg::Neutral neutralMode_ = msg::Neutral::BRAKE;

  msg::MotorCapabilities leftMotorCaps_ = {};
  msg::MotorCapabilities rightMotorCaps_ = {};

  bool active_ = false;
};

}  // namespace Subsystems
