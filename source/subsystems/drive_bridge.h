// drive_bridge.h -- boundary conversions between msg:: (wire/blackboard
// types) and Drive:: (source/drive/, self-contained -- sprint 100). This is
// the ONE shared home for every msg::<->Drive:: field mapping the wafer
// adapter (subsystems/drivetrain.cpp, THE CUTOVER, ticket 100-007) and the
// wire admission handler (commands/binary_channel.cpp, the SAME ticket)
// both need -- kept in exactly one place so the mapping can never drift
// between "queue-time admission" (binary_channel.cpp, synchronous, at wire
// time) and "the adapter's own solve" (drivetrain.cpp, per-tick). Header-
// only: every function below is a small, pure, inline value conversion --
// no state, no new translation unit needed, nothing here is a control law
// (source/drive/ owns 100% of the control math -- see architecture-
// update.md (100) M7's "zero control math" boundary).
//
// This file lives OUTSIDE source/drive/ and is explicitly allowed to
// include drive/ headers: source/drive/'s isolation rule (SUC-008) is
// one-directional -- nothing under source/drive/ may reference anything
// outside itself, but code outside source/drive/ referencing INTO it is
// exactly the adapter's whole job (tests/_infra/drive/drive_api.cpp already
// established this same shape for the tier-0 ctypes ABI).
#pragma once

#include "drive/drivetrain.h"
#include "drive/motion_plan.h"
#include "drive/types.h"
#include "messages/common.h"
#include "messages/envelope.h"
#include "messages/motion.h"
#include "messages/motor.h"
#include "messages/planner.h"
#include "runtime/commands.h"

namespace Subsystems {

inline Drive::ProfileLimits driveProfileLimits(float velocity, float accel, float decel,
                                                float jerk) {
  Drive::ProfileLimits p;
  p.velocity = velocity;
  p.accel = accel;
  p.decel = decel;
  p.jerk = jerk;
  return p;
}

// driveLimitsFromConfig -- msg::PlannerConfig -> Drive::Limits, THE single
// conversion point this ticket defines (per its own acceptance criteria:
// "holds a Drive::Drivetrain (immutable config), populated from
// msg::PlannerConfig -> Drive::Limits at configure()"). Field mapping:
//   - linear/rotational ProfileLimits channels reuse PlannerConfig's
//     PRE-EXISTING fields 1-7 (a_max/a_decel/v_body_max/yaw_rate_max/
//     yaw_acc_max/j_max/yaw_jerk_max) -- the SAME fields the retired
//     Motion::JerkTrajectory/SegmentExecutor consumed for the identical
//     linear/rotational split (jerk_trajectory.h's own isRotational
//     branch); the rotational channel is symmetric (accel == decel),
//     yaw_acc_max serving both, exactly matching that file's own
//     established "rotational... needs no such mirroring" precedent.
//   - vWheelMax/trimVMax/trimOmegaMax/wheelStepMax/trackKS/trackKTheta/
//     trackKCross/minSpeed reuse the NEW PlannerConfig fields 15-31
//     architecture-update.md M1 (ticket 100-001) landed, matching
//     Drive::Limits' own field names (types.h) 1:1.
//   - `steer_headroom` (PlannerConfig field 16) has NO Drive::Limits
//     consumer: source/drive/types.h never grew a field for it across
//     tickets 002-004 (the trim-headroom formula Drivetrain::plan() uses
//     is `trimVMax + trimOmegaMax*trackwidth/2`, not steer_headroom) --
//     deliberately not read here, not an oversight.
inline Drive::Limits driveLimitsFromConfig(const msg::PlannerConfig& cfg) {
  Drive::Limits limits;
  limits.linear = driveProfileLimits(cfg.v_body_max, cfg.a_max, cfg.a_decel, cfg.j_max);
  limits.rotational =
      driveProfileLimits(cfg.yaw_rate_max, cfg.yaw_acc_max, cfg.yaw_acc_max, cfg.yaw_jerk_max);
  limits.vWheelMax = cfg.v_wheel_max;
  limits.trimVMax = cfg.trim_v_max;
  limits.trimOmegaMax = cfg.trim_omega_max;
  limits.wheelStepMax = cfg.wheel_step_max;
  limits.trackKS = cfg.track_k_s;
  limits.trackKTheta = cfg.track_k_theta;
  limits.trackKCross = cfg.track_k_cross;
  limits.minSpeed = cfg.min_speed;
  return limits;
}

// driveGoal -- msg::MotionSegment's v2 primitive fields (arc_length/
// delta_heading/exit_speed -- architecture-update.md M1, ticket 100-001)
// -> a pure Drive::Goal. Every LEGACY MotionSegment field (distance/
// direction/final_heading/speed_max/accel_max/jerk_max/yaw_rate_max/
// yaw_accel_max/yaw_jerk_max/time/v/omega/stream) is deliberately NOT read
// here: those fields belong to the RETIRED (pre-cutover) segment/replace
// shape, and a v2 primitive segment has no per-segment override at all --
// Drive::Goal/PlanRequest carry none; Drivetrain::plan() always solves
// against the ONE construction-time Limits (source/drive/drivetrain.cpp).
inline Drive::Goal driveGoal(const msg::MotionSegment& seg) {
  Drive::Goal goal;
  goal.arcLength = seg.arc_length;
  goal.deltaHeading = seg.delta_heading;
  goal.exitSpeed = seg.exit_speed;
  return goal;
}

// driveMoverRequest -- msg::MotionSegment's time/v/omega arm (MOVER's own
// wire shape, fields 10-12 -- protos/motion.proto's "time/velocity arm
// (MOVER teleop primitive)" section) -> Rt::MoverRequest, THE single
// conversion point for MOVER (100-008) -- the `replace`-arm dual of
// driveGoal() above. `v`/`omega` are SIGNED body-frame targets (v_y is
// always 0.0f -- no holonomic drivetrain yet, matching Drive::Twist's own
// doc comment); `time` [ms] passes straight through as
// MoverRequest::deadman, Drive::Drivetrain::planVelocity()'s own second
// argument (source/drive/drivetrain.h) -- both are already [ms], no unit
// conversion needed. Every OTHER MotionSegment field (distance/direction/
// final_heading/speed_max/accel_max/jerk_max/yaw_rate_max/yaw_accel_max/
// yaw_jerk_max/stream/arc_length/delta_heading/exit_speed) is deliberately
// NOT read here -- those belong to either the RETIRED legacy per-segment
// shape or the Goal-shaped arc/pivot primitive `segment` already owns;
// MOVER's own real primitive shape carries no override/goal fields at all,
// the same "v2 primitive has no per-segment override" posture driveGoal()'s
// own doc comment states for its shape.
inline Rt::MoverRequest driveMoverRequest(const msg::MotionSegment& seg) {
  Rt::MoverRequest request;
  request.target = Drive::Twist{seg.v, 0.0f, seg.omega};
  request.deadman = seg.time;
  return request;
}

// driveWheelState -- msg::MotorState (Hardware's per-wheel observation) ->
// Drive::WheelState. An absent Opt<float> (no fresh sample this pass) maps
// to positionValid/velocityValid == false and a 0.0f placeholder value --
// never a fabricated observation (types.h's own WheelState doc comment).
inline Drive::WheelState driveWheelState(const msg::MotorState& obs) {
  Drive::WheelState w;
  w.position = obs.position.has ? obs.position.val : 0.0f;
  w.velocity = obs.velocity.has ? obs.velocity.val : 0.0f;
  w.positionValid = obs.position.has;
  w.velocityValid = obs.velocity.has;
  return w;
}

inline Drive::Pose drivePose(const msg::Pose2D& pose) { return Drive::Pose{pose.x, pose.y, pose.h}; }

// wirePose2D -- the inverse of drivePose() above: Drive::Pose -> msg::Pose2D.
// Needed starting 100-009 (PlanRecord.goal/anchor are both Pose2D, the ONLY
// two places this direction of the conversion is needed -- every other
// msg::<->Drive:: boundary in this file only ever goes msg:: -> Drive::,
// since source/drive/'s outputs before this ticket were all scalar
// WheelVelocities/Status, never a Pose).
inline msg::Pose2D wirePose2D(const Drive::Pose& pose) {
  msg::Pose2D p;
  p.x = pose.x;
  p.y = pose.y;
  p.h = pose.h;
  return p;
}

inline Drive::Twist driveTwist(const msg::BodyTwist3& twist) {
  return Drive::Twist{twist.v_x, twist.v_y, twist.omega};
}

// driveBodyState -- bb.bodyState (msg::PoseEstimate: fused pose + body
// twist, published every pass by MainLoop::commit(), sprint 099-004,
// architecture-update.md (099) Addition 2) -> Drive::BodyState. This is
// StepInput.measured's own source (motion_plan.h) and
// Drivetrain::replan()/planVelocity()'s `measured`/`current` argument.
inline Drive::BodyState driveBodyState(const msg::PoseEstimate& bodyState) {
  Drive::BodyState b;
  b.pose = drivePose(bodyState.pose);
  b.twist = driveTwist(bodyState.twist);
  return b;
}

// toMotionStatus -- Drive::Status -> msg::MotionStatus. The two enums were
// landed (ticket 100-001, protos/motion.proto's MotionStatus) with
// IDENTICAL enumerator names AND ordinal values (RUNNING=0/SETTLING=1/
// REPLAN_DUE=2/DONE_STOP=3/DONE_HANDOFF=4/ABORT_TIMEOUT=5/
// ABORT_REPLAN_LIMIT=6) specifically so this mapping is a pure numeric
// identity, never a lossy translation -- a static_cast, not a switch table.
inline msg::MotionStatus toMotionStatus(Drive::Status status) {
  return static_cast<msg::MotionStatus>(status);
}

// errCodeForVerdict -- every non-OK Drive::Verdict maps to the SAME typed
// wire error, msg::ErrCode::ERR_RANGE (the closest existing ErrCode for
// "the requested motion is infeasible/out of range" -- envelope.proto's
// ErrCode enum was deliberately NOT grown with one enumerator per
// Drive::Verdict by this ticket, a minimal-schema-growth choice noted in
// ticket 100-007's completion notes). The SPECIFIC Drive::Verdict ordinal
// still reaches the client via Error.field (msg::Error{code, field()}),
// so "any other verdict... a specific, distinguishable reason, never a
// silent no-op" (SUC-003's postcondition) holds without a new wire enum.
inline msg::ErrCode errCodeForVerdict(Drive::Verdict verdict) {
  (void)verdict;
  return msg::ErrCode::ERR_RANGE;
}

// drivePlanRecord -- (100-009) Drive::MotionPlan's const query surface ->
// msg::PlanRecord, THE single conversion point for both of this ticket's
// PlanRecord producers: Subsystems::Drivetrain::activePlanRecord() (the
// adapter's own already-solved plan_, `replanCount` = the LIVE state_.
// replanCount) and BinaryChannel::handlePlanDump()'s own throwaway preview
// solves of each still-QUEUED ring_ Goal (`replanCount` = 0 -- a preview
// has never run, so there is no replan history to report). `entry_speed`
// has no direct MotionPlan query (motion_plan.h's own public surface never
// stored the PlanRequest.entrySpeed that seeded the solve) -- it is
// reconstructed as referenceAt(0.0f).v, the master profile's own t=0
// boundary velocity, which the "closed-form, exact" query IS the entry
// speed for every non-pivot plan (a pivot's own v is always literally 0.0f
// regardless of entrySpeed, matching Goal's own "pivot: exitSpeed must be
// 0" convention -- entrySpeed into a pivot is likewise never meaningful).
inline msg::PlanRecord drivePlanRecord(const Drive::MotionPlan& plan, uint32_t replanCount) {
  msg::PlanRecord record;
  record.goal = wirePose2D(plan.goal());
  record.anchor = wirePose2D(plan.anchor());
  record.v_eff = plan.effectiveCeiling();
  record.duration = plan.duration();
  record.exit_speed = plan.exitSpeed();
  record.entry_speed = plan.referenceAt(0.0f).v;
  record.replan_count = replanCount;
  return record;
}

// driveMotionTrace -- (100-009) Drive::TrackRecord -> msg::MotionTrace, THE
// single conversion point for Subsystems::Drivetrain::tick()'s own
// per-pass capture (lastRecord_, committed to bb.motionTrace every pass
// plan_.step() runs -- source/subsystems/drivetrain.cpp). A CURATED subset
// per MotionTrace's own doc comment (envelope.proto): every StepInput
// replay field EXCEPT measured/left/right (Telemetry's own pose=/twist=/
// enc=/vel= already carry those at the same period -- a developer's
// tooling zips the two same-period frames back into a full StepInput, see
// envelope.proto's MotionTrace doc comment), the sampled RefState's
// world-frame point, tracked error, post-clamp trims, the body command,
// the final wheel setpoints, and `segSeq` (NOT part of TrackRecord itself
// -- threaded in by the caller, the adapter's own segSeq_ generation
// counter, so a client can tell which segment a streamed sample belongs to
// without a separate PlanRecord round-trip).
inline msg::MotionTrace driveMotionTrace(const Drive::TrackRecord& record, uint32_t segSeq) {
  msg::MotionTrace trace;
  trace.t = record.in.t;
  trace.ref_x = record.ref.x;
  trace.ref_y = record.ref.y;
  trace.ref_theta = record.ref.theta;
  trace.ref_v = record.ref.v;
  trace.ref_omega = record.ref.omega;
  trace.e_along = record.eAlong;
  trace.e_cross = record.eCross;
  trace.e_theta = record.eTheta;
  trace.v_trim = record.vTrim;
  trace.omega_trim = record.omegaTrim;
  trace.v_cmd = record.vCmd;
  trace.omega_cmd = record.omegaCmd;
  trace.wheel_left = record.wheelLeft;
  trace.wheel_right = record.wheelRight;
  trace.pose_step = record.in.poseStep;
  trace.pose_step_theta = record.in.poseStepTheta;
  trace.seg_seq = segSeq;
  trace.trim_saturated = record.trimSaturated;
  trace.status = toMotionStatus(record.status);
  return trace;
}

}  // namespace Subsystems
