// Planner.cpp — Planner subsystem wrapper (renamed from MotionController2 in 060-006).
//
// Composes the existing MotionController by reference.  The existing MotionController
// logic is unchanged.
//
// Design note on configure() / RobotConfig shadow:
//   MotionController holds a const RobotConfig& that was passed at construction
//   (Robot passes &config). Updating motion limits via configure(PlannerConfig)
//   cannot reach that reference because it is const. Planner therefore maintains a
//   local RobotConfig copy (_cfg) that starts as a copy of the original and is
//   updated by configure(). The copy is used by Planner's state/reporting logic;
//   the underlying MotionController still reads limits from its own original cfg.
//
// C++11, no heap allocation in tick(), no virtual dispatch, no STL.

// Sprint 050, Ticket 004: EKFTiny must be included BEFORE any header that
// transitively pulls in tinyekf.h.
#define EKF_N 5
#define EKF_M 2
#include "state/EKFTiny.h"

#include "superstructure/Planner.h"
#include "subsystems/drive/Drive.h"           // Drive definition (for state())
#include "kinematics/BodyKinematics.h"        // inverse() for TIMED/DISTANCE/STREAM
#include "messages/drivetrain.h"              // msg::DrivetrainCommand
#include "messages/planner.h"                 // msg::PlannerCommand
#include "messages/common.h"                  // msg::CommandBatch
#include "types/Inputs.h"                     // HardwareState, MotorCommands, TargetState
#include "types/Config.h"                     // RobotConfig, DriveMode
#include "control/StopCondition.h"            // makeTimeStop, makeDistanceStop, makeHeadingStop
#include <cstring>                            // strncpy
#include <cmath>                              // fabsf

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
Planner::Planner(MotionController& mc,
                 const subsystems::Drive& drive,
                 const RobotConfig& cfg)
    : _mc(mc)
    , _drive(drive)
    , _cfg(cfg)        // local shadow copy — updated by configure()
    , _target(_desired)  // _target is an alias for _desired (TargetState = DesiredState)
{
    // Wire the BVC inside _mc to publish body twist into our _desired so that
    // tick() can read _desired.bodyTwist after driveAdvance().
    //
    // Side-effect note: this redirects the existing BVC publish target.
    // In isolated test use, _mc has not been wired to a live Robot, so this is safe.
    _mc.setBvcStateRef(&_desired);
}

// ---------------------------------------------------------------------------
// apply — stage the goal command.
// Dispatches on PlannerCommand::GoalKind → the appropriate begin*() call.
// ReplyFn is a no-op sink for now; EVT routing comes in ticket 059-003.
// ---------------------------------------------------------------------------
void Planner::apply(const msg::PlannerCommand& cmd)
{
    const uint32_t now = 0;  // apply() is called outside the tick loop;
                             // the actual now_ms is passed in tick().
    // We stage the command into the MC immediately on apply() so that the next
    // tick() call finds the MC in the correct state. The MotionController begin*()
    // calls are idempotent with respect to timing — they just configure the
    // internal MotionCommand; driveAdvance() advances it every tick.
    //
    // Note: corr_id from PlannerCommand::corr_id[] is passed as a string.
    // The MotionController begin*() functions accept a const char* corrId.

    const char* corrId = (cmd.corr_id[0] != '\0') ? cmd.corr_id : nullptr;

    switch (cmd.goal_kind) {

    case msg::PlannerCommand::GoalKind::VELOCITY: {
        // beginVelocity takes (v_mms, omega_rads).
        float v     = cmd.goal.velocity.v_x;
        float omega = cmd.goal.velocity.omega;
        _mc.beginVelocity(v, omega, now, _target, _noopReply, nullptr, corrId);
        break;
    }

    case msg::PlannerCommand::GoalKind::GOTO_GOAL: {
        // beginGoTo takes (tx, ty, speedMms).
        float tx    = cmd.goal.goto_goal.x;
        float ty    = cmd.goal.goto_goal.y;
        float speed = cmd.goal.goto_goal.speed;
        _mc.beginGoTo(tx, ty, speed, now, _target, _noopReply, nullptr, corrId);
        break;
    }

    case msg::PlannerCommand::GoalKind::TURN: {
        // beginTurn takes (headingCdeg, epsCdeg).
        // TurnGoal.heading → convert to centidegrees.
        static constexpr float RAD_TO_CDEG = 18000.0f / 3.14159265f;
        float headingCdeg = cmd.goal.turn.heading * RAD_TO_CDEG;
        float epsCdeg     = 300.0f;  // default 3° tolerance
        _mc.beginTurn(headingCdeg, epsCdeg, now, _target, _noopReply, nullptr, corrId);
        break;
    }

    case msg::PlannerCommand::GoalKind::DISTANCE: {
        // beginDistance takes (leftMms, rightMms, targetMm).
        // DistanceGoal: distance_mm, speed_mmps.
        // Convert straight speed to wheel speeds: L=R=speed.
        float leftMms  = cmd.goal.distance.speed;
        float rightMms = cmd.goal.distance.speed;
        // Negative distance → reverse both wheel directions.
        if (cmd.goal.distance.distance < 0.0f) {
            leftMms  = -leftMms;
            rightMms = -rightMms;
        }
        int32_t targetMm = (int32_t)(cmd.goal.distance.distance);
        if (targetMm < 0) targetMm = -targetMm;  // beginDistance takes unsigned magnitude
        _mc.beginDistance(leftMms, rightMms, targetMm, now, _target, _noopReply, nullptr, corrId);
        break;
    }

    case msg::PlannerCommand::GoalKind::TIMED: {
        // beginTimed takes (leftMms, rightMms, durationMs).
        // TimedGoal: vx_mmps, omega_rads, duration_ms.
        // Convert body twist to differential wheel speeds via inverse kinematics.
        float vL = 0.0f;
        float vR = 0.0f;
        BodyKinematics::inverse(cmd.goal.timed.v_x,
                                cmd.goal.timed.omega,
                                _cfg.trackwidthMm, vL, vR);
        uint32_t durationMs = cmd.goal.timed.duration;
        _mc.beginTimed(vL, vR, durationMs, now, _target, _noopReply, nullptr, corrId);
        break;
    }

    case msg::PlannerCommand::GoalKind::ROTATION: {
        // beginRotation takes (relCdeg).
        // RotationGoal: angle_rad.
        static constexpr float RAD_TO_CDEG = 18000.0f / 3.14159265f;
        float relCdeg = cmd.goal.rotation.angle * RAD_TO_CDEG;
        _mc.beginRotation(relCdeg, now, _target, _noopReply, nullptr, corrId);
        break;
    }

    case msg::PlannerCommand::GoalKind::STREAM: {
        // beginStream takes (leftMms, rightMms).
        // StreamGoal: vx_mmps, vy_mmps (ignored for differential), omega_rads.
        float vL = 0.0f;
        float vR = 0.0f;
        BodyKinematics::inverse(cmd.goal.stream.v_x,
                                cmd.goal.stream.omega,
                                _cfg.trackwidthMm, vL, vR);
        _mc.beginStream(vL, vR, now, _target, _noopReply, nullptr);
        break;
    }

    case msg::PlannerCommand::GoalKind::STOP:
        _mc.stop(now, _noopReply, nullptr);
        break;

    case msg::PlannerCommand::GoalKind::NONE:
    default:
        // No-op.
        break;
    }
}

// ---------------------------------------------------------------------------
// tick — advance goal closure one tick.
//
// Sequence:
//   1. Populate _hw from _drive.state() (fused pose + twist).
//   2. Call _mc.driveAdvance(_hw, _cmds, _target, now).
//   3. Read commanded body twist from _desired.bodyTwist.
//   4. Pack msg::DrivetrainCommand{TWIST} into CommandBatch.
//   5. Update _state.
// ---------------------------------------------------------------------------
msg::CommandBatch Planner::tick(uint32_t now)
{
    // ------------------------------------------------------------------
    // STEP 1: Populate _hw from _drive.state() (fused pose / twist).
    //
    // HardwareState uses legacy ::PoseEstimate / ::BodyTwist3 (source/state/).
    // msg::DrivetrainState uses msg::PoseEstimate / msg::BodyTwist3 (messages/).
    // Both have the same field semantics; copy field-by-field.
    // ------------------------------------------------------------------
    const msg::DrivetrainState& drvState = _drive.state();

    _hw.fused.pose.x              = drvState.get_fused().get_pose().get_x();
    _hw.fused.pose.y              = drvState.get_fused().get_pose().get_y();
    _hw.fused.pose.h              = drvState.get_fused().get_pose().get_h();
    _hw.fused.twist.vx_mmps       = drvState.get_fused().get_twist().get_v_x();
    _hw.fused.twist.vy_mmps       = drvState.get_fused().get_twist().get_v_y();
    _hw.fused.twist.omega_rads    = drvState.get_fused().get_twist().get_omega();

    // Encoder pose (for DISTANCE stop condition evaluation via driveAdvance).
    _hw.encMm[0] = drvState.enc()[0];  // [0]=R
    _hw.encMm[1] = drvState.enc()[1];  // [1]=L

    // Sensor fields (line / color) for SENSOR/LINE_ANY/COLOR stop conditions.
    //
    // 060-004: Planner::_hw is populated solely from drive.state(),
    // which carries no sensor data.  MotionCommand::_stops[].evaluate(inputs, …)
    // passes our private _hw as `inputs`, so without this copy, all line/color
    // stop conditions see zero values and never fire.
    //
    // The authoritative sensor state lives in robot.state.actual (sensors.tick
    // writes there via lineSensor._inputs / colorSensor_._inputs, both of which
    // hold &state.actual).  MotionController::hardwareState() exposes the same
    // pointer (it was set to &state.actual in Robot.cpp).
    //
    // Note: sensors.tick() runs at step 7 (after planner.tick at step 4).
    // This copies the values that sensors.tick() wrote on the PREVIOUS tick,
    // which introduces a one-tick lag for sensor stops.  That lag is acceptable
    // and matches the legacy loop (which also evaluates sensors before
    // sensor_tick's current-tick write in the next pass).
    {
        const HardwareState* live = _mc.hardwareState();
        if (live != nullptr) {
            for (int i = 0; i < 4; ++i) _hw.line[i]  = live->line[i];
            _hw.colorR = live->colorR;
            _hw.colorG = live->colorG;
            _hw.colorB = live->colorB;
            _hw.colorC = live->colorC;
        }
    }

    // ------------------------------------------------------------------
    // STEP 2: Advance goal logic via driveAdvance.
    //
    // _cmds: sink for motor output (discarded — Drive owns the real motor path).
    // _target: our local DesiredState; BVC writes bodyTwist here via setBvcStateRef.
    //
    // Note: if _mc._hwState != nullptr (i.e. the MC is wired to the live Robot's
    // HardwareState), getPoseFloat() inside driveAdvance will read from there
    // rather than _hw.inputs. This is intentional in the live wiring: the live
    // pose is authoritative. In isolated test use, _mc._hwState is null, so
    // driveAdvance falls back to our _hw parameter.
    // ------------------------------------------------------------------
    _mc.driveAdvance(_hw, _cmds, _target, now);

    // ------------------------------------------------------------------
    // STEP 3: Read commanded body twist from _desired.bodyTwist.
    //
    // BVC.advance() writes bodyTwist into _desired (via setBvcStateRef(&_desired)
    // wired in the constructor).  After driveAdvance() returns, _desired.bodyTwist
    // holds the profiled live setpoint: {vx, 0, omega}.
    //
    // 060-004: When the MC is IDLE with no active command (e.g. after stop() /
    // watchdog X / distance complete), driveAdvance() returns without advancing
    // the BVC, leaving _desired.bodyTwist at the last profiled value.  Propagating
    // that stale non-zero twist to Drive keeps the motors running indefinitely.
    // Zero the body twist explicitly so Drive receives {0,0,0} in IDLE.
    // ------------------------------------------------------------------
    float vx    = _desired.bodyTwist.vx_mmps;
    float omega = _desired.bodyTwist.omega_rads;
    if (_mc.mode() == DriveMode::IDLE && !_mc.hasActiveCommand()) {
        vx    = 0.0f;
        omega = 0.0f;
        _desired.bodyTwist = {0.0f, 0.0f, 0.0f};
    }

    // ------------------------------------------------------------------
    // STEP 4: Pack DrivetrainCommand{TWIST} into CommandBatch.
    // ------------------------------------------------------------------
    msg::CommandBatch batch{};

    msg::DrivetrainCommand drvCmd;
    msg::BodyTwist3 twist{};
    twist.v_x    = vx;
    twist.v_y    = 0.0f;
    twist.omega = omega;
    drvCmd.setTwist(twist);

    // CommandBatch.cmds_[] holds OutCommand (verb_id/args encoding).
    // For Phase 3 integration, we use a direct DrivetrainCommand embedding
    // convention: verb_id=1 (TWIST), args[0]=vx, args[1]=vy, args[2]=omega.
    // The command bus dispatcher decodes these.
    if (batch.cmds_count < 8) {
        msg::OutCommand& oc = batch.cmds_[batch.cmds_count++];
        oc.verb_id    = 1;  // TWIST verb
        oc.args_[0]   = vx;
        oc.args_[1]   = 0.0f;
        oc.args_[2]   = omega;
        oc.args_count = 3;
    }
    batch.count = batch.cmds_count;

    // ------------------------------------------------------------------
    // STEP 5: Update _state.
    // ------------------------------------------------------------------
    // Map legacy DriveMode → msg::DriveMode.
    switch (_mc.mode()) {
    case DriveMode::IDLE:      _state.mode = msg::DriveMode::IDLE;     break;
    case DriveMode::STREAMING: _state.mode = msg::DriveMode::STREAMING; break;
    case DriveMode::DISTANCE:  _state.mode = msg::DriveMode::DISTANCE;  break;
    case DriveMode::GO_TO:     _state.mode = msg::DriveMode::GO_TO;     break;
    case DriveMode::VELOCITY:  _state.mode = msg::DriveMode::VELOCITY;  break;
    default:                   _state.mode = msg::DriveMode::IDLE;     break;
    }
    _state.active          = _mc.hasActiveCommand();
    _state.body_twist.v_x    = vx;
    _state.body_twist.v_y    = 0.0f;
    _state.body_twist.omega = omega;

    // Goal position / target fields from _target (DesiredState).
    _state.target_x         = _target.targetXWorld;
    _state.target_y         = _target.targetYWorld;
    _state.target_speed    = _target.targetSpeedMms;
    _state.distance_target  = _target.distanceTargetMm;
    _state.deadline         = _target.deadlineMs;

    return batch;
}

// ---------------------------------------------------------------------------
// configure — store updated planner config (motion limits only).
//
// Updates the local RobotConfig shadow (_cfg) with the motion-only fields from
// the PlannerConfig message. The wrapped MotionController reads limits from its
// own original RobotConfig ref, which is not directly reachable here; the
// intent is to prepare for a future ticket where Planner owns the config flow.
// ---------------------------------------------------------------------------
void Planner::configure(const msg::PlannerConfig& cfg)
{
    _planCfg = cfg;

    // Update the local shadow RobotConfig with the motion-limit fields.
    // These match the mapping in toPlannerConfig() (PlannerConfig.cpp).
    if (cfg.a_max       != 0.0f) _cfg.aMax         = cfg.a_max;
    if (cfg.a_decel     != 0.0f) _cfg.aDecel        = cfg.a_decel;
    if (cfg.v_body_max  != 0.0f) _cfg.vBodyMax      = cfg.v_body_max;
    if (cfg.yaw_rate_max!= 0.0f) _cfg.yawRateMax    = cfg.yaw_rate_max;
    if (cfg.yaw_acc_max != 0.0f) _cfg.yawAccMax     = cfg.yaw_acc_max;
    if (cfg.j_max       != 0.0f) _cfg.jMax          = cfg.j_max;
    if (cfg.yaw_jerk_max!= 0.0f) _cfg.yawJerkMax    = cfg.yaw_jerk_max;
    if (cfg.arrive_tol       != 0.0f) _cfg.arriveTolMm     = cfg.arrive_tol;
    if (cfg.turn_in_place_gate  != 0.0f) _cfg.turnInPlaceGate = cfg.turn_in_place_gate;
    if (cfg.turn_threshold   != 0.0f) _cfg.turnThresholdMm = cfg.turn_threshold;
    if (cfg.done_tol         != 0.0f) _cfg.doneTolMm       = cfg.done_tol;
    if (cfg.min_speed       != 0.0f) _cfg.minSpeedMms     = (int32_t)cfg.min_speed;
}
