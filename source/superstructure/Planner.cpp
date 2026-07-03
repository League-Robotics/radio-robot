// Planner.cpp — Planner subsystem.
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
#include "control/MotorController.h"          // MotorController (for beginDistance)
#include "control/StopCondition.h"            // makeTimeStop, makeDistanceStop, makeHeadingStop
#include "state/PhysicalStateEstimate.h"      // getPose (044-001 seam)
#include <cstring>                            // strncpy
#include <cmath>                              // fabsf, sqrtf, cosf, sinf

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
Planner::Planner(MotorController& mc_ctrl, Odometry& odo,
                 const subsystems::Drive& drive,
                 const RobotConfig& cfg)
    : _mc_ctrl(mc_ctrl)
    , _odo(odo)
    , _cfg(cfg)           // live reference — binds to Robot's single RobotConfig
    , _hwState(nullptr)
    , _robot(nullptr)     // set later by setRobotCtx()
    , _bvc(mc_ctrl, cfg)  // _bvc must be initialised before _activeCmd
    , _activeCmd()
    , _safeOneShotDisable(false)
    , _mode(DriveMode::IDLE)
    , _tgtL(0.0f)
    , _tgtR(0.0f)
    , _dDistTarget(0.0f)
    , _dOmega(0.0f)
    , _dEnc0(0.0f)
    , _gPhase(GPhase::IDLE)
    , _gTargetXWorld(0.0f)
    , _gTargetYWorld(0.0f)
    , _gSpeed(0.0f)
    , _pursueBacktrackTicks(0)
    , _lastTickMs(0)
    , _currentTimeMs(0)
    , _drive(drive)
    , _hw{}
    , _cmds{}
    , _desired{}
    , _target(_desired)   // _target is an alias for _desired (TargetState = DesiredState)
    , _state{}
    , _planCfg{}
{
    // Wire the BVC to publish body twist into our _desired so that
    // tick() can read _desired.bodyTwist after driveAdvance().
    _bvc.setStateRef(&_desired);
}

// ---------------------------------------------------------------------------
// emitEvt — inline EVT emission via the MotionEventSink stored in target.
//
// Calls target.sink.emitFn(base, corrId, ctx) if set.
// Clears target.corrId after emitting so a subsequent completion on the
// same target does not re-use a stale id.
//
// Sprint 026-002: calls through MotionEventSink rather than formatting inline.
// ---------------------------------------------------------------------------

/*static*/ void Planner::emitEvt(const char* base, TargetState& target)
{
    if (target.sink.emitFn) {
        target.sink.emitFn(base, target.corrId, target.sink.ctx);
    }
    target.corrId[0] = '\0';  // consumed
}

void Planner::stop(uint32_t now_ms, ReplyFn fn, void* ctx)
{
    // Cancel any active MotionCommand before calling fullStop().
    if (_activeCmd.active()) {
        _activeCmd.cancel(MotionCommand::StopStyle::HARD);
    }
    _gPhase = GPhase::IDLE;  // reset G phase on hard stop
    fullStop(fn, ctx);
    (void)now_ms;
}

void Planner::cancel(uint32_t now_ms, ReplyFn fn, void* ctx)
{
    _activeCmd.cancel(MotionCommand::StopStyle::HARD);
    _mc_ctrl.stop();
    _mode   = DriveMode::IDLE;
    _gPhase = GPhase::IDLE;  // reset G phase on any cancel
    (void)now_ms;
    (void)fn;
    (void)ctx;
}

void Planner::disableSafetyOneShot()
{
    _safeOneShotDisable = true;
}

void Planner::softStop(uint32_t now_ms)
{
    if (_activeCmd.active()) {
        // Active MotionCommand: arm its SOFT ramp-down.
        // tick() will advance BVC toward (0,0) and emit EVT done when converged.
        _activeCmd.softStop(now_ms);
    } else {
        // No active MotionCommand (STREAMING or IDLE mode): just set BVC target
        // to (0,0) and let the profiler ramp down.  No EVT done in this case.
        _bvc.setTarget(0.0f, 0.0f);
    }
}

// ---------------------------------------------------------------------------
// driveAdvance — cooperative-loop task entry point (014-005).
//
// Runs at a fixed period set by RobotConfig::controlPeriod (default 10 ms).
// Executes the drive-mode state machines:
//   1. STREAMING watchdog — emits EVT safety_stop inline on keepalive timeout.
//   2. G-mode — advances PRE_ROTATE and PURSUE; emits EVT done G inline.
//   T/D-mode are now handled by the MotionCommand path (TIME/DISTANCE stop conditions).
//
// All EVT completions are emitted inline via target.sink.emitFn() (sprint 026-002)
// — safe because there is no fiber boundary in the single cooperative main loop.
//
// NOTE: OTOS correction is NOT done here.  It is the sole responsibility of
// Robot::otosCorrect() called at the slow cadence in LoopScheduler
// (ticket 005 wired this; ticket 006 moved it to the scheduler task).
// ---------------------------------------------------------------------------

void Planner::driveAdvance(HardwareState& inputs, MotorCommands& cmds,
                           TargetState& target, uint32_t now_ms)
{
    // Throttle to controlPeriod cadence.
    if ((now_ms - _lastTickMs) < (uint32_t)_cfg.controlPeriod) return;

    float dt_s      = (float)(now_ms - _lastTickMs) / 1000.0f;
    _lastTickMs     = now_ms;
    _currentTimeMs  = now_ms;

    // Motor controller tick and odometry predict are called by Robot::controlCollectSplitPhase()
    // and odometry.predict() before driveAdvance() is reached (014-003/004).
    (void)cmds;

    // ── MotionCommand tick (VW / R / G PURSUE / future MotionCommand-based verbs) ─
    // When a MotionCommand is active, tick it exactly once and return early.
    // The old S/T/D/G if-chain runs ONLY when no MotionCommand is active.
    // This also prevents the STREAMING watchdog branch below from firing for VW.
    if (_activeCmd.active()) {
        // G PURSUE hook: recompute (v, ω) from current pose each tick and call
        // setTarget BEFORE _activeCmd.tick() so the BVC advances with the
        // updated target this tick.
        if (_mode == DriveMode::GO_TO && _gPhase == GPhase::PURSUE) {
            float x, y, h_rad;
            getPoseFloat(x, y, h_rad);

            float dxW = _gTargetXWorld - x;
            float dyW = _gTargetYWorld - y;
            float dx  =  dxW * cosf(h_rad) + dyW * sinf(h_rad);
            float dy  = -dxW * sinf(h_rad) + dyW * cosf(h_rad);

            float d2          = dx * dx + dy * dy;
            float d_remaining = sqrtf(d2);

            // Re-gate counter (D8 027-004): if the target is behind the robot
            // for 3 consecutive ticks, cancel PURSUE and restart PRE_ROTATE.
            // fabsf(bearing) > π/2 means dx < 0 (target behind robot-frame x axis).
            float bearing_rf = atan2f(dy, dx);
            if (fabsf(bearing_rf) > 1.5707963f) {  // π/2
                if (++_pursueBacktrackTicks >= 3) {
                    _pursueBacktrackTicks = 0;
                    // N11 fix (030-009): use cancelQuiet() instead of cancel() to
                    // suppress the spurious "EVT cancelled #<corrId>" that would
                    // otherwise be emitted for the G command's correlation id.
                    // The G command is still in progress (transitioning back to
                    // PRE_ROTATE), so emitting "EVT cancelled" for the G's corrId
                    // falsely signals to the host that the G has failed.
                    _activeCmd.cancelQuiet();
                    _startPreRotate(bearing_rf, _gSpeed, now_ms, target);
                    return;
                }
            } else {
                _pursueBacktrackTicks = 0;
            }

            // Terminal decel cap: v_cap = sqrt(2 * aDecel * d_remaining).
            // Clamps the commanded speed to ensure the BVC has time to
            // decelerate to zero before the POSITION stop fires.
            float v     = _gSpeed;
            float v_cap = sqrtf(2.0f * _cfg.aDecel * d_remaining);
            if (v_cap < v) v = v_cap;

            // Curvature clamp (D8 027-004): bound κ so passing abeam the target
            // (small d, dy ≠ 0) cannot drive ω into a tight orbit.
            // kappaMax = 2 / max(d_remaining, 2·arriveTolerance) limits the turning
            // radius to at most 0.5·arriveTolerance at the tightest point.
            float kappaMax = 2.0f / fmaxf(d_remaining,
                                          2.0f * _cfg.arriveTolerance);
            float kappa = (d2 > 0.1f)
                ? fmaxf(-kappaMax, fminf(kappaMax, 2.0f * dy / d2))
                : 0.0f;
            float omega = v * kappa;

            _activeCmd.setTarget(v, omega);
        }

        // D decel hook: clamp commanded speed downward as the robot nears
        // the distance target.  Computes d_remaining from the raw encoder
        // average (same field used by the DISTANCE stop condition in
        // StopCondition::evaluate) so the decel profile and the stop fire
        // at the same point.  Only clamps downward; does not increase speed.
        if (_mode == DriveMode::DISTANCE) {
            // Array convention: [0]=R (FR), [1]=L (FL) — see ActualState.h.
            float enc_avg     = (inputs.encMm[1] + inputs.encMm[0]) * 0.5f;
            float d_traveled  = fabsf(enc_avg - _dEnc0);
            float d_remaining = _dDistTarget - d_traveled;
            if (d_remaining > 0.0f) {
                float v_cap = sqrtf(2.0f * _cfg.aDecel * d_remaining);
                if (v_cap < _bvc.targetV()) {
                    _activeCmd.setTarget(v_cap, _dOmega);
                }
            }
        }

        bool still_running = _activeCmd.tick(inputs, now_ms, dt_s);
        if (!still_running) {
            // MotionCommand terminated.
            //
            // PRE_ROTATE special case (sprint 024-001): when the PRE_ROTATE
            // MotionCommand finishes, check the current bearing.
            //   - bearing <= gateRad (HEADING stop fired) → transition to PURSUE.
            //   - bearing >  gateRad (TIME net fired)     → runaway; emit "EVT done G"
            //     and go IDLE so the caller gets a clean terminal event.
            if (_mode == DriveMode::GO_TO && _gPhase == GPhase::PRE_ROTATE) {
                float x, y, h_rad;
                getPoseFloat(x, y, h_rad);
                float dxW   = _gTargetXWorld - x;
                float dyW   = _gTargetYWorld - y;
                float dx_rf =  dxW * cosf(h_rad) + dyW * sinf(h_rad);
                float dy_rf = -dxW * sinf(h_rad) + dyW * cosf(h_rad);
                float bearingNow = fabsf(atan2f(dy_rf, dx_rf));
                float gateRad    = _cfg.turnInPlaceGate * (3.14159265f / 180.0f);

                if (bearingNow <= gateRad) {
                    // HEADING stop fired → start the PURSUE phase.
                    // Compute distance for the PURSUE TIME net.
                    float distanceMm     = sqrtf(dxW * dxW + dyW * dyW);
                    float pursueSpd      = (_gSpeed > 1.0f) ? _gSpeed : 1.0f;
                    float pursueTimeoutMs = 2.0f * (distanceMm / pursueSpd) * 1000.0f + 4000.0f;

                    _bvc.reset();
                    _activeCmd.configure(_gSpeed, 0.0f, &_bvc);
                    _activeCmd.addStop(makePositionStop(_gTargetXWorld, _gTargetYWorld,
                                                       _cfg.arriveTolerance));
                    _activeCmd.addStop(makeTimeStop(pursueTimeoutMs));
                    _activeCmd.setReplySink(target.replyFn, target.replyCtx, target.corrId);
                    _activeCmd.setStopStyle(MotionCommand::StopStyle::SOFT);
                    _activeCmd.setDoneEvt("EVT done G");
                    const HardwareState& hw = _hwState ? *_hwState : inputs;
                    _activeCmd.start(hw, now_ms);
                    _gPhase = GPhase::PURSUE;
                    // Do NOT go IDLE; PURSUE is now active.
                    return;
                } else {
                    // TIME net fired (runaway spin): emit terminal EVT and go IDLE.
                    // _activeCmd had no reply sink, so we emit directly here.
                    emitEvt("EVT done G", target);
                    _mc_ctrl.stop();
                    _bvc.reset();
                    _mode = DriveMode::IDLE;
                    target.mode = DriveMode::IDLE;
                    _gPhase = GPhase::IDLE;
                    return;
                }
            }

            // Normal completion (non-PRE_ROTATE): stop motors, reset, go IDLE.
            // Without this the last BVC wheel target persists and the motor PID
            // keeps driving it forever (runaway), since IDLE mode no longer
            // advances the BVC to write fresh (zero) setpoints. _mc_ctrl.stop()
            // zeros tgtLMms/tgtRMms and resets the PID, so driving=false next tick.
            _mc_ctrl.stop();
            _bvc.reset();
            _mode = DriveMode::IDLE;
            target.mode = DriveMode::IDLE;
            // Reset G phase so a subsequent go-to command starts clean.
            if (_gPhase != GPhase::IDLE) _gPhase = GPhase::IDLE;
        }
        return;
    }

    // S-mode keepalive watchdog has been removed from driveAdvance.
    // The system watchdog in LoopScheduler now handles keepalive enforcement for
    // all modes (STREAMING, VELOCITY, etc.) — it fires EVT safety_stop + X after
    // sTimeout of inbound command silence (Sprint 020, Ticket 005).
    (void)inputs;

    // ── BVC tick for STREAMING mode ─────────────────────────────────────────
    // STREAMING mode sets BVC targets but does not have an active MotionCommand
    // to call _bvc.advance(). Tick the BVC directly here so the profiler
    // advances and wheel setpoints are written every control period.
    //
    // PRE_ROTATE is no longer ticked here (sprint 024-001): it now runs via
    // _activeCmd (which calls _bvc.advance() internally in tick()), so the
    // PRE_ROTATE branch was removed to prevent double-ticking the BVC.
    if (_mode == DriveMode::STREAMING) {
        _bvc.advance(dt_s);
    }

    // G-mode: no additional state-machine branches needed here.
    // PRE_ROTATE now runs via _activeCmd (handled in the block above).
    // PURSUE runs via _activeCmd (handled in the block above).
    // Both phases are driven entirely by the MotionCommand path at the top
    // of driveAdvance — control never reaches here while GO_TO is active
    // with a running _activeCmd.
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void Planner::fullStop(ReplyFn fn, void* ctx)
{
    _mc_ctrl.stop();
    _mode  = DriveMode::IDLE;
    _tgtL  = 0.0f;
    _tgtR  = 0.0f;
    (void)fn;
    (void)ctx;
}

/**
 * Read the current odometry pose and convert to floating-point values.
 *
 * @param x      Output: x position in mm (float)
 * @param y      Output: y position in mm (float)
 * @param h_rad  Output: heading in radians
 */
void Planner::getPoseFloat(float& x, float& y, float& h_rad) const {
    if (_hwState == nullptr) {
        x = 0.0f; y = 0.0f; h_rad = 0.0f;
        return;
    }
    int32_t xi, yi, hi;
    // 044-001: read pose through the PhysicalStateEstimate seam. getPose is a
    // static forwarder to Odometry::getPose (same fused-pose fields), so
    // the returned pose is byte-identical to the prior direct Odometry call.
    // 070-003: narrowed to take the one PoseEstimate sub-struct it reads,
    // instead of the whole HardwareState.
    PhysicalStateEstimate::getPose(_hwState->fused, xi, yi, hi);
    x     = static_cast<float>(xi);
    y     = static_cast<float>(yi);
    h_rad = static_cast<float>(hi) * (3.14159265f / 18000.0f);  // cdeg → rad
}

// ---------------------------------------------------------------------------
// apply — stage the goal command.
// Dispatches on PlannerCommand::GoalKind → the appropriate begin*() call.
// ReplyFn is a no-op sink; EVT routing comes via the command bus.
//
// now_ms is the caller-supplied real system time at the point apply() is
// called (CR-11 fix). It is threaded straight through to every begin*() call,
// which passes it on to MotionCommand::start() to baseline MotionBaseline.t0Ms.
// Previously this was a hard-coded local `now = 0`, so t0Ms was always 0 —
// any TIME stop then computed elapsed = now_ms - 0 = full uptime and fired on
// the very next tick() once uptime exceeded the stop's duration, instead of
// after the goal's actual duration had elapsed.
// ---------------------------------------------------------------------------
void Planner::apply(const msg::PlannerCommand& cmd, uint32_t now_ms)
{
    // We stage the command into the Planner immediately on apply() so that the
    // next tick() call finds the planner in the correct state. The begin*()
    // calls are idempotent with respect to timing — they just configure the
    // internal MotionCommand; driveAdvance() advances it every tick.
    //
    // Note: corr_id from PlannerCommand::corr_id[] is passed as a string.
    // The begin*() functions accept a const char* corrId.

    const char* corrId = (cmd.corr_id[0] != '\0') ? cmd.corr_id : nullptr;

    switch (cmd.goal_kind) {

    case msg::PlannerCommand::GoalKind::VELOCITY: {
        // beginVelocity takes (v_mms, omega_rads).
        float v     = cmd.goal.velocity.v_x;
        float omega = cmd.goal.velocity.omega;
        beginVelocity(v, omega, now_ms, _target, _noopReply, nullptr, corrId);
        break;
    }

    case msg::PlannerCommand::GoalKind::GOTO_GOAL: {
        // beginGoTo takes (tx, ty, speedMms).
        float tx    = cmd.goal.goto_goal.x;
        float ty    = cmd.goal.goto_goal.y;
        float speed = cmd.goal.goto_goal.speed;
        beginGoTo(tx, ty, speed, now_ms, _target, _noopReply, nullptr, corrId);
        break;
    }

    case msg::PlannerCommand::GoalKind::TURN: {
        // beginTurn takes (headingCdeg, epsCdeg).
        // TurnGoal.heading → convert to centidegrees.
        static constexpr float RAD_TO_CDEG = 18000.0f / 3.14159265f;
        float headingCdeg = cmd.goal.turn.heading * RAD_TO_CDEG;
        float epsCdeg     = 300.0f;  // default 3° tolerance
        beginTurn(headingCdeg, epsCdeg, now_ms, _target, _noopReply, nullptr, corrId);
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
        beginDistance(leftMms, rightMms, targetMm, now_ms, _target, _noopReply, nullptr, corrId);
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
                                _cfg.trackwidth, vL, vR);
        uint32_t durationMs = cmd.goal.timed.duration;
        beginTimed(vL, vR, durationMs, now_ms, _target, _noopReply, nullptr, corrId);
        break;
    }

    case msg::PlannerCommand::GoalKind::ROTATION: {
        // beginRotation takes (relCdeg).
        // RotationGoal: angle_rad.
        static constexpr float RAD_TO_CDEG = 18000.0f / 3.14159265f;
        float relCdeg = cmd.goal.rotation.angle * RAD_TO_CDEG;
        beginRotation(relCdeg, now_ms, _target, _noopReply, nullptr, corrId);
        break;
    }

    case msg::PlannerCommand::GoalKind::STREAM: {
        // beginStream takes (leftMms, rightMms).
        // StreamGoal: vx_mmps, vy_mmps (ignored for differential), omega_rads.
        float vL = 0.0f;
        float vR = 0.0f;
        BodyKinematics::inverse(cmd.goal.stream.v_x,
                                cmd.goal.stream.omega,
                                _cfg.trackwidth, vL, vR);
        beginStream(vL, vR, now_ms, _target, _noopReply, nullptr);
        break;
    }

    case msg::PlannerCommand::GoalKind::STOP:
        stop(now_ms, _noopReply, nullptr);
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
//   2. Call driveAdvance(_hw, _cmds, _target, now).
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
    // hold &state.actual).  hardwareState() exposes the same pointer
    // (it was set to &state.actual in Robot.cpp).
    //
    // Note: sensors.tick() runs at step 7 (after planner.tick at step 4).
    // This copies the values that sensors.tick() wrote on the PREVIOUS tick,
    // which introduces a one-tick lag for sensor stops.  That lag is acceptable
    // and matches the legacy loop (which also evaluates sensors before
    // sensor_tick's current-tick write in the next pass).
    {
        const HardwareState* live = _hwState;
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
    // ------------------------------------------------------------------
    driveAdvance(_hw, _cmds, _target, now);

    // ------------------------------------------------------------------
    // STEP 3: Read commanded body twist from _desired.bodyTwist.
    //
    // BVC.advance() writes bodyTwist into _desired (via setBvcStateRef(&_desired)
    // wired in the constructor).  After driveAdvance() returns, _desired.bodyTwist
    // holds the profiled live setpoint: {vx, 0, omega}.
    //
    // 060-004: When the planner is IDLE with no active command (e.g. after stop()
    // / watchdog X / distance complete), driveAdvance() returns without advancing
    // the BVC, leaving _desired.bodyTwist at the last profiled value.  Propagating
    // that stale non-zero twist to Drive keeps the motors running indefinitely.
    // Zero the body twist explicitly so Drive receives {0,0,0} in IDLE.
    // ------------------------------------------------------------------
    float vx    = _desired.bodyTwist.vx_mmps;
    float omega = _desired.bodyTwist.omega_rads;
    if (_mode == DriveMode::IDLE && !_activeCmd.active()) {
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
    switch (_mode) {
    case DriveMode::IDLE:      _state.mode = msg::DriveMode::IDLE;     break;
    case DriveMode::STREAMING: _state.mode = msg::DriveMode::STREAMING; break;
    case DriveMode::DISTANCE:  _state.mode = msg::DriveMode::DISTANCE;  break;
    case DriveMode::GO_TO:     _state.mode = msg::DriveMode::GO_TO;     break;
    case DriveMode::VELOCITY:  _state.mode = msg::DriveMode::VELOCITY;  break;
    default:                   _state.mode = msg::DriveMode::IDLE;     break;
    }
    _state.active          = _activeCmd.active();
    _state.body_twist.v_x    = vx;
    _state.body_twist.v_y    = 0.0f;
    _state.body_twist.omega = omega;

    // Goal position / target fields from _target (DesiredState).
    _state.target_x         = _target.targetXWorld;
    _state.target_y         = _target.targetYWorld;
    _state.target_speed    = _target.targetSpeed;
    _state.distance_target  = _target.distanceTarget;
    _state.deadline         = _target.deadline;

    return batch;
}

// ---------------------------------------------------------------------------
// configure — store updated planner config (motion limits only).
//
// 067-001: _cfg is now a live `const RobotConfig&` bound to Robot's single
// RobotConfig, so it already reflects every committed SET — the whitelist
// patch this function used to apply into a private _cfg copy would no
// longer compile against a const reference, and is no longer needed.
// _planCfg is retained as a stored snapshot of the projected PlannerConfig
// (confirmed-dead — never read anywhere — but left in place; see
// architecture-update.md Design Rationale Decision 4).
// ---------------------------------------------------------------------------
void Planner::configure(const msg::PlannerConfig& cfg)
{
    _planCfg = cfg;
}
