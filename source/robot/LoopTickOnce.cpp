#include "LoopTickOnce.h"
#include "Robot.h"
#include "CommandProcessor.h"
#include "Inputs.h"
#include "HaltController.h"
#include "MotionCommand.h"
#include "robot/BusDrain.h"
#include "subsystems/drive/Drive.h"
#include "superstructure/Planner.h"
#include <cstdio>
#include <cmath>   // fmaxf, fabsf

// ---------------------------------------------------------------------------
// loopTickOnce — one iteration of the firmware cooperative loop body.
//
// Called by:
//   - LoopScheduler::run_blocks()  (firmware, after controlCollectSplitPhase)
//   - sim_tick()                   (sim, after hal.tick() + controlCollectSplitPhase)
//
// See LoopTickOnce.h for the full contract.
//
// ---------------------------------------------------------------------------
// ORDERED-TICK SEQUENCE (060-005) — eight-step message-driven path.
//
// This is the sole control path; the legacy loop was deleted in 060-005.
//
//   1. (COMMS DRAIN — presently empty; drive.periodic() crutch removed in 060-001)
//   2. drive2.tickUpdate(now)   — SENSE: encoders + EKF predict + OTOS
//  2b. Sync robot.state.actual from drive2.state() (legacy command handlers
//      read state.actual; drive2 is the authoritative source — see step 2b below)
//   3. BUS DRAIN        — safety + dequeueOne + drainCommandBatch
//  3b. planner.syncWireContext(state.desired) — propagate live reply sink
//   4. planner.tick(now)        — advance goal; returns CommandBatch{TWIST}
//   5. BUS DRAIN        — route planner batch → drive2.apply
//   6. drive2.tickAction(now)   — ACT: BVC → wheel PID → motor output
//  6b. hal.tick(now, drive2.outputs()) — deliver motor commands to plant
//   7. sensors.tick(now) + ports.periodic(ts, now) — timed sensor reads
//   8. TELEMETRY        — emit TLM frame when period elapsed
// ---------------------------------------------------------------------------

void loopTickOnce(Robot& robot, CommandProcessor& cmd, CommandQueue& queue,
                  LoopTickState& ts, uint32_t now)
{
    const RobotConfig& cfg = robot.config;

    // =========================================================
    // STEP 2 — drive2.tickUpdate(now): SENSE
    //
    // Runs the Drive2 outlier filter + EKF predict + OTOS
    // correction on Drive2's private _hw.  The Drive2 estimate
    // is the authoritative source in the ordered-tick path.
    // =========================================================
    // fuseOtos: bypass Drive2's internal OTOS lag gate when ts.fuseOtos is set
    // (mirrors the ts.fuseOtos bypass in the legacy path; used by sim tests that
    // call sim_set_otos_fusion(1) to force per-tick OTOS updates).
    robot.drive.tickUpdate(now, ts.fuseOtos);

    // =========================================================
    // STEP 2b — Sync robot.state.actual from drive2.state()
    //
    // 060-004: legacy command handlers (GET VEL, DBG OTOS, HALT
    // conditions, the D-decel hook in driveAdvance, stop conditions
    // in HaltController::evaluate / StopCondition::evaluate, and
    // SystemCommands::handleZero) all read robot.state.actual.
    // In the ordered-tick path, drive2 is the authoritative source
    // for encoder, velocity, and pose data.  Sync state.actual from
    // drive2.state() after tickUpdate so legacy code sees live values.
    //
    // This sync runs once per tick, keeping state.actual consistent
    // with the most recent SENSE phase.
    // =========================================================
    {
        const msg::DrivetrainState& ds = robot.drive.state();
        // Per-wheel encoder accumulator (mm).  [0]=R (FR), [1]=L (FL).
        robot.state.actual.encMm[0] = ds.enc()[0];
        robot.state.actual.encMm[1] = ds.enc()[1];
        // Per-wheel velocity (mm/s).
        robot.state.actual.velMms[0] = ds.vel()[0];
        robot.state.actual.velMms[1] = ds.vel()[1];
        // Fused pose (EKF output).
        robot.state.actual.fused.pose.x   = ds.fused.pose.x;
        robot.state.actual.fused.pose.y   = ds.fused.pose.y;
        robot.state.actual.fused.pose.h   = ds.fused.pose.h;
        // Fused twist (EKF body-frame velocity).
        robot.state.actual.fused.twist.vx_mmps    = ds.fused.twist.v_x;
        robot.state.actual.fused.twist.vy_mmps    = ds.fused.twist.v_y;
        robot.state.actual.fused.twist.omega_rads = ds.fused.twist.omega;
        // Encoder pose (dead-reckoning only, no OTOS).
        robot.state.actual.encoder.pose.x = ds.encoder.pose.x;
        robot.state.actual.encoder.pose.y = ds.encoder.pose.y;
        robot.state.actual.encoder.pose.h = ds.encoder.pose.h;
        // Optical pose (raw OTOS reading, pre-EKF).
        robot.state.actual.optical.pose.x = ds.optical.pose.x;
        robot.state.actual.optical.pose.y = ds.optical.pose.y;
        robot.state.actual.optical.pose.h = ds.optical.pose.h;
        // OTOS freshness envelope.
        robot.state.actual.otos.lagMs      = ds.otos.lag;
        robot.state.actual.otos.lastUpdMs  = ds.otos.last_upd;
        robot.state.actual.otos.valid      = ds.otos.valid;
    }

    // =========================================================
    // STEP 3 — BUS DRAIN: safety + dequeueOne + motion verbs
    //
    // Safety is evaluated first (keepalive watchdog + halt
    // controller), then one enqueued command is dispatched
    // (dequeueOne), matching the legacy ordering.  The comms
    // batch from step 1 would normally arrive here; in the
    // current implementation the comms drain produces no
    // explicit CommandBatch, so we run evaluateSafety +
    // dequeueOne directly.
    // =========================================================
    robot.superstructure.evaluateSafety(cmd, queue, ts, robot.state.actual, now);
    cmd.dequeueOne(queue);

    // =========================================================
    // STEP 3b — sync wire reply context into planner
    //
    // 060-004: G/TURN/D wire commands are dispatched by
    // dequeueOne → handleGoTo/handleTurn/handleDistance which
    // call mc.beginGoTo/beginTurn/beginDistance(... replyFn ...).
    // Those calls store the real replyFn into robot.state.desired.
    // MC2's _desired uses _noopReply (set via MC2::apply()).
    // driveAdvance() line-275 calls:
    //   _activeCmd.setReplySink(target.replyFn, ...)
    // where target = MC2._desired — overwriting the real replyFn
    // stored in _activeCmd from beginGoTo with _noopReply.
    // syncWireContext copies the live replyFn/replyCtx/corrId
    // from state.desired into MC2._desired so driveAdvance emits
    // EVT done G / EVT done TURN on the correct channel.
    // =========================================================
    robot.planner.syncWireContext(robot.state.desired);

    // =========================================================
    // STEP 4 — planner.tick(now): advance goal state machine
    //
    // MotionController2::tick() calls _mc.driveAdvance() with
    // its own _hw (populated from drive2.state()) and returns a
    // CommandBatch containing a DrivetrainCommand{TWIST}.
    // =========================================================
    msg::CommandBatch plannerBatch = robot.planner.tick(now);

    // =========================================================
    // STEP 5 — BUS DRAIN: route planner batch → drive2.apply
    //
    // drainCommandBatch routes the TWIST OutCommand to
    // drive2.apply(DrivetrainCommand{TWIST}).
    // =========================================================
    drainCommandBatch(plannerBatch, robot.drive, robot.planner, queue, cmd);

    // =========================================================
    // STEP 6 — drive2.tickAction(now): ACT
    //
    // Applies the staged DrivetrainCommand via BVC → wheel PID
    // → motor output.  Drive's BVC is separate from Planner's internal BVC.
    // =========================================================
    robot.drive.tickAction(now);

    // =========================================================
    // STEP 6b — HAL ACTUATOR TICK
    //
    // The HAL tick delivers the motor commands to the plant.
    // 060-002: Drive2's constructor binds MotorController to
    // drive2._outputs (setCommandsRef(&_outputs)), and Robot.cpp
    // no longer overrides that binding in the ordered-tick path.
    // So the buffer the MotorController actually wrote to is
    // drive2._outputs — pass drive2.outputs() here.
    // =========================================================
    robot.hal.tick(now, robot.drive.outputs());

    // =========================================================
    // STEP 7 — sensors.tick(now): timed line/color reads
    //
    // sensors.tick() is the SOLE sensor-schedule authority in the
    // ordered-tick path.  It drives both sensor reads when their
    // lag gates (_lastLineTick / _lastColorTick in Sensors.h) fire,
    // independent of LoopTickState.lastLine / lastColor (which are
    // NOT read or written here).  lineSensor.periodic() and
    // colorSensor_.periodic() are NOT called in this path.
    // =========================================================
    robot.sensors.tick(now);
    // ports.periodic: Ports is not yet wrapped in a Ports2 facade;
    // keep it here until a Ports2 subsystem replaces it.
    robot.ports.periodic(ts, now);

    // =========================================================
    // STEP 8 — TELEMETRY
    //
    // 060-001: buildTlmFrame reads from drive2.state() and
    // sensors.state() — robot.state.actual is no longer required.
    // _tlmBoundFn / _tlmBoundCtx: the reply channel bound by the
    // last STREAM command (set in SystemCommands::handleStream).
    // telemetryEmit guards fn == nullptr, so STREAM not issued →
    // no emission.
    // =========================================================
    if (cfg.tlmPeriodMs > 0 &&
        (int32_t)(now - ts.lastTlm) >= (int32_t)cfg.tlmPeriodMs) {
        robot.telemetryEmit(now, robot._tlmBoundFn, robot._tlmBoundCtx);
        ts.lastTlm = now;
    }
}
