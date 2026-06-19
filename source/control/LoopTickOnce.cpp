#include "LoopTickOnce.h"
#include "Robot.h"
#include "CommandProcessor.h"
#include "Inputs.h"
#include "HaltController.h"
#include "MotionController.h"
#include "MotionCommand.h"
#include <cstdio>
#include <cmath>   // fmaxf, fabsf — control-collect outlier filter (039-002)

// ---------------------------------------------------------------------------
// loopTickOnce — one iteration of the firmware cooperative loop body.
//
// Called by:
//   - LoopScheduler::run_blocks()  (firmware, after controlCollectSplitPhase)
//   - sim_tick()                   (sim, after hal.tick() + controlCollectSplitPhase)
//
// See LoopTickOnce.h for the full contract.
// ---------------------------------------------------------------------------
void loopTickOnce(Robot& robot, CommandProcessor& cmd, CommandQueue& queue,
                  LoopTickState& ts, uint32_t now)
{
    const RobotConfig& cfg = robot.config;

    // ===== CONTROL COLLECT: outlier filter → PID → wedge push ===============
    //
    // 043-002 (Phase E): the CONTROL COLLECT block (~100 lines — outlier filter,
    // motorController.controlTick(), and the wedge push into PhysicalStateEstimate)
    // moved VERBATIM into Drive::periodic(now, fn, ctx).  The five filter-streak
    // members (_filterRejectStreakL/R, _prevDriving, _prevAnyWedged,
    // _lastControlMs) moved from Robot onto Drive as value members.  Same position
    // (before cmd.dequeueOne(queue)), same order, same numerics — the golden-TLM
    // canary is the byte-exact oracle.
    //
    // The EVT enc_filter_hold emission inside the block uses Robot's TLM sink
    // (_tlmBoundFn/_tlmBoundCtx); those are passed as the fn/ctx parameters
    // (architecture-update.md OQ-2) so the emission is byte-identical.
    robot.drive.periodic(now, robot._tlmBoundFn, robot._tlmBoundCtx);

    // ===== QUEUE: dispatch one enqueued command per tick ===================
    // Commands arrive via cmd.process() → queue.push_back().
    // dequeueOne() dispatches the front command, keeping behaviour identical
    // to the former immediate-dispatch path (enqueue + dequeue in same tick).
    cmd.dequeueOne(queue);

    // ===== SAFETY: centralized per-tick safety evaluation (042-003) =========
    // Formerly two consecutive inline blocks here — (1) the keepalive/system
    // watchdog and (2) the halt-controller (SAFE/X/ESTOP) evaluation.  Both
    // bodies moved VERBATIM, in the SAME ORDER, into Superstructure::evaluateSafety.
    // This single call sits in the SAME position the blocks did: after
    // dequeueOne(queue), before driveAdvance.  driveAdvance stays below, in the
    // same order.  The golden-TLM canary is the byte-exact oracle.
    robot.superstructure.evaluateSafety(cmd, queue, ts, robot.state.inputs, now);

    // ===== DRIVE: advance drive state machine =================================
    robot.motionController.driveAdvance(
        robot.state.inputs, robot.state.commands, robot.state.target, now);

    // ===== ODOMETRY: dead-reckon pose from encoder deltas ====================
    robot.estimate.addOdometryObservation(robot.state.inputs, cfg.trackwidthMm,
                           cfg.rotationalSlip, now);

    // ===== HAL ACTUATOR TICK: deliver commanded velocity to the HAL ===========
    // Pass the commanded actuator state to the HAL so a bench-mode sensor plant
    // (BenchOtosSensor in NezhaHAL) can integrate it.  Must run before the OTOS
    // block so the plant advances its accumulators before otosCorrect() calls
    // readTransformed().  Production NezhaHAL / MockHAL implement this as a
    // near-no-op when bench mode is off; the robot core no longer reaches into
    // the concrete HAL to do this (034-002, replaces robot.benchOtosTick).
    robot.hal.tick(now, robot.state.commands);

    // ===== OTOS: timed I2C pose read + EKF fusion ============================
    // In firmware: run when enOtos is set and lagOtosMs has elapsed.
    // In sim: the lagOtosMs gate is bypassed; fusion runs every tick when
    // fuseOtos is true (matches the original sim_tick() behaviour which called
    // otosCorrect() unconditionally each tick).
    if (ts.fuseOtos) {
        robot.otosCorrect(now);
    } else if (cfg.lagOtosMs > 0 &&
               (int32_t)(now - ts.lastOtos) >= (int32_t)cfg.lagOtosMs) {
        robot.otosCorrect(now);
        ts.lastOtos = now;
    }

    // ===== LINE: timed I2C read ===============================================
    // 043-001 (Phase E): the lag gate + read + timer bump moved VERBATIM into
    // LineSensor::periodic(ts, now).  Same order/position, same numerics — the
    // golden-TLM canary is the byte-exact oracle.
    robot.lineSensor.periodic(ts, now);

    // ===== COLOUR: timed read =================================================
    // 043-001: verbatim COLOUR block now in ColorSensor::periodic(ts, now).
    // (Robot member is colorSensor_ — the IColorSensor& device ref keeps the
    // colorSensor name to avoid macro collisions; see Robot.cpp annotation.)
    robot.colorSensor_.periodic(ts, now);

    // ===== PORTS: timed GPIO read =============================================
    // 043-001: verbatim PORTS block now in Ports::periodic(ts, now).
    robot.ports.periodic(ts, now);

    // ===== TELEMETRY: timed TLM frame emit ====================================
    // N3 fix (030-003): emit with the STREAM-bound fn+ctx pair, not ts.activeCtx
    // (which is the last *command* channel, not the bound stream channel).
    // Mixed serial+radio field setup: STREAM over serial then a radio command
    // would have passed ts.activeCtx = &radio to serialReplyTlm, casting Radio*
    // to SerialPort* — UB.  Using the bound pair keeps TLM on the channel that
    // issued STREAM regardless of which channel subsequent commands arrive on.
    // telemetryEmit guards fn == nullptr, so SET tlmPeriod without STREAM is safe.
    if (cfg.tlmPeriodMs > 0 &&
        (int32_t)(now - ts.lastTlm) >= (int32_t)cfg.tlmPeriodMs) {
        robot.telemetryEmit(now, robot._tlmBoundFn, robot._tlmBoundCtx);
        ts.lastTlm = now;
    }
}
