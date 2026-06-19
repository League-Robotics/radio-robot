#include "LoopTickOnce.h"
#include "Robot.h"
#include "CommandProcessor.h"
#include "RobotState.h"
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
    // Migrated VERBATIM from Robot::controlCollectSplitPhase (039-002, OQ-2 b).
    // The per-loop split-phase encoder READ now happens earlier, in
    // Hardware::tick(now) → Motor::tick() (driven by the caller before this
    // function — LoopScheduler::run_blocks / sim_tick).  The cached value is read
    // here via positionMm(); the speed-scaled outlier filter, PID velocity
    // differentiation (inside controlTick), and the wedge push into Odometry stay
    // in the control layer so the golden-TLM frame is byte-for-byte unchanged.
    //
    // The retry re-reads still go through readEncoderMmFSettle() so the I2C bytes
    // on the wire (and the hardware retry behaviour) are identical to pre-039.
    {
        Robot& r = robot;
        uint32_t now_ms = now;

        // WedgeTest-proven pattern (sprint 015): read BOTH encoders every tick,
        // right motor (M1) first, then left (M2). Write-on-change is already
        // handled by Motor::setSpeed(). Single re-read on implausible delta.
        bool driving = (r.state.commands.tgtLMms != 0.0f ||
                        r.state.commands.tgtRMms != 0.0f);
        if (driving) {
            // Outlier threshold SCALES with commanded speed.  See the original
            // Robot::controlCollectSplitPhase comment block for the full rationale
            // (scaled vs fixed gate, slow-calibration garbage reads).
            const float kMaxDeltaMm = fmaxf(40.0f,
                fmaxf(fabsf((float)r.state.commands.tgtLMms),
                      fabsf((float)r.state.commands.tgtRMms)) * 0.2f);
            static constexpr int kRetries = 2;

            // Right (M1) first — proven ordering from WedgeTest.
            {
                float newR = r.motorR.positionMm();
                float dR   = newR - r.state.inputs.encRMm;
                if (dR > kMaxDeltaMm || dR < -kMaxDeltaMm) {
                    newR = r.state.inputs.encRMm;             // default: hold old
                    for (int k = 0; k < kRetries; ++k) {
                        float r2  = r.motorR.readEncoderMmFSettle(r.config);
                        float dr2 = r2 - r.state.inputs.encRMm;
                        if (dr2 <= kMaxDeltaMm && dr2 >= -kMaxDeltaMm) { newR = r2; break; }
                    }
                    if (r._filterRejectStreakR < 255) ++r._filterRejectStreakR;
                } else {
                    r._filterRejectStreakR = 0;
                }
                r.state.inputs.encRMm = newR;
            }

            // Left (M2) second.
            {
                float newL = r.motorL.positionMm();
                float dL   = newL - r.state.inputs.encLMm;
                if (dL > kMaxDeltaMm || dL < -kMaxDeltaMm) {
                    newL = r.state.inputs.encLMm;             // default: hold old
                    for (int k = 0; k < kRetries; ++k) {
                        float r2  = r.motorL.readEncoderMmFSettle(r.config);
                        float dr2 = r2 - r.state.inputs.encLMm;
                        if (dr2 <= kMaxDeltaMm && dr2 >= -kMaxDeltaMm) { newL = r2; break; }
                    }
                    if (r._filterRejectStreakL < 255) ++r._filterRejectStreakL;
                } else {
                    r._filterRejectStreakL = 0;
                }
                r.state.inputs.encLMm = newL;
            }

            // (033-005b) Emit EVT enc_filter_hold at threshold crossing (onset).
            if (r._filterRejectStreakR == Robot::kFilterRejectStreakThreshold &&
                    r._tlmBoundFn != nullptr) {
                char evtBuf[64];
                snprintf(evtBuf, sizeof(evtBuf),
                         "EVT enc_filter_hold wheel=R streak=%u",
                         (unsigned)r._filterRejectStreakR);
                r._tlmBoundFn(evtBuf, r._tlmBoundCtx);
            }
            if (r._filterRejectStreakL == Robot::kFilterRejectStreakThreshold &&
                    r._tlmBoundFn != nullptr) {
                char evtBuf[64];
                snprintf(evtBuf, sizeof(evtBuf),
                         "EVT enc_filter_hold wheel=L streak=%u",
                         (unsigned)r._filterRejectStreakL);
                r._tlmBoundFn(evtBuf, r._tlmBoundCtx);
            }
        } else {
            // Not driving: reset streak counters so they don't carry over.
            r._filterRejectStreakL = 0;
            r._filterRejectStreakR = 0;
        }
        r._prevDriving = driving;
        r._lastControlMs = now_ms;
        // refreshedWheel=3: both wheels updated; 0: idle, no velocity update.
        r.motorController.controlTick(r.state.inputs, r.state.commands, now_ms,
                                      driving ? 3 : 0);

        // (033-005e) Push wedge state into Odometry after every control tick.
        bool anyWedged = r.motorController.wheelWedgedL() ||
                         r.motorController.wheelWedgedR();
        r.estimate.setWedgeActive(anyWedged);
        if (anyWedged) {
            r.estimate.setEncOmegaHealthy(false);
        } else if (r._prevAnyWedged) {
            r.estimate.setEncOmegaHealthy(true);
        }
        r._prevAnyWedged = anyWedged;
    }

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
