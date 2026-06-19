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

    // ===== SYSTEM WATCHDOG: fire safety_stop + X after sTimeoutMs of silence =
    // ts.watchdogMs == 0 means no command has been received yet this session;
    // the watchdog stays disarmed until the first command arrives.
    // Signed delta avoids uint32 underflow (project memory: watchdog-uint32-underflow).
    //
    // TIME-stop exemption (sprint 024-003): self-terminating commands that
    // carry a TIME stop condition (T, D, G, TURN, RT, G PRE_ROTATE) are
    // exempt from the keepalive requirement — their TIME net fires regardless
    // of host silence.  Open-ended streaming commands (S / VW / R) have no
    // TIME stop and remain keepalive-bound.
    {
        MotionController& mc = robot.motionController;
        bool needsWatchdog =
            (mc.mode() != DriveMode::IDLE) || mc.hasActiveCommand();

        // Exempt commands that carry their own TIME backstop.
        if (mc.hasActiveCommand() && mc.activeCmd().hasTimeStop()) {
            needsWatchdog = false;
        }

        if (cfg.safetyEnabled && ts.watchdogMs != 0 &&
            ts.activeFn != nullptr && needsWatchdog) {
            int32_t wdDelta = (int32_t)(now - ts.watchdogMs);
            if (wdDelta > (int32_t)cfg.sTimeoutMs) {
                ts.watchdogMs = now;  // re-arm to avoid firing every tick
                char wdBuf[64];
                CommandProcessor::replyEvt(wdBuf, sizeof(wdBuf),
                                           "safety_stop", "",
                                           ts.activeFn, ts.activeCtx);
                // Bypass the queue for internal emergency stop: detach queue
                // so process() dispatches X immediately, then restore.
                cmd.setQueue(nullptr);
                cmd.process("X", ts.activeFn, ts.activeCtx);
                cmd.setQueue(&queue);
            }
        }
    }

    // ===== HALT CONDITIONS: evaluate user-registered stop conditions ========
    // Runs after the watchdog check, before the motion tick.
    {
        if (ts.activeFn != nullptr) {
            HaltAction ha = robot.haltController.evaluate(
                robot.state.inputs, now, ts.activeFn, ts.activeCtx);
            // Bypass the queue for halt-triggered emergency stops.
            if (ha == HaltAction::HARD) {
                cmd.setQueue(nullptr);
                cmd.process("X", ts.activeFn, ts.activeCtx);
                cmd.setQueue(&queue);
            } else if (ha == HaltAction::SOFT) {
                cmd.setQueue(nullptr);
                cmd.process("X soft", ts.activeFn, ts.activeCtx);
                cmd.setQueue(&queue);
            }
        }
    }

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
    if (cfg.lagLineMs > 0 &&
        (int32_t)(now - ts.lastLine) >= (int32_t)cfg.lagLineMs) {
        robot.lineRead();
        ts.lastLine = now;
    }

    // ===== COLOUR: timed read =================================================
    if (cfg.lagColorMs > 0 &&
        (int32_t)(now - ts.lastColor) >= (int32_t)cfg.lagColorMs) {
        robot.colorRead();
        ts.lastColor = now;
    }

    // ===== PORTS: timed GPIO read =============================================
    if (cfg.lagPortsMs > 0 &&
        (int32_t)(now - ts.lastPorts) >= (int32_t)cfg.lagPortsMs) {
        robot.portsRead();
        ts.lastPorts = now;
    }

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
