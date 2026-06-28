// Superstructure.cpp — thin Goal dispatch (Phase D, Seam 3).
//
// requestGoal() is a plain switch that routes each Goal to the EXISTING
// begin* call the verb handler previously made directly.  Behaviour is
// byte-identical to the pre-seam state (042-001); the golden-TLM canary is the
// oracle.  No state-graph, no transition-table — just dispatch.

#include "Superstructure.h"
#include "MotionController.h"
#include "Robot.h"
#include "CommandProcessor.h"   // CommandProcessor::replyEvt, setQueue, process
#include "CommandQueue.h"       // CommandQueue (X injection bypass)
#include "HaltController.h"     // HaltController::evaluate, HaltAction
#include "Inputs.h"         // HardwareState
#include "LoopTickOnce.h"       // LoopTickState
#include "Config.h"             // RobotConfig (DriveMode via Config.h)

// ---------------------------------------------------------------------------
// goalAllowed — stub gate.  Returns true unconditionally this sprint.
//
// This mirrors the current gating exactly: there is no off-table denial today,
// so every goal that reached a begin* call before still reaches it now.  The
// hook exists so future pre-conditions have one canonical home.
// ---------------------------------------------------------------------------
bool Superstructure::goalAllowed(const GoalRequest& /*gr*/) const
{
    return true;
}

// ---------------------------------------------------------------------------
// requestGoal — single guarded transition function.
//
// goalAllowed() first (a denied goal returns without calling begin*), then a
// plain switch dispatches to the same begin* call, with the same arguments,
// that the verb handler made directly.  Each case reads only the GoalRequest
// fields relevant to its goal; the TargetState is gr.robot->state.desired, the
// same reference the direct calls passed.
// ---------------------------------------------------------------------------
void Superstructure::requestGoal(const GoalRequest& gr)
{
    if (!goalAllowed(gr)) {
        return;  // future use: denied goal makes no begin* call
    }

    TargetState& target = gr.robot->state.desired;

    switch (gr.goal) {
    case Goal::STREAM:
        // handleVW "stream" branch: beginStream(vL, vR, now, target, fn, ctx).
        _mc.beginStream(gr.leftMms, gr.rightMms, gr.now_ms,
                        target, gr.replyFn, gr.replyCtx);
        break;

    case Goal::TIMED:
        // handleVW "t" branch: beginTimed(vL, vR, ms, now, target, fn, ctx, corrId).
        _mc.beginTimed(gr.leftMms, gr.rightMms, gr.durationMs, gr.now_ms,
                       target, gr.replyFn, gr.replyCtx, gr.corrId);
        break;

    case Goal::DISTANCE:
        // handleVW "dist" branch: robot->distanceDrive(vL, vR, mm, fn, ctx, corrId).
        // Routed through Robot to preserve the atomic encoder reset
        // (beginDistance + resetEncoders).
        gr.robot->distanceDrive((int32_t)gr.leftMms, (int32_t)gr.rightMms,
                                gr.targetMm, gr.replyFn, gr.replyCtx, gr.corrId);
        break;

    case Goal::GOTO:
        // handleVW "x"+"y" branch: beginGoTo(x, y, speed, now, target, fn, ctx, corrId).
        _mc.beginGoTo(gr.tx, gr.ty, gr.speedMms, gr.now_ms,
                      target, gr.replyFn, gr.replyCtx, gr.corrId);
        break;

    case Goal::TURN:
        // handleVW "h" branch: beginTurn(h_cdeg, eps, now, target, fn, ctx, corrId).
        _mc.beginTurn(gr.headingCdeg, gr.epsCdeg, gr.now_ms,
                      target, gr.replyFn, gr.replyCtx, gr.corrId);
        break;

    case Goal::ROTATE:
        // handleVW "rot" branch: beginRotation(rot_cdeg, now, target, fn, ctx, corrId).
        _mc.beginRotation(gr.relCdeg, gr.now_ms,
                          target, gr.replyFn, gr.replyCtx, gr.corrId);
        break;

    case Goal::VELOCITY:
        // handleVW open-ended NEW-command branch:
        //   beginVelocity(v, omega, now, target, fn, ctx, corrId).
        _mc.beginVelocity(gr.v_mms, gr.omega_rads, gr.now_ms,
                          target, gr.replyFn, gr.replyCtx, gr.corrId);
        break;

    case Goal::ARC:
        // handleVW "radius" branch: beginArc(speed, radius, now, target, fn, ctx, corrId).
        _mc.beginArc(gr.speedMms, gr.radiusMm, gr.now_ms,
                     target, gr.replyFn, gr.replyCtx, gr.corrId);
        break;

    case Goal::IDLE:
    case Goal::ESTOP:
        // Reserved — no caller routes these through requestGoal this sprint.
        break;
    }
}

// ---------------------------------------------------------------------------
// evaluateSafety — per-tick safety evaluation (042-003).
//
// The two block bodies below are MOVED VERBATIM from loopTickOnce, in the SAME
// ORDER (watchdog first, then halt-controller).  Only their HOME changed: the
// same statements run in the same order with the same effects.  The three
// formerly-external references are re-sourced to the Superstructure-held
// members / the inputs parameter so the block bodies stay textually identical:
//   robot.motionController → _mc   (bound to the local `mc` ref, as before)
//   robot.config           → _cfg  (bound to the local `cfg` ref, as before)
//   robot.haltController    → _hc
//   robot.state.actual      → inputs (parameter)
// NO reordering, NO logic change.  driveAdvance is NOT called here — it stays
// in loopTickOnce immediately after this call.
// ---------------------------------------------------------------------------
void Superstructure::evaluateSafety(CommandProcessor& cmd, CommandQueue& queue,
                                    LoopTickState& ts, const HardwareState& inputs,
                                    uint32_t now)
{
    const RobotConfig& cfg = _cfg;

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
        MotionController& mc = _mc;
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
                                           "safety_stop", "reason=watchdog",
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
            HaltAction ha = _hc.evaluate(
                inputs, now, ts.activeFn, ts.activeCtx);
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
}
