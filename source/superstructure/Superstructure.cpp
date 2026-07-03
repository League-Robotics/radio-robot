// Superstructure.cpp — thin Goal dispatch (Phase D, Seam 3).
//
// requestGoal() is a plain switch that routes each Goal to the EXISTING
// begin* call the verb handler previously made directly.  Behaviour is
// byte-identical to the pre-seam state (042-001); the golden-TLM canary is the
// oracle.  No state-graph, no transition-table — just dispatch.

#include "Superstructure.h"
#include "Planner.h"
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
// plain switch dispatches to the matching begin* call.  After sprint 053:
//   - Goal::VELOCITY covers VW, S (streamSeed=true), T (stops[0]=TIME), and
//     R (arc, omega=speed/radius pre-computed in handleR).
//   - Goal::STREAM, Goal::TIMED, Goal::ARC have been removed — all collapsed
//     to VELOCITY (open-loop twist path).
//   - Goal::DISTANCE is kept for the atomic encoder reset.
//   - Goal::GOTO / TURN / ROTATE are closed-loop and kept as-is.
// ---------------------------------------------------------------------------
void Superstructure::requestGoal(const GoalRequest& gr)
{
    if (!goalAllowed(gr)) {
        return;  // future use: denied goal makes no begin* call
    }

    TargetState& target = gr.robot->state.desired;

    switch (gr.goal) {
    case Goal::DISTANCE:
        // handleVW "dist" branch: robot->distanceDrive(vL, vR, mm, fn, ctx, corrId).
        // Routed through Robot to preserve the atomic encoder reset
        // (beginDistance + resetEncoders).
        gr.robot->distanceDrive((int32_t)gr.leftMms, (int32_t)gr.rightMms,
                                gr.targetMm, gr.replyFn, gr.replyCtx, gr.corrId);
        if (_planner.hasActiveCommand()) {
            if (gr.doneLabel) _planner.activeCmd().setDoneEvt(gr.doneLabel);
            // 065-001 / CR-01: addStop() can return false if the wire-supplied
            // stop=/sensor= clauses would overflow kMaxStopConds on top of the
            // DISTANCE+TIME pair beginDistance() already installed. Never let
            // that happen silently (a truncated clause list could drop the
            // operator's only safety-relevant stop) — cancel the just-started
            // command and reply a wire-visible ERR instead.
            for (uint8_t i = 0; i < gr.nStops; ++i) {
                if (!_planner.activeCmd().addStop(gr.stops[i])) {
                    _planner.activeCmd().cancel(MotionCommand::StopStyle::HARD);
                    char rbuf[80];
                    CommandProcessor::replyErr(rbuf, sizeof(rbuf), "stopoverflow", nullptr,
                                               gr.corrId, gr.replyFn, gr.replyCtx);
                    break;
                }
            }
        }
        break;

    case Goal::GOTO:
        // handleVW "x"+"y" branch: beginGoTo(x, y, speed, now, target, fn, ctx, corrId).
        _planner.beginGoTo(gr.tx, gr.ty, gr.speedMms, gr.now_ms,
                           target, gr.replyFn, gr.replyCtx, gr.corrId);
        break;

    case Goal::TURN:
        // handleVW "h" branch: beginTurn(h_cdeg, eps, now, target, fn, ctx, corrId).
        _planner.beginTurn(gr.headingCdeg, gr.epsCdeg, gr.now_ms,
                           target, gr.replyFn, gr.replyCtx, gr.corrId);
        break;

    case Goal::ROTATE:
        // handleVW "rot" branch: beginRotation(rot_cdeg, now, target, fn, ctx, corrId).
        _planner.beginRotation(gr.relCdeg, gr.now_ms,
                               target, gr.replyFn, gr.replyCtx, gr.corrId);
        break;

    case Goal::VELOCITY:
        // VW open-ended NEW-command branch, and S-command (streamSeed=true) branch.
        // beginVelocity(v, omega, now, target, fn, ctx, corrId, seedImmediate).
        // When gr.streamSeed is true (S command), the BVC is seeded at the target
        // speed immediately (no trapezoid ramp-up), preserving S's original semantics.
        _planner.beginVelocity(gr.v_mms, gr.omega_rads, gr.now_ms,
                               target, gr.replyFn, gr.replyCtx, gr.corrId, gr.streamSeed);
        if (_planner.hasActiveCommand()) {
            if (gr.doneLabel) _planner.activeCmd().setDoneEvt(gr.doneLabel);
            // 065-001 / CR-01: defense in depth (see Goal::DISTANCE case above).
            // beginVelocity() installs zero stops internally, so this is not
            // known to overflow in practice today, but never silently truncate.
            for (uint8_t i = 0; i < gr.nStops; ++i) {
                if (!_planner.activeCmd().addStop(gr.stops[i])) {
                    _planner.activeCmd().cancel(MotionCommand::StopStyle::HARD);
                    char rbuf[80];
                    CommandProcessor::replyErr(rbuf, sizeof(rbuf), "stopoverflow", nullptr,
                                               gr.corrId, gr.replyFn, gr.replyCtx);
                    break;
                }
            }
        }
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
//   robot.planner          → _planner (bound to the local `mc` ref, as before)
//   robot.config           → _cfg     (bound to the local `cfg` ref, as before)
//   robot.haltController   → _hc
//   robot.state.actual     → inputs (parameter)
// NO reordering, NO logic change.  driveAdvance is NOT called here — it stays
// in loopTickOnce immediately after this call.
// ---------------------------------------------------------------------------
void Superstructure::evaluateSafety(CommandProcessor& cmd, CommandQueue& queue,
                                    LoopTickState& ts, const HardwareState& inputs,
                                    uint32_t now)
{
    const RobotConfig& cfg = _cfg;

    // ===== SYSTEM WATCHDOG: fire safety_stop + X after sTimeout of silence =
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
        Planner& mc = _planner;
        bool needsWatchdog =
            (mc.mode() != DriveMode::IDLE) || mc.hasActiveCommand();

        // Exempt commands that carry their own TIME backstop.
        if (mc.hasActiveCommand() && mc.activeCmd().hasTimeStop()) {
            needsWatchdog = false;
        }

        if (cfg.safetyEnabled && ts.watchdogMs != 0 &&
            ts.activeFn != nullptr && needsWatchdog) {
            int32_t wdDelta = (int32_t)(now - ts.watchdogMs);
            // 065-003 / CR-05b: a second, `+`-independent staleness signal.
            // wdDelta alone resets on ANY `+`/motion line (ticket 002's
            // scoped keepalive), which is still satisfied by a background
            // keepalive thread even if the layer that is supposed to be
            // refreshing the VW/S/R target has itself stalled (e.g. a
            // frozen GUI event loop). vwDelta is stamped only by
            // beginVelocity()/beginRawVelocity() — the actual, authoritative
            // point of truth for "a velocity target was genuinely
            // refreshed" — so it catches that gap independent of `+`.
            // _lastVelocityRefreshMs is causally guaranteed non-zero here:
            // needsWatchdog can only be true once an open-ended command is
            // active, and no such command becomes active without first
            // calling beginVelocity()/beginRawVelocity().
            int32_t vwDelta = (int32_t)(now - _planner.lastVelocityRefreshMs());
            bool stale = (wdDelta > (int32_t)cfg.sTimeout) ||
                         (vwDelta > (int32_t)cfg.sTimeout);
            if (stale) {
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
