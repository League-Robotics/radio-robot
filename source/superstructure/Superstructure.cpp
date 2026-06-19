// Superstructure.cpp — thin Goal dispatch (Phase D, Seam 3).
//
// requestGoal() is a plain switch that routes each Goal to the EXISTING
// begin* call the verb handler previously made directly.  Behaviour is
// byte-identical to the pre-seam state (042-001); the golden-TLM canary is the
// oracle.  No state-graph, no transition-table — just dispatch.

#include "Superstructure.h"
#include "MotionController.h"
#include "Robot.h"

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
// fields relevant to its goal; the TargetState is gr.robot->state.target, the
// same reference the direct calls passed.
// ---------------------------------------------------------------------------
void Superstructure::requestGoal(const GoalRequest& gr)
{
    if (!goalAllowed(gr)) {
        return;  // future use: denied goal makes no begin* call
    }

    TargetState& target = gr.robot->state.target;

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
