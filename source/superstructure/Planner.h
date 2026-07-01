#pragma once
// =============================================================================
// Planner.h — Planner subsystem wrapper (renamed from MotionController2, 060-006)
//
// Wraps the existing MotionController behind the 4-verb message-contract API.
// This is ADDITIVE: the existing MotionController logic is unchanged. The new
// class delegates to it by reference.
//
// Role: GOAL CLOSURE only. Planner generates a time-varying body-twist setpoint
// from a goal + pose estimate and decides when the goal is reached.
// Velocity loops live in Drive (subsystems::Drive) / MotorController.
//
// Contract (no virtual dispatch, no heap allocation in tick()):
//   apply(PlannerCommand)     — STAGES the goal, no emission
//   tick(now) → CommandBatch — advances goal logic, returns DrivetrainCommand
//   state() → PlannerState   — const-ref to live planner state
//   configure(PlannerConfig) — delta-apply motion params
//
// See: clasi/issues/message-based-subsystem-architecture.md §Planner
// =============================================================================

#include "MotionController.h"
#include "types/Inputs.h"          // HardwareState, MotorCommands, TargetState
#include "types/Config.h"          // RobotConfig
#include "messages/planner.h"      // msg::PlannerCommand, PlannerState, PlannerConfig
#include "messages/common.h"       // msg::CommandBatch
#include "messages/drivetrain.h"   // msg::DrivetrainCommand

namespace subsystems {
class Drive;
}

// ---------------------------------------------------------------------------
// Planner — message-driven Planner subsystem wrapper.
//
// Construction: takes the existing MotionController and Drive by reference,
// plus the RobotConfig for geometry / limits.
//
// Side effect of construction: calls _mc.setBvcStateRef(&_desired) to wire the
// internal BVC to publish body twist into _desired. This is intentional for
// isolated test use. In the live wiring, Robot constructs Planner after Drive.
//
// apply() STAGES only — no hardware I/O.
// tick(now) does all work and RETURNS a CommandBatch containing a
// DrivetrainCommand{TWIST} setpoint. Caller (the bus dispatcher) forwards
// this to Drive.apply().
// ---------------------------------------------------------------------------
class Planner {
public:
    // Constructor — wraps existing components by reference.
    // cfg: motion-limits source (aMax, vBodyMax, etc.); stored as local copy
    // so configure() can update it without disturbing the original.
    Planner(MotionController& mc,
            const subsystems::Drive& drive,
            const RobotConfig& cfg);

    // ---- 4-verb contract (no virtual dispatch) ----

    // Stage the goal command. No hardware I/O, no emission.
    // Dispatches on PlannerCommand::GoalKind → the appropriate begin*() call.
    void apply(const msg::PlannerCommand& cmd);

    // Advance goal closure one tick.
    // 1. Populate _hw from _drive.state() (fused pose + twist).
    // 2. Call _mc.driveAdvance(_hw, _cmds, _target, now).
    // 3. Read commanded body twist from _desired.bodyTwist.
    // 4. Pack DrivetrainCommand{TWIST} into returned CommandBatch.
    // 5. Update _state.
    msg::CommandBatch tick(uint32_t now);

    // Read-only state snapshot — no I/O, no copy.
    const msg::PlannerState& state() const { return _state; }

    // Store updated planner config (motion limits only).
    // Approach: maintain a local RobotConfig copy _cfg that shadows the original.
    // The existing MotionController reads limits from its own const RobotConfig&
    // which is the original; _cfg is used here to populate PlannerState and for
    // future toPlannerConfig projections.
    void configure(const msg::PlannerConfig& cfg);

    // syncWireContext — copy the reply fn/ctx/corrId from robot.state.desired
    // into _desired before tick() calls driveAdvance().
    //
    // 060-004: G/TURN/D commands are dispatched through the wire path
    // (handleGoTo, handleTurn, handleDistance → mc.beginGoTo / beginTurn /
    // beginDistance) which writes the real replyFn into robot.state.desired.
    // But driveAdvance() uses Planner's _desired (= _target), which has _noopReply.
    // When PRE_ROTATE completes, driveAdvance line 275 calls:
    //   _activeCmd.setReplySink(target.replyFn, target.replyCtx, target.corrId)
    // overwriting the activeCmd's real replyFn with _noopReply.
    // This sync propagates the real reply context from the wire path into
    // Planner's _desired so driveAdvance uses the correct reply sink for EVT emission.
    void syncWireContext(const DesiredState& wire) {
        _desired.replyFn  = wire.replyFn;
        _desired.replyCtx = wire.replyCtx;
        // corrId is a char array — copy element-by-element (no strncpy in header).
        for (int i = 0; i < 16; ++i) _desired.corrId[i] = wire.corrId[i];
        _desired.sink = wire.sink;
    }

    // -------------------------------------------------------------------------
    // MotionController compatibility API — delegated to _mc (ticket 061-001).
    // Becomes native in 061-004.
    //
    // All signatures exactly match their MotionController counterparts so that
    // later tickets can change a call site's receiver from `motionController`
    // to `planner` without altering argument lists.
    // -------------------------------------------------------------------------

    DriveMode mode() const { return _mc.mode(); }

    void beginStream(float leftMms, float rightMms, uint32_t now_ms,
                     TargetState& target, ReplyFn fn, void* ctx) {
        _mc.beginStream(leftMms, rightMms, now_ms, target, fn, ctx);
    }

    void beginVelocity(float v_mms, float omega_rads, uint32_t now_ms,
                       TargetState& target, ReplyFn fn, void* ctx,
                       const char* corr_id = nullptr, bool seedImmediate = false) {
        _mc.beginVelocity(v_mms, omega_rads, now_ms, target, fn, ctx,
                          corr_id, seedImmediate);
    }

    void beginTimed(float leftMms, float rightMms, uint32_t durationMs, uint32_t now_ms,
                    TargetState& target, ReplyFn fn, void* ctx,
                    const char* corr_id = nullptr) {
        _mc.beginTimed(leftMms, rightMms, durationMs, now_ms, target, fn, ctx, corr_id);
    }

    void beginDistance(float leftMms, float rightMms, int32_t targetMm, uint32_t now_ms,
                       TargetState& target, ReplyFn fn, void* ctx,
                       const char* corr_id = nullptr) {
        _mc.beginDistance(leftMms, rightMms, targetMm, now_ms, target, fn, ctx, corr_id);
    }

    void beginGoTo(float tx, float ty, float speedMms, uint32_t now_ms,
                   TargetState& target, ReplyFn fn, void* ctx,
                   const char* corr_id = nullptr) {
        _mc.beginGoTo(tx, ty, speedMms, now_ms, target, fn, ctx, corr_id);
    }

    void beginTurn(float headingCdeg, float epsCdeg, uint32_t now_ms,
                   TargetState& target, ReplyFn fn, void* ctx,
                   const char* corr_id = nullptr) {
        _mc.beginTurn(headingCdeg, epsCdeg, now_ms, target, fn, ctx, corr_id);
    }

    void beginRotation(float relCdeg, uint32_t now_ms,
                       TargetState& target, ReplyFn fn, void* ctx,
                       const char* corr_id = nullptr) {
        _mc.beginRotation(relCdeg, now_ms, target, fn, ctx, corr_id);
    }

    void stop(uint32_t now_ms, ReplyFn fn, void* ctx) {
        _mc.stop(now_ms, fn, ctx);
    }

    void cancel(uint32_t now_ms, ReplyFn fn, void* ctx) {
        _mc.cancel(now_ms, fn, ctx);
    }

    void softStop(uint32_t now_ms) { _mc.softStop(now_ms); }

    void beginRawVelocity(float v_mms, float omega_rads) {
        _mc.beginRawVelocity(v_mms, omega_rads);
    }

    void disableSafetyOneShot() { _mc.disableSafetyOneShot(); }

    bool hasActiveCommand() const { return _mc.hasActiveCommand(); }

    void emitToActiveChannel(const char* evt, TargetState& target) {
        _mc.emitToActiveChannel(evt, target);
    }

    MotionCommand& activeCmd() { return _mc.activeCmd(); }

    void setHardwareState(HardwareState* s) { _mc.setHardwareState(s); }

    void setRobotCtx(Robot* r) { _mc.setRobotCtx(r); }

    void setBvcStateRef(DesiredState* ds) { _mc.setBvcStateRef(ds); }

    const HardwareState* hardwareState() const { return _mc.hardwareState(); }

private:
    MotionController&         _mc;       // existing goal-closure engine (by ref)
    const subsystems::Drive&  _drive;    // source of fused pose/twist
    RobotConfig               _cfg;      // local shadow copy; configure() updates it

    // Internal state owned by Planner (passed to driveAdvance).
    HardwareState             _hw      = {};  // populated from _drive.state() each tick
    MotorCommands             _cmds    = {};  // sink for driveAdvance motor outputs (discarded)
    DesiredState              _desired = {};  // BVC publish target (wired via setBvcStateRef)
    TargetState&              _target;   // alias to _desired (TargetState = DesiredState)

    // Published planner state.
    msg::PlannerState         _state   = {};

    // Stored planner config snapshot (set by configure()).
    msg::PlannerConfig        _planCfg = {};

    // No-op reply sink used for all begin*() calls (EVT completion events
    // are routed via the command bus; not needed here).
    static void _noopReply(const char* /*msg*/, void* /*ctx*/) {}
};
