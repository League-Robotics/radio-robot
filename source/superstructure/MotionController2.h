#pragma once
// =============================================================================
// MotionController2 — Phase 3 Planner subsystem wrapper
//
// Wraps the existing MotionController behind the 4-verb message-contract API.
// This is ADDITIVE: the existing MotionController logic is unchanged. The new
// class delegates to it by reference and is not wired into the live
// loopTickOnce path until ticket 059-005 (the cutover).
//
// Role: GOAL CLOSURE only. MotionController2 generates a time-varying body-
// twist setpoint from a goal + pose estimate and decides when the goal is
// reached. Velocity loops live in Drive2 / MotorController.
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
class Drive2;
}

// ---------------------------------------------------------------------------
// MotionController2 — message-driven Planner subsystem.
//
// Construction: takes the existing MotionController and Drive2 by reference,
// plus the RobotConfig for geometry / limits.
//
// Side effect of construction: calls _mc.setBvcStateRef(&_desired) to wire the
// internal BVC to publish body twist into _desired. This is intentional for
// isolated test use. In the live wiring (ticket 059-005), Robot will update its
// own setBvcStateRef call after constructing MotionController2.
//
// apply() STAGES only — no hardware I/O.
// tick(now) does all work and RETURNS a CommandBatch containing a
// DrivetrainCommand{TWIST} setpoint. Caller (the bus dispatcher) forwards
// this to Drive2.apply().
// ---------------------------------------------------------------------------
class MotionController2 {
public:
    // Constructor — wraps existing components by reference.
    // cfg: motion-limits source (aMax, vBodyMax, etc.); stored as local copy
    // so configure() can update it without disturbing the original.
    MotionController2(MotionController& mc,
                      const subsystems::Drive2& drive2,
                      const RobotConfig& cfg);

    // ---- 4-verb contract (no virtual dispatch) ----

    // Stage the goal command. No hardware I/O, no emission.
    // Dispatches on PlannerCommand::GoalKind → the appropriate begin*() call.
    void apply(const msg::PlannerCommand& cmd);

    // Advance goal closure one tick.
    // 1. Populate _hw from _drive2.state() (fused pose + twist).
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
    // future toPlannerConfig projections. This matches the note in the ticket:
    // "the implementer should document the chosen approach in a comment."
    void configure(const msg::PlannerConfig& cfg);

    // syncWireContext — copy the reply fn/ctx/corrId from robot.state.desired
    // into _desired before tick() calls driveAdvance().
    //
    // 060-004: G/TURN/D commands are dispatched through the wire path
    // (handleGoTo, handleTurn, handleDistance → mc.beginGoTo / beginTurn /
    // beginDistance) which writes the real replyFn into robot.state.desired.
    // But driveAdvance() uses MC2's _desired (= _target), which has _noopReply.
    // When PRE_ROTATE completes, driveAdvance line 275 calls:
    //   _activeCmd.setReplySink(target.replyFn, target.replyCtx, target.corrId)
    // overwriting the activeCmd's real replyFn with _noopReply.
    // This sync propagates the real reply context from the wire path into MC2's
    // _desired so driveAdvance uses the correct reply sink for EVT emission.
    void syncWireContext(const DesiredState& wire) {
        _desired.replyFn  = wire.replyFn;
        _desired.replyCtx = wire.replyCtx;
        // corrId is a char array — copy element-by-element (no strncpy in header).
        for (int i = 0; i < 16; ++i) _desired.corrId[i] = wire.corrId[i];
        _desired.sink = wire.sink;
    }

private:
    MotionController&          _mc;       // existing goal-closure engine (by ref)
    const subsystems::Drive2&  _drive2;   // source of fused pose/twist
    RobotConfig                _cfg;      // local shadow copy; configure() updates it

    // Internal state owned by MC2 (passed to driveAdvance).
    HardwareState              _hw      = {};  // populated from _drive2.state() each tick
    MotorCommands              _cmds    = {};  // sink for driveAdvance motor outputs (discarded)
    DesiredState               _desired = {};  // BVC publish target (wired via setBvcStateRef)
    TargetState&               _target;   // alias to _desired (TargetState = DesiredState)

    // Published planner state.
    msg::PlannerState          _state   = {};

    // Stored planner config snapshot (set by configure()).
    msg::PlannerConfig         _planCfg = {};

    // No-op reply sink used for all begin*() calls (EVT completion events
    // are routed via the command bus in ticket 059-003; not needed here).
    static void _noopReply(const char* /*msg*/, void* /*ctx*/) {}
};
