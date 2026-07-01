#pragma once
#include <stdint.h>
#include "Config.h"
#include "Protocol.h"
#include "StopCondition.h"

// ---------------------------------------------------------------------------
// Superstructure — Seam 3 of the FRC Elite Architecture adaptation (Phase D).
//
// A THIN, switch-over-Goal coordinator that provides ONE guarded transition
// point for goal starts.  `requestGoal(GoalRequest)` is the single external
// entry point: it calls `goalAllowed(gr)` (stub: returns true) and then
// dispatches via a plain `switch(gr.goal)` to the EXISTING
// Planner::beginX(...) call (or Robot::distanceDrive for DISTANCE).
//
// Behaviour-preservation contract (042-001): after this seam is inserted the
// effect is IDENTICAL to the pre-ticket state — requestGoal produces exactly
// the same beginX() call, with the same arguments, that the verb handler made
// directly.  The golden-TLM canary must stay byte-exact.
//
// This is a thin dispatcher, NOT a state machine.  There is intentionally NO
// state-graph and NO transition-table (issue §4, D2 L3-4).  The goalAllowed()
// hook is the seam for future pre-conditions; the switch body is just the
// existing beginX() call.  Extending to a richer interlock later is additive.
//
// Dependency rule: this header depends only on Config (RobotConfig) and
// Protocol (ReplyFn).  It forward-declares Planner, HaltController,
// and Robot so it pulls in NO vendor/device headers and NO io/ headers.  The
// vendor-confinement grep gate must see zero hits in source/superstructure/.
// ---------------------------------------------------------------------------

class Planner;
class HaltController;
struct Robot;
class CommandProcessor;
class CommandQueue;
struct LoopTickState;
// HardwareState is now a using-alias for ActualState (sprint 047-001);
// cannot be forward-declared as a struct. Include the full definition.
#include "types/Inputs.h"

// ---------------------------------------------------------------------------
// Goal — the set of drive goals the Superstructure can be asked to start.
//
// One value per begin* family.  IDLE/ESTOP are reserved for future use
// (no caller routes them through requestGoal this sprint).
//
// After sprint 053: STREAM, TIMED, ARC collapsed to VELOCITY (open-loop
// twist commands share a single code path via beginVelocity).  DISTANCE is
// kept to preserve the atomic encoder reset (Robot::distanceDrive).
// GOTO / TURN / ROTATE are closed-loop controllers and are kept as-is.
// ---------------------------------------------------------------------------
enum class Goal {
    IDLE,
    DISTANCE,  // Robot::distanceDrive (beginDistance + resetEncoders)
    GOTO,      // beginGoTo
    TURN,      // beginTurn
    ROTATE,    // beginRotation
    VELOCITY,  // beginVelocity (covers VW, S, T, R open-loop arcs)
    ESTOP
};

// ---------------------------------------------------------------------------
// GoalRequest — flat POD carrying the union of every begin* parameter set.
//
// Every begin* variant draws the arguments it needs from these fields; unused
// fields are zero-initialised by the caller's aggregate initialiser and are
// simply not read by the matching switch case.  `robot` supplies the
// TargetState (robot->state.target) and the DISTANCE path
// (robot->distanceDrive) so the routed calls stay byte-identical to the
// direct calls they replace.
// ---------------------------------------------------------------------------
struct GoalRequest {
    Goal      goal;
    Robot*    robot;        // supplies state.target and the DISTANCE path
    uint32_t  now_ms;
    ReplyFn   replyFn;
    void*     replyCtx;
    const char* corrId;     // originating command correlation id (may be null)

    // Wheel-speed goals (STREAM, TIMED, DISTANCE)
    float     leftMms;
    float     rightMms;
    uint32_t  durationMs;   // TIMED
    int32_t   targetMm;     // DISTANCE

    // GoTo (GOTO)
    float     tx;
    float     ty;
    float     speedMms;     // GOTO

    // Heading goal (TURN)
    float     headingCdeg;
    float     epsCdeg;

    // Relative rotation (ROTATE)
    float     relCdeg;

    // Body-twist (VELOCITY) — covers VW, S, T, and R (arc) open-loop commands.
    // R computes omega = speed/radius inline in handleR before building GoalRequest.
    float     v_mms;
    float     omega_rads;

    // Stop-condition plumbing (populated by verb handlers)
    StopCondition stops[4];   // stop conditions to apply after begin
    uint8_t       nStops;     // number of valid entries in stops[]
    bool          streamSeed; // true → seed BVC immediately (S-command semantics)
    const char*   doneLabel;  // EVT label for setDoneEvt; nullptr = use default
};

// ---------------------------------------------------------------------------
// Superstructure — single coordinator for goal transitions.
// ---------------------------------------------------------------------------
class Superstructure {
public:
    Superstructure(Planner& planner, HaltController& hc,
                   const RobotConfig& cfg)
        : _planner(planner), _hc(hc), _cfg(cfg) {}

    // requestGoal — the ONLY external entry point for goal starts.
    //
    // Calls goalAllowed(gr); if it returns false the goal is denied and no
    // begin* call is made (future use — the stub always allows).  Otherwise
    // dispatches via switch(gr.goal) to the matching begin* call.  Thin
    // dispatch only: NO _checkSafeOneShot here (begin* already calls it), NO
    // reply formatting (the verb handler owns replies and stop-condition
    // forwarding), NO state tracking.
    void requestGoal(const GoalRequest& gr);

    // goalAllowed — pre-condition gate (stub).  Returns true unconditionally
    // this sprint: it mirrors the current gating exactly (there is no off-table
    // behaviour today), and is the seam where future pre-conditions will live.
    bool goalAllowed(const GoalRequest& gr) const;

    // evaluateSafety — centralizes the per-tick safety evaluation that formerly
    // lived as two consecutive inline blocks in loopTickOnce (042-003):
    //   (1) keepalive/system watchdog  — needsWatchdog logic + X injection
    //   (2) halt-controller            — haltController.evaluate() + X / X soft
    // The two block bodies are MOVED VERBATIM, in the SAME ORDER (watchdog
    // first, then halt-controller); only their HOME moved into Superstructure.
    // NO reordering, NO logic change.  driveAdvance is NOT called here — it
    // stays in loopTickOnce immediately after this call.  Called once per tick
    // from loopTickOnce in the SAME position as the former inline blocks; the
    // golden-TLM canary stays byte-exact.
    void evaluateSafety(CommandProcessor& cmd, CommandQueue& queue,
                        LoopTickState& ts, const HardwareState& inputs,
                        uint32_t now);

    // Accessor for the wrapped Planner (used by loopTickOnce in a
    // later Phase-D ticket; harmless to expose now).
    Planner& planner() { return _planner; }

private:
    Planner&            _planner;
    HaltController&     _hc;
    const RobotConfig&  _cfg;
};
