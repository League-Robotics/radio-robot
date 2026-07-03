#pragma once
#include <stdint.h>
#include "StopCondition.h"
#include "Protocol.h"

// ---------------------------------------------------------------------------
// HaltController — user-facing named stop-condition registry (ticket 020-007).
//
// Owns up to kMaxEntries user-registered StopEntry records. Each tick the
// caller invokes evaluate(); if any condition fires HaltController emits
// "EVT halt id=<n>", clears all conditions, and returns HaltAction::HARD or
// HaltAction::SOFT to tell LoopScheduler which X variant to dispatch.
//
// Baselines (timer and distance) are captured per-entry at add() time so that
// e.g. "HALT TIME 5000" fires ~5 s after registration without requiring a
// prior ZERO T / ZERO D. ZERO T / ZERO D still work as explicit overrides:
// they re-baseline all currently-registered entries of the matching type.
//
// Wire commands (HALT TIME/DIST/CLEAR) add/remove conditions by calling
// add() / clear() from their handlers.
//
// Design note: LoopScheduler owns HaltController as a public member so that
// command handlers (registered in Robot::buildCommandTable) can reach it via
// RobotSysCtx::sched->haltController without adding a separate context type.
// ---------------------------------------------------------------------------

// HaltAction — returned by evaluate() each tick.
enum class HaltAction : uint8_t {
    NONE,  // no condition fired; keep running
    HARD,  // fire X (immediate stop)
    SOFT   // fire X soft (ramp to zero)
};

// StopStyle — stored per entry; determines HaltAction when the entry fires.
enum class StopStyle : uint8_t { HARD, SOFT };

// StopEntry — one registered halt condition.
struct StopEntry {
    StopCondition  cond;
    MotionBaseline base;    // per-entry baseline captured at registration time
    uint8_t        id;
    StopStyle      style;
    bool           active;
    char           str[40]; // original command string for HALT INFO/LIST
};

class HaltController {
public:
    static constexpr int kMaxEntries = 8;

    HaltController() = default;

    // add — register a new condition.
    // now_ms and enc_avg_mm are the current system time and average encoder
    // position at registration time; they are used to baseline TIME and DIST
    // conditions so that e.g. "HALT TIME 5000" fires ~5 s after registration
    // even without a prior ZERO T / ZERO D command.
    // Returns the assigned ID (monotonically increasing uint8_t); -1 on full.
    int add(const StopCondition& cond, StopStyle style, const char* str,
            uint32_t now_ms, float enc_avg_mm);

    // remove — deactivate the entry with the given id.
    // Returns true if found and removed.
    bool remove(uint8_t id);

    // clear — remove all entries; return the count that were active.
    int clear();

    // info — write a description of entry id into buf (at most bufLen-1 chars).
    // Returns true if id was found.
    bool info(uint8_t id, char* buf, int bufLen) const;

    // list — call fn(msg, ctx) once per active entry with a summary line.
    void list(ReplyFn fn, void* ctx) const;

    // setTimerBaseline — override the t0 reference for all active TIME entries.
    // Called by ZERO T to re-baseline time conditions to a specific instant.
    void setTimerBaseline(uint32_t now_ms);

    // setDistBaseline — override the enc0 reference for all active DIST entries.
    // Called by ZERO D to re-baseline distance conditions to a specific odometry point.
    void setDistBaseline(float enc_avg_mm);

    // count — return the number of currently active entries.
    int count() const;

    // evaluate — check all active conditions each tick.
    // If any fires: emits "EVT halt id=<n>" via evtFn/evtCtx, clears all
    // conditions, and returns HARD or SOFT. Otherwise returns NONE.
    HaltAction evaluate(const HardwareState& s, uint32_t now_ms,
                        ReplyFn evtFn, void* evtCtx);

private:
    StopEntry _entries[kMaxEntries] = {};
    int       _count  = 0;
    uint8_t   _nextId = 0;

    // clearAll — internal: mark all entries inactive and reset _count.
    // Wire note: when any condition fires, clearAll() wipes ALL registered
    // conditions (not just the one that fired). This is by design — a single
    // halt event terminates the entire halt registry for the session.
    void clearAll();
};
