// HaltController.cpp — user-facing named stop-condition registry.
//
// See HaltController.h for the API.
// Sprint 020, Ticket 007.

#include "HaltController.h"
#include "CommandProcessor.h"
#include <cstdio>
#include <cstring>

// ---------------------------------------------------------------------------
// add — register a new StopCondition.
//
// now_ms and enc_avg_mm are captured at registration time so that TIME and
// DIST conditions baseline from the moment HALT TIME/DIST is issued, not the
// boot epoch. This prevents an unexpected instant trip when HALT TIME is
// registered minutes after boot without a prior ZERO T command.
//
// Returns the assigned uint8_t ID, or -1 if the table is full.
// ---------------------------------------------------------------------------

int HaltController::add(const StopCondition& cond, StopStyle style, const char* str,
                         uint32_t now_ms, float enc_avg_mm)
{
    if (_count >= kMaxEntries) return -1;

    StopEntry& e = _entries[_count];
    e.cond   = cond;
    e.id     = _nextId++;
    e.style  = style;
    e.active = true;
    int slen = str ? (int)strlen(str) : 0;
    if (slen >= (int)sizeof(e.str)) slen = (int)sizeof(e.str) - 1;
    if (slen > 0) memcpy(e.str, str, (size_t)slen);
    e.str[slen] = '\0';

    // Initialize the per-entry baseline. For TIME conditions, baseline t0Ms to
    // now_ms (registration time). For DIST conditions, baseline enc0Mm to the
    // current encoder position. All other baseline fields start at zero; HEADING
    // and ROTATION conditions are absolute and do not need a t0/enc0 baseline.
    e.base = {};
    if (cond.kind == StopCondition::Kind::TIME) {
        e.base.t0Ms = now_ms;
    } else if (cond.kind == StopCondition::Kind::DISTANCE) {
        e.base.enc0Mm = enc_avg_mm;
    }

    ++_count;
    return (int)e.id;
}

// ---------------------------------------------------------------------------
// remove — free the slot for the entry with the given id.
//
// Compacts the array by shifting subsequent entries down so the freed slot
// is immediately reusable by the next add(). This prevents the table from
// being exhausted by repeated add/remove cycles within a session.
// ---------------------------------------------------------------------------

bool HaltController::remove(uint8_t id)
{
    for (int i = 0; i < _count; ++i) {
        if (_entries[i].id == id && _entries[i].active) {
            // Compact: shift entries above i down by one.
            for (int j = i; j < _count - 1; ++j) {
                _entries[j] = _entries[j + 1];
            }
            --_count;
            return true;
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
// setTimerBaseline — override t0Ms for all currently-active TIME entries.
//
// Called by ZERO T to re-baseline time conditions to a specific instant.
// Has no effect on entries added after this call (they get their own baseline
// from add()).
// ---------------------------------------------------------------------------

void HaltController::setTimerBaseline(uint32_t now_ms)
{
    for (int i = 0; i < _count; ++i) {
        if (_entries[i].active &&
            _entries[i].cond.kind == StopCondition::Kind::TIME) {
            _entries[i].base.t0Ms = now_ms;
        }
    }
}

// ---------------------------------------------------------------------------
// setDistBaseline — override enc0Mm for all currently-active DIST entries.
//
// Called by ZERO D to re-baseline distance conditions to a specific odometry
// point. Has no effect on entries added after this call.
// ---------------------------------------------------------------------------

void HaltController::setDistBaseline(float enc_avg_mm)
{
    for (int i = 0; i < _count; ++i) {
        if (_entries[i].active &&
            _entries[i].cond.kind == StopCondition::Kind::DISTANCE) {
            _entries[i].base.enc0Mm = enc_avg_mm;
        }
    }
}

// ---------------------------------------------------------------------------
// clear — deactivate all entries; return the count that were active.
// ---------------------------------------------------------------------------

int HaltController::clear()
{
    int n = 0;
    for (int i = 0; i < _count; ++i) {
        if (_entries[i].active) ++n;
    }
    clearAll();
    return n;
}

// ---------------------------------------------------------------------------
// info — write a human-readable description for a given id into buf.
// ---------------------------------------------------------------------------

bool HaltController::info(uint8_t id, char* buf, int bufLen) const
{
    for (int i = 0; i < _count; ++i) {
        if (_entries[i].id == id) {
            snprintf(buf, (size_t)bufLen, "id=%u active=%s str=\"%s\"",
                     (unsigned)_entries[i].id,
                     _entries[i].active ? "yes" : "no",
                     _entries[i].str);
            return true;
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
// list — emit one line per active entry via fn/ctx.
// ---------------------------------------------------------------------------

void HaltController::list(ReplyFn fn, void* ctx) const
{
    for (int i = 0; i < _count; ++i) {
        if (!_entries[i].active) continue;
        char line[80];
        snprintf(line, sizeof(line), "OK HALT id=%u str=\"%s\"",
                 (unsigned)_entries[i].id, _entries[i].str);
        fn(line, ctx);
    }
}

// ---------------------------------------------------------------------------
// count — return the number of currently active entries.
// ---------------------------------------------------------------------------

int HaltController::count() const
{
    int n = 0;
    for (int i = 0; i < _count; ++i) {
        if (_entries[i].active) ++n;
    }
    return n;
}

// ---------------------------------------------------------------------------
// clearAll — internal: reset the entry table and counter.
// ---------------------------------------------------------------------------

void HaltController::clearAll()
{
    for (int i = 0; i < _count; ++i) {
        _entries[i].active = false;
    }
    _count = 0;
}

// ---------------------------------------------------------------------------
// evaluate — check all active conditions each tick.
//
// Each entry carries its own MotionBaseline captured at add() time, so TIME
// and DIST conditions fire relative to registration rather than the boot epoch.
// Iterates entries in order; the first condition that fires wins.
//
// Wire protocol note: when any condition fires, clearAll() is called to wipe
// ALL registered conditions — not only the one that fired. This is intentional:
// a halt event is a session-level signal and the entire halt registry is cleared.
// The host must re-register any conditions it wants to remain active after a halt.
// ---------------------------------------------------------------------------

HaltAction HaltController::evaluate(const HardwareState& s, uint32_t now_ms,
                                     ReplyFn evtFn, void* evtCtx)
{
    if (_count == 0) return HaltAction::NONE;

    for (int i = 0; i < _count; ++i) {
        if (!_entries[i].active) continue;

        // Use the per-entry baseline captured at registration time.
        if (_entries[i].cond.evaluate(s, now_ms, _entries[i].base)) {
            // Capture fired entry details before clearAll() wipes them.
            uint8_t   firedId    = _entries[i].id;
            StopStyle firedStyle = _entries[i].style;

            // Emit EVT halt before clearing so the id is still valid.
            char evtBuf[64];
            char evtBody[32];
            snprintf(evtBody, sizeof(evtBody), "id=%u", (unsigned)firedId);
            CommandProcessor::replyEvt(evtBuf, sizeof(evtBuf),
                                       "halt", evtBody,
                                       evtFn, evtCtx);

            // Clear ALL registered conditions (see wire protocol note above).
            clearAll();

            return (firedStyle == StopStyle::SOFT)
                   ? HaltAction::SOFT : HaltAction::HARD;
        }
    }

    return HaltAction::NONE;
}
