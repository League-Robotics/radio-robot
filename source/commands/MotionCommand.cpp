// MotionCommand.cpp — body-velocity active command lifecycle.
//
// See MotionCommand.h for full API documentation.
// Architecture reference: .clasi/sprints/017-.../architecture-update.md §MotionCommand
// Sprint 017, Ticket 003.

#include "MotionCommand.h"
#include "BodyVelocityController.h"
#include <cstdio>
#include <cstring>

// ---------------------------------------------------------------------------
// Configuration phase
// ---------------------------------------------------------------------------

void MotionCommand::configure(float v, float omega, BodyVelocityController* bvc)
{
    _bvc       = bvc;
    _vTgt      = v;
    _omegaTgt  = omega;

    // Clear all per-command state so a recycled instance has no residue.
    _nStops          = 0;
    for (uint8_t i = 0; i < kMaxStopConds; ++i) {
        _stops[i] = StopCondition{};
    }
    _replyFn         = nullptr;
    _replyCtx        = nullptr;
    _corrId[0]       = '\0';
    _stopStyle       = StopStyle::SOFT;
    _origin          = Origin::RETARGETABLE;   // reset; caller must call setOrigin() to override
    _active          = false;
    _stopping        = false;
    _softDeadline    = 0;
    _baseline        = MotionBaseline{};
    // Reset done EVT label to default; caller must call setDoneEvt() after
    // configure() to override it for the new command.
    strncpy(_doneEvtLabel, "EVT done", sizeof(_doneEvtLabel) - 1);
    _doneEvtLabel[sizeof(_doneEvtLabel) - 1] = '\0';
    // Reset fired-condition state for the new command.
    _firedKind    = StopCondition::Kind::NONE;
    _firedChannel = 0;
}

void MotionCommand::setDoneEvt(const char* label)
{
    if (!label) return;
    strncpy(_doneEvtLabel, label, sizeof(_doneEvtLabel) - 1);
    _doneEvtLabel[sizeof(_doneEvtLabel) - 1] = '\0';
}

bool MotionCommand::addStop(const StopCondition& c)
{
    if (_nStops >= kMaxStopConds) {
        // Overflow is a recoverable condition, not a fatal error (065-001 / CR-01):
        // the caller (Superstructure::requestGoal) checks this return value and
        // turns it into a safe cancel + wire-visible "ERR stopoverflow" instead of
        // continuing with silently-incomplete stop coverage. Never assert here —
        // the sim build carries no NDEBUG, so a live assert would abort the whole
        // process hosting the sim (pytest run / TestGUI tick-thread); real
        // firmware would panic mid-drive via the CODAL assert path.
        return false;
    }
    _stops[_nStops++] = c;
    return true;
}

void MotionCommand::setReplySink(ReplyFn fn, void* ctx, const char* corrId)
{
    _replyFn  = fn;
    _replyCtx = ctx;
    if (corrId && corrId[0] != '\0') {
        strncpy(_corrId, corrId, sizeof(_corrId) - 1);
        _corrId[sizeof(_corrId) - 1] = '\0';
    } else {
        _corrId[0] = '\0';
    }
}

void MotionCommand::setStopStyle(StopStyle s)
{
    _stopStyle = s;
}

// ---------------------------------------------------------------------------
// Predicate: hasTimeStop
// ---------------------------------------------------------------------------

bool MotionCommand::hasTimeStop() const
{
    for (uint8_t i = 0; i < _nStops; ++i) {
        if (_stops[i].kind == StopCondition::Kind::TIME) {
            return true;
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
// Execution phase
// ---------------------------------------------------------------------------

void MotionCommand::start(const HardwareState& inputs, uint32_t now)
{
    // Capture motion baseline.
    // Array convention: [0]=R (FR), [1]=L (FL) — see ActualState.h.
    _baseline.t0         = now;
    _baseline.enc0       = (inputs.encPos[1] + inputs.encPos[0]) * 0.5f;
    _baseline.encDiff0   = inputs.encPos[0] - inputs.encPos[1];
    _baseline.heading0   = inputs.fused.pose.h;
    _baseline.pose0X     = inputs.fused.pose.x;
    _baseline.pose0Y     = inputs.fused.pose.y;

    // 072-002: commanded-direction signs, captured from the command's
    // commanded v/omega at start() time. ±1.0, or 0.0 if exactly zero.
    // Consumed by StopCondition's DISTANCE/ROTATION/SAFETY_MARGIN Kinds to
    // gate on signed travel in the commanded direction (Decision 1).
    _baseline.vSign     = (_vTgt > 0.0f)     ? 1.0f : (_vTgt < 0.0f     ? -1.0f : 0.0f);
    _baseline.omegaSign = (_omegaTgt > 0.0f) ? 1.0f : (_omegaTgt < 0.0f ? -1.0f : 0.0f);

    _active   = true;
    _stopping = false;

    // Hand target to BVC; BVC will start ramping on the next tick.
    if (_bvc) {
        _bvc->setTarget(_vTgt, _omegaTgt);
    }
}

void MotionCommand::setTarget(float v, float omega)
{
    _vTgt     = v;
    _omegaTgt = omega;

    if (_bvc) {
        _bvc->setTarget(_vTgt, _omegaTgt);
    }
}


bool MotionCommand::tick(const HardwareState& inputs, uint32_t now, float dt_s)
{
    if (!_active) return false;

    // ------------------------------------------------------------------
    // SOFT ramp-down sub-phase.
    // ------------------------------------------------------------------
    if (_stopping) {
        // Still ramping to (0, 0) — advance BVC.
        if (_bvc) _bvc->advance(dt_s);

        // Check termination conditions: converged OR deadline passed.
        bool converged = _bvc ? _bvc->atTarget() : true;
        int32_t dt_deadline = (int32_t)(now - _softDeadline);
        bool deadline_hit   = (dt_deadline >= 0);

        if (converged || deadline_hit) {
            _active   = false;
            _stopping = false;
            emitEvt(_doneEvtLabel);
        }
        return _active;
    }

    // ------------------------------------------------------------------
    // Normal running sub-phase.
    // ------------------------------------------------------------------

    // Advance BVC one tick (profile → inverse → saturate → setTarget).
    if (_bvc) _bvc->advance(dt_s);

    // Evaluate stop conditions (OR-across-array: first hit terminates).
    bool stopped = false;
    for (uint8_t i = 0; i < _nStops; ++i) {
        if (_stops[i].evaluate(inputs, now, _baseline)) {
            // DIAGNOSTIC (transient turn-skip hunt): when a ROTATION arc stop
            // fires, emit how it ended. A healthy turn shows ms~1-2s and arc~tgt;
            // a tick-0 skip shows ms~0 with arc already >= tgt, and whichever of
            // base/cur is the outlier pinpoints the garbage encoder read that
            // corrupted the arc baseline.  One line per turn — low radio cost.
            if (_stops[i].kind == StopCondition::Kind::ROTATION && _replyFn) {
                float d = inputs.encPos[0] - inputs.encPos[1] - _baseline.encDiff0;
                if (d < 0.0f) d = -d;
                char dbg[80];
                snprintf(dbg, sizeof(dbg),
                         "EVT ROTSTOP ms=%u arc=%d tgt=%d base=%d cur=%d",
                         (unsigned)(now - _baseline.t0), (int)(d * 0.5f),
                         (int)_stops[i].a, (int)_baseline.encDiff0,
                         (int)(inputs.encPos[0] - inputs.encPos[1]));
                _replyFn(dbg, _replyCtx);
            }
            // Record which condition fired so emitEvt can append reason=<token>.
            _firedKind    = _stops[i].kind;
            _firedChannel = _stops[i].sensor;
            stopped = true;
            break;
        }
    }

    if (stopped) {
        // 072-002: SAFETY_MARGIN is safety-class — it forces an immediate HARD
        // teardown regardless of the command's configured _stopStyle, and
        // forces the emitted EVT label to "EVT safety_stop" (bypassing
        // _doneEvtLabel), reusing the exact label the keepalive watchdog
        // already emits (architecture-update.md Decision 2). This is the same
        // mechanism the existing _stopStyle == HARD check already uses, not a
        // new one — just one more condition on the same branch.
        bool safetyForced = (_firedKind == StopCondition::Kind::SAFETY_MARGIN);
        if (_stopStyle == StopStyle::HARD || safetyForced) {
            // Immediate teardown.
            if (_bvc) _bvc->reset();
            _active   = false;
            _stopping = false;
            emitEvt(safetyForced ? "EVT safety_stop" : _doneEvtLabel);
        } else {
            // SOFT: ramp to (0, 0) over up to kSoftDeadline.
            _stopping     = true;
            _softDeadline = now + kSoftDeadline;
            if (_bvc) _bvc->setTarget(0.0f, 0.0f);
        }
    }

    return _active;
}

void MotionCommand::cancel(StopStyle s)
{
    if (!_active) return;

    // HARD cancel (cancel is always an emergency abort regardless of style arg).
    if (_bvc) _bvc->reset();
    _active   = false;
    _stopping = false;
    emitEvt("EVT cancelled");

    (void)s;  // style argument reserved for future use
}

void MotionCommand::cancelQuiet()
{
    if (!_active) return;

    // N11: suppress "EVT cancelled" by clearing the reply sink before teardown.
    // Used for internal phase transitions (e.g. PURSUE backtrack re-gate) where
    // the enclosing command (G) is still in progress and the cancel is an
    // implementation detail, not a terminal event for the host-visible corrId.
    _replyFn  = nullptr;
    _replyCtx = nullptr;

    if (_bvc) _bvc->reset();
    _active   = false;
    _stopping = false;
    // emitEvt("EVT cancelled") is intentionally omitted — sink cleared above.
}

void MotionCommand::softStop(uint32_t now)
{
    // No-op if not active or already ramping down.
    if (!_active || _stopping) return;

    // Arm SOFT ramp-down: BVC target → (0,0); tick() will emit EVT done
    // once the BVC converges or the 3 s deadline passes.
    _stopping     = true;
    _softDeadline = now + kSoftDeadline;
    if (_bvc) _bvc->setTarget(0.0f, 0.0f);
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

// Map StopCondition::Kind + channel to a reason token string.
// Returns "" (empty) for NONE — no reason token appended for cancel paths.
// Channel names for SENSOR mirror kSensorChannels in MotionCommands.cpp.
// 0-3: line[0..3]; 4-7: colorR/G/B/C; 8-11: analogIn[0..3].
static const char* mc_reasonToken(StopCondition::Kind kind, uint8_t channel)
{
    using K = StopCondition::Kind;
    switch (kind) {
        case K::NONE:     return "";
        case K::TIME:     return "time";
        case K::DISTANCE: return "dist";
        case K::ROTATION: return "rot";
        case K::HEADING:  return "heading";
        case K::POSITION: return "pos";
        case K::LINE_ANY: return "line";
        case K::COLOR:    return "color";
        case K::SAFETY_MARGIN: return "runaway";
        case K::SENSOR: {
            // Channel-name table: matches kSensorChannels in MotionCommands.cpp.
            static const char* kNames[12] = {
                "line0", "line1", "line2", "line3",
                "colorR", "colorG", "colorB", "colorC",
                "analogIn0", "analogIn1", "analogIn2", "analogIn3",
            };
            if (channel < 12) return kNames[channel];
            return "sensor";
        }
        default: return "";
    }
}

void MotionCommand::emitEvt(const char* base)
{
    if (!_replyFn) return;

    char msg[80];
    if (_corrId[0] != '\0') {
        snprintf(msg, sizeof(msg), "%s #%s", base, _corrId);
    } else {
        // Manual copy to avoid another snprintf call with no format args.
        int i = 0;
        while (base[i] && i < (int)sizeof(msg) - 1) {
            msg[i] = base[i];
            ++i;
        }
        msg[i] = '\0';
    }

    // Append reason=<token> if a stop condition fired.
    const char* reason = mc_reasonToken(_firedKind, _firedChannel);
    if (reason[0] != '\0') {
        int len = 0;
        while (msg[len]) ++len;
        snprintf(msg + len, sizeof(msg) - len, " reason=%s", reason);
    }

    _replyFn(msg, _replyCtx);
}
