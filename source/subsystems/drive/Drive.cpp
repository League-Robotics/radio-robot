#include "Drive.h"
#include <cstdio>
#include <cmath>   // fmaxf, fabsf — control-collect outlier filter (039-002)

namespace subsystems {

// ---------------------------------------------------------------------------
// updateInputs — encoder-filter writes into HardwareState.
//
// The per-wheel encoder writes (_inputs.encRMm / encLMm) live inside the outlier
// filter blocks in periodic(), which is the verbatim CONTROL COLLECT body.  This
// method is the conceptual seam documented for Phase F; the writes are performed
// by periodic() today, so updateInputs() is intentionally not invoked separately
// (calling it would not be byte-exact).  Kept as a no-op declaration to satisfy
// the subsystem updateInputs()/periodic() shape without altering behavior.
// ---------------------------------------------------------------------------
void Drive::updateInputs()
{
    // Encoder writes are inlined into periodic() (verbatim CONTROL COLLECT block).
    // No-op here — see header annotation and architecture-update.md.
}

// ---------------------------------------------------------------------------
// periodic — VERBATIM CONTROL COLLECT block from loopTickOnce (043-002).
//
// Migrated unchanged from loopTickOnce's CONTROL COLLECT block (which itself was
// migrated verbatim from Robot::controlCollectSplitPhase, 039-002 OQ-2 b).  The
// speed-scaled outlier filter, PID velocity differentiation (inside
// controlTick), and the wedge push into PhysicalStateEstimate stay in the control
// layer so the golden-TLM frame is byte-for-byte unchanged.
//
// `r.` member access is rewritten to Drive members:
//   r.motorL/motorR        -> _motorL/_motorR
//   r.state.inputs         -> _inputs
//   r.state.commands       -> _commands
//   r.config               -> _cfg
//   r.motorController       -> _mc
//   r.estimate             -> _est
//   r._filterRejectStreak* / r._prevDriving / r._lastControlMs / r._prevAnyWedged
//                          -> the Drive value members (moved from Robot)
//   Robot::kFilterRejectStreakThreshold -> Drive::kFilterRejectStreakThreshold
//   r._tlmBoundFn / r._tlmBoundCtx       -> fn / ctx parameters (OQ-2)
// ---------------------------------------------------------------------------
void Drive::periodic(uint32_t now, ReplyFn fn, void* ctx)
{
    uint32_t now_ms = now;

    // WedgeTest-proven pattern (sprint 015): read BOTH encoders every tick,
    // right motor (M1) first, then left (M2). Write-on-change is already
    // handled by Motor::setSpeed(). Single re-read on implausible delta.
    bool driving = (_commands.tgtLMms != 0.0f ||
                    _commands.tgtRMms != 0.0f);
    if (driving) {
        // Outlier threshold SCALES with commanded speed.  See the original
        // Robot::controlCollectSplitPhase comment block for the full rationale
        // (scaled vs fixed gate, slow-calibration garbage reads).
        const float kMaxDeltaMm = fmaxf(40.0f,
            fmaxf(fabsf((float)_commands.tgtLMms),
                  fabsf((float)_commands.tgtRMms)) * 0.2f);
        static constexpr int kRetries = 2;

        // Right (M1) first — proven ordering from WedgeTest.
        {
            float newR = _motorR.positionMm();
            float dR   = newR - _inputs.encRMm;
            if (dR > kMaxDeltaMm || dR < -kMaxDeltaMm) {
                newR = _inputs.encRMm;             // default: hold old
                for (int k = 0; k < kRetries; ++k) {
                    float r2  = _motorR.readEncoderMmFSettle(_cfg);
                    float dr2 = r2 - _inputs.encRMm;
                    if (dr2 <= kMaxDeltaMm && dr2 >= -kMaxDeltaMm) { newR = r2; break; }
                }
                if (_filterRejectStreakR < 255) ++_filterRejectStreakR;
            } else {
                _filterRejectStreakR = 0;
            }
            _inputs.encRMm = newR;
        }

        // Left (M2) second.
        {
            float newL = _motorL.positionMm();
            float dL   = newL - _inputs.encLMm;
            if (dL > kMaxDeltaMm || dL < -kMaxDeltaMm) {
                newL = _inputs.encLMm;             // default: hold old
                for (int k = 0; k < kRetries; ++k) {
                    float r2  = _motorL.readEncoderMmFSettle(_cfg);
                    float dr2 = r2 - _inputs.encLMm;
                    if (dr2 <= kMaxDeltaMm && dr2 >= -kMaxDeltaMm) { newL = r2; break; }
                }
                if (_filterRejectStreakL < 255) ++_filterRejectStreakL;
            } else {
                _filterRejectStreakL = 0;
            }
            _inputs.encLMm = newL;
        }

        // (033-005b) Emit EVT enc_filter_hold at threshold crossing (onset).
        if (_filterRejectStreakR == Drive::kFilterRejectStreakThreshold &&
                fn != nullptr) {
            char evtBuf[64];
            snprintf(evtBuf, sizeof(evtBuf),
                     "EVT enc_filter_hold wheel=R streak=%u",
                     (unsigned)_filterRejectStreakR);
            fn(evtBuf, ctx);
        }
        if (_filterRejectStreakL == Drive::kFilterRejectStreakThreshold &&
                fn != nullptr) {
            char evtBuf[64];
            snprintf(evtBuf, sizeof(evtBuf),
                     "EVT enc_filter_hold wheel=L streak=%u",
                     (unsigned)_filterRejectStreakL);
            fn(evtBuf, ctx);
        }
    } else {
        // Not driving: reset streak counters so they don't carry over.
        _filterRejectStreakL = 0;
        _filterRejectStreakR = 0;
    }
    _prevDriving = driving;
    _lastControlMs = now_ms;
    // refreshedWheel=3: both wheels updated; 0: idle, no velocity update.
    _mc.controlTick(_inputs, _commands, now_ms,
                    driving ? 3 : 0);

    // (033-005e) Push wedge state into Odometry after every control tick.
    bool anyWedged = _mc.wheelWedgedL() ||
                     _mc.wheelWedgedR();
    _est.setWedgeActive(anyWedged);
    if (anyWedged) {
        _est.setEncOmegaHealthy(false);
    } else if (_prevAnyWedged) {
        _est.setEncOmegaHealthy(true);
    }
    _prevAnyWedged = anyWedged;
}

}  // namespace subsystems
