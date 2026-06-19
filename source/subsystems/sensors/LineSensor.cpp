#include "LineSensor.h"

namespace subsystems {

// ---------------------------------------------------------------------------
// updateInputs — read 4-channel line sensor into HardwareState.
//
// VERBATIM body of the former Robot::lineRead() (source/robot/Robot.cpp,
// 043-001).  The only change: systemTime() -> the `now` parameter (same value
// loopTickOnce threads through the tick — architecture-update.md OQ-3).
// ---------------------------------------------------------------------------
void LineSensor::updateInputs(uint32_t now)
{
    if (!_line.is_initialized()) return;
    if (_line.readValues(_inputs.line)) {
        _inputs.lineVS.lastUpdMs = now;
        _inputs.lineVS.valid     = true;
    }
}

// ---------------------------------------------------------------------------
// periodic — VERBATIM LINE timed block from loopTickOnce (043-001):
// lag gate on cfg.lagLineMs / ts.lastLine, updateInputs(now), bump ts.lastLine.
// ---------------------------------------------------------------------------
void LineSensor::periodic(LoopTickState& ts, uint32_t now)
{
    if (_cfg.lagLineMs > 0 &&
        (int32_t)(now - ts.lastLine) >= (int32_t)_cfg.lagLineMs) {
        updateInputs(now);
        ts.lastLine = now;
    }
}

}  // namespace subsystems
