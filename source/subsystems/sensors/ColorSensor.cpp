#include "ColorSensor.h"

namespace subsystems {

// ---------------------------------------------------------------------------
// updateInputs — non-blocking RGBC poll into HardwareState.
//
// VERBATIM body of the former Robot::colorRead() (source/robot/Robot.cpp,
// 043-001).  The only change: systemTime() -> the `now` parameter (same value
// loopTickOnce threads through the tick — architecture-update.md OQ-3).
// ---------------------------------------------------------------------------
void ColorSensor::updateInputs(uint32_t now)
{
    if (!_colorSensor.is_initialized()) return;
    if (_colorSensor.pollRGBC(_inputs.colorR,
                              _inputs.colorG,
                              _inputs.colorB,
                              _inputs.colorC)) {
        _inputs.colorVS.lastUpdMs = now;
        _inputs.colorVS.valid     = true;
    }
}

// ---------------------------------------------------------------------------
// periodic — VERBATIM COLOUR timed block from loopTickOnce (043-001):
// lag gate on cfg.lagColor / ts.lastColor, updateInputs(now), bump ts.lastColor.
// ---------------------------------------------------------------------------
void ColorSensor::periodic(LoopTickState& ts, uint32_t now)
{
    if (_cfg.lagColor > 0 &&
        (int32_t)(now - ts.lastColor) >= (int32_t)_cfg.lagColor) {
        updateInputs(now);
        ts.lastColor = now;
    }
}

}  // namespace subsystems
