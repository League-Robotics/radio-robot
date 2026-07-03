#include "Ports.h"

namespace subsystems {

// ---------------------------------------------------------------------------
// updateInputs — read digital and analogue GPIO ports into HardwareState.
//
// VERBATIM body of the former Robot::portsRead() (source/robot/Robot.cpp,
// 043-001).  The only change: systemTime() -> the `now` parameter (same value
// loopTickOnce threads through the tick — architecture-update.md OQ-3).
// ---------------------------------------------------------------------------
void Ports::updateInputs(uint32_t now)
{
    for (uint8_t i = 0; i < 4; ++i) {
        _inputs.digitalIn[i] = (_portio.readDigital(i) != 0);
        _inputs.analogIn[i]  = (int16_t)_portio.readAnalog(i);
    }
    _inputs.portsVS.lastUpdMs = now;
    _inputs.portsVS.valid     = true;
}

// ---------------------------------------------------------------------------
// periodic — VERBATIM PORTS timed block from loopTickOnce (043-001):
// lag gate on cfg.lagPorts / ts.lastPorts, updateInputs(now), bump ts.lastPorts.
// ---------------------------------------------------------------------------
void Ports::periodic(LoopTickState& ts, uint32_t now)
{
    if (_cfg.lagPorts > 0 &&
        (int32_t)(now - ts.lastPorts) >= (int32_t)_cfg.lagPorts) {
        updateInputs(now);
        ts.lastPorts = now;
    }
}

}  // namespace subsystems
