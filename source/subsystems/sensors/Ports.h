#pragma once
#include <stdint.h>
#include "io/capability/IPortIO.h"
#include "RobotState.h"
#include "Config.h"
#include "LoopTickOnce.h"

// ---------------------------------------------------------------------------
// Ports — thin subsystem wrapper for the timed digital/analogue GPIO read.
//
// Phase E (043-001): owns the PORTS timed-block concern that previously lived as
// an inline block in loopTickOnce calling Robot::portsRead().  Bodies moved
// VERBATIM — no numeric or ordering change.  The former portsRead() already
// called systemTime() for portsVS.lastUpdMs; that call is replaced by the `now`
// parameter (same value loopTickOnce uses for ts.lastPorts — architecture-
// update.md OQ-3).
//
// No virtual dispatch, no SubsystemBase.  No printf / telemetryEmit inside any
// method body (Phase F logging-contract pre-cut).
//
// Namespaced under `subsystems` for consistency with LineSensor/ColorSensor
// (which must be namespaced to avoid the io/real driver class-name collision).
// ---------------------------------------------------------------------------
namespace subsystems {

class Ports {
public:
    Ports(IPortIO& portio, HardwareState& inputs, const RobotConfig& cfg)
        : _portio(portio), _inputs(inputs), _cfg(cfg) {}

    // updateInputs — verbatim former Robot::portsRead() body, systemTime() -> now.
    void updateInputs(uint32_t now);

    // periodic — verbatim PORTS timed-block lag gate + updateInputs() + timer bump.
    void periodic(LoopTickState& ts, uint32_t now);

private:
    IPortIO&           _portio;
    HardwareState&     _inputs;
    const RobotConfig& _cfg;
};

}  // namespace subsystems
