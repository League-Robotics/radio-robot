#pragma once
#include <stdint.h>
#include "hal/capability/ILineSensor.h"
#include "Inputs.h"
#include "Config.h"
#include "LoopTickOnce.h"

// ---------------------------------------------------------------------------
// LineSensor — thin subsystem wrapper for the timed 4-channel line-sensor read.
//
// Phase E (043-001): owns the LINE timed-block concern that previously lived as
// an inline block in loopTickOnce calling Robot::lineRead().  Bodies moved
// VERBATIM — no numeric or ordering change.  The only non-verbatim change:
// systemTime() inside the former lineRead() body is replaced by the `now`
// parameter threaded through periodic()/updateInputs() (same value loopTickOnce
// already uses for ts.lastLine — see architecture-update.md OQ-3).
//
// No virtual dispatch, no SubsystemBase: a standalone value-type class held by
// Robot.  Holds references into the device interface, HardwareState, and config;
// all are live for the lifetime of the owning Robot.
//
// No printf / telemetryEmit inside any method (Phase F logging-contract pre-cut).
//
// Namespaced under `subsystems` because `LineSensor` is already a global class
// name — the io/real device driver (source/io/real/LineSensor.h, class
// LineSensor : public ILineSensor) reachable in the firmware build via NezhaHAL.
// The architecture-update.md names this subsystem LineSensor; the namespace
// keeps that name + file name while resolving the ODR collision.  Robot exposes
// it as the member `lineSensor`, so call sites read robot.lineSensor.periodic().
// ---------------------------------------------------------------------------
namespace subsystems {

class LineSensor {
public:
    LineSensor(ILineSensor& line, HardwareState& inputs, const RobotConfig& cfg)
        : _line(line), _inputs(inputs), _cfg(cfg) {}

    // updateInputs — verbatim former Robot::lineRead() body, systemTime() -> now.
    void updateInputs(uint32_t now);

    // periodic — verbatim LINE timed-block lag gate + updateInputs() + timer bump.
    void periodic(LoopTickState& ts, uint32_t now);

private:
    ILineSensor&       _line;
    HardwareState&     _inputs;
    const RobotConfig& _cfg;
};

}  // namespace subsystems
