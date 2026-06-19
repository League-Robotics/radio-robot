#pragma once
#include <stdint.h>
#include "io/capability/IColorSensor.h"
#include "RobotState.h"
#include "Config.h"
#include "LoopTickOnce.h"

// ---------------------------------------------------------------------------
// ColorSensor — thin subsystem wrapper for the timed RGBC poll.
//
// Phase E (043-001): owns the COLOUR timed-block concern that previously lived
// as an inline block in loopTickOnce calling Robot::colorRead().  Bodies moved
// VERBATIM — no numeric or ordering change.  systemTime() inside the former
// colorRead() body is replaced by the `now` parameter (same value loopTickOnce
// already uses for ts.lastColor — architecture-update.md OQ-3).
//
// No virtual dispatch, no SubsystemBase.  No printf / telemetryEmit inside any
// method body (Phase F logging-contract pre-cut).
//
// Namespaced under `subsystems`: `ColorSensor` is already a global class name —
// the io/real device driver (source/io/real/ColorSensor.h, class ColorSensor :
// public IColorSensor) reachable in the firmware build via NezhaHAL.  The
// namespace keeps the architecture-update.md name + file name while resolving
// the ODR collision.
// ---------------------------------------------------------------------------
namespace subsystems {

class ColorSensor {
public:
    ColorSensor(IColorSensor& colorSensor, HardwareState& inputs, const RobotConfig& cfg)
        : _colorSensor(colorSensor), _inputs(inputs), _cfg(cfg) {}

    // updateInputs — verbatim former Robot::colorRead() body, systemTime() -> now.
    void updateInputs(uint32_t now);

    // periodic — verbatim COLOUR timed-block lag gate + updateInputs() + timer bump.
    void periodic(LoopTickState& ts, uint32_t now);

private:
    IColorSensor&      _colorSensor;
    HardwareState&     _inputs;
    const RobotConfig& _cfg;
};

}  // namespace subsystems
