#pragma once
// =============================================================================
// Sensors.h — Sensors subsystem facade (ticket 057-003)
//
// C++11 / -fno-rtti / -fno-exceptions / no heap / no STL containers.
//
// Pure-observation subsystem: empty command axis.
// Aggregates LineSensor + ColorSensor behind a single message-contract facade.
//
// Construction wires in the two existing subsystem wrappers (LineSensor,
// ColorSensor) and the shared HardwareState they write into. tick() manages
// its own lag timers and calls updateInputs() on each subsystem directly
// (no LoopTickState dependency), then projects HardwareState fields into the
// owned SensorsState.
//
// Contract verbs:
//   tick(now)           — drive both sensor reads when lag gates fire;
//                         refresh the owned SensorsState from HardwareState.
//   state() const       — const ref to owned SensorsState (no copy, no heap).
//   configure(lc, cc)   — store configs; next tick() picks them up.
//
// No virtual dispatch. No heap allocation. C++11 clean.
// =============================================================================

#include "subsystems/sensors/SensorsState.h"
#include "subsystems/sensors/LineSensor.h"
#include "subsystems/sensors/ColorSensor.h"
#include "messages/sensors.h"
#include "state/ActualState.h"  // HardwareState (= ActualState)

namespace subsystems {

// ---------------------------------------------------------------------------
// Sensors — pure-observation facade over LineSensor + ColorSensor.
// ---------------------------------------------------------------------------
class Sensors {
public:
    // hw: the shared HardwareState that LineSensor/ColorSensor write into.
    // Reading back from hw after updateInputs() is how tick() gets sensor data.
    Sensors(LineSensor& line, ColorSensor& color, const HardwareState& hw)
        : _line(line), _color(color), _hw(hw) {}

    // tick(now) — drive both sensors when their lag gates fire, refresh state.
    void tick(uint32_t now);

    // state() — const ref to owned SensorsState (tick() keeps it fresh).
    const SensorsState& state() const { return _state; }

    // configure(lc, cc) — store both configs; next tick() reads updated lag.
    void configure(const msg::LineSensorConfig& lc, const msg::ColorSensorConfig& cc);

private:
    LineSensor&          _line;
    ColorSensor&         _color;
    const HardwareState& _hw;
    SensorsState         _state   = {};

    msg::LineSensorConfig  _lineCfg  = {};
    msg::ColorSensorConfig _colorCfg = {};

    // Per-sensor lag timers (mirrors LoopTickState.lastLine / lastColor).
    uint32_t _lastLineTick  = 0;
    uint32_t _lastColorTick = 0;
};

}  // namespace subsystems
