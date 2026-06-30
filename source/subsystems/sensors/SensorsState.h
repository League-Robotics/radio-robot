#pragma once
// =============================================================================
// SensorsState.h — Aggregate state for the Sensors subsystem facade
//
// C++11 / -fno-rtti / -fno-exceptions / no heap / no STL containers.
//
// The Sensors subsystem (ticket 003) aggregates LineSensor + ColorSensor
// behind a single facade. This header defines the POD state type returned by
// Sensors::state() — a composition of the two generated message state types.
//
// Returned as a const reference from Sensors::state():
//
//   const subsystems::SensorsState& s = sensors.state();
//   bool line_ok   = s.line.get_connected();
//   uint32_t red   = s.color.get_r();
//
// =============================================================================

#include "messages/sensors.h"   // msg::LineSensorState, msg::ColorSensorState

namespace subsystems {

// ---------------------------------------------------------------------------
// SensorsState — POD aggregate of line-sensor and color-sensor state slices.
//
// Both fields are zero-initialized on construction (default member initializers
// on the msg:: types guarantee this). The Sensors subsystem's tick() updates
// each slice independently as readings arrive.
// ---------------------------------------------------------------------------
struct SensorsState {
    msg::LineSensorState  line  = {};
    msg::ColorSensorState color = {};
};

}  // namespace subsystems
