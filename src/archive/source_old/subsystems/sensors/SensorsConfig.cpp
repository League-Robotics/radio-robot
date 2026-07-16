// =============================================================================
// SensorsConfig.cpp — RobotConfig → msg::*Config projection functions
//                     for the Sensors subsystem (ticket 057-003)
//
// C++11 / -fno-rtti / -fno-exceptions / no heap / no STL containers.
//
// Projection functions translate RobotConfig fields to the generated
// msg::LineSensorConfig and msg::ColorSensorConfig types consumed by
// Sensors::configure().
// =============================================================================

#include "SensorsConfig.h"

namespace subsystems {

// ---------------------------------------------------------------------------
// toLineSensorConfig — project RobotConfig → msg::LineSensorConfig.
//
// Mapped fields:
//   lagLine            → lag_line_ms  (sensor polling budget, ms)
//
// Fields with no current RobotConfig counterpart (threshold, norm_min/max,
// channel_map) default-initialize to zero per the msg:: generated types.
// ---------------------------------------------------------------------------
msg::LineSensorConfig toLineSensorConfig(const RobotConfig& rc)
{
    msg::LineSensorConfig cfg;
    cfg.lag_line = rc.lagLine;
    // threshold / norm_min / norm_max / channel_map: no RobotConfig mapping;
    // left at zero-initialized defaults.
    return cfg;
}

// ---------------------------------------------------------------------------
// toColorSensorConfig — project RobotConfig → msg::ColorSensorConfig.
//
// Mapped fields:
//   lagColor           → lag_color_ms  (color sensor polling budget, ms)
//
// Fields with no current RobotConfig counterpart (integration, gain,
// cal_r/g/b) default-initialize to zero/0.0f.
// ---------------------------------------------------------------------------
msg::ColorSensorConfig toColorSensorConfig(const RobotConfig& rc)
{
    msg::ColorSensorConfig cfg;
    cfg.lag_color = rc.lagColor;
    // integration / gain / cal_r / cal_g / cal_b: no RobotConfig mapping;
    // left at zero-initialized defaults.
    return cfg;
}

}  // namespace subsystems
