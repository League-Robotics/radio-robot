#pragma once
// =============================================================================
// SensorsConfig.h — RobotConfig → msg::*Config projection declarations
//                   for the Sensors subsystem (ticket 057-003)
//
// C++11 / -fno-rtti / -fno-exceptions / no heap / no STL containers.
// =============================================================================

#include "messages/sensors.h"
#include "types/Config.h"

namespace subsystems {

// Map RobotConfig::lagLineMs (and future threshold / norm fields) into the
// generated LineSensorConfig message type.
msg::LineSensorConfig  toLineSensorConfig(const RobotConfig& rc);

// Map RobotConfig::lagColorMs (and future integration/gain/cal fields) into
// the generated ColorSensorConfig message type.
msg::ColorSensorConfig toColorSensorConfig(const RobotConfig& rc);

}  // namespace subsystems
