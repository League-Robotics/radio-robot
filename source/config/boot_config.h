// boot_config.h — Config: the robot's build-time boot configuration.
//
// The functions declared here are DEFINED by the AUTO-GENERATED
// source/config/boot_config.cpp, which scripts/gen_boot_config.py rewrites
// from the active robot JSON (data/robots/active_robot.json, or ROBOT_CONFIG)
// before every firmware build. That generated .cpp is the whole boot config:
// nothing but the per-port msg::MotorConfig defaults and the msg::Drivetrain-
// Config default, with per-robot calibration baked in at compile time.
//
// main.cpp calls these instead of hardcoding calibration in the dev loop. A
// robot's calibration lives in its JSON config (and, once running, is
// correctable live over the wire via `DEV M <n> CFG` / `DEV DT CFG`), never in
// main.cpp — mirroring the old tree's generated defaultRobotConfig()
// (source/robot/DefaultConfig.cpp) that this replaces for the message-based
// subsystems tree.
#pragma once

#include <stdint.h>

#include "messages/drivetrain.h"
#include "messages/motor.h"

namespace Config {

// Number of per-port MotorConfig entries defaultMotorConfigs() fills. Must
// equal Subsystems::NezhaHardware::kPortCount — main.cpp static_asserts the two
// agree so a port-count change forces this generator to be re-run.
constexpr uint32_t kMotorConfigCount = 4;

// Fill out[0 .. kMotorConfigCount-1] with the per-port boot MotorConfig
// defaults (out[i].port == i+1). Calibration is baked from the active robot
// JSON where a matching key exists; otherwise the bench-tuned firmware
// defaults are used (see boot_config.cpp / gen_boot_config.py).
void defaultMotorConfigs(msg::MotorConfig* out);

// The boot DrivetrainConfig default — trackwidth (baked from the robot JSON)
// and the drive-pair port binding.
msg::DrivetrainConfig defaultDrivetrainConfig();

}  // namespace Config
