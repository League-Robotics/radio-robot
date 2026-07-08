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

// OtosBootConfig (086-005) — the OTOS lever-arm mounting offset plus
// linear/angular scale multipliers, baked from the active robot JSON's
// geometry.odometry_offset_mm (x/y/yaw_rad) and calibration.
// otos_linear_scale/otos_angular_scale. Additive to defaultMotorConfigs()/
// defaultDrivetrainConfig() above — no existing mapping is touched.
//
// Boot-time-baked only, deliberately NOT a live SET/wire surface
// (architecture-update.md Design Rationale 4; sprint 085-005 removed a dead
// `SET odomOffX/Y/Yaw` push because no such wire key exists). Ticket 086-006's
// Hal::OtosOdometer leaf is constructed directly with these values (main.cpp)
// — the offset feeds Hal::OtosOdometer's own private sensorToCentre()/
// centreToSensor() methods (092-004 — folded from the former standalone
// source/hal/lever_arm.h); the scale multipliers are converted to the OTOS chip's
// raw int8 register scalar once at Hal::OtosOdometer::begin() (the same
// scaleToInt8()-style conversion source_old/hal/real/OtosSensor.cpp::begin()
// applied), NOT re-derived at every OL/OA wire call (those operate on the
// raw register scalar directly — docs/protocol-v2.md §11).
struct OtosBootConfig {
  float offsetX = 0.0f;      // [mm] mounting offset from chassis centre to sensor
  float offsetY = 0.0f;      // [mm]
  float offsetYaw = 0.0f;    // [rad] mounting yaw offset
  float linearScale = 1.0f;   // OTOS linear scale multiplier (e.g. 1.067); 1.0 = no correction
  float angularScale = 1.0f;  // OTOS angular scale multiplier (e.g. 0.987); 1.0 = no correction
};

// The boot OtosBootConfig default — mounting offset + scale multipliers
// baked from the robot JSON where present; identity defaults (zero offset,
// 1.0 scale = no correction) otherwise.
OtosBootConfig defaultOtosBootConfig();

}  // namespace Config
