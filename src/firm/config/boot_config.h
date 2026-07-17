// boot_config.h — Config: the robot's build-time boot configuration.
//
// DEFINED by the AUTO-GENERATED config/boot_config.cpp — see DESIGN.md.
// Never hand-edit boot_config.cpp; never hardcode calibration in main.cpp.
#pragma once

#include <stdint.h>

#include "messages/drivetrain.h"
#include "messages/motor.h"
#include "messages/planner.h"

namespace Config {

// Number of per-motor MotorConfig entries defaultMotorConfigs() fills. Must
// equal Subsystems::NezhaHardware::kMotorCount — main.cpp static_asserts the
// two agree. See DESIGN.md §3.
constexpr uint32_t kMotorConfigCount = 4;

// Fill out[0 .. kMotorConfigCount-1] with the per-motor boot MotorConfig
// defaults, indexed 0-based (out[i].port == i+1 -- .port is a
// wire/serialized key, the 1-based brick label, unchanged). Calibration is
// baked from the active robot JSON where a matching key exists; otherwise
// the bench-tuned firmware defaults are used (see boot_config.cpp /
// gen_boot_config.py, and DESIGN.md).
void defaultMotorConfigs(msg::MotorConfig* out);

// The boot DrivetrainConfig default — trackwidth (baked from the robot JSON)
// and the drive-pair port binding.
msg::DrivetrainConfig defaultDrivetrainConfig();

// The OTOS lever-arm mounting offset plus linear/angular scale multipliers,
// baked from the active robot JSON's geometry.odometry_offset_mm
// (x/y/yaw_rad) and calibration.otos_linear_scale/otos_angular_scale.
// Additive to defaultMotorConfigs()/defaultDrivetrainConfig() above — no
// existing mapping is touched.
//
// Boot-time-baked only, deliberately NOT a live SET/wire surface — see
// DESIGN.md §3/§4 for why. Consumed directly by main.cpp's
// Hal::OtosOdometer construction; the scale multipliers are converted to
// the OTOS chip's raw register scalar once at Hal::OtosOdometer::begin(),
// not re-derived per wire call (docs/protocol-v2.md §11).
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

// The boot PlannerConfig default — motion-limit fields (a_max/a_decel/
// v_body_max/yaw_rate_max/yaw_acc_max/j_max/yaw_jerk_max), the outer
// heading-loop PD gains (heading_kp/heading_kd), and the Drive::Limits
// tracker/policy fields, each baked from the robot JSON's control.* keys
// where present, else a firmware default (see gen_boot_config.py).
// arrive_tol/turn_in_place_gate stay unset (0.0f default) — unused by any
// current consumer.
msg::PlannerConfig defaultPlannerConfig();

}  // namespace Config
