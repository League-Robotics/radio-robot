// boot_config.h — Config: the robot's build-time boot configuration.
//
// DEFINED by the AUTO-GENERATED config/boot_config.cpp — see DESIGN.md.
// Never hand-edit boot_config.cpp; never hardcode calibration in main.cpp.
#pragma once

#include <stdint.h>

#include "messages/drivetrain.h"
#include "messages/motor.h"

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
// Boot-time-baked only, deliberately NOT a live SET/wire surface itself —
// see DESIGN.md §3/§4 for why. Consumed directly by main.cpp's
// Devices::Otos construction; the scale multipliers are converted to the
// OTOS chip's raw register scalar once at Devices::Otos::begin(), not
// re-derived per wire call (docs/protocol-v2.md §11). 109-004 added a
// SEPARATE, live runtime override on top of this boot bake —
// `OtosConfigPatch` (config.proto), applied by RobotLoop::handleConfig
// directly against Devices::Otos's setLinearScalar()/setAngularScalar()/
// setOffset()/init() — this struct itself is still never touched at
// runtime; only the chip's own registers are re-written.
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

// EstimatorBootConfig (117) — App::StateEstimator's fail-closed boot-time
// fusion-weight defaults, baked from the robot JSON's `estimator` section
// (data/robots/robot_config.schema.json). Field-for-field mirror of
// App::StateEstimator::FusionWeights (app/state_estimator.h), but declared
// independently here rather than reusing that type directly: config/ may
// only depend on messages/ (docs/design/design.md §5's dependency
// diagram), never on app/. main.cpp (ticket 004) converts this into an
// App::FusionWeights at the one place both types are visible, the same
// pattern toDeviceMotorConfig() already uses for msg::MotorConfig ->
// Devices::MotorConfig.
//
// headingOtos/omegaOtos are committed 0.0 in every robot JSON this sprint
// (stakeholder's encoder-only-v1 decision) — dimensionless [0..1] blend
// weights, no unit tag (coding-standards.md). staleness carries a reasoned
// per-robot placeholder (each robot JSON's own inline comment documents
// the derivation).
//
// stopLead (turn-prediction campaign): App::MoveQueue's own fail-closed
// boot-time anticipation lead -- see move_queue.h's tick() doc comment for
// what it does (StateEstimator::bodyAt(now + stopLead)-based Distance/
// Angle stop-condition evaluation). Lives in EstimatorBootConfig/
// EstimatorConfigPatch (not its own patch arm) as the "smallest coherent
// path": it is consumed by MoveQueue, not StateEstimator, but the
// prediction it anticipates INTO is StateEstimator's own, and every other
// piece of estimator-adjacent boot/live-tune plumbing (this struct,
// EstimatorConfigPatch, the CONFIG_ESTIMATOR wire target) already exists
// for exactly this shape of value -- a small, required, robot-JSON-baked
// float with a live-tunable wire override. Required (like every other
// field in this struct) -- gen_boot_config.py's own estimator_config_for_
// config() fails codegen if a robot JSON is missing estimator.stop_lead_ms.
struct EstimatorBootConfig {
  float headingOtos = 0.0f;  // [0..1] blend weight: body heading vs OTOS heading
  float omegaOtos = 0.0f;    // [0..1] blend weight: body omega vs OTOS omega
  uint32_t staleness = 200;  // [ms] max OTOS reading age still eligible to blend
  uint32_t stopLead = 0;     // [ms] App::MoveQueue stop-condition anticipation lead
};

// The boot EstimatorBootConfig default — fail-closed baked fusion weights,
// see EstimatorBootConfig's own doc comment above and
// gen_boot_config.py's estimator_config_for_config().
EstimatorBootConfig defaultEstimatorConfig();

}  // namespace Config
