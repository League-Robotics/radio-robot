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
// The turn-prediction campaign's own boot-time anticipation-lead field
// (formerly declared here, App::MoveQueue's own former stop-condition
// time-lead) -- DELETED (118 ticket 004, land-at-zero-completion-delete-
// stop-lead.md): the anticipation mechanism it fed no longer exists (see
// move_queue.h's own tick() doc comment for the land-at-zero completion
// predicate that replaces it) -- there is no lead value left to bake.
struct EstimatorBootConfig {
  float headingOtos = 0.0f;  // [0..1] blend weight: body heading vs OTOS heading
  float omegaOtos = 0.0f;    // [0..1] blend weight: body omega vs OTOS omega
  uint32_t staleness = 200;  // [ms] max OTOS reading age still eligible to blend
};

// The boot EstimatorBootConfig default — fail-closed baked fusion weights,
// see EstimatorBootConfig's own doc comment above and
// gen_boot_config.py's estimator_config_for_config().
EstimatorBootConfig defaultEstimatorConfig();

// ShaperBootConfig (decel-into-the-goal campaign, follow-on to
// clasi/issues/angle-stop-overshoot-61-73-percent-on-hardware.md's own
// "Option 1... remains the path to closing that residual further") —
// Motion::VelocityShaper's own accel/decel magnitude ceilings, baked from
// the robot JSON's `control.a_max`/`control.a_decel`/`control.alpha_max`/
// `control.alpha_decel` (data/robots/robot_config.schema.json).
// Field-for-field mirror of App::ShaperLimits (app/move_queue.h), declared
// independently here for the SAME reason EstimatorBootConfig/FusionWeights
// stay independently declared: config/ may depend only on messages/
// (docs/design/design.md §5's dependency diagram), never on app/. main.cpp
// converts this into an App::ShaperLimits at the one composition-root
// place both types are visible, the same toFusionWeights()/
// toDeviceMotorConfig() pattern.
//
// aMax/aDecel/jMax/yawJerkMax are NOT new fields — they are the deleted
// msg::PlannerConfig's own `a_max`/`a_decel`/`j_max`/`yaw_jerk_max`,
// orphaned dead data since 115-003's motion-stack excision
// (gen_boot_config.py's own module docstring used to document all four as
// "unread by this generator"); this campaign reads them again, into a NEW
// consumer (Motion::VelocityShaper/App::MoveQueue) rather than the deleted
// planner. alphaMax/alphaDecel ARE new (a_max/a_decel's own angular
// sibling — no msg::PlannerConfig predecessor existed for either); yaw_
// jerk_max already existed as j_max's own angular sibling, so no NEW
// angular jerk field was needed the way alphaMax/alphaDecel were for
// accel/decel.
//
// jMax/yawJerkMax (jerk-limited S-curve stage, 2026-07-22 stakeholder
// correction on top of this struct's own first accel-limited pass):
// Motion::VelocityShaper's own jerk magnitude ceilings — how fast the
// commanded ACCELERATION itself may change, bounding the S-curve's own
// "corners" (see velocity_shaper.h's own file header for the full jerk-
// limited algorithm). `j_max`/`yaw_jerk_max` already existed as REQUIRED,
// unread `control.*` keys in every robot JSON since sprint 114 (098-001) —
// this campaign is the first consumer.
//
// REQUIRED (config-as-truth, sprint 114's own fail-closed posture,
// extended here): a robot JSON missing any of the six `control.a_max`/
// `a_decel`/`alpha_max`/`alpha_decel`/`j_max`/`yaw_jerk_max` keys fails
// codegen loudly (same MissingRobotConfigKeyError gen_boot_config.py's own
// `_require()` already raises for every other REQUIRED field) rather than
// silently shipping an unshaped (or zero-shaped, which would refuse to
// move at all — see App::ShaperLimits's own "0 == disabled" doc comment,
// move_queue.h) boot image.
struct ShaperBootConfig {
  float aMax = 0.0f;         // [mm/s^2] linear accel-ramp ceiling
  float aDecel = 0.0f;       // [mm/s^2] linear decel-taper ceiling
  float alphaMax = 0.0f;     // [rad/s^2] angular accel-ramp ceiling
  float alphaDecel = 0.0f;   // [rad/s^2] angular decel-taper ceiling
  float jMax = 0.0f;         // [mm/s^3] linear jerk ceiling
  float yawJerkMax = 0.0f;   // [rad/s^3] angular jerk ceiling
};

// The boot ShaperBootConfig default — fail-closed baked accel/decel
// ceilings, see ShaperBootConfig's own doc comment above and
// gen_boot_config.py's shaper_config_for_config().
ShaperBootConfig defaultShaperConfig();

}  // namespace Config
