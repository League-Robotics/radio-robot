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

// kRobotProfileName — the calibration-profile identifier baked in at
// codegen time: the active robot JSON's own filename stem (e.g.
// "tovez_nocal" for data/robots/tovez_nocal.json), or "unconfigured" when
// no robot JSON was found (gen_boot_config.py's own "(firmware defaults)"
// sentinel path). `ID:`'s reply content (App::Comms, comms.cpp) reports
// this alongside kDrivetrainType below — distinct from `DEVICE:`'s own
// hardware identity (formatBanner(), com/banner.cpp).
extern const char kRobotProfileName[];

// kDrivetrainType — "differential" or "mecanum" (the robot JSON's own
// identity.drivetrain_type enum, robot_config.schema.json, defaulting to
// "differential" per that schema's own documented default when the key
// is absent). `ID:`'s other field, alongside kRobotProfileName above.
// Deliberately NOT derived from any wire-level msg::DrivetrainConfig
// field — defaultDrivetrainConfig() never bakes DrivetrainConfig::
// half_track (it keeps its wire default-member-initializer 0.0f for
// every profile), so that field cannot answer this question; this string
// constant is baked directly from the schema's own authoritative field
// instead (gen_boot_config.py's drivetrain_type_for_config()).
extern const char kDrivetrainType[];

// The OTOS lever-arm mounting offset plus linear/angular scale multipliers,
// baked from the active robot JSON's geometry.odometry_offset_mm
// (x/y/yaw_rad) and calibration.otos_linear_scale/otos_angular_scale.
// Additive to defaultMotorConfigs()/defaultDrivetrainConfig() above — no
// existing mapping is touched.
//
// Boot-time-baked only, deliberately NOT a live SET/wire surface itself —
// see DESIGN.md §3/§4 for why. Consumed directly by main.cpp's
// Devices::RealOtos construction; the scale multipliers are converted to the
// OTOS chip's raw register scalar once at Devices::RealOtos::begin(), not
// re-derived per wire call (docs/protocol-v2.md §11). A SEPARATE, live
// runtime override exists on top of this boot bake — `OtosConfigPatch`
// (config.proto), applied by RobotLoop::handleConfig directly against
// Devices::Otos's setLinearScalar()/setAngularScalar()/setOffset()/init()
// — this struct itself is never touched at runtime; only the chip's own
// registers are re-written.
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

// EstimatorBootConfig — App::StateEstimator's fail-closed boot-time
// fusion-weight defaults, baked from the robot JSON's `estimator` section
// (data/robots/robot_config.schema.json). Field-for-field mirror of
// App::StateEstimator::FusionWeights (app/state_estimator.h), but declared
// independently here rather than reusing that type directly: config/ may
// only depend on messages/ (docs/design/design.md §5's dependency
// diagram), never on app/. main.cpp converts this into an
// App::FusionWeights at the one place both types are visible, the same
// pattern toDeviceMotorConfig() already uses for msg::MotorConfig ->
// Devices::MotorConfig.
//
// headingOtos/omegaOtos are committed 0.0 in every robot JSON today
// (encoder-only) — dimensionless [0..1] blend weights, no unit tag
// (coding-standards.md). staleness carries a reasoned per-robot
// placeholder (each robot JSON's own inline comment documents the
// derivation).
struct EstimatorBootConfig {
  float headingOtos = 0.0f;  // [0..1] blend weight: body heading vs OTOS heading
  float omegaOtos = 0.0f;    // [0..1] blend weight: body omega vs OTOS omega
  uint32_t staleness = 200;  // [ms] max OTOS reading age still eligible to blend
};

// The boot EstimatorBootConfig default — fail-closed baked fusion weights,
// see EstimatorBootConfig's own doc comment above and
// gen_boot_config.py's estimator_config_for_config().
EstimatorBootConfig defaultEstimatorConfig();

// ShaperBootConfig — accel/decel/jerk magnitude ceilings, baked from the
// robot JSON's `control.a_max`/`control.a_decel`/`control.alpha_max`/
// `control.alpha_decel` (data/robots/robot_config.schema.json). Currently
// unread by any live consumer — main.cpp constructs Motion::PlannerLimits
// from its own plant-validated constants, not from this struct; kept
// declared because removing it is a boot-config schema change, not a
// src/firm/src/motion code-boundary one.
//
// jMax/yawJerkMax bound how fast the commanded ACCELERATION itself may
// change (the S-curve's own "corners").
//
// REQUIRED, same fail-closed posture as every other struct here: a robot
// JSON missing any of the six `control.a_max`/`a_decel`/`alpha_max`/
// `alpha_decel`/`j_max`/`yaw_jerk_max` keys fails codegen loudly (same
// MissingRobotConfigKeyError gen_boot_config.py's own `_require()` already
// raises for every other REQUIRED field) rather than silently shipping an
// unshaped boot image.
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

// DriveBootConfig -- App::Drive's per-robot wheel calibration, baked from
// the robot JSON's `control.duty_per_speed_left`/`duty_per_speed_right`/
// `crawl_pulse` (data/robots/robot_config.schema.json). The `kff` CONFIG
// key is not an escape hatch for this: it sets both wheels to a single
// value, flattening the measured ~10% L/R asymmetry with no way to
// restore it short of a rebuild.
//
// dutyPerSpeed* is the INVERSE of the measured plant gain: duty =
// speed * dutyPerSpeed. Per-wheel because the two gearboxes genuinely
// differ (measured ~560 mm/s per duty left vs ~510 right).
//
// crawlPulse ships at 0 (OFF). True breakaway is 1-6% duty and
// state-dependent; a naively larger value (e.g. 0.20, sized against an
// apparent 0.10-0.19 "dead zone") pulses across a wide speed band for a
// stiction problem that largely is not there (see the standalone duty_min
// prober, src/tests/firmware/duty_min/RESULTS.md). Re-enable only if slow
// speeds genuinely stall, sized from prober data (~0.05).
//
// REQUIRED, same fail-closed posture as every other struct here: a robot
// JSON missing any of the three keys fails codegen loudly. App::Drive
// itself carries NO calibration defaults (drive.h) -- an unconfigured
// Drive refuses to drive rather than quietly using another robot's
// numbers.
struct DriveBootConfig {
  float dutyPerSpeedLeft = 0.0f;   // [duty/(mm/s)] 1 / measured plant gain, left
  float dutyPerSpeedRight = 0.0f;  // [duty/(mm/s)] 1 / measured plant gain, right
  float crawlPulse = 0.0f;         // [-1, 1] sub-breakaway pulse amplitude; 0 = off

  // Commanded->actual correction, per wheel per direction of approach:
  // measured = gain*commanded + intercept (docs/design/
  // wheel-speed-command-mapping.md). Drive inverts it for the feedforward.
  // gain 1 / intercept 0 == no correction (an uncalibrated robot).
  float gainLeftAccel = 1.0f;        // measured/commanded slope, left wheel, accel
  float interceptLeftAccel = 0.0f;   // [mm/s] its intercept
  float gainLeftDecel = 1.0f;        // measured/commanded slope, left wheel, decel
  float interceptLeftDecel = 0.0f;   // [mm/s] its intercept
  float gainRightAccel = 1.0f;        // measured/commanded slope, right wheel, accel
  float interceptRightAccel = 0.0f;   // [mm/s] its intercept
  float gainRightDecel = 1.0f;        // measured/commanded slope, right wheel, decel
  float interceptRightDecel = 0.0f;   // [mm/s] its intercept
};

// The boot DriveBootConfig default -- see DriveBootConfig's own doc comment
// above and gen_boot_config.py's drive_config_for_config().
DriveBootConfig defaultDriveConfig();

// PlannerBootConfig -- Motion::Planner's full tuning surface (profile
// ceilings, loop timing, settle/rest, duty-stage PID, trim loop), baked
// from the active robot JSON's `planner` block
// (data/robots/robot_config.schema.json).
//
// Field-for-field mirror of the TUNABLE subset of Motion::PlannerLimits
// (src/motion/planner/planner_types.h) -- NOT the whole struct: trackWidth
// and velocityFilterWeight stay sourced from DrivetrainConfig, and
// otosStaleness/headingOtosWeight are not set here -- they keep
// PlannerLimits' own struct defaults. Declared independently here, rather
// than reusing Motion::PlannerLimits directly, because config/ may depend
// only on messages/ (docs/design/design.md §5's dependency diagram),
// never on src/motion -- the same reasoning EstimatorBootConfig's own doc
// comment above gives for not reusing App::StateEstimator::FusionWeights
// directly. main.cpp (the one place both types are visible) converts this
// struct into a Motion::PlannerLimits, the same toDeviceMotorConfig()
// pattern already uses for msg::MotorConfig -> Devices::MotorConfig.
//
// velKff/velKaff/trimKaff are DERIVED, not stored raw in the robot JSON:
// the JSON carries the measured plant primitives (`planner.plant_gain`
// [mm/s per duty], `planner.plant_tau` [s]) and gen_boot_config.py's
// planner_config_for_config() computes velKff = 1/plant_gain,
// velKaff = plant_tau/plant_gain, trimKaff = plant_tau/2 once, in the
// generator -- store the measurement, derive the gain, so the derivation
// stays in one reviewed place.
//
// REQUIRED, same fail-closed posture as every other struct here: a robot
// JSON missing the `planner` block (or any of its keys) fails codegen
// loudly (MissingRobotConfigKeyError) -- a robot must never boot with
// another robot's plant measurements.
struct PlannerBootConfig {
  // Profile ceilings.
  float vMax = 0.0f;        // [mm/s] linear velocity ceiling
  float aMax = 0.0f;        // [mm/s^2] linear accel-ramp ceiling
  float aDecel = 0.0f;      // [mm/s^2] linear decel-taper ceiling
  float omegaMax = 0.0f;    // [rad/s] angular velocity ceiling
  float alphaMax = 0.0f;    // [rad/s^2] angular accel-ramp ceiling
  float alphaDecel = 0.0f;  // [rad/s^2] angular decel-taper ceiling
  float jerkMax = 0.0f;     // [mm/s^3] linear jerk ceiling
  float yawJerkMax = 0.0f;  // [rad/s^3] angular jerk ceiling

  // Loop timing -- the MEASURED delivered cycle period, not a nominal.
  float controlPeriod = 0.0f;   // [ms]
  float actuationDelay = 0.0f;  // [ms] command-staged-to-wheels latency

  // Settle/rest and heading hold.
  bool requireSettle = false;
  float settleRestVelocity = 0.0f;    // [mm/s]
  float settleRestOmega = 0.0f;       // [rad/s]
  float settleWindow = 0.0f;          // [ms] max extra wait past profile-complete
  float settleEpsilonLinear = 0.0f;   // [mm]
  float settleEpsilonAngular = 0.0f;  // [rad]
  float headingHoldGain = 0.0f;       // [1/s] rad/s of correction per rad of error

  // Duty-stage PID (Motion::Planner's M4 stage). velKff/velKaff derived
  // from planner.plant_gain/planner.plant_tau -- see this struct's own
  // doc comment above.
  float velKff = 0.0f;          // [duty/(mm/s)] feedforward slope, = 1/plant_gain
  float velKp = 0.0f;           // [duty/(mm/s)] proportional
  float velKi = 0.0f;           // [duty/(mm/s)/s] integral rate
  float velIMax = 0.0f;         // [duty] integrator clamp
  float velKaff = 0.0f;         // [duty/(mm/s^2)] accel feedforward, = plant_tau/plant_gain
  float velIAccelGate = 0.0f;   // [mm/s^2] integral ramp gate
  float dutyFloor = 0.0f;       // [-1,1] stiction floor

  // Velocity-domain trim (wheel_trim.h). trimKaff derived from
  // planner.plant_tau -- see this struct's own doc comment above.
  float trimKp = 0.0f;             // [1] dimensionless: mm/s of trim per mm/s of error
  float trimKi = 0.0f;             // [1/s]
  float trimIMax = 0.0f;           // [mm/s] integrator clamp
  float trimKaff = 0.0f;           // [s] accel feedforward, = plant_tau/2
  float trimMax = 0.0f;            // [mm/s] total trim authority
  float decelPlanFraction = 0.0f;  // [1] fraction of decel ceiling to plan brake-start against
};

// The boot PlannerBootConfig default -- see PlannerBootConfig's own doc
// comment above and gen_boot_config.py's planner_config_for_config().
PlannerBootConfig defaultPlannerLimits();

}  // namespace Config
