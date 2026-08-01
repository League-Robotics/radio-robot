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
// sentinel path). Sprint 124 architecture Decision 4: `ID:`'s reply
// content (App::Comms, comms.cpp) reports this alongside kDrivetrainType
// below — distinct from `DEVICE:`'s own hardware identity (formatBanner(),
// com/banner.cpp). One new generated string constant, the same pattern
// types/version_generated.h already established for FIRMWARE_VERSION_STR
// — zero new generator machinery.
extern const char kRobotProfileName[];

// kDrivetrainType — "differential" or "mecanum" (the robot JSON's own
// identity.drivetrain_type enum, robot_config.schema.json, defaulting to
// "differential" per that schema's own documented default when the key
// is absent). Sprint 124 architecture Decision 4: `ID:`'s other field,
// alongside kRobotProfileName above. Deliberately NOT derived from any
// wire-level msg::DrivetrainConfig field — defaultDrivetrainConfig()
// never bakes DrivetrainConfig::half_track (it keeps its wire
// default-member-initializer 0.0f for every profile), so that field
// cannot answer this question; this string constant is baked directly
// from the schema's own authoritative field instead (gen_boot_config.py's
// drivetrain_type_for_config()).
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
// (formerly declared here, feeding a now-deleted per-Move stop-condition
// time-lead mechanism) -- DELETED (118 ticket 004, land-at-zero-
// completion-delete-stop-lead.md): the anticipation mechanism it fed no
// longer exists (see docs/design/history/land-at-zero-margin-derivation.md
// for the land-at-zero completion predicate that replaced it, itself
// deleted as dead code in sprint 128 ticket 014) -- there is no lead
// value left to bake.
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
// accel/decel/jerk magnitude ceilings, baked from the robot JSON's
// `control.a_max`/`control.a_decel`/`control.alpha_max`/
// `control.alpha_decel` (data/robots/robot_config.schema.json). These
// fields originally fed a velocity-shaping consumer that was itself
// deleted in sprint 128 ticket 014 as dead code (zero callers,
// superseded by Motion::Planner's own PlannerLimits) -- this struct and
// its generator (gen_boot_config.py) are themselves currently unread by
// any live consumer (main.cpp constructs Motion::PlannerLimits from its
// own plant-validated constants, not from this struct); kept declared
// here rather than deleted outright since removing it is a boot-config
// schema change outside this ticket's scope, not a src/firm/src/motion
// code-boundary one.
//
// aMax/aDecel/jMax/yawJerkMax are NOT new fields — they are the deleted
// msg::PlannerConfig's own `a_max`/`a_decel`/`j_max`/`yaw_jerk_max`,
// orphaned dead data since 115-003's motion-stack excision
// (gen_boot_config.py's own module docstring used to document all four as
// "unread by this generator"). alphaMax/alphaDecel ARE new (a_max/
// a_decel's own angular sibling — no msg::PlannerConfig predecessor
// existed for either); yaw_jerk_max already existed as j_max's own
// angular sibling, so no NEW angular jerk field was needed the way
// alphaMax/alphaDecel were for accel/decel.
//
// jMax/yawJerkMax (jerk-limited S-curve stage, 2026-07-22 stakeholder
// correction on top of this struct's own first accel-limited pass): how
// fast the commanded ACCELERATION itself may change, bounding the
// S-curve's own "corners". `j_max`/`yaw_jerk_max` already existed as
// REQUIRED, unread `control.*` keys in every robot JSON since sprint 114
// (098-001).
//
// REQUIRED (config-as-truth, sprint 114's own fail-closed posture,
// extended here): a robot JSON missing any of the six `control.a_max`/
// `a_decel`/`alpha_max`/`alpha_decel`/`j_max`/`yaw_jerk_max` keys fails
// codegen loudly (same MissingRobotConfigKeyError gen_boot_config.py's own
// `_require()` already raises for every other REQUIRED field) rather than
// silently shipping an unshaped boot image.
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

// DriveBootConfig (command-ingestion-ring-buffered-comms-subsystem-routing-
// two-stops.md §6) -- App::Drive's per-robot wheel calibration, baked from
// the robot JSON's `control.duty_per_speed_left`/`duty_per_speed_right`/
// `crawl_pulse` (data/robots/robot_config.schema.json).
//
// Both of these used to be hard-coded C++: the duty-per-speed pair as
// member initializers ON App::Drive itself, the crawl amplitude as a bare
// `drive.setCrawlPulse()` call in main.cpp. That baked ONE robot's
// gearboxes, on one battery, measured on one evening, into the class
// definition -- every other robot silently inherited it, and changing it
// meant editing C++ and reflashing. The `kff` CONFIG key is not an escape
// hatch: it sets both wheels to a single value, flattening the measured
// ~10% L/R asymmetry with no way to restore it short of a rebuild.
//
// dutyPerSpeed* is the INVERSE of the measured plant gain: duty =
// speed * dutyPerSpeed. Per-wheel because the two gearboxes genuinely
// differ (speed_sweep 2026-07-27: L ~560, R ~510 mm/s per duty).
//
// crawlPulse ships at 0 (OFF). The previously committed 0.20 was sized
// against the duty sweep's apparent 0.10-0.19 "dead zone", which the
// standalone duty_min prober (src/tests/firmware/duty_min/RESULTS.md)
// later showed to be an artifact of that sweep's own criterion (all three
// cold-start 500 ms repeats must move). True breakaway is 1-6% duty and
// state-dependent, so at 0.20 the crawl/continuous boundary sat at
// ~107 mm/s commanded and every speed below that ran pulsed -- ripple
// across a wide band for a stiction problem that largely is not there.
// Re-enable only if slow speeds genuinely stall, sized from the prober
// data (~0.05), through this config key.
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

// PlannerBootConfig (129-009, config consolidation) -- Motion::Planner's
// full tuning surface (profile ceilings, loop timing, settle/rest,
// duty-stage PID, trim loop), baked from the active robot JSON's new
// `planner` block (data/robots/robot_config.schema.json). Before this
// ticket every one of these values was a C++ literal assembled directly
// in main.cpp -- this struct/loader moves them to the same config-as-truth
// path every other per-robot calibration already uses (sprint 114).
//
// Field-for-field mirror of the TUNABLE subset of Motion::PlannerLimits
// (src/motion/planner/planner_types.h) -- NOT the whole struct: trackWidth
// and velocityFilterWeight stay sourced from DrivetrainConfig (unchanged),
// and otosStaleness/headingOtosWeight are untouched by this ticket (never
// set by main.cpp before this move, so they keep PlannerLimits' own
// struct defaults). Declared independently here, rather than reusing
// Motion::PlannerLimits directly, because config/ may depend only on
// messages/ (docs/design/design.md §5's dependency diagram), never on
// src/motion -- the same reasoning EstimatorBootConfig's own doc comment
// above gives for not reusing App::StateEstimator::FusionWeights directly.
// main.cpp (the one place both types are visible) converts this struct
// into a Motion::PlannerLimits, the same toDeviceMotorConfig() pattern
// already uses for msg::MotorConfig -> Devices::MotorConfig.
//
// velKff/velKaff/trimKaff are DERIVED, not stored raw in the robot JSON:
// the JSON carries the measured plant primitives (`planner.plant_gain`
// [mm/s per duty], `planner.plant_tau` [s]) and gen_boot_config.py's
// planner_config_for_config() computes velKff = 1/plant_gain,
// velKaff = plant_tau/plant_gain, trimKaff = plant_tau/2 once, in the
// generator -- "store the measurement, derive the gain, so the derivation
// stays in one reviewed place" (ticket 03's own instruction).
//
// REQUIRED, same fail-closed posture as every other struct here: a robot
// JSON missing the `planner` block (or any of its keys) fails codegen
// loudly (MissingRobotConfigKeyError) -- a robot must never boot with
// another robot's plant measurements (sprint 114's own convention,
// `Config::DriveBootConfig`'s own history is exactly this failure mode).
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
