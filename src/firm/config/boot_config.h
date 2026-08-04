// boot_config.h — Config: the robot's build-time boot configuration.
//
// DEFINED by the AUTO-GENERATED config/boot_config.cpp — see DESIGN.md.
// Never hand-edit boot_config.cpp; never hardcode calibration in main.cpp.
#pragma once

#include <stdint.h>

#include "messages/drivetrain.h"
#include "messages/motor.h"
#include "messages/robot_config.h"

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
// runtime override exists on top of this boot bake — the OTOS group's own
// live wire arm (robot_config.proto's `Otos`, 132-008), applied by
// App::Configurator::install(OTOS) directly against Devices::Otos's
// setLinearScalar()/setAngularScalar()/setOffset() (`init()` has no
// Config::Robot-shaped wire path any more, 132-013) — this struct itself
// is never touched at runtime; only the chip's own registers are
// re-written.
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

// ShaperBootConfig/defaultShaperConfig() -- DELETED, 132-015 (dead-code
// sweep, the-configuration-object.md). Formerly accel/decel/jerk magnitude
// ceilings baked from the robot JSON's `control.a_max`/`control.a_decel`/
// `control.alpha_max`/`control.alpha_decel`/`control.j_max`/
// `control.yaw_jerk_max` -- confirmed by a fresh grep to have ZERO live
// consumers (main.cpp constructs Motion::PlannerLimits from
// PlannerBootConfig/defaultPlannerLimits() below, never from this struct;
// its one intended consumer, a velocity-shaping stage, was itself deleted
// as dead code in sprint 128 ticket 014). The mirroring
// `msg::Planner.shaper_*` schema fields (robot_config.proto) are deleted
// (`reserved`) the same ticket; see that message's own trailing comment.
//
// DriveBootConfig/defaultDriveConfig() -- DELETED, 132-015. Formerly
// App::Drive's per-robot wheel calibration
// (dutyPerSpeedLeft/Right/crawlPulse/gain*/intercept* -- control.
// duty_per_speed_left/right/crawl_pulse and the 8 wheel_gain_*/
// wheel_intercept_* keys). Confirmed by a fresh grep to have ZERO live
// consumers: ticket 006 ("Configurator owns Config::Robot") retargeted
// RobotGraph's composition root onto Configurator::loadBaked() +
// Config::defaultDriveGroup() (msg::Drive, below the "Config::Robot group
// defaults" banner in boot_config.cpp) -- the last caller of this
// function -- leaving it exactly the "no callers left" state this file's
// own module docstring (gen_boot_config.py) predicted a "later cleanup
// ticket" (this one) would find and delete.
//
// WheelControllerBootConfig/defaultWheelControllerConfig() -- DELETED,
// 132-015, same reason and same ticket-006 retarget as DriveBootConfig
// immediately above (superseded by Config::defaultWheelControlGroup(),
// msg::WheelControl). App::installShaperLimits()/installDriveCalibration()/
// installWheelController() (app/boot_calibration.{h,cpp}), the three free
// functions that used to install these two structs' values onto
// Motion::Planner/App::Drive, are deleted alongside them -- Configurator::
// install() (app/configurator.cpp) now does that fan-out instead, reading
// Config::Robot directly.
//
// PlannerBootConfig -- Motion::Planner's tuning surface (profile ceilings,
// loop timing, settle/rest, heading hold), baked from the active robot
// JSON's `planner` block (data/robots/robot_config.schema.json).
//
// Field-for-field mirror of the TUNABLE subset of Motion::PlannerLimits
// (src/motion/planner/planner_types.h) -- NOT the whole struct: trackWidth
// and velocityFilterWeight stay sourced from DrivetrainConfig. Declared
// independently here, rather than reusing Motion::PlannerLimits directly,
// because config/ may depend only on messages/ (docs/design/design.md
// §5's dependency diagram), never on src/motion -- the same reasoning
// EstimatorBootConfig's own doc comment above gives for not reusing
// App::StateEstimator::FusionWeights directly. main.cpp (the one place
// both types are visible) converts this struct into a
// Motion::PlannerLimits, the same toDeviceMotorConfig() pattern already
// uses for msg::MotorConfig -> Devices::MotorConfig.
//
// 130-009 (PlannerLimits 34->23(->18) reshape): this struct's own dead
// fields are cut alongside PlannerLimits' -- `requireSettle`/
// `settleWindow` (the settle-confirm defer path, dissolved by 130-008),
// the M4 duty-stage gains `velKff`/`velKp`/`velKi`/`velIMax`/`velKaff`/
// `velIAccelGate`/`dutyFloor` (WheelPid/stageDuty() deleted by 130-007),
// and the planner-side velocity trim `trimKp`/`trimKi`/`trimIMax`/
// `trimKaff`/`trimMax` (Motion::WheelTrim deleted by 130-005, superseded
// by App::Drive's own controller -- Config::WheelControllerBootConfig
// above is that controller's config, a disjoint field set). The JSON's
// `plant_gain`/`plant_tau` measured primitives stay (still read for
// `trimKaff`'s derivation... no -- trimKaff itself is cut too; plant_tau
// alone has no PlannerBootConfig consumer left as of this ticket, kept in
// the schema/JSON as recorded measured data, same "declared but unread by
// this generator" posture ShaperBootConfig's own doc comment above
// documents for a different struct).
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

  // Loop timing. 131-005: App::RobotLoop::cycle()'s trailing pacing block
  // targets an ABSOLUTE end-of-cycle deadline, so the delivered period
  // converges to App::RobotLoop::kCycle by construction rather than
  // needing to be independently re-measured and baked in here whenever
  // kCycle changes (the prior framing -- "the MEASURED delivered cycle
  // period, not a nominal" -- is exactly the "note is the symptom"
  // pattern that let a stale measured constant ship twice; see
  // robot_loop.h's kCycle doc comment for the full history). This field
  // is simply "= kCycle" on every current robot profile.
  float controlPeriod = 0.0f;   // [ms]
  float actuationDelay = 0.0f;  // [ms] command-staged-to-wheels latency

  // Settle/rest and heading hold.
  float settleRestVelocity = 0.0f;    // [mm/s]
  float settleRestOmega = 0.0f;       // [rad/s]
  float settleEpsilonLinear = 0.0f;   // [mm]
  float settleEpsilonAngular = 0.0f;  // [rad]
  float headingHoldGain = 0.0f;       // [1/s] rad/s of correction per rad of error

  float decelPlanFraction = 0.0f;  // [1] fraction of decel ceiling to plan brake-start against
};

// The boot PlannerBootConfig default -- see PlannerBootConfig's own doc
// comment above and gen_boot_config.py's planner_config_for_config().
PlannerBootConfig defaultPlannerLimits();

// ---------------------------------------------------------------------------
// Config::Robot group defaults (132-005, sprint 132 "configuration
// discipline: one owned object, live wire config with read-back, patch-
// surface retirement, JSON reshape, and per-wheel drive calibration").
//
// Seven no-argument functions, one per msg::ConfigGroupTarget
// (messages/robot_config.h, generated from src/protos/robot_config.proto
// by ticket 002) -- the same robot-JSON `_require()`/`_get()` mappings
// gen_boot_config.py already performs above, retargeted onto the NEW
// generated group struct layout instead of the hand-declared *BootConfig
// structs above. See gen_boot_config.py's own module docstring
// ("132-005") for why this is ADDITIVE for this ticket, not a replacement:
// boot_wiring.cpp/boot_calibration.cpp/main.cpp still call the functions
// above every boot; ticket 006 ("Configurator owns Config::Robot") is what
// retargets RobotGraph's composition root onto these instead, via
// `Configurator::loadBaked()`.
//
// Every consumer group in msg::ConfigGroupTarget gets one function here:
// Geometry/Motors/Drive/WheelControl/Planner/PlannerShaper/Otos/Estimator.
// Motors' travel_calib_left/right and fwd_sign_left/right are the
// drive-pair-only slice of the per-port arrays defaultMotorConfigs() above
// bakes (no per-port array in this schema -- see robot_config.proto's own
// header checklist). Planner/PlannerShaper -- SPLIT, 132-017 (JSON
// reshape ticket, stakeholder-sanctioned mid-sprint scope addition): the
// six shaper-ceiling fields (a_max/a_decel/alpha_max/alpha_decel/jerk_max/
// yaw_jerk_max) moved from Planner (boot-only) into their own LIVE
// PlannerShaper group -- see robot_config.proto's PlannerShaper message
// header comment for why. The dead Config::ShaperBootConfig surface
// (unrelated `shaper_*`-prefixed field set) was deleted outright, 132-015
// -- not to be confused with this split.
// ---------------------------------------------------------------------------
msg::Geometry defaultGeometryGroup();
msg::Motors defaultMotorsGroup();
msg::Drive defaultDriveGroup();
msg::WheelControl defaultWheelControlGroup();
msg::Planner defaultPlannerGroup();
msg::PlannerShaper defaultPlannerShaperGroup();
msg::Otos defaultOtosGroup();
msg::Estimator defaultEstimatorGroup();

}  // namespace Config
