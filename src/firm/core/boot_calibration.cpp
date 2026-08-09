// boot_calibration.cpp -- see boot_calibration.h for why this is a
// separate TU from boot_wiring.cpp.
#include "core/boot_calibration.h"

#include <cmath>

// bootPlannerLimits() below reads Config::defaultPlannerLimits()/
// Config::PlannerBootConfig directly -- 132-015 moved this include here
// (from boot_calibration.h) since it is the only remaining use of
// config/boot_config.h in this translation unit; the header itself no
// longer names any boot_config.h type in its own declarations now that
// DriveBootConfig/WheelControllerBootConfig are deleted.
#include "config/boot_config.h"

namespace Core {

namespace {

// kConfigureRestVelocity -- mirrors NezhaMotor::kReconfigureRestVelocity/
// MotorArmor::kRestVelocity (both 5.0f, nezha_motor.h/motor_armor.h) --
// a leaf/subsystem-local constant for a similar guard, deliberately NOT
// shared with either (nezha_motor.h's own kReconfigureRestVelocity
// comment establishes this project's precedent: each guard gets its own
// named constant at its own layer).
constexpr float kConfigureRestVelocity = 5.0f;  // [mm/s]

}  // namespace

Hal::MotorConfig toDeviceMotorConfig(const msg::MotorConfig& src) {
  Hal::MotorConfig cfg;
  cfg.wheelTravelCalib = src.travel_calib;
  cfg.fwdSign = src.fwd_sign;
  cfg.slewRate = src.slew_rate;
  cfg.port = src.port;
  // Hal::MotorConfig's reversalDwell/outputDeadband are plain required
  // floats -- gen_boot_config.py's required-key gate guarantees
  // src.reversal_dwell/src.output_deadband are always set (.has == true)
  // here, so read .val directly rather than changing the wire
  // msg::MotorConfig itself (still Opt<float> -- the wire schema is out
  // of scope here).
  cfg.reversalDwell = src.reversal_dwell.val;
  cfg.outputDeadband = src.output_deadband.val;
  cfg.polled = src.polled;
  return cfg;
}

float effectiveTrackWidth(const msg::DrivetrainConfig& drivetrainConfig) {
  const float kScrub = drivetrainConfig.rotational_slip;
  return (kScrub > 0.0f) ? (drivetrainConfig.trackwidth / kScrub)
                        : drivetrainConfig.trackwidth;
}

Motion::PlannerLimits bootPlannerLimits(const msg::DrivetrainConfig& drivetrainConfig,
                                        float trackWidth) {
  const Config::PlannerBootConfig src = Config::defaultPlannerLimits();

  Motion::PlannerLimits out;
  out.ceilings.vMax = src.vMax;
  out.ceilings.aMax = src.aMax;
  out.ceilings.aDecel = src.aDecel;
  out.ceilings.omegaMax = src.omegaMax;
  out.ceilings.alphaMax = src.alphaMax;
  out.ceilings.alphaDecel = src.alphaDecel;
  out.ceilings.jerkMax = src.jerkMax;
  out.ceilings.yawJerkMax = src.yawJerkMax;

  out.plant.controlPeriod = src.controlPeriod;
  out.plant.actuationDelay = src.actuationDelay;

  out.landing.settleRestVelocity = src.settleRestVelocity;
  out.landing.settleRestOmega = src.settleRestOmega;
  out.landing.settleEpsilonLinear = src.settleEpsilonLinear;
  out.landing.settleEpsilonAngular = src.settleEpsilonAngular;
  out.landing.decelPlanFraction = src.decelPlanFraction;
  out.landing.alignTol = src.alignTol;
  out.landing.alignMaxNudges = src.alignMaxNudges;

  out.tracking.headingHoldGain = src.headingHoldGain;

  // trackWidth/velocityFilterWeight are the two PlannerLimits fields NOT
  // sourced from Config::PlannerBootConfig -- see that struct's own doc
  // comment (config/boot_config.h). trackWidth is the caller's own
  // (scrub-corrected, by default effectiveTrackWidth()'s) value;
  // velocityFilterWeight mirrors the robot JSON's vel_filt_alpha EMA
  // weight, with the same >0.05 sanity floor main.cpp always applied.
  out.plant.trackWidth = trackWidth;
  out.plant.velocityFilterWeight = drivetrainConfig.vel_filt_alpha > 0.05f
                                        ? drivetrainConfig.vel_filt_alpha
                                        : 1.0f;
  return out;
}

// installShaperLimits/installRotationCalibration/installDriveCalibration/
// installWheelController -- DELETED, 132-015 (dead-code sweep). All three
// were confirmed by a fresh grep to have zero remaining call sites
// (Configurator::install(), configurator.cpp, does this fan-out inline
// now, reading Config::Robot directly -- see this file's own header for
// how that ticket-006 retarget left these with "no callers left"); see
// boot_config.h's own note on DriveBootConfig/WheelControllerBootConfig
// (deleted the same ticket) for the struct-level half of this cleanup.
// installRotationCalibration itself was already deleted earlier (132-007).

// configurePlanner -- see boot_calibration.h's own doc comment.
void configurePlanner(Motion::Planner& planner, const Config::Robot& config) {
  // 132-017 split: the six shaper ceilings live on config.plannerShaper
  // now (a LIVE ConfigGroupTarget), not config.planner (the boot-only
  // remainder) -- see robot_config.proto's PlannerShaper header comment.
  planner.applyShaperLimits(config.plannerShaper.a_max, config.plannerShaper.a_decel,
                            config.plannerShaper.alpha_max, config.plannerShaper.alpha_decel,
                            config.plannerShaper.jerk_max, config.plannerShaper.yaw_jerk_max);
}

// configureMotor -- see boot_calibration.h's own doc comment.
bool configureMotor(Hal::Motor& motor, const Config::Robot& config, bool isLeft) {
  const bool atRest =
      std::fabs(motor.velocity()) < kConfigureRestVelocity && motor.appliedDuty() == 0.0f;
  if (!atRest) return false;

  motor.applyTravelCalib(isLeft ? config.motors.travel_calib_left
                                : config.motors.travel_calib_right);
  return true;
}

// configureOtos -- see boot_calibration.h's own doc comment. linear_scale/
// angular_scale are converted through Hardware::scaleToRegister() (otos.h)
// before reaching setLinearScalar()/setAngularScalar() -- those two setters
// take the chip's raw int8 register domain directly, never the config
// MULTIPLIER domain (1.0 = no correction) config.otos itself holds. This is
// the SAME conversion RealOtos::begin() applies to the baked value at boot
// (otos.cpp) -- 132-010 closes trap 3 (the-configuration-object.md) by
// applying it here too, so a live OTOS push and a boot bake agree on what a
// given multiplier means. setOffset()'s x/y/yaw are unaffected -- that
// setter already takes the value directly, no domain conversion (otos.h's
// own setOffset() doc comment spells out the distinction from the scale
// registers).
void configureOtos(Hal::Otos& otos, const Config::Robot& config) {
  otos.setLinearScalar(static_cast<float>(Hardware::scaleToRegister(config.otos.linear_scale)));
  otos.setAngularScalar(static_cast<float>(Hardware::scaleToRegister(config.otos.angular_scale)));
  otos.setOffset(config.otos.offset_x, config.otos.offset_y, config.otos.offset_yaw);
}

// configureNavigator -- see boot_calibration.h's own doc comment.
void configureNavigator(Motion::NavigatorLimits& limits, const Config::Robot& config) {
  limits.trackWidth = config.effectiveTrackWidth();  // [mm] scrub-corrected, matches Drive/Odometry/PlannerLimits
  limits.speed = config.navigator.speed;
  limits.maxWheelStep = config.navigator.max_wheel_step;
  limits.behindAngle = config.navigator.behind_angle;
  limits.turnFirstAngle = config.navigator.turn_first_angle;
  limits.approachRadius = config.navigator.approach_radius;
  limits.approachSpeed = config.navigator.approach_speed;
  limits.defaultArrivalTolerance = config.navigator.default_arrival_tolerance;
  limits.yawSign = config.navigator.yaw_sign;
}

}  // namespace Core
