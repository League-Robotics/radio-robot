// boot_calibration.cpp -- see boot_calibration.h for why this is a
// separate TU from boot_wiring.cpp.
#include "app/boot_calibration.h"

#include <cmath>

namespace App {

namespace {

// kConfigureRestVelocity -- mirrors NezhaMotor::kReconfigureRestVelocity/
// MotorArmor::kRestVelocity (both 5.0f, nezha_motor.h/motor_armor.h) --
// a leaf/subsystem-local constant for a similar guard, deliberately NOT
// shared with either (nezha_motor.h's own kReconfigureRestVelocity
// comment establishes this project's precedent: each guard gets its own
// named constant at its own layer).
constexpr float kConfigureRestVelocity = 5.0f;  // [mm/s]

}  // namespace

Devices::MotorConfig toDeviceMotorConfig(const msg::MotorConfig& src) {
  Devices::MotorConfig cfg;
  cfg.wheelTravelCalib = src.travel_calib;
  cfg.fwdSign = src.fwd_sign;
  cfg.slewRate = src.slew_rate;
  cfg.port = src.port;
  // Devices::MotorConfig's reversalDwell/outputDeadband are plain required
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

void installShaperLimits(Motion::Planner& planner, const Motion::PlannerLimits& limits) {
  planner.applyShaperLimits(limits.ceilings.aMax, limits.ceilings.aDecel,
                            limits.ceilings.alphaMax, limits.ceilings.alphaDecel,
                            limits.ceilings.jerkMax, limits.ceilings.yawJerkMax);
}

// installRotationCalibration -- DELETED (132-007); see boot_calibration.h's
// own note at this declaration's old spot.

void installDriveCalibration(Drive& drive, const Config::DriveBootConfig& driveConfig) {
  // DEAD (see this function's own doc comment, boot_calibration.h) --
  // body deliberately left at its pre-132-009 "MEASURED, NOT CONFIGURED"
  // shape, not updated to match Configurator::install()'s reversal.
  drive.setDutyPerSpeed(Drive::kDutyPerSpeed, Drive::kDutyPerSpeed);
  drive.setWheelCorrection(
      driveConfig.gainLeftAccel, driveConfig.interceptLeftAccel, driveConfig.gainLeftDecel,
      driveConfig.interceptLeftDecel, driveConfig.gainRightAccel, driveConfig.interceptRightAccel,
      driveConfig.gainRightDecel, driveConfig.interceptRightDecel);
  drive.setCrawlPulse(driveConfig.crawlPulse);
}

void installWheelController(Drive& drive, const Config::WheelControllerBootConfig& config) {
  Drive::ControlGains gains;
  gains.kp = config.kp;
  gains.ki = config.ki;
  gains.iMax = config.iMax;
  gains.kaff = config.kaff;
  gains.pidMax = config.pidMax;
  drive.setControlGains(gains);

  Drive::AdaptationBounds bounds;
  bounds.vMin = config.vMin;
  bounds.biasMax = config.biasMax;
  bounds.tauAdapt = config.tauAdapt;
  bounds.aSteady = config.aSteady;
  bounds.deficitThreshold = config.deficitThreshold;
  bounds.deficitWindow = config.deficitWindow;
  drive.setAdaptationBounds(bounds);
}

// configurePlanner -- see boot_calibration.h's own doc comment.
void configurePlanner(Motion::Planner& planner, const Config::Robot& config) {
  planner.applyShaperLimits(config.planner.a_max, config.planner.a_decel,
                            config.planner.alpha_max, config.planner.alpha_decel,
                            config.planner.jerk_max, config.planner.yaw_jerk_max);
}

// configureMotor -- see boot_calibration.h's own doc comment.
bool configureMotor(Devices::Motor& motor, const Config::Robot& config, bool isLeft) {
  const bool atRest =
      std::fabs(motor.velocity()) < kConfigureRestVelocity && motor.appliedDuty() == 0.0f;
  if (!atRest) return false;

  motor.applyTravelCalib(isLeft ? config.motors.travel_calib_left
                                : config.motors.travel_calib_right);
  return true;
}

// configureOtos -- see boot_calibration.h's own doc comment.
void configureOtos(Devices::Otos& otos, const Config::Robot& config) {
  otos.setLinearScalar(config.otos.linear_scale);
  otos.setAngularScalar(config.otos.angular_scale);
  otos.setOffset(config.otos.offset_x, config.otos.offset_y, config.otos.offset_yaw);
}

}  // namespace App
