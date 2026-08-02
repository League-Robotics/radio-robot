// boot_calibration.cpp -- see boot_calibration.h for why this is a
// separate TU from boot_wiring.cpp.
#include "app/boot_calibration.h"

namespace App {

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
  out.vMax = src.vMax;
  out.aMax = src.aMax;
  out.aDecel = src.aDecel;
  out.omegaMax = src.omegaMax;
  out.alphaMax = src.alphaMax;
  out.alphaDecel = src.alphaDecel;
  out.jerkMax = src.jerkMax;
  out.yawJerkMax = src.yawJerkMax;

  out.controlPeriod = src.controlPeriod;
  out.actuationDelay = src.actuationDelay;

  out.requireSettle = src.requireSettle;
  out.settleRestVelocity = src.settleRestVelocity;
  out.settleRestOmega = src.settleRestOmega;
  out.settleWindow = src.settleWindow;
  out.settleEpsilonLinear = src.settleEpsilonLinear;
  out.settleEpsilonAngular = src.settleEpsilonAngular;
  out.headingHoldGain = src.headingHoldGain;

  out.velKff = src.velKff;
  out.velKp = src.velKp;
  out.velKi = src.velKi;
  out.velIMax = src.velIMax;
  out.velKaff = src.velKaff;
  out.velIAccelGate = src.velIAccelGate;
  out.dutyFloor = src.dutyFloor;

  out.trimKp = src.trimKp;
  out.trimKi = src.trimKi;
  out.trimIMax = src.trimIMax;
  out.trimKaff = src.trimKaff;
  out.trimMax = src.trimMax;
  out.decelPlanFraction = src.decelPlanFraction;

  // trackWidth/velocityFilterWeight are the two PlannerLimits fields NOT
  // sourced from Config::PlannerBootConfig -- see that struct's own doc
  // comment (config/boot_config.h). trackWidth is the caller's own
  // (scrub-corrected, by default effectiveTrackWidth()'s) value;
  // velocityFilterWeight mirrors the robot JSON's vel_filt_alpha EMA
  // weight, with the same >0.05 sanity floor main.cpp always applied.
  out.trackWidth = trackWidth;
  out.velocityFilterWeight = drivetrainConfig.vel_filt_alpha > 0.05f
                                 ? drivetrainConfig.vel_filt_alpha
                                 : 1.0f;
  return out;
}

void installShaperLimits(Motion::Planner& planner, const Motion::PlannerLimits& limits) {
  planner.applyShaperLimits(limits.aMax, limits.aDecel, limits.alphaMax, limits.alphaDecel,
                            limits.jerkMax, limits.yawJerkMax);
}

void installRotationCalibration(RobotLoop& robotLoop,
                                const msg::DrivetrainConfig& drivetrainConfig) {
  constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;
  robotLoop.setRotationCalibration(
      drivetrainConfig.rotation_gain_pos, drivetrainConfig.rotation_offset * kDegToRad,
      drivetrainConfig.rotation_gain_neg, drivetrainConfig.rotation_offset_neg * kDegToRad);
}

void installDriveCalibration(Drive& drive, const Config::DriveBootConfig& driveConfig) {
  // MEASURED, NOT CONFIGURED (stakeholder, 2026-07-31): one baked constant
  // for both wheels, deliberately ignoring driveConfig.dutyPerSpeedLeft/
  // Right -- see Drive::kDutyPerSpeed's own doc comment (drive.h) for the
  // measurement and rationale.
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

}  // namespace App
