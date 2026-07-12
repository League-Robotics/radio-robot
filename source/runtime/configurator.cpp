// configurator.cpp -- see configurator.h for the class-level contract.
//
// The four fold*() free functions below are this ticket's concretization of
// "field-masked, not full-replace" (source/runtime/commands.h's ConfigDelta
// comment): each copies ONLY the fields whose bit is set in delta.mask from
// the delta's matching value member onto the caller-supplied persistent
// config, leaving every other field of that persistent config untouched.
// Each returns whether the fold actually changed anything (a bitwise
// before/after compare) -- Configurator::applyOne() uses this to decide
// whether to call the target's own configure() at all (AC-2: "...calls
// configure() on that target only when the fold actually changes
// anything").
#include "runtime/configurator.h"

#include <cstring>

namespace Rt {

namespace {

bool foldDrivetrain(msg::DrivetrainConfig& cfg, const ConfigDelta& delta) {
  const msg::DrivetrainConfig before = cfg;
  const msg::DrivetrainConfig& v = delta.drivetrain;
  const uint64_t m = delta.mask;

  if (m & bitOf(DrivetrainConfigField::kFwdSignL)) cfg.fwd_sign_l = v.fwd_sign_l;
  if (m & bitOf(DrivetrainConfigField::kFwdSignR)) cfg.fwd_sign_r = v.fwd_sign_r;
  if (m & bitOf(DrivetrainConfigField::kTravelCalibL)) cfg.travel_calib_l = v.travel_calib_l;
  if (m & bitOf(DrivetrainConfigField::kTravelCalibR)) cfg.travel_calib_r = v.travel_calib_r;
  if (m & bitOf(DrivetrainConfigField::kTrackwidth)) cfg.trackwidth = v.trackwidth;
  if (m & bitOf(DrivetrainConfigField::kHalfTrack)) cfg.half_track = v.half_track;
  if (m & bitOf(DrivetrainConfigField::kHalfWheelbase)) cfg.half_wheelbase = v.half_wheelbase;
  if (m & bitOf(DrivetrainConfigField::kTravelCalibWheel)) {
    std::memcpy(cfg.travel_calib_wheel_, v.travel_calib_wheel_, sizeof(cfg.travel_calib_wheel_));
    cfg.travel_calib_wheel_count = v.travel_calib_wheel_count;
  }
  if (m & bitOf(DrivetrainConfigField::kFwdSignWheel)) {
    std::memcpy(cfg.fwd_sign_wheel_, v.fwd_sign_wheel_, sizeof(cfg.fwd_sign_wheel_));
    cfg.fwd_sign_wheel_count = v.fwd_sign_wheel_count;
  }
  if (m & bitOf(DrivetrainConfigField::kVWheelMax)) cfg.v_wheel_max = v.v_wheel_max;
  if (m & bitOf(DrivetrainConfigField::kSteerHeadroom)) cfg.steer_headroom = v.steer_headroom;
  if (m & bitOf(DrivetrainConfigField::kVelGains)) cfg.vel_gains = v.vel_gains;
  if (m & bitOf(DrivetrainConfigField::kVelFiltAlpha)) cfg.vel_filt_alpha = v.vel_filt_alpha;
  if (m & bitOf(DrivetrainConfigField::kSyncGain)) cfg.sync_gain = v.sync_gain;
  if (m & bitOf(DrivetrainConfigField::kMinWheel)) cfg.min_wheel = v.min_wheel;
  if (m & bitOf(DrivetrainConfigField::kAlphaPos)) cfg.alpha_pos = v.alpha_pos;
  if (m & bitOf(DrivetrainConfigField::kAlphaYaw)) cfg.alpha_yaw = v.alpha_yaw;
  if (m & bitOf(DrivetrainConfigField::kOtosGate)) cfg.otos_gate = v.otos_gate;
  if (m & bitOf(DrivetrainConfigField::kOtosLinearScale)) cfg.otos_linear_scale = v.otos_linear_scale;
  if (m & bitOf(DrivetrainConfigField::kOtosAngularScale)) cfg.otos_angular_scale = v.otos_angular_scale;
  if (m & bitOf(DrivetrainConfigField::kRotationGainPos)) cfg.rotation_gain_pos = v.rotation_gain_pos;
  if (m & bitOf(DrivetrainConfigField::kRotationGainNeg)) cfg.rotation_gain_neg = v.rotation_gain_neg;
  if (m & bitOf(DrivetrainConfigField::kRotationOffset)) cfg.rotation_offset = v.rotation_offset;
  if (m & bitOf(DrivetrainConfigField::kRotationOffsetNeg)) cfg.rotation_offset_neg = v.rotation_offset_neg;
  if (m & bitOf(DrivetrainConfigField::kRotationalSlip)) cfg.rotational_slip = v.rotational_slip;
  if (m & bitOf(DrivetrainConfigField::kOdomOffX)) cfg.odom_off_x = v.odom_off_x;
  if (m & bitOf(DrivetrainConfigField::kOdomOffY)) cfg.odom_off_y = v.odom_off_y;
  if (m & bitOf(DrivetrainConfigField::kOdomYaw)) cfg.odom_yaw = v.odom_yaw;
  if (m & bitOf(DrivetrainConfigField::kOdomUpsideDown)) cfg.odom_upside_down = v.odom_upside_down;
  if (m & bitOf(DrivetrainConfigField::kEkfQXy)) cfg.ekf_q_xy = v.ekf_q_xy;
  if (m & bitOf(DrivetrainConfigField::kEkfQTheta)) cfg.ekf_q_theta = v.ekf_q_theta;
  if (m & bitOf(DrivetrainConfigField::kEkfROtosXy)) cfg.ekf_r_otos_xy = v.ekf_r_otos_xy;
  if (m & bitOf(DrivetrainConfigField::kEkfROtosTheta)) cfg.ekf_r_otos_theta = v.ekf_r_otos_theta;
  if (m & bitOf(DrivetrainConfigField::kEkfQV)) cfg.ekf_q_v = v.ekf_q_v;
  if (m & bitOf(DrivetrainConfigField::kEkfQOmega)) cfg.ekf_q_omega = v.ekf_q_omega;
  if (m & bitOf(DrivetrainConfigField::kEkfROtosV)) cfg.ekf_r_otos_v = v.ekf_r_otos_v;
  if (m & bitOf(DrivetrainConfigField::kEkfREncV)) cfg.ekf_r_enc_v = v.ekf_r_enc_v;
  if (m & bitOf(DrivetrainConfigField::kLagOtos)) cfg.lag_otos = v.lag_otos;
  if (m & bitOf(DrivetrainConfigField::kDrivetrainType)) cfg.drivetrain_type = v.drivetrain_type;
  if (m & bitOf(DrivetrainConfigField::kLeftPort)) cfg.left_port = v.left_port;
  if (m & bitOf(DrivetrainConfigField::kRightPort)) cfg.right_port = v.right_port;

  return std::memcmp(&before, &cfg, sizeof(before)) != 0;
}

bool foldMotor(msg::MotorConfig& cfg, const ConfigDelta& delta) {
  const msg::MotorConfig before = cfg;
  const msg::MotorConfig& v = delta.motor;
  const uint64_t m = delta.mask;

  if (m & bitOf(MotorConfigField::kTravelCalib)) cfg.travel_calib = v.travel_calib;
  if (m & bitOf(MotorConfigField::kFwdSign)) cfg.fwd_sign = v.fwd_sign;
  if (m & bitOf(MotorConfigField::kVelGainsKp)) cfg.vel_gains.kp = v.vel_gains.kp;
  if (m & bitOf(MotorConfigField::kVelGainsKi)) cfg.vel_gains.ki = v.vel_gains.ki;
  if (m & bitOf(MotorConfigField::kVelGainsKff)) cfg.vel_gains.kff = v.vel_gains.kff;
  if (m & bitOf(MotorConfigField::kVelGainsIMax)) cfg.vel_gains.i_max = v.vel_gains.i_max;
  if (m & bitOf(MotorConfigField::kVelGainsKaw)) cfg.vel_gains.kaw = v.vel_gains.kaw;
  if (m & bitOf(MotorConfigField::kVelFiltAlpha)) cfg.vel_filt_alpha = v.vel_filt_alpha;
  if (m & bitOf(MotorConfigField::kMinDuty)) cfg.min_duty = v.min_duty;
  if (m & bitOf(MotorConfigField::kSlewRate)) cfg.slew_rate = v.slew_rate;
  if (m & bitOf(MotorConfigField::kReversalDwell)) cfg.reversal_dwell = v.reversal_dwell;
  if (m & bitOf(MotorConfigField::kOutputDeadband)) cfg.output_deadband = v.output_deadband;
  if (m & bitOf(MotorConfigField::kPolled)) cfg.polled = v.polled;

  return std::memcmp(&before, &cfg, sizeof(before)) != 0;
}

bool foldPlanner(msg::PlannerConfig& cfg, const ConfigDelta& delta) {
  const msg::PlannerConfig before = cfg;
  const msg::PlannerConfig& v = delta.planner;
  const uint64_t m = delta.mask;

  if (m & bitOf(PlannerConfigField::kAMax)) cfg.a_max = v.a_max;
  if (m & bitOf(PlannerConfigField::kADecel)) cfg.a_decel = v.a_decel;
  if (m & bitOf(PlannerConfigField::kVBodyMax)) cfg.v_body_max = v.v_body_max;
  if (m & bitOf(PlannerConfigField::kYawRateMax)) cfg.yaw_rate_max = v.yaw_rate_max;
  if (m & bitOf(PlannerConfigField::kYawAccMax)) cfg.yaw_acc_max = v.yaw_acc_max;
  if (m & bitOf(PlannerConfigField::kJMax)) cfg.j_max = v.j_max;
  if (m & bitOf(PlannerConfigField::kYawJerkMax)) cfg.yaw_jerk_max = v.yaw_jerk_max;
  if (m & bitOf(PlannerConfigField::kArriveTol)) cfg.arrive_tol = v.arrive_tol;
  if (m & bitOf(PlannerConfigField::kTurnInPlaceGate)) cfg.turn_in_place_gate = v.turn_in_place_gate;
  if (m & bitOf(PlannerConfigField::kMinSpeed)) cfg.min_speed = v.min_speed;

  return std::memcmp(&before, &cfg, sizeof(before)) != 0;
}

bool foldOdometer(msg::OdometerConfig& cfg, const ConfigDelta& delta) {
  const msg::OdometerConfig before = cfg;
  const msg::OdometerConfig& v = delta.odometer;
  const uint64_t m = delta.mask;

  if (m & bitOf(OdometerConfigField::kLinearScalar)) cfg.linear_scalar = v.linear_scalar;
  if (m & bitOf(OdometerConfigField::kAngularScalar)) cfg.angular_scalar = v.angular_scalar;

  return std::memcmp(&before, &cfg, sizeof(before)) != 0;
}

}  // namespace

Configurator::Configurator(Subsystems::Drivetrain& drivetrain, Subsystems::PoseEstimator& poseEstimator,
                           Subsystems::Hardware& hardware,
                           const msg::DrivetrainConfig& bootDrivetrainConfig,
                           const msg::PlannerConfig& bootPlannerConfig)
    : drivetrain_(drivetrain),
      poseEstimator_(poseEstimator),
      hardware_(hardware),
      drivetrainConfig_(bootDrivetrainConfig),
      plannerConfig_(bootPlannerConfig),
      odometerConfig_() {
  for (uint32_t i = 0; i < Subsystems::Hardware::kMotorCount; ++i) {
    motorConfig_[i] = hardware_.motorConfig(i);
  }
}

void Configurator::applyOne(Blackboard& bb) {
  if (bb.configIn.empty()) return;
  const ConfigDelta delta = bb.configIn.take();

  switch (delta.target) {
    case ConfigDelta::kDrivetrain: {
      // Drivetrain-scoped config re-propagates to PoseEstimator too --
      // both share msg::DrivetrainConfig (ticket 087-004) and both must
      // stay configured from the SAME value: any drivetrain-scoped delta
      // re-propagates the FULL candidate msg::DrivetrainConfig to BOTH
      // Drivetrain::configure() and PoseEstimator::configure() (established
      // by the now-deleted text SET handler, source/commands/
      // config_commands.{h,cpp}, removed 097-007).
      if (foldDrivetrain(drivetrainConfig_, delta)) {
        drivetrain_.configure(drivetrainConfig_);
        poseEstimator_.configure(drivetrainConfig_);
      }
      bb.drivetrainConfig = drivetrainConfig_;
      break;
    }
    case ConfigDelta::kMotor: {
      // Hardware::motor()'s own [0, kMotorCount) out-of-range convention
      // (clamp to the last index) -- mirrored here since delta.port is
      // caller-supplied (already a 0-based index -- see commands.h's own
      // doc comment on ConfigDelta::port) and this is the one place it gets
      // used as an array index into motorConfig_[].
      uint32_t idx = delta.port;
      if (idx >= Subsystems::Hardware::kMotorCount) {
        idx = Subsystems::Hardware::kMotorCount - 1;
      }
      if (foldMotor(motorConfig_[idx], delta)) {
        // Hardware has no top-level configure() (ticket 087-004's own
        // Implementation Notes flagged this gap) -- apply through the
        // per-motor Hal::Motor faceplate instead, exactly as SET/DEV M CFG
        // already do today.
        hardware_.motor(idx).configure(motorConfig_[idx]);
        // 091-002: `polled` is a Hardware-level (poll-schedule) fact, not a
        // per-motor Hal::Motor one -- Hal::Motor::configure() above has no
        // concept of it. Route it through the ONE door that does
        // (Hardware::setPolled(), a no-op default for SimHardware) --
        // ONLY when this delta actually touched the bit, mirroring every
        // other field's own mask-gated application above.
        if (delta.mask & bitOf(MotorConfigField::kPolled)) {
          hardware_.setMotorPolled(idx, motorConfig_[idx].polled);
        }
      }
      bb.motorConfig[idx] = motorConfig_[idx];
      break;
    }
    case ConfigDelta::kPlanner: {
      // 094-002: Subsystems::Planner was relocated out of source/ -- there
      // is no live subsystem left to call configure() on. Still fold +
      // publish so bb.plannerConfig stays a truthful record of what was
      // asked for (mirrors kOdometer's own "always fold+publish" shape).
      foldPlanner(plannerConfig_, delta);
      bb.plannerConfig = plannerConfig_;
      break;
    }
    case ConfigDelta::kOdometer: {
      if (foldOdometer(odometerConfig_, delta)) {
        // (090-003) hardware_.odometer() is NEVER null (Hal::NullOdometer
        // default, subsystems/hardware.h) -- configure() is safe to call
        // unconditionally: a real device applies it, a NullOdometer
        // discards it inertly. bb.odometerConfig is still folded+published
        // below regardless, so it stays a truthful record of what was asked
        // for either way.
        hardware_.odometer()->configure(odometerConfig_);
      }
      bb.odometerConfig = odometerConfig_;
      break;
    }
  }
}

void Configurator::publish(Blackboard& bb) {
  bb.drivetrainConfig = drivetrainConfig_;
  for (uint32_t i = 0; i < Subsystems::Hardware::kMotorCount; ++i) {
    bb.motorConfig[i] = motorConfig_[i];
  }
  bb.plannerConfig = plannerConfig_;
  bb.odometerConfig = odometerConfig_;
}

bool Configurator::pending(const Blackboard& bb) const { return !bb.configIn.empty(); }

}  // namespace Rt
