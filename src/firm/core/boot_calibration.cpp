// boot_calibration.cpp -- see boot_calibration.h for why this is a
// separate TU from boot_wiring.cpp.
#include "core/boot_calibration.h"

namespace Core {

Hal::MotorConfig toDeviceMotorConfig(const msg::MotorConfig& src) {
  Hal::MotorConfig cfg;
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
  // writeThrottle -- config-derived (this file's own header, TRAP note):
  // the kernel is the ONLY writer of duty now, and its own cycle period is
  // kKernelCyclePeriod, never RobotLoop::kCycle (that constant no longer
  // has anything to do with motor writes at all).
  cfg.writeThrottle = static_cast<float>(kKernelCyclePeriod - 5) * 1000.0f;  // [us]
  return cfg;
}

Control::DifferentialDrive::Config buildDriveKernelConfig(const Config::Robot& config) {
  Control::DifferentialDrive::Config cfg;

  const float travelCalibL = config.motors.travel_calib_left;    // [mm/deg]
  const float travelCalibR = config.motors.travel_calib_right;   // [mm/deg]
  const float travelCalibMean = 0.5f * (travelCalibL + travelCalibR);

  // mm/s -> counts/s: 1 count = 0.1 deg, so counts = mm * 10 / travel_calib
  // (travel_calib is [mm/deg]). Same factor for mm/s^2 -> counts/s^2 and
  // mm -> counts (a pure unit-domain change, not a rate).
  const float mmsToCounts = (travelCalibMean > 0.0f) ? (10.0f / travelCalibMean) : 0.0f;

  // fullDutyVelocity: the ONE per-wheel conversion. duty_per_speed_left/
  // right are already a single population-scale plant gain (configurator.
  // cpp's own dutyPerSpeed doc comment -- deliberately never re-fit
  // per-wheel); the per-wheel split here comes only from travel_calib_
  // left/right's own asymmetry, then averaged into the kernel's one
  // fullDutyVelocity slot (the kernel has no per-wheel plant-gain field).
  const float fullDutyVelL =
      (config.drive.duty_per_speed_left > 0.0f && travelCalibL > 0.0f)
          ? 10.0f / (config.drive.duty_per_speed_left * travelCalibL)
          : 0.0f;
  const float fullDutyVelR =
      (config.drive.duty_per_speed_right > 0.0f && travelCalibR > 0.0f)
          ? 10.0f / (config.drive.duty_per_speed_right * travelCalibR)
          : 0.0f;
  cfg.fullDutyVelocity = 0.5f * (fullDutyVelL + fullDutyVelR);  // [counts/s]

  // Stage A wheel correction: gains are dimensionless ratios (copy);
  // intercepts are speeds (mm/s -> counts/s).
  cfg.wheelGain[0][0] = config.drive.wheel_gain_left_accel;
  cfg.wheelIntercept[0][0] = config.drive.wheel_intercept_left_accel * mmsToCounts;
  cfg.wheelGain[0][1] = config.drive.wheel_gain_left_decel;
  cfg.wheelIntercept[0][1] = config.drive.wheel_intercept_left_decel * mmsToCounts;
  cfg.wheelGain[1][0] = config.drive.wheel_gain_right_accel;
  cfg.wheelIntercept[1][0] = config.drive.wheel_intercept_right_accel * mmsToCounts;
  cfg.wheelGain[1][1] = config.drive.wheel_gain_right_decel;
  cfg.wheelIntercept[1][1] = config.drive.wheel_intercept_right_decel * mmsToCounts;

  cfg.crawlPulse = config.drive.crawl_pulse;  // [-1,1] dimensionless, copy

  // Stage B/C gains + bounds.
  cfg.kp = config.wheelControl.pid_kp;                          // [1] copy
  cfg.ki = config.wheelControl.pid_ki;                          // [1/s] copy
  cfg.iMax = config.wheelControl.pid_i_max * mmsToCounts;       // [counts/s]
  cfg.kaff = config.wheelControl.pid_kaff;                      // [s] copy
  cfg.pidMax = config.wheelControl.pid_max * mmsToCounts;       // [counts/s]
  cfg.vMin = config.wheelControl.v_min * mmsToCounts;           // [counts/s]
  cfg.posErrMax = config.wheelControl.pos_err_max * mmsToCounts;  // [counts]
  cfg.biasMax = config.wheelControl.bias_max * mmsToCounts;     // [counts/s]
  cfg.tauAdapt = config.wheelControl.tau_adapt;                 // [s] copy
  cfg.aSteady = config.wheelControl.a_steady * mmsToCounts;     // [counts/s^2]
  cfg.deficitThreshold = config.wheelControl.deficit_threshold * mmsToCounts;  // [counts/s]
  cfg.deficitWindow = config.wheelControl.deficit_window;       // [ms] copy
  cfg.stallSpeed = config.wheelControl.stall_speed * mmsToCounts;    // [counts/s]
  cfg.stallDemand = config.wheelControl.stall_demand * mmsToCounts;  // [counts/s]
  cfg.stallWindow = config.wheelControl.stall_window;            // [ms] copy

  // No robot-JSON key yet for either -- see this function's own header
  // doc comment.
  cfg.twistHoldGain = 0.0f;

  // maxDuty: the kernel's Config default is FAIL-CLOSED now (0 = every
  // mode refused), so the bake must grant authority EXPLICITLY. It can no
  // longer ride a permissive class default -- that is the point of the
  // change: an unconfigured kernel refuses rather than driving at full
  // authority.
  //
  // CONFIGURATION-DISCIPLINE GAP, stated rather than hidden: this is a C++
  // literal, not a value from the robot JSON, so it breaks invariant 1
  // ("every value the robot uses comes from the file") the same way
  // twistHoldGain above does. Not a regression -- the value was previously
  // an even less visible class-member default -- but a debt:
  // `drive.max_duty` needs a real key in robot_config.proto, the four
  // robot JSONs and the schema, with this line then reading it. Tracked in
  // the issue's conformance plan.
  cfg.maxDuty = 100.0f;  // [%] full authority rail

  cfg.cyclePeriod = kKernelCyclePeriod;

  return cfg;
}

// configureOtos -- see boot_calibration.h's own doc comment. Ported
// verbatim; unaffected by the exploratory-kernel rewrite.
void configureOtos(Hal::Otos& otos, const Config::Robot& config) {
  otos.setLinearScalar(static_cast<float>(Hardware::scaleToRegister(config.otos.linear_scale)));
  otos.setAngularScalar(static_cast<float>(Hardware::scaleToRegister(config.otos.angular_scale)));
  otos.setOffset(config.otos.offset_x, config.otos.offset_y, config.otos.offset_yaw);
}

}  // namespace Core
