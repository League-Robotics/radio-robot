#include "app/drive.h"

namespace App {

Drive::Drive(Devices::Motor& left, Devices::Motor& right, float trackWidth)
    : left_(left), right_(right), trackWidth_(trackWidth) {}

// 125-003 INTERIM: still a velocity mm/s target under the duty-shaped name
// -- see drive.h's own file header for why and what replaces this.
void Drive::setDuty(float left, float right) {
  vLeft_ = left;
  vRight_ = right;
}

void Drive::stop() {
  vLeft_ = 0.0f;
  vRight_ = 0.0f;
  rawMode_ = false;
  rawDutyLeft_ = 0.0f;
  rawDutyRight_ = 0.0f;
}

void Drive::setRawDuty(float dutyLeft, float dutyRight) {
  rawMode_ = true;
  rawDutyLeft_ = dutyLeft;
  rawDutyRight_ = dutyRight;
}

// 125-003 INTERIM closed loop -- see drive.h's own file header for the
// full rationale (Decision 1/2, sprint.md). velDeadband is passed as
// 0.0f: the pre-125-003 wire field it fed (msg::MotorConfig.min_duty) was
// never populated by gen_boot_config.py in practice (see the pre-move
// wheel_velocity_pid.cpp comment history) -- 0.0f matches production
// reality exactly, not a new simplification.
//
// Measurement-lead compensation (fixes 125-003's disclosed defect): the
// measured velocity is one loop cycle staler here than it was inside
// NezhaMotor::tick(), which measurably cost ~5 deg on managed 90-deg
// turns (the original ticket's own bench note). During a commanded ramp
// the stale reading lags the true state by commandedAccel * dt, so the
// PID chased a phantom error. Predicting the measurement forward by that
// one known cycle -- the same physics-derived compensation the motion
// planner's duty stage uses (filter-lag comp, planner.cpp) -- removes the
// phantom without a tuning knob. Exactly zero at steady state.
void Drive::tick() {
  if (rawMode_) {
    // Planner-era path (2026-07-26 integration): the planner already
    // closed the velocity loop and produced per-wheel duty; stage it
    // verbatim (armor/shaping live in the leaf). The interim velocity
    // PID below is bypassed entirely while raw mode is latched.
    //
    // Quiet at zero: while both the staged and last-written pairs are
    // exactly zero there is nothing to say to the hardware -- and saying
    // it anyway flips the motors into Mode::Active from the very first
    // idle boot cycle, closing NezhaMotor::reconfigure()'s unconditional
    // Mode::None window and making every boot-time config push race the
    // at-rest gate against sensing transients (measured: a calibration
    // push right after connect refused on a 5.3 mm/s decode blip).
    const bool quiet = rawDutyLeft_ == 0.0f && rawDutyRight_ == 0.0f &&
                       rawWrittenLeft_ == 0.0f && rawWrittenRight_ == 0.0f;
    if (!quiet) {
      left_.setDuty(rawDutyLeft_);
      right_.setDuty(rawDutyRight_);
      rawWrittenLeft_ = rawDutyLeft_;
      rawWrittenRight_ = rawDutyRight_;
    }
    return;
  }
  // The lead uses the PREVIOUS interval's commanded accel -- the change
  // the plant has been executing during the stale window and the sample
  // therefore missed. This tick's own new change is deliberately NOT led:
  // a fresh step is real error the sample legitimately hasn't seen, and
  // kp must act on it (leading it too made the PID blind to steps --
  // measured as unmanaged presets landing ~20 deg short in the sim
  // acceptance suite).
  const float accelL =
      (vLeftPrevious_ - vLeftPrevious2_) / kNominalPeriod;  // [mm/s^2]
  const float accelR =
      (vRightPrevious_ - vRightPrevious2_) / kNominalPeriod;  // [mm/s^2]
  vLeftPrevious2_ = vLeftPrevious_;
  vRightPrevious2_ = vRightPrevious_;
  vLeftPrevious_ = vLeft_;
  vRightPrevious_ = vRight_;
  const float measuredL = left_.velocity() + accelL * kNominalPeriod;
  const float measuredR = right_.velocity() + accelR * kNominalPeriod;
  float dutyL = pidL_.compute(vLeft_, measuredL, kNominalPeriod, gainsL_, /*velDeadband=*/0.0f);
  float dutyR = pidR_.compute(vRight_, measuredR, kNominalPeriod, gainsR_, /*velDeadband=*/0.0f);
  left_.setDuty(dutyL);
  right_.setDuty(dutyR);
}

}  // namespace App
