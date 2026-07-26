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
}

// 125-003 INTERIM closed loop -- see drive.h's own file header for the full
// rationale (Decision 1/2, sprint.md) and its known one-cycle feedback-
// staleness degradation. velDeadband is passed as 0.0f: the pre-125-003
// wire field it fed (msg::MotorConfig.min_duty) was never populated by
// gen_boot_config.py in practice (see the pre-move wheel_velocity_pid.cpp
// comment history) -- 0.0f matches production reality exactly, not a new
// simplification.
void Drive::tick() {
  float dutyL = pidL_.compute(vLeft_, left_.velocity(), kNominalPeriod, gainsL_, /*velDeadband=*/0.0f);
  float dutyR = pidR_.compute(vRight_, right_.velocity(), kNominalPeriod, gainsR_, /*velDeadband=*/0.0f);
  left_.setDuty(dutyL);
  right_.setDuty(dutyR);
}

}  // namespace App
