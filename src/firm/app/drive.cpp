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

// Apply the cycle's duty pair to the two leaves. Quiet at zero: while
// both the commanded and last-written pairs are exactly zero there is
// nothing to say to the hardware -- writing anyway would flip the motors
// out of Mode::None from the first idle boot cycle and make boot-time
// config pushes race the at-rest reconfigure gate.
void Drive::tick(float dutyLeft, float dutyRight) {
  const bool quiet = dutyLeft == 0.0f && dutyRight == 0.0f &&
                     writtenLeft_ == 0.0f && writtenRight_ == 0.0f;
  if (quiet) return;
  left_.setDuty(dutyLeft);
  right_.setDuty(dutyRight);
  writtenLeft_ = dutyLeft;
  writtenRight_ = dutyRight;
}

}  // namespace App
