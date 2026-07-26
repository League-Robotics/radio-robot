#include "app/drive.h"

namespace App {

Drive::Drive(Devices::Motor& left, Devices::Motor& right, float trackWidth)
    : left_(left), right_(right), trackWidth_(trackWidth) {}

// 125-002 PLACEHOLDER: renamed from setWheels() (Motion::WheelSink's own
// duty-sink retool) -- still a straight pass-through stage, unclamped,
// exactly as before. See drive.h's own file header for why; ticket 007
// lands the real duty implementation (|duty|<=1/NaN->0 clamp, WheelObserver
// wiring).
void Drive::setDuty(float left, float right) {
  vLeft_ = left;
  vRight_ = right;
}

void Drive::stop() {
  vLeft_ = 0.0f;
  vRight_ = 0.0f;
}

void Drive::tick() {
  left_.setVelocity(vLeft_);
  right_.setVelocity(vRight_);
}

}  // namespace App
