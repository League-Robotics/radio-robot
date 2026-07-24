#include "app/drive.h"

namespace App {

Drive::Drive(Devices::Motor& left, Devices::Motor& right, float trackWidth)
    : left_(left), right_(right), trackWidth_(trackWidth) {}

void Drive::setWheels(float v_left, float v_right) {
  vLeft_ = v_left;
  vRight_ = v_right;
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
