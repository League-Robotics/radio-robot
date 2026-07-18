#include "app/drive.h"

#include "kinematics/body_kinematics.h"

namespace App {

Drive::Drive(Devices::Motor& left, Devices::Motor& right, float trackWidth)
    : left_(left), right_(right), trackWidth_(trackWidth) {}

void Drive::setTwist(float v_x, float omega) {
  v_x_ = v_x;
  omega_ = omega;
}

void Drive::stop() {
  v_x_ = 0.0f;
  omega_ = 0.0f;
}

void Drive::tick() {
  float vL = 0.0f;
  float vR = 0.0f;
  BodyKinematics::inverse(v_x_, omega_, trackWidth_, vL, vR);
  left_.setVelocity(vL);
  right_.setVelocity(vR);
}

}  // namespace App
