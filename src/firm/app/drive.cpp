#include "app/drive.h"

#include "kinematics/body_kinematics.h"

namespace App {

Drive::Drive(Devices::Motor& left, Devices::Motor& right, float trackWidth)
    : left_(left), right_(right), trackWidth_(trackWidth) {}

void Drive::setTwist(float v_x, float v_y, float omega) {
  (void)v_y;  // accepted, ignored -- see this method's own header comment
  v_x_ = v_x;
  omega_ = omega;
  targetKind_ = TargetKind::kTwist;
}

void Drive::setWheels(float v_left, float v_right) {
  vLeft_ = v_left;
  vRight_ = v_right;
  targetKind_ = TargetKind::kWheels;
}

void Drive::stop() {
  v_x_ = 0.0f;
  omega_ = 0.0f;
  vLeft_ = 0.0f;
  vRight_ = 0.0f;
}

void Drive::tick() {
  float vL = 0.0f;
  float vR = 0.0f;

  if (targetKind_ == TargetKind::kWheels) {
    vL = vLeft_;
    vR = vRight_;
  } else {
    BodyKinematics::inverse(v_x_, omega_, trackWidth_, vL, vR);
  }

  left_.setVelocity(vL);
  right_.setVelocity(vR);
}

}  // namespace App
