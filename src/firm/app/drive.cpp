#include "app/drive.h"

#include "kinematics/body_kinematics.h"

namespace App {

Drive::Drive(Devices::Motor& left, Devices::Motor& right, float trackWidth)
    : left_(left), right_(right), trackWidth_(trackWidth) {}

void Drive::configure(const msg::PlannerConfig& config) {
  actuationLag_ = config.actuation_lag;
}

void Drive::setTwist(float v_x, float omega, float a_x, float alpha) {
  v_x_ = v_x;
  omega_ = omega;
  a_x_ = a_x;
  alpha_ = alpha;
}

void Drive::stop() {
  v_x_ = 0.0f;
  omega_ = 0.0f;
  a_x_ = 0.0f;
  alpha_ = 0.0f;
}

void Drive::tick() {
  float vL = 0.0f;
  float vR = 0.0f;
  BodyKinematics::inverse(v_x_, omega_, trackWidth_, vL, vR);

  // 112-002: model feedforward -- the SAME inverse() map, reused for
  // acceleration (kinematics is linear, so this is exact: aL = a_x -
  // alpha*b/2, aR = a_x + alpha*b/2). a_x_/alpha_ default to 0 (setTwist()'s
  // own defaulted parameters), so this is a no-op unless a caller supplies
  // them.
  float aL = 0.0f;
  float aR = 0.0f;
  BodyKinematics::inverse(a_x_, alpha_, trackWidth_, aL, aR);

  left_.setVelocity(vL + actuationLag_ * aL);
  right_.setVelocity(vR + actuationLag_ * aR);
}

}  // namespace App
