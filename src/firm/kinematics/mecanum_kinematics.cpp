// mecanum_kinematics.cpp -- the four-wheel holonomic maps.
// See mecanum_kinematics.h for the equations, the wheel-order convention,
// and why nothing constructs this yet.
#include "kinematics/mecanum_kinematics.h"


namespace Kinematics {

void MecanumKinematics::inverse(float v_x, float v_y, float omega,
                                float trackWidth, float wheelBase,
                                float wheels[4]) {
  const float k = 0.5f * (wheelBase + trackWidth);  // [mm] lx + ly
  const float yaw = k * omega;                      // [mm/s]

  wheels[0] = v_x - v_y - yaw;  // FL
  wheels[1] = v_x + v_y + yaw;  // FR
  wheels[2] = v_x + v_y - yaw;  // BL
  wheels[3] = v_x - v_y + yaw;  // BR
}

void MecanumKinematics::forward(const float wheels[4], float trackWidth,
                                float wheelBase, float& v_x_out,
                                float& v_y_out, float& omega_out) {
  const float k = 0.5f * (wheelBase + trackWidth);  // [mm] lx + ly

  v_x_out = (wheels[0] + wheels[1] + wheels[2] + wheels[3]) * 0.25f;
  v_y_out = (-wheels[0] + wheels[1] + wheels[2] - wheels[3]) * 0.25f;
  omega_out = (-wheels[0] + wheels[1] - wheels[2] + wheels[3]) * 0.25f / k;
}

void MecanumKinematics::inverse(const Twist& twist,
                                float wheels[]) const {
  inverse(twist.v_x, twist.v_y, twist.omega, trackWidth_, wheelBase_, wheels);
}

void MecanumKinematics::forward(const float wheels[],
                                Twist& twist) const {
  forward(wheels, trackWidth_, wheelBase_, twist.v_x, twist.v_y, twist.omega);
}

}  // namespace Kinematics
