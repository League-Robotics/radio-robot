/**
 * differential_kinematics.cpp -- the two-wheel maps.
 *
 * Bodies are byte-for-byte the former BodyKinematics free functions
 * (src/motion/body_kinematics.cpp); see differential_kinematics.h for the
 * full API documentation and unit conventions.
 */
#include "kinematics/differential_kinematics.h"

#include <math.h>


namespace Kinematics {

void DifferentialKinematics::inverse(float v, float omega, float b,
                                     float& vL_out, float& vR_out) {
    float half_b = b * 0.5f;
    vL_out = v - omega * half_b;
    vR_out = v + omega * half_b;
}

void DifferentialKinematics::forward(float vL, float vR, float b,
                                     float& v_out, float& omega_out) {
    v_out     = (vR + vL) * 0.5f;
    omega_out = (vR - vL) / b;
}

void DifferentialKinematics::saturate(float vL, float vR,
                                      float vWheelMax, float steerHeadroom,
                                      float& vL_out, float& vR_out) {
    float ceiling = vWheelMax - steerHeadroom;
    float absL = fabsf(vL);
    float absR = fabsf(vR);
    float maxAbs = (absL > absR) ? absL : absR;

    if (maxAbs > ceiling) {
        float s = ceiling / maxAbs;
        vL_out = s * vL;
        vR_out = s * vR;
    } else {
        vL_out = vL;
        vR_out = vR;
    }
}

// ---------------------------------------------------------------------------
// Kinematics::Model overrides -- the statics above, with this instance's own
// track width.
// ---------------------------------------------------------------------------

void DifferentialKinematics::inverse(const Twist& twist,
                                     float wheels[]) const {
    inverse(twist.v_x, twist.omega, trackWidth_, wheels[0], wheels[1]);
}

void DifferentialKinematics::forward(const float wheels[],
                                     Twist& twist) const {
    float v, omega;
    forward(wheels[0], wheels[1], trackWidth_, v, omega);
    twist.v_x   = v;
    twist.v_y   = 0.0f;
    twist.omega = omega;
}

}  // namespace Kinematics
