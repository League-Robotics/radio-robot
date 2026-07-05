/**
 * body_kinematics.cpp — implementation of stateless differential-drive maps.
 *
 * See body_kinematics.h for full API documentation and unit conventions.
 *
 * References:
 *   docs/kinematics-model.md §1.3 (inverse/forward maps)
 *   docs/kinematics-model.md §1.7 (saturation scaling, curvature preservation)
 */
#include "body_kinematics.h"
#include "messages/common.h"
#include <math.h>

namespace BodyKinematics {

void inverse(float v, float omega, float b, float& vL_out, float& vR_out) {
    float half_b = b * 0.5f;
    vL_out = v - omega * half_b;
    vR_out = v + omega * half_b;
}

void forward(float vL, float vR, float b, float& v_out, float& omega_out) {
    v_out     = (vR + vL) * 0.5f;
    omega_out = (vR - vL) / b;
}

void saturate(float vL, float vR,
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
// Array-form overloads (046-002) — differential adapter for IKinematics.
// wheels[2] = {vL, vR}. v_y is always 0 for differential.
// ---------------------------------------------------------------------------

void inverse(msg::BodyTwist3 t, float b, float wheels[2]) {
    // v_y is ignored (differential cannot strafe).
    inverse(t.v_x, t.omega, b, wheels[0], wheels[1]);
}

void forward(const float wheels[2], float b, msg::BodyTwist3& t_out) {
    float v, omega;
    forward(wheels[0], wheels[1], b, v, omega);
    t_out.v_x   = v;
    t_out.v_y   = 0.0f;
    t_out.omega = omega;
}

void saturate(float wheels[2], float vWheelMax, float steerHeadroom, float out[2]) {
    saturate(wheels[0], wheels[1], vWheelMax, steerHeadroom, out[0], out[1]);
}

} // namespace BodyKinematics
