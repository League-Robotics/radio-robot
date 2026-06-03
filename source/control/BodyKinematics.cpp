/**
 * BodyKinematics.cpp — implementation of stateless differential-drive maps.
 *
 * See BodyKinematics.h for full API documentation and unit conventions.
 *
 * References:
 *   docs/kinematics-model.md §1.3 (inverse/forward maps)
 *   docs/kinematics-model.md §1.7 (saturation scaling, curvature preservation)
 */
#include "BodyKinematics.h"
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

} // namespace BodyKinematics
