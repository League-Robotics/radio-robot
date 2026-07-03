/**
 * MecanumKinematics.cpp — 4-wheel X-roller mecanum kinematic maps (046-002).
 *
 * See MecanumKinematics.h for full API, equations, and unit conventions.
 *
 * This file is pure math and has NO hardware, CODAL, or I2C dependencies.
 * It compiles cleanly in BOTH the host/sim build and the firmware build.
 * The firmware CMakeLists.txt selects which drivetrain sources to compile,
 * but the host/sim build always includes this file so that host unit tests
 * cover the mecanum math regardless of the active firmware drivetrain.
 */
#include "MecanumKinematics.h"
#include <math.h>

namespace MecanumKinematics {

// Wheel index constants (documentation aid; not used in expressions below).
// static const int FR = 0, FL = 1, BR = 2, BL = 3;

void inverse(BodyTwist3 t, const RobotGeometry& geom,
             const int8_t signs[4], float wheels[4]) {
    float k = geom.halfTrack + geom.halfWheelbase;
    float vx = t.vx_mmps;
    float vy = t.vy_mmps;
    float om = t.omega_rads;

    // Raw wheel speeds (X-roller equations):
    float raw_fr =  vx - vy - k * om;   // [0] FR
    float raw_fl =  vx + vy + k * om;   // [1] FL
    float raw_br =  vx + vy - k * om;   // [2] BR
    float raw_bl =  vx - vy + k * om;   // [3] BL

    wheels[0] = raw_fr * static_cast<float>(signs[0]);
    wheels[1] = raw_fl * static_cast<float>(signs[1]);
    wheels[2] = raw_br * static_cast<float>(signs[2]);
    wheels[3] = raw_bl * static_cast<float>(signs[3]);
}

void forward(const float wheels[4], const RobotGeometry& geom,
             const int8_t signs[4], BodyTwist3& t_out) {
    float k = geom.halfTrack + geom.halfWheelbase;

    // Undo sign encoding: since signs[i] == ±1, multiply == divide.
    float w0 = wheels[0] * static_cast<float>(signs[0]);  // FR
    float w1 = wheels[1] * static_cast<float>(signs[1]);  // FL
    float w2 = wheels[2] * static_cast<float>(signs[2]);  // BR
    float w3 = wheels[3] * static_cast<float>(signs[3]);  // BL

    t_out.vx_mmps    = ( w0 + w1 + w2 + w3) * 0.25f;
    t_out.vy_mmps    = (-w0 + w1 + w2 - w3) * 0.25f;
    t_out.omega_rads = (-w0 + w1 - w2 + w3) / (4.0f * k);
}

void saturate(float wheels[4], float vWheelMax, float out[4]) {
    float maxAbs = 0.0f;
    for (int i = 0; i < 4; ++i) {
        float a = fabsf(wheels[i]);
        if (a > maxAbs) maxAbs = a;
    }

    if (maxAbs > vWheelMax) {
        float s = vWheelMax / maxAbs;
        for (int i = 0; i < 4; ++i) {
            out[i] = s * wheels[i];
        }
    } else {
        for (int i = 0; i < 4; ++i) {
            out[i] = wheels[i];
        }
    }
}

} // namespace MecanumKinematics
