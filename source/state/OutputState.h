#pragma once
#include <stdint.h>
#include "kinematics/IKinematics.h"  // kWheelCount (global constexpr)

// ---------------------------------------------------------------------------
// OutputState — actuator output commands (047-001).
//
// PWM and dirty-flag outputs produced by the control loop.  No #ifdef inside
// the struct body: pwm[] and tgtMms[] use Kinematics::kWheelCount.
//
// MotorCommands is a using-alias for OutputState so that existing function
// signatures (void foo(const MotorCommands& c)) continue to compile and
// bind to state.outputs without any call-site changes.
// ---------------------------------------------------------------------------
struct OutputState {
    // ----- Per-wheel PWM outputs (#ifdef-free) -----
    // [0]=FR, [1]=FL, [2]=BR, [3]=BL (mecanum); [0]=R, [1]=L (differential).
    int16_t  pwm[kWheelCount] = {};

    // ----- Per-wheel target speed arrays -----
    // [0]=FR, [1]=FL, [2]=BR, [3]=BL.
    float    tgtMms[kWheelCount] = {};  // all-wheel speed targets, mm/s

    // ----- Port outputs and dirty flags -----
    bool     digitalOut[4]   = {};
    int16_t  analogOut[4]    = {};
    bool     digitalDirty[4] = {};
    bool     analogDirty[4]  = {};
};

// MotorCommands is a backward-compat alias for OutputState.  All existing
// function signatures that accept MotorCommands& or const MotorCommands&
// continue to compile and bind to state.outputs without any call-site changes.
using MotorCommands = OutputState;
