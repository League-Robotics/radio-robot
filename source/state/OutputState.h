#pragma once
#include <stdint.h>
#include "kinematics/IKinematics.h"  // kWheelCount (global constexpr)

// ---------------------------------------------------------------------------
// OutputState — actuator output commands (047-001).
//
// PWM and dirty-flag outputs produced by the control loop.  No #ifdef inside
// the struct body: pwm[] uses Kinematics::kWheelCount.
//
// Also retains the backward-compat scalar fields (tgtLMms, tgtRMms, pwmL,
// pwmR, digitalDirty, analogDirty) from MotorCommands so that all existing
// consumers compile unchanged during the Phase A–C migration.
//
// MotorCommands is a using-alias for OutputState so that existing function
// signatures (void foo(const MotorCommands& c)) continue to compile and
// bind to state.outputs without any call-site changes.
// ---------------------------------------------------------------------------
struct OutputState {
    // ----- Per-wheel PWM outputs (new uniform array, #ifdef-free) -----
    // [0]=FR, [1]=FL, [2]=BR, [3]=BL (mecanum); [0]=R, [1]=L (differential).
    int16_t  pwm[kWheelCount] = {};

    // ----- Backward-compat scalar L/R fields (from MotorCommands) -----
    float    tgtLMms = 0.0f;   // FL target speed (semantic "left"), mm/s
    float    tgtRMms = 0.0f;   // FR target speed (semantic "right"), mm/s
    int16_t  pwmL    = 0;      // FL raw PWM output
    int16_t  pwmR    = 0;      // FR raw PWM output

    // ----- 4-wheel target arrays (mecanum code uses these alongside scalars) -----
    // [0]=FR, [1]=FL, [2]=BR, [3]=BL.
    float    tgtMms[kWheelCount] = {};  // all-wheel targets

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
