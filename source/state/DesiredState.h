#pragma once
#include <stdint.h>
#include "hal/capability/Pose2D.h"    // BodyTwist3
#include "kinematics/IKinematics.h"  // Kinematics::kWheelCount
#include "types/Config.h"            // DriveMode
#include "types/Protocol.h"          // ReplyFn
#include "control/MotionEventSink.h" // MotionEventSink

// ---------------------------------------------------------------------------
// DesiredState — commanded/planned robot state (047-001).
//
// Absorbs all TargetState fields and adds BVC-published body twist,
// per-wheel speed targets, and digital/analogue port outputs.
//
// No #ifdef inside the struct body: BodyTwist3.vy is always present (written
// as 0.0f on differential builds) and wheelSpeeds uses Kinematics::kWheelCount.
//
// TargetState is a using-alias for DesiredState so that existing function
// signatures (void foo(TargetState& t)) continue to compile without edits.
// ---------------------------------------------------------------------------
struct DesiredState {
    // ----- BVC profiled live setpoints (new in 047-001) -----
    BodyTwist3 bodyTwist    = {0.0f, 0.0f, 0.0f};  // profiled live setpoint
    BodyTwist3 bodyTwistRaw = {0.0f, 0.0f, 0.0f};  // commanded before clamp/profile

    // ----- Per-wheel speed targets (new in 047-001) -----
    float wheelSpeeds[kWheelCount] = {};   // [mm/s]

    // ----- Drive mode and go-to target (from TargetState) -----
    DriveMode mode          = DriveMode::IDLE;
    float     targetXWorld  = 0.0f;
    float     targetYWorld  = 0.0f;
    float     targetSpeed   = 0.0f;   // [mm/s]
    float     distanceTarget = 0.0f;  // [mm]
    uint32_t  deadline      = 0;      // [ms]

    // ----- Port outputs (from TargetState / MotorCommands) -----
    bool     digitalOut[4]  = {};
    int16_t  analogOut[4]   = {};

    // ----- Reply sink / correlation (from TargetState) -----
    ReplyFn  replyFn  = nullptr;
    void*    replyCtx = nullptr;
    char     corrId[16] = {};
    MotionEventSink sink = {};
};

// TargetState is a backward-compat alias for DesiredState.  All existing
// function signatures that accept TargetState& continue to compile and
// bind to state.desired without any call-site changes.
using TargetState = DesiredState;
