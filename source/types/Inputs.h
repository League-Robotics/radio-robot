#pragma once
#include <stdint.h>
#include "Config.h"
#include "Protocol.h"
#include "MotionEventSink.h"

// ---------------------------------------------------------------------------
// ValueSet — freshness / validity envelope for a sensor group.
//
// lagMs      : expected sensor latency in ms; initialised from RobotConfig.
// lastUpdMs  : system time (ms) of the most recent valid reading.
// valid      : true once at least one reading has been received.
// ---------------------------------------------------------------------------
struct ValueSet {
    uint32_t lagMs;
    uint32_t lastUpdMs;
    bool     valid;
};

// ---------------------------------------------------------------------------
// MotorCommands — actuator outputs produced by the control loop.
//
// 046-005 design note: the mecanum build KEEPS the existing scalar fields
// (tgtLMms, tgtRMms, pwmL, pwmR) so that ALL shared code in Drive.cpp,
// MotionController.cpp, SimHardware.cpp etc. compiles in both builds with
// ZERO call-site changes.  The mecanum build ADDS 4-element arrays alongside
// (tgtMms[4], pwm[4]) for mecanum-specific code (BVC, MotorController).
//
// MotorController in the mecanum build syncs BOTH representations when
// setTarget() is called: it writes tgtMms[0..3] (rear) AND keeps
// tgtLMms/tgtRMms in sync so shared code that reads them (Drive.cpp driving
// detection) continues to work correctly.
//
// Reference-member approach was rejected: reference members cannot appear in
// structs using aggregate `= {}` zero-initialisation (C++ requires that every
// reference member be explicitly bound in a user-provided constructor, and
// `RobotStateContainer s{}` would not compile).
// ---------------------------------------------------------------------------
#ifdef ROBOT_DRIVETRAIN_MECANUM
struct MotorCommands {
    // Scalar L/R fields — IDENTICAL layout to the differential struct so that
    // shared code (Drive.cpp, MotionController.cpp, SimHardware.cpp, etc.)
    // compiles without any call-site changes in the mecanum build.
    float    tgtLMms = 0.0f;     // FL target speed (semantic "left"), mm/s
    float    tgtRMms = 0.0f;     // FR target speed (semantic "right"), mm/s
    int16_t  pwmL    = 0;        // FL raw PWM output
    int16_t  pwmR    = 0;        // FR raw PWM output
    bool     digitalOut[4]   = {};
    int16_t  analogOut[4]    = {};
    bool     digitalDirty[4] = {};
    bool     analogDirty[4]  = {};

    // 4-wheel arrays (mecanum-only code uses these).
    // [0]=FR, [1]=FL, [2]=BR, [3]=BL.
    // FR  maps to tgtRMms (semantic "right front"),
    // FL  maps to tgtLMms (semantic "left front").
    // MotorController::setTarget(float*,int) writes all 4 AND syncs tgtLMms/tgtRMms.
    float    tgtMms[4] = {};   // all-wheel targets (synced with tgtLMms/tgtRMms)
    int16_t  pwm[4]    = {};   // all-wheel PWM outputs (synced with pwmL/pwmR)
};
#else
struct MotorCommands {
    float    tgtLMms;           // left-wheel target speed, mm/s
    float    tgtRMms;           // right-wheel target speed, mm/s
    int16_t  pwmL;              // raw PWM output, left motor
    int16_t  pwmR;              // raw PWM output, right motor
    bool     digitalOut[4];     // digital output channels 0–3
    int16_t  analogOut[4];      // analogue output channels 0–3
    bool     digitalDirty[4];   // channel has unsent update
    bool     analogDirty[4];    // channel has unsent update
};
#endif  // ROBOT_DRIVETRAIN_MECANUM

// ---------------------------------------------------------------------------
// HardwareState — all sensor readings (latest values + freshness envelopes).
//
// 046-005 design note: same strategy as MotorCommands — the mecanum build
// KEEPS the existing scalar encLMm/encRMm/velLMms/velRMms fields so shared
// code (Drive.cpp, MotionController.cpp, Odometry.cpp, etc.) compiles in
// both builds unchanged.  The mecanum build ADDS encMm[4]/velMms[4] alongside
// for mecanum-specific code and adds fusedVy (lateral body velocity).
//
// MotorController in the mecanum build keeps the scalar fields and the arrays
// in sync: encMm[0]/[1] alias FR/FL, velMms[0]/[1] alias FR/FL.  BR/BL
// (indices 2/3) have no corresponding scalar; they are mecanum-only.
// ---------------------------------------------------------------------------
#ifdef ROBOT_DRIVETRAIN_MECANUM
struct HardwareState {
    // Scalar L/R fields — IDENTICAL layout to the differential struct.
    // Shared code (Drive.cpp, MotionController.cpp, Odometry.cpp, etc.)
    // reads/writes these without any change in the mecanum build.
    float    encLMm = 0.0f;   // FL (front-left) cumulative distance, mm
    float    encRMm = 0.0f;   // FR (front-right) cumulative distance, mm
    ValueSet enc;              // freshness for encoder readings

    float    velLMms = 0.0f;  // FL velocity, mm/s
    float    velRMms = 0.0f;  // FR velocity, mm/s

    // 4-wheel arrays (mecanum-only code uses these; kept in sync by MotorController).
    // [0]=FR, [1]=FL, [2]=BR, [3]=BL.
    float    encMm[4]  = {};  // per-wheel cumulative distances (encMm[0]=encRMm, [1]=encLMm)
    float    velMms[4] = {};  // per-wheel velocities          (velMms[0]=velRMms, [1]=velLMms)

    // Lateral body velocity (mecanum-only; written by MotorController/BVC).
    float    fusedVy = 0.0f;  // lateral velocity from forward kinematics, mm/s

    // Dead-reckoning pose (updated each encoder tick)
    float    poseX    = 0.0f;
    float    poseY    = 0.0f;
    float    poseHrad = 0.0f;
    ValueSet pose;

    // EKF fused velocity
    float    fusedV     = 0.0f;
    float    fusedOmega = 0.0f;

    // OTOS acceleration
    float    otosAccelX = 0.0f;
    float    otosAccelY = 0.0f;

    // OTOS optical odometry sensor
    float    otosX = 0.0f;
    float    otosY = 0.0f;
    float    otosH = 0.0f;
    ValueSet otos;

    // Line sensor (4-channel)
    uint16_t line[4] = {};
    ValueSet lineVS;

    // Color sensor (RGBC)
    uint16_t colorR = 0;
    uint16_t colorG = 0;
    uint16_t colorB = 0;
    uint16_t colorC = 0;
    ValueSet colorVS;

    // General-purpose I/O ports
    bool     digitalIn[4] = {};
    int16_t  analogIn[4]  = {};
    ValueSet portsVS;
};
#else
struct HardwareState {
    // Encoder odometry
    float    encLMm;    // left-wheel cumulative distance, mm
    float    encRMm;    // right-wheel cumulative distance, mm
    ValueSet enc;       // freshness for encoder readings

    // Derived wheel velocities
    float    velLMms;   // left-wheel velocity, mm/s
    float    velRMms;   // right-wheel velocity, mm/s

    // Dead-reckoning pose (updated each encoder tick)
    float    poseX;     // robot X position, mm
    float    poseY;     // robot Y position, mm
    float    poseHrad;  // robot heading, radians
    ValueSet pose;      // freshness for pose (updated with enc)

    // EKF fused velocity (Sprint 023 — written by Odometry::predict and correctEKF)
    float    fusedV;       // body-frame linear speed, mm/s (from 5-state EKF)
    float    fusedOmega;   // yaw rate, rad/s (from 5-state EKF)

    // OTOS acceleration (passthrough; written by Robot::otosCorrect — Sprint 023)
    float    otosAccelX;   // forward acceleration, mm/s^2
    float    otosAccelY;   // lateral acceleration, mm/s^2

    // OTOS optical odometry sensor
    float    otosX;     // OTOS X reading, mm
    float    otosY;     // OTOS Y reading, mm
    float    otosH;     // OTOS heading, radians
    ValueSet otos;      // freshness for OTOS readings

    // Line sensor (4-channel)
    uint16_t line[4];   // raw line sensor values, channels 0–3
    ValueSet lineVS;    // freshness for line readings

    // Color sensor (RGBC)
    uint16_t colorR;    // red channel
    uint16_t colorG;    // green channel
    uint16_t colorB;    // blue channel
    uint16_t colorC;    // clear/ambient channel
    ValueSet colorVS;   // freshness for color readings

    // General-purpose I/O ports
    bool     digitalIn[4];  // digital input channels 0–3
    int16_t  analogIn[4];   // analogue input channels 0–3
    ValueSet portsVS;       // freshness for port readings
};
#endif  // ROBOT_DRIVETRAIN_MECANUM

// ---------------------------------------------------------------------------
// TargetState — the current drive command from the radio / command processor.
// ---------------------------------------------------------------------------
struct TargetState {
    DriveMode mode;             // active drive mode (IDLE, STREAMING, …)
    float     targetXWorld;     // go-to target X in world frame, mm
    float     targetYWorld;     // go-to target Y in world frame, mm
    float     targetSpeedMms;   // requested travel speed, mm/s
    float     distanceTargetMm; // remaining distance for DISTANCE mode, mm
    uint32_t  deadlineMs;       // wall-clock deadline (used by MotionCommand TIME stop), ms
    ReplyFn   replyFn;          // callback to send drive-complete reply
    void*     replyCtx;         // opaque context for replyFn
    char      corrId[16];       // correlation ID for the pending command
    MotionEventSink sink;       // narrow event sink for async EVT completions (sprint 026-002)
};

// ---------------------------------------------------------------------------
// RobotStateContainer — single authoritative state blob passed through the
// cooperative main loop (replaces per-subsystem private caches).
// ---------------------------------------------------------------------------
struct RobotStateContainer {
    MotorCommands commands;
    HardwareState inputs;
    TargetState   target;
};

// ---------------------------------------------------------------------------
// defaultInputs — zero-initialise the container, then seed each ValueSet's
// lagMs from the corresponding RobotConfig lag field.
// ---------------------------------------------------------------------------
inline RobotStateContainer defaultInputs(const RobotConfig& cfg) {
    RobotStateContainer s{};
    s.inputs.otos.lagMs   = cfg.lagOtosMs;
    s.inputs.lineVS.lagMs = cfg.lagLineMs;
    s.inputs.colorVS.lagMs = cfg.lagColorMs;
    s.inputs.portsVS.lagMs = cfg.lagPortsMs;
    // enc / pose lag: encoder readings are always synchronous in the control
    // loop, so lagMs is left at 0 (zero-initialised).
    return s;
}
