#pragma once
#include <stdint.h>
#include "state/PoseEstimate.h"     // PoseEstimate
#include "kinematics/IKinematics.h" // Kinematics::kWheelCount
#include "types/ValueSet.h"         // ValueSet

// ---------------------------------------------------------------------------
// ActualState — all measured/estimated robot state (047-001).
//
// Three side-by-side pose estimates let consumers compare encoder dead-reckoning
// vs. raw optical vs. EKF-fused beliefs for fusion validation.
//
// Array sizes use Kinematics::kWheelCount (2 differential, 4 mecanum) — no
// #ifdef inside the struct body.
//
// Backward-compat scalar fields (encLMm, encRMm, velLMms, velRMms, poseX,
// poseY, poseHrad, fusedV, fusedOmega, otosX, otosY, otosH, fusedVy) are
// retained alongside the new arrays / PoseEstimate members so that all existing
// consumers compile unchanged during the Phase A–C migration.
//
// HardwareState is a using-alias for ActualState so that existing function
// signatures (void foo(HardwareState& s)) continue to compile and bind
// correctly to state.actual.
// ---------------------------------------------------------------------------
struct ActualState {
    // ----- Three pose estimates (new in 047-001) -----
    // encoder : dead-reckoned from wheel deltas only; fusion never writes here.
    // optical : raw OTOS pose + twist as reported (pre-correction).
    // fused   : EKF output — authoritative belief.
    PoseEstimate encoder = {};
    PoseEstimate optical = {};
    PoseEstimate fused   = {};

    // ----- Per-wheel arrays (sized by drivetrain, #ifdef-free) -----
    // [0]=FR, [1]=FL, [2]=BR, [3]=BL (mecanum); [0]=R, [1]=L (differential).
    float    encMm [kWheelCount] = {};  // cumulative, mm
    float    velMms[kWheelCount] = {};  // per-wheel velocity, mm/s

    // ----- Backward-compat scalar encoder/velocity fields -----
    // Kept so existing code (Drive.cpp, Odometry.cpp, telemetry, etc.) compiles
    // unchanged.  Phase C migrates consumers to encMm[]/velMms[] arrays.
    float    encLMm  = 0.0f;   // FL (front-left) cumulative distance, mm  ([1])
    float    encRMm  = 0.0f;   // FR (front-right) cumulative distance, mm ([0])
    float    velLMms = 0.0f;   // FL velocity, mm/s  ([1])
    float    velRMms = 0.0f;   // FR velocity, mm/s  ([0])

    // ----- Encoder freshness envelope -----
    ValueSet enc = {};

    // ----- Backward-compat flat pose fields (written by Odometry → fused) -----
    float    poseX    = 0.0f;
    float    poseY    = 0.0f;
    float    poseHrad = 0.0f;
    ValueSet pose     = {};

    // ----- EKF fused body velocity (backward-compat) -----
    float    fusedV     = 0.0f;   // body-frame linear speed, mm/s
    float    fusedOmega = 0.0f;   // yaw rate, rad/s
    float    fusedVy    = 0.0f;   // lateral velocity, mm/s (mecanum; 0 on differential)

    // ----- OTOS raw readings (backward-compat) -----
    float    otosX = 0.0f;
    float    otosY = 0.0f;
    float    otosH = 0.0f;
    ValueSet otos  = {};

    // ----- OTOS acceleration (passthrough for telemetry) -----
    float    otosAccelX = 0.0f;
    float    otosAccelY = 0.0f;

    // ----- Line sensor (4-channel) -----
    uint16_t line[4]  = {};
    ValueSet lineVS   = {};

    // ----- Color sensor (RGBC) -----
    uint16_t colorR = 0;
    uint16_t colorG = 0;
    uint16_t colorB = 0;
    uint16_t colorC = 0;
    ValueSet colorVS = {};

    // ----- General-purpose I/O ports -----
    bool     digitalIn[4] = {};
    int16_t  analogIn[4]  = {};
    ValueSet portsVS      = {};
};

// HardwareState is a backward-compat alias for ActualState.  All existing
// function signatures that accept HardwareState& or const HardwareState&
// continue to compile and bind to state.actual without any call-site changes.
using HardwareState = ActualState;
