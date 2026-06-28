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
// HardwareState is a using-alias for ActualState so that existing function
// signatures (void foo(HardwareState& s)) continue to compile and bind
// correctly to state.actual.
// ---------------------------------------------------------------------------
struct ActualState {
    // ----- Three pose estimates (047-001) -----
    // encoder : dead-reckoned from wheel deltas only; EKF never writes here.
    // optical : raw OTOS pose + twist as reported (pre-correction).
    // fused   : EKF output — authoritative belief.
    PoseEstimate encoder = {};
    PoseEstimate optical = {};
    PoseEstimate fused   = {};

    // ----- Per-wheel arrays (sized by drivetrain, #ifdef-free) -----
    // [0]=FR, [1]=FL, [2]=BR, [3]=BL (mecanum); [0]=R, [1]=L (differential).
    float    encMm [kWheelCount] = {};  // cumulative, mm
    float    velMms[kWheelCount] = {};  // per-wheel velocity, mm/s

    // ----- Encoder freshness envelope -----
    ValueSet enc = {};

    // ----- OTOS freshness envelope -----
    // otos.valid: true when a fresh OTOS reading exists this tick.
    // otos.lastUpdMs: timestamp of the last successful OTOS read.
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
