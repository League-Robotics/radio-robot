#pragma once
#include <stdint.h>

// ---------------------------------------------------------------------------
// Telemetry field bitmask constants (used in RobotConfig::tlmFields)
// ---------------------------------------------------------------------------
constexpr uint8_t TLM_FIELD_ENC   = (1u << 0);  // enc=l,r
constexpr uint8_t TLM_FIELD_POSE  = (1u << 1);  // pose=x,y,h
constexpr uint8_t TLM_FIELD_VEL   = (1u << 2);  // vel=vl,vr  (deferred Sprint 010)
constexpr uint8_t TLM_FIELD_LINE  = (1u << 3);  // line=4ch
constexpr uint8_t TLM_FIELD_COLOR = (1u << 4);  // color=4ch
constexpr uint8_t TLM_FIELD_ALL   = 0xFFu;      // all fields (default)

struct RobotConfig {
    // Motor forward-direction signs: +1 = CW is forward, -1 = CCW is forward.
    // fwdSignL: left wheel (M2), default +1.
    // fwdSignR: right wheel (M1), default -1 (motor mounted mirrored).
    int8_t fwdSignL;
    int8_t fwdSignR;

    // Encoder calibration (mm per degree of motor rotation).
    // Also used by Motor::readSpeed() for chip-native velocity conversion:
    //   mm/s = (raw / kUnitFactor) * mmPerDeg * sign
    // (kUnitFactor is a named constant in Motor.cpp; see readSpeed() comment.)
    float mmPerDegL;
    float mmPerDegR;

    // Feed-forward and motor scale factors
    float kFF;
    float kScaleLF;
    float kScaleLB;
    float kScaleRF;
    float kScaleRB;

    // Slower-wheel adjustment
    float kAdjThreshold;
    float kAdjGain;

    // Geometry
    float trackwidthMm;

    // Ratio PID gains
    float ratioPidKp;
    float ratioPidKi;
    float ratioPidKd;
    float ratioPidMax;

    // Go-to tolerances
    float turnThresholdMm;
    float doneTolMm;

    // Command scaling
    float   distScale;
    float   turnScale;

    // Timing and speed parameters
    int32_t minSpeedMms;
    int32_t tickMs;
    int32_t sTimeoutMs;

    // Telemetry streaming period in ms (0 = off). Set via STREAM command.
    int32_t tlmPeriodMs;

    // Telemetry field-subscription bitmask. Set via STREAM fields=...
    // Bit 0 = enc, Bit 1 = pose, Bit 2 = vel, Bit 3 = line, Bit 4 = color.
    // 0xFF = all fields (default).
    uint8_t tlmFields;

    // One-shot SNAP pending flag. Set by SNAP command; cleared after one TLM frame.
    bool tlmSnapPending;
};

inline RobotConfig defaultRobotConfig() {
    RobotConfig p{};
    p.fwdSignL        = +1;
    p.fwdSignR        = -1;
    p.mmPerDegL       = 0.487f;
    p.mmPerDegR       = 0.481f;
    p.kFF             = 0.15f;
    p.kScaleLF        = 1.0f;
    p.kScaleLB        = 1.0f;
    p.kScaleRF        = 1.0f;
    p.kScaleRB        = 1.0f;
    p.kAdjThreshold   = 0.5f;
    p.kAdjGain        = 0.05f;
    p.trackwidthMm    = 120.0f;
    p.ratioPidKp      = 300.0f;
    p.ratioPidKi      = 0.0f;
    p.ratioPidKd      = 0.0f;
    p.ratioPidMax     = 30.0f;
    p.turnThresholdMm = 50.0f;
    p.doneTolMm       = 5.0f;
    p.distScale       = 0.94f;
    p.turnScale       = 1.07f;
    p.minSpeedMms     = 50;
    p.tickMs          = 20;
    p.sTimeoutMs      = 200;
    p.tlmPeriodMs     = 0;
    p.tlmFields       = 0xFF;
    p.tlmSnapPending  = false;
    return p;
}

struct MotorGains {
    float kp;
    float ki;
    float kff;
};

enum class DriveMode : uint8_t {
    IDLE      = 0,
    STREAMING = 1,
    TIMED     = 2,
    DISTANCE  = 3,
    GO_TO     = 4
};
