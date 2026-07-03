#pragma once
#include "MicroBit.h"
#include "hal/capability/IPositionMotor.h"
#include <stdint.h>

/**
 * Servo — CODAL pin driver for a hobby servo.
 *
 * Wraps MicroBitPin::setServoValue() to provide a clamped 0..maxDegrees
 * interface. Supports both standard 180° servos and 360° continuous-rotation
 * servos via the configurable maxDegrees parameter.
 *
 * 039-003: Servo now implements the capability-typed IPositionMotor (the former
 * IServo, which is an alias shim for IPositionMotor).  The angle-set body is
 * moved VERBATIM — the clamp logic and the pin drive are unchanged.  A hobby
 * servo has no motion mode, so commandAngle() ignores the `mode` byte.
 */
class Servo : public IPositionMotor {
public:
    explicit Servo(MicroBitPin& pin, uint16_t maxDegrees = 180);

    // IPositionMotor interface ----------------------------------------------

    // Set servo angle [deg]. Clamps to [0, maxDegrees] before driving the pin.
    // Records the clamped value; retrieve it with currentAngle().
    // `mode` is ignored — a hobby servo has no Nezha ServoMotionMode.
    void commandAngle(uint16_t angle, uint8_t mode) override;

    // Return the last clamped angle [deg] passed to commandAngle(). Defaults
    // to 0 before any set call.
    uint16_t currentAngle() const override { return _currentAngle; }

    // Convenience (OQ-3): hobby-servo entry point — forwards to
    // commandAngle(angle, 0).  Mode byte 0 is the hobby-servo default.
    void setAngle(uint8_t degrees) { commandAngle(degrees, 0); }

private:
    MicroBitPin& _pin;
    uint16_t     _maxDegrees;
    uint16_t     _currentAngle = 0;
};
