#pragma once
#include <stdint.h>

/**
 * IPositionMotor — position-move capability (039-001).
 *
 * Phase A introduces this header as the canonical name for the
 * position/angle-control capability.  During the transition (T1) the body is
 * the verbatim former IServo interface, and `source/hal/IServo.h` becomes a
 * `using IServo = IPositionMotor;` shim so every existing consumer
 * (Servo, MockServo, ServoController, Hardware::gripper) compiles unchanged.
 * The capability-typed method rename (setAngleDeg / currentAngleDeg) and the
 * fold of the Motor position-move subset land in T3 — bodies are not changed
 * here.
 */
class IPositionMotor {
public:
    virtual ~IPositionMotor() = default;

    // Set servo angle, clamped to the servo's configured maximum.
    virtual void setAngle(uint8_t degrees) = 0;

    // Return the last clamped angle passed to setAngle(). Defaults to 0 before
    // any setAngle() call.
    virtual int16_t currentAngle() const = 0;
};
