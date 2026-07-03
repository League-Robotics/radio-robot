#pragma once
#include <stdint.h>

/**
 * IPositionMotor — position/angle-move capability (039-001, canonicalized 039-003).
 *
 * Phase A introduces this header as the canonical name for the
 * position/angle-control capability.  It folds two previously-separate notions:
 *   1. the former IServo (hobby-servo angle set), and
 *   2. the Motor on-chip position-move subset (Nezha 0x5D moveToAngle / 0x70
 *      timedMove) — reached on a Motor via IVelocityMotor::asPositionMotor().
 *
 * Canonical method taxonomy (Sprint 039 architecture §5):
 *   - commandAngle(uint16_t angle, uint8_t mode): command an absolute angle
 *     [deg].  `mode` carries the Nezha ServoMotionMode for the on-chip-motor
 *     implementation (1=shortest path, 2=CW, 3=CCW); a hobby servo (Servo impl)
 *     ignores `mode`.  OQ-3 resolution: mode byte 0 is the hobby-servo default;
 *     concrete impls expose a convenience `setAngle(uint8_t)` that forwards
 *     commandAngle(angle, 0).
 *   - currentAngle(): last commanded/clamped angle [deg].
 *
 * `source/hal/IServo.h` is a `using IServo = IPositionMotor;` alias shim so every
 * existing IServo consumer compiles unchanged during the Phase A transition; it
 * is deleted in Phase F.
 *
 * NOTE (behaviour preservation, 039-003): the method bodies that back this
 * interface are moved VERBATIM — no wire bytes and no numerics change.  The
 * Servo impl's clamp logic is identical to the former IServo::setAngle body; the
 * Motor on-chip path forwards to the unchanged moveToAngle() (0x5D) frame.
 */
class IPositionMotor {
public:
    virtual ~IPositionMotor() = default;

    // Command an absolute angle [deg].  `mode` is the Nezha ServoMotionMode
    // for on-chip motor implementations (1=shortest, 2=CW, 3=CCW); the hobby-servo
    // implementation ignores `mode` and simply clamps + drives the pin.
    virtual void commandAngle(uint16_t angle, uint8_t mode) = 0;

    // Last commanded/clamped angle [deg].  Defaults to 0 before any
    // commandAngle() call.
    virtual uint16_t currentAngle() const = 0;
};

// 044-004 (Phase F): the former `source/io/IServo.h` alias shim is deleted; its
// `using IServo = IPositionMotor;` alias is folded in here so every consumer that
// still names IServo (Servo, ServoController, Robot::gripper) compiles unchanged.
// Behaviour-preserving rename housekeeping — no wire bytes change.
using IServo = IPositionMotor;
