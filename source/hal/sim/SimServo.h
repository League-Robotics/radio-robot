#pragma once
#include <stdint.h>
#include "hal/capability/IPositionMotor.h"

/**
 * SimServo — host-compilable IPositionMotor implementation for the SIM gripper.
 *
 * Records the last angle passed to setAngleDeg(). currentAngleDeg() returns the
 * stored value. Defaults to 0 before any set call. No PhysicsWorld dependency —
 * the gripper has no chassis-physics coupling, so it is a pure position store.
 *
 * 040-004: renamed from MockServo (the last surviving Mock* in source/io/sim/)
 * with no behaviour change, so the SIM path is uniformly Sim*-named. The stored
 * angle and accessor semantics are unchanged from the former MockServo.
 */
class SimServo : public IPositionMotor {
public:
    // IPositionMotor interface -----------------------------------------------
    void     setAngleDeg(uint16_t deg, uint8_t mode) override;
    uint16_t currentAngleDeg() const override;

    // Convenience (OQ-3): forwards to setAngleDeg(deg, 0).
    void setAngle(uint8_t degrees) { setAngleDeg(degrees, 0); }

private:
    uint16_t _angle = 0;
};
