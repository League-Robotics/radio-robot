#pragma once
#include <stdint.h>
#include "hal/capability/IPositionMotor.h"

/**
 * SimServo — host-compilable IPositionMotor implementation for the SIM gripper.
 *
 * Records the last angle passed to commandAngle(). currentAngle() returns the
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
    void     commandAngle(uint16_t angle, uint8_t mode) override;
    uint16_t currentAngle() const override;

    // Convenience (OQ-3): forwards to commandAngle(angle, 0).
    void setAngle(uint8_t degrees) { commandAngle(degrees, 0); }

private:
    uint16_t _angle = 0;
};
