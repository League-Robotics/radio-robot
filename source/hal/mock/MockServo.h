#pragma once
#include <stdint.h>
#include "io/capability/IPositionMotor.h"

/**
 * MockServo — host-compilable IPositionMotor implementation for unit tests.
 *
 * Records the last angle passed to setAngleDeg(). currentAngleDeg() returns the
 * stored value. Defaults to 0 before any set call.
 *
 * 039-003: canonicalized from IServo (now an alias) to IPositionMotor.  Host
 * behaviour is unchanged — the stored angle and accessor semantics match the
 * former MockServo::setAngle/currentAngle.
 */
class MockServo : public IPositionMotor {
public:
    // IPositionMotor interface -----------------------------------------------
    void     setAngleDeg(uint16_t deg, uint8_t mode) override;
    uint16_t currentAngleDeg() const override;

    // Convenience (OQ-3): forwards to setAngleDeg(deg, 0).
    void setAngle(uint8_t degrees) { setAngleDeg(degrees, 0); }

private:
    uint16_t _angle = 0;
};
