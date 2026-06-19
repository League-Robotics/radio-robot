#include "Servo.h"

Servo::Servo(MicroBitPin& pin, uint16_t maxDegrees)
    : _pin(pin)
    , _maxDegrees(maxDegrees)
{
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

void Servo::setAngleDeg(uint16_t deg, uint8_t mode)
{
    // 039-003: body moved VERBATIM from the former Servo::setAngle(uint8_t).
    // A hobby servo has no Nezha motion mode, so `mode` is ignored.
    (void)mode;
    uint16_t clamped = (deg > _maxDegrees) ? _maxDegrees : deg;
    _pin.setServoValue((int)clamped);
    _currentAngle = clamped;
}
