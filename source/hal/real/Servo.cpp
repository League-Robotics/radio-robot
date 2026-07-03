#include "Servo.h"

Servo::Servo(MicroBitPin& pin, uint16_t maxDegrees)
    : _pin(pin)
    , _maxDegrees(maxDegrees)
{
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

void Servo::commandAngle(uint16_t angle, uint8_t mode)
{
    // 039-003: body moved VERBATIM from the former Servo::setAngle(uint8_t).
    // A hobby servo has no Nezha motion mode, so `mode` is ignored.
    (void)mode;
    uint16_t clamped = (angle > _maxDegrees) ? _maxDegrees : angle;
    _pin.setServoValue((int)clamped);
    _currentAngle = clamped;
}
