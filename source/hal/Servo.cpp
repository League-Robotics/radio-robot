#include "Servo.h"

Servo::Servo(MicroBitPin& pin, uint16_t maxDegrees)
    : _pin(pin)
    , _maxDegrees(maxDegrees)
{
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

void Servo::setAngle(uint8_t degrees)
{
    uint8_t clamped = (degrees > _maxDegrees) ? (uint8_t)_maxDegrees : degrees;
    _pin.setServoValue((int)clamped);
    _currentAngle = (int16_t)clamped;
}
