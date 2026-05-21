#include "MotorController.h"

MotorController::MotorController(NezhaV2& motor, const CalibParams& cal)
    : _motor(motor)
    , _cal(cal)
    , _targetL(0.0f)
    , _targetR(0.0f)
    , _integralL(0.0f)
    , _integralR(0.0f)
    , _prevEncL(0)
    , _prevEncR(0)
    , _actualVelL(0.0f)
    , _actualVelR(0.0f)
{
    gains.kFF     = 0.15f;
    gains.kP      = 0.05f;
    gains.kI      = 0.20f;
    gains.iClamp  = 60.0f;
    gains.kRatio  = 0.01f;
}

void MotorController::setTarget(float leftMms, float rightMms)
{
    _targetL = leftMms;
    _targetR = rightMms;
}

void MotorController::stop()
{
    _targetL = 0.0f;
    _targetR = 0.0f;
    resetIntegrators();
    _motor.setPwm(0, 0);
}

void MotorController::resetIntegrators()
{
    _integralL = 0.0f;
    _integralR = 0.0f;
}

void MotorController::tick(float dt_s)
{
    if (dt_s <= 0.0f) return;

    int32_t encL = _motor.readEncoder(true,  _cal);
    int32_t encR = _motor.readEncoder(false, _cal);

    _actualVelL = (encL - _prevEncL) / dt_s;
    _actualVelR = (encR - _prevEncR) / dt_s;
    _prevEncL   = encL;
    _prevEncR   = encR;

    // Left wheel PI + FF
    float errorL    = _targetL - _actualVelL;
    _integralL     += errorL * dt_s;
    _integralL      = clamp(_integralL, -gains.iClamp, gains.iClamp);
    float outputL   = gains.kFF * _targetL + gains.kP * errorL + gains.kI * _integralL;
    float pwmL      = clamp(outputL, -100.0f, 100.0f);

    // Right wheel PI + FF
    float errorR    = _targetR - _actualVelR;
    _integralR     += errorR * dt_s;
    _integralR      = clamp(_integralR, -gains.iClamp, gains.iClamp);
    float outputR   = gains.kFF * _targetR + gains.kP * errorR + gains.kI * _integralR;
    float pwmR      = clamp(outputR, -100.0f, 100.0f);

    // Ratio cross-coupling (straight-line assist)
    if (_targetL != 0.0f && _targetR != 0.0f &&
        ((_targetL > 0.0f) == (_targetR > 0.0f)))
    {
        float ratio       = _targetR / _targetL;
        float actualRatio = (_actualVelL != 0.0f) ? _actualVelR / _actualVelL : ratio;
        float ratioErr    = ratio - actualRatio;
        float correction  = gains.kRatio * ratioErr;
        pwmL -= correction * 0.5f;
        pwmR += correction * 0.5f;
        pwmL  = clamp(pwmL, -100.0f, 100.0f);
        pwmR  = clamp(pwmR, -100.0f, 100.0f);
    }

    _motor.setPwm(static_cast<int8_t>(pwmL), static_cast<int8_t>(pwmR));
}

void MotorController::getActualVelocity(float& leftMms, float& rightMms) const
{
    leftMms  = _actualVelL;
    rightMms = _actualVelR;
}

void MotorController::getEncoderPositions(int32_t& leftMm, int32_t& rightMm) const
{
    leftMm  = _motor.readEncoder(true,  _cal);
    rightMm = _motor.readEncoder(false, _cal);
}

void MotorController::resetEncoderAccumulators()
{
    _motor.resetEncoders();
    _prevEncL = 0;
    _prevEncR = 0;
}

float MotorController::clamp(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
