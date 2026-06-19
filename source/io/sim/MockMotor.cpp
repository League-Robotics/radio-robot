#include "MockMotor.h"
#include "types/Config.h"
#include <random>

// Gaussian noise helper — returns a sample from N(0, sigma), or 0 if sigma <= 0.
static float gaussianNoise(std::mt19937& rng, float sigma) {
    if (sigma <= 0.0f) return 0.0f;
    std::normal_distribution<float> dist(0.0f, sigma);
    return dist(rng);
}

void MockMotor::setSpeed(int8_t pct) {
    _cmdSpeed = pct;
}

void MockMotor::requestEncoder() {
    // No-op: encoder is always ready in the mock.
}

int32_t MockMotor::collectEncoder() const {
    return static_cast<int32_t>(_encoderMm);
}

float MockMotor::readEncoderMmF(const RobotConfig& /*cfg*/) const {
    return _encoderMm;
}

float MockMotor::readEncoderMmFAtomic(const RobotConfig& /*cfg*/) const {
    return _encoderMm;
}

float MockMotor::readEncoderMmFSettle(const RobotConfig& /*cfg*/) const {
    return _encoderMm;
}

void MockMotor::resetEncoder() {
    _encoderMm = 0.0f;
    _cmdSpeed  = 0;
    // Realign the tick() cache (039-002) — mirrors Motor::resetEncoder so the
    // accessor + outlier-filter baseline (state.inputs.encLMm/R) stay in lockstep.
    _lastPositionMm   = 0.0f;
    _lastVelocityMmps = 0.0f;
    _hasLastTick      = false;
}

void MockMotor::integrate(uint32_t dt_ms) {
    float vel     = (_cmdSpeed / 100.0f) * kNominalMaxMms * _offsetFactor;
    _trueVelMms   = vel;
    float slip    = _slipStraight + _slipTurnExtra * _turnRate;
    float noisy   = vel * (1.0f - slip) + gaussianNoise(_rng, _encoderNoiseSigma);
    _encoderMm   += noisy * (static_cast<float>(dt_ms) / 1000.0f);
}

void MockMotor::tick(uint32_t now_ms) {
    // Sensor tick (039-002): promote the integrated encoder position into the
    // accessor cache.  COPY only — no re-integration (integrate(dt_ms) driven by
    // MockHAL::tick(now,cmds) is the single integration site, OQ-2).  A simple
    // position-difference velocity is cached for velocityMmps(); like Motor, this
    // value is NOT consumed by the PID, so it does not affect the golden-TLM frame.
    float pos = _encoderMm;
    if (_hasLastTick) {
        float elapsed_s = static_cast<float>(now_ms - _lastTickMs) / 1000.0f;
        if (elapsed_s > 0.0f) {
            _lastVelocityMmps = (pos - _lastPositionMm) / elapsed_s;
        }
    } else {
        _hasLastTick = true;
    }
    _lastPositionMm = pos;
    _lastTickMs     = now_ms;
}
