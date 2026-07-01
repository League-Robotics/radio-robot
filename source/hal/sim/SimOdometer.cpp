#include "SimOdometer.h"
#include <cmath>

#ifdef HOST_BUILD
#include <random>

// Gaussian noise helper — bit-identical to the retired MockOtosSensor.cpp
// (same std::normal_distribution over the std::mt19937{43u} stream).
static float otosGaussian(std::mt19937& rng, float sigma) {
    if (sigma <= 0.0f) return 0.0f;
    std::normal_distribution<float> dist(0.0f, sigma);
    return dist(rng);
}
#endif

bool SimOdometer::readTransformed(Pose2D& poseOut, float /*headingRad*/) const {
    // Read failure or LIFT → INVALID; emit {0,0,0} and signal the same-tick skip.
    if (_readFailure || _lift) {
        poseOut = {0.0f, 0.0f, 0.0f};
        return false;
    }
    if (_useSimModel) {
        poseOut.x = _odomX;
        poseOut.y = _odomY;
        poseOut.h = _odomH;
    } else {
        poseOut.x = _injectedX;
        poseOut.y = _injectedY;
        poseOut.h = _injectedH;
    }
    return true;
}

bool SimOdometer::readVelocityTransformed(BodyTwist& velOut,
                                          float /*headingRad*/) const {
    if (_readFailure || _lift) {
        velOut = {0.0f, 0.0f};
        return false;
    }
    velOut = {_velV, _velOmega};
    return true;
}

BodyAccel SimOdometer::readAccelTransformed() const {
    return {_accAx, _accAy};
}

void SimOdometer::getPositionRaw(int16_t& x, int16_t& y, int16_t& h) const {
    x = _rawX;
    y = _rawY;
    h = _rawH;
}

void SimOdometer::setPositionRaw(int16_t x, int16_t y, int16_t h) {
    _rawX = x;
    _rawY = y;
    _rawH = h;
    // Re-reference the accumulator readTransformed() actually returns to the
    // EKF (source/robot/Robot.cpp otosCorrect() -> Odometry::correctEKF()).
    // The real OtosSensor::setPositionRaw() (source/hal/real/OtosSensor.cpp)
    // writes the chip's POSITION registers directly, and readTransformed()
    // scales those SAME registers by the chip's LSB resolution
    // (kPosMmPerLsb = 0.305 mm/LSB, kHdgRadPerLsb = 0.00549 deg/LSB in rad) on
    // every read. So on the real chip, writing the raw registers IS the
    // accumulator -- there's no separate host-side shadow to fall out of
    // sync. Mirror that here: convert the same raw ints with the same LSB
    // scale into the float accumulator, so `OZ` (setPositionRaw(0,0,0)) truly
    // zeroes the pose readTransformed() returns, matching the real OTOS.
    constexpr float kPosMmPerLsb  = 0.305f;
    constexpr float kHdgRadPerLsb = 0.00549f * (3.14159265f / 180.0f);
    _odomX = static_cast<float>(x) * kPosMmPerLsb;
    _odomY = static_cast<float>(y) * kPosMmPerLsb;
    _odomH = static_cast<float>(h) * kHdgRadPerLsb;
}

void SimOdometer::setInjectedPose(float x, float y, float h) {
    _injectedX = x;
    _injectedY = y;
    _injectedH = h;
    // Also reset the odometry accumulator so camera fixes reset the OTOS model
    // (mirrors MockOtosSensor::setInjectedPose).
    _odomX = x;
    _odomY = y;
    _odomH = h;
}

void SimOdometer::tick(float velL, float velR, float tw, uint32_t dt_ms) {
#ifdef HOST_BUILD
    if (!_useSimModel || tw <= 0.0f) return;
    float dt_s    = static_cast<float>(dt_ms) / 1000.0f;
    float dC      = (velL + velR) * 0.5f * dt_s;
    float dTh     = (velR - velL) / tw * dt_s;
    // Gaussian noise (zero-mean, as before).
    float noisyDC  = dC  * (1.0f + otosGaussian(_rng, _linearNoiseSigma));
    float noisyDTh = dTh * (1.0f + otosGaussian(_rng, _yawNoiseSigma));
    // Deterministic scale error (ticket 057-005): multiplies the noisy delta by
    // (1 + scaleErr).  Applied after Gaussian noise so both compose naturally.
    // Default _linearScaleErr == 0 and _angularScaleErr == 0 → no-op.
    noisyDC  *= (1.0f + _linearScaleErr);
    noisyDTh *= (1.0f + _angularScaleErr);
    float hMid    = _odomH + noisyDTh * 0.5f;
    _odomX += noisyDC * cosf(hMid);
    _odomY += noisyDC * sinf(hMid);
    _odomH += noisyDTh;
    // Deterministic drift (ticket 057-005): additive offset accumulated per tick.
    // Default _driftPerTickMm == 0 and _driftPerTickRad == 0 → no-op.
    _odomX += _driftPerTickMm;
    _odomH += _driftPerTickRad;
    // Wrap heading to [-pi, pi].
    while (_odomH >  static_cast<float>(M_PI)) _odomH -= 2.0f * static_cast<float>(M_PI);
    while (_odomH < -static_cast<float>(M_PI)) _odomH += 2.0f * static_cast<float>(M_PI);

    // Body-frame velocity/accel from the same noisy arc segment (consistent with
    // the position channel). v = arc-distance / dt, omega = dTheta / dt.
    if (dt_s > 0.0f) {
        float newV     = noisyDC  / dt_s;
        float newOmega = noisyDTh / dt_s;
        _accAx    = (newV - _prevVelV) / dt_s;
        _accAy    = 0.0f;
        _prevVelV = newV;
        _velV     = newV;
        _velOmega = newOmega;
    }
#else
    (void)velL; (void)velR; (void)tw; (void)dt_ms;
#endif
}
