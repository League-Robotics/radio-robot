#include "SimOdometer.h"
#include "types/Config.h"
#include "hal/capability/OtosLeverArm.h"
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
        // Lever-arm round-trip (ticket 066-001, CR-07/CR-08): project the
        // accumulated CENTRE estimate through centreToSensor() to synthesize
        // what the chip's own optical-flow tracker would report (a sim-only
        // step — real hardware's chip organically observes its own
        // sensor-frame motion), then immediately call sensorToCentre() — the
        // SAME function OtosSensor::readTransformed() calls — to recover the
        // centre.  Correct OtosLeverArm.h math makes this an exact no-op; a
        // future regression there (the db11b7c failure mode) makes it NOT
        // cancel, exactly as it would on hardware.
        float sensorX = 0.0f, sensorY = 0.0f;
        centreToSensor(_odomX, _odomY, _odomH, _cfg.odomOffX, _cfg.odomOffY,
                       sensorX, sensorY);
        float centreX = 0.0f, centreY = 0.0f;
        sensorToCentre(sensorX, sensorY, _odomH, _cfg.odomOffX, _cfg.odomOffY,
                       centreX, centreY);
        poseOut.x = centreX;
        poseOut.y = centreY;
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

int32_t SimOdometer::controlPeriodMs() const {
    return _cfg.controlPeriodMs;
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

void SimOdometer::tick(uint32_t dt_ms) {
#ifdef HOST_BUILD
    // Ground-truth sampling (ticket 066-001, CR-07/CR-08): read the plant's
    // current true CENTRE pose.  _prevTrueX/Y/H is rebaselined to this value
    // on EVERY tick() call below — including the WARN-frozen and
    // sim-model-disabled early returns — so the accumulator never has to
    // "catch up" on motion that happened while sampling was skipped; the
    // next active tick's delta only ever reflects motion since THIS tick.
    float curTrueX = _plant.truePoseX();
    float curTrueY = _plant.truePoseY();
    float curTrueH = _plant.truePoseH();

    if (_warnOptical) {
        // WARN (065-006): model "frozen pose, near-zero velocity" — skip the
        // odometry-accumulator update entirely (pose stays pinned at
        // whatever _odomX/Y/H held when the warn condition began) and zero
        // the velocity/accel outputs.  Encoders (driven independently of
        // this model) are unaffected.
        _velV     = 0.0f;
        _velOmega = 0.0f;
        _accAx    = 0.0f;
        _accAy    = 0.0f;
        _prevVelV = 0.0f;
        _prevTrueX = curTrueX;
        _prevTrueY = curTrueY;
        _prevTrueH = curTrueH;
        return;
    }
    if (!_useSimModel) {
        _prevTrueX = curTrueX;
        _prevTrueY = curTrueY;
        _prevTrueH = curTrueH;
        return;
    }
    float dt_s = static_cast<float>(dt_ms) / 1000.0f;

    // World-frame delta since the previous sample.
    float dx  = curTrueX - _prevTrueX;
    float dy  = curTrueY - _prevTrueY;
    float dTh = curTrueH - _prevTrueH;
    // Wrap dTh to (-pi, pi] in case _truePoseH wrapped across the boundary
    // between samples (PhysicsWorld::update() wraps _truePoseH every step —
    // CR-15 item 1 / ticket 066-001); the true per-tick angular change is
    // always small, so wrapping the raw diff recovers it exactly.
    while (dTh >  static_cast<float>(M_PI)) dTh -= 2.0f * static_cast<float>(M_PI);
    while (dTh <= -static_cast<float>(M_PI)) dTh += 2.0f * static_cast<float>(M_PI);

    // Recover the body-frame forward arc dC by projecting the world-frame
    // delta onto the plant's own midpoint heading — the exact inverse of the
    // midpoint-arc integration PhysicsWorld::update() used to produce (dx,dy)
    // from dC in the first place, so this recovers dC exactly (mod float
    // rounding) regardless of any chassis-truth slip PhysicsWorld applied to
    // dTh: whatever actually happened to the plant is what gets sampled here.
    float plantHMid = _prevTrueH + dTh * 0.5f;
    float dC = dx * cosf(plantHMid) + dy * sinf(plantHMid);

    _prevTrueX = curTrueX;
    _prevTrueY = curTrueY;
    _prevTrueH = curTrueH;

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
    (void)dt_ms;
#endif
}
