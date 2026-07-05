#include "hal/sim/sim_odometer.h"

#include <math.h>

#ifdef HOST_BUILD
#include <random>

// Gaussian noise helper — same shape as physics_world.cpp's pwGaussianNoise,
// against SimOdometer's own independent generator.
static float otosGaussianNoise(std::mt19937& rng, float sigma) {
    if (sigma <= 0.0f) return 0.0f;
    std::normal_distribution<float> dist(0.0f, sigma);
    return dist(rng);
}
#endif

namespace Hal {

SimOdometer::SimOdometer(const PhysicsWorld& plant) : plant_(plant) {}

bool SimOdometer::connected() const { return true; }

msg::PoseEstimate SimOdometer::pose() const
{
    msg::PoseEstimate p;
    p.pose.x = odomX_;
    p.pose.y = odomY_;
    p.pose.h = odomH_;
    p.twist.v_x = velV_;
    p.twist.v_y = 0.0f;   // differential-only ground truth this sprint — no lateral component
    p.twist.omega = velOmega_;
    p.stamp.valid = hasLastTick_;
    p.stamp.last_upd = lastTick_;
    p.stamp.lag = 0;
    return p;
}

void SimOdometer::tick(uint32_t now)
{
    float curTrueX = plant_.truePoseX();
    float curTrueY = plant_.truePoseY();
    float curTrueH = plant_.truePoseH();

    if (!hasLastTick_) {
        // First call ever: establish the ground-truth sampling baseline —
        // no delta to integrate yet (mirrors Hal::SimMotor/PhysicsWorld's
        // own "first tick just baselines" convention).
        hasLastTick_ = true;
        lastTick_ = now;
        prevTrueX_ = curTrueX;
        prevTrueY_ = curTrueY;
        prevTrueH_ = curTrueH;
        return;
    }

    uint32_t dtMs = now - lastTick_;
    lastTick_ = now;
    if (dtMs == 0) {
        // Defensive only — Subsystems::SimHardware's dt=0 re-entry guard
        // (architecture-update.md (081) Decision 4) already prevents this
        // class's tick() from ever being called twice with an unchanged
        // `now`; this branch only matters for a direct/unit-test caller.
        return;
    }
    float dt_s = static_cast<float>(dtMs) / 1000.0f;

    // World-frame delta since the previous sample.
    float dx  = curTrueX - prevTrueX_;
    float dy  = curTrueY - prevTrueY_;
    float dTh = curTrueH - prevTrueH_;
    // Wrap dTh to (-pi, pi] in case truePoseH() wrapped across the boundary
    // between samples (PhysicsWorld::update() wraps its heading every
    // step) — the true per-tick angular change is always small, so
    // wrapping the raw diff recovers it exactly.
    while (dTh >  static_cast<float>(M_PI)) dTh -= 2.0f * static_cast<float>(M_PI);
    while (dTh <= -static_cast<float>(M_PI)) dTh += 2.0f * static_cast<float>(M_PI);

    // Recover the body-frame forward arc dC by projecting the world-frame
    // delta onto the plant's own midpoint heading — the exact inverse of
    // the midpoint-arc integration PhysicsWorld::update() used to produce
    // (dx, dy) from dC in the first place.
    float plantHMid = prevTrueH_ + dTh * 0.5f;
    float dC = dx * cosf(plantHMid) + dy * sinf(plantHMid);

    prevTrueX_ = curTrueX;
    prevTrueY_ = curTrueY;
    prevTrueH_ = curTrueH;

#ifdef HOST_BUILD
    float noisyDC  = dC  * (1.0f + otosGaussianNoise(rng_, linearNoiseSigma_));
    float noisyDTh = dTh * (1.0f + otosGaussianNoise(rng_, yawNoiseSigma_));
#else
    float noisyDC = dC;
    float noisyDTh = dTh;
#endif
    // Deterministic scale error: multiplies the noisy delta by
    // (1 + scaleErr). Applied after Gaussian noise so both compose
    // naturally. Default zero -> no-op.
    noisyDC  *= (1.0f + linearScaleErr_);
    noisyDTh *= (1.0f + angularScaleErr_);

    float hMid = odomH_ + noisyDTh * 0.5f;
    odomX_ += noisyDC * cosf(hMid);
    odomY_ += noisyDC * sinf(hMid);
    odomH_ += noisyDTh;

    // Deterministic drift: additive offset accumulated per tick. Default
    // zero -> no-op.
    odomX_ += linearDriftPerTick_;
    odomH_ += yawDriftPerTick_;

    // Wrap heading to (-pi, pi].
    while (odomH_ >  static_cast<float>(M_PI)) odomH_ -= 2.0f * static_cast<float>(M_PI);
    while (odomH_ < -static_cast<float>(M_PI)) odomH_ += 2.0f * static_cast<float>(M_PI);

    if (dt_s > 0.0f) {
        velV_     = noisyDC  / dt_s;
        velOmega_ = noisyDTh / dt_s;
    }
}

void SimOdometer::setLinearNoiseSigma(float sigma)   { linearNoiseSigma_ = sigma; }
void SimOdometer::setYawNoiseSigma(float sigma)       { yawNoiseSigma_ = sigma; }
void SimOdometer::setLinearScaleError(float err)      { linearScaleErr_ = err; }
void SimOdometer::setAngularScaleError(float err)     { angularScaleErr_ = err; }
void SimOdometer::setLinearDriftPerTick(float drift)  { linearDriftPerTick_ = drift; }
void SimOdometer::setYawDriftPerTick(float drift)     { yawDriftPerTick_ = drift; }

}  // namespace Hal
