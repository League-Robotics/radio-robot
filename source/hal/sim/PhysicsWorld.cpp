#include "PhysicsWorld.h"
#include "control/Odometry.h"   // effectiveSlip() — the SAME helper Odometry::predict uses
#include <math.h>

#ifdef HOST_BUILD
#include <random>

// Gaussian noise helper — returns a sample from N(0, sigma), or 0 if sigma <= 0.
// Bit-identical to MockMotor.cpp::gaussianNoise so the reported-encoder noise
// stream matches the retired model draw-for-draw (OQ-1 Option A).
static float pwGaussianNoise(std::mt19937& rng, float sigma) {
    if (sigma <= 0.0f) return 0.0f;
    std::normal_distribution<float> dist(0.0f, sigma);
    return dist(rng);
}
#endif

// ---------------------------------------------------------------------------
// update(dt_ms) — one canonical midpoint-arc integration step.
//
// Two structurally separate sub-steps, kept apart for golden-TLM bit-exactness:
//
//   Sub-step A — encoder accumulation.
//   Sub-step B — chassis pose integration (slip applied here).
// ---------------------------------------------------------------------------
void PhysicsWorld::update(uint32_t dt_ms) {
    if (dt_ms == 0) return;
    float dt_s = static_cast<float>(dt_ms) / 1000.0f;

    // -----------------------------------------------------------------------
    // Sub-step A: encoder accumulation (true wheel travel; no slip here).
    //
    // GOLDEN-TLM CRITICAL — DO NOT REFACTOR / SIMPLIFY.
    // This expression MUST match MockMotor::integrate bit-for-bit for the
    // golden-TLM canary (zero-slip / zero-noise / offset-factor-1.0 inputs).
    // MockMotor::integrate computes:
    //     float vel   = (_cmdSpeed / 100.0f) * kNominalMaxMms * _offsetFactor;
    //     float noisy = vel * (1.0f - slip) + gaussianNoise(...);   // golden: noisy == vel
    //     _encoderMm += noisy * (static_cast<float>(dt_ms) / 1000.0f);
    // With slip == 0 and noise == 0 the accumulation reduces to vel * dt_s, the
    // exact float operation order below.  Same float type (float, not double),
    // same operation order, no algebraic simplification.
    // -----------------------------------------------------------------------
    float velL = (_pwmL / 100.0f) * _nominalMaxMms * _offsetFactorL;
    float velR = (_pwmR / 100.0f) * _nominalMaxMms * _offsetFactorR;
    _trueEncLMm += velL * dt_s;
    _trueEncRMm += velR * dt_s;
    _trueVelLMms = velL;
    _trueVelRMms = velR;

    // -----------------------------------------------------------------------
    // Sub-step A': reported-encoder accumulation (OQ-1 Option A — legacy
    // MockMotor::integrate model, used by SimMotor::positionMm()).
    //
    // GOLDEN-TLM CRITICAL — DO NOT REFACTOR / SIMPLIFY.  This is the EXACT
    // MockMotor::integrate body (same float type, same operation order, same
    // per-wheel std::mt19937{42u} noise stream):
    //     float slip  = _slipStraight + _slipTurnExtra * _turnRate;
    //     float noisy = vel * (1.0f - slip) + gaussianNoise(rng, sigma);
    //     _encoderMm += noisy * (dt_ms / 1000.0f);
    // With slip == 0, noise == 0, offset-factor 1.0 (golden-TLM fixture) this
    // reduces to vel * dt_s — bit-identical to the true accumulator above AND to
    // the value the retired MockMotor produced.  The sim_field_profile fixture
    // sets _slipTurnExtra (< 0 → encoder over-reports arc on turns), preserving
    // the slip-fence behaviour byte-for-byte.
    // -----------------------------------------------------------------------
    float encSlip = _slipStraight + _slipTurnExtra * _turnRate;
#ifdef HOST_BUILD
    float noisyL = velL * (1.0f - encSlip) + pwGaussianNoise(_rngL, _encNoiseSigmaL);
    float noisyR = velR * (1.0f - encSlip) + pwGaussianNoise(_rngR, _encNoiseSigmaR);
#else
    float noisyL = velL * (1.0f - encSlip);
    float noisyR = velR * (1.0f - encSlip);
#endif
    // Encoder error injection (ticket 058-001): apply per-wheel scale error and
    // slip to the reported delta AFTER the legacy slip+noise step, before
    // accumulation.  True accumulator is untouched — ground truth is preserved.
    // Both default to zero (1.0f * 1.0f = no change) so golden-TLM is unaffected.
    float deltaL = noisyL * dt_s * (1.0f + _encScaleErrL) * (1.0f - _encSlipL);
    float deltaR = noisyR * dt_s * (1.0f + _encScaleErrR) * (1.0f - _encSlipR);
    _reportedEncLMm += deltaL;
    _reportedEncRMm += deltaR;

    // -----------------------------------------------------------------------
    // Sub-step B: chassis pose integration (NOT on the TLM path; clean formula).
    //
    // Slip is applied to the body-rotation term dTheta here (parallel to
    // Odometry::predict), NOT to the encoder in sub-step A.  effectiveSlip()
    // maps 0/unset → 1.0 (no slip), so the golden-TLM fixture (which never calls
    // sim_set_motor_slip) leaves this sub-step's heading unscaled and the canary
    // is unaffected.
    // -----------------------------------------------------------------------
    float dL   = velL * dt_s;
    float dR   = velR * dt_s;
    float slip = effectiveSlip(_rotationalSlip);
    float dTh  = ((dR - dL) / _trackwidthMm) * slip;
    float hMid = _truePoseH + dTh * 0.5f;
    _truePoseX += (dL + dR) * 0.5f * cosf(hMid);
    _truePoseY += (dL + dR) * 0.5f * sinf(hMid);
    _truePoseH += dTh;
    // Wrap heading to (-pi, pi] (CR-15 item 1 / ticket 066-001) — matches the
    // wrap SimOdometer already applies to its own _odomH accumulator.  Becomes
    // load-bearing once SimOdometer samples _truePoseH directly (see
    // SimOdometer::tick()) instead of maintaining an independently-wrapped copy.
    while (_truePoseH >  static_cast<float>(M_PI)) _truePoseH -= 2.0f * static_cast<float>(M_PI);
    while (_truePoseH < -static_cast<float>(M_PI)) _truePoseH += 2.0f * static_cast<float>(M_PI);
}

// ---------------------------------------------------------------------------
// reset() — zero all ground-truth state.  Dynamics parameters (trackwidth,
// nominalMaxMms, slip, offset factors) are configuration, not state, and are
// left intact — matching the MockMotor::resetEncoder semantics (which zeros the
// accumulator but keeps the configured slip / offset / noise).
// ---------------------------------------------------------------------------
void PhysicsWorld::reset() {
    _pwmL = 0;
    _pwmR = 0;

    _truePoseX = 0.0f;
    _truePoseY = 0.0f;
    _truePoseH = 0.0f;

    _trueEncLMm  = 0.0f;
    _trueEncRMm  = 0.0f;
    _trueVelLMms = 0.0f;
    _trueVelRMms = 0.0f;

    _reportedEncLMm = 0.0f;
    _reportedEncRMm = 0.0f;

    for (int i = 0; i < 4; ++i) {
        _lineRaw[i]   = 0;
        _colorRGBC[i] = 0;
        _port[i]      = 0;
    }
}
