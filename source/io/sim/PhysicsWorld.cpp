#include "PhysicsWorld.h"
#include "control/Odometry.h"   // effectiveSlip() — the SAME helper Odometry::predict uses
#include <math.h>

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

    for (int i = 0; i < 4; ++i) {
        _lineRaw[i]   = 0;
        _colorRGBC[i] = 0;
        _port[i]      = 0;
    }
}
