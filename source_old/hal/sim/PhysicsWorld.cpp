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
// clampScrub — 069-002's body-rotational/body-linear scrub clamp.
//
// Deliberately NOT effectiveSlip() (Odometry.h). effectiveSlip()'s
// 0/unset-means-1.0 and [0.5, 1.0] floor encode _rotationalSlip's own
// migration/hardware-calibration history (see architecture-update.md
// Decision 2) — history that _bodyRotationalScrub/_bodyLinearScrub, brand-new
// fields with no prior deployed config, do not share. clampScrub() clamps
// only at the boundaries that would make the physics nonsensical:
//   <= 0   → kMinScrub (a small positive floor). A true 0 or negative scrub
//            would mean the chassis never rotates/translates at all or moves
//            backwards relative to the commanded arc — not a scrub, a
//            division-by-zero / sign-flip pathology — so it floors rather
//            than silently mapping to 1.0 (which would mask a caller passing
//            a nonsensical value as "no scrub configured").
//   > 1.0  → 1.0 (ceiling). A value above 1.0 would mean the chassis
//            rotates/travels MORE than naive kinematics predicts — a
//            different physical claim ("amplification") this sprint does not
//            model.
//   (0, 1] → pass-through.
// ---------------------------------------------------------------------------
static constexpr float kMinScrub = 0.01f;

static float clampScrub(float f) {
    if (f <= 0.0f) return kMinScrub;
    if (f > 1.0f)  return 1.0f;
    return f;
}

// ---------------------------------------------------------------------------
// update(dt) — one canonical midpoint-arc integration step.
//
// Two structurally separate sub-steps, kept apart for golden-TLM bit-exactness:
//
//   Sub-step A — encoder accumulation.
//   Sub-step B — chassis pose integration (slip applied here).
// ---------------------------------------------------------------------------
void PhysicsWorld::update(uint32_t dt) {  // [ms]
    if (dt == 0) return;
    float dt_s = static_cast<float>(dt) / 1000.0f;

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
    //
    // 072-001: two NEW stages sit between the algebraic formula and the
    // encoder/pose consumers below — the stiction/breakaway gate and the
    // optional first-order lag filter (architecture-update.md Step 4b's
    // insertion diagram). At default parameters (stictionPwm == 0, tau <= 0)
    // neither stage performs any arithmetic on the value it passes through —
    // the gate's condition (fabsf(pwm) < 0) is never true, and the lag stage
    // takes the tau<=0 assignment-only path — so velL/velR below are
    // BIT-IDENTICAL to algVelL/algVelR, preserving the golden-TLM guarantee.
    // -----------------------------------------------------------------------
    float algVelL = (_pwmL / 100.0f) * _nominalMaxSpeed * _offsetFactorL;
    float algVelR = (_pwmR / 100.0f) * _nominalMaxSpeed * _offsetFactorR;

    // NEW: stiction/breakaway gate — stateless PWM dead-zone (Decision 3).
    // |pwm| < stictionPwmSide => this tick's target velocity is forced to 0,
    // regardless of the wheel's velocity on the previous tick. Once
    // |pwm| >= stictionPwmSide, the unmodified algebraic velocity applies.
    float velTargetL = (fabsf(static_cast<float>(_pwmL)) < _stictionPwmL) ? 0.0f : algVelL;
    float velTargetR = (fabsf(static_cast<float>(_pwmR)) < _stictionPwmR) ? 0.0f : algVelR;

    // NEW: optional first-order response lag (independent, separately
    // defaulted-off knob — Decision 3). tau <= 0 skips the expf() call
    // entirely so vel == velTarget bit-for-bit; tau > 0 exponentially
    // converges the persistent _lagVelL/R state toward velTarget.
    float velL;
    if (_motorLagL <= 0.0f) {
        velL = velTargetL;
    } else {
        float alphaL = 1.0f - expf(-dt_s / (_motorLagL * 0.001f));
        velL = _lagVelL + (velTargetL - _lagVelL) * alphaL;
    }
    _lagVelL = velL;

    float velR;
    if (_motorLagR <= 0.0f) {
        velR = velTargetR;
    } else {
        float alphaR = 1.0f - expf(-dt_s / (_motorLagR * 0.001f));
        velR = _lagVelR + (velTargetR - _lagVelR) * alphaR;
    }
    _lagVelR = velR;

    _trueEncL += velL * dt_s;
    _trueEncR += velR * dt_s;
    _trueVelL = velL;
    _trueVelR = velR;

    // -----------------------------------------------------------------------
    // Sub-step A': reported-encoder accumulation (OQ-1 Option A — legacy
    // MockMotor::integrate model, used by SimMotor::position()).
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
    _reportedEncL += deltaL;
    _reportedEncR += deltaR;

    // -----------------------------------------------------------------------
    // Sub-step B: chassis pose integration (NOT on the TLM path; clean formula).
    //
    // Slip is applied to the body-rotation term dTheta here (parallel to
    // Odometry::predict), NOT to the encoder in sub-step A.  effectiveSlip()
    // maps 0/unset → 1.0 (no slip), so the golden-TLM fixture (which never calls
    // sim_set_motor_slip) leaves this sub-step's heading unscaled and the canary
    // is unaffected.
    //
    // 069-002: _bodyRotationalScrub/_bodyLinearScrub are independent,
    // default-neutral (1.0) multipliers, combined MULTIPLICATIVELY with the
    // existing, UNCHANGED effectiveSlip(_rotationalSlip) term — see
    // architecture-update.md §4b/Decision 4. At the 1.0 default this is
    // byte-identical to the pre-069-002 expression.
    // -----------------------------------------------------------------------
    float dL       = velL * dt_s;
    float dR       = velR * dt_s;
    float slip     = effectiveSlip(_rotationalSlip) * clampScrub(_bodyRotationalScrub);
    float dTh      = ((dR - dL) / _trackwidth) * slip;
    float hMid     = _truePoseH + dTh * 0.5f;
    float linScrub = clampScrub(_bodyLinearScrub);
    _truePoseX += (dL + dR) * 0.5f * linScrub * cosf(hMid);
    _truePoseY += (dL + dR) * 0.5f * linScrub * sinf(hMid);
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
// nominalMaxSpeed, slip, offset factors) are configuration, not state, and are
// left intact — matching the MockMotor::resetEncoder semantics (which zeros the
// accumulator but keeps the configured slip / offset / noise).
// ---------------------------------------------------------------------------
void PhysicsWorld::reset() {
    _pwmL = 0;
    _pwmR = 0;

    _truePoseX = 0.0f;
    _truePoseY = 0.0f;
    _truePoseH = 0.0f;

    _trueEncL  = 0.0f;
    _trueEncR  = 0.0f;
    _trueVelL = 0.0f;
    _trueVelR = 0.0f;

    _reportedEncL = 0.0f;
    _reportedEncR = 0.0f;

    // 072-001: the lag filter's persistent output state is STATE, not
    // configuration — zeroed here like every other accumulator above. The
    // stiction threshold / lag time-constant knobs themselves are
    // configuration and are left intact, matching this function's existing
    // convention (see the doc comment above).
    _lagVelL = 0.0f;
    _lagVelR = 0.0f;

    for (int i = 0; i < 4; ++i) {
        _lineRaw[i]   = 0;
        _colorRGBC[i] = 0;
        _port[i]      = 0;
    }
}
