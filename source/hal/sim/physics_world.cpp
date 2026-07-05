#include "hal/sim/physics_world.h"

#include <math.h>

#ifdef HOST_BUILD
#include <random>

// Gaussian noise helper — returns a sample from N(0, sigma), or 0 if
// sigma <= 0. Bit-identical to source_old's MockMotor.cpp::gaussianNoise so
// the reported-encoder noise stream matches the retired model draw-for-draw.
static float pwGaussianNoise(std::mt19937& rng, float sigma) {
    if (sigma <= 0.0f) return 0.0f;
    std::normal_distribution<float> dist(0.0f, sigma);
    return dist(rng);
}
#endif

namespace Hal {

namespace {

// ---------------------------------------------------------------------------
// effectiveSlip — migration-safe rotationalSlip clamp, ported from
// source_old/control/Odometry.h (used there by Odometry::predict() and
// Planner::beginRotation()). The new tree has no shared Odometry/Planner
// class yet (out of this sprint's scope), so this stays a small, file-local
// helper rather than a premature shared extraction — its only caller is
// PhysicsWorld::update() below, exactly as source_old's PhysicsWorld.cpp
// used it via the #include this ports.
//
// Value semantics:
//   0.0 (or negative, or unset) -> 1.0  (no correction; legacy config-safe)
//   (0.0, 0.5)                  -> 0.5  (clamp floor — unrealistic slip)
//   [0.5, 1.0]                  -> pass-through
//   > 1.0                       -> 1.0  (clamp ceiling)
// ---------------------------------------------------------------------------
float effectiveSlip(float rawSlip) {
    if (rawSlip <= 0.0f) return 1.0f;
    if (rawSlip < 0.5f)  return 0.5f;
    if (rawSlip > 1.0f)  return 1.0f;
    return rawSlip;
}

// ---------------------------------------------------------------------------
// clampScrub — body-rotational/body-linear scrub clamp. Deliberately NOT
// effectiveSlip(): effectiveSlip()'s 0/unset-means-1.0 and [0.5, 1.0] floor
// encode rotationalSlip_'s own migration/hardware-calibration history —
// history bodyRotationalScrub_/bodyLinearScrub_ (brand-new fields with no
// prior deployed config) do not share. clampScrub() clamps only at the
// boundaries that would make the physics nonsensical:
//   <= 0   -> kMinScrub (a small positive floor) — a true 0 or negative
//             scrub would mean the chassis never rotates/translates at all
//             or moves backwards relative to the commanded arc, not a
//             scrub.
//   > 1.0  -> 1.0 (ceiling) — amplification is not a physical claim this
//             sprint models.
//   (0, 1] -> pass-through.
// ---------------------------------------------------------------------------
constexpr float kMinScrub = 0.01f;

float clampScrub(float f) {
    if (f <= 0.0f) return kMinScrub;
    if (f > 1.0f)  return 1.0f;
    return f;
}

}  // namespace

// ---------------------------------------------------------------------------
// update(dt) — one canonical midpoint-arc integration step.
//
// Two structurally separate sub-steps, kept apart for golden-TLM
// bit-exactness:
//   Sub-step A / A' — encoder accumulation (true, then reported).
//   Sub-step B — chassis pose integration (slip applied here).
// ---------------------------------------------------------------------------
void PhysicsWorld::update(uint32_t dt) {  // [ms]
    if (dt == 0) return;
    float dt_s = static_cast<float>(dt) / 1000.0f;

    // -----------------------------------------------------------------------
    // Sub-step A: encoder accumulation (true wheel travel; no slip here).
    //
    // GOLDEN-TLM CRITICAL — DO NOT REFACTOR / SIMPLIFY. This expression MUST
    // match source_old's MockMotor::integrate bit-for-bit for the zero-slip /
    // zero-noise / offset-factor-1.0 inputs (the zero-error determinism
    // gate). Same float type (float, not double), same operation order, no
    // algebraic simplification.
    //
    // The stiction/breakaway gate and the optional first-order lag filter
    // sit between the algebraic formula and the encoder/pose consumers
    // below. At default parameters (stictionPwm == 0, tau <= 0) neither
    // stage performs any arithmetic on the value it passes through — the
    // gate's condition (fabsf(pwm) < 0) is never true, and the lag stage
    // takes the tau<=0 assignment-only path — so velL/velR below are
    // BIT-IDENTICAL to algVelL/algVelR, preserving the golden-TLM guarantee.
    // -----------------------------------------------------------------------
    float algVelL = (pwmL_ / 100.0f) * nominalMaxSpeed_ * offsetFactorL_;
    float algVelR = (pwmR_ / 100.0f) * nominalMaxSpeed_ * offsetFactorR_;

    // Stiction/breakaway gate — stateless PWM dead-zone. |pwm| <
    // stictionPwmSide => this tick's target velocity is forced to 0,
    // regardless of the wheel's velocity on the previous tick. Once
    // |pwm| >= stictionPwmSide, the unmodified algebraic velocity applies.
    float velTargetL = (fabsf(static_cast<float>(pwmL_)) < stictionPwmL_) ? 0.0f : algVelL;
    float velTargetR = (fabsf(static_cast<float>(pwmR_)) < stictionPwmR_) ? 0.0f : algVelR;

    // Optional first-order response lag (independent, separately defaulted-
    // off knob). tau <= 0 skips the expf() call entirely so vel ==
    // velTarget bit-for-bit; tau > 0 exponentially converges the persistent
    // lagVelL_/R_ state toward velTarget.
    float velL;
    if (motorLagL_ <= 0.0f) {
        velL = velTargetL;
    } else {
        float alphaL = 1.0f - expf(-dt_s / (motorLagL_ * 0.001f));
        velL = lagVelL_ + (velTargetL - lagVelL_) * alphaL;
    }
    lagVelL_ = velL;

    float velR;
    if (motorLagR_ <= 0.0f) {
        velR = velTargetR;
    } else {
        float alphaR = 1.0f - expf(-dt_s / (motorLagR_ * 0.001f));
        velR = lagVelR_ + (velTargetR - lagVelR_) * alphaR;
    }
    lagVelR_ = velR;

    trueEncL_ += velL * dt_s;
    trueEncR_ += velR * dt_s;
    trueVelL_ = velL;
    trueVelR_ = velR;

    // -----------------------------------------------------------------------
    // Sub-step A': reported-encoder accumulation (legacy MockMotor::integrate
    // model, used by Hal::SimMotor::position()).
    //
    // GOLDEN-TLM CRITICAL — DO NOT REFACTOR / SIMPLIFY. With slip == 0,
    // noise == 0, offset-factor 1.0 this reduces to vel * dt_s — bit-
    // identical to the true accumulator above.
    // -----------------------------------------------------------------------
    float encSlip = slipStraight_ + slipTurnExtra_ * turnRate_;
#ifdef HOST_BUILD
    float noisyL = velL * (1.0f - encSlip) + pwGaussianNoise(rngL_, encNoiseSigmaL_);
    float noisyR = velR * (1.0f - encSlip) + pwGaussianNoise(rngR_, encNoiseSigmaR_);
#else
    float noisyL = velL * (1.0f - encSlip);
    float noisyR = velR * (1.0f - encSlip);
#endif
    // Encoder error injection: apply per-wheel scale error and slip to the
    // reported delta AFTER the legacy slip+noise step, before accumulation.
    // True accumulator is untouched — ground truth is preserved. Both
    // default to zero (1.0f * 1.0f = no change) so the zero-error
    // determinism gate holds.
    float deltaL = noisyL * dt_s * (1.0f + encScaleErrL_) * (1.0f - encSlipL_);
    float deltaR = noisyR * dt_s * (1.0f + encScaleErrR_) * (1.0f - encSlipR_);
    reportedEncL_ += deltaL;
    reportedEncR_ += deltaR;

    // -----------------------------------------------------------------------
    // Sub-step B: chassis pose integration (NOT on the TLM path; clean
    // formula).
    //
    // Slip is applied to the body-rotation term dTheta here, NOT to the
    // encoder in sub-step A. effectiveSlip() maps 0/unset -> 1.0 (no slip),
    // so the zero-error determinism gate (which never calls setSlip())
    // leaves this sub-step's heading unscaled.
    //
    // bodyRotationalScrub_/bodyLinearScrub_ are independent, default-neutral
    // (1.0) multipliers, combined MULTIPLICATIVELY with the existing,
    // unchanged effectiveSlip(rotationalSlip_) term. At the 1.0 default
    // this is byte-identical to the pre-scrub expression.
    // -----------------------------------------------------------------------
    float dL       = velL * dt_s;
    float dR       = velR * dt_s;
    float slip     = effectiveSlip(rotationalSlip_) * clampScrub(bodyRotationalScrub_);
    float dTh      = ((dR - dL) / trackwidth_) * slip;
    float hMid     = truePoseH_ + dTh * 0.5f;
    float linScrub = clampScrub(bodyLinearScrub_);
    truePoseX_ += (dL + dR) * 0.5f * linScrub * cosf(hMid);
    truePoseY_ += (dL + dR) * 0.5f * linScrub * sinf(hMid);
    truePoseH_ += dTh;
    // Wrap heading to (-pi, pi] — matches the wrap Hal::SimOdometer applies
    // to its own accumulator.
    while (truePoseH_ >  static_cast<float>(M_PI)) truePoseH_ -= 2.0f * static_cast<float>(M_PI);
    while (truePoseH_ < -static_cast<float>(M_PI)) truePoseH_ += 2.0f * static_cast<float>(M_PI);
}

// ---------------------------------------------------------------------------
// reset() — zero all ground-truth state. Dynamics parameters (trackwidth,
// nominalMaxSpeed, slip, offset factors) are configuration, not state, and
// are left intact.
// ---------------------------------------------------------------------------
void PhysicsWorld::reset() {
    pwmL_ = 0;
    pwmR_ = 0;

    truePoseX_ = 0.0f;
    truePoseY_ = 0.0f;
    truePoseH_ = 0.0f;

    trueEncL_ = 0.0f;
    trueEncR_ = 0.0f;
    trueVelL_ = 0.0f;
    trueVelR_ = 0.0f;

    reportedEncL_ = 0.0f;
    reportedEncR_ = 0.0f;

    // The lag filter's persistent output state is STATE, not configuration —
    // zeroed here like every other accumulator above. The stiction
    // threshold / lag time-constant knobs themselves are configuration and
    // are left intact.
    lagVelL_ = 0.0f;
    lagVelR_ = 0.0f;
}

}  // namespace Hal
