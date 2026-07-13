#include "devices/velocity_pid.h"

#include <cmath>

namespace Devices {

namespace {
float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
}  // namespace

// ---------------------------------------------------------------------------
// Embedded velocity PID — ported byte-for-byte from
// source/hal/velocity_pid.cpp's Hal::MotorVelocityPid::compute() (itself
// ported from source_old/control/VelocityController.cpp::update() in sprint
// 081-001 — see that file's own header for the full transaction history).
// Only the surrounding types changed (msg::Gains -> Devices::Gains, minDuty
// -> velDeadband — see velocity_pid.h's file header); the control law below
// is unchanged.
//
// VelocityController composes cmon-pid's backcalculation_t<pid_bwe> with
// Kd=0, Tf=kTinyTf (~1e-6s). For that configuration, pid_bwe's transfer-
// function coefficients collapse (A1 = Tf/(h+Tf) ~ 0, C3 = Kd/Tf = 0, A3 ~
// Kp), so the general transfer-function machinery reduces to a plain
// discrete PI with back-calculation anti-windup. This function implements
// that reduced form directly rather than pulling in cmon-pid, avoiding a
// second vendored dependency for a fresh (not-yet-bench-tuned) config
// surface (MotorConfig.velGains) with no established prior calibration to
// match bit-for-bit.
//
// One documented divergence from source_old's literal behavior: in the
// deadband (integrator-freeze) branch, source_old calls cmon-pid's
// ReInit(0, I_old), which (per the derivation above, with the D register
// holding a stale Kp*err term from the last non-deadband tick) does not
// exactly hold the integrator at I_old despite the code comment's stated
// intent ("keep I where it is"). This port implements that STATED intent
// literally (freeze the integrator unchanged) rather than reproducing the
// stale-D subtraction, which reads as an unintended quirk of the composed
// transfer-function shim rather than a deliberate design element.
//
// Output domain: duty fraction [-1, 1] (matching the write path's
// setDutyCycle-equivalent contract), not the old [-100, 100] PWM-percent
// domain.
// ---------------------------------------------------------------------------
float MotorVelocityPid::compute(float target, float measured, float dt,
                                 const Gains& gains, float velDeadband)
{
    if (dt <= 0.0f) dt = kNominalDt;

    float spAbs = fabsf(target);

    // velDeadband plays minWheelSpeed's role here (integrator-freeze
    // deadband threshold on |target|) — device_config.h's field comment
    // documents the full rename history from the misleading wire-key name
    // this Devices-local field carries forward the SEMANTICS of, not the
    // name of. <= (not <) so an exact target==0.0f still counts as "in the
    // deadband" even when velDeadband itself is 0.0 (unconfigured) — the
    // common case for a fresh boot config: a literal zero target always
    // means "come to a stop," independent of whether a stiction floor has
    // been tuned.
    bool inDeadband = spAbs <= velDeadband;

    // 086-002 root fix: a plain FREEZE (leave integral_ at whatever it
    // held) preserves whatever bias the integrator built up sustaining the
    // PRIOR, unrelated motion (e.g. a fast turn) straight into the new
    // near-zero-target regime. Once the ramp's target lands at (or below)
    // the deadband, that carried-over bias — combined with a fresh, still-
    // large kp*err — is exactly what the issue's own instrumentation shows
    // landing as an oversized, wrong-signed correction once the wheel
    // coasts past zero: not because the fresh correction is undamped, but
    // because it is riding on top of a stale one. Resetting the integrator
    // on the tick the deadband is FIRST entered (edge-triggered on
    // wasInDeadband_, not level-held — a continuing low/zero target keeps
    // freezing exactly as before, so a genuine bench-tuned low-speed creep
    // still gets zero ongoing integral action, unchanged from the pre-fix
    // behavior) clears that stale bias before it can leak into the stop,
    // while never touching the armor's own reversal-dwell gate (an
    // unrequested full-scale reversal never passes through this deadband at
    // all, since |target| stays large on both sides of that flip).
    if (inDeadband && !wasInDeadband_) {
        integral_ = 0.0f;
    }
    wasInDeadband_ = inDeadband;

    float err = target - measured;
    float spSign = (target >= 0.0f) ? 1.0f : -1.0f;
    float ff = gains.kff * spAbs;

    // Output uses the OLD integrator (pre-update, but post-reset above),
    // matching VelocityController::update()'s I_old ordering.
    float iOld = integral_;
    float rawDuty = spSign * ff + gains.kp * err + iOld;
    float output = clampf(rawDuty, -1.0f, 1.0f);

    if (!inDeadband) {
        float newIntegral = iOld + gains.ki * dt * err;
        // Anti-windup back-calculation against +/- iMax (mirrors
        // backcalculation_t<pid_bwe>::Update's saturation check on
        // kp*err + newIntegral, with C3=0 and D~=kp*err for Kd=0).
        float u = gains.kp * err + newIntegral;
        float tw = (gains.kaw > 0.0f) ? (1.0f / gains.kaw) : 1e6f;
        float cW = (tw > dt) ? (dt / tw) : 1.0f;
        if (u > gains.iMax) {
            newIntegral += cW * (gains.iMax - u);
        } else if (u < -gains.iMax) {
            newIntegral += cW * (-gains.iMax - u);
        }
        integral_ = newIntegral;
    }
    // else: frozen — integral_ left unchanged (see file-level comment on
    // the deliberate divergence from source_old's ReInit() call here, and
    // the reset-on-entry comment above for the one behavior change from
    // that original freeze semantics).

    return output;
}

}  // namespace Devices
