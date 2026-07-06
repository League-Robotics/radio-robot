#include "hal/velocity_pid.h"

#include <math.h>

namespace Hal {

namespace {
float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
}  // namespace

// ---------------------------------------------------------------------------
// Embedded velocity PID — ported control law from
// source_old/control/VelocityController.cpp::update(). Extracted byte-for-
// byte (sprint 081-001) out of NezhaMotor::runVelocityPid() into this
// standalone, host-clean class so a future simulated leaf runs the
// IDENTICAL control law rather than a re-derived approximation.
//
// VelocityController composes cmon-pid's backcalculation_t<pid_bwe> with
// Kd=0, Tf=kTinyTf (~1e-6s). For that configuration, pid_bwe's transfer-
// function coefficients collapse (A1 = Tf/(h+Tf) ~ 0, C3 = Kd/Tf = 0, A3 ~
// Kp), so the general transfer-function machinery reduces to a plain
// discrete PI with back-calculation anti-windup. This function implements
// that reduced form directly rather than pulling in cmon-pid, avoiding a
// second vendored dependency for a fresh (not-yet-bench-tuned) config
// surface (MotorConfig.vel_gains) with no established prior calibration to
// match bit-for-bit. Ticket 003's Verification section gates this ticket
// on "compiles, and the ported [I2C] sequencing matches source_old
// byte-for-byte" — not PID numerical fidelity, which is ticket 7's bench
// pass.
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
// Output domain: duty fraction [-1, 1] (matching Hal::Motor::setDutyCycle's
// contract), not the old [-100, 100] PWM-percent domain — MotorConfig.vel_
// gains is a brand-new surface this sprint, so there is no compatibility
// requirement to preserve the old scale; Gains are tuned against this scale
// in ticket 7's bench pass.
// ---------------------------------------------------------------------------
float MotorVelocityPid::compute(float target, float measured, float dt,
                                 const msg::Gains& gains, float minDuty)
{
    if (dt <= 0.0f) dt = kNominalDt;

    float spAbs = fabsf(target);

    // minDuty plays minWheelSpeed's role here (integrator-freeze deadband
    // threshold on |target|) despite its proto name — see nezha_motor.h's
    // field comment and the ticket's own note that MotorConfig.min_duty's
    // doc string ("stiction floor / integrator-freeze threshold") is
    // exactly VelocityController's minWheelSpeed semantics, just carried
    // under a different generated field name. <= (not <) so an exact
    // target==0.0f still counts as "in the deadband" even when minDuty
    // itself is 0.0 (unconfigured) — the common case for a fresh boot
    // config (086-002): a literal zero target always means "come to a
    // stop," independent of whether a stiction floor has been tuned.
    bool inDeadband = spAbs <= minDuty;

    // 086-002 root fix (architecture-update.md Grounding fact 2 / Design
    // Rationale 1, Invariant B): a plain FREEZE (leave integral_ at
    // whatever it held) preserves whatever bias the integrator built up
    // sustaining the PRIOR, unrelated motion (e.g. a fast turn) straight
    // into the new near-zero-target regime. Once the ramp's target lands
    // at (or below) the deadband, that carried-over bias — combined with a
    // fresh, still-large kp*err — is exactly what the issue's own
    // instrumentation shows landing as an oversized, wrong-signed
    // correction once the wheel coasts past zero: not because the fresh
    // correction is undamped, but because it is riding on top of a stale
    // one. Resetting the integrator on the tick the deadband is FIRST
    // entered (edge-triggered on wasInDeadband_, not level-held — a
    // continuing low/zero target keeps freezing exactly as before, so a
    // genuine bench-tuned low-speed creep still gets zero ongoing integral
    // action, unchanged from the pre-fix behavior) clears that stale bias
    // before it can leak into the stop, while never touching
    // armoredWrite()'s own reversal-dwell gate — Invariant A (an
    // unrequested full-scale reversal never passes through this deadband
    // at all, since |target| stays large on both sides of that flip) is
    // preserved by construction, not by a special case.
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
        // Anti-windup back-calculation against +/- i_max (mirrors
        // backcalculation_t<pid_bwe>::Update's saturation check on
        // kp*err + newIntegral, with C3=0 and D~=kp*err for Kd=0).
        float u = gains.kp * err + newIntegral;
        float tw = (gains.kaw > 0.0f) ? (1.0f / gains.kaw) : 1e6f;
        float cW = (tw > dt) ? (dt / tw) : 1.0f;
        if (u > gains.i_max) {
            newIntegral += cW * (gains.i_max - u);
        } else if (u < -gains.i_max) {
            newIntegral += cW * (-gains.i_max - u);
        }
        integral_ = newIntegral;
    }
    // else: frozen — integral_ left unchanged (see file-level comment on
    // the deliberate divergence from source_old's ReInit() call here, and
    // the reset-on-entry comment above for the one behavior change from
    // that original freeze semantics).

    return output;
}

}  // namespace Hal
