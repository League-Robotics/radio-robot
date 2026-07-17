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
// Embedded velocity PID — a discrete PI (+ feedforward) with back-
// calculation anti-windup, implemented directly rather than via a general
// transfer-function PID library (this reduced form is exactly what such a
// library collapses to for Kd=0 and a near-zero filter time constant, so
// nothing is lost by writing it directly — see DESIGN.md §4).
//
// The deadband (integrator-freeze) branch freezes the integrator exactly
// unchanged while |target| stays in the deadband, and resets it to zero on
// the tick the deadband is first entered (edge-triggered) — see DESIGN.md
// §4 for why the reset-on-entry behavior matters.
//
// Output domain: duty fraction [-1, 1] (matching the write path's
// setDutyCycle-equivalent contract).
// ---------------------------------------------------------------------------
float MotorVelocityPid::compute(float target, float measured, float dt,
                                 const Gains& gains, float velDeadband)
{
    if (dt <= 0.0f) dt = kNominalDt;

    float spAbs = fabsf(target);

    // velDeadband is the integrator-freeze deadband threshold on |target|
    // (device_config.h's field comment covers why it is named this way,
    // not `minDuty`). <= (not <) so an exact target==0.0f still counts as
    // "in the deadband" even when velDeadband itself is 0.0 (unconfigured)
    // — the common case for a fresh boot config: a literal zero target
    // always means "come to a stop," independent of whether a stiction
    // floor has been tuned.
    bool inDeadband = spAbs <= velDeadband;

    // A plain FREEZE (leave integral_ at whatever it held) preserves
    // whatever bias the integrator built up sustaining the PRIOR, unrelated
    // motion (e.g. a fast turn) straight into the new near-zero-target
    // regime. That carried-over bias — combined with a fresh, still-large
    // kp*err — is what produces an oversized, wrong-signed correction once
    // the wheel coasts past zero: not because the fresh correction is
    // undamped, but because it is riding on top of a stale one. Resetting
    // the integrator on the tick the deadband is FIRST entered
    // (edge-triggered on wasInDeadband_, not level-held — a continuing
    // low/zero target keeps freezing exactly as before, so a genuine
    // bench-tuned low-speed creep still gets zero ongoing integral action)
    // clears that stale bias before it can leak into the stop, while never
    // touching the armor's own reversal-dwell gate (an unrequested
    // full-scale reversal never passes through this deadband at all, since
    // |target| stays large on both sides of that flip).
    if (inDeadband && !wasInDeadband_) {
        integral_ = 0.0f;
    }
    wasInDeadband_ = inDeadband;

    float err = target - measured;
    float spSign = (target >= 0.0f) ? 1.0f : -1.0f;
    float ff = gains.kff * spAbs;

    // Output uses the OLD integrator (pre-update, but post-reset above).
    float iOld = integral_;
    float rawDuty = spSign * ff + gains.kp * err + iOld;
    float output = clampf(rawDuty, -1.0f, 1.0f);

    if (!inDeadband) {
        float newIntegral = iOld + gains.ki * dt * err;
        // Anti-windup back-calculation against +/- iMax.
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
    // else: frozen — integral_ left unchanged (see reset-on-entry comment
    // above).

    return output;
}

}  // namespace Devices
