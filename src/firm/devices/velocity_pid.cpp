#include "devices/velocity_pid.h"

#include <cmath>

namespace Devices {

namespace {
float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// Rest-noise floor for the exact-zero-target exemption below (2026-07-22
// bench fix, refined same day after a stakeholder live report caught the
// first cut's own regression -- see compute()'s own comment at the
// exemption's use site for the full incident). `measured` must be within
// this magnitude of zero, IN ADDITION to `target == 0.0f`, before the
// exemption applies -- gating on `target == 0.0f` ALONE (the first cut)
// also suppressed the P-term's active braking while STILL DECELERATING
// FROM REAL SPEED the instant a Move's target snaps to zero (Drive::
// stop()/an emptied MoveQueue give NO deceleration ramp of their own --
// the bang-bang P4 Move model runs at full commanded velocity until its
// stop condition fires, then target goes directly to 0.0f), which is a
// real, wanted correction, not noise -- confirmed by two sim regressions
// the first cut introduced (STOP-convergence taking measurably longer to
// cross 5mm/s from ~500mm/s; SUC-050's own angle-stop tolerance missed by
// 0.4%). `velDeadband` itself is NOT that threshold on real hardware --
// `gen_boot_config.py` never populates `msg::MotorConfig.min_duty` (the
// wire field `main.cpp:59` copies into runtime `velDeadband`), so it is
// always `0.0f` in practice (see device_config.h's own default) -- this
// constant is the floor's real, always-live value. 15mm/s matches the
// bench's own observed at-rest noise envelope (2026-07-22: idle velocity
// readings alternating up to ~+-14mm/s with zero net position change) and
// the pre-existing, if not-yet-wired, `tovez_nocal.json`
// `drive.motor_deadband=15.0` a human already picked for this exact
// concept. `velDeadband` still wins if a future boot-config fix ever
// makes it larger than this floor (`fmaxf` below) -- this constant is a
// floor, not a ceiling override.
constexpr float kZeroTargetRestNoiseFloor = 15.0f;  // [mm/s]
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

    // Exact-zero target AND already-near-rest measured velocity: a genuine
    // "settled at a stop" state (Drive::stop()'s own v_x_=0.0f/omega_=0.0f,
    // or a completed Move's queue drain, once real motion has actually
    // died down), not a small nonzero setpoint the plant can't quite
    // produce, and NOT a still-in-flight deceleration from real speed.
    // `measured` here is whatever RESIDUAL velocity the plant reports at
    // that instant. Bench fix, 2026-07-22 (stakeholder finding): without
    // SOME exemption, the P-term below (`kp * err`) fires off pure
    // noise once truly at rest, and writeShapedDuty()'s deadband boost
    // (nezha_motor.cpp) then lifts whatever tiny, noise-signed output
    // results up to the FULL outputDeadband_ magnitude -- a real (if
    // small), alternating-sign duty burst every tick the noise flips
    // sign ("clicking" at rest). Confirmed on the bench: 20s idle
    // telemetry showed one wheel's encoder position drifting -12mm while
    // its reported velocity alternated sign (~+-10mm/s) the entire time
    // target was 0.
    //
    // REFINED same day after a stakeholder live report caught the first
    // cut's own regression: gating on `target == 0.0f` ALONE also
    // suppressed active braking the INSTANT a Move's target snaps to zero
    // while the wheel is still genuinely moving fast (the P4 Move model
    // is bang-bang -- full commanded velocity until the stop condition
    // fires, then target goes directly to 0.0f, no deceleration ramp of
    // its own) -- confirmed by two sim regressions: STOP-convergence from
    // ~500mm/s measurably slower to cross 5mm/s, and SUC-050's own
    // angle-stop tolerance missed by 0.4%, plausibly the same lost-
    // braking tail. Requiring `fabsf(measured)` to ALSO already be near
    // zero (within `kZeroTargetRestNoiseFloor`, or `velDeadband` if a
    // future boot-config fix ever makes that live and larger -- see
    // that constant's own comment) restricts the exemption to the
    // genuinely-at-rest case: real deceleration from speed keeps its full
    // active P-term braking all the way down to the noise floor, and only
    // the LAST, noise-dominated tail below it gets hard-zeroed instead of
    // dithered. A hard 0.0f there matches writeShapedDuty()'s OWN "duty ==
    // 0.0f EXACTLY... NOT boosted" contract one layer up. Distinct from
    // the (still fully intact) small-NONZERO-target boost path
    // (scenarioDeadbandBoostSettlesNotHuntsAcrossResidualSweep,
    // devices_motor_harness.cpp) that sprint 114 ticket 005 added: this
    // exemption fires ONLY when the caller's own target is a literal
    // 0.0f, never for a small-but-nonzero commanded velocity.
    float restThreshold = (velDeadband > kZeroTargetRestNoiseFloor)
                               ? velDeadband : kZeroTargetRestNoiseFloor;
    restGateEngaged_ = (target == 0.0f && fabsf(measured) <= restThreshold);
    if (restGateEngaged_) {
        return 0.0f;
    }

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
