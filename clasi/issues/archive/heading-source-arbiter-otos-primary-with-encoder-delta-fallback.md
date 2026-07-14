---
status: obsolete
---

> **OBSOLETE (2026-07-14 stakeholder triage).** Superseded by the single-loop
> firmware rebuild (`clasi/issues/single-loop-firmware-de-fiber-delete-the-elite-plumbing-telemetry-only-return-path.md`;
> review: `docs/code_review/2026-07-13-devices-drive-review.md`). Heading arbitration moves host-side with pose fusion (robot reports raw encoder odometry + raw OTOS in telemetry; host arbitrates). Carried as host-planner-design-lessons-from-drive-v2-review.md item 10.

# Heading-source arbiter: OTOS-primary heading with rebased encoder-delta fallback (gated switch, not blend)

## Summary

Stakeholder-proposed improvement (2026-07-12) to the fused-heading channel:
replace sprint 099 D4's *EKF-blend-with-innovation-gate* for OTOS heading with
a **source arbiter** — a gated switch that uses the OTOS heading verbatim when
its per-interval change is plausible, and falls back to encoder heading
*deltas rebased onto the last trusted OTOS heading* when it is not. The
position channel keeps D4's Mahalanobis gate unchanged; this issue is about
heading only.

Motivated by the WPILib comparison study
([docs/design/wpilib-motion-stack-comparison.md](../../docs/design/wpilib-motion-stack-comparison.md),
Lesson L6): WPILib's odometry *discards* encoder-derived Δθ and takes the
absolute heading sensor's value each step (Appendix C.1 — "gyro overrides
kinematic dtheta"; heading is snapped, never integrated), and its estimator's
recommended configuration never lets vision correct heading at all (per-axis
gain `k = 0` when `q = 0`, Appendix C.2). The ecosystem norm is
**single-source heading with a software offset, not a blend**. The OTOS,
unlike an FRC gyro, has glitch failure modes (tracking loss, re-lock jumps),
so the adaptation is that pattern *plus a plausibility gate* — which is
exactly this proposal.

## The mechanism (stakeholder's design)

Per OTOS sample interval:

1. Compute the OTOS heading change `dThetaOtos` over the interval, and the
   encoder-derived heading change `dThetaEnc` over the **same timestamped
   interval**.
2. Two plausibility checks on the OTOS delta (both are *delta* checks —
   deliberately, see Design point 2):
   - **Rate limit** — is `dThetaOtos` physically plausible against the
     previous heading (|dThetaOtos| within commanded/possible yaw rate for
     the interval, plus margin)? Catches teleport-style glitches regardless
     of what the encoders say.
   - **Cross-sensor agreement** — is `dThetaOtos` plausible against
     `dThetaEnc`? Catches in-rate-limit slew glitches (OTOS drifting while
     the wheels say the robot is not turning), and automatically catches the
     frozen-but-connected OTOS (delta ≈ 0 during a commanded turn).
3. **Plausible → use the OTOS heading** (sensor-verbatim; no filter lag).
4. **Implausible → advance the fused heading by `dThetaEnc`** from the last
   trusted value.

The load-bearing detail: the encoder channel is a **delta provider
continuously rebased to the last trusted OTOS heading**, never a free-running
absolute series. At glitch time the fallback continues from an OTOS-grade
anchor and accumulates encoder-grade drift only for the duration of the
outage. (Same insight 099 already uses for the camera-fix ring — the clean,
never-corrected encoder series is valuable because its *deltas* are
composable — and structurally WPILib's `VisionUpdate.compensate` overlay
applied to heading.)

The underlying error-independence assumption: encoder heading errors (wheel
slip, trackwidth error) are uncorrelated with OTOS optical/IMU glitches, so
two independent checks can vote. See Design point 3 for where this assumption
is weakest.

## Design points required for completeness

These came out of the design discussion and must be resolved by whichever
sprint adopts this:

1. **Interval alignment.** OTOS reads land on the I2C flip-flop schedule
   (one read per `kReadPeriod`, only in a `REQUEST_DUE` slot — 099 D2), so
   `dThetaOtos` spans a variable number of loop passes. The encoder delta it
   is judged against must span the *same* timestamped interval, or the
   agreement check false-rejects at cadence boundaries.

2. **Re-acquisition rule (lockout prevention).** If the OTOS re-locks after a
   glitch with a persistent absolute offset, its deltas become plausible again
   immediately but its absolute heading disagrees forever. Trust checks stay
   on deltas; the absolute is reconciled by either:
   - (a) folding the accumulated offset into a software offset — WPILib's
     `m_gyroOffset` recompute pattern (comparison doc, Appendix C.1):
     corrections never touch the sensor, they adjust an offset; or
   - (b) taking the step and letting it flow through `bb.poseStepped`, where
     the motion-v2 stack already has a defined reaction (≤3° absorbed by
     trims, larger → `REPLAN_DUE`). Attractive because the machinery already
     exists (sprint 100 M5) and the step is honest rather than hidden.

3. **Slip stresses the independence assumption exactly during hard
   maneuvers.** Aggressive pivots cause wheel slip — then the *encoder* delta
   is the wrong one, and a flat agreement tolerance would veto a correct OTOS
   reading in favor of slipping encoders. Mitigations:
   - rate-schedule the agreement tolerance with commanded |omega| (same shape
     as sprint 100's replan envelopes);
   - asymmetric switching — fall back instantly, return to OTOS only after N
     consecutive plausible samples (099 D4's rejection-streak concept,
     inverted into a re-trust dwell). The asymmetry also prevents source
     flapping, which matters because the heading outer loop (kθ = 6) would
     amplify jitter from a chattering arbiter.
   - Default polarity is principled: one interval of in-rate disagreement
     cannot attribute blame, but encoder heading error is *bounded* by
     commanded dynamics while an OTOS glitch is unbounded — when you cannot
     tell, the bounded-error source is the safe choice.

4. **Not validatable on the stand.** On the stand the situation inverts:
   encoders report rotation, OTOS correctly reports none, so the arbiter
   always "falls back" to the physically-wrong encoder deltas — which is the
   historically expected bench behavior (nothing breaks), but the arbiter's
   real behavior only shows on the floor/playfield against camera truth.

5. **Observability.** The arbiter's decision must be visible: at minimum a
   current-source flag and a fallback/rejection counter (TLM budget
   permitting, or via the existing pose/validity cells), so bench and field
   runs can show *when* and *why* the source switched.

## Relation to existing artifacts

- **Sprint 099 (planned, NOT yet executed):** this revises the heading half
  of D4 (`EkfTiny` innovation gate) into a source arbiter ahead of — or in
  place of — the EKF's `updateHeading()` path. The position channel keeps the
  Mahalanobis gate. **Decide before 099's tickets execute** — cheap now,
  expensive after.
- **Sprint 100 M11 (bench acceptance):** the 098 pivot-grid re-run already
  gates "the encoder→EKF heading-source switch." That re-run is the natural
  A/B experiment: EKF-blend vs. this arbiter, judged against camera heading
  truth. Ticket 099-009's aprilcam end-to-end script provides the camera
  side.
- **Sprint 100 M5 (policy):** the pose-step absorb/replan thresholds are the
  ready-made consumer for re-acquisition option (b).
- **WPILib evidence:** comparison doc §5 L6 (the lesson this answers),
  Appendix C.1 (gyro-override odometry + `m_gyroOffset` reset pattern),
  Appendix C.2 (estimator overlay + per-axis trust, "make the vision heading
  standard deviation very large").
- **Project knowledge:** `.clasi/knowledge` OTOS entries (per-pass I2C tick
  incident 098-004; OTOS heading distinguishes stand from floor;
  `conn=`/`fusableThisPass()` discipline) all constrain the implementation.

## Acceptance sketch (for the adopting sprint)

- Tier-0/unit: arbiter truth table (plausible/implausible × rate/agreement),
  wrap-around cases, interval-alignment cases, re-acquisition after synthetic
  re-lock offset, no flapping under noise (dwell honored).
- Sim: fault-knob heading glitches (teleport, slew, freeze) → fused heading
  follows the arbiter spec; encoder-slip case → OTOS retained per
  rate-scheduled tolerance.
- Field: camera-verified heading during pivots and chained runs, with at
  least one induced OTOS occlusion/glitch; fused heading error vs. camera
  truth bounded through the outage and after re-acquisition.
