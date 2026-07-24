---
status: pending
filed: 2026-07-24
filed_by: team-lead (stakeholder restructuring directive)
related:
- extract-motion-library-to-src-motion.md
- bench-move-commands-intermittently-never-reach-firmware.md
- bench-accuracy-campaign-s3.md
tickets: []
---

# Firmware base hardening: bounded wheel moves + per-wheel command observer, gated and frozen

## Stakeholder directive (2026-07-24)

This repo's focus is now the FIRMWARE BASE: the layer that takes the most
basic motion command — left/right wheel speeds — runs it to completion
properly, tracks those speeds well, and reports truthfully (wheel state +
raw sensors). Harden it behind a numeric gate, then FREEZE it: the base
becomes the thing we know we will not have to change while the motion
library (separate branch/worktree/repo) builds on top.

## The base contract

1. **Command primitive: bounded wheel moves.** `MoveWheels(v_left, v_right)`
   + stop condition (distance | time) + required timeout — already on the
   wire. "Run to completion properly" and the host-silence safety backstop
   live HERE, in the frozen layer. (Stakeholder-confirmed scope: bounded,
   not raw streaming.)
2. **Per-wheel command observer (the feed-forward insight).** We know what
   we commanded and when; encoder samples arrive slowly (per 40 ms cycle,
   slower effective on hardware I2C). Stop ignoring the command: propagate a
   characterized per-wheel response model (dead time, rise shape, deadband
   floor) as the EXPECTED wheel state between samples, and CORRECT the
   estimate on every real encoder sample. Predict-correct, not
   predict-and-act-blind — the distinction between an observer and the three
   failed open-loop predictors (stop_lead, margins, analytic coast): model
   error here survives at most one encoder interval before a measurement
   trims it. The observer's per-wheel (position, velocity, estimate age)
   becomes the wheel state the boundary hands upward, alongside the raw
   encoder values (both reported; consumers choose).
3. **Truthful telemetry.** Same-generation L/R pairs (121-005's fix stands
   as a base invariant with a test), commanded targets AND observed AND raw
   encoder state all visible per frame, sensors passed through raw. The
   base never lies and never hides latency.

## Characterization (constants with derivations — never swept)

- Sim: the plant is first-order by construction — the observer must track it
  EXACTLY (validates machinery; any residual is a bug, not tuning).
- Bench: per-wheel step-response battery (both directions, several speeds,
  loaded on the stand): dead time, effective tau, deadband floor,
  reversal-dwell effect; recorded per-robot in the JSON with measurement
  provenance. Sensitivity note vs battery voltage stated (observer's
  correction step absorbs slow drift; characterize, don't chase).
- Every constant names its measurement; the no-sweep rule applies in full.

## The base gate (numbers, then freeze)

- **Tracking:** commanded step ±200 mm/s — observed and encoder-measured
  velocity settle within a stated band/time (numbers set by the first
  characterization run, then ratcheted).
- **Observer fidelity:** sim — exact (≤0.1 mm / ≤0.1 mm/s vs plant truth);
  bench — stated band between observer estimate and encoder truth at the
  next sample (e.g. ≤5 mm/s during cruise, ≤ one dead-time's worth during
  transients), measured across the step battery.
- **Completion:** bounded wheel moves stop within one cycle + coast of the
  stop condition; timeout backstop fires exactly when due.
- **Telemetry:** pairing test, rate test (~25 Hz sustained), gap-free seq.
- Gate green ⇒ base is FROZEN: subsequent base changes require a
  stakeholder-signed issue; the motion library treats the boundary + gate
  numbers as a stable platform contract.

## Explicitly out of scope (motion library's job, other repo/branch)

Twist semantics, kinematics, odometry/pose, estimator/OTOS fusion, shaping,
chain hand-off, settle completion, heading hold, tours. The base does not
know the robot has a body — only two wheels, a clock, sensors, and a wire.

## Sequencing

After (or interleaved with) the extraction issue — the boundary must exist
for the observer's outputs to have a consumer shape. Bench halves of the
gate ride the existing bench-session cadence (transport reliability issue
first, same stand time). This issue plus extraction replace the previous
122+ exactness sequence AS THIS REPO'S PLAN; the exactness sequence itself
(settle completion, heading hold, fusion, tours, S-bars) transfers to the
motion library's plan, unchanged in substance, executed against
`motion_tests` first.

## REVISION (stakeholder decision, 2026-07-24): PID moves to motion; base primitive is DUTY

The rate argument settles it: the PID cannot update faster than the loop
(encoder freshness ~80 ms bounds it) wherever it lives, so motor residency
buys nothing. Revised base contract, superseding item 1 above:

- **Command primitive: per-wheel DUTY** (`[-1,1]`), one visible write per
  wheel per cycle. The velocity PID, the kff mapping, and therefore the
  bounded WHEEL-SPEED move (`MoveWheels` + stop conditions + timeout) move
  UP into the motion library's lowest tier — a speed-stop cannot be tracked
  without the PID.
- **Safety: zero-on-silence** — a cycle that hands the motor no duty writes
  zero — plus the plausibility clamp (|duty| ≤ 1, NaN → 0). Simpler and
  stronger than a speed-tracking rump; the wire-level Move bounding
  (timeout backstop) rides with the motion library, which is statically
  linked in the same image.
- **The observer stays in the base** (per-wheel, hardware-characterized
  truth reporting), now consuming commanded duty + encoder samples — its
  model is duty→velocity directly, matching both the sim `WheelPlant` and
  the bench step-response characterization 1:1.
- **`appliedDuty` feedback:** the dwell/deadband shaping remains in the base
  (brick protection); the actually-written duty is reported in the per-wheel
  sample so the motion-side PID's anti-windup sees actuator truth.
- `NezhaMotor` target size drops again: protocol + dwell/deadband + clamp,
  ~200 lines; `MotorVelocityPid` relocates to the motion library with its
  gains as motion config.

Base gate items update accordingly: "tracking" moves to the motion library's
gate; the base gate keeps observer fidelity, completion of the duty write
path (shaping visible, applied duty reported), telemetry truthfulness, and
zero-on-silence verified.
