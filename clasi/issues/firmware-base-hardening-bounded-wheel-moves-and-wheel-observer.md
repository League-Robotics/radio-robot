---
status: in-progress
filed: 2026-07-24
filed_by: team-lead (stakeholder restructuring directive)
related:
- extract-motion-library-to-src-motion.md
- bench-move-commands-intermittently-never-reach-firmware.md
- bench-accuracy-campaign-s3.md
tickets:
- 125-001
- 125-002
- 125-003
- 125-004
- 125-005
- 125-006
- 125-007
- 125-008
- 125-009
- 125-010
- 125-011
- 125-012
- 125-013
- 125-014
- 125-015
- 125-016
- 125-017
split_into:
- firmware-base-hardening-characterization-gate-and-freeze.md
sprint: '125'
---
## Stakeholder directive (2026-07-24)

This repo's focus is now the FIRMWARE BASE: the layer that takes the most
basic motion command — left/right wheel duty — runs it to completion
properly, tracks wheel state well, and reports truthfully (wheel state +
raw sensors). Harden it, then FREEZE it (gate/freeze criteria split out
to `firmware-base-hardening-characterization-gate-and-freeze.md`, sprint
125): the base becomes the thing we know we will not have to change while
the motion library (separate branch/worktree/repo) builds on top.

**Scope note (split 2026-07-24):** this issue now covers the base-CONTRACT
change only — the duty primitive, the per-wheel command observer, and the
`NezhaMotor` shrink (sprint 124). The characterization battery, the
numeric gate, and the freeze declaration are split off to
`firmware-base-hardening-characterization-gate-and-freeze.md` (sprint
125), which depends on this issue landing first.

## The base contract

1. **Command primitive: per-wheel DUTY** (`[-1,1]`), one visible write per
   wheel per cycle (see the REVISION below — this superseded an earlier
   bounded-wheel-speed-move contract). Safety: zero-on-silence (a cycle
   that hands the motor no duty writes zero) plus the plausibility clamp
   (`|duty| ≤ 1`, NaN → 0).
2. **Per-wheel command observer (the feed-forward insight).** We know what
   we commanded and when; encoder samples arrive slowly (per 40 ms cycle,
   slower effective on hardware I2C). Stop ignoring the command: propagate
   a characterized per-wheel response model (dead time, rise shape,
   deadband floor) as the EXPECTED wheel state between samples, and
   CORRECT the estimate on every real encoder sample. Predict-correct, not
   predict-and-act-blind — the distinction between an observer and the
   three failed open-loop predictors (stop_lead, margins, analytic coast):
   model error here survives at most one encoder interval before a
   measurement trims it. The observer's per-wheel (position, velocity,
   estimate age) becomes the wheel state the boundary hands upward,
   alongside the raw encoder values (both reported; consumers choose).
3. **Truthful telemetry.** Same-generation L/R pairs (121-005's fix stands
   as a base invariant with a test), commanded duty AND observed AND raw
   encoder state all visible per frame, sensors passed through raw. The
   base never lies and never hides latency.

## Explicitly out of scope (motion library's job, other repo/branch)

Twist semantics, kinematics, odometry/pose, estimator/OTOS fusion,
shaping, chain hand-off, settle completion, heading hold, tours,
`MoveWheels`/the velocity PID (moved to `src/motion` — see REVISION). The
base does not know the robot has a body — only two wheels, a clock,
sensors, and a wire.

## Sequencing

Second of the three sprints in this repo's firmware-base-hardening
restructuring: 123 (COBS+CRC framing) lands first — it frees the
telemetry headroom this sprint's wider per-wheel frame needs and adds the
integrity a duty-primitive rewrite deserves. This sprint (124) is the
base-contract change. Sprint 125 characterizes, gates, and freezes it
(split off, see `firmware-base-hardening-characterization-gate-and-freeze.md`).
Bench halves of the gate ride the existing bench-session cadence
(transport reliability issue first, same stand time).

## REVISION (stakeholder decision, 2026-07-24): PID moves to motion; base primitive is DUTY

The rate argument AS DECIDED: the PID cannot update faster than the loop
(encoder freshness, then believed ~80 ms, bounds it) wherever it lives, so
motor residency buys nothing. [CORRECTION 2026-07-26: that freshness
premise was measured FALSE — the register is live at ≤ 16 ms
(`docs/design/encoder-refresh-characterization.md`); the relocation
stands on its other grounds (one estimate → one controller, motion_tests
tunability, NezhaMotor doing one job).] Revised base contract,
superseding an earlier bounded-wheel-speed-move primitive:

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
- **`appliedDuty` feedback:** the dwell/deadband shaping remains in the
  base (brick protection); the actually-written duty is reported in the
  per-wheel sample so the motion-side PID's anti-windup sees actuator
  truth.
- `NezhaMotor` target size drops again: protocol + dwell/deadband + clamp,
  ~200 lines; `MotorVelocityPid` relocates to the motion library with its
  gains as motion config.

## Design reference

`docs/design/base-explicit-loop-sketch.md` — the explicit-dataflow loop
design (three principles, full `NezhaMotor` inventory with KEEP/MOVE/
DELETE verdicts, normative loop dataflow, boundary structs, resolved/open
questions). Primary design reference for this sprint's ticket work.
