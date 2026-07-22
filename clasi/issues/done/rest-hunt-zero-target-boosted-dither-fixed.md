---
status: done
---

# Rest-hunt "clicking": exact-zero-target dither from the sprint-114 deadband boost, exposed by the Pilot deletion — FIXED

## Description (as reported)

2026-07-22, stakeholder live report during a bench-fix session (SET
routing / `set_config_binary()` / completion-ack work): after a Move
completed, the robot audibly CLICKED at rest — the wheel oscillating one
duty cycle forward/back continuously. Telemetry corroborated it earlier
in the same session: 20s of idle telemetry (before any fix) showed one
wheel's encoder position drifting -12mm while its reported velocity
alternated sign (~±10-14mm/s) the ENTIRE time, despite the commanded
target being exactly zero. `STOP` does not silence it, because `STOP`
itself commands the same zero-velocity target that triggers the bug.

## Root cause

This is the deadband-dead-zone lineage's next chapter
(`deadband-dead-zone-and-boost-fix` memory note; sprint 114 ticket 005,
`deadband-compensation-small-commands-must-produce-real-motion.md`):
`writeShapedDuty()` (`nezha_motor.cpp`) boosts any genuine nonzero-but-
sub-deadband duty up to `±outputDeadband_` so small terminal corrections
still move the wheel. That boost was validated (sprint 114's own
`scenarioDeadbandBoostSettlesNotHuntsAcrossResidualSweep`,
`devices_motor_harness.cpp`) against `App::Pilot`'s old error-driven
target shape — a target that SHRINKS toward zero as real motion closes
the residual, never sitting at a literal `0.0f` for long. `App::Pilot`
was deleted wholesale by 115-003 (gut-to-minimal-firmware S1 motion-stack
excision). The current `MoveQueue`-driven architecture instead holds an
EXACT `0.0f` target indefinitely once a Move ends (`Drive::stop()`/an
emptied queue) — a regime the sprint-114 validation never covered.

At that exact-zero target, `MotorVelocityPid::compute()` still computed
an active `kp * err` proportional term against whatever `measured`
residual/noisy velocity the plant happened to report — pure estimator
noise, not a real correction — and `writeShapedDuty()`'s boost then
lifted that noise-signed output up to the full deadband duty, in
whichever direction the noise landed on, every tick it flipped sign:
the clicking.

## Fix

`src/firm/devices/velocity_pid.cpp`, `MotorVelocityPid::compute()`: when
`target == 0.0f` **and** `fabsf(measured)` is already within a rest-noise
floor (`kZeroTargetRestNoiseFloor = 15mm/s`, or `velDeadband` if a future
boot-config fix ever makes that live and larger — see that constant's own
comment for why `velDeadband` alone is not it today: `gen_boot_config.py`
never populates `msg::MotorConfig.min_duty`, so it is always `0.0f` in
practice), return a hard `0.0f` before ever computing `err`/`kp * err` —
matching `writeShapedDuty()`'s own existing "duty == 0.0f EXACTLY... NOT
boosted" contract one layer up.

**First cut (same session) gated on `target == 0.0f` alone** — caught by
a second stakeholder live report minutes later, itself confirmed by two
NEW sim regressions the first cut introduced: STOP-convergence from
~500mm/s measurably slower to cross a 5mm/s tolerance, and
`test_move_protocol`'s own `SUC-050` angle-stop tolerance missed by
0.4%. Root cause of the regression: the P4 `Move` model is bang-bang (full
commanded velocity until the stop condition fires, then `target` snaps
directly to `0.0f` — no deceleration ramp of its own), so an unconditional
`target == 0.0f` gate ALSO killed the P-term's real, wanted active
braking the instant a Move ended while the wheel was still genuinely
moving fast. Requiring `measured` to ALSO already be near rest fixes
this: real deceleration from speed keeps full active braking all the way
down to the noise floor; only the last, noise-dominated tail below it is
hard-zeroed instead of dithered.

The small-but-*nonzero*-target boost path sprint 114 ticket 005 added is
UNTOUCHED — this exemption only ever fires for a literal `target ==
0.0f`, never a small nonzero commanded velocity.

## Verification

- New/updated unit scenarios, `devices_motor_harness.cpp`:
  - `scenarioExactZeroTargetIgnoresResidualMeasuredVelocityNoise` — exact-
    zero target + alternating small residual noise (±4mm/s, below the
    rest floor) → `appliedDuty()` stays exactly `0.0f` every tick.
  - `scenarioExactZeroTargetStillBrakesWhileMeasuredIsLarge` (added in
    the refinement) — exact-zero target + a still-large measured velocity
    (300mm/s, a plausible mid-deceleration reading) → the PID still
    produces a real, negative (actively-braking) duty, not `0.0f`.
  - `scenarioDeadbandBoostSettlesNotHuntsAcrossResidualSweep` (sprint 114,
    pre-existing) still passes unchanged — the nonzero-target boost path
    is provably intact.
- Sim system tests (previously-regressed by the first cut, now clean):
  `test_move_protocol.py::test_move_protocol_scenarios_pass`,
  `test_scripted_twist_demo.py::test_scripted_twist_demo_compiles_and_tells_the_story`.
- Real hardware (`tovez`, `/dev/cu.usbmodem2121102`), reflashed with the
  refined fix, verified THREE separate times:
  1. 20s idle immediately post-flash: encoder delta `(0, 0)`, velocity
     `stdev=0.0` both wheels (vs. the pre-fix 20s capture: L drifted
     -12mm, velocity alternating sign up to ±10-14mm/s).
  2. 12s idle, fresh check in direct response to the stakeholder's live
     report: `0/184` frames with nonzero velocity, `0mm` drift.
  3. Immediately after real driving (two 300mm distance-stop Moves + a
     2000ms time-stop Move, all completing cleanly, no
     `fault_move_timeout`, no terminal stall): a brief transient
     nonzero-velocity tail in the telemetry (estimator settling, ~a few
     seconds), decaying to a clean `0/N` within the next 5s window, with
     encoder position held flat (`delta=(0,0)`) throughout — the
     documented "noise floor, not real creep" signature, not a
     reappearance of the bug.
  Normal move termination unaffected: 300mm distance stops landed at
  +317.0mm/+319.0mm (+5.7%/+6.3%, consistent with the pre-existing, known
  ~20mm actuation-lag tail); the 2000ms time-stop landed at +2056.5ms
  elapsed-from-first-motion (+2.8%, well within the ±100ms acceptance
  band) and traveled 328mm.

## Related

- `.clasi/knowledge`/memory: `deadband-dead-zone-and-boost-fix` (sprint
  114's own original fix and rationale).
- `clasi/sprints/114-.../issues/deadband-compensation-small-commands-must-produce-real-motion.md`
  (sprint 114 ticket 005 — the boost this fix narrows, not removes).
- `src/firm/devices/DESIGN.md` §4 — updated with the full before/refine
  narrative.
