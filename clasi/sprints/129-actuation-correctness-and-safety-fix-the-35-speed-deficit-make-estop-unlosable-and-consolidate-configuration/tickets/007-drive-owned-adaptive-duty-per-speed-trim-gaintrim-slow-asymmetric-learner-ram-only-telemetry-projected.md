---
id: '007'
title: 'Drive-owned adaptive duty-per-speed trim (gainTrim): slow asymmetric learner,
  RAM-only, telemetry-projected'
status: open
use-cases: [SUC-007]
depends-on: ['006']
github-issue: ''
issue: 04-continuous-duty-per-speed-calibration.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Drive-owned adaptive duty-per-speed trim (gainTrim): slow asymmetric learner, RAM-only, telemetry-projected

## Description

**Design settled by the stakeholder 2026-08-01, revising the shape
planned during architecture review.** Depends on ticket 006 — this
learner corrects a residual around an already-accurate static
`dutyPerSpeed` baseline, not a still-wrong one (attempting this before
006 would adapt away from the 2.9x compounded error instead of a small
true residual).

**This is Drive's own calibration, not a `Motion::WheelTrim` instance and
not a motion-side calibrator.** The learner uses only `App::Drive`'s own
knowledge — its calibrated map, its commanded speed, and the encoder
measurement `RobotLoop` already has — so it belongs entirely inside
`App::Drive`. `Motion::WheelTrim` is untouched and keeps its one existing
call site in `Motion::Planner`; this ticket creates **zero** new
firm↔motion dependency. See `sprint.md`'s Design Rationale, Decision 1,
for the full three-stage decision trail.

**It is a calibration, not a control loop — timescale separation is the
core of the design:**

1. `App::RobotLoop` passes measured wheel speeds into
   `Drive::tick(speedLeft, speedRight, measuredLeft, measuredRight)` —
   plain floats, sourced from `RobotState`.
2. `App::Drive` gains private `gainTrimLeft_`/`gainTrimRight_` (`[1]`,
   dimensionless, boot-reset to 1.0). `duty = speed * dutyPerSpeed *
   gainTrim`.
3. Learning time constant τ ≈ 60 s of **driven** time (learn-eligible
   cycles only — `lambda ≈ 1/(60·loopHz)`), so a hand loading a wheel for
   a few seconds barely moves it — the fast loop
   (`Motion::WheelTrim`, unchanged) owns disturbance rejection on the
   managed path; this loop owns drift in the calibration itself.
4. **Asymmetric rates**: learn upward (more duty needed) at the slow τ;
   relax back toward 1.0 about 5x faster (loads only ever push duty up,
   so slow-up/fast-down sheds transient pollution without a symmetric
   learner's tendency to drift from noise).
5. **Learning gates** (all must hold to update): commanded speed above a
   minimum; measured speed not ≈0 (that's the wheel-frozen fault —
   ticket 002 — not a gain lesson); no command transient (steady-command
   gate); on the managed path, skip when `Motion::WheelTrim`'s own
   velocity trim is already large relative to command (a succeeding fast
   loop would otherwise be misread by the slow loop as a gain error).
6. **Clamp [0.8, 1.25]** — the wider [0.5, 2.0] planning-time contingency
   (for adapting against an unmeasured baseline) is moot given ticket
   006's sequencing; ship the tighter bound directly. Reset to 1.0 on
   `estop()`.
7. `gainTrimLeft()`/`gainTrimRight()` accessors; `RobotLoop` projects them
   into `RobotState`, published live in telemetry and the TestGUI.
8. **Persistence: RAM-only, always** — the firmware never writes flash
   (stakeholder: "we're not ready for the complexity of storing it").
   `gainTrim` boot-resets to 1.0 every time; carrying learning forward
   across boots is ticket 008's job (host-side bake), not this one's.
9. `drive.h`'s "there is no controller here" comment must be rewritten to
   describe the slow adaptive feedforward honestly — it is no longer
   accurate as written.

## Acceptance Criteria

- [ ] Unit: a known plant-gain error converges `gainTrim` toward truth at
      the slow rate; the [0.8, 1.25] clamp holds against a divergent
      error; duty is continuous across an update (no step).
- [ ] Sim: a straight leg with deliberately mismatched L/R plant gains
      converges to two different `gainTrim` values and drives straight
      (the exact case that failed repeatedly on the real bench) — per the
      standing SIM-equals-bench rule, demonstrate this in Sim before the
      bench trial.
- [ ] Bench, freewheeling: `gainTrim` converges toward the value
      `duty_sweep.py` (ticket 006) would independently measure.
- [ ] Bench, load test: physically grab/load one wheel for ~10 s on the
      stand — `gainTrim` moves negligibly during the load and relaxes
      back after release, proving the ~60 s time constant actually
      rejects a disturbance rather than chasing it.
- [ ] Bench, managed-path interplay: during a `Move`, `Motion::WheelTrim`
      (fast) and Drive's `gainTrim` (slow) both converge without
      oscillation or fighting — verify this on the stand, don't just
      assert it from the timescale-separation argument.
- [ ] `gainTrimLeft/Right` are visible live in the TestGUI.
- [ ] `grep -rn "WheelTrim" src/firm/app/drive.{h,cpp}` returns nothing —
      confirms no cross-tree dependency was introduced.

## Testing

- **Existing tests to run**: `app_drive_harness.cpp`, `motion_tests`,
  the planner `ctest` suite (confirm `Motion::WheelTrim`'s own tests are
  unaffected — it should show zero diff), firmware pytest tiers.
- **New tests to write**: unit tests for the learning-gate logic (each
  gate independently, and combined), the asymmetric-rate behavior, the
  clamp, and bumpless continuity across an update; a Sim test for the
  mismatched-L/R-gains convergence case.
- **Bench verification (required, two-part)**: freewheeling convergence
  and the physical load test, both per Acceptance Criteria — this ticket
  is not done on tests alone.
- **Verification command**: `uv run pytest`; firmware tiers per project
  standard.

## Implementation Plan

- **Approach**: implement entirely within `App::Drive` — no new class, no
  cross-tree call. `RobotLoop` is the only other file that needs new
  plumbing (measured-speed input, `gainTrim` output projection).
- **Files to modify**: `src/firm/app/drive.h`, `src/firm/app/drive.cpp`,
  `src/firm/app/robot_loop.cpp`, `src/firm/types/robot_state.h` (new
  telemetry-projected fields), `src/firm/app/telemetry.{h,cpp}` (publish
  `gainTrimLeft/Right`), TestGUI telemetry panel (display).
- **Documentation updates**: `drive.h`'s file-header "there is no
  controller here" claim (rewrite honestly — see point 9 above); note the
  timescale-separation rationale inline at the learner's rate constants,
  since that reasoning is exactly the kind of load-bearing comment issue
  01 wants kept, not deleted.
