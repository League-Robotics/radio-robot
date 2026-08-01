---
id: '004'
title: 'Wheel-speed controller algorithm: conversion map + fast PID + bias trim'
status: open
use-cases: [SUC-001]
depends-on: ['001', '003']
github-issue: ''
issue:
- wheel-speed-controller-moves-into-drive.md
- 04-continuous-duty-per-speed-calibration.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Wheel-speed controller algorithm: conversion map + fast PID + bias trim

## Description

Implement the three-timescale controller in `App::Drive` per
`wheel-speed-controller-moves-into-drive.md` Phase 2 (Stage A
conversion map, Stage B fast PID, Stage C bias trim), consuming ticket
001's population defaults/bounds and ticket 003's extended interface.

**This ticket supersedes `04-continuous-duty-per-speed-calibration.md`'s
gain-scaling adaptation design** (sprint Architecture Design Rationale
Decision 3) — implement Stage C's bias/intercept adaptation instead,
not issue 04's `dutyPerSpeed`-scaling approach. Fold in issue 04's two
valuable provisions: its safety intent (achieved here via `bias`
clamped to `±biasMax`, `tauAdapt`-slow update, reset on `estop()` —
a stronger bumpless-by-construction property than 04's own bleed-the-
integral formula, since the SLOPE never adapts, only the additive
intercept) and note that its observability mandate is completed in
ticket 005 (telemetry/TestGUI), not here.

Resolve, as implementation-time decisions documented in this ticket's
own notes:
- **Open Question 1**: `bias` per-wheel vs. per-wheel-per-direction.
- **Open Question 2**: speed-floor policy — round a sub-`vMin` command
  up to `vMin`, or refuse it outright.
- **Open Question 4**: whether the fast PID needs a nonzero `kp` at all
  given the profiler's every-tick re-plan-from-measured-position, or
  whether that already supplies equivalent correction at the planning
  layer.

Apply ticket 001's power-ceiling verdict: if a hard simultaneous-wheel
current-limit ceiling was confirmed (not a session/battery artifact),
`biasMax`/`pidMax` must be derated so the controller never advertises
recovery authority the power budget cannot actually deliver (sprint
Architecture Design Rationale Decision 5).

## Acceptance Criteria

- [ ] Stage A: `v_corrected = gain[dir]*v_cmd + intercept[dir] + bias`,
      `dutyFF = dutyPerSpeed * v_corrected`, `gain`/`intercept` sourced
      from ticket 001's population defaults with per-robot override.
- [ ] Stage B: fast PID (`p + i + ff`, clamped `±pidMax`) with the
      integrator frozen outside steady state (`|a_cmd| < aSteady`) and
      reset on `estop()`.
- [ ] Stage C: `bias` adapts only when steady (`|a_cmd| < aSteady`,
      `|v_cmd| >= vMin`, fresh sample), clamped to `±biasMax` — derated
      per ticket 001's power-ceiling verdict if a hard ceiling was found.
- [ ] Speed-floor and deficit-flag policy implemented and documented
      (Open Question 2 resolved with stated rationale).
- [ ] Unit tests: convergence under a known plant-gain error; the
      `biasMax` clamp holds against a divergent error; duty is
      continuous (bumpless) across a `bias` update.
- [ ] `04-continuous-duty-per-speed-calibration.md` explicitly noted as
      superseded by this ticket's Stage C in the ticket's own notes and
      commit message — no parallel gain-scaling implementation exists.

## Testing

- **Existing tests to run**: existing `app_drive` unit tests (must
  still pass with all-zero gains producing zero correction, matching
  `WheelTrim`'s original fail-closed guarantee).
- **New tests to write**: per-stage convergence/clamp/continuity unit
  tests (host build); a sim scenario with deliberately mismatched L/R
  plant gains converging to two different `bias` values and driving
  straight (the standing SIM-equals-bench case that has failed on the
  bench before).
- **Verification command**: `uv run pytest`

## Implementation Plan

**Approach**: implement Stage A/B/C as clearly separated, independently
testable methods/fields inside `Drive` — mitigating the god-component
concern flagged in the sprint's Architecture Design Rationale ("Drive's
growing size is deliberate... Stage A/B/C stay separately named,
separately unit-tested"). No new class introduced (per sprint 128's
own reasoning against a third wheel-control generation coexisting).

**Files to create/modify**:
- `src/firm/app/drive.{h,cpp}` (Stage A/B/C implementation)
- `data/robots/{tovez,togov,tovez_nocal}.json` (`biasMax`, `vMin`,
  `tauAdapt`, `aSteady`, `kp`, `ki`, `iMax`, `kaff`, `pidMax`,
  `deficitThreshold`, `deficitWindow` keys)
- `robot_config.schema.json`, `robot_config.py`, `gen_boot_config.py`
  (schema + codegen for the new keys)

**Testing plan**: per-stage unit tests (host build, no hardware
required); the mismatched-L/R-gain sim scenario above.

**Documentation updates**: `drive.h`'s header comment rewrite — deletes
"there is no controller here" and the now-false "closed-loop control
lives in Motion::Planner's own duty stage" line.
