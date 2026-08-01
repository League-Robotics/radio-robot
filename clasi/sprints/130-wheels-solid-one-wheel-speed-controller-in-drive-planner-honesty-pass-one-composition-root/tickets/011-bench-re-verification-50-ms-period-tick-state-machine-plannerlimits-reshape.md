---
id: '011'
title: 'Bench re-verification: 50 ms period, tick() state machine, PlannerLimits reshape'
status: open
use-cases: [SUC-004]
depends-on: ['007', '008', '009']
github-issue: ''
issue: planner-honesty-pass-50ms-period-tick-state-machine-limits-reduction.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Bench re-verification: 50 ms period, tick() state machine, PlannerLimits reshape

## Description

On-stand re-verification of the combined 50 ms-period + `tick()`-
state-machine + `PlannerLimits`-reshape changes (tickets 007-009), per
`planner-honesty-pass-50ms-period-tick-state-machine-limits-
reduction.md`'s own Verification section. Re-verify tuning at the new
dt: trim gains (already relocated to `Drive` by ticket 005) and
`decelPlanFraction` were tuned at ~47 ms — confirm behavior at 50 ms
before re-blessing any tuned constants.

## Acceptance Criteria

- [ ] `cycle_period` telemetry reads 50 ms ± jitter, stable under load,
      on the bench.
- [ ] Bench square tour closure at 50 ms is at least as good as the
      47 ms baseline before re-blessing tuned constants (goldens
      re-blessed with a stated why, per the golden process).
- [ ] `SET pid.kp` over the wire either visibly tunes the controller or
      returns an error (re-confirm ticket 005/007's wire-key repoint
      survives the 50 ms/state-machine changes intact).
- [ ] The offset guard (ticket 009) passes on the regenerated ctypes
      mirror, confirmed against THIS bench run's harness build (not
      just at implementation time).

## Testing

- **Existing tests to run**: full `planner_tests` ctest suite as a
  pre-check; bench square tour.
- **New tests to write**: none expected — this is a verification
  ticket; fix-forward if the re-verification finds a regression.
- **Verification command**: on-stand bench run per
  `.claude/rules/hardware-bench-testing.md`.

## Implementation Plan

**Approach**: bench/HITL verification, no new production logic expected
unless the re-verification surfaces a regression from the 50 ms/
state-machine/`PlannerLimits` changes, in which case fix forward within
this ticket.

**Files to create/modify**:
- Bench scripts (existing square-tour/telemetry-capture tools)
- Golden data files (re-blessed with a dated, justified entry if tuned
  constants need adjustment at 50 ms)

**Testing plan**: on the stand, per `.claude/rules/hardware-bench-
testing.md`.

**Documentation updates**: golden re-bless entries with dated
rationale, per the project's golden-blessing convention.
