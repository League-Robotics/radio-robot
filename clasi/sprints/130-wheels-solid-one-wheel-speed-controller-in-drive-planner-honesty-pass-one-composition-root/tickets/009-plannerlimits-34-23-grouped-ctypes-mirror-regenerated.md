---
id: 009
title: "PlannerLimits 34 \u2192 23, grouped, ctypes mirror regenerated"
status: open
use-cases: [SUC-004]
depends-on: ['008']
github-issue: ''
issue: planner-honesty-pass-50ms-period-tick-state-machine-limits-reduction.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# PlannerLimits 34 → 23, grouped, ctypes mirror regenerated

## Description

Cut `PlannerLimits` from 34 to 23 fields per
`planner-honesty-pass-50ms-period-tick-state-machine-limits-
reduction.md` item 3: remove `velKff`/`velKp`/`velKi`/`velIMax`/
`velKaff`/`velIAccelGate`/`dutyFloor` (duty-stage-only, dead since
ticket 007), `headingOtosWeight`/`otosStaleness` (OTOS blend off
everywhere — tracked separately in
`clasi/issues/later/estimator-v2-otos-fusion-sim-first.md`), and
`requireSettle`/`settleWindow` (feature false everywhere, dissolved by
ticket 008's `Settling`-state deletion — `plannedStopWindow()` falls
back to its built-in default allowance once `settleWindow` is gone).
Group the remaining 23 into `ceilings`/`plant`/`landing`/`tracking`
sub-structs. Break the append-only ctypes-mirror constraint ONCE:
regenerate `planner_harness.py` and its offset guards
(`plannerStructSizes`) against the new layout. Update every composition
root (`composeRobot()` from ticket 002, `simPlannerLimits()`, test
`benchLimits()`) and `applyShaperLimits()`/`applyTrimGains()`
signatures if the grouping suggests passing sub-structs.

Sequenced last in the honesty-pass chain because the trim-gain fields
are already gone by this point (relocated to `Drive`/robot-JSON in
ticket 005) and the `Settling`-state fields are already dissolved
(ticket 008) — one reshape, not two (the source issue's own
explicit concern).

## Acceptance Criteria

- [ ] `PlannerLimits` carries exactly the 23 fields the source issue
      lists, grouped into `ceilings`/`plant`/`landing`/`tracking`.
- [ ] `planner_harness.py`'s ctypes mirror and offset guards
      (`plannerStructSizes`) regenerated and passing against the new
      layout.
- [ ] Every composition root (`composeRobot()`, `simPlannerLimits()`,
      test `benchLimits()`) updated to construct the new struct shape.
- [ ] No dead field (`velKff`, `dutyFloor`, `requireSettle`, etc.)
      remains anywhere in the struct or its consumers.

## Testing

- **Existing tests to run**: the offset-guard test; full `planner_tests`
  ctest suite; full sim pytest suite (construction sites must all
  compile against the new layout).
- **New tests to write**: none beyond re-running the regenerated offset
  guard — this ticket is a mechanical reshape, not new behavior.
- **Verification command**: `uv run pytest`; ctest for `motion_tests`.

## Implementation Plan

**Approach**: mechanical regeneration + one deliberate ABI break, done
last in the planner-honesty sequence per the dependency ordering above.

**Files to create/modify**:
- `src/motion/planner/planner_types.h` (field cut + sub-struct
  grouping)
- `src/tests/sim/support/planner_harness.py` (ctypes mirror)
- The offset-guard generator/test (`plannerStructSizes`)
- Every `PlannerLimits` construction site (`composeRobot()`,
  `simPlannerLimits()`, `benchLimits()`)
- `applyShaperLimits()`/`applyTrimGains()` signatures if grouping
  suggests sub-structs

**Testing plan**: offset-guard test regenerated and passing; full
`planner_tests` + sim pytest suites green.

**Documentation updates**: inline field comments only — no separate doc
file changes expected.
