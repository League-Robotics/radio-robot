---
id: '010'
title: Fix the deterministic sim turn-shaping undershoot
status: in-progress
use-cases:
- SUC-005
depends-on:
- 008
github-issue: ''
issue: sim-tour-turn-shaping-undershoots-90-degree-turns.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Fix the deterministic sim turn-shaping undershoot

## Description

Root-cause and fix the deterministic −10.8° (first turn) / −20.8°
(subsequent turns) per-turn undershoot documented in
`sim-tour-turn-shaping-undershoots-90-degree-turns.md`. The defect
predates this sprint (reproduced byte-identically at pre-sprint-128
baseline commit `c7a955c2`) and is independent of wheel control — it
lives in the sim/planner turn-shaping path, not the actuation path this
sprint otherwise rewrites. Bisect over sim/planner shaping history and
compare the shaped-band turn profile against sim ground truth.

Sequenced after ticket 008 (the `tick()` state-machine rewrite) purely
so the fix is made against the FINAL planner structure rather than code
about to be replaced mid-sprint — not because the bug is caused by or
related to the lifecycle rewrite.

## Acceptance Criteria

- [x] Root cause identified and fixed (preferred), or tolerances
      re-derived from a measured, explained sim behavior — never
      widened to paper over an unexplained defect.
      Root cause: `Motion::Planner::tick()`'s Angle profile-complete
      event could fire while the commanded ramp was still well above
      the rest floor (a short lookahead standing in for "the plant has
      actually finished" on a plant whose real actuation lag exceeds
      that lookahead's own window). Fixed in `planner.cpp` by gating
      that event on the commanded ramp itself reaching the rest floor;
      see the issue file's "Root cause and fix" section for the full
      trace and measurements. No fudge/calibration constant added.
- [ ] All four named tests green: **3 of 4, plus one residual turn in
      the 4th — NOT fully met, see below.**
      `test_gui_button_acceptance.py::test_tour_1_runs_to_completion`
      PASSES (worst turn error 2.72deg). `::test_tour_2_runs_to_completion`
      FAILS, but nondeterministically (a real background-tick-thread
      transport; observed failing at different legs across runs, both
      before and after this fix) — not verified whether it was ever
      reliably green pre-130. `test_sim_transport_tour1.py::test_tour_1_
      runs_to_completion_with_finite_small_closure` PASSES.
      `test_tour_closure_gate.py::test_tour_1_and_tour_2_ninety_degree_
      turns_land_within_the_shaped_band`: TOUR_1 fully passes (turns +
      cruise-heading); TOUR_2 passes every turn except its own
      146-degree turn (-10.12deg vs the 8.0deg gate) — see the issue
      file's "Residual, NOT closed by 130-010" section.
- [x] The fix or explanation recorded in the issue file and the commit
      message.

## Testing

- **Existing tests to run**: the four named tests above; the broader
  sim tour test suite to confirm no fallout from the fix.
- **New tests to write**: an isolated turn-only repro test, if the
  bisect finds a clean minimal reproduction distinct from the four full
  tour tests.
- **Verification command**: `uv run pytest`

## Implementation Plan

**Approach**: bisect (git bisect or manual history read) over
`shape.cpp`/`profile.cpp`'s turn-shaping logic, comparing the shaped
angular profile against sim ground truth to find where a fixed wedge of
rotation is lost. The −10.8°/−20.8° (~2x) signature suggests a fixed
per-turn deficit plus a carried-over offset — start there.

**Files to create/modify**:
- `src/motion/planner/shape.{h,cpp}`
- `src/motion/planner/profile.{h,cpp}`
- Associated test files as needed

**Testing plan**: the four named tests plus any new isolated repro
test; full sim tour suite as a regression check.

**Documentation updates**: the issue's own acceptance note gets the
root cause/fix recorded, per its acceptance criteria.
