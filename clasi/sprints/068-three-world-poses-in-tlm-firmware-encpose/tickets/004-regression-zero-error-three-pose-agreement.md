---
id: '004'
title: 'Regression: zero-error three-pose agreement'
status: done
use-cases:
- SUC-003
depends-on:
- '002'
github-issue: ''
issue: tlm-three-world-poses-encoder-only-pose.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Regression: zero-error three-pose agreement

## Description

This ticket is the sprint's closing acceptance gate. With all sim
error-injection knobs (encoder noise, turn-slip scrub, OTOS noise/drift)
set to zero, there is nothing left for the three pose estimators
(encoder-only, OTOS, EKF-fused) to disagree about — so a new sim
regression test asserts all three wire poses (`encpose=`, `otos=`,
`pose=`) AND plant ground truth (`sim.get_true_pose()`) agree within a
small tolerance throughout a multi-leg maneuver (straight legs and turns,
in the shape of the TestGUI's `TOUR_1` sequence).

This test's purpose is to catch a future regression in any of the three
pose estimators, or in the TLM wire encoding of any of them, specifically
and by name — not only when noise happens to be nonzero (per
`architecture-update.md` SUC-003).

This ticket only requires Ticket 002 (host-side `parse_tlm`/`TLMFrame`
support for `encpose`) — it does not require Ticket 003 (TestGUI changes),
since the test drives the sim directly via `tick_collect_tlm` and parses
frames through the protocol layer, not through the TestGUI.

## Acceptance Criteria

- [x] New sim regression test (tier: `tests/simulation/system/`) drives a
      multi-leg maneuver (straight legs and at least one turn, matching
      the shape of the TestGUI's `TOUR_1` sequence) with all sim
      error-injection knobs at zero (e.g.
      `sim.set_field_profile(slip_turn_extra=0.0, ...)` and equivalent
      zeroing of OTOS noise/drift — confirm the exact zeroing API by
      inspecting the sim error-injection module).
      Implemented as
      `tests/simulation/system/test_068_004_zero_error_three_pose_agreement.py`.
      Zeroing required TWO knobs, not one: `set_field_profile(slip_turn_extra=0.0,
      fuse_otos=True)` (SimOdometer's own noise/drift/scale-error setters
      already default to zero on a fresh `Sim()`, so `fuse_otos=True`
      introduces no error) PLUS `SET rotSlip=0` — `RobotConfig::rotationalSlip`
      (default 0.92) is Odometry::predict()'s turn-arc correction factor;
      with the plant's simulated scrub zeroed there is nothing left to
      correct for, and leaving `rotationalSlip` non-zero produced a real
      262 mm / 37° `encpose=` disagreement (empirically confirmed while
      writing this test, documented in the test module's docstring).
      `SET rotSlip=0` is this project's established "no correction"
      sentinel (`effectiveSlip()` maps 0 → 1.0).
- [x] For each collected TLM frame, `frame.encpose`, `frame.otos`, and
      `frame.pose` are compared against `sim.get_true_pose()` (plant
      ground truth) at the same tick; all four agree within a small
      numeric tolerance throughout the maneuver.
      Tolerances: 5.0 mm (x/y) and 5.0 centidegrees (heading), several
      times the ~1.4 mm / ~1 centidegree residual actually observed
      (wire-encoding integer truncation + float32 accumulation).
- [x] The test collects frames via `tick_collect_tlm` (or the project's
      established equivalent — confirm exact helper name/signature by
      inspecting existing `tests/simulation/system/` tests before writing
      new code).
      Ticked in `STEP_MS`-sized (24 ms) increments so `sim.get_true_pose()`
      can be sampled immediately after the same tick that produced each
      collected frame (tick_collect_tlm's replyStore-draining semantics
      discard non-TLM lines every internal tick, so EVT-based completion
      detection does not compose with a single large collection window —
      per-leg completion is instead detected from `frame.mode == "I"`
      within each collected frame).
- [x] `tests/_infra/golden_tlm_capture.json` (regenerated in Ticket 001)
      remains valid: `test_golden_tlm_unchanged` passes with `encpose=`
      present in every frame of the fixed sequence.
      Re-confirmed: `uv run python -m pytest tests/simulation/unit/test_golden_tlm.py`
      → 1 passed.
- [x] Full default pytest suite green: `uv run python -m pytest` reports
      2520 passed (baseline after sprint 067) plus this sprint's new/
      modified tests, 0 failed. This is the sprint's closing acceptance
      gate — do not consider this ticket (or the sprint) done until the
      full suite is confirmed green.
      Confirmed: `uv run python -m pytest` → **2528 passed, 0 failed**
      (2527-passed ticket-004 baseline + 1 new test in this ticket).

## Testing

- **Existing tests to run**: full default suite via
  `uv run python -m pytest` (this IS the ticket's primary deliverable —
  confirming baseline plus new tests are green); re-confirm
  `test_golden_tlm_unchanged` passes.
- **New tests to write**: the zero-error, multi-leg, three-pose-plus-
  ground-truth agreement test described above.
- **Verification command**: `uv run python -m pytest`

## Implementation Plan

**Approach**: Write one new sim regression test module under
`tests/simulation/system/` that zeroes all injected sim error, drives a
`TOUR_1`-shaped multi-leg maneuver, and asserts pointwise agreement
between `frame.encpose`, `frame.otos`, `frame.pose`, and
`sim.get_true_pose()` at each collected tick. Follow this project's
established sim-test conventions (check sibling files in
`tests/simulation/system/` for maneuver-driving and TLM-collection
helpers before writing new plumbing).

**Files to create**:
- A new test file under `tests/simulation/system/` (name per existing
  test-file conventions in that directory — check sibling files for the
  established naming pattern, e.g. ticket-number-prefixed or
  feature-named).

**Files to modify**: none expected beyond the new test file. If the test
surfaces a genuine disagreement among the three poses (i.e., Tickets
001-002's changes are correct but expose a pre-existing estimator bug),
treat that as a signal to investigate rather than loosening the tolerance
to make the test pass.

**Testing plan**:
- Implement the zero-error multi-leg test per the acceptance criteria
  above.
- Run it in isolation first to confirm it passes and is not flaky (repeat
  a few times if the maneuver includes any timing-sensitive collection).
- Run the full default suite (`uv run python -m pytest`) and confirm the
  final green count matches or exceeds the 2520-passed / 0-failed baseline
  plus all new tests from this sprint's four tickets.
- This is the sprint's final gate — report the exact pass count achieved.

**Documentation updates**: none — `architecture-update.md` already
documents this test's design in full (SUC-003, Step 5 "New regression
test"). No wire-protocol change, no `RobotConfig` schema change.
