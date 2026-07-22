---
id: '005'
title: "Sim system scenario \u2014 StateEstimator tracks SimPlant truth across varied\
  \ MOVE patterns"
status: in-progress
use-cases:
- SUC-060
depends-on:
- '004'
github-issue: ''
issue: predict-to-now-odometry-estimator-ring-capture-dump-validation-trajectory-controller.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sim system scenario — StateEstimator tracks SimPlant truth across varied MOVE patterns

## Description

Prove `App::StateEstimator` (wired in by ticket 004) tracks the
simulator's own ground-truth wheel/body state across the same varied
motion-pattern set the stakeholder's validation methodology specifies:
both directions, steps, reversals, pivots. This is the sim-side half of
the sprint's core proof (bench-side is ticket 008); it runs the real
firmware (compiled `-DHOST_BUILD`) against `SimPlant`, the ONE sim object
this project's tests and TestGUI share — never a divergent, test-only
path.

Also covers the "stretch, if cheap" cross-check the source issue names:
a throwaway host-build replay harness (mirroring the existing
`sim/unit/*_harness.cpp` convention — a small, ad hoc compiled binary,
not a new `sim_ctypes.cpp`/dylib ABI surface) that feeds the SAME input
sequence through the firmware's own `StateEstimator` and through ticket
006's Python `one_step_ahead.py` reference, confirming they agree to
float noise. This is explicitly optional/best-effort per the source
issue's own "if cheap" qualifier — if the harness turns out to need new
`sim_ctypes.cpp` exports or other non-trivial plumbing, skip it and say
so in this ticket's own completion notes rather than forcing it.

## Acceptance Criteria

- [ ] `src/tests/sim/system/test_state_estimator_tracking.py` (+ a new
      `*_harness.cpp` if the existing `SimApi`/`sim_api_harness.cpp`
      surface doesn't already expose what's needed) drives the full
      pattern set (steady, ramp, reversal, pivot; both directions;
      straights and turns) through `SimApi`/`SimPlant`.
- [ ] At each step (or a documented sampling cadence), `StateEstimator::
      whereAmI()`/`wheelNow()`/`wheelAt()`/`bodyAt()` predictions are
      compared against `SimPlant`'s own ground-truth wheel/body state;
      error stays within a documented, justified tolerance for every
      pattern phase.
- [ ] The test explicitly reports (not silently passes/fails) which
      pattern phases showed the largest tracking error, as input to
      ticket 007's RMS notebook's own phase breakdown.
- [ ] (Stretch — optional) a throwaway host-build replay harness
      cross-checks the firmware estimator's own output against ticket
      006's Python `one_step_ahead.py` reference on the same input
      sequence, to float noise. If skipped, this ticket's completion
      notes state why (e.g. "would require a new `sim_ctypes.cpp` export,
      not cheap — deferred").

## Implementation Plan

**Approach.** Mirror `test_move_protocol.py`'s existing shape (sprint
116 ticket 008): a scripted sequence of MOVE commands through `SimApi`,
stepping the sim clock, reading back both the plant's ground truth
(`SimPlant`'s own accessors) and the firmware's telemetered state
(`frame_`/estimator query results, exposed however `SimApi` already
exposes `RobotLoop` internals for test purposes — check existing
precedent in `sim_api_harness.cpp` before adding new exports).

**Files to create:**
- `src/tests/sim/system/test_state_estimator_tracking.py`.
- A harness `.cpp` file if needed (naming to mirror existing
  `*_harness.cpp` files in `sim/system/`).
- (Stretch) a throwaway replay harness, e.g.
  `src/tests/sim/unit/estimator_replay_harness.cpp`, reading a small
  input fixture and printing predicted-vs-actual for a Python test to
  diff against `one_step_ahead.py`'s own output.

**Files to modify:** none expected beyond what's needed to expose
estimator query results through the existing `SimApi` test surface, if
not already exposed.

**Documentation updates:** none — this is a test-only ticket.

## Testing

- **Existing tests to run**: `src/tests/sim/system/test_move_protocol.py`
  (confirm no interaction/regression), full `uv run python -m pytest`.
- **New tests to write**: `test_state_estimator_tracking.py` as described
  above; optional replay cross-check test.
- **Verification command**: `uv run python -m pytest src/tests/sim/system/test_state_estimator_tracking.py`.
