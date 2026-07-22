---
id: '005'
title: "Sim system scenario \u2014 StateEstimator tracks SimPlant truth across varied\
  \ MOVE patterns"
status: done
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

- [x] `src/tests/sim/system/test_state_estimator_tracking.py` (+ a new
      `*_harness.cpp` if the existing `SimApi`/`sim_api_harness.cpp`
      surface doesn't already expose what's needed) drives the full
      pattern set (steady, ramp, reversal, pivot; both directions;
      straights and turns) through `SimApi`/`SimPlant`.
- [x] At each step (or a documented sampling cadence), `StateEstimator::
      whereAmI()`/`wheelNow()`/`wheelAt()`/`bodyAt()` predictions are
      compared against `SimPlant`'s own ground-truth wheel/body state;
      error stays within a documented, justified tolerance for every
      pattern phase.
- [x] The test explicitly reports (not silently passes/fails) which
      pattern phases showed the largest tracking error, as input to
      ticket 007's RMS notebook's own phase breakdown.
- [x] (Stretch — optional) a throwaway host-build replay harness
      cross-checks the firmware estimator's own output against ticket
      006's Python `one_step_ahead.py` reference on the same input
      sequence, to float noise. If skipped, this ticket's completion
      notes state why (e.g. "would require a new `sim_ctypes.cpp` export,
      not cheap — deferred").

## Completion Notes (2026-07-22)

- `src/tests/sim/system/state_estimator_tracking_harness.cpp` +
  `test_state_estimator_tracking.py` (mirrors `move_protocol_harness.cpp`'s
  own shape) drive `TestSim::SimHarness` (the REAL `App::RobotLoop` +
  wired-in `App::StateEstimator`, ticket 004) through 7 pattern phases:
  `forward_ramp`/`forward_steady`, `reversal_transient`/`steady_reverse`,
  `pivot_ramp`/`pivot_steady`, and `chained_steps` (4 chained moves, both
  directions, straight + turn).
- Every cycle of every phase, the harness makes a genuine one-cycle-ahead
  (`SimHarness::kCycleDtUs` = 50ms) `StateEstimator::wheelAt()`/`bodyAt()`
  prediction (exercising the actual ZOH extrapolation math, not just
  `wheelNow()`/`whereAmI()` at age zero), then checks it against
  `TestSim::SimPlant`'s own ground truth (`WheelPlant::position()`/
  `velocity()`, `SimHarness::trueX()`/`trueY()`/`trueHeading()`) once that
  next cycle has actually run. Per-phase tolerances were derived
  empirically (see the harness's own per-`runPhase()` comments) and keep
  >2x margin over the observed worst case in every phase.
- AC #3: the harness prints a `REPORT:` section ranking phases by max
  one-cycle-ahead wheel-distance error and by max body-position error.
  Observed worst phase (this run): `reversal_transient` (~7.0mm), closely
  followed by `forward_ramp` (~6.8mm) — both still-transient (inside
  `TestSim::kDefaultTau`'s ~130ms settle window) phases, as expected for a
  held-constant-velocity ZOH assumption. Steady-state phases all measured
  well under 0.2mm / 0.002rad.
- AC #4 (stretch replay harness): SKIPPED. See the harness's own file
  header for the full rationale — cross-checking the wired-in estimator's
  C++ output against ticket 006's Python `one_step_ahead.py` reference
  would need a new `sim_ctypes.cpp` export (or an ad hoc bridge invented
  from scratch) to hand a Python process this binary's own in-process
  per-cycle Frame stream; not "cheap" per the ticket's own qualifier.
  Ticket 006's `test_one_step_ahead.py` already unit-tests the Python
  reference against the same documented ZOH formula (matching ticket 002's
  C++ source on hand-computed fixtures), covering the "same math" concern
  this stretch item was after; this ticket's own job (proving the WIRING)
  is a different, already-covered concern.
- No production/`src/firm/` code changed — test-only ticket, per the
  ticket's own Implementation Plan.

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
