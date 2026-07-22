---
id: '007'
title: "Leave-one-out RMS validation notebook \u2014 estimator_validation.ipynb"
status: in-progress
use-cases:
- SUC-061
depends-on:
- '006'
github-issue: ''
issue: predict-to-now-odometry-estimator-ring-capture-dump-validation-trajectory-controller.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Leave-one-out RMS validation notebook — estimator_validation.ipynb

## Description

Build `src/tests/notebooks/estimator_validation.ipynb`, the notebook that
runs the stakeholder's own specified methodology end to end over a
captured TLM-log CSV (ticket 006's `estimator_capture.py` output): for
every stream (each wheel's position/velocity, body heading), walk the
whole log leave-one-out one-step-ahead (exclude sample k, take k−1 as
ZOH basis, predict k's timestamp, diff against actual k) using ticket
006's `one_step_ahead.py` reference; RMS the errors broken out by pattern
phase (steady, ramp, reversal, pivot); check the ZOH lag signature
(`a·k` velocity error, `½a·k²` distance error on ramps) against theory —
the evidence that decides whether a fit-based (non-ZOH) predictor is
warranted later; propagate per-step error through position integration
to project a leg-level accumulated position/heading error.

Output is proposed RMS accept thresholds and the phase/lag-signature
tables — explicitly NOT self-ratified (ticket 008 and the stakeholder own
that decision from real, or sim-substitute, data).

## Acceptance Criteria

- [ ] The notebook loads a TLM-log CSV (sprint 115 `tlm_log.py` / ticket
      006 `estimator_capture.py` output format) and runs the leave-one-out
      one-step-ahead walk per stream via `one_step_ahead.py` (ticket 006)
      — no duplicated prediction math inside the notebook itself.
- [ ] Pattern-phase segmentation (steady/ramp/reversal/pivot) is derived
      from the captured command sequence or from the telemetered
      velocity/mode signal — documented which, in the notebook itself.
- [ ] Per-stream, per-phase RMS tables are produced (each wheel's
      position/velocity error; body heading error).
- [ ] The ZOH lag-signature check (`a·k` velocity, `½a·k²` distance during
      ramps) is computed and compared against the theoretical prediction,
      with a clear pass/fail-style verdict on whether the lag signature
      matches theory.
- [ ] Per-step error is propagated through position integration to
      produce a leg-level accumulated position/heading error projection.
- [ ] The notebook's final cell(s) clearly state the proposed accept
      thresholds as PROPOSED, not ratified — no code path in this
      notebook writes a "thresholds accepted" artifact on its own.
- [ ] The notebook runs end-to-end against at least a sim-captured CSV
      (bench CSV substituted when available, ticket 008).

## Implementation Plan

**Approach.** Mirror the existing analysis-notebook precedent in
`src/tests/notebooks/` (e.g. `turn_sweep_analysis.ipynb`,
`drivetrain_stress.ipynb`) for structure: load CSV → derive per-stream
series → analysis cells → summary tables/plots. Keep all actual
prediction/residual math delegated to `one_step_ahead.py` (ticket 006) —
this notebook is presentation/orchestration, not a second implementation.

**Files to create:**
- `src/tests/notebooks/estimator_validation.ipynb`.

**Files to modify:** none expected beyond what ticket 006 already
covers for `src/tests/DESIGN.md` (this ticket's notebook is covered by
that same bullet — confirm it's listed; add a follow-up edit here only
if ticket 006 landed before this notebook's exact filename was decided).

**Documentation updates:** none beyond confirming `src/tests/DESIGN.md`
already names this notebook (ticket 006's own edit).

## Testing

- **Existing tests to run**: none directly (notebooks are not
  pytest-collected) — confirm `one_step_ahead.py`'s own tests
  (`test_one_step_ahead.py`, ticket 006) still pass, since this notebook
  depends on that module's contract.
- **New tests to write**: none required (exploratory artifact, per
  `src/tests/DESIGN.md`'s own `notebooks/` convention) — but the notebook
  itself must execute cleanly end-to-end as its own acceptance criterion.
- **Verification command**: run the notebook headless (e.g. `jupyter
  nbconvert --to notebook --execute`) against a sim-captured CSV fixture.
