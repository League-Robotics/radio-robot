---
id: '002'
title: PoseEstimator hardware fused-pose investigation and fix
status: open
use-cases: [SUC-002]
depends-on: []
github-issue: ''
issue: poseestimator-fused-pose-frozen-on-hardware.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# PoseEstimator hardware fused-pose investigation and fix

## Description

During sprint 089 ticket 007's bench verification (2026-07-07), robot on
the stand, `Subsystems::PoseEstimator`'s fused pose (`TLM pose=`) stayed
frozen at `(0, 0, -7)` across 1.3+ m of real encoder travel. `RT`
(whose `STOP_ROTATION` reads the raw encoder-arc differential directly, not
`fusedPose`) completed correctly and is unaffected; `TURN`'s `STOP_HEADING`
and `G`'s target-region detection both depend on `fusedPose` and so cannot
complete on hardware today
(`clasi/issues/poseestimator-fused-pose-frozen-on-hardware.md`). This is a
pre-existing defect, unrelated to 089's Ruckig migration -- the sim plant
does not reproduce it.

This is a CODE-INVESTIGATION ticket, not a pre-solved fix. Read
`architecture-update.md`'s Grounding section and Decision 5 in full first
-- they already rule out the two simplest hypotheses (a stuck
`leftObs.position.has`/`rightObs.position.has` -- would also break `RT`,
which it does not; a stuck `Hal::Odometer::fusableThisPass()` -- can only
suppress OTOS correction, cannot by itself freeze `encoderPose()`) and name
two remaining, more specific candidates distinguished by ONE cheap
diagnostic: does `Subsystems::PoseEstimator::encoderPose()` (`TLM
encpose=`) ALSO freeze on hardware, or only `fusedPose()` (`pose=`)? Run
this diagnostic FIRST -- it determines which of Decision 5's two candidates
(a step-1/encoder-accumulator bookkeeping issue affecting BOTH readings, or
an `EkfTiny::predict()`-specific `dt`/numerical-stability issue affecting
ONLY the fused reading) is actually in play, before investing time in
either blind.

## Acceptance Criteria

- [ ] The diagnostic (does `encpose=` freeze too, or only `pose=`?) is run
      and its result recorded in this ticket's completion notes -- either
      from the existing 089-007 raw trace if it already contains `encpose=`
      samples, or a fresh bench capture.
- [ ] Root cause is identified via code investigation (comparing the
      working sim path -- `Hal::SimOdometer`/`Hal::NullOdometer` feeding
      `PoseEstimator::tick()` -- against the real hardware path --
      `Hal::OtosOdometer`, `main_loop.cpp`'s `poseEstimator_.tick(...)` call
      site, and `PoseEstimator`'s own internal encoder dead-reckoning) and
      recorded in this ticket's completion notes, whether or not a bench
      re-test is achievable afterward.
- [ ] The most plausible fix is implemented.
- [ ] **Sim, BLOCKING**: full `uv run python -m pytest tests/sim` stays
      green after the fix -- `PoseEstimator`'s existing sim coverage is a
      regression guard (sim does not reproduce the hardware defect itself,
      so it cannot be the reproduction test).
- [ ] Where the root cause is confirmed and a targeted white-box unit test
      is feasible (e.g. if it is a `dt`/baseline-bookkeeping issue
      reproducible with a synthetic tick sequence), add one; if the defect
      is genuinely only reproducible on real hardware, state that
      explicitly rather than fabricating a sim test that does not actually
      exercise the fixed mechanism.
- [ ] **Bench, BEST-EFFORT**: re-run the `TURN` accuracy check (086/087
      tolerance bars) and the `G` settle smoke check from 089-007. If
      unreachable (hardware unavailable, or the fix cannot be confirmed
      live), descope to a fresh `clasi/issues/` follow-on rather than
      blocking sprint close.

## Implementation Plan

**Approach**:
1. Read `architecture-update.md` Grounding + Decision 5 in full.
2. Run the diagnostic (encpose= vs pose=) first -- see Description.
3. Depending on the diagnostic's result, investigate the matching candidate
   from Decision 5:
   - Both frozen: check `PoseEstimator::tick()`'s step-1 guard and
     `haveEncBaseline_`/`encBaselineResetPending_` bookkeeping against what
     `main_loop.cpp` actually passes as `leftObs`/`rightObs` on hardware vs.
     sim (`bb.motors[p.left-1]`/`bb.motors[p.right-1]` indexing).
   - Only fused frozen: check `EkfTiny::predict()`'s `dt` computation
     (`haveLastTick_`/`lastTick_`) for a degenerate/zero `dt` on hardware,
     or a numerically stuck EKF state/covariance.
4. Land the fix; add regression coverage per the Acceptance Criteria.
5. Attempt the bench re-verification; record honestly.

**Files to modify/create**: most likely `source/subsystems/pose_estimator.{h,cpp}`;
possibly `source/hal/otos/otos_odometer.{h,cpp}` or
`source/runtime/main_loop.cpp` if the investigation implicates the wiring
instead (architecture-update.md Step 3/5 name all three as candidates, not
a certain one).

**Testing plan**:
- **Existing tests to run**: full `uv run python -m pytest tests/sim`.
- **New tests to write**: a targeted regression test for the confirmed
  mechanism, if sim-reproducible (see Acceptance Criteria).
- **Verification command**: `uv run python -m pytest tests/sim`.

**Documentation updates**: none expected beyond this ticket's completion
notes recording the root-cause finding.

## Testing

- **Existing tests to run**: `uv run python -m pytest tests/sim` (full
  suite -- `PoseEstimator`'s existing coverage is the regression guard).
- **New tests to write**: a targeted regression test for the confirmed
  mechanism, if feasible; otherwise an explicit note on why it is
  hardware-only.
- **Verification command**: `uv run python -m pytest tests/sim`.
