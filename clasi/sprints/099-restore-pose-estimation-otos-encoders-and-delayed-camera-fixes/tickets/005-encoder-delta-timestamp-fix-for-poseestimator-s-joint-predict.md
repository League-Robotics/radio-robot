---
id: '005'
title: Encoder-delta timestamp fix for PoseEstimator's joint predict
status: open
use-cases: [SUC-002]
depends-on: ['004']
github-issue: ''
issue: restore-pose-estimation-otos-encoders-delayed-camera-fixes.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Encoder-delta timestamp fix for PoseEstimator's joint predict

## Description

`bb.motors[]`'s wheel positions refresh on the I2C flip-flop's own
~40-80ms-per-motor cadence, while `PoseEstimator::tick()` (as of ticket
004) runs every 20ms main-loop pass. The existing joint arc-integration
math (`dCenter = (dL+dR)/2`, `dTheta = (dR-dL)/trackwidth`,
`pose_estimator.cpp`) implicitly assumes `dL`/`dR` are simultaneous,
same-epoch samples — they usually are not, since the flip-flop only
refreshes ONE motor's position per slot.

Per architecture-update.md's Decision 6 analysis: the *total* accumulated
translation/rotation over many ticks is exact regardless of this staleness
(a telescoping sum — each wheel's true delta is counted exactly once,
whichever tick its fresh sample lands on). What is **not** exact is the
*local* attribution: a tick where only one wheel refreshed computes
`dCenter`/`dTheta` as if the robot pivoted around a point for that
instant, when physically both wheels were moving continuously — the same
*class* of bug this codebase already found and fixed once for OTOS
heading (`otos_odometer.h`'s "same-instant-heading contract," commit
db11b7c), applied here to left/right encoder skew. This does not cancel
over a sustained one-directional turn (exactly the motion the 098
heading-loop work cares most about) — a real, currently-uncharacterized
source of heading bias in `fusedPose`/`encoderPose`.

This ticket gates the joint arc-integration step on both wheels having a
**genuinely fresh, paired** sample since the last joint step, using each
wheel's own `MotorState.sampled_at` (already present, unused for this
purpose today) to decouple the 20ms tick cadence from the ~40-80ms
flip-flop cadence for this ONE computation. Sim/bench-testable and
independently revertible from ticket 004's basic wiring if the evidence
shows this bias was smaller than the EKF's own process noise already
absorbed.

## Acceptance Criteria

- [ ] `PoseEstimator` gains two new private fields tracking the
      `sampled_at` value each wheel had at the last APPLIED joint step
      (e.g. `prevSampledAtLeft_`/`prevSampledAtRight_`).
- [ ] `tick()`'s encoder-delta/arc-integration step (both the
      `encoderPose()`-backing accumulator and the values fed to
      `EkfTiny::predict()`) fires ONLY when both `leftObs.sampled_at.val`
      and `rightObs.sampled_at.val` have advanced past their respective
      previously-consumed values since the last joint step. A tick where
      one or both are unchanged since the last joint step performs no
      accumulator/predict advance for THIS tick (not even a zero-delta
      no-op predict — genuinely skipped, matching `tick()`'s existing
      "no observation this pass" early-return precedent for missing
      `.has` fields).
- [ ] When the joint step DOES fire, `dCenter`/`dTheta` are computed from
      the position deltas since the LAST joint step (not necessarily the
      immediately-prior tick's positions) — i.e. this is a genuine
      decoupling of "how often we integrate" from "how often `tick()` is
      called," not merely a relabeling.
- [ ] `EkfTiny::predict()`'s own `dt`/process-noise scaling is UNCHANGED
      by this ticket (still wall-clock `now - lastTick_`, per
      architecture-update.md's Decision 6 analysis: process noise
      correctly grows with true elapsed time regardless of encoder
      staleness — only the geometric arc-integration attribution was
      wrong, not the noise-growth rate). Do not conflate the two.
- [ ] `PoseEstimator::resetEncoderBaseline()`'s existing deferred-apply
      contract (`encBaselineResetPending_`, applied on the first
      genuinely time-advancing tick) is preserved and interacts correctly
      with the new paired-freshness gate — a reset pending during a
      not-yet-paired-fresh tick still defers correctly.
- [ ] New/extended `pose_estimator_harness.cpp` and/or `nezha_flipflop_
      harness.cpp` cases: a sequence of ticks where only one wheel's
      `sampled_at` advances per tick produces IDENTICAL total accumulated
      `encoderPose()` displacement to a synchronous-pair baseline (proving
      the telescoping-sum total is unaffected), but the PER-TICK
      intermediate values differ from the pre-fix (naive) behavior in the
      direction the analysis predicts (no local misattribution when only
      one side is stale).
- [ ] Full sim suite passes.
- [ ] Sim-only gate for this ticket (no bench requirement — this is a
      geometric-accuracy refinement, not a new hazard class; the effect
      is validated against known sim-plant ground truth, matching
      architecture-update.md's own gate assignment).

## Implementation Plan

**Approach**: read `pose_estimator.cpp`'s current `tick()` in full before
editing (small, well-understood function). Add the paired-freshness gate
as a precondition on the existing arc-integration block; when not
satisfied, skip straight to the (unconditional) `EkfTiny::predict()` call
with `dCenter=0, dTheta=0` for THIS tick's contribution (predict still
runs every tick per its own existing contract — only the geometric
delta's SOURCE changes, not whether predict is called) — clarify this
distinction precisely during implementation and in the ticket's
completion notes, since it is easy to conflate "skip the joint step" with
"skip predict()" and the architecture doc is explicit these are
different.

**Files to modify**:
- `source/subsystems/pose_estimator.h` — two new private fields, updated
  `tick()` doc comment.
- `source/subsystems/pose_estimator.cpp` — the gated joint-step logic.

**Testing plan**:
- Extend `tests/sim/unit/pose_estimator_harness.cpp` with a staggered-
  sample-timing test proving the telescoping-sum total is unaffected and
  the local per-tick behavior matches the fix's intent.
- Full sim suite.

**Documentation updates**: none required (internal accuracy fix, no wire/
config change).
