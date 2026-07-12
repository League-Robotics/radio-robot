---
id: '005'
title: Encoder-delta timestamp fix for PoseEstimator's joint predict
status: done
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

- [x] `PoseEstimator` gains two new private fields tracking the
      `sampled_at` value each wheel had at the last APPLIED joint step
      (e.g. `prevSampledAtLeft_`/`prevSampledAtRight_`).
- [x] `tick()`'s encoder-delta/arc-integration step (the
      `encoderPose()`-backing accumulator) fires ONLY when both
      `leftObs.sampled_at.val` and `rightObs.sampled_at.val` have advanced
      past their respective previously-consumed values since the last
      joint step. A tick where one or both are unchanged since the last
      joint step performs no accumulator advance for THIS tick, matching
      `tick()`'s existing "no observation this pass" early-return
      precedent for missing `.has` fields.
      **Reconciled during implementation** (team-lead pre-authorized,
      confirmed against architecture-update.md Decision 6 before coding):
      this criterion's original parenthetical additionally said "(not even
      a zero-delta no-op predict — genuinely skipped)", directly
      contradicting AC-3/AC-4 below and the Implementation Plan, both of
      which require `EkfTiny::predict()` to run every tick with an
      UNCHANGED wall-clock `dt`. Decision 6's own "Consequences" paragraph
      settles this: only "that step's own `dTheta`/`dCenter`" is gated —
      it says nothing about gating `predict()` itself, and process noise
      must keep growing with true elapsed time regardless of encoder
      staleness. The erroneous "predict genuinely skipped" phrase is
      removed from this criterion's text above (was: applied to both "the
      `encoderPose()`-backing accumulator and the values fed to
      `EkfTiny::predict()`"); the implementation gates ONLY the geometric
      delta's source (`dCenter=0, dTheta=0` on a non-paired tick), never
      whether `predict()` runs. See AC-3/AC-4 and the code's own comments
      in `pose_estimator.cpp`/`pose_estimator.h` for the corrected,
      internally-consistent contract.
- [x] When the joint step DOES fire, `dCenter`/`dTheta` are computed from
      the position deltas since the LAST joint step (not necessarily the
      immediately-prior tick's positions) — i.e. this is a genuine
      decoupling of "how often we integrate" from "how often `tick()` is
      called," not merely a relabeling.
- [x] `EkfTiny::predict()`'s own `dt`/process-noise scaling is UNCHANGED
      by this ticket (still wall-clock `now - lastTick_`, per
      architecture-update.md's Decision 6 analysis: process noise
      correctly grows with true elapsed time regardless of encoder
      staleness — only the geometric arc-integration attribution was
      wrong, not the noise-growth rate). Do not conflate the two.
      `predict()` itself runs unconditionally every tick (see AC-2's
      reconciliation note above); only its `dCenter`/`dTheta` ARGUMENTS
      are zeroed on a non-paired tick, never its `dt` or whether it is
      called at all.
- [x] `PoseEstimator::resetEncoderBaseline()`'s existing deferred-apply
      contract (`encBaselineResetPending_`, applied on the first
      genuinely time-advancing tick) is preserved and interacts correctly
      with the new paired-freshness gate — a reset pending during a
      not-yet-paired-fresh tick still defers correctly.
- [x] New/extended `pose_estimator_harness.cpp` and/or `nezha_flipflop_
      harness.cpp` cases: a sequence of ticks where only one wheel's
      `sampled_at` advances per tick produces IDENTICAL total accumulated
      `encoderPose()` displacement to a synchronous-pair baseline (proving
      the telescoping-sum total is unaffected), but the PER-TICK
      intermediate values differ from the pre-fix (naive) behavior in the
      direction the analysis predicts (no local misattribution when only
      one side is stale).
- [x] Full sim suite passes.
- [x] Sim-only gate for this ticket (no bench requirement — this is a
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

## Completion Notes

**AC-2 reconciliation (predict-every-tick vs. genuinely-skipped-predict).**
This ticket's original text was internally contradictory: AC-2's
parenthetical said a non-paired tick performs "no accumulator/predict
advance ... not even a zero-delta no-op predict — genuinely skipped,"
while AC-3, AC-4, and the Implementation Plan all required
`EkfTiny::predict()` to run every tick with `dt` still wall-clock
`now - lastTick_`. Before writing any code, this was checked against
architecture-update.md's Decision 6 "Consequences" paragraph, which is the
authority: "the joint-step computation only fires once both sides have
advanced ... using the true elapsed time between those two stamps ... for
that step's own `dTheta`/`dCenter`" — it gates the geometric delta only,
never `predict()` itself, and the surrounding Decision 6 prose is explicit
that "process noise correctly grows with true elapsed time regardless of
encoder staleness — only the geometric arc-integration attribution was
wrong, not the noise-growth rate." This confirms `predict()` must run
unconditionally every tick with an unchanged `dt`; AC-2's "genuinely
skipped predict" phrase was the erroneous one and has been struck from
AC-2's text above (the checkbox list) with an explanation, per the
team-lead's pre-authorized resolution. This is a ticket-internal wording
fix, not an architecture change — Decision 6 itself was never ambiguous
about which quantity is gated.

**Implementation summary.** `PoseEstimator` gained
`prevSampledAtLeft_`/`prevSampledAtRight_` (the `sampled_at` recorded at
the last APPLIED joint step). `tick()` now computes a `pairedFresh` gate
(`leftObs.sampled_at.val`/`rightObs.sampled_at.val` both advanced past
those fields since the last joint step, OR this is the very first
application ever — `!haveEncBaseline_` — matching the pre-fix code's own
"first tick captures the baseline, zero delta" warm-up precedent so the
gate cannot introduce a NEW phantom-jump risk at boot or after a
`resetEncoderBaseline()`). When the gate fires: `dCenter`/`dTheta` are
computed against `prevEncLeft_`/`prevEncRight_` (the position at the last
APPLIED step, not the immediately-prior tick), the `encoderPose()`
accumulator advances, and `prevEncLeft_`/`prevEncRight_`/
`prevSampledAtLeft_`/`prevSampledAtRight_` are all updated together. When
it does not fire, `dCenter`/`dTheta` stay `0.0f` for that tick's
`EkfTiny::predict()` call, the accumulator and all four `prev*_` fields
are left untouched, and `predict()` still runs with the same wall-clock
`dt` as any other tick (`lastTick_`/`haveLastTick_` update unconditionally,
independent of the gate). `resetEncoderBaseline()`'s existing
`encBaselineResetPending_`/`dt > 0` deferred-apply mechanism needed no
code change: clearing `haveEncBaseline_` is what makes the next joint-step
candidate bypass the freshness check again (treated as a fresh first
application), so a reset consumed on a non-paired tick still correctly
defers the actual zero-delta baseline capture until whichever tick's joint
step next fires.

**Test evidence.** Added
`scenarioStaggeredSampleTimingMatchesSynchronousTotalNoLocalMisattribution()`
to `tests/sim/unit/pose_estimator_harness.cpp`: five arc segments run
through (a) a synchronous-pair baseline (both wheels fresh every tick,
matching the pre-fix code's implicit assumption) and (b) a staggered run
(each segment split into a left-only tick then a right-only tick, only one
side's `sampled_at`/position advancing per tick). Result: the staggered
run's TOTAL final `encoderPose()` displacement matches the synchronous
baseline to within float rounding (telescoping-sum total unaffected by
staleness — confirmed, not just asserted by comment: both runs converge
segment-by-segment to bit-identical intermediate states by construction,
since a firing joint step always captures the FULL accumulated delta since
the last firing for both wheels, regardless of how many non-paired ticks
intervened). Per-tick evidence: after every left-only tick,
`encoderPose()` is provably unchanged from immediately before it (no local
pivot misattribution while the right side is stale), and — for every
segment with a nonzero `dL` — a hand-computed "naive" (pre-fix-style)
one-sided delta is shown to diverge from the fixed code's zero-change
result by >1mm, confirming the fix's result is a genuine behavioral
divergence from the old formula, not a vacuous no-op. One test-construction
bug was found and fixed while building this scenario (not a production
bug): the synchronous baseline's very first tick silently "eats" segment 0
as `haveEncBaseline_`'s pre-existing zero-delta warm-up capture, so the
staggered run needed a matching explicit warm-up tick before segment 0 to
stay apples-to-apples — without it, the two runs differed by exactly
segment 0's straight-line displacement (40mm), which was the first
(incorrect) result observed before the fix.

**Verification.** `just build` — clean (only pre-existing, unrelated
`libraries/tinyekf/tinyekf.h` unused-function warnings). Full pytest suite
— `1287 passed, 4 xfailed, 1 xpassed`, matching the pre-ticket baseline
exactly (0 failed, no regressions in any of the seven pre-existing
`pose_estimator_harness.cpp` scenarios, all of which use synchronous
`sampled_at` throughout and therefore exercise the gate as
always-fires — numerically identical to their pre-fix behavior). No bench
gate required (sim-only ticket per architecture-update.md's own gate
assignment).
