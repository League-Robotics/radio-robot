---
id: '002'
title: Heading PD cascade, tolerance/dwell completion, and replan retirement
status: open
use-cases: [SUC-001, SUC-002]
depends-on: ['001']
github-issue: ''
issue:
- heading-loop-cascade-control-turns-terminate-on-target.md
- real-robot-motion-calibration-undershoot.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Heading PD cascade, tolerance/dwell completion, and replan retirement

## Description

The sprint's core behavioral change. Implement the outer heading PD cascade
for PRE_PIVOT/TERMINAL_PIVOT in `Motion::SegmentExecutor`
(`segment_executor.cpp`), replace their `STOP_ROTATION`-arc-threshold +
ride-the-tail completion with a tolerance+dwell gate, and retire
`maybeReplanPivot()`'s sub-gross EXTEND branch to a no-op for these two
phases (the gross-divergence reanchor branch stays live as stall
protection). TRANSLATE and BLEND are untouched — this ticket is scoped to
the rotational channel's PRE_PIVOT/TERMINAL_PIVOT phases only.

Reference: `architecture-update.md` M3/M4/M5, Decision 1 (why this lands
inline in the existing class, not a new one), Decision 3 (tolerance/dwell
as file-local constants), Open Question 2 (dwell-vs-STOP_TIME budget —
this ticket resolves it as a concrete assertion, item below).

Depends on 001 — needs `config_.heading_kp`/`config_.heading_kd` to read.

## Acceptance Criteria

**The cascade**

- [ ] Each tick, for PRE_PIVOT/TERMINAL_PIVOT only (NOT BLEND, NOT
      TRANSLATE): sample desired `(theta_desired, omega_desired)` from
      `rotational_` at `rotationalElapsed(now)`; derive `theta_measured` as
      the encoder-differential heading relative to the phase's OWN
      baseline — `((encRight.position.val - encLeft.position.val) -
      baseline_.encDiff0) / trackwidth_` — matching the same
      relative-to-phase-start convention `baseline_.encDiff0` already
      establishes for the existing divergence-replan math; derive
      `omega_measured` as `(encRight.velocity.val - encLeft.velocity.val) /
      trackwidth_`, falling back to the plan-sampled `omega_desired` when
      either wheel's `velocity.has` is false (mirrors
      `maybeReplanPivot()`'s existing reanchor-seed fallback exactly).
- [ ] Commanded `omega = omega_desired + config_.heading_kp *
      (theta_desired - theta_measured) + config_.heading_kd *
      (omega_desired - omega_measured)`, replacing the raw
      `rotational_.sample(...).velocity` currently returned for these two
      phases.

**Tolerance/dwell completion**

- [ ] New file-local `constexpr` constants in `segment_executor.cpp`:
      `kHeadingTol = 0.00873f` (~0.5°, `[rad]`), `kHeadingRateTol =
      0.0175f` (~1°/s, `[rad/s]`), `kHeadingDwellMs = 150` (150 ms — within
      the issue's suggested 100-200 ms range), documented with the same
      style of derivation comment as the existing `kDivergenceThreshold`
      family — labeled explicitly as a first-cut, code-edit-iterable
      constant (architecture-update.md Decision 3), NOT a `PlannerConfig`
      field.
- [ ] Completion for PRE_PIVOT/TERMINAL_PIVOT: `|rotationalTarget_ -
      theta_measured| < kHeadingTol` AND `|omega_measured| <
      kHeadingRateTol`, held continuously for `>= kHeadingDwellMs`,
      REPLACES `STOP_ROTATION`'s role for these two phases —
      `STOP_ROTATION` is no longer appended to `stops_[]` in
      `beginPrePivot()`/`beginTerminalPivot()`. `STOP_TIME` stays appended,
      unchanged, as the independent stall/non-convergence backstop.
- [ ] The dwell timer resets whenever the AND condition goes false, and is
      (re)initialized at phase start (`beginPrePivot()`/
      `beginTerminalPivot()`).
- [ ] Once the gate fires, the phase completes through the SAME
      `stopping_`/`advancePhase()` machinery already in place (no second
      completion pathway) — verified by the no-reverse-creep item below.
- [ ] **Dwell-vs-STOP_TIME budget** (Open Question 2): a sim assertion
      proves the added dwell (≤200 ms) does not cause the `STOP_TIME`
      safety net's own nominal-duration budget (`beginPrePivot()`/
      `beginTerminalPivot()`'s `nominal * 2.0f + 2000.0f` formula) to be
      exhausted before the tolerance+dwell gate can fire, for a
      representative SLOW (low-ceiling) turn.

**Replan retirement (SUC-002)**

- [ ] `maybeReplanPivot()`'s sub-gross (`kRotDivergenceThreshold`,
      EXTEND-only) branch becomes a no-op for PRE_PIVOT/TERMINAL_PIVOT
      specifically. The gross-divergence (`kRotGrossDivergenceThreshold`,
      reanchor) branch is UNCHANGED and still live for these phases.
      BLEND's own replan suppression (already disabled via
      `phaseReplanDeadline_ == chain instant`) is untouched.
      `maybeReplanTranslate()` (linear channel) is untouched.

**Sim acceptance — `tests/sim/unit/segment_executor_harness.cpp`**

- [ ] New scenario: the PD correction term is nonzero and in the
      correcting direction when a deliberately-lagging or
      deliberately-leading plant variant is driven against the executor
      (proves the loop is actually closing, not a no-op).
- [ ] New scenario: tolerance+dwell completion does NOT fire prematurely
      mid-cruise (while `|target error|` or `|rate|` still exceeds
      tolerance) and DOES fire once both conditions hold for the dwell
      window.
- [ ] New scenario (SUC-002, dwell-vs-budget): the dwell-vs-STOP_TIME
      budget assertion itemized above, for a representative slow turn.
- [ ] New scenario (SUC-002, stall-protection): holding one wheel's
      encoder reading artificially fixed (a simulated stall) against a
      nonzero PRE_PIVOT or TERMINAL_PIVOT command proves the
      gross-divergence reanchor STILL FIRES within the same ~2-pass budget
      as today.
- [ ] New scenario (SUC-002, replan retirement): under NOMINAL tracking lag
      (the kind the pre-sprint code's sub-gross EXTEND branch WOULD have
      fired on) proves that branch no longer fires for PRE_PIVOT/
      TERMINAL_PIVOT post-sprint.
- [ ] The EXISTING `scenarioNoReverseCreepInTerminalDecelTrace` regression
      scenario (094-001's named regression gate) is re-run UNMODIFIED and
      stays green — the literal-`0.0f` snap on rotational convergence and
      the "sampled omega never changes sign" invariant both still hold
      with the PD cascade live.
- [ ] Every other existing `segment_executor_harness.cpp` scenario
      (straight segment, translate-then-terminal-pivot, pure in-place
      turn, auto-decel-stays-idle, stop-mid-TRANSLATE) stays green;
      tolerances re-verified — note any tolerance changes explicitly in
      this ticket's completion notes (the old dead-time-projected-firing
      widened tolerances may no longer apply to PRE_PIVOT/TERMINAL_PIVOT
      now that they use the tolerance+dwell gate instead of that firing
      path).
- [ ] TRANSLATE-phase behavior (`maybeReplanTranslate()`, `STOP_DISTANCE`
      completion) is provably untouched — the existing TRANSLATE scenarios
      pass unmodified, no new coverage needed.
- [ ] Full `uv run python -m pytest` stays green, no regression from the
      pre-ticket baseline (the sim plant has ~no tracking asymmetry, so
      this must be a no-op-to-improvement per architecture-update.md's own
      Impact note — a sim regression here means the gains/constants need
      adjustment, not that the mechanism is wrong).
- [ ] `just build-sim` and `just build-clean` both succeed.

## Testing

- **Existing tests to run**: full `uv run python -m pytest`; explicit
  focus on `tests/sim/unit/segment_executor_harness.cpp`'s existing 6
  scenarios.
- **New tests to write**: the 6 new scenarios itemized above (PD
  correction direction, no-premature-completion, dwell-vs-STOP_TIME
  budget, stall-still-fires, nominal-lag-no-longer-fires, plus the
  no-reverse-creep re-confirmation of the existing scenario).
- **Verification command**: `uv run python -m pytest`.

## Implementation Plan

**Approach**: Surgical edit inside `segment_executor.cpp`'s existing
rotational tick branch (the `else` branch of `tick()`, currently shared by
PRE_PIVOT/TERMINAL_PIVOT) and `maybeReplanPivot()`; no new files, no new
classes (architecture-update.md Decision 1). Factor the measured-heading/
measured-rate derivation into a small private helper if useful for both the
PD term and the completion gate (both need `theta_measured`) — implementer's
judgment on exact factoring; the acceptance criteria constrain BEHAVIOR,
not internal shape.

**Files to modify**: `source/motion/segment_executor.h` (new dwell-timer
member, new tolerance/dwell constants), `source/motion/segment_executor.cpp`
(the PD law, the completion gate, `maybeReplanPivot()`'s narrowed scope,
`beginPrePivot()`/`beginTerminalPivot()`'s stop-set no longer appending
`STOP_ROTATION`), `tests/sim/unit/segment_executor_harness.cpp` (new
scenarios).

**Files to create**: none.

**Testing plan**: as above — sim-only ticket, no firmware/hardware
verification (that is ticket 003's job).

**Documentation updates**: none beyond in-code comments —
`architecture-update.md` already documents the design; this ticket
implements it, matching this file's own existing convention of carrying
the "why" in doc comments (as it already does for the divergence replan/
dead-time/graceful-decel machinery this ticket edits).
