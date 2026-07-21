---
id: "116"
title: "State estimator v1: encoder-only ZOH, capture script, and validation notebook"
status: roadmap
branch: sprint/116-state-estimator-v1-encoder-only-zoh-capture-script-and-validation-notebook
worktree: false
use-cases: []
issues: []
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 116: State estimator v1: encoder-only ZOH, capture script, and validation notebook

## Goals

Sprint 2 of the predict-to-now odometry arc (see
`clasi/issues/predict-to-now-odometry-estimator-ring-capture-dump-validation-trajectory-controller.md`).
Build `App::StateEstimator` v1 — encoder-only zero-order-hold (ZOH)
extrapolation over the sprint-115 rings — with fail-closed config keys, a
bench capture script, and a leave-one-out RMS validation notebook, plus a
`libfirmware_host` cross-check. Requires sprint 115 (the `Measurements`
rings + ring-dump command arm) to be closed and bench-proven first.

## Problem

Sprint 115 gives the firmware a bounded history of timestamped
measurements per source, but nothing yet answers "what is the state
(wheel distance/velocity, body pose/twist) right now" between
measurements — every existing predict-to-now logic in the tree is
heading-only and duplicated ad hoc (`HeadingSource::headingLead()`). A
controller cannot be built on an unvalidated prediction model; this
sprint's whole job is to build the v1 model and prove its error
characteristics on real bench data before anything consumes it.

## Solution

- `App::StateEstimator` (`src/firm/app/state_estimator.{h,cpp}`): pure
  computation over published ring records, never touches the I2C bus.
  Per-wheel `WheelEstimate{distance, velocity, basisStamp, valid}` and
  `BodyEstimate{x, y, heading, v_x, v_y, omega, basisStamp, valid}` as
  peers (not one derived from the other). API: `wheelAt(wheel, t)`,
  `bodyAt(t)`, `whereAmI()` (= `bodyAt(now)`), `wheelNow(wheel)`,
  `reset(x, y, heading)`, `innovations()`.
  v1 = ZOH: `distance = basis.position + basis.velocity * age`; heading =
  fused heading + fused omega * age (`headingLead()`'s equation promoted
  to full state, then `headingLead()` itself is retired in favor of this).
- Fusion v1: complementary blend per channel across the source rings
  (weights for OTOS heading/omega; `w_xy`/`w_ext` exist but default 0 —
  external fusion is sprint 118). All weights are fail-closed boot keys
  (`data/robots/*.json` → `gen_boot_config.py`) + live-tunable via
  `handleConfig()`. Not an EKF.
- Cycle placement: kPace block, after `applyOtosSample()`/
  `odom_.integrate()`, before `pilot_.plan()`. Greenfield alongside the
  legacy path — `App::Odometry`/`HeadingSource` keep feeding Pilot/TLM
  unchanged until sprint 119 switches consumers.
- `src/tests/bench/estimator_bench_run.py`: sends motion commands at
  varied speeds/durations (both directions, turns, straights), fills the
  rings, dumps via 115's debug commands.
- `src/tests/notebooks/estimator_validation.ipynb`: leave-one-out,
  one-step-ahead walk over the dumped rings + RMS analysis broken out by
  pattern phase (steady, ramp, reversal, pivot) + position-integration
  projection of accumulated error over a leg. Secondary cross-check:
  replay the same rings through the firmware estimator compiled into
  `libfirmware_host.dylib`, confirm it matches the notebook to float
  noise.
- Rebaseline-discontinuity absorption: a hard re-anchor (encoder zero,
  `reset()`) must not leave the estimator producing a garbage prediction
  across the discontinuity.

## Success Criteria

- Sim first, then stand: capture → dump → one-step-ahead walk.
- RMS ≈ measurement noise at constant velocity.
- ZOH lag signature (`a·k` velocity error, `½a·k²` distance error during
  ramps) matches theory — this is the first thing the notebook confirms,
  and decides whether a fit-based predictor is warranted later.
- Position-error integration yields a leg-level accumulated-error
  projection.
- Accept thresholds themselves are NOT pre-committed — the stakeholder
  ratifies them from the real RMS tables this sprint produces (full
  ratification gate is sprint 118; this sprint's own gate is "the numbers
  exist and match the ZOH lag theory," not a pass/fail threshold).

## Scope

### In Scope

- `App::StateEstimator` v1 (encoder-only path; OTOS heading/omega fusion
  weight live but external stays 0).
- Fail-closed config keys for fusion weights + staleness thresholds.
- `estimator_bench_run.py` capture script.
- `estimator_validation.ipynb` leave-one-out RMS notebook.
- `libfirmware_host` cross-check.
- Rebaseline-discontinuity handling.

### Out of Scope

- Fake OTOS (sprint 117) — this sprint validates against whatever OTOS
  presence the bench already has (AUTO fallback to encoder heading is
  fine; the estimator's OTOS-fusion weight is simply untested against
  real OTOS data until 117 lands).
- `PoseFix`/external fusion, threshold ratification gate (sprint 118).
- Trajectory controller / any change to `Motion::Executor` completion
  behavior (sprint 119) — `Odometry`/`HeadingSource` keep driving Pilot
  unchanged this sprint.

## Test Strategy

Sim first, then stand, per the arc's own stated methodology (see the arc
issue's "Verification" section) — capture → dump → leave-one-out
one-step-ahead walk → RMS analysis, cross-checked against
`libfirmware_host`. `uv run python -m pytest` + sim suite; `just
build-clean`; `mbdeploy deploy`; hardware bench gate per
`.claude/rules/hardware-bench-testing.md`.

## Architecture

(Architecture for this sprint's change, sized to the change — a
one-paragraph note for a trivial sprint, a fuller write-up with
component/data-model detail for a substantial one. May read "N/A —
trivial" when the change has no architectural impact.)

### Architecture Overview

(High-level structure and component relationships, if applicable.)

### Design Rationale

(Significant decisions with alternatives considered and reasoning, if
applicable.)

### Migration Concerns

(Data migration, backward compatibility, deployment sequencing — or
"None" if not applicable.)

## Use Cases

(Use cases sized to the change — may read "N/A — trivial" for small
sprints that don't warrant new or updated use cases.)

### SUC-001: (Title)
Parent: UC-XXX

- **Actor**: (Who)
- **Preconditions**: (What must be true before)
- **Main Flow**:
  1. (Step)
- **Postconditions**: (What is true after)
- **Acceptance Criteria**:
  - [ ] (Criterion)

## GitHub Issues

(GitHub issues linked to this sprint's tickets. Format: `owner/repo#N`.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [ ] Sprint planning document is complete (sprint.md, including its
      Architecture and Use Cases sections)
- [ ] Architecture review passed (or skipped, for changes with no
      architectural impact)
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|

Tickets execute serially in the order listed.
