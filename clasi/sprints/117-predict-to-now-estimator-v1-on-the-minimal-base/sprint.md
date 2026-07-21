---
id: '117'
title: Predict-to-now estimator v1 on the minimal base
status: roadmap
branch: sprint/117-predict-to-now-estimator-v1-on-the-minimal-base
worktree: false
use-cases: []
issues:
- predict-to-now-odometry-estimator-ring-capture-dump-validation-trajectory-controller.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 117: Predict-to-now estimator v1 on the minimal base

## Goals

**Re-scope note (carried from the source issue, 2026-07-21):** the
predict-to-now issue's original mechanism — on-chip measurement rings,
ring-dump commands, capture builds — is superseded by the minimal-firmware
gut (sprints 115-116): the tightened telemetry frame, timestamped and
emitted every loop iteration, logged host-side, is now the dataset. What
stands from the source issue and is in scope here: the estimator core
(`whereAmI()`/`stateAt(t)`), wheel + body peer estimates, ZOH v1
extrapolation, and the leave-one-out one-step-ahead RMS validation
methodology — now run over the host TLM log instead of dumped rings. Fake
OTOS, external/camera pose + clock sync, and the remaining-distance
trajectory controller are the issue's further-out goals; they are noted
here as roadmap context but are **not** detail-planned or built in this
sprint (see Out of Scope).

- Build `App::StateEstimator`: `wheelAt(wheel, t)`, `bodyAt(t)`,
  `whereAmI()` (= `bodyAt(now)`), `wheelNow(wheel)`, `reset(x, y,
  heading)`, `innovations()` — wheel and body state as peer first-class
  estimates, each with its own residual stream.
- Ship ZOH v1 extrapolation (`distance = basis.position + basis.velocity
  × age`; heading = fused heading + fused omega × age — the existing
  `headingLead()` equation promoted to full state) plus a v1
  complementary-blend fusion (config-tunable weights, staleness gating).
- Validate with the stakeholder's methodology: leave-one-out one-step-
  ahead RMS analysis, run in sim first and then over a real bench capture
  of the host-logged TLM stream (via sprint 115's `tlm_log.py`), broken
  out by pattern phase (steady, ramp, reversal, pivot).
- Wire the estimator into the loop's kPace block (after
  `applyOtosSample()`/`odom_.integrate()`) without regressing motion
  timing.

## Problem

Weeks of motion-control tuning never produced a completing tour; the two
standing blockers (turn non-termination, terminal straight-leg wedge) were
both terminal-behavior failures of the executor's completion machinery —
now deleted by sprints 115-116. The firmware has never fused measurements
or answered "where is the robot right now" — control consumed raw
per-cycle deltas, and `measurement_ring.h` existed but nothing published
into it. The abandoned predict-to-now arc's own ring-capture/dump plan is
now unnecessary: the post-gut minimal base already emits a complete
timestamped frame every cycle and the host already logs it (sprint 115),
so the estimator can be validated directly against that log.

## Solution

Build `App::StateEstimator` as pure computation over already-published
measurement state — no new on-chip rings, reading from the same frame
data sprint 115 already assembles into `RobotLoop::frame_` each cycle.
Wheel and body estimates are peers, each independently valid/stale. v1 is
plain zero-order-hold extrapolation from the newest sample; fusion v1 is a
simple complementary blend (not an EKF), weights fail-closed and
live-tunable via `handleConfig()`. Validate the way the stakeholder
specified: drive varied motion patterns, capture the TLM log, and for
every measurement k, exclude it, extrapolate from k−1, and compare against
actual k — walking the whole log, per stream — then RMS the one-step-ahead
errors by pattern phase and propagate them through position integration to
project leg-level accumulated error. Run this in sim first (fast
iteration, no hardware risk) and then on the bench. Detailed module
boundaries, exact file layout, and sim/bench sequencing are established at
Detail Mode for this sprint, re-derived from the post-gut/post-protocol
base rather than inherited wholesale from the source issue's original
(ring-oriented) per-stage sprint table.

## Success Criteria

- `App::StateEstimator` compiles into firmware and the host cross-check
  target (`libfirmware_host`), with wheel and body estimates each backed
  by their own residual stream.
- Leave-one-out one-step-ahead RMS analysis runs end-to-end over a real
  bench TLM-log capture spanning steady/ramp/reversal/pivot motion, both
  directions, turns and straights.
- The ZOH lag signature (`a·k` velocity error, `½a·k²` distance error
  during ramps) is checked against theory — this is the evidence that
  decides whether a fit-based (non-ZOH) predictor is warranted later.
- RMS tables and accept thresholds are reviewed and ratified by the
  stakeholder from the real data — not pre-committed before the capture.
- Estimator wired into the cycle at the correct placement with no
  measurable motion-timing regression (encoder tracking vs. commanded
  speed unchanged from pre-estimator runs).
- Sim and bench notebook results cross-checked against the firmware
  estimator replayed through `libfirmware_host.dylib` to float noise.

## Scope

### In Scope

- `App::StateEstimator` core: `wheelAt`/`bodyAt`/`whereAmI`/`wheelNow`/
  `reset`/`innovations`; `WheelEstimate`/`BodyEstimate` peer state
  structs.
- ZOH v1 extrapolation; v1 complementary-blend fusion with fail-closed,
  live-tunable weights; staleness gating.
- Cycle placement wiring (kPace block, after OTOS/odometry integration).
- Capture tooling and the leave-one-out RMS notebook, reading from the
  sprint-115 `tlm_log.py` CSV output (not on-chip ring dumps).
- Confirming `PING`'s `t=` clock-sync activation (landed with 115/116's
  protocol work) is sufficient for this sprint's needs, or identifying
  what remains — full external-pose clock-sync build-out is out of scope
  here (see below).

### Out of Scope (future work, noted but not planned here)

- **Fake OTOS** test device (`Devices::PoseSensor` extraction,
  `FakeOtos`, `ROBOT_FAKE_OTOS` build seam) — a later sprint once the
  estimator core is bench-proven.
- **External/camera pose source** (`PoseFix` revival + velocity extension
  + firmware consumer) and its clock-sync build-out beyond the `PING t=`
  activation already landing in 115/116.
- **The remaining-distance trajectory controller** — the source issue's
  stated end goal (replacing the deleted Executor's completion machinery,
  closing the turn-non-termination and terminal-wedge blocker issues) —
  is explicitly deferred until the estimator is bench-proven per the
  stakeholder's stated sequencing ("full arc planned up front, estimator
  first; the controller sprint is detailed only after the estimator gate").
- Any on-chip measurement-ring or ring-dump mechanism — superseded by the
  telemetry-log dataset per the re-scope note above.

## Test Strategy

`uv run python -m pytest` + sim suite; `just build-clean`; `mbdeploy
deploy` (hex by full UID); hardware bench gate per
`.claude/rules/hardware-bench-testing.md`. Core proof, sim first then
bench: capture a TLM-log CSV over varied motion patterns (steady, ramp,
reversal, pivot; both directions; turns and straights) via sprint 115's
logging tool; run the leave-one-out one-step-ahead walk per stream
(encoder, OTOS when present); RMS-analyze by pattern phase; propagate
per-step error through position integration to project leg-level
accumulated error; cross-check the same data replayed through
`libfirmware_host.dylib` against the notebook to float noise.

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
