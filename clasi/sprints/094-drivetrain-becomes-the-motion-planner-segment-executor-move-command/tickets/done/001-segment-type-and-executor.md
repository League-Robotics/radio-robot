---
id: "094-001"
title: "Segment type + SegmentExecutor (the lift)"
status: done
use-cases: ["SUC-001"]
depends-on: []
issue: drivetrain-becomes-the-motion-planner-segment-executing-subsystem.md
---

# 094-001: Segment type + SegmentExecutor (the lift)

## Description

Create the new, pose-free `Motion::Segment` POD type and a new
`Motion::SegmentExecutor` class that lifts the non-GOTO internals of
`Subsystems::Planner` (two `Motion::JerkTrajectory` channels, encoder-only
stop-condition evaluation, `Motion::MotionBaseline` capture, the
divergence replan and its compile-split dead-time, the presolved
decel-to-zero) and adds the one genuinely new piece of control logic this
sprint needs: a 3-phase PRE_PIVOT → TRANSLATE → TERMINAL_PIVOT sequencer
that turns one `Segment` into a chain of single-channel Ruckig solves.

This ticket is **read-only from `planner.cpp`** — it lifts logic by
reference, it does not modify `Subsystems::Planner` itself (that happens in
094-002). It has no dependency on `Subsystems::Drivetrain` or the
blackboard; it is built and tested in isolation, exactly like
`Motion::JerkTrajectory`/`Subsystems::Planner` already are.

Reference: `clasi/sprints/094-.../architecture-update.md` §3
(`Motion::SegmentExecutor`), the segment-shape section, and Design
Rationale — this ticket does not re-derive those decisions, only implements
them.

## Acceptance Criteria

- [x] `source/motion/segment.h` defines `Motion::Segment` exactly per the
      architecture doc's field list (`distance`, `direction`,
      `finalHeading` + `speedMax`/`accelMax`/`jerkMax`/`yawRateMax`/
      `yawAccelMax`/`yawJerkMax`, all `// [unit]`-tagged, no unit suffixes
      in any field name) — no `duration` field.
- [x] `source/motion/segment_executor.{h,cpp}` defines `Motion::
      SegmentExecutor`: `configure(const msg::PlannerConfig&)`,
      `start(const Motion::Segment&, now, trackwidth)` (or equivalent —
      trackwidth is needed to convert PRE_PIVOT/TERMINAL_PIVOT's
      `direction`/`finalHeading` deltas into a per-wheel-arc `STOP_ROTATION`
      threshold, mirroring `handleRT`'s existing `arc = |angle| *
      trackwidth/2` computation, motion_commands.cpp:614), `tick(now,
      encLeft, encRight) -> BodyTwist` (or the class's own chosen shape —
      the executor has no motor/blackboard/CODAL dependency), `active()`/
      `idle()`, and a way to query whether the whole segment (including its
      trailing graceful stop) has converged.
- [x] The 3-phase sequencer: PRE_PIVOT (skipped if `|direction| ≈ 0`) →
      TRANSLATE (skipped if `|distance| ≈ 0`) → TERMINAL_PIVOT (skipped if
      `finalHeading ≈ direction`), each phase a fresh `MotionBaseline` +
      Ruckig solve on the linear or rotational channel per the
      architecture doc's phase table.
- [x] Degenerate cases verified: a pure in-place turn (`distance=0`) skips
      TRANSLATE; a plain straight (`direction=0, finalHeading=0`) skips
      both pivots.
- [x] The compile-split dead-time (`kOutputHops`/`kDeadTime`, sim `2.0`=40ms
      / firmware `4.0`=80ms via `#ifdef HOST_BUILD`) is preserved verbatim
      — same values, same `#ifdef` split, ported from `planner.cpp:150-156`.
- [x] The divergence replan (`maybeReplanDistance`/`maybeReplanRotational`
      equivalents) is preserved with the same thresholds
      (`kDivergenceThreshold`/`kGrossDivergenceThreshold`/
      `kRotDivergenceThreshold`/`kRotGrossDivergenceThreshold`/
      `kMinReplanInterval`) ported from `planner.cpp:554-569`.
- [x] The presolved graceful decel-to-zero (`armDistanceStopDecel`/
      `armRotationalStopDecel`/`armVelocityStopDecel` equivalents,
      `planner.cpp:763-803`) is preserved, including the literal-`0.0f` snap
      on rotational convergence (`planner.cpp:964-966`) and its documented
      rationale (defeats the PID zero-deadband residual reverse-spin).
- [x] Host unit tests (new, alongside the existing `planner_harness.cpp`/
      `jerk_trajectory_harness.cpp` precedent) cover: a straight segment (no
      terminal pivot), a translate-then-terminal-pivot segment, a pure
      in-place turn (`distance=0`), auto decel-to-zero once the segment's
      own phases complete, a stop triggered mid-segment (mid-TRANSLATE),
      and — the sprint's named regression gate — **no reverse-creep** in
      the terminal decel trace (assert the sampled velocity never changes
      sign after the stop condition fires and before it settles to
      literal 0).
- [x] `just build-sim` succeeds; `uv run python -m pytest` stays green
      (no existing test is broken by this purely-additive ticket).

## Completion Note

Implemented as planned — copy-and-adapt from `planner.h`/`planner.cpp`, no
modification to either.

**Created**:
- `source/motion/segment.h` — `Motion::Segment` POD (9 floats, all
  `// [unit]`-tagged, no `duration` field).
- `source/motion/segment_executor.h`/`.cpp` — `Motion::SegmentExecutor`:
  `configure()`, `start(segment, now, trackwidth)`, `stop(now)` (forced
  graceful decel abandoning any remaining phases), `tick(now, encLeft,
  encRight) -> msg::BodyTwist3`, `active()`/`idle()`/`converged()`. Two
  `Motion::JerkTrajectory` channels; per-phase `MotionBaseline` capture
  (pose fields left dead/0, pose-free); `maybeReplanTranslate()`/
  `maybeReplanPivot()` (same 5 thresholds/guards as Planner's divergence
  replan); `kOutputHops`/`kDeadTime` ported verbatim including the
  `#ifdef HOST_BUILD` split; `armTranslateStopDecel()`/`armPivotStopDecel()`
  (presolved decel-to-zero) with the literal-`0.0f` snap on rotational
  convergence, same rationale comment ported. New 3-phase sequencer
  (PRE_PIVOT → TRANSLATE → TERMINAL_PIVOT) with degenerate-phase skipping;
  arc/angle conversion via `arcScale_ = trackwidth/2` (constant by
  construction, since this executor derives its own arc from its own
  angle target — unlike Planner's RT, which took an independently supplied
  arc).
- `tests/sim/unit/segment_executor_harness.cpp` + `test_segment_executor.py`
  — 6 scenarios (straight/no-pivot, translate-then-terminal-pivot, pure
  in-place turn, auto decel-to-zero stays idle, forced stop mid-TRANSLATE
  abandons the pending TERMINAL_PIVOT, and the named no-reverse-creep
  regression with an explicit literal-0.0f assertion), driven by a
  self-consistent zero-lag/zero-slip encoder-integrator plant (not a
  hand-derived closed form) to close the loop against the executor's own
  commanded twist.

**Verification**: `just build-sim` succeeds (the new files are picked up
automatically by `source/`'s recursive CODAL glob, no build-file edit
needed). `uv run python -m pytest tests/sim` — 38 passed (0 failed),
including the new harness. Full `uv run python -m pytest` — 306 passed, 10
failed + 90 errors, all pre-existing and confined to `tests/testgui/`
(`PySide6` not installed in this worktree's venv — a `uv sync --group gui`
gap, unrelated to this ticket); no `tests/sim` regression.

No AC could not be satisfied.

## Implementation Plan

**Approach**: Copy-and-adapt, not copy-and-modify-in-place — write fresh
`segment.h`/`segment_executor.{h,cpp}` files, reading `planner.h`/
`planner.cpp` as the reference for exact constants/thresholds/sequencing,
but restructuring the per-goal-kind dispatch (`DISTANCE`/`TIMED`/`TURN`/
`ROTATION`/`STREAM` cases) into the 3-phase sequencer's simpler shape (every
phase is either a DISTANCE-shaped or a turn-in-place-shaped solve — there
is no TIMED/VELOCITY/STREAM analog in a `Segment`, so `stageVelocityGoal()`'s
Pattern B has no equivalent here; only Pattern A — solve-to-rest-at-a-known-
target — is needed, since every phase's target is fully known when the
phase starts).

**Files to create**:
- `source/motion/segment.h` — `Motion::Segment` POD.
- `source/motion/segment_executor.h` — class declaration, phase enum,
  per-phase state.
- `source/motion/segment_executor.cpp` — implementation.
- A new host-unit-test file under `tests/sim/unit/` (mirroring
  `planner_harness.cpp`'s existing pattern) exercising the AC list above.

**Files to modify**: none — this ticket adds files only.

**Testing plan**: host-side unit tests only this ticket (no firmware/sim
integration yet — the executor has no caller until 094-004). Run via
`uv run python -m pytest` against whatever harness convention the existing
`planner_harness.cpp`/`jerk_trajectory_harness.cpp` tests use (check
`tests/sim/unit/CMakeLists.txt` or equivalent build wiring for the harness
pattern to match).

**Documentation updates**: none required by this ticket (architecture-update.md
already documents the design; no wire-protocol or ticket-facing doc changes
originate here).
