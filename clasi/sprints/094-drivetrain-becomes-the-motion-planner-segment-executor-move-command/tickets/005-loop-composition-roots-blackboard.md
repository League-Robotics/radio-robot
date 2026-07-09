---
id: "094-005"
title: "Loop + composition roots + blackboard (flash-size gate)"
status: open
use-cases: ["SUC-001", "SUC-002", "SUC-004"]
depends-on: ["094-002", "094-004"]
issue: drivetrain-becomes-the-motion-planner-segment-executing-subsystem.md
---

# 094-005: Loop + composition roots + blackboard (flash-size gate)

## Description

Wire 094-004's new `Drivetrain(Hardware&)` into both composition roots
(`source/main.cpp`, `tests/_infra/sim/sim_api.cpp` — the 1:1-mirror
invariant: both change in lockstep), reorder `Rt::MainLoop::tick()` to
`hardware_.serviceBus(...)` → `drivetrain_.tick(now, bb.segmentIn,
bb.driveIn)` → commit (deleting `routeOutputs()` — nothing left to route),
add `bb.segmentIn` (`Rt::WorkQueue<Motion::Segment, 8>`) to
`Rt::Blackboard`, and re-add boot-only jerk-limit config defaults in both
composition roots (093 deleted `defaultPlannerConfig()`; this ticket
re-adds a small equivalent, not the whole function's old scope).

This ticket carries the sprint's **top risk**: re-linking Ruckig into the
live tick path (via `Drivetrain` → `Motion::SegmentExecutor` →
`Motion::JerkTrajectory`) returns the firmware to roughly its pre-093
footprint. The `arm-none-eabi-size` before/after gate below is mandatory
and must be recorded in this ticket's own completion notes, not merely
mentioned in passing.

## Acceptance Criteria

- [ ] `main.cpp` constructs `Drivetrain drivetrain(hardware);` (order:
      `hardware` before `drivetrain`, matching the existing
      declaration-order convention) instead of the parameterless
      `Drivetrain drivetrain;`.
- [ ] `main.cpp` re-adds a small boot-only jerk config application (e.g.
      `drivetrain.configureMotion(defaultMotionConfig())` or folded into
      the existing `drivetrain.configure(dtConfig)` path — ticket
      executor's call on the exact shape) supplying `jMax ≈ 5000 mm/s^3`,
      `yawJerkMax ≈ 100 rad/s^3` (replacing the `0.0` trapezoid sentinel),
      applied once at construction — no runtime `SET`/`GET` path is
      revived.
- [ ] `tests/_infra/sim/sim_api.cpp`'s `SimHandle` makes the identical
      change (construction order, jerk defaults) — verified as a genuine
      lockstep mirror, not a divergent sim-only shortcut.
- [ ] `Rt::Blackboard` gains `Rt::WorkQueue<Motion::Segment, 8> segmentIn;`
      — verified to satisfy the existing "every Blackboard member is a
      host-safe POD" bar (`Motion::Segment` has zero CODAL dependency, per
      094-001's own AC).
- [ ] `Rt::MainLoop::tick()` becomes: `hardware_.serviceBus(now, ...)` →
      `drivetrain_.tick(now, bb.segmentIn, bb.driveIn)` → commit
      (`bb.motors[]`/`bb.drivetrain`). `routeOutputs()` and its declaration
      in `main_loop.h` are deleted.
- [ ] `bb.driveIn`'s doc comment is updated: it is now the S/STOP
      escape-hatch input to `Drivetrain` only (no more Planner producer, no
      more `routeOutputs()` consumer) — matches architecture-update.md's
      "What Changed" section.
- [ ] **Mandatory flash-budget gate**: `arm-none-eabi-size build/MICROBIT`
      run and recorded (in this ticket's own body, as a completion note)
      BEFORE this ticket's changes are built (i.e. against the 093
      baseline, Ruckig-stripped) and AFTER (Ruckig re-linked into the live
      tick path). The image must fit flash with headroom comparable to the
      last pre-093 Ruckig-in-use build (~43 KB/11.7% free was the
      historical figure — record the actual after-figure, do not simply
      assert it matches).
- [ ] A sim end-to-end test drives one segment through the full composition
      root (construct `SimHandle`, post a segment via the internal API this
      ticket exposes — or, if 094-006 hasn't landed yet, a direct
      `bb.segmentIn.post(...)` + `sim_tick()` loop) and confirms it executes
      and settles, proving the loop reorder + blackboard wiring is correct
      end to end before the wire command surface (094-006) is added on top.
- [ ] `just build` (firmware) and `just build-sim` succeed.
- [ ] `uv run python -m pytest` stays green, including the 093 four-verb
      focused suite (`PING`/`HELLO`/`S`/`STOP` still work unchanged).

## Implementation Plan

**Approach**: This ticket is the "make it all compile and run together"
integration point — 094-001 through 094-004 land in isolation; this ticket
wires them into the two composition roots and the loop. Keep the diff to
`main_loop.cpp` minimal (delete `routeOutputs()`, reorder the two tick
calls) — do not touch `commit()`'s existing per-port loop.

**Files to modify**:
- `source/runtime/main_loop.h`/`.cpp` — delete `routeOutputs()`; reorder
  `tick()`.
- `source/runtime/blackboard.h` — add `segmentIn`; update `driveIn`'s doc
  comment.
- `source/main.cpp` — construct `Drivetrain drivetrain(hardware);`; re-add
  boot jerk defaults.
- `tests/_infra/sim/sim_api.cpp` — identical `SimHandle` changes.

**Testing plan**: `just build` + `just build-clean` (to get an accurate,
non-incremental `arm-none-eabi-size` reading — see
`stale-incremental-build-on-volumes` project knowledge: incremental builds
can go stale silently) for the flash-size gate; `just build-sim` +
`uv run python -m pytest` for the sim gate; a new end-to-end sim test
posting directly to `bb.segmentIn` (bypassing the wire layer, which
094-006 has not yet added) to prove the loop-reorder + blackboard wiring
works before the command surface is layered on.

**Documentation updates**: none beyond the doc-comment updates already
listed in the AC (`bb.driveIn`'s comment, `main_loop.h`'s class comment
describing the new two-step sequence).
