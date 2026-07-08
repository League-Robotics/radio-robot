---
id: '001'
title: Drivetrain owns motor-observation port resolution
status: open
use-cases: [SUC-001]
depends-on: []
github-issue: ''
issue: drivetrain-owns-motor-observation-resolution.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Drivetrain owns motor-observation port resolution

## Description

Move the `p.left - 1`/`p.right - 1` port→cell resolution for the bound wheel
pair from `MainLoop::tick()` into `Subsystems::Drivetrain::tick()`, where
`ports_` (the port binding) already lives. Rename the blackboard field
`bb.motor` → `bb.motors` (stakeholder decision, 2026-07-07 — it is an array
of per-port observations, so the plural is correct). Add a range assert so
an out-of-range bound port cannot silently walk off the array. This is an
ownership-split fix, not a readability nit: Drivetrain owns the port
binding but the loop performed the *use* of it. Independent of every other
ticket in this sprint — do it first (they all share `main_loop.cpp` and
must serialize).

**Do NOT** pass a `Hardware&` into `Drivetrain::tick()` — considered and
rejected (see architecture-update.md Decision 1): it would break the x[k]
committed-snapshot discipline the two-plane ordered-tick model
(sprints 060/087) exists to enforce, and reintroduce direct-call coupling
the blackboard command plane replaced.

## Acceptance Criteria

- [ ] `Rt::Blackboard::motor` is renamed to `Rt::Blackboard::motors` in
      `source/runtime/blackboard.h`; every reference in the tree (grepped
      repo-wide, not only the source issue's stated Scope) compiles against
      the new name.
- [ ] `Subsystems::Drivetrain::tick()`'s signature changes to accept the
      whole per-port motor-observation array instead of two individual
      `msg::MotorState` references; it resolves `ports_.left - 1` /
      `ports_.right - 1` internally.
- [ ] A range assert (`ports_.left`/`ports_.right` within the array's bound)
      is added inside `Drivetrain::tick()`, where none existed before.
- [ ] `MainLoop::tick()`'s call to `drivetrain_.tick(...)` passes the array
      directly; the `- 1` port arithmetic no longer appears at that call
      site. The loop's own `p = drivetrain_.ports()` local is still needed
      for its OTHER two call sites (`poseEstimator_.tick()`/
      `planner_.tick()`) and stays unchanged — only the Drivetrain call
      site's own indexing moves.
- [ ] `uv run python -m pytest tests/sim` is green, the exact same
      pass/fail set as before this ticket.
- [ ] No wire-observable difference in driven speed/ratio-governor behavior
      for any drive verb (S/T/D/R/TURN/RT/G).

## Implementation Plan

**Approach**:
1. Rename `bb.motor` → `bb.motors` in `blackboard.h` (field + every doc
   comment referencing it).
2. Grep the WHOLE repo for `bb.motor` / `.motor[` — not just
   `main_loop.cpp` — per this project's own "rename sprint: latent
   call-site breakage" lesson (earlier rename sprints found later tickets
   discovering earlier renames still unconverged at real call sites hidden
   by mocks/tests). Update every hit: `main_loop.cpp`'s COMMIT loop
   (`bb.motor[port-1] = hardware_.state(port)`), the Drivetrain/
   PoseEstimator/Planner call sites, and any test harness constructing or
   reading a `Rt::Blackboard` directly (e.g.
   `tests/sim/unit/runtime_blackboard_harness.cpp`,
   `dev_loop_pose_estimator_harness.cpp`,
   `main_loop_order_independence_harness.cpp`, `planner_harness.cpp`).
3. Change `Drivetrain::tick()`'s signature to take the full per-port array.
   Prefer a pointer + count (e.g. `const msg::MotorState* motors,
   uint32_t motorCount`) over a fixed-size array bound to
   `Subsystems::Hardware::kPortCount` — `drivetrain.h` currently depends on
   nothing outside `hal/capability/hal_command.h` and `messages/`; a
   pointer+count avoids adding a new compile-time dependency on
   `subsystems/hardware.h` just to name the array's bound.
4. Inside `Drivetrain::tick()`, resolve
   `leftObs = motors[ports_.left - 1]` / `rightObs = motors[ports_.right - 1]`
   and assert both indices are in `[1, motorCount]` before indexing.
5. Update `main_loop.cpp`'s Drivetrain call site to pass `bb.motors`
   (and its count) directly; leave the `p = drivetrain_.ports()` local in
   place for the two other call sites in the same function.

**Files to modify**: `source/runtime/blackboard.h`,
`source/runtime/main_loop.cpp`, `source/subsystems/drivetrain.h`,
`source/subsystems/drivetrain.cpp`, plus any test harness under
`tests/sim/unit/` a repo-wide grep for `bb.motor`/`Drivetrain::tick(`/
`drivetrain.tick(`/`.motor[` turns up.

**Documentation updates**: none beyond the code's own doc comments (this
is an internal rename/relocation; no user-facing docs reference `bb.motor`).

## Testing

- **Existing tests to run**: `uv run python -m pytest tests/sim` (full
  suite, ~3–13 min). Specifically watch anything backed by
  `runtime_blackboard_harness.cpp` or `main_loop_order_independence_harness.cpp`,
  and any Drivetrain-specific unit test.
- **New tests to write**: none required (behavior-preserving). A small unit
  test exercising an out-of-range bound port against the new range assert
  is a reasonable defensive addition at the implementer's discretion, not
  required for acceptance.
- **Verification command**: `uv run python -m pytest tests/sim`
