---
id: '001'
title: Drivetrain owns motor-observation port resolution
status: done
use-cases:
- SUC-001
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

- [x] `Rt::Blackboard::motor` is renamed to `Rt::Blackboard::motors` in
      `source/runtime/blackboard.h`; every reference in the tree (grepped
      repo-wide, not only the source issue's stated Scope) compiles against
      the new name.
- [x] `Subsystems::Drivetrain::tick()`'s signature changes to accept the
      whole per-port motor-observation array instead of two individual
      `msg::MotorState` references; it resolves `ports_.left - 1` /
      `ports_.right - 1` internally.
- [x] A range assert (`ports_.left`/`ports_.right` within the array's bound)
      is added inside `Drivetrain::tick()`, where none existed before.
- [x] `MainLoop::tick()`'s call to `drivetrain_.tick(...)` passes the array
      directly; the `- 1` port arithmetic no longer appears at that call
      site. The loop's own `p = drivetrain_.ports()` local is still needed
      for its OTHER two call sites (`poseEstimator_.tick()`/
      `planner_.tick()`) and stays unchanged — only the Drivetrain call
      site's own indexing moves.
- [x] `uv run python -m pytest tests/sim` is green, the exact same
      pass/fail set as before this ticket.
- [x] No wire-observable difference in driven speed/ratio-governor behavior
      for any drive verb (S/T/D/R/TURN/RT/G).

## Completion Notes (2026-07-08)

Implemented exactly per plan:

- `source/runtime/blackboard.h`: `Blackboard::motor[]` → `motors[]` (field +
  file-header doc comment).
- `source/subsystems/drivetrain.h`/`.cpp`: `tick()` signature changed from
  `(uint32_t now, const msg::MotorState& leftObs, const msg::MotorState&
  rightObs, Mailbox<DrivetrainCommand>& driveIn)` to `(uint32_t now, const
  msg::MotorState* motors, uint32_t motorCount, Mailbox<DrivetrainCommand>&
  driveIn)` — a pointer+count rather than a fixed-size array bound to
  `Subsystems::Hardware::kPortCount`, per the plan's own rationale (keeps
  `drivetrain.h` free of a new dependency on `subsystems/hardware.h`).
  Internally resolves `ports().left/right - 1` against `motors[]`, guarded
  by two `assert()` (`<cassert>`) range checks against `motorCount` before
  indexing — the codebase has no existing runtime-assert convention (only
  `static_assert` elsewhere), so this uses the standard library facility
  directly; `-Wno-unused-parameter` (already set project-wide) covers the
  NDEBUG-stripped case.
- `source/runtime/main_loop.cpp`: the Drivetrain call site now passes
  `bb.motors, kPortCount` with no `- 1` arithmetic; `p =
  drivetrain_.ports()` is retained unchanged for the
  `poseEstimator_.tick()`/`planner_.tick()` call sites and the COMMIT loop,
  none of which are in this ticket's scope.
- Repo-wide `bb.motor[]`/`b.motor[]`/doc-comment sweep (not just the
  issue's stated Scope) also touched: `source/commands/dev_commands.cpp`
  (2 call sites), `source/commands/dev_commands.h` (2 doc comments),
  `source/telemetry/tlm_frame.cpp`/`.h` (2 call sites + 2 doc comments).
- Test harnesses updated for the new `Drivetrain::tick()` signature and/or
  the `bb.motors` rename: `tests/sim/unit/drivetrain_harness.cpp` (all 7
  direct `dt.tick()` call sites — added a small `motorsAt()` helper so each
  scenario places its `obsVelocity()` values at the same 1-based port
  indices it configured via `configWithPorts()`), `dev_loop_pose_estimator_
  harness.cpp` and `main_loop_order_independence_harness.cpp` (both build a
  small local `motors[kPortCount]` array from their existing
  `committedLeft`/`committedRight` cells at the bound-pair indices before
  calling `drivetrain.tick()`), `runtime_blackboard_harness.cpp` and
  `tlm_frame_harness.cpp` (`bb.motor[...]` → `bb.motors[...]`, plus
  comments).
- Final repo-wide grep (`bb\.motor\b`, `b\.motor\b`, `[.>]motor\[`,
  excluding `motorCaps`/`motorConfig`/`motorResetIn`/`motorIn[`) returns
  zero hits in `source/`/`tests/` — the rename is complete. Historical,
  already-closed sprint docs (`clasi/sprints/done/087-.../architecture-
  update*.md` and their archived tickets/issues) and `docs/architecture/
  architecture-update-087.md` still show `bb.motor[...]` — left untouched
  deliberately (frozen historical record of that sprint's own design
  discussion, out of this ticket's scope per its own "Documentation
  updates: none" note).
- `Hardware&` was NOT passed into `Drivetrain::tick()` (the rejected
  alternative) — the observation-array approach was used throughout.

**Verification**: `uv run python -m pytest tests/sim` →
`308 passed, 2 xfailed in 99.90s (0:01:39)` (the 2 xfails are the
pre-existing, unrelated `tests/sim/system/test_tour_geometry.py` xfails —
same pass/xfail set as before this ticket). Also ran the 5 directly
affected tests in isolation first
(`test_drivetrain.py`, `test_dev_loop_pose_estimator.py`,
`test_main_loop_order_independence.py`, `test_tlm_frame.py`,
`test_runtime_blackboard.py`) — all green — and did a standalone
`cmake --build` of `tests/_infra/sim/build` (the shared `firmware_host`
host lib covering `main_loop.cpp`/`dev_commands.cpp`/`tlm_frame.cpp`) to
confirm clean compilation before the full suite run.

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
