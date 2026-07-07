---
id: "006"
title: "CommandRouter and pointerless command-family translators"
status: open
use-cases: [SUC-003, SUC-004, SUC-006]
depends-on: ["003", "004", "005"]
github-issue: ""
issue: plan-file-a-design-issue-blackboard-architecture-state-objects-command-queues.md
# completes_issue: Controls whether linked issues are archived when this ticket
# is moved to done. Default: true (archive when all referencing tickets are done).
# Set to false (scalar) to suppress archival for ALL linked issues on this ticket.
# Set to a mapping {filename.md: false} to suppress archival per issue filename.
# Use false for tickets that partially address a multi-sprint umbrella issue.
completes_issue: true
# exception: Written by a lower agent when it cannot proceed (see architecture §exception-protocol).
# exception:
#   thrown_by: "programmer"          # "programmer" | "sprint-planner"
#   thrown_at: "2026-05-07T14:23:00Z"
#   attempted: |
#     Description of what was attempted before giving up.
#   conflict: "architecture-update.md §3 — reason the agent is blocked"
#   surface: "internal"              # "user-visible" | "internal"
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# CommandRouter and pointerless command-family translators

## Description

Implement `CommandRouter` (`source/runtime/command_router.{h,cpp}`) and
rewrite the bodies of all six command families (dev, telemetry, motion,
config, pose, otos) so every handler becomes a pure translator against the
Blackboard: read state cells it needs, post a typed command onto the
appropriate queue, **never** hold or dereference a `Subsystems::*` pointer.
Delete the six `*State` structs (`DevLoopState`, `TelemetryState`,
`MotionLoopState`, `ConfigCommandState`, `PoseCommandState`,
`OtosCommandState`) and their subsystem-pointer fields, the three
config-shadow caches, and the cross-family `sTimeoutWatchdog` pointer.

`SET`'s synchronous validate-then-`ERR` behavior is preserved exactly
(reads the current-config state cell, folds+validates the candidate,
replies `ERR` immediately on failure — nothing enqueued; posts a
`ConfigDelta` and replies `OK` on success — Decision 3). `DEV DT`'s
`driveIn` posting and `SI`/`ZERO`'s `poseResetIn`/`motorResetIn` fan-out
both read `Drivetrain`'s authority state / port bindings from the snapshot
(never a `Drivetrain*`): `DEV DT` implements Decision 1's producer-side
authority gate (only posts to `driveIn` when `DEV DT` currently holds
authority, per ticket 003's published state cell), and `SI`/`ZERO`
implement Decision 7's **router-side** half of the state-reset split
(`PoseEstimator`'s own drain, from ticket 004, is the other half).

## Acceptance Criteria

- [ ] `CommandRouter::route(statement, bb)` dispatches every existing wire
      verb (`SET`, `DEV *`, `S`/`T`/`D`/`G`/`R`/`TURN`/`RT`/`VW`/`_VW`/`X`/
      `STOP`, `SI`, `ZERO enc`, `OI`/`OL`/`OA`/`OZ`, `GRIP`, `P`/`PA`,
      `GET`) to the correct blackboard queue or a direct state-cell read,
      with **zero regression** in reply text/timing versus today
      (`docs/protocol-v2.md` is unaffected).
- [ ] `SET`'s validate-then-`ERR` happens synchronously in the handler
      (reads the published current-config cell per Decision 3, not the
      Configurator's internal pending-delta bookkeeping); only the accepted
      path posts a `ConfigDelta`.
- [ ] `DEV DT`'s drive-command posting checks `Drivetrain`'s published
      authority-mode state cell (ticket 003) before posting to `driveIn`,
      per Decision 1; confirm and preserve today's exact `DEV DT` authority
      contract (`dev_commands.cpp`) for what happens when authority is not
      held.
- [ ] `SI` fans out to `bb.poseResetIn` (`kSetPose`) and
      `bb.otosSetPoseIn`; `ZERO enc` fans out to `bb.poseResetIn`
      (`kResetBaseline`) and `bb.motorResetIn[left]`/`[right]` — all reading
      the port binding from the snapshot (`Drivetrain`'s state cell), never
      a `Drivetrain*`.
- [ ] The six `*State` structs (`DevLoopState`, `TelemetryState`,
      `MotionLoopState`, `ConfigCommandState`, `PoseCommandState`,
      `OtosCommandState`) and `DevLoop` no longer exist anywhere in
      `source/`.
- [ ] Grepping `source/commands/` for `Subsystems::` outside comments
      returns nothing (SUC-006's acceptance criterion, verified literally).
- [ ] The three config-shadow caches
      (`motorConfigShadow[]`/`drivetrainConfigShadow`,
      `drivetrainShadow`/`motorShadow[]`/`plannerShadow`, `configShadow`)
      and the cross-family `sTimeoutWatchdog` pointer no longer exist.
- [ ] Every existing command-family test (`test_config_registry.py`,
      `test_config_pose_set_otos_surface.py`, `test_pose_commands.py`,
      `test_otos_commands.py`, `test_otos_commands_nodev.py`,
      `test_dev_command_outbox.py`, `test_motion_commands*.py`,
      `test_protocol_roundtrips.py`, and their harnesses) passes against
      the rewritten translators with no wire-visible behavior change.

## Implementation Plan

**Approach.** New `source/runtime/command_router.{h,cpp}`. Rewrite
`source/commands/{dev,telemetry,motion,config,pose,otos}_commands.{h,cpp}`
bodies in place (same command-table registration shape, same
`CommandDescriptor`/`Commandable` interface, new pointerless internals).
This is the largest single ticket in the sprint by file count — the six
command families are planned as one ticket because they share one
indivisible cutover point (the six `*State` structs and `DevLoop` are
deleted together; a partial cutover would leave some families still
pointer-holding against a wiring shape `main.cpp` no longer provides). If
this proves too large for one focused session, the programmer may split by
command family (e.g. config+dev, then motion+pose+otos+telemetry) and flag
the split explicitly as a deviation, keeping the "all six cut over
together" invariant intact across the split.

**Files to modify:**
- `source/runtime/command_router.{h,cpp}` (new)
- `source/commands/dev_commands.{h,cpp}`
- `source/commands/telemetry_commands.{h,cpp}`
- `source/commands/motion_commands.{h,cpp}`
- `source/commands/config_commands.{h,cpp}`
- `source/commands/pose_commands.{h,cpp}`
- `source/commands/otos_commands.{h,cpp}`
- every test file/harness under `tests/sim/unit/` that exercises these six
  families

**Testing plan:**
- Run the full existing command-family test suite (listed above) against
  the rewritten translators, confirming byte-identical reply text for
  every existing scenario.
- Add a test asserting a `SET` candidate that fails validation leaves
  `bb.configIn` untouched (nothing queued on `ERR`).
- Verify (grep, and/or a structural test if a repo lint hook exists) that
  zero `Subsystems::` pointers are held anywhere in `source/commands/`.
- **Verification command**: `uv run pytest tests/sim/unit/test_config_registry.py tests/sim/unit/test_config_pose_set_otos_surface.py tests/sim/unit/test_pose_commands.py tests/sim/unit/test_otos_commands.py tests/sim/unit/test_otos_commands_nodev.py tests/sim/unit/test_dev_command_outbox.py tests/sim/unit/test_motion_commands.py tests/sim/unit/test_motion_commands_arc_turn.py tests/sim/unit/test_motion_commands_goto.py tests/sim/unit/test_protocol_roundtrips.py`

**Documentation updates:** `docs/protocol-v2.md` should need **no** changes
(wire contract unaffected) — confirm this remains true and note it
explicitly in the ticket's completion; any accidental wire drift found is
a regression to fix, not a spec update.
