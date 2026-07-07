---
id: '003'
title: 'Faceplate regularization: Drivetrain and Planner blackboard wiring'
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-006
depends-on:
- '002'
github-issue: ''
issue: plan-file-a-design-issue-blackboard-architecture-state-objects-command-queues.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Faceplate regularization: Drivetrain and Planner blackboard wiring

## Description

Adapt `Subsystems::Drivetrain` and `Subsystems::Planner`'s `tick()`
signatures to take blackboard-sourced queue arguments in place of their
current internal input path, per `architecture-update.md` Step 5's
"Modified" entries and the Faceplate reference example.
`Drivetrain::tick()` gains a `Rt::Mailbox<msg::DrivetrainCommand>& driveIn`
parameter it drains each pass (pop, latest-wins) instead of however it
currently receives its setpoint. `Planner`'s output edge
(`hasCommand()`/`takeCommand()`, unchanged in shape) is the *other*
producer of the same `driveIn` Mailbox, per Decision 1's authority-gated
arbitration.

This ticket's job is narrower than "implement Decision 1's arbitration" —
it only needs to make `Drivetrain`'s authority mode (`active()`/
`standby()`) *readable from a state cell*, so that the producer-side gate
(enforced in ticket 006's `CommandRouter` and ticket 007's loop
`routeOutputs`) can check "who currently has authority" without holding a
`Drivetrain*`. The gating logic itself is out of this ticket's scope.

## Acceptance Criteria

- [x] `Drivetrain::tick(uint32_t now, const msg::MotorState& leftObs, const
      msg::MotorState& rightObs, Rt::Mailbox<msg::DrivetrainCommand>&
      driveIn)` compiles, pops `driveIn` when non-empty, and drives the
      existing setpoint-governance logic unchanged otherwise — no change to
      the governed-output math itself.
- [x] `msg::DrivetrainState` (or another architecturally-appropriate state
      message, confirmed during implementation) exposes the current
      `active()`/`standby()` authority mode as a plain readable field, so a
      producer can decide whether it is allowed to post `driveIn` without
      holding a `Drivetrain*`.
- [x] `Drivetrain`'s output edge (`hasCommand()`/`takeCommand()` ->
      `Hal::DrivetrainToHardwareCommand`) is unchanged in shape (Decision 2
      defers the `motorIn[]` unpack to the loop's `routeOutputs`, ticket
      007).
- [x] `Planner`'s `tick()` signature and its own output edge
      (`hasCommand()`/`takeCommand()` -> `msg::DrivetrainCommand`) are
      unchanged in shape; documented in-code (a comment on the output edge)
      as the second producer of `driveIn` per Decision 1, cross-referencing
      `architecture-update.md`.
- [x] `source/subsystems/drivetrain.h` and `planner.h` include only
      `messages/*.h` and `runtime/queue.h` (the generic `Mailbox`/
      `WorkQueue` templates) — never `blackboard.h`, never each other's
      header (the "subsystems never include `blackboard.h`" boundary rule
      from the architecture's self-review).
- [x] Existing `tests/sim/unit/test_drivetrain.py` and `test_planner.py`
      (and the harnesses they drive) pass with the new signature, updated
      to construct a bare `Rt::Mailbox<msg::DrivetrainCommand>` directly —
      no `Blackboard` instance required (SUC-002's enumerable-dependency
      goal).

## Implementation Plan

**Approach.** Modify `drivetrain.{h,cpp}` and `planner.{h,cpp}` `tick()`
signatures; thread the new `Mailbox` parameter through to the existing
internal setpoint-consumption code path (today's mechanism is replaced by a
`driveIn.take()` at the top of `tick()` when `driveIn` is non-empty). Add
the authority-mode field to `msg::DrivetrainState`. Update the two
subsystems' own tests and harnesses to match.

**Files to modify:**
- `source/subsystems/drivetrain.{h,cpp}`
- `source/subsystems/planner.{h,cpp}`
- `source/messages/drivetrain.h` (add the authority-mode field to
  `DrivetrainState`)
- `tests/sim/unit/drivetrain_harness.cpp`, `tests/sim/unit/planner_harness.cpp`
- `tests/sim/unit/test_drivetrain.py`, `tests/sim/unit/test_planner.py`

**Testing plan:**
- Update both harnesses to construct a bare `Rt::Mailbox<msg::DrivetrainCommand>`,
  post a command, `tick()`, and assert governed output — no `Blackboard`,
  no mocks.
- Add a case exercising an empty `driveIn` (tick() holds/decays gracefully,
  matching today's no-new-command behavior).
- Run the full existing `Drivetrain`/`Planner` test suite to confirm no
  regression in governed-output math.
- **Verification command**: `uv run pytest tests/sim/unit/test_drivetrain.py tests/sim/unit/test_planner.py`

**Documentation updates:** none beyond `architecture-update.md`.

## Implementation Notes (post-execution)

- The authority-mode field was added at the proto source
  (`protos/drivetrain.proto`'s `DrivetrainState.active`, field 12) and
  `source/messages/drivetrain.h` was regenerated via `uv run python3
  scripts/gen_messages.py` (never hand-edited) — the regen touched only that
  one header, adding exactly `bool active = false;`. An inventory-map entry
  was also added in `scripts/gen_messages.py` (`_INVENTORY_MAP`) for
  traceability parity with every other field.
- Changing `Drivetrain::tick()`'s signature broke two call sites this
  ticket's own scope did not originally list, both still calling the OLD
  3-arg `tick()`: `source/dev_loop.cpp` (the pre-ticket-007 shared loop body,
  slated for wholesale deletion in ticket 007) and
  `tests/sim/unit/dev_loop_pose_estimator_harness.cpp`'s `oneReferencePass()`
  (a hand-written mirror of `dev_loop.cpp`'s structure). Both were fixed
  with a minimal, mechanical patch: a local, never-posted-to
  `Rt::Mailbox<msg::DrivetrainCommand>` passed as `driveIn`, preserving
  today's exact behavior (empty `driveIn` -> the "otherwise" governance path
  runs unchanged, exactly as AC1 requires) without implementing any of
  Decision 1's arbitration. This is a compile-fix only, not new wiring;
  ticket 007 replaces both call sites for real.
- Full verification: `tests/sim/unit/test_drivetrain.py` +
  `tests/sim/unit/test_planner.py` (2 passed), then `tests/sim -q` (254
  passed — identical to the pre-ticket baseline, no regressions, no golden
  shifts needed).
