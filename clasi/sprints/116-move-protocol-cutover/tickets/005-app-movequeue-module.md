---
id: '005'
title: App::MoveQueue module
status: in-progress
use-cases:
- SUC-050
- SUC-051
- SUC-052
- SUC-053
- SUC-054
- SUC-055
depends-on:
- '001'
- '002'
- '003'
- '004'
github-issue: ''
issue:
- protocol-set-point-the-minimal-firmware-s-complete-command-surface.md
- gut-to-minimal-firmware-motion-stack-excision-move-protocol-minimal-telemetry.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# App::MoveQueue module

## Description

New module `App::MoveQueue` (`src/firm/app/move_queue.{h,cpp}` +
`DESIGN.md`). Purpose: owns the lifecycle of the robot's queued and active
bounded motions. Boundary: inside — the 5-slot array (1 active + 4
pending), replace/flush/enqueue/`ERR_FULL` bookkeeping, advancing
active→next-pending on stop/timeout, owning and driving one `Motion::
StopCondition` (ticket 002) for whichever `Move` is active; outside —
deciding what a *valid* `Move` looks like (ticket 006's `RobotLoop::
handleMove()` job), how a velocity variant becomes wheel duty (`Drive`'s
job, ticket 004), what "traveled far enough" means numerically
(`StopCondition` + `Odometry`, tickets 002/003). Constructor dependencies:
`Drive&`, `Odometry&`, `const Devices::Clock&` — the same three
collaborators `Deadman` (clock only) and `RobotLoop` (drive+odom, already)
depend on today; no new dependency direction (sprint.md Architecture Step
3/4).

This is the integration point for tickets 001-004 — depends on all of
them. Ticket 006 (`RobotLoop` cutover) is the only consumer that wires
this into the live composition roots; this ticket's own tests exercise
`MoveQueue` standalone.

## Acceptance Criteria

- [ ] `enqueue(Move, corrId)`: `replace=true` flushes every pending slot
      and preempts the active `Move` immediately, the new `Move` activates
      the same call; `replace=false` appends behind the active `Move`,
      returning `ERR_FULL` if 4 are already pending — and on that
      rejection, the existing active + 4 pending contents are provably
      unchanged (byte-for-byte, not just "still 4 pending" — sprint.md
      SUC-052).
- [ ] Activation (on `enqueue()` when the queue was empty/`replace`, or on
      the queue auto-advancing) stages the `Move`'s velocity variant
      through `Drive` (`setTwist`/`setWheels`) and constructs a
      `Motion::StopCondition` with a baseline captured from `Odometry` at
      that exact moment.
- [ ] `tick(now, odom)`: ticks the active `StopCondition`; on stop-met or
      timeout, ends the active `Move` — emits a completion ack against
      `Move.id` (outcome distinguishes stop-met from timeout, so the
      caller can set `kFlagFaultMoveTimeout` correctly) — and activates
      the next pending `Move` the SAME cycle (seamless hand-off, no
      intervening zero-velocity cycle — SUC-051) or leaves the queue
      empty for the caller to observe via `active()`.
- [ ] `flush()` (used by STOP, ticket 006): drains every pending slot and
      ends the active one with no completion ack for the flushed
      pending entries (sprint.md Architecture Open Question 2's resolved
      convention — only an activated-then-ended `Move` ever gets a
      completion ack) — implemented and tested explicitly, not left
      implicit.
- [ ] `active()` bool accessor for the caller's `frame_.mode`/`driving_`
      derivation.
- [ ] Fan-out stays at 3 injected collaborators (`Drive`, `Odometry`,
      `Devices::Clock`) plus the owned (not injected) `StopCondition` —
      no additional dependency added beyond what the architecture doc
      specifies.

## Testing

- **Existing tests to run**: none directly (new module); full
  `python build.py` must stay clean once wired.
- **New tests to write**: a new sim unit-test harness (e.g.
  `app_move_queue_harness.cpp` + `test_app_move_queue.py`) covering:
  enqueue/chain (seamless hand-off), replace (flush+preempt same cycle),
  overflow (`ERR_FULL`, contents unchanged), drain (empty queue after
  the last `Move` ends), and the flushed-while-pending no-completion-ack
  convention.
- **Verification command**: `python build.py && uv run python -m pytest
  src/tests/sim/unit/test_app_move_queue.py`
