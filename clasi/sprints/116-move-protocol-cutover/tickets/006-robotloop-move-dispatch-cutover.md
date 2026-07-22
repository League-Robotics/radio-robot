---
id: '006'
title: RobotLoop MOVE dispatch cutover
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
- '005'
github-issue: ''
issue:
- protocol-set-point-the-minimal-firmware-s-complete-command-surface.md
- gut-to-minimal-firmware-motion-stack-excision-move-protocol-minimal-telemetry.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# RobotLoop MOVE dispatch cutover

## Description

The single integration ticket where the old (`Twist`+`Deadman`) and new
(`Move`+`MoveQueue`) dispatch paths cross over — per sprint.md's Migration
Concerns, no intermediate state should exist where both are partially
present. Deletes `app/deadman.{h,cpp}` and both its test harnesses;
replaces `RobotLoop::handleTwist()` with `handleMove()` (same
config-completeness gate `handleTwist()` already has, now validating a
`Move`'s shape — velocity variant present, stop variant present, `timeout
> 0`, else `ERR_BADARG` — before delegating to `moveQueue_.enqueue()`);
`handleStop()` additionally calls `moveQueue_.flush()`; the cycle body's
per-cycle, unconditional `deadman_.expired()` branch
(`robot_loop.cpp:485-497`, inside the `runAndWait(kSettle, ...)` block at
`robot_loop.cpp:477-498`) is replaced by an unconditional
`moveQueue_.tick(now, odom_)` call at the same schedule position — this is
the load-bearing safety property (sprint.md SUC-053): it must run every
cycle regardless of whether a command arrived, the same way
`deadman_.expired()` did. `kFlagFaultMoveTimeout` (bit 15, declared since
115, unwired) gets its first live `setFlag()` call. `frame_.mode`/
`driving_` derive from `moveQueue_.active()` instead of the hand-toggled
bool.

Both composition roots rewire together: `main.cpp` (construct
`App::MoveQueue` instead of `App::Deadman`, update the `RobotLoop`
constructor call — `Deadman&` param removed, `MoveQueue&` added) and
`src/sim/sim_harness.h` (same rewiring, plus `injectMove()` replacing
`injectTwist()`, built on ticket 001's `armorMoveCommand()` helper).

## Acceptance Criteria

- [ ] `app/deadman.{h,cpp}` deleted; `app_deadman_harness.cpp` and
      `test_app_deadman.py` deleted; no `App::Deadman` symbol remains
      anywhere in `src/` outside `src/archive/` (SUC-053).
- [ ] `RobotLoop`'s constructor signature: `Deadman&` parameter removed,
      `MoveQueue&` parameter added.
- [ ] `handleMove()` replaces `handleTwist()`: config-completeness gate
      first (unconfigured → `ERR_NOT_CONFIGURED`, unchanged position/
      semantics from `handleTwist()`); shape validation (missing velocity
      variant / missing stop variant / non-positive `timeout` →
      `ERR_BADARG`); delegates to `moveQueue_.enqueue()`.
- [ ] `handleStop()` calls `moveQueue_.flush()` in addition to its
      existing `drive_.stop()`/ack behavior.
- [ ] `cycle()`'s dispatch block calls `moveQueue_.tick(now, odom_)`
      unconditionally, every cycle, at the schedule position the deleted
      `deadman_.expired()` branch occupied — verified by a test that
      sends zero commands after a `Move` ends and confirms motors reach
      and stay at zero with no further host traffic (SUC-053).
- [ ] `kFlagFaultMoveTimeout` is set live on a timeout-ended `Move`'s
      ending cycle (SUC-054).
- [ ] `frame_.mode`/`driving_` derive from `moveQueue_.active()`.
- [ ] `main.cpp` and `src/sim/sim_harness.h` both construct
      `App::MoveQueue` and pass it into `RobotLoop`'s constructor; neither
      constructs `App::Deadman` anymore.
- [ ] `sim_harness.h` gains `injectMove()` (built on ticket 001's
      `armorMoveCommand()`); `injectTwist()` is deleted.
- [ ] `app_robot_loop` test sweep (`app_robot_loop_harness.cpp`/
      `test_app_robot_loop.py`) updated: TWIST-dispatch tests replaced by
      MOVE-dispatch tests, plus a new explicit test for SUC-055 (a CONFIG
      patch applied mid-`Move` does not change the `Move`'s completion
      time/distance/angle outcome).
- [ ] Repo-wide grep confirms no remaining `handleTwist`/`Deadman`/
      `deadman_` references outside `src/archive/`.

## Testing

- **Existing tests to run**: full sim suite (`uv run python -m pytest`) —
  this ticket's blast radius is the widest of the sprint (both
  composition roots), so a full run, not a targeted subset, is the bar.
- **New tests to write**: MOVE dispatch tests (config-gate refusal,
  `ERR_BADARG` shape validation, successful enqueue+ack) in
  `test_app_robot_loop.py`; the SUC-053 no-deadman drain test; the
  SUC-054 timeout-fault-flag test; the SUC-055 CONFIG-mid-MOVE
  non-interaction test.
- **Verification command**: `python build.py && uv run python -m pytest`
