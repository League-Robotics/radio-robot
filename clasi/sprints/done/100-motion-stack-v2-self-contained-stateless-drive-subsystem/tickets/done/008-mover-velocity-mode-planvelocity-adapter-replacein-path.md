---
id: '008'
title: 'MOVER velocity mode: planVelocity + adapter replaceIn path'
status: done
use-cases: [SUC-010]
depends-on: ['007']
github-issue: ''
issue: motion-stack-v2-a-self-contained-stateless-motion-control-subsystem.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# MOVER velocity mode: planVelocity + adapter replaceIn path

## Preconditions

Robot USB-attached (or otherwise reachable per the setup ticket 007
already established) for this ticket's HITL acceptance step. Coordinate
timing with the team-lead — this ticket's HITL step can often piggyback
on the same session as ticket 007's, since both need the robot on the
stand.

## Description

Wire MOVER (deadman-velocity teleop) through `Drive::Drivetrain::
planVelocity()` and the adapter's existing `replaceIn` latest-wins
mailbox. MOVER's wire shape is UNCHANGED (`time`/`v`/`omega` +
`primitive=true`) — only what SOLVES it changes.

## Acceptance Criteria

- [x] `Subsystems::Drivetrain`'s `replaceIn` drain path calls
      `planVelocity(target, deadman, current)` instead of the old
      segment-replace path, for a `MotionSegment` carrying MOVER's
      `time`/`v`/`omega` + `primitive=true` shape. Verify by diffing the
      wire bytes of a MOVER command before/after this ticket that the
      WIRE shape is byte-identical — only the firmware-side handling
      changes.
- [x] Each fresh MOVER replaces the held plan (latest-wins, matching
      `replaceIn`'s existing `Mailbox` semantics — no new queueing
      behavior introduced).
- [x] Deadman expiry (no fresh MOVER within the window) results in the
      terminal machine (ticket 005) decelerating to a literal `0.0f` —
      no separate watchdog logic duplicated in the adapter (grep-
      verifiable: no new timer/deadline field added to `Subsystems::
      Drivetrain` for this purpose).
- [x] BLEND (`stream=true` on the `segment`/`replace` arm outside the
      MOVER shape) continues to reply `ERR` — explicitly verified NOT
      accidentally enabled by this ticket's changes.
- [ ] HITL: a streamed MOVER sequence (via `tests/bench`'s existing
      teleop tooling, e.g. `gamepad_teleop.py`) drives the robot smoothly
      on the stand at commanded `(v, omega)`; releasing the deadman
      brings the robot to a literal-zero setpoint within the terminal
      machine's dwell. **UNCHECKED — team-lead to validate on the robot**
      (firmware builds clean over DeviceBusHardware, `MICROBIT.hex`
      regenerated this ticket; not yet flashed/driven on the stand by
      this session).
- [x] Tier-1 sim test: a MOVER sequence with a deliberately-expired
      deadman decelerates and stops; a fresh MOVER before expiry
      replaces the plan without a velocity discontinuity.
- [x] `uv run python -m pytest` passes.

## Testing

- **Existing tests to run**: `uv run python -m pytest`; ticket 007's
  regenerated golden TLM (must stay passing).
- **New tests to write**: tier-1 sim (deadman expiry, fresh-MOVER
  replacement, no-discontinuity check); HITL streamed-teleop session.
- **Verification command**: `uv run pytest`

## Implementation Plan

**Approach**: this is a small, additive change to the already-rewritten
wafer adapter (ticket 007) — the `replaceIn` drain path already exists;
this ticket changes WHAT it calls (`planVelocity()` instead of the old
segment-replace), not the queue mechanism itself.

**Files to modify**: `source/subsystems/drivetrain.{h,cpp}` (the
`replaceIn` drain call site only).

**Testing plan**: tier-1 sim (deadman expiry, fresh-MOVER replacement);
HITL (stand, streamed teleop).

**Documentation updates**: none.
