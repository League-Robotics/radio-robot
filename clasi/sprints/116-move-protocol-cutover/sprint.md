---
id: '116'
title: MOVE protocol cutover
status: roadmap
branch: sprint/116-move-protocol-cutover
worktree: false
use-cases: []
issues:
- gut-to-minimal-firmware-motion-stack-excision-move-protocol-minimal-telemetry.md
- protocol-set-point-the-minimal-firmware-s-complete-command-surface.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 116: MOVE protocol cutover

## Goals

- Cut the command surface over to the bounded **MOVE** protocol per the
  protocol set-point issue: one `Move` command (twist|wheels velocity
  variant + time|distance|angle stop condition + required `timeout`
  backstop + `replace` flag against a 1-active + 4-pending queue) plus
  `STOP` as the immediate halt.
- Delete the legacy `Twist` arm and the `app/deadman.*` module — every
  motion becomes structurally self-bounding (stop condition or timeout),
  which supersedes the deadman rather than needing it alongside MOVE.
  This is sprint 115's deferred piece: S1 kept TWIST+deadman specifically
  so the robot stayed drivable at its own gate.
- Deliver the protocol document itself — the written command-surface
  contract (transport/framing, command table, response semantics, error
  taxonomy) matching the set-point issue — as the minimal firmware's set
  point when this sprint closes.

## Problem

The interim TWIST+deadman surface has two overlapping bounding mechanisms
(a per-command duration vs. a separate deadman lease) and two motion verbs
(a bare TWIST plus a planned, never-shipped, separate Wheels command)
where one bounded MOVE suffices. Host silence today only ends safely
because of the deadman watchdog; there's no queue, no distance/angle stop
condition, and no structural guarantee that a command completes on its
own terms.

## Solution

Add `Move` arm 21 (`MoveTwist{v_x, v_y, omega} | MoveWheels{v_left,
v_right}` velocity oneof + `time|distance|angle` stop oneof + required
`timeout` + `replace` + `id`), a new `Motion::StopCondition` object
(captures its baseline — activation time, odometry path length, odometry
heading — at activation, ticked every cycle) and `App::MoveQueue` (1
active + 4 pending; `replace=true` flushes and preempts; `replace=false`
enqueues, ERR_FULL past 4 pending; completion ack against `Move.id`;
timeout → stop + move-timeout fault flag). Delete `app/deadman.*`, the
`Twist` arm (reserve 19), and the `ConfigDelta.watchdog` arm (reserve 4).
The active MOVE's velocity stages through `Drive` (`setTwist`/`setWheels`,
last-wins) exactly as today — never a direct motor write. Host gets
`NezhaProtocol.move_twist(...)`/`move_wheels(...)`/`stop()`. The written
protocol document ships alongside the code as this sprint's other
deliverable — the converged contract the minimal firmware speaks once
this sprint closes.

## Success Criteria

- Full protocol gate passes on the stand: `HELLO`, `PING` (`t=` present),
  `CONFIG` patch persists across power-cycle, `MOVE` × both velocity
  variants × all three stop conditions, `STOP` — each acked with the
  correct `corr`/`err` via the single ack slot.
- Stop-condition behavior verified: a time MOVE ends on schedule; a
  distance/angle MOVE ends within tolerance of the commanded
  distance/heading change (measured via encoders, on the stand); a
  distance MOVE that cannot progress ends at `timeout` with the
  move-timeout fault flag set.
- Chaining verified: MOVE B (`replace=false`) sent while A runs hands off
  seamlessly at A's expiry; `replace=true` preempts mid-motion; a 5th
  pending MOVE gets `ERR_FULL`; an empty queue's expiry stops motors with
  **zero host traffic** (the no-deadman contract).
- 10-minute soak (≥5-10 Hz alternating MOVEs) clean: no reboot/lockup,
  seq monotonic, drop rate at or better than the sprint-115 baseline.
- The protocol document lands in `docs/` and matches the shipped contract
  exactly (command table, error taxonomy, response semantics).

## Scope

### In Scope

- `envelope.proto`: `Move` arm 21 (fresh; the old arc-Move's 20 stays
  reserved); delete `Twist` → reserve 19; delete `ConfigDelta.watchdog` →
  reserve 4.
- New `Motion::StopCondition` (kind + threshold + activation baselines
  from clock/`App::Odometry`; `tick()` → stop); `App::Odometry` gains a
  `pathLength()` accessor.
- New `App::MoveQueue` (1 active + 4 pending; replace/flush/ERR_FULL
  semantics; completion ack against `Move.id`; owned by `RobotLoop`,
  ticked where the deadman check used to live).
- Deletion of `app/deadman.*`.
- Host: `NezhaProtocol.move_twist(...)`/`move_wheels(...)`/`stop()`;
  `wait_for_ack` unchanged (still single-slot).
- Tests: stop-condition units (time/distance/angle + timeout), queue
  semantics (chain/replace/overflow/drain), robot_loop dispatch tests,
  sim system scenarios (seamless chaining, empty-queue stop).
- The protocol document (transport/framing, command table, response
  semantics, error taxonomy) as a delivered artifact.

### Out of Scope

- Estimator/state-prediction work (sprint 117) — this sprint only
  finishes the command-surface half of the gut; the telemetry frame it
  rides on (flags bit 15 for move-timeout) was already defined in
  sprint 115.
- Any host motion/tour code changes beyond what `protocol.py`'s move_*
  helpers force.
- Arc/segment moves, trajectory profiles, jerk limiting, heading cascade,
  pose-fix injection — explicitly out of the converged protocol per the
  set-point issue; recoverable from the `pre-gut-motion-stack` tag if
  ever needed.

## Test Strategy

`uv run python -m pytest` + sim suite green; `just build-clean`;
`mbdeploy deploy` (hex verified by full UID — robot, not the relay
dongle); then the hardware protocol gate on the stand: round-trip every
command (HELLO/PING/CONFIG/MOVE×variants×stop-conditions/STOP) with
correct acks; stop-condition behavior (time/distance/angle/timeout-fault);
chaining/preemption/overflow/no-deadman-expiry; a ≥10-minute soak at
alternating MOVE rates per the gut protocol's soak gate.

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
