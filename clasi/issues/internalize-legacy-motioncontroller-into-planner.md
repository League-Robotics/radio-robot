---
status: pending
---

# Internalize the legacy `MotionController` into `Planner` (finish the de-scaffolding)

## Context

Sprint 060 completed the ordered-tick cutover: the legacy control loop and the
`subsystems::Drive`/`bvc`/`Planner` (formerly `Drive2`/`bvc2`/`MotionController2`)
de-scaffolding all landed. However, ticket 060-005 **deferred** one piece: the old
imperative `MotionController` class (`source/superstructure/MotionController.h/.cpp`)
is still a **public `Robot` value member**, wrapped by reference inside `Planner`
(the renamed `MotionController2`).

This is not a surviving "legacy control path" — it is the real S/T/D/G state-machine
+ kinematics that `Planner` delegates to. But it remains exposed on the public
`Robot.h` surface, which the cutover intended to clean up. Ticket 060-005's acceptance
criteria did not require its removal, so it was correctly deferred rather than rushed.

## Goal

Make `MotionController` an internal implementation detail of `Planner`:
- `Planner` owns the `MotionController` as a **private value member** (instead of
  `Robot` holding it and passing a reference).
- The `MotionController` type no longer appears in `Robot.h` or any other public
  include surface.
- Re-route the call sites that currently reach `robot.motionController` directly.

## Known call sites to re-route

From the 060-005 deferral note (verify with `grep -rn "motionController" source/`):
`SystemCommands.cpp`, `MotionCommands.cpp`, `MotionControllerBegin.cpp`,
`RobotTelemetry.cpp` (the `motionController.mode()` mode-char read — route through
`planner.mode()` once `Planner` exposes it), `Robot.cpp` (construction + wiring),
and `Robot::otosCorrect()`.

## Acceptance

- `grep -rn "motionController" source/` shows no direct `Robot`-member access; the
  member is gone from `Robot.h`.
- `Planner` owns `MotionController` privately; behavior unchanged.
- `RobotTelemetry.cpp` mode char comes from `planner.mode()`; golden-TLM stays green
  (byte-identical mode char).
- `uv run python -m pytest` green except the 2 known-baseline config-golden failures.

## Notes

Follow-on from sprint 060 (ticket 060-005 deferral, ticket 060-006 used the
`Planner` name to avoid colliding with the still-present `MotionController`).
Relates to [[message-based-subsystem-architecture]]. Structural/internal only —
no use-case-visible behavior change.
