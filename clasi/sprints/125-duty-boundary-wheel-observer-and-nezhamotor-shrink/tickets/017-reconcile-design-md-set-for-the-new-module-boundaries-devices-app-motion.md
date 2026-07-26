---
id: '017'
title: Reconcile DESIGN.md set for the new module boundaries (devices/app/motion)
status: open
use-cases: []
depends-on:
- '011'
- '012'
- '015'
- '016'
github-issue: ''
issue: firmware-base-hardening-bounded-wheel-moves-and-wheel-observer.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Reconcile DESIGN.md set for the new module boundaries (devices/app/motion)

## Description

Closing-time ticket, required by `close_sprint`'s design-doc validation
(`design_docs: enabled` in `.clasi/config.yaml`) — not optional cleanup.
Reconcile: `src/firm/devices/DESIGN.md` (`NezhaMotor` shrink, `MotorArmor`
narrowing, `MotorConfig` field trim), `src/firm/app/DESIGN.md`
(`App::WheelObserver`, `App::Sensors`, `App::Drive`'s ownership+phase-
method shape, `App::RobotLoop`'s zero-device-member final shape),
`src/motion/DESIGN.md` (`Motion::WheelSink`'s duty-sink retooling,
`Motion::WheelVelocityPid`, `MoveQueue`'s PID/`commandedTwist()`
additions, `StateEstimator`'s write-through change). Update
`docs/design/design.md` too if the system-level component/dependency
diagram changed (compare against sprint architecture Step 4's diagrams).
If the ownership-reshuffle tail (tickets 013-016) split to a 125b instead
of landing in 125, scope this ticket down accordingly and leave a clear
note for 125b's own closing-time doc reconciliation.

## Acceptance Criteria

- [ ] `close_sprint`'s design-doc validation passes (`validate_design` or
      the `clasi design validate` CLI) with no unresolved references.
- [ ] Each touched `DESIGN.md`'s own module/boundary descriptions match
      what actually landed (not what was originally planned, if any
      ticket's implementation diverged from this sprint's architecture —
      reconcile against the CODE, not just copy the architecture prose
      verbatim).
- [ ] `docs/design/design.md` updated if the system-level diagram changed;
      explicitly left alone with a one-line note if it didn't need to.

## Testing

- **Existing tests to run**: `clasi design validate` (or the
  `validate_design` MCP tool) against the full `docs/design/` tree.
- **New tests to write**: none — this is documentation reconciliation.
- **Verification command**: `clasi design validate`.
