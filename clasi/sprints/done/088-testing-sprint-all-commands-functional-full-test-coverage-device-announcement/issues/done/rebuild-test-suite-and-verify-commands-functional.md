---
status: done
sprint: 088
tickets:
- 088-008
- 088-009
---

# Rebuild/expand the test suite and verify every registered command is functional (motion/config via encoders on the stand)

## Context

Stakeholder mandate (2026-07-07): **this sprint focuses on testing.** Beyond the
per-command smoke suite (see
[full-command-smoke-test-suite.md](clasi/issues/full-command-smoke-test-suite.md)),
"expand out and rebuild all the other tests" and "get the robot actually
functioning, where all the commands are there." The explicit functional bar:
**all motion and configuration commands must actually function**, provable **by
looking at the encoders** on the stand (wheels off the ground, safe to drive).
The stakeholder acknowledges a few commands can't be fully exercised on the stand.

The current tree (post-077 greenfield, through sprint 087) has: a **skeleton**
`tests/sim/` suite (some command/subsystem tests exist —
`tests/sim/unit/test_*_commands.py`, `test_protocol_roundtrips.py`), **parked**
`tests/playfield/` scripts, and `tests/bench/` HITL CLI tools
([tests/CLAUDE.md](tests/CLAUDE.md)). Coverage of the full registered command
surface and several subsystems is incomplete.

## Scope

1. **Sim coverage up to the command surface + core subsystems.** Complement the
   smoke suite with real (non-smoke) sim/unit + sim/system tests for the
   subsystems that back the commands: `Drivetrain`, `Hal::Motor`/`NezhaMotor`
   (via `SimMotor`), `PoseEstimator`, `Planner`, `Communicator`,
   `CommandRouter`/`Configurator`. Fill obvious gaps where a command family has no
   behavioral test today.
2. **Functional bench verification (the hard requirement).** On the stand, deploy
   the firmware and confirm every **motion** verb (`S`, `T`, `D`, `R`, `TURN`,
   `RT`, `G`, `STOP`) actually drives the wheels and the **encoders increment** in
   the expected direction and roughly in proportion to the command; confirm every
   **config** verb (`SET`/`GET`, and the `DEV` config subcommands) takes effect;
   round-trip over the real link (serial at the bench, radio relay where
   applicable). This depends on the wheel-direction fix
   ([tovez-drive-motor-reversed-fwd-sign.md](clasi/issues/tovez-drive-motor-reversed-fwd-sign.md))
   landing first, or straight-drive verification will read as a failure.
3. **Document stand limits.** Commands that can't be fully validated on the stand
   (e.g. OTOS absolute position without real translation/floor, camera `SI`
   pose-inject, playfield-frame goto) get smoke/dispatch-level coverage only, with
   a written note of why and what would validate them (bench-with-motion or
   playfield).

## Bounds (decision — keep this finite)

This is **not** a from-scratch rebuild of every historical test in `tests_old/`.
It is: (a) the smoke suite, (b) behavioral sim coverage of the registered command
families and the subsystems that implement them, and (c) on-stand functional proof
for motion/config via encoders. `tests/playfield/` stays parked (needs the
playfield, not the stand); `tests/bench/` gains/refreshes only the CLI tools needed
to run the functional verification.

## Acceptance

- Every registered command has at least smoke coverage; every motion/config
  command additionally has a behavioral sim test.
- **On the stand:** each motion verb drives the wheels with encoders incrementing
  as expected; each config verb's effect is observable; verified over the real
  link. Captured as a bench checklist/log in the sprint.
- `uv run python -m pytest` (the `tests/sim/` gate) is green.
- A short written record of which commands were fully bench-verified vs.
  smoke-only-on-stand, and why.
