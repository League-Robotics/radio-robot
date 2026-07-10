---
status: done
tickets:
- NONE
---

# Bench-verify the motors-running watchdog fire-gate over the radio-relay path

## Context

Sprint 091 ticket 003 (`clasi/sprints/091-hardware-leaf-and-safety-cleanups-
estop-rename-configured-poll-set-motors-running-watchdog-gate/tickets/
003-gate-the-serial-silence-watchdog-s-fire-on-commanded-motors-running-
state.md`) gated the serial-silence safety watchdog's FIRE on commanded
motors-running state (`bb.drivetrain.active || any(bb.motors[i].active)` --
see that sprint's `architecture-update.md` Decision 3): idle + comms
silence past the window no longer fires (no neutralize, no
`EVT dev_watchdog`); a live drive command + silence past the window still
fires exactly as before (immediate same-pass `estop()` + fire-once
`EVT dev_watchdog`).

That ticket's acceptance was proven with sim tests only
(`tests/sim/unit/test_watchdog_policy.py`:
`test_watchdog_does_not_fire_when_idle`,
`test_watchdog_does_not_fire_after_explicit_neutralize`, plus the
pre-existing driving-fires tests, unmodified). The radio-path HITL bench
the issue (`watchdog-arm-only-while-motors-running.md`) originally asked
for was explicitly deferred out of that ticket's acceptance because **the
radio relay dongle was unplugged during that sprint's run** and could not
be exercised.

This issue also closes out sprint 087's still-unverified watchdog-over-radio
concern: the greenfield loop rewrite's `uBit.sleep(1)` yield point (see
`.clasi/knowledge/radio-needs-loop-yield.md`) must not starve the radio
receive path enough to make the watchdog's `feed()` (fed on arrival of ANY
inbound command, any channel) arrive late over radio specifically, which a
direct-USB-serial bench pass cannot rule out.

## Ask

On the stand, over the radio relay (`!GO` data-plane handshake, per
`.clasi/knowledge/` radio-relay notes -- NOT `rogo`/the older `>`-prefix
protocol):

- **Idle case**: with the robot idle (no `DEV M`/`DEV DT` motion verb
  active), narrow the watchdog window (`DEV WD <small>`), then go silent
  over the relay past the window. Confirm **no** neutralize and **no**
  `EVT dev_watchdog` are observed.
- **Driving case**: issue a live drive command (e.g. `DEV M <n> VEL <v>` or
  a `T`/`D` motion verb) over the relay, then go silent past the window.
  Confirm an immediate neutralize (wheels stop) and `EVT dev_watchdog` are
  observed over the relay reply channel.
- While at it, confirm the relay round-trip itself keeps up under a long
  drive (i.e. the loop's `uBit.sleep(1)` yield is not starving the radio
  receive path enough to falsely trigger -- or fail to trigger -- either
  case above).

## Acceptance

- Both cases (idle-no-fire, driving-fires) confirmed over the radio-relay
  path on the stand, with wheels observed directly (not just encoder
  telemetry) for the driving case's neutralize.
- Result appended to a bench-verification log (sprint 088's
  `bench-verification-log.md` or a fresh one), referencing this issue and
  sprint 091 ticket 003.
- If the radio path reveals a discrepancy from the sim-proven behavior
  (e.g. a feed/check timing gap specific to the relay), file a follow-up
  issue describing it rather than silently patching the gate.

## Closed 2026-07-09 — obsoleted by the sprint 093 loop gut (stakeholder triage)

The mechanism this issue verifies no longer exists. Sprint 093's rewritten
`Rt::MainLoop` **removed the serial-silence safety watchdog itself** (see
`source/runtime/main_loop.h`: "no watchdog-feed hook (093 removes the
watchdog itself)"), and the `DEV WD` / `DEV M` verbs the test procedure
depends on are unregistered (`buildTable()` in
`source/runtime/command_router.cpp` no longer wires the `dev` family).
There is nothing left to bench-verify: the 091 motors-running fire-gate is
dead code in an unwired family. Comms-loss safety was redesigned into the
`MOVER` segment's deadman-velocity semantics (teleop path). If a
serial-silence watchdog is ever reintroduced, it will be a new design
requiring its own verification issue — do not resurrect this one.
