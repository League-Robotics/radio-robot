---
id: 087
title: 'Two-plane blackboard: synchronous-update loop, Configurator, and command-queue
  transport (greenfield)'
status: done
branch: sprint/087-two-plane-blackboard-synchronous-update-loop-configurator-and-command-queue-transport-greenfield
use-cases: []
issues:
- plan-file-a-design-issue-blackboard-architecture-state-objects-command-queues.md
- preserve-serial-silence-safety-watchdog-in-greenfield-loop.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 087: Two-plane blackboard: synchronous-update loop, Configurator, and command-queue transport (greenfield)

## Goals

Rearchitect the firmware's command tier onto a two-plane blackboard (state
objects + command queues), a synchronous-update (double-buffered) execution
model, a single `Configurator` config-application authority, and a
cyclic-executive main loop — replacing the six pointer-holding `*State`
structs and `dev_loop.*` with a design where subsystems are constructible
in isolation and unit-testable from an enumerable `tick()` signature alone.
Greenfield by deletion: delete `source/main.cpp`'s loop and all of
`source/dev_loop.*`, then rebuild.

## Problem

Every command family today is wired through a `*State` struct holding raw
subsystem pointers (`DevLoopState`, `TelemetryState`, `MotionLoopState`,
`ConfigCommandState`, `PoseCommandState`, `OtosCommandState`, plus the
`DevLoop` holder), inverting the intended dependency direction, producing
three scattered config-shadow caches, one cross-family pointer
reach-through, and a non-uniform subsystem "faceplate." See
`architecture-update.md` Step 1 for the full grounded problem statement.

## Solution

See `architecture-update.md` in full: a `Rt::Blackboard` (state cells +
`Mailbox`/`WorkQueue` command queues), a `Configurator` (the one legitimate
subsystem-reference holder), a `CommandRouter` (pointerless command-family
translators), and a cyclic-executive loop (mandatory tick -> commit ->
`uBit.sleep(1)`-yielding best-effort slack — Decision 9, required so
CODAL's event-driven radio RX is not starved). The design is ported
verbatim in intent from
`clasi/issues/plan-file-a-design-issue-blackboard-architecture-state-objects-command-queues.md`.

## Success Criteria

All six SUCs in `usecases.md` are met end-to-end (see ticket 009's
acceptance criteria for the full closing sweep): deterministic
order-independent ticking, isolated subsystem testability, one config
authority, atomic state-reset at the clock edge, cadence protected from
config/routing load (including the radio-safe yield), and zero subsystem
pointers in command handlers. Confirmed by the full automated suite plus a
hardware-bench gate exercised over the **radio** transport specifically,
including the serial-silence safety watchdog neutralizing under comms
silence on the stand.

## Scope

### In Scope

- `Rt::Mailbox`/`Rt::WorkQueue` primitives and `Rt::Blackboard` (tickets
  001-002).
- Faceplate regularization for `Drivetrain`, `Planner`, `PoseEstimator`,
  `Hardware` (tickets 003-004).
- `Configurator` and `CommandRouter` + rewrite of all six command families
  (tickets 005-006).
- New main loop, deleting `dev_loop.*`, rewiring `main.cpp` and
  `tests/_infra/sim/sim_api.cpp` in lockstep, preserving the serial-silence
  safety watchdog (ticket 007).
- `Telemetry` reading the committed blackboard snapshot (ticket 008).
- Control retuning (if needed) and the closing HITL bench gate, including
  radio-specific verification and the watchdog's bench acceptance (ticket
  009).

### Out of Scope

- Any wire/protocol-v2 change (this is an internal rewiring; verified as
  unaffected throughout).
- Threading of statement ingestion (explicitly deferred, Open Question 5).
- Any change to sprint 086's territory (`velocity_pid.*`, motor-policy
  armor) beyond wrapping it in the new `tick()` signatures.

## Test Strategy

Per-ticket unit/harness tests against bare `Rt::Mailbox`/`Rt::WorkQueue`/
`Rt::Blackboard` instances (no full wiring needed, per the
enumerable-dependency goal), full regression of the existing
`tests/sim/unit/` and `tests/sim/system/` suites at every ticket, and a
final hardware-bench gate (ticket 009) exercised over **both** serial and
radio transports — radio specifically to catch a missing slack-loop yield
(Decision 9), plus the serial-silence watchdog's bench/HITL acceptance.

## Architecture Notes

See `architecture-update.md` for the full design, Decisions 1-9, and the
architecture self-review (verdict: APPROVE). Key resolved modeling choices:
`driveIn` is a single coalescing `Mailbox` gated by `Drivetrain`'s existing
`active()`/`standby()` authority (Decision 1); `motorIn` is a per-port
`Mailbox` array (Decision 2); the `Configurator` holds subsystem references
(Decision 4); the slack loop yields via `uBit.sleep(1)` every iteration,
required for CODAL's event-driven radio RX, not just pacing (Decision 9).

## GitHub Issues

(None linked yet.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [ ] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [ ] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Command-plane queue primitives (`Rt::Mailbox`, `Rt::WorkQueue`) | — |
| 002 | `Rt::Blackboard` state and command planes | 001 |
| 003 | Faceplate regularization: `Drivetrain` and `Planner` blackboard wiring | 002 |
| 004 | Faceplate regularization: `PoseEstimator` and `Hardware` blackboard wiring | 002 |
| 005 | `Configurator`: single config-application authority | 002, 003, 004 |
| 006 | `CommandRouter` and pointerless command-family translators | 003, 004, 005 |
| 007 | Cyclic-executive main loop: delete `dev_loop`, rewire `main.cpp` and `sim_api.cpp`, preserve serial-silence watchdog | 002, 003, 004, 005, 006 |
| 008 | `Telemetry` reads the committed blackboard snapshot | 002, 007 |
| 009 | Control retuning and HITL bench acceptance gate | 008 (transitively all) |

Tickets execute serially in the order listed. Issue back-references:
`plan-file-a-design-issue-blackboard-architecture-state-objects-command-queues.md`
is linked on every ticket (001-009); `preserve-serial-silence-safety-watchdog-in-greenfield-loop.md`
is linked only on 007 (implementation, sim-side) and 009 (Bench/HITL
radio-path closeout — `completes_issue` is set `false` for that issue on
007 so it archives only once 009 also completes).
