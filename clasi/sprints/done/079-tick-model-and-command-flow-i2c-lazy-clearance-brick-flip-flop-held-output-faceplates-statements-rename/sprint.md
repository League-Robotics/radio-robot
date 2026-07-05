---
id: 079
title: 'Tick model and command flow: I2C lazy clearance, brick flip-flop, held-output
  faceplates, statements rename'
status: done
branch: sprint/079-tick-model-and-command-flow-i2c-lazy-clearance-brick-flip-flop-held-output-faceplates-statements-rename
use-cases: []
issues:
- i2c-bus-lazy-clearance-timers.md
- tick-model-command-flow-and-the-command-board-design-sketch.md
- rename-wire-lines-to-statements.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 079: Tick model and command flow: I2C lazy clearance, brick flip-flop, held-output faceplates, statements rename

## Goals

Implement the 2026-07-04 tick-model design (all 10 stakeholder decisions) plus
its substrate and vocabulary issues:

1. **I2C lazy clearance timers** — per-device `lastEnd`/`readyAt` deadlines in
   `I2CBus` (`preClear`/`postClear` `// [us]` params), replacing the
   unconditional ~8 ms/tick encoder settle spins; non-spinning
   `I2CBus::clear(addr)` peek.
2. **Brick flip-flop** — the HAL as brick sequencer (`activePort_`, REQUEST_DUE
   / COLLECT_DUE), wiring the split-phase `requestEncoder()`/`collectEncoder()`;
   only in-use ports cycled; HAL ticked twice per pass (sanctioned).
3. **Three-beat main loop** — feed/tick/ask everywhere: `tick()` returns void,
   producers hold output behind `hasX()`/`takeX()`; Communicator backpressure;
   processor as pure transformer with per-consumer outboxes
   (`CommandProcessorToHalCommand`, `CommandProcessorToDrivetrainCommand`);
   `DrivetrainToHalCommand` with port binding in `DrivetrainConfig`
   (`DEV DT PORTS` becomes drivetrain config); CommandQueue deleted.
4. **Statements rename** — wire lines are statements
   (`CommunicatorToCommandProcessorStatement`), naming rule 4 amended to
   `<Producer>To<Consumer><Payload>`, doc-language sweep; CommandProcessor
   class name kept.

## Problem

`NezhaMotor::tick()` blocks ~8 ms of every 10 ms control tick spinning out
0x46 settle windows; with 4 ports the loop hits ~32 ms and comms/watchdog
latency inherits it. Command flow is inconsistent: some edges returned from
tick, some pushed, a vestigial CommandQueue wired to nothing, and "command"
names both wire lines and internal messages.

## Solution

Per the design sketch (`clasi/issues/tick-model-command-flow-and-the-command-board-design-sketch.md`):
lazy per-device clearance in the bus object; the HAL owns the brick schedule
(one bus action or a pass per slice); every producer holds its output and main
visibly moves every command; the processor transforms statements into commands
and replies with no device write access; the rename lands with the faceplate
reshape so `communicator.h` is touched once.

## Success Criteria

- Control loop iterates ~0.2–1 ms with zero unconditional settle spins;
  per-motor sample cadence ≈80–90 Hz (2 ports in use), comms polled every pass.
- All inter-subsystem edges are held+taken structs; CommandQueue gone;
  statements vocabulary throughout source/docs; wire strings unchanged.
- Host tests green at the subsystem level; stand pass per the verification
  sketch (encoder cadence/evenness, in-use-port cycling, statement round-trips
  serial + radio, watchdog latency, lazy-timer A/B, `vel_filt_alpha` retune).

## Scope

### In Scope

- `source/com/i2c_bus.{h,cpp}` (+ HOST_BUILD stub) lazy clearance timers.
- `source/hal/nezha/` flip-flop scheduler, split-phase wiring, spin removal.
- `source/subsystems/` Communicator/Drivetrain held-output reshape,
  `DrivetrainConfig` ports.
- `source/commands/` processor-as-transformer, per-consumer outboxes,
  CommandQueue deletion, DevLoopState slimming.
- `source/main.cpp` Part-2 loop.
- `.claude/rules/naming-and-style.md` rule 4 amendment; `docs/protocol-v2.md`
  statement-language sweep.
- Stand verification incl. the lazy-timer A/B gate and alpha retune.

### Out of Scope

- The reversal-latch armor (sprint 078 — must already be merged; do not touch
  dwell/deadband semantics).
- Wire-format changes of any kind (verbs, reply text, keys stay).
- Moving motors into the Drivetrain (ruled out — decision 3).
- OTOS/line/color devices joining the HAL scheduler (future; Case 5 only
  validated as settle-window traffic in the A/B).

## Test Strategy

Subsystem-level host tests (decision 10): Drivetrain and processor with plain
structs in/out; HAL against a scripted HOST_BUILD `I2CBus` fake — flip-flop
sequencing, throttle, dwell interaction, in-use tracking, clearance-timer
math. Both `ROBOT_DEV_BUILD` forks build. Stand pass per the design's
verification sketch, with the lazy-timer stand A/B (latch rate with
settle-window traffic vs without, TLM enc-constancy diagnosis) as the
acceptance gate, and `vel_filt_alpha` retuned at the new cadence via step
responses.

## Architecture Notes

- All 10 stakeholder decisions of 2026-07-04 are settled — the sprint
  implements, it does not re-litigate.
- Statement feed must COPY the line (Communicator edge aliases its buffer).
- Duty writes at collect time only; postClear attaches to the request write.
- Clearance spins stay OUTSIDE the IRQ-guard masked window; waits remain spins
  (vendor no-interleave property), the non-spinning peek is for the scheduler.
- Depends on sprint 078 (armor) being merged first — design risk 2.

## GitHub Issues

(GitHub issues linked to this sprint's tickets. Format: `owner/repo#N`.)

## Definition of Ready

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed (self-review recorded 2026-07-05, verdict APPROVE)
- [x] Stakeholder has approved the sprint plan — recorded 2026-07-05
      (auto-approve mode; planner's latitude decisions accepted as-is: `hal_command.h`
      contract-type placement, config-plane statements as direct calls,
      Drivetrain-owned authority arbitration, `DevLoopState` as the concrete
      outbox with `CommandProcessor` kept domain-blind, the 2-entry+broadcast
      outbox shape, `CommandProcessor` name kept). Sprint is in `ticketing`
      phase; all six tickets below are created.

## Tickets

| # | Title | Depends On | Issue(s) |
|---|-------|------------|----------|
| [001](tickets/001-i2cbus-lazy-clearance-timers-clear-peek-host-build-scripted-fake.md) | I2CBus lazy clearance timers, `clear()` peek, `HOST_BUILD` scripted fake | — | i2c-bus-lazy-clearance-timers.md |
| [002](tickets/002-statements-rename-rule-4-amendment-communicator-held-statement-reshape.md) | Statements rename, rule-4 amendment, Communicator held-statement reshape | — | rename-wire-lines-to-statements.md, tick-model-command-flow-and-the-command-board-design-sketch.md |
| [003](tickets/003-shared-hal-command-edge-types-and-drivetrain-reshape.md) | Shared HAL command-edge types + Drivetrain reshape (ports/active/standby) | 002 | tick-model-command-flow-and-the-command-board-design-sketch.md |
| [004](tickets/004-nezhahal-brick-flip-flop-and-distribution-nezhamotor-split-phase-wiring.md) | NezhaHal brick flip-flop + distribution; NezhaMotor split-phase wiring | 001, 003 | i2c-bus-lazy-clearance-timers.md, tick-model-command-flow-and-the-command-board-design-sketch.md |
| [005](tickets/005-commandprocessor-pure-transformer-commandqueue-deletion-main-cpp-three-beat-loop.md) | CommandProcessor pure transformer, `CommandQueue` deletion, main.cpp three-beat loop | 002, 003, 004 | tick-model-command-flow-and-the-command-board-design-sketch.md, rename-wire-lines-to-statements.md |
| [006](tickets/006-stand-verification-cadence-in-use-cycling-a-b-gate-alpha-retune-watchdog-latency-round-trips.md) | Stand verification: cadence/evenness, A/B gate, alpha retune, watchdog latency, round-trips | 001, 002, 003, 004, 005 | i2c-bus-lazy-clearance-timers.md, tick-model-command-flow-and-the-command-board-design-sketch.md, rename-wire-lines-to-statements.md |

Tickets execute serially in the order listed. No execution lock or branch
has been acquired — the sprint-planner stops here per its charter (planning
only, no code, no lock, no branch).
