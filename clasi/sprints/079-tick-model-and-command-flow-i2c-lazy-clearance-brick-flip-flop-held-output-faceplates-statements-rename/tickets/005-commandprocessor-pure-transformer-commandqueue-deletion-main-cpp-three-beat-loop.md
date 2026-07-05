---
id: '005'
title: CommandProcessor pure transformer, CommandQueue deletion, main.cpp three-beat
  loop
status: open
use-cases: [SUC-004, SUC-005, SUC-006, SUC-007]
depends-on: ['002', '003', '004']
github-issue: ''
issue:
- tick-model-command-flow-and-the-command-board-design-sketch.md
- rename-wire-lines-to-statements.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# CommandProcessor pure transformer, CommandQueue deletion, main.cpp three-beat loop

## Description

Reshape the DEV command layer so handlers stop calling `Hal`/`Drivetrain`
write methods directly, delete `CommandQueue`, and rewrite `main.cpp`'s loop
to the three-beat Part-2 shape. Depends on ticket 002 (Communicator's
`hasStatement()/takeStatement()`), ticket 003 (`Drivetrain::ports()/
active()/standby()`, `CommandProcessorToHalCommand`), and ticket 004
(`NezhaHal::apply()` overloads this ticket stages toward).

Per `architecture-update.md`'s "Config-plane vs. command-plane", "The
processor is `DevLoopState` + `CommandProcessor`", and "The Part-2 loop"
sections:

**`DevLoopState` reshape** (`source/commands/dev_commands.h`):
- Sheds `leftPort`/`rightPort`/`drivetrainActive` (replaced by
  `drivetrain->ports()`/`->active()`).
- Gains the outbox: `bool hasHalCommand; Hal::CommandProcessorToHalCommand
  halCommand; bool hasDrivetrainCommand; msg::DrivetrainCommand
  drivetrainCommand;`.
- Keeps `hal`/`drivetrain`/`watchdog` pointers and
  `motorConfigShadow[]`/`drivetrainConfigShadow` — these are still needed
  for reads (`STATE`/`CAPS`), config-plane writes (`CFG`, `PORTS`, `WD`),
  and capability-cache refresh, per the config-plane/command-plane split
  (Design Rationale 2 — do **not** route CFG/PORTS/WD through the outbox).

**Handler reshape** (`source/commands/dev_commands.cpp`):
- `DEV M <n> DUTY/VEL/POS/VOLT/NEUTRAL/RESET`: pre-validate via the new
  `Hal::motorCommandAllowed(caps, kind)` free function (extracted from
  `Hal::Motor::apply()`'s switch — see below) instead of calling
  `motor.apply(cmd)` to discover rejection after the fact; on acceptance,
  stage `state.hasHalCommand = true; state.halCommand = {false, 1,
  {{port, cmd}, {}}};` (single addressed entry) instead of calling
  `motor.apply()` directly. `isBoundPort()` reads `state.drivetrain->ports()`
  instead of `state.leftPort`/`rightPort`. On acceptance for a bound port,
  ALSO stage `state.hasDrivetrainCommand = true; state.drivetrainCommand =
  {control_kind=NONE, standby=true};` (authority steal, mode untouched).
- `DEV DT VW/WHEELS/NEUTRAL`: stage into `state.drivetrainCommand`/
  `hasDrivetrainCommand` instead of calling `drivetrain->apply()` directly.
- `DEV DT STOP`: stage `state.halCommand` with `count=2` addressing the
  bound pair (read via `drivetrain->ports()`) + `state.drivetrainCommand =
  {NEUTRAL, standby=true}` — do **not** broadcast.
- `DEV STOP`: stage a broadcast `state.halCommand` (`allPorts=true`) +
  `state.drivetrainCommand = {NEUTRAL, standby=true}`, via a new shared
  free-function pair `buildBroadcastNeutral(msg::Neutral)` /
  `buildDrivetrainStop(msg::Neutral)` (the "one audited path" — also used
  by `main.cpp`'s watchdog-fire path below).
- `DEV M <n> CFG`/`DEV DT CFG`/`DEV DT PORTS`/`DEV WD`: **unchanged** —
  these stay direct, parse-time, config-plane calls
  (`motor.configure()`/`drivetrain.configure()`/`watchdog.setWindow()`),
  including `DEV DT PORTS`'s capability-cache refresh
  (`drivetrain->setMotorCapabilities(...)`), per the config-plane/
  command-plane split. `DEV DT PORTS` now merges `left_port`/`right_port`
  into `drivetrainConfigShadow` and calls `configure()`, same shape as any
  other CFG key.
- `emitDrivetrainState()`'s `active=`/`ports=` fields read
  `state.drivetrain->active()`/`->ports()` instead of
  `state.drivetrainActive`/`leftPort`/`rightPort`.

**`Hal::Motor::apply()` refactor** (`source/hal/capability/motor.h`):
extract the capability-gate switch into a reusable free function
`inline bool motorCommandAllowed(const msg::MotorCapabilities& caps,
msg::MotorCommand::ControlKind kind)`; `apply()` calls it in place of its
inline `if` checks (behavior unchanged, defined once).

**`CommandQueue` deletion** (decision 5): delete
`source/commands/command_queue.h`; remove `setQueue()`/`hasQueue()`/
`dequeueOne()`/`_queue` from `command_processor.{h,cpp}`; remove the
`#include "command_queue.h"` from both files.

**`main.cpp` three-beat loop**: implement exactly per
`architecture-update.md`'s "The Part-2 loop" code block — `hal.tick(now)`
twice per pass (slice 1 collects at the top, slice 2 requests/writes after
the outbox drain + Drivetrain tick), the outbox drain (`devState.
hasHalCommand`/`hasDrivetrainCommand`), `drivetrain.active()`/`ports()`/
`hasCommand()`/`takeCommand()`, and the watchdog-fire path calling
`buildBroadcastNeutral()`/`buildDrivetrainStop()` directly (not via the
outbox — main.cpp is top-of-tree). Remove the old bound-pair explicit
double-tick (`hal.motor(leftPort).tick(now); hal.motor(rightPort).tick(now);`
after the main sweep) — it's superseded by the sanctioned double
`hal.tick()` call.

## Acceptance Criteria

- [ ] No `dev_commands.cpp` handler calls `Hal::Motor::apply()` or
      `Subsystems::Drivetrain::apply()` directly for a setpoint-shaped verb
      (DUTY/VEL/POS/VOLT/NEUTRAL/RESET/VW/WHEELS/NEUTRAL/STOP/STOP) — grep
      confirms; CFG/PORTS/WD handlers still call `configure()`/
      `setWindow()` directly (unchanged, config-plane).
- [ ] `Hal::motorCommandAllowed()` exists in `hal/capability/motor.h`;
      `Motor::apply()` uses it; `dev_commands.cpp` uses it for
      pre-validation before staging.
- [ ] `DevLoopState` has the outbox fields; `leftPort`/`rightPort`/
      `drivetrainActive` are gone.
- [ ] `isBoundPort()` reads `drivetrain->ports()`.
- [ ] `DEV DT STOP` stages an addressed (count=2, the bound pair) HAL
      command, not a broadcast — host test confirms an independent,
      unbound motor is untouched by `DEV DT STOP`.
- [ ] `DEV STOP`/watchdog fire both use `buildBroadcastNeutral()`/
      `buildDrivetrainStop()`; watchdog fire applies them immediately
      (not via the outbox); `DEV STOP` stages them.
- [ ] `source/commands/command_queue.h` is deleted; no remaining reference
      to `CommandQueue` anywhere in `source/` (grep confirms, matching the
      source issue's own pre-verified isolation).
- [ ] `main.cpp` matches the three-beat loop shape (both `hal.tick()`
      slices, outbox drain, no old bound-pair explicit double-tick).
- [ ] Host tests (structs in/out, no fakes — per the Test Strategy) cover:
      capability pre-validation rejecting an unsupported mode before
      staging; a bound-port DUTY/VEL/etc. staging both a HAL command and a
      standby-only Drivetrain command; an unbound-port command leaving
      Drivetrain untouched; `DEV STOP`'s broadcast shape;
      `DEV DT STOP`'s addressed-pair shape.
- [ ] Both `ROBOT_DEV_BUILD` forks build; the non-`ROBOT_DEV_BUILD` `#else`
      fallback in `main.cpp` still builds and still has no DEV/HAL wiring.
- [ ] Wire behavior is unchanged: every `OK`/`ERR` reply text, verb, and
      `DEV STATE`/`CAPS` field shape is byte-identical to before this
      ticket (bench smoke check — full stand pass is ticket 006).

## Implementation Plan

**Approach**: `Motor::apply()`'s `motorCommandAllowed()` extraction first
(small, mechanical, testable in isolation), then `DevLoopState`'s outbox
fields + handler reshape (the bulk of the ticket, one command family at a
time: `DEV M` motion verbs, then `DEV DT` motion verbs, then the two STOP
paths), then `CommandQueue` deletion (independent, can happen any time
after the queue's last real caller is confirmed gone — it already has none),
then `main.cpp`'s loop rewrite last (integrates everything).

**Files to modify**:
- `source/hal/capability/motor.h` — `motorCommandAllowed()`.
- `source/commands/dev_commands.h` — `DevLoopState` reshape.
- `source/commands/dev_commands.cpp` — every motion-verb handler,
  `isBoundPort()`, `emitDrivetrainState()`, `neutralizeAll`/
  `neutralizeDrivetrain` replaced by `buildBroadcastNeutral()`/
  `buildDrivetrainStop()`.
- `source/commands/command_processor.h`, `.cpp` — remove `CommandQueue`
  integration.
- `source/main.cpp` — full Part-2 loop.
- `docs/protocol-v2.md` §16 — correct the "every DEV handler... dispatches
  it through `apply()`... rather than calling a primitive setter directly"
  sentence to describe the outbox handoff (this is the doc-sweep item this
  sprint's rename issue flagged as belonging with the behavioral change,
  not the vocabulary ticket).

**Files to delete**:
- `source/commands/command_queue.h`.

**Testing plan**:
- Existing tests: `uv run python -m pytest`; `just build` both forks.
- New tests: host-level (`DevLoopState` + real `Drivetrain`/a HAL test
  double or the real `NezhaHal` against ticket 001's scripted fake, per
  what's cheapest to wire — the design sketch says Drivetrain/processor
  "need no fakes at all," so prefer plain structs and a minimal test HAL
  stand-in over the full `NezhaHal`) exercising each acceptance criterion
  above as a discrete case.
- Bench smoke check (light): `PING`, `DEV M 1 DUTY 30`, `DEV M 1 STATE`,
  `DEV DT VW 100 0 0`, `DEV DT STATE`, `DEV STOP` over serial — confirm
  reply text is unchanged from before this ticket. Full stand gate is
  ticket 006.

**Documentation updates**: `docs/protocol-v2.md` §16 correction (above).
