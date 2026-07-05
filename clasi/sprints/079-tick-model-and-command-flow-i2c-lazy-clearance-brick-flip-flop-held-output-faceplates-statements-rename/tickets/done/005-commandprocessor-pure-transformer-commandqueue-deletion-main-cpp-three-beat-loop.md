---
id: '005'
title: CommandProcessor pure transformer, CommandQueue deletion, main.cpp three-beat
  loop
status: done
use-cases:
- SUC-004
- SUC-005
- SUC-006
- SUC-007
depends-on:
- '002'
- '003'
- '004'
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

- [x] No `dev_commands.cpp` handler calls `Hal::Motor::apply()` or
      `Subsystems::Drivetrain::apply()` directly for a setpoint-shaped verb
      (DUTY/VEL/POS/VOLT/NEUTRAL/RESET/VW/WHEELS/NEUTRAL/STOP/STOP) — grep
      confirms; CFG/PORTS/WD handlers still call `configure()`/
      `setWindow()` directly (unchanged, config-plane).
- [x] `Hal::motorCommandAllowed()` exists in `hal/capability/motor.h`;
      `Motor::apply()` uses it; `dev_commands.cpp` uses it for
      pre-validation before staging.
- [x] `DevLoopState` has the outbox fields; `leftPort`/`rightPort`/
      `drivetrainActive` are gone.
- [x] `isBoundPort()` reads `drivetrain->ports()`.
- [x] `DEV DT STOP` stages an addressed (count=2, the bound pair) HAL
      command, not a broadcast — host test confirms an independent,
      unbound motor is untouched by `DEV DT STOP`.
- [x] `DEV STOP`/watchdog fire both use `buildBroadcastNeutral()`/
      `buildDrivetrainStop()`; watchdog fire applies them immediately
      (not via the outbox); `DEV STOP` stages them.
- [x] `source/commands/command_queue.h` is deleted; no remaining reference
      to `CommandQueue` anywhere in `source/` (grep confirms, matching the
      source issue's own pre-verified isolation).
- [x] `main.cpp` matches the three-beat loop shape (both `hal.tick()`
      slices, outbox drain, no old bound-pair explicit double-tick).
- [x] Host tests (structs in/out, no fakes — per the Test Strategy) cover:
      capability pre-validation rejecting an unsupported mode before
      staging; a bound-port DUTY/VEL/etc. staging both a HAL command and a
      standby-only Drivetrain command; an unbound-port command leaving
      Drivetrain untouched; `DEV STOP`'s broadcast shape;
      `DEV DT STOP`'s addressed-pair shape.
- [x] Both `ROBOT_DEV_BUILD` forks build; the non-`ROBOT_DEV_BUILD` `#else`
      fallback in `main.cpp` still builds and still has no DEV/HAL wiring.
- [x] Wire behavior is unchanged: every `OK`/`ERR` reply text, verb, and
      `DEV STATE`/`CAPS` field shape is byte-identical to before this
      ticket (bench smoke check — full stand pass is ticket 006).

## Stand-Smoke Findings (important — read before ticket 006)

The required light bench smoke (PING, `DEV M 1 DUTY 30` + `STATE`,
`DEV DT VW`/`STATE`, `DEV STOP`, watchdog fire) confirms wire text is
byte-identical to before this ticket. But driving the smoke sequence on the
real robot (NEZHA2, `/dev/cu.usbmodem2121102`) surfaced two real findings,
isolated by bisecting with `git stash` (this ticket's changes vs. ticket
004's landing, both flashed and driven on the same hardware):

1. **Ticket 004's `dev_commands.cpp` (pre-this-ticket) never actually
   activated the brick flip-flop.** Every DEV M/DEV DT handler called
   `Hal::Motor::apply()` on the leaf reference returned by
   `hal->motor(port)` directly — never `Hal::NezhaHal::apply()` (the
   distribution overload that sets `portInUse_[port] = true`). With no
   port ever marked in-use, `NezhaHal::tick()`'s `anyPortInUse()` guard
   was permanently false, so `NezhaMotor::tick()` — and therefore
   `armoredWrite()`/`collectEncoder()` — never ran for ANY port since
   ticket 004 landed. Confirmed on hardware: pre-ticket, `DEV M 1 DUTY 30`
   replies `OK ... applied=0.30` but every subsequent `STATE` poll reports
   `applied=0.00`, `wedged=0` (the wedge detector never even runs).
   **This ticket's outbox reshape fixes this as a byproduct** — `main.cpp`
   now drains `devState.halCommand` through the real `hal.apply()`, which
   does mark ports in-use, so this is the first code to correctly engage
   the flip-flop for DEV-sourced commands, exactly per decision 1.
2. **Once genuinely activated, the flip-flop's encoder collect never
   reflects real motion.** Post-ticket, on ALL FOUR ports, `DEV M <n> DUTY
   30` drives `applied` to the commanded value and the port's wedge
   detector fires (`wedged=1`) within ~1 s (proving `tick()`/
   `collectEncoder()` are now really running every cycle), but `pos`/`vel`
   never move from their post-reset baseline — the same symptom `DEV M <n>
   VEL 120` produces (PID integrator saturates to `applied=1.00` chasing a
   permanently-zero feedback). This matches architecture-update.md's own
   flagged, explicitly-unverified **Risk 1 ("Shared-0x10 clobber... an
   abandoned collect... is a hardware timing question, not a code-review
   one, and stays ticket 006's stand-gate responsibility")** — ticket 004
   itself was never stand-tested ("No stand pass required in this ticket
   (ticket 006 covers hardware)"), so this is very likely the first time
   the real flip-flop timing has been exercised against actual hardware.

Neither finding is in this ticket's scope to fix (both live in
`nezha_hal.cpp`/`nezha_motor.cpp`/the request-collect timing, not the
command layer this ticket touches), and the wire-level acceptance criterion
above is satisfied regardless. Flagging prominently for the team lead /
ticket 006: **real closed-loop motor motion is not currently confirmed
working on the stand** — ticket 006's stand gate needs to budget real
investigation into the collect timing (not just a `vel_filt_alpha`
retune) before its own acceptance can be met. The robot was left in a
safe, fully-neutral state (`DEV STOP`, `DEV WD 1000` restored) at the end
of every session.

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
