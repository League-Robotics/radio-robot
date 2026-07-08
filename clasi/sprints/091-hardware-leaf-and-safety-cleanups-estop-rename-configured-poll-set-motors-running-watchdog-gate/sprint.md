---
id: 091
title: 'Hardware-leaf and safety cleanups: estop rename, configured poll-set, motors-running
  watchdog gate'
status: ticketing
branch: sprint/091-hardware-leaf-and-safety-cleanups-estop-rename-configured-poll-set-motors-running-watchdog-gate
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
issues:
- rename-emergencyneutralize-to-estop.md
- replace-portinuse-with-configured-poll-set.md
- watchdog-arm-only-while-motors-running.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 091: Hardware-leaf and safety cleanups: estop rename, configured poll-set, motors-running watchdog gate

## Goals

Close out three small, independent pool issues that clean up a hardware-leaf
naming debt and two safety-relevant gaps left over from sprints 087-090:

1. Rename `Rt::MainLoop::emergencyNeutralize()` to `estop()` — a pure,
   zero-behavior-change rename that the watchdog work (goal 3) then builds
   on, so it goes first.
2. Replace `NezhaHardware`'s command-derived, latch-forever `portInUse_`
   schedule flag with an explicit, configured poll-set established once at
   construction — deleting three side-effect write sites and the broadcast-
   exemption special case.
3. Gate the serial-silence safety watchdog's FIRE on motors actually
   running (commanded, not measured), so an idle robot sitting quietly
   never gets a spurious neutralize + `EVT dev_watchdog`, while a live
   drive command going silent still fires exactly as today.

## Problem

- `emergencyNeutralize()` carries a stakeholder `// FIXME rename to "estop"`
  comment (in-progress since 2026-07-07, deferred out of sprint 088). It is
  about to be invoked by name in the watchdog work below, so it should be
  renamed first rather than layering more call sites onto the old name.
- `NezhaHardware::portInUse_` is presented as an "in-use" ownership flag but
  its only real job is I2C poll-schedule membership; it is mutated as a
  side effect of ordinary command flow (3 scattered write sites), never
  released ("a single `DEV M 3` during a bench session permanently adds
  port 3 to the round-robin... with no way back short of reboot"), and is
  invisible to `SimHardware`/sim tests (no schedule concept there at all).
- The serial-silence watchdog fires on ANY comms silence past its window,
  including while the robot is completely idle with motors stopped — a
  spurious neutralize + `EVT dev_watchdog` with no runaway to prevent. The
  watchdog's purpose is to catch a *live drive command* going unmonitored;
  idle silence should stay quiet.

## Solution

- Ticket 001: mechanical rename, `main_loop.h`/`main_loop.cpp` + the two
  test files that reference the old name by comment.
- Ticket 002: add a `polled` fact to `msg::MotorConfig` (config-plane,
  established once at `NezhaHardware` construction as a constant
  `polled_[kPortCount]` mask; `anyPortInUse()`/`nextPortInUse()` renamed to
  `anyPolled()`/`nextPolled()`), baked at boot to the robot's normal drive
  pair (`LEFT_PORT`/`RIGHT_PORT`, matching `gen_boot_config.py`'s existing
  per-port specialization), with an explicit `DEV M <n> CFG polled=<bool>`
  escape hatch so the coupled PID/governor bench rig (ports 3/4, or any
  standalone port — see `docs/protocol-v2.md` §16, `tests/bench/
  pid_hold_speed.py`, `tests/bench/ratio_governor_curve.py`) can keep
  working exactly as it does today, just via an explicit config action
  instead of an implicit command side effect. `DEV M <n>` addressed at an
  unpolled port is rejected `ERR nodev` (the ticket's one deliberate,
  documented behavior change — see architecture-update.md Decision 2).
- Ticket 003: add a small, symmetric `active` bool to `Hal::Motor`/
  `msg::MotorState` (commanded, toggled in the base class's existing
  NEUTRAL-special-cased `apply()` dispatch — mirrors `DrivetrainState.
  active`'s own semantics exactly), then gate the watchdog's fire on
  `bb.drivetrain.active || any(bb.motors[i].active)` instead of firing
  unconditionally. Sim tests prove both the idle-no-fire and driving-fires
  cases (extending `tests/sim/unit/test_watchdog_policy.py`). The issue's
  radio-path HITL bench cannot be run this sprint (the relay dongle is
  unplugged) — deferred to a fresh `clasi/issues/` follow-on, not left as
  an unmet/blocking acceptance criterion.

## Success Criteria

- `uv run python -m pytest tests/sim` green (baseline 309 passed / 2
  xfailed), including new tests for both deliberate behavior changes.
- No `emergencyNeutralize` identifier remains anywhere in the tree.
- `portInUse_`, its three write sites, and the broadcast-exemption branch
  are gone; `anyPolled()`/`nextPolled()` read a constant, config-established
  mask.
- The watchdog does not fire while idle; fires exactly as before while
  driving.
- A fresh follow-on issue exists for the deferred radio-path HITL watchdog
  bench.

## Scope

### In Scope

- `source/runtime/main_loop.{h,cpp}` (estop rename; watchdog fire-gate).
- `source/subsystems/nezha_hardware.{h,cpp}` (poll-set).
- `source/hal/capability/motor.h` (new `active` bool, base-class `apply()`).
- `source/messages/motor.h` + `protos/motor.proto` (`MotorConfig.polled`,
  `MotorState.active` — regenerated via `scripts/gen_messages.py`).
- `source/config/boot_config.cpp` + `scripts/gen_boot_config.py`
  (`polled` baked to the drive pair).
- `tests/_infra/sim/sim_api.cpp` (sim's own `defaultMotorConfigSet()` needs
  the same `polled` baking so existing sim tests keep passing).
- `source/commands/dev_commands.{h,cpp}` (the `DEV M <n> CFG polled=`
  escape hatch; the `ERR nodev` rejection gate).
- `tests/sim/unit/test_watchdog_policy.py`, `test_nezha_flipflop.py`
  (+ its harness), `test_dev_command_outbox.py` (+ its harness),
  `nezha_flipflop_harness.cpp`, `dev_command_outbox_harness.cpp`.
- `docs/protocol-v2.md` (poll-set/`polled` CFG key + watchdog fire-gate
  documentation updates).

### Out of Scope

- The radio-path HITL watchdog bench (relay dongle unplugged this run) —
  deferred to a fresh issue.
- Any change to `SimHardware`'s all-ports-always-tick scheduling (it has no
  poll-set concept and none is being added — only the command-layer
  `polled` gate applies uniformly to both Hardware owners).
- Mecanum/holonomic drivetrain support (still out of scope per sprint 048;
  this drivetrain remains differential-only).
- Any wire-format change beyond the new `DEV M <n> CFG polled=` key and the
  `ERR nodev` reply — no existing verb/reply shape changes.

## Test Strategy

All three tickets are gated on `uv run python -m pytest tests/sim` staying
green. Ticket 001 is a pure rename (no new test, existing tests keep
passing verbatim once the identifier is updated in the two files that
reference it). Ticket 002 updates `test_nezha_flipflop.py`'s harness
scenarios (idle/in-use/rotation semantics all shift from "command-derived"
to "config-derived") and `test_dev_command_outbox.py`'s harness (the
unbound-port scenario now proves the `ERR nodev` rejection, plus a new
scenario for the `polled=true` escape hatch accepting a previously-rejected
port). Ticket 003 extends `test_watchdog_policy.py` with an explicit
idle-silence-does-not-fire test and confirms the existing driving-fires
tests still pass under the new gated `check()` call.

## Architecture Notes

See `architecture-update.md` for the full methodology. Three load-bearing
decisions worth flagging here:

1. **Poll-set escape hatch (ticket 002).** The issue's literal text ("never
   mutated by command flow... established once at construction") would, if
   taken as an absolute boot-time-only fact, silently break the coupled
   PID/governor bench rig's standalone-port workflow (`ratio_governor_
   curve.py`'s primary protocol drives port 4 independently of whatever
   pair the Drivetrain is bound to; `pid_hold_speed.py` drives ports 3/4
   standalone). This sprint resolves the tension by keeping poll-set
   membership boot-config-established BUT giving it one explicit,
   config-plane (not command-plane) mutator: `DEV M <n> CFG polled=<bool>`.
   This preserves the issue's real goal (delete hidden, one-way,
   command-derived side effects) without breaking real, currently-working
   bench tooling.
2. **`DEV M <n>`-on-unpolled-port decision (ticket 002).** Rejected with
   `ERR nodev`, mirroring the existing capability-rejection shape
   (`ERR unsupported <mode>`) and the existing device-presence convention
   (`ERR nodev` on OI/OZ/OR/OV with no odometer) — chosen over "applied but
   silently unsampled" because an unpolled port's `VEL`/`POS` closed loop
   would never actually run (no `tick()`), which is a worse, silent-failure
   footgun than a clear error.
3. **Watchdog fire-gate predicate (ticket 003).** `DrivetrainState.active`
   alone is insufficient — it goes false the instant any bound-port `DEV M`
   motion verb steals authority (`isBoundPort()`'s standby-steal), so a
   spinning wheel under a bare `DEV M 1 VEL 100` would otherwise never trip
   the watchdog. The gate is `bb.drivetrain.active || any(bb.motors[i].
   active)`, where the new per-port `active` bool is commanded (toggled in
   `Hal::Motor::apply()`'s existing NEUTRAL-special-case dispatch), never
   measured. Standalone bench-motor sessions are explicitly out of this
   gate's field-safety threat model (HITL, human always present per
   `.claude/rules/hardware-bench-testing.md`) only insofar as this ticket
   does not add anything BEYOND per-port commanded state — which it does
   cover, so the bench case is in fact protected too.

## GitHub Issues

(No GitHub issues linked; this sprint addresses three `clasi/issues/` pool
items: `rename-emergencyneutralize-to-estop.md`,
`replace-portinuse-with-configured-poll-set.md`,
`watchdog-arm-only-while-motors-running.md`.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [x] Stakeholder has approved the sprint plan (autonomous auto-approve mode — see team-lead dispatch instructions)

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Rename `Rt::MainLoop::emergencyNeutralize()` to `estop()` | — |
| 002 | Configured poll-set: replace `NezhaHardware`'s `portInUse_` with `polled_` + a `DEV M CFG polled=` escape hatch | 001 |
| 003 | Gate the serial-silence watchdog's fire on commanded motors-running state | 001, 002 |

Tickets execute serially in the order listed.
