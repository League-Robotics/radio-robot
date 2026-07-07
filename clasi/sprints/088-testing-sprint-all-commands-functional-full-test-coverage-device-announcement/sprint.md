---
id: 088
title: 'Testing sprint: all commands functional + full test coverage + device announcement'
status: planning-docs
branch: sprint/088-testing-sprint-all-commands-functional-full-test-coverage-device-announcement
use-cases: []
issues:
- tovez-drive-motor-reversed-fwd-sign.md
- help-should-reflect-registered-commands.md
- robot-device-announcement-on-connect-and-hello.md
- remove-statement-terminology.md
- full-command-smoke-test-suite.md
- rebuild-test-suite-and-verify-commands-functional.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 088: Testing sprint: all commands functional + full test coverage + device announcement

## Goals

Get the robot's command surface actually functioning and prove it: every
registered command exists, dispatches correctly on both serial and radio,
and — for motion and configuration verbs — visibly does the right thing on
the stand (encoders incrementing, wheels rolling the correct direction).
This is a testing-focused sprint; the stakeholder's north star is "get the
robot actually functioning, where all the commands are there."

## Problem

Six concrete defects/gaps block that bar, plus one large terminology
cleanup the stakeholder wants done now rather than later:

1. `VER` is registered but never replies (times out) despite being
   structurally identical to `PING`.
2. The two drive-pair motors are mirror-mounted; boot config bakes
   `fwd_sign=+1` on every port, so straight-drive commands spin the wheels
   in opposite directions instead of both going forward.
3. `HELP` returns a hardcoded 5-verb string instead of the live registered
   command table, so the stakeholder can't see that motion/config/etc.
   verbs are actually wired up.
4. There is no boot-time or on-demand device-identity announcement
   (`DEVICE:NEZHA2:robot:<name>:<serial>`) on serial or radio, though the
   host already parses and caches this line.
5. The stakeholder has reversed a prior sprint's naming decision: the term
   "statement" must be removed. Wire-inbound things are "commands";
   internal typed representations are "messages." ~85 source sites, 1 host
   site, and the naming rule itself need renaming.
6. Test coverage of the registered command surface is incomplete: there is
   no smoke test proving every command dispatches on both channels, no
   completeness guard against future command-family additions, and no
   documented on-stand proof that every motion/config command drives real
   hardware (encoders) correctly.

## Solution

Fix the three small firmware defects first (VER, wheel sign, HELP) so the
command surface is honest and drivable. Land the device-announcement
feature and the statement→command/message rename early, so the test
infrastructure built later in the sprint targets final names and can
exercise the announcement. Extend the sim command harness so a test can
choose SERIAL vs RADIO as the return channel and observe the reply on that
channel (today both resolve to one sink). Build the per-command smoke
suite plus a completeness meta-test against the live registered table, and
fill in behavioral sim coverage for the subsystems backing motion/config
commands. Close with a mandatory on-the-stand bench pass (wheels off the
ground, safe to drive) that proves every motion verb drives the wheels
with encoders incrementing correctly and every config verb takes effect,
over the real link (serial + radio relay).

## Success Criteria

- `VER` replies correctly on both channels.
- A straight `D`/`T`/`S` command drives both wheels forward with both
  encoders incrementing positive; the fix is baked in `gen_boot_config.py`
  + the robot JSON, not hand-edited into generated code.
- `HELP` enumerates the live registered command table (system + dev +
  telemetry + motion + config + pose + otos), sourced from the table
  itself, not a literal string.
- The device-announcement banner is the first line on both serial and
  radio at boot, and `HELLO` re-emits it on the arriving channel.
- No `statement`/`Statement` identifier remains in `source/` or `host/`;
  the naming rule is rewritten to the command/message vocabulary; firmware
  and host build and all tests stay green (pure rename, no behavior
  change).
- Every registered command has a smoke test exercising both channels; a
  completeness meta-test fails if a registered verb lacks one.
- Every motion and config command has behavioral sim coverage in addition
  to its smoke test.
- On the stand: every motion verb (`S T D R TURN RT G STOP`) drives the
  wheels with encoders incrementing as expected, and every config verb's
  effect is observable, over the real link (serial + radio relay) —
  captured as a bench checklist/log in the sprint.
- `uv run python -m pytest` (the `tests/sim/` + `tests/unit/` gate) is
  green throughout.

## Scope

### In Scope

- `VER` dispatch/wiring bug fix.
- Per-port `fwd_sign` in `scripts/gen_boot_config.py` + `data/robots/
  tovez*.json` (config-generation fix, not a hand-edit of generated code).
- `HELP` dynamic enumeration of the live `CommandProcessor` table.
- Boot + `HELLO` device-announcement banner (`Communicator::begin()` hook,
  re-added `HELLO` verb).
- The full statement→command/message rename sweep (source, host, naming
  rule, live docs).
- Sim command harness channel extension (SERIAL vs RADIO, distinct reply
  sinks) — prerequisite for the smoke suite.
- Full per-command smoke-test suite + completeness meta-test.
- Behavioral sim coverage of the subsystems backing motion/config commands
  (`Drivetrain`, `Hal::Motor`/`NezhaMotor` via `SimMotor`, `PoseEstimator`,
  `Planner`, `Communicator`, `CommandRouter`/`Configurator`) where gaps
  exist.
- Mandatory on-the-stand bench verification of every motion + config verb
  via encoders, over serial and radio relay, sequenced last (after the
  wheel-direction fix and the rename land).

### Out of Scope

- A from-scratch rebuild of `tests_old/`'s full historical coverage
  (bounded per `rebuild-test-suite-and-verify-commands-functional.md`).
- `tests/playfield/` (stays parked — needs the playfield, not the stand).
- Full validation of commands that can't be exercised on the stand (OTOS
  absolute position without real translation, camera `SI` pose-inject,
  playfield-frame `G`/goto) — smoke/dispatch-level only, documented.
- The `ID` verb's pre-existing model-token casing mismatch and missing
  `caps=` field (noted in the device-announcement issue, explicitly
  deferred).
- Renaming pre-existing `msg::*Command`-suffixed internal payload types
  (e.g. `msg::MotorCommand`) or the `Hal::DrivetrainToHardwareCommand`-style
  edge types that already used "Command" to mean "carries a parsed
  message" — out of scope for this sprint's rename (see
  architecture-update.md's rename rationale).
- `rt-open-loop-overshoot-under-synchronous-update.md` and
  `watchdog-arm-only-while-motors-running.md` — separate issues, not
  linked to this sprint.

## Test Strategy

Three layers, matching `tests/CLAUDE.md`'s domain split:

1. **`tests/sim/unit/`** — the extended channel-aware harness backs both
   the new per-command smoke suite (one test per registered verb, serial +
   radio, well-formed-reply assertion) and new/expanded behavioral tests
   for the subsystems that implement motion/config commands. A
   completeness meta-test enumerates the live command table (via the same
   mechanism `HELP` now uses) and fails if a registered verb has no smoke
   test.
2. **`tests/bench/`** — CLI tools (existing + any refreshed) drive the
   on-stand functional verification: every motion verb observed via
   encoders, every config verb's effect observed, over serial and radio
   relay.
3. **`tests/playfield/`** — untouched, stays parked.

`uv run python -m pytest` (collecting `tests/sim/` + `tests/unit/` per
`pyproject.toml`'s `testpaths`) must stay green throughout. The bench pass
is captured as a written log/checklist in the sprint, not pytest-automated
(HITL by nature).

## Architecture Notes

See `architecture-update.md` for the full design. Summary: no new runtime
module. This sprint modifies existing components (`system_commands.cpp`
gains `HELP`'s live-table access and a new `HELLO` handler; `Communicator`
gains a boot-announcement call site; `scripts/gen_boot_config.py` gains
per-port `fwd_sign`), executes a large but mechanical identifier rename
across the sprint-087 command/message vocabulary, and extends the sim
harness (`tests/_infra/sim/sim_api.cpp`) with a second, channel-distinct
reply sink.

## GitHub Issues

(No GitHub issues linked to this sprint yet.)

## Missing issue file

`ver-command-returns-no-reply.md` was referenced in this sprint's dispatch
but does not exist in `clasi/issues/` (confirmed: not present on disk, not
in git history, not findable anywhere in the repo). The VER bug itself is
real and is investigated directly in `architecture-update.md` (structural
comparison against `PING` — no compile-time or dispatch-table defect found
by static reading; genuinely requires bisection on-target during ticket
execution). This issue is included in the sprint's ticket plan despite the
missing file. The team-lead should create the issue file (or confirm one
should not exist) so `clasi/issues/` and this sprint's linkage stay
consistent; only 6 of the 7 referenced issues are linked above.

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [ ] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [ ] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | VER command dispatch/wiring bug fix | — |
| 002 | Per-port fwd_sign wheel-direction fix | — |
| 003 | HELP dynamic command-table enumeration | — |
| 004 | Remove statement terminology: rename to command/message vocabulary | — |
| 005 | Device announcement: boot banner and HELLO verb | 003 |
| 006 | Sim command harness channel extension (SERIAL vs RADIO) | 004 |
| 007 | Full command smoke-test suite and completeness meta-test | 001, 002, 003, 005, 006 |
| 008 | Behavioral sim coverage for motion and config command subsystems | 006, 007 |
| 009 | On-stand bench functional verification via encoders | 001, 002, 003, 004, 005 |

Tickets execute serially in the order listed.
