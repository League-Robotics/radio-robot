---
id: '052'
title: "Stop conditions first-class \u2014 Phase 1 (additive)"
status: done
branch: sprint/052-stop-conditions-first-class-phase-1-additive
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
issues:
- stop-conditions-as-a-first-class-system-primitive.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 052: Stop conditions first-class — Phase 1 (additive)

## Goals

Expose the existing `StopCondition` infrastructure as a first-class wire
feature: any open-loop motion verb (VW, S, R, T, D) can carry one or more
`stop=<kind>:<args>` clauses, every EVT completion reports the reason it
stopped via a trailing `reason=<token>`, and the Python host library gains
builders and a structured return value for both.

This is strictly additive. No existing behavior is removed or altered.

## Problem

Today stop conditions are only partially exposed on the wire:

1. `sensor=` is attachable only to T, D, and TURN — not to VW, S, or R.
2. No stop reason is reported back on EVT completion — the host cannot tell
   whether a `T` ended because time elapsed or a sensor trip fired.
3. The Python protocol library offers no structured `stop=` builder and
   `wait_for_evt_done` returns only `"done" | "safety_stop" | "timeout"`.

## Solution

### 1a — Unified stop= parser (firmware)

Add `mc_parseStopToken` in `source/commands/MotionCommands.cpp`, generalizing
the existing `mc_parseSensorToken` to handle all 7 stop kinds using the
existing factory helpers in `StopCondition.h`. Accept `stop=` on VW, S, R,
T, and D. The `sensor=` form remains a back-compat alias. Each parsed clause
calls `mc.addStop(...)`.

### 1b — Record and report stop reason (firmware)

Extend `MotionCommand` with `_firedKind` / `_firedChannel` fields set when
`tick()` detects a firing condition. Extend `emitEvt` to append
`reason=<token>`. Add `reason=watchdog` at the safety-stop emit site in
`Superstructure::evaluateSafety`.

### 1c — Host-side support (Python)

Add `Stop` builder class and extend all motion command methods in
`host/robot_radio/robot/protocol.py` to accept `stop=[...]`. Update
`wait_for_evt_done` to return `(outcome, reason)` tuple.

### 1d — Docs

Update `docs/protocol-v2.md` §10 and `source/COMMANDS.md`.

## Success Criteria

- `VW 200 0 stop=d:300` drives ~300 mm and emits `EVT done VW reason=dist`.
- `T 200 200 1000` (no explicit stop) emits `EVT done T reason=time`.
- `sensor=line0:ge:512` still works identically (back-compat).
- `EVT safety_stop reason=watchdog` emitted on watchdog fire.
- Simulation suite: `uv run --with pytest python -m pytest tests/simulation -q` — no new failures beyond the 2 pre-existing baseline.
- Firmware clean build: `python build.py --clean` exits 0.

## Scope

### In Scope

- Unified `stop=` parser for all 7 StopCondition kinds on VW, S, R, T, D.
- Back-compat `sensor=` alias.
- Recording fired stop kind/channel in MotionCommand.
- `reason=<token>` trailing token on EVT done and EVT safety_stop.
- Python `Stop` builder and updated `wait_for_evt_done` return.
- `docs/protocol-v2.md` §10 and `source/COMMANDS.md` updates.

### Out of Scope

- Phase 2: collapsing the stringify/re-parse round-trip (sprint 053).
- Collapsing Goal::STREAM/TIMED/DISTANCE/ARC/VELOCITY (sprint 053).
- Shrinking MotionCommand::Origin (sprint 053).
- G command stop= support (G is closed-loop; deferred to Phase 2).

## Test Strategy

Canonical command: `uv run --with pytest python -m pytest tests/simulation -q`

Pre-existing baseline: exactly 2 failures
(`test_default_config_pin::test_default_robot_config_unchanged`,
`test_robot_config::TestSchemaValidation::test_tovez_validates_against_schema`).
No new failures are acceptable.

Per-ticket test focus:
- Ticket 001: unit tests in `test_stop_condition_coverage.py` or
  `test_motion_command.py` covering each stop= kind; back-compat sensor=.
- Ticket 002: unit tests asserting `reason=<token>` on EVT strings; watchdog
  path tested in an existing or new scenario test.
- Ticket 003: tests in `test_protocol_v2.py` — Stop builder serialization,
  `wait_for_evt_done` tuple return.
- Ticket 005 (validation): full sim suite, firmware clean build, focused
  end-to-end scenario (VW + stop=d:300 → reason=dist).

## Architecture Notes

- `emitEvt` buffer expands from 48 to 80 chars to fit longest reason token.
- `reason=` is a trailing additive token: prefix-match on `EVT done T` still
  works for existing hosts.
- The golden-TLM canary captures only TLM lines; adding `reason=` to EVT
  lines does NOT affect the canary.
- `stop=` is a repeatable key; `ArgSchema.packKv` supports only a single key.
  Handlers iterate the raw token list for `stop=` themselves.

## GitHub Issues

(None linked yet — linked via issue frontmatter reference.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [ ] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Unified stop= parser (firmware) | — |
| 002 | Record and report stop reason (firmware) | 001 |
| 003 | Host-side stop= builders and reason= parsing | 002 |
| 004 | Documentation: protocol-v2.md and COMMANDS.md | 002 |
| 005 | Validation: sim tests, firmware clean build, end-to-end | 003, 004 |

Tickets execute serially in the order listed.
