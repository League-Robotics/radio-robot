---
id: '007'
title: Full command smoke-test suite and completeness meta-test
status: done
use-cases:
- SUC-007
depends-on:
- '001'
- '002'
- '003'
- '005'
- '006'
github-issue: ''
issue: full-command-smoke-test-suite.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Full command smoke-test suite and completeness meta-test

## Description

Stakeholder request: a smoke test per registered command, each proving
the command (1) can be sent through serial, (2) can be sent through
radio, (3) produces an effect (a well-formed reply — `OK ...` or a
*defined* `ERR ...` — proving reachability, not hardware presence, for
hardware-gated verbs like OTOS `OI`/`OL`/`OA` on `SimMotor`). The
registered surface is `systemCommands` (PING VER HELP ECHO ID HELLO) +
`devCommands` (DEV family) + `telemetryCommands` (STREAM SNAP) +
`motionCommands` (S T D R TURN RT G STOP) + `configCommands` (SET GET) +
`poseCommands` (SI ZERO) + `otosCommands` (OI OZ OR OP OV OL OA). Depends
on tickets 001/002/003/005 (VER/fwd_sign/HELP/HELLO landing so the
commands under test are actually correct) and ticket 006 (the
channel-aware harness this suite is built on).

## Implementation Plan

**Approach**: Enumerate the live registered command table (reuse ticket
003's `listVerbs()`-backed mechanism, exposed to the test harness if not
already reachable, or drive the enumeration off `HELP`'s own live reply)
to keep the smoke list and the completeness guard both sourced from the
same live table, not a second hand-maintained list. Write one smoke-test
function per registered command; DEV family granularity is one test per
subcommand (`DEV M`, `DEV DT`, `DEV WD`, `DEV STATE`, etc. — matching the
same unit the registered table uses), not one monolithic `DEV` test. Each
test calls ticket 006's `sim_command_on(..., CHANNEL_SERIAL)` then
`sim_command_on(..., CHANNEL_RADIO)` and asserts a well-formed reply on
each. Add a completeness meta-test that enumerates the live table and
fails if any registered verb lacks a smoke test (and vice versa) — verify
it actually fails by temporarily hiding a verb during development (do not
commit that state).

**Files to create/modify**: new file(s) under `tests/sim/unit/`, e.g.
`test_command_smoke.py` (smoke suite) and a completeness meta-test in the
same file or a sibling.

**Testing plan**: this ticket's product IS the test suite — verify by
running it directly, and by confirming the completeness guard is a real
guard (temporarily drop a verb, confirm the meta-test fails, then
restore).

**Documentation updates**: none required.

## Acceptance Criteria

- [x] One smoke-test function per registered command (or per `DEV`
      subcommand, matching `HELP`'s granularity), each exercising both
      SERIAL and RADIO via ticket 006's harness.
- [x] Each smoke test asserts a well-formed reply (`OK ...` or a defined
      `ERR ...`), proving reachability, not hardware presence.
- [x] A completeness meta-test enumerates the live registered table and
      fails if any registered verb lacks a smoke test (and vice versa) —
      confirmed to actually fail when a verb is temporarily removed
      during development (not committed in that state).
- [x] Lands in `tests/sim/unit/` (reconciling "tests/unit" with the
      harness's actual location — `architecture-update.md` Decision 6 —
      because the suite needs the ctypes-loaded firmware sim harness that
      only `tests/sim/`'s fixtures wire up).
- [x] `uv run python -m pytest` runs the suite green as part of the
      standard gate.

## Testing

- **Existing tests to run**: full `uv run python -m pytest`.
- **New tests to write**: the smoke suite + completeness meta-test itself
  (this ticket's primary deliverable).
- **Verification command**: `uv run python -m pytest tests/sim/unit/test_command_smoke.py`
  then the full `uv run python -m pytest`.
