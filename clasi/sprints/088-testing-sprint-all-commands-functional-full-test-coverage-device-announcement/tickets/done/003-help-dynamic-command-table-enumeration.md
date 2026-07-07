---
id: '003'
title: HELP dynamic command-table enumeration
status: done
use-cases:
- SUC-003
depends-on: []
github-issue: ''
issue: help-should-reflect-registered-commands.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# HELP dynamic command-table enumeration

## Description

`HELP` (`system_commands.cpp:62-67`) returns a hardcoded string
(`"PING VER HELP ECHO ID"`) instead of enumerating the live registered
command table. The other six command families are wired up and functional
(`command_router.cpp:24-41`'s `buildTable()`); `HELP` just doesn't report
them. `HELP`'s handler has no path to the live table today because
`systemCommands()` takes no arguments (called as `systemCommands()` at
`command_router.cpp:25`), while the other six families are already called
as `family(router)` and already `#include "runtime/command_router.h"`
(confirmed via `dev_commands.h:121,184`).

## Implementation Plan

**Approach**: Add `int CommandProcessor::listVerbs(char* buf, int size) const`
(`command_processor.{h,cpp}`) that iterates `_cmds` and space-joins each
descriptor's `prefix` — keeping `_cmds` private. Add a forwarding
`int Rt::CommandRouter::listVerbs(char* buf, int size) const { return processor_.listVerbs(buf, size); }`
(`command_router.{h,cpp}`). Change `systemCommands()` to
`systemCommands(Rt::CommandRouter& router)`, joining the other six
families' construction signature — add `#include "runtime/command_router.h"`
to `system_commands.h` and update `command_router.cpp`'s `buildTable()`
call site from `systemCommands()` to `systemCommands(router)`. Bind only
`HELP`'s descriptor `handlerCtx = &router` (every other liveness handler's
`handlerCtx` stays `nullptr`). Rewrite `handleHelp` to cast `handlerCtx`
to `Rt::CommandRouter*`, call `listVerbs()`, and reply
`OK help <verbs> [#id]`.

**Files to create/modify**: `source/commands/command_processor.{h,cpp}`,
`source/runtime/command_router.{h,cpp}`, `source/commands/system_commands.{h,cpp}`.

**Testing plan**: a sim test sending `HELP` and asserting the reply
contains representative verbs from every family (not just the five
liveness verbs) — e.g. `SET`, `S `, `OI`, `DEV`. A regression check that
`HELP`'s reply fits its reply buffer without truncation.

**Documentation updates**: none — `docs/protocol-v2.md:237` already
documents the intended full-list `HELP` output; this ticket makes the
firmware match it.

## Acceptance Criteria

- [x] In the dev build, `HELP` lists every registered verb (system + dev +
      telemetry + motion + config + pose + otos), not just the five
      liveness verbs.
- [x] `CommandProcessor::_cmds` stays private; the new `listVerbs()`
      accessor is the only way `HELP` reaches it.
- [x] Adding or removing a command family changes `HELP`'s output with no
      edit to the `HELP` handler itself.
- [x] Reply shape stays `OK help <space-separated verbs> [#id]`.
- [x] HITL bench: `HELP` over the real link returns the full, accurate
      verb set. Verified 2026-07-07 over serial against the confirmed
      robot device (`mbdeploy list` ROLE=NEZHA2, UID
      `9906360200052820a8fdb5e413abb276...`, `/dev/cu.usbmodem2121102`),
      firmware built via `just build-clean` (v0.20260707.1) and flashed via
      `mbdeploy deploy <UID> --hex MICROBIT.hex`:
      `OK help PING VER HELP ECHO ID DEV M DEV DT DEV STATE DEV STOP DEV WD
      STREAM SNAP S T D R TURN RT G STOP GET SET SI ZERO OI OZ OR OP OV OL
      OA` — byte-identical to the sim harness's output. `PING`/`VER`/`ID`
      spot-checked on the same link and unaffected. (The full on-stand
      motion/config exercise remains ticket 009's scope, per its own
      dependency on this ticket.)

## Testing

- **Existing tests to run**: `tests/sim/unit/test_protocol_roundtrips.py`
  (or wherever the liveness family is currently tested) plus the full
  suite.
- **New tests to write**: a sim test asserting `HELP`'s reply contains
  representative verbs from every registered family.
- **Verification command**: `uv run python -m pytest`.
