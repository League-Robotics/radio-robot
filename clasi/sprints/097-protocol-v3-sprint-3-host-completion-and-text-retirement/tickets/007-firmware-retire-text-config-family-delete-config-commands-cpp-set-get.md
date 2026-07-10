---
id: '007'
title: 'Firmware: retire text config family (delete config_commands.cpp SET/GET)'
status: open
use-cases: [SUC-007]
depends-on: ['005']
github-issue: ''
issue: protocol-v3-schema-driven-binary-command-plane-protobuf.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Firmware: retire text config family (delete config_commands.cpp SET/GET)

## Description

Delete `source/commands/config_commands.{h,cpp}` in full — the text
`SET`/`GET` handlers, both `strcmp` chains (`applyConfigKey`,
`formatConfigKeyFromBb`), the `CFG` snprintf reply emitter, and
`formatFixed`/`parseFloatStrict`/`parseLongStrict`. `SET`/`GET` are
already unregistered (096 Decision 1 — config's binary arm is the only
live path since 096), so this ticket changes zero wire-observable
behavior; it removes dead-on-the-wire source code once ticket 005 has
confirmed the host's `get_config()`/`set_config()` (ticket 002) work
correctly over the binary `config`/`get` arms.

**Binary parity: 096, sim-exhaustive** (differential-vs-google.protobuf
byte-parity + fuzz + behavioral tests exercising `BinaryChannel`'s
`config`/`get` arms end-to-end against `Rt::ConfigDelta`/the
Configurator). Hardware bench for config (change a PID gain over binary,
observe wheel behavior change on the stand) is part of the team-lead's
post-sprint consolidated session, per `sprint.md`'s own sequencing — this
ticket's own gate is sim + ARM-build-clean, not a substitute for that
session.

`dev_commands.cpp`'s own, separate, lower-level `DEV *CFG` strcmp chains
are explicitly OUT of scope — 096 Decision 3 already drew this boundary
("a different, lower-level debug surface this sprint does not touch");
no binary `dev` arm exists or is planned. Do not touch `dev_commands.
{h,cpp}`.

## Acceptance Criteria

- [ ] `source/commands/config_commands.h` and `.cpp` no longer exist.
- [ ] `grep -rn "config_commands.h"` (or any reference to
      `configCommands()`) returns no hits anywhere in `source/`.
- [ ] `dev_commands.{h,cpp}` is byte-for-byte untouched by this ticket's
      diff.
- [ ] `command_router.cpp`'s `buildTable()` comment referencing
      `configCommands()` (096's own note that it is "deliberately NOT
      re-added") is updated or removed as appropriate now that the
      function no longer exists at all.
- [ ] Any `tests/sim/unit/*` test currently exercising text SET/GET is
      re-pointed at the binary `config`/`get` arms — coverage maintained.
- [ ] `tests/sim` is green.
- [ ] `just build` (ARM) succeeds; the flash delta (`.map` before/after)
      is measured and recorded in this ticket's completion notes.
- [ ] Completion notes explicitly state: this ticket's own gate is sim +
      ARM-build-clean; the consolidated HITL bench (team-lead, post-
      sprint) is the final real-hardware gate, including the "change a
      PID gain over binary, observe wheel behavior on the stand" bench
      criterion from the issue's own Sprint 2 bench gate.

## Implementation Plan

### Approach

1. Delete `source/commands/config_commands.h` and `.cpp`.
2. Grep the tree for any remaining `#include "commands/config_commands.h"`
   or `configCommands(` reference (there should be none outside
   `command_router.cpp`'s own comment, per 096's design) and remove/update.
3. Update any `tests/sim/unit/*` test exercising text SET/GET to drive the
   binary `config`/`get` arms instead.
4. Build (`just build`), capture the `.map` flash delta.

### Files to modify

- `source/commands/config_commands.h` (deleted)
- `source/commands/config_commands.cpp` (deleted)
- `source/runtime/command_router.cpp` (comment cleanup only, if needed)
- Affected `tests/sim/unit/*` test files

### Testing plan

- `tests/sim` full run — must be green.
- `just build` — ARM build must succeed; record `.map` flash delta.
- Grep-clean checks listed in Acceptance Criteria.

### Documentation updates

- None in this ticket (ticket 009 owns `docs/protocol-v3.md`).
