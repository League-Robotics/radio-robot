---
id: '007'
title: 'Firmware: gut the text config family'
status: open
use-cases: [SUC-007]
depends-on: []
github-issue: ''
issue: protocol-v3-schema-driven-binary-command-plane-protobuf.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Firmware: gut the text config family

## Description

**REWRITTEN — see `architecture-update-r2.md` Decision 9 (supersedes r1's
Decision 8 and this ticket's own r1-conservative no-op scope in full).**
Eric's 2026-07-10 redirect: gut the firmware text plane unconditionally.
`config_commands.{h,cpp}` is deleted in full this sprint — **DELETE**:
the text `SET`/`GET` handlers, both `strcmp` chains (`applyConfigKey`,
`formatConfigKeyFromBb`), the `CFG` snprintf reply emitter, and
`formatFixed`/`parseFloatStrict`/`parseLongStrict`. `SET`/`GET` are
already unregistered at the wire (096 Decision 1) — this ticket removes
the now-fully-dead source, not just the registration.

**Binary parity: 096, sim-exhaustive** (differential-vs-google.protobuf
byte-parity + fuzz + behavioral tests exercising `BinaryChannel`'s
`config`/`get` arms end-to-end against `Rt::ConfigDelta`/the
Configurator). This ticket no longer depends on ticket 005's verification
(`depends-on: []`) — deletion is unconditional under Decision 9, not
gated on host consumer migration state.

**Every live text `SET`/`GET` sender r1 found — `robot_mcp.py`'s
`push_calibration`, `cli.py`'s `_push_calibration` (`rogo sync-cal`),
`calibration/push.py`, `calibrate_verify.py`, TestGUI's own test suite —
BREAKS against this firmware once this ticket lands, until rewired to the
`rogo` translator proxy (ticket 004).** This is an accepted,
stakeholder-approved consequence of Decision 9 ("worry about the
consumer later"), not a regression to fix here. State it plainly in
completion notes.

`dev_commands.cpp`'s own, separate, lower-level `DEV *CFG` strcmp chains
remain explicitly OUT of scope — 096 Decision 3's boundary, unchanged; no
binary `dev` arm exists or is planned. Do not touch `dev_commands.
{h,cpp}`.

## Acceptance Criteria

- [ ] `source/commands/config_commands.h` and `.cpp` no longer exist.
- [ ] `grep -rn "config_commands.h"` (or any reference to
      `configCommands()`) returns no hits anywhere in `source/`.
- [ ] `dev_commands.{h,cpp}` is byte-for-byte untouched by this ticket's
      diff.
- [ ] `command_router.cpp`'s `buildTable()` comment referencing
      `configCommands()` (096's own note that it is "deliberately NOT
      re-added") is updated or removed now that the function no longer
      exists at all.
- [ ] Any `tests/sim/unit/*` test currently exercising text SET/GET is
      re-pointed at the binary `config`/`get` arms — coverage maintained.
- [ ] `tests/sim` is green.
- [ ] `just build` (ARM) succeeds; the flash delta (`.map` before/after)
      is measured and recorded — expected to be a real, meaningful
      reduction (both `strcmp` chains + the `CFG` emitter).
- [ ] Completion notes state plainly which live text `SET`/`GET` senders
      (listed in Description) now break against this firmware, and that
      rewiring them to the `rogo` proxy (ticket 004) is deferred to
      `realign-host-tooling-to-gutted-four-verb-wire-surface.md`.

## Implementation Plan

### Approach

1. Delete `source/commands/config_commands.h` and `.cpp`.
2. Grep the tree for any remaining `#include "commands/config_commands.h"`
   or `configCommands(` reference and remove/update (expected: only
   `command_router.cpp`'s own comment, per 096's design).
3. Update any `tests/sim/unit/*` test exercising text SET/GET to drive the
   binary `config`/`get` arms instead.
4. Build (`just build`), capture the `.map` flash delta.

### Files to modify

- `source/commands/config_commands.h` (deleted)
- `source/commands/config_commands.cpp` (deleted)
- `source/runtime/command_router.cpp` (comment cleanup)
- Affected `tests/sim/unit/*` test files

### Testing plan

- `tests/sim` full run — must be green.
- `just build` — ARM build must succeed; record `.map` flash delta.
- Grep-clean checks listed in Acceptance Criteria.

### Documentation updates

- None in this ticket (ticket 009 owns `docs/protocol-v3.md`; it must now
  describe `SET`/`GET` as binary-only, with the `rogo` proxy as the
  text-compatibility path).
