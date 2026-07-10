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

**REVISED SCOPE — see `architecture-update-r1.md` Decision 8.** The
original plan (delete `config_commands.{h,cpp}` in full, since `SET`/`GET`
are unregistered at the wire) assumed "unregistered at the wire" meant
"no live consumer." That is false: `SET`/`GET` text is sent directly by
several live, production code paths that bypass `NezhaProtocol` entirely
and reach the firmware over whatever text verbs it still accepts on a
CONNECTED serial line — the fact that `config_commands.cpp`'s handlers are
unregistered in `buildTable()` today would mean these senders are ALREADY
getting `ERR unknown`, which is itself a live-consumer-breaking regression
this ticket must not compound further by deleting the source entirely
before those senders migrate:

- **`host/robot_radio/io/robot_mcp.py`**: `push_calibration(_robot._proto,
  _config)` — `NezhaProtocol` has no `push_calibration` method, so
  `calibration/push.py`'s own documented fallback always takes the
  "extract `_conn`, send raw text `SET`" path. **This is the MCP server**,
  one of the sprint's own explicitly-protected consumers.
- **`host/robot_radio/io/cli.py`**: `_push_calibration()` (called from
  `rogo sync-cal`) sends raw text `SET ml=`/`SET mr=`/`SET tw=`/
  `SET odomOff*=`.
- **`host/robot_radio/calibration/push.py`**: same `SET` sequence, a
  second, parallel implementation.
- **`host/calibrate_verify.py`**: raw text `GET tw ml mr`.
- **TestGUI's own test suite** (`tests/testgui/
  test_calibration_push_on_connect.py`) exercises `GET`/`SET` via
  `transport.command()`.

Per the issue's own rule ("deleted only after its binary replacement is
bench-proven AND its consumers migrated") and the sprint's own "MCP server
change zero call sites" success criterion, **`config_commands.{h,cpp}` is
NOT deleted this sprint.** `SET`/`GET` stay exactly as they are today:
source present, unregistered at the wire (096 Decision 1's status quo,
unchanged) — this ticket does not re-register them either, it simply does
not delete their source. Migrating the callers above to
`NezhaProtocol.set_config()`/`.get_config()` (binary, done in ticket 002)
is `realign-host-tooling-to-gutted-four-verb-wire-surface.md`'s own scope
(now updated to explicitly own it), not this ticket's.

The original binary-parity evidence remains true and is preserved for
whenever `realign-host-tooling` clears the way: **Binary parity: 096,
sim-exhaustive** (differential-vs-google.protobuf byte-parity + fuzz +
behavioral tests exercising `BinaryChannel`'s `config`/`get` arms
end-to-end against `Rt::ConfigDelta`/the Configurator).

`dev_commands.cpp`'s own, separate, lower-level `DEV *CFG` strcmp chains
remain explicitly OUT of scope — 096 Decision 3's boundary, unchanged; no
binary `dev` arm exists or is planned. Do not touch `dev_commands.
{h,cpp}`.

## Acceptance Criteria

- [ ] Before any deletion, re-verify (fresh grep, not a stale citation of
      this ticket's own Description) whether `robot_mcp.py`'s
      `push_calibration` call, `cli.py`'s `_push_calibration`,
      `calibration/push.py`, and `calibrate_verify.py` still send raw text
      `SET`/`GET`. If — and only if — this fresh check finds ALL of them
      have migrated to `NezhaProtocol.set_config()`/`.get_config()` (binary),
      `config_commands.{h,cpp}` may be deleted following the original plan
      below. Otherwise, make no deletion.
- [ ] `source/commands/config_commands.h`/`.cpp` are BYTE-FOR-BYTE
      UNCHANGED, still present, still unregistered — the expected outcome
      this sprint.
- [ ] `dev_commands.{h,cpp}` is byte-for-byte untouched by this ticket's
      diff.
- [ ] `tests/sim` is green (expected: unaffected, no config text deleted).
- [ ] `just build` (ARM) succeeds; the flash delta (`.map` before/after)
      is measured and recorded — expected to be **zero** this sprint (no
      source deleted).
- [ ] Completion notes explicitly state: `config_commands.{h,cpp}` is
      preserved this sprint per `architecture-update-r1.md` Decision 8,
      deferred to `realign-host-tooling-to-gutted-four-verb-wire-surface.md`;
      the "change a PID gain over binary, observe wheel behavior on the
      stand" bench criterion from the issue's own Sprint 2 bench gate
      still applies to the EXISTING binary `config`/`get` arms (096) and
      is unaffected by this ticket's own no-op outcome.

## Implementation Plan

### Approach

1. Re-verify the live-consumer list above with a fresh grep.
2. If (and only if) every listed consumer has migrated, delete
   `config_commands.{h,cpp}` following the original plan (delete the
   files; grep for and remove any `#include`/`configCommands(` reference;
   update `command_router.cpp`'s own comment). Otherwise, make no source
   changes.
3. Build (`just build`), capture the `.map` flash delta (expected zero).

### Files to modify

- None expected this sprint (all consumers found live). If the
  re-verification in step 1 finds otherwise:
  `source/commands/config_commands.h` (deleted), `.cpp` (deleted),
  `source/runtime/command_router.cpp` (comment cleanup).

### Testing plan

- `tests/sim` full run — must be green (expected unaffected).
- `just build` — ARM build must succeed; record `.map` flash delta.
- Grep-clean live-consumer re-verification per Acceptance Criteria.

### Documentation updates

- None in this ticket (ticket 009 owns `docs/protocol-v3.md`; it must now
  describe `SET`/`GET` as still source-present-but-unregistered, matching
  096's status quo, not deleted, per this revision).
