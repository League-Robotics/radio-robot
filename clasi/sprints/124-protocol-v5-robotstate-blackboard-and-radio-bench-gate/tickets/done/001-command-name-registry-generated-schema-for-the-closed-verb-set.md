---
id: '001'
title: 'Command-name registry: generated schema for the closed verb set'
status: done
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: protocol-v5-one-line-packets-command-prefix-and-newline-cobs.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Command-name registry: generated schema for the closed verb set

## Description

Foundational ticket — everything downstream dispatches/documents against
this. Today the closed verb set (and which verbs carry binary vs.
cleartext data) lives independently in firmware dispatch, the host
codec, and the published protocol doc — three places that can drift
(sprint 124 architecture Decision 2).

Add a new generated schema (e.g. `src/protos/commands.proto`) processed
by `gen_messages.py`, declaring every v5 verb (`HELLO`, `PING`, `ID`,
`VER` on the command side; `DEVICE`, `PONG`, `ID`, `VER` on the reply
side; plus the existing binary command/reply verbs) with a per-verb
binary/cleartext flag. `gen_messages.py` emits a firmware-side dispatch
table and a host-consumable constant set from this one source, the same
pattern it already uses for message schemas.

## Acceptance Criteria

- [x] New schema declares every v5 verb with a binary/cleartext flag,
      matching the grammar in sprint 124's architecture (Step 1 of
      `protocol-v5-one-line-packets-command-prefix-and-newline-cobs.md`).
- [x] `gen_messages.py` emits a firmware-side dispatch table (or constant
      set `App::Comms` can dispatch against) and a host-side constant
      module (`wire_codec.py`/`serial_conn.py` can import), both from
      this one schema.
- [x] Adding a future verb requires editing exactly one file (the new
      schema) — no parallel edit needed in firmware, host, or docs to
      keep the verb set consistent.
- [ ] No hand-maintained verb list remains anywhere that duplicates this
      registry (grep-enforceable once ticket 005 lands). **Deliberately
      NOT satisfied by this ticket** — see Completion Notes below.

## Completion Notes

Built exactly what the description scopes, no more:

- `src/protos/commands.proto` (new) — the `Verb` enum, 12 real verbs
  (`HELLO`/`PING`/`ID`/`VER`/`DEVICE`/`PONG` cleartext,
  `MOVE`/`CONFIG`/`STOP`/`TLM`/`OK`/`ERR` binary) plus the required
  proto3 `VERB_UNSPECIFIED = 0` sentinel (excluded from every generated
  row). Verb inventory taken from the issue's §4 reply-plane table and
  the current `CommandEnvelope`/`ReplyEnvelope` oneof arms
  (`envelope.proto`), not guessed.
- `src/protos/options.proto` — new `extend google.protobuf.EnumValueOptions`
  block, `(binary)` bool at extension number 50100 (a separate number
  space from the existing `FieldOptions` block at 50000-50006 — different
  extended message types don't share a space, but the gap keeps that
  visible at a glance).
- `src/scripts/gen_messages.py` — extended to emit, from the one schema:
  `src/firm/messages/commands.h` (`enum class Verb` + generated
  `VerbEntry`/`kVerbTable[]`/`kVerbCount`) and
  `src/host/robot_radio/io/wire_commands.py` (`VERBS`/`VERB_BY_NAME`/
  `BINARY_VERBS`/`CLEARTEXT_VERBS`), both derived from the identical
  `_verb_rows()` walk over the same enum descriptor so the two outputs
  cannot independently drift. `_run_codegen_pipeline()` now returns a
  `host_outputs` dict (files outside `src/firm/messages/`) alongside the
  existing firmware `outputs`; `main()` writes both.
- Registry scope is deliberately narrow, per sprint.md Step 3's module
  table: name + binary/cleartext flag only — no direction (command vs.
  reply), no handler, no data shape. `ID`/`VER` are each one row even
  though they answer as both a command and a reply verb.
- `comms.cpp`'s parsing behavior is untouched, as directed — the registry
  exists but nothing dispatches against it yet (ticket 005's job).

**AC4 is explicitly NOT satisfied and should not be read as an oversight.**
Verified by grep: `kTextCommands[]`/`isRecognizedTextCommand()`
(`src/firm/com/serial_port.cpp`) and `_looks_like_text()`/
`_TEXT_SAFE_BYTES` (`src/host/robot_radio/io/wire_codec.py`) are still
present, completely unmodified — they are the two heuristic "verb lists"
sprint 124's architecture calls out for deletion, and deleting them is
ticket 005's own acceptance criterion (framing grammar cutover), not
this one's. This ticket adds zero NEW duplication (nothing here
re-lists verb names by hand anywhere outside `commands.proto` itself),
but the pre-existing duplication is untouched and will stay untouched
until 005 lands and its own grep-enforceable check can pass.

**Testing performed:**
- `src/tests/unit/test_command_registry.py` (new) — differential test:
  parses `commands.h`'s generated `kVerbTable[]` and `exec()`s the
  generated `wire_commands.py`, asserts the two agree byte-for-byte on
  verb name + binary flag (including row count via `kVerbCount`/
  `len(VERBS)`); a second test pins the actual verb inventory against
  the issue's §4 table independent of the generator's own self-
  consistency.
- Full suite: `uv run python -m pytest` — **1436 passed, 2 skipped,
  9 xfailed, 2 xpassed** (skips/xfails/xpasses pre-exist this change,
  unrelated to it).
- `just build-sim` (host C++ build) — clean.
- `just build` (full ARM firmware build via `mbdeploy`'s toolchain) —
  clean; `commands.h` is generated and available but not yet `#include`d
  anywhere (nothing dispatches against it this ticket), so it did not
  need to compile as part of any translation unit — confirmed the
  generator's other outputs (all touched by the `options.proto` change)
  still build the firmware and host-sim library with no new warnings
  beyond pre-existing vendor-SDK ones.
- `gen_pb2.py` (host compiled protobuf bindings) — regenerated
  `commands_pb2.py`; manually verified via the Python descriptor API
  that the `(binary)` extension round-trips correctly for all 12 verbs.

**Decisions made that the ticket left to implementation:**
- The registry excludes direction/handler/data-shape metadata entirely
  (see Description above) — this follows sprint.md Step 3's module
  table literally rather than adding fields "just in case."
- `VERB_UNSPECIFIED = 0` as the required proto3 sentinel, name chosen to
  read unambiguously as "not a real verb" in both generated outputs.
- Host output path: `src/host/robot_radio/io/wire_commands.py` (sits
  next to `wire_codec.py`, the module ticket 005 will import it from).

## Testing

- **Existing tests to run**: existing `gen_messages.py` golden-output
  tests (`src/tests/unit/test_gen_messages*.py` or equivalent).
- **New tests to write**: a test asserting the generated firmware table
  and the generated host constants agree byte-for-byte on verb names and
  binary/cleartext flags (a differential test, matching the golden-vector
  pattern ticket 004 will formalize for the codec itself).
- **Verification command**: `uv run pytest`
