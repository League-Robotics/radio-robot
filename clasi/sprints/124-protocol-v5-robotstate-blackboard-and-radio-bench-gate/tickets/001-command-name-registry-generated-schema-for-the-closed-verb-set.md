---
id: '001'
title: 'Command-name registry: generated schema for the closed verb set'
status: open
use-cases: [SUC-001]
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

- [ ] New schema declares every v5 verb with a binary/cleartext flag,
      matching the grammar in sprint 124's architecture (Step 1 of
      `protocol-v5-one-line-packets-command-prefix-and-newline-cobs.md`).
- [ ] `gen_messages.py` emits a firmware-side dispatch table (or constant
      set `App::Comms` can dispatch against) and a host-side constant
      module (`wire_codec.py`/`serial_conn.py` can import), both from
      this one schema.
- [ ] Adding a future verb requires editing exactly one file (the new
      schema) — no parallel edit needed in firmware, host, or docs to
      keep the verb set consistent.
- [ ] No hand-maintained verb list remains anywhere that duplicates this
      registry (grep-enforceable once ticket 005 lands).

## Testing

- **Existing tests to run**: existing `gen_messages.py` golden-output
  tests (`src/tests/unit/test_gen_messages*.py` or equivalent).
- **New tests to write**: a test asserting the generated firmware table
  and the generated host constants agree byte-for-byte on verb names and
  binary/cleartext flags (a differential test, matching the golden-vector
  pattern ticket 004 will formalize for the codec itself).
- **Verification command**: `uv run pytest`
