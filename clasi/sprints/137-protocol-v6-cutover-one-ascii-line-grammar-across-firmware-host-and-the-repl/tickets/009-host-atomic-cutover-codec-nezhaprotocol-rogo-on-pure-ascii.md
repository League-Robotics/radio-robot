---
id: '009'
title: 'Host atomic cutover: codec, NezhaProtocol, rogo on pure ASCII'
status: open
use-cases: [SUC-001, SUC-002, SUC-003, SUC-005]
depends-on: ['008']
github-issue: ''
issue:
- protocol-v6-spec.md
- protocol-and-implementation-simplification-review.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Host atomic cutover: codec, NezhaProtocol, rogo on pure ASCII

## Description

The host half of the atomic cutover (spec §13 stage 4): rewrite
`io/wire_codec.py` to the pure ASCII codec (spec §11.1's ~150-line reference
shape — encode `verb + ':'.join(fields) + '\n'`, decode by `split(':')`),
update `robot/protocol.py`'s `NezhaProtocol` to build ASCII lines for every
verb (not just `GET`/`SET`/`TLM`/acks from earlier tickets — now motion
verbs too), and update `rogo` (`io/cli.py`, `io/server.py`, `io/client.py`)
to call the new ASCII-only `NezhaProtocol`. **Must land back-to-back with
ticket 008** — the host cannot talk to pre-008 firmware after this lands,
and pre-009 host cannot talk to post-008 firmware.

## Acceptance Criteria

- [ ] `io/wire_codec.py` encodes/decodes pure ASCII lines only — no COBS, no
      CRC, no protobuf import remaining in the encode/decode path (the
      protobuf runtime dependency itself is removed in ticket 013, but this
      ticket's new code path does not use it).
- [ ] `NezhaProtocol` builds ASCII lines for all 30 verbs (spec §3)
      including motion (`move`, `wheels`, `goto`, `stop`, `estop`, `seed`,
      `cal`) and session (`hello`, `ping`, `id`, `ver`, `status`, `help`)
      verbs.
- [ ] `TLMFrame` construction comes from the ticket 005 `thdr`/`t` parse
      path, not the binary decode.
- [ ] `rogo serve`/`rogo repl` and TestGUI's `binary_bridge.py` continue to
      work calling `NezhaProtocol` methods by name (no interface change
      expected per `sprint.md`'s Impact section) — verified by running the
      existing bench-script smoke sequence against the sim.
- [ ] Host role/banner detection (`serial_conn.py`) matches the verb
      case-insensitively for this release (spec §4.1's migration note) so a
      stale v5 banner and the new `device` banner (space-separated, spec
      §4.1) are both recognized.
- [ ] `mbdeploy`'s ROLE column (keys on the `NEZHA2` field, not the verb) is
      confirmed unaffected.

## Testing

- **Existing tests to run**: host `robot_radio` protocol/codec suites,
  `rogo` CLI smoke tests, sim harness end-to-end tests.
- **New tests to write**: an ASCII round-trip test per verb category
  against the sim; a banner case-insensitivity test.
- **Verification command**: `uv run pytest src/tests -k "protocol or
  wire_codec or rogo"`, plus a sim end-to-end smoke run.
