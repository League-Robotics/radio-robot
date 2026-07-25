---
id: '005'
title: 'Framing grammar cutover: uniform command/reply lines, PONG/ID/VER, deletion
  of the old text/binary heuristics'
status: open
use-cases: [SUC-001]
depends-on: ['001', '003']
github-issue: ''
issue: protocol-v5-one-line-packets-command-prefix-and-newline-cobs.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Framing grammar cutover: uniform command/reply lines, PONG/ID/VER, deletion of the old text/binary heuristics

## Description

The centerpiece framing change. Implement the uniform grammar
`<COMMAND>[':' <data>]'\n'` in both directions, both transports:
`App::Comms` (command prefix parse/emit, dispatch by registry lookup
from ticket 001, using ticket 003's delimiter/CRC-scope codec),
`Com::SerialPort`/`Com::Radio` (unconditional `\n` split, no
text/binary heuristic at the transport layer, `send()`/`sendText()`
terminator convergence), and the host mirrors (`wire_codec.py`,
`serial_conn.py`).

New reply-plane handlers: `PONG:t=<ms>` (replacing `OK pong t=<ms>`),
`ID:<fields>` (configured-robot identity — drivetrain type + calibration
profile name/version, per sprint 124 architecture Decision 4 — exact
field names finalized in this ticket), `VER:<version>` (reads the
existing generated build-version constant, `src/firm/types/
version_generated.h` — no new version-tracking infrastructure). Confirm
the existing `DEVICE:` banner (already restored in 123-006, not
re-planned here) parses unmodified under the new grammar and that both
its emission sites end up `\n`-terminated once `sendReliable()` drops
`\r\n`.

Delete, don't deprecate: `kTextCommands[]`/`isRecognizedTextCommand()`
(`serial_port.cpp`), `_looks_like_text()`/`_TEXT_SAFE_BYTES`
(`wire_codec.py`), the dead host text-branch routing (`TLM`/`EVT`/
`OK|ERR|CFG|ID` in `serial_conn.py`'s `_handle_text_line()`), and
`App::FrameKind` as a transport-level concept (the transport delivers a
line; `Comms` decides text-vs-binary from the parsed command via the
registry).

## Acceptance Criteria

- [ ] The 123-006 hardware repro (`move_wheels` embedding a literal
      `0x0A`) executes 10/10 over direct USB (radio-relay confirmation
      is ticket 013's job).
- [ ] `kTextCommands[]`/`isRecognizedTextCommand()` and
      `_looks_like_text()`/`_TEXT_SAFE_BYTES` are deleted, not just
      unused (grep-enforceable).
- [ ] The dead host text branches (`TLM`/`EVT`/`OK|ERR|CFG|ID` routing,
      `_CORR_ID_RE`) are gone, and no test depends on them.
- [ ] `HELLO`→`DEVICE:`, `PING`→`PONG:`, `ID`→`ID:`, `VER`→`VER:` all
      answer on both transports (serial confirmed here; relay confirmed
      in ticket 013), and every emitted line parses under the grammar.
- [ ] `DEVICE:NEZHA2:robot:<name>:<serial>` parses unmodified as command
      `DEVICE` + cleartext data on the first `:` — no format change,
      byte-freeze holds.
- [ ] Both banner emission sites (`main.cpp` boot, `robot_loop.cpp`
      `boot()`) are `\n`-terminated, not `\r\n`.
- [ ] Host reads the wire via plain `readline()` in at least one test
      path — the demonstration that the demuxer is no longer load-bearing.
- [ ] Grammar edge cases pass: data containing `:`; data containing
      `0x00`; a no-data verb with a stray trailing `:`; an unknown
      command (counted malformed, not crashed); a truncated line.

## Testing

- **Existing tests to run**: `test_serial_conn_binary_plane.py`,
  `test_protocol_binary_client.py`, `app_comms_harness.cpp`.
- **New tests to write**: the grammar edge-case tests above; a
  host/firmware round-trip test interleaving text-plane and binary-plane
  lines on the same connection.
- **Verification command**: `uv run pytest` plus the C++ sim-tests build;
  a serial bench smoke test per `.claude/rules/hardware-bench-testing.md`.
