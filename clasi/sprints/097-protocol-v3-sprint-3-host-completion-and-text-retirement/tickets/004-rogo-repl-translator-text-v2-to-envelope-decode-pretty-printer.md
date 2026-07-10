---
id: '004'
title: 'rogo REPL translator: text-v2-to-envelope + --decode pretty-printer'
status: in-progress
use-cases:
- SUC-004
depends-on:
- '002'
- '003'
github-issue: ''
issue: protocol-v3-schema-driven-binary-command-plane-protobuf.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# rogo REPL translator: text-v2-to-envelope + --decode pretty-printer

## Description

Make `rogo send <text>` a text-v2-to-envelope translator instead of a raw
pass-through to `NezhaProtocol.send()`, so a human at a terminal keeps
typing familiar v2 syntax while the wire carries binary. Reuses ticket
002's Legacy Verb Translator (M4) for the verb-to-envelope mapping — this
ticket does not reimplement that logic.

For a verb with a proven binary replacement (S/D/T/RT/MOVE/MOVER/ECHO/
PING/ID/SET/GET/STREAM/SNAP), `rogo send` builds and sends the matching
envelope via M4. For a retained rump verb (PING, ID, HELLO, HELP, STOP),
it sends the verb as plain text, unchanged — the rump exists precisely so
a bare-terminal human needs no translation at all for these five.

Add a `--decode` flag that pretty-prints a received `*B<base64>` reply's
decoded fields instead of the raw armored line.

`rogo binary <arm>` (095/096) is unaffected — it remains available for
direct, low-level envelope construction; this ticket adds a second, more
convenient path on top of it, it does not replace it.

## Acceptance Criteria

- [ ] `rogo send S 200 200` produces the same on-wire effect as
      `rogo binary drive --left 200 --right 200` (binary), and a
      human-readable reply.
- [ ] `rogo send D 200 200 300` produces the equivalent binary `segment`
      envelope (via M4's translation) and a human-readable reply.
- [ ] `rogo send STOP` sends plain text `STOP` (the rump verb, unchanged),
      not an envelope.
- [ ] `rogo send PING`, `rogo send ID`, `rogo send HELLO`, `rogo send
      HELP` each send plain text (the rump), unchanged.
- [ ] `rogo send --decode ...` prints decoded reply fields (not a raw
      `*B...` line) for a binary-plane command.
- [ ] `rogo binary <arm>` subcommands are byte-for-byte unaffected by this
      ticket's diff.
- [ ] `tests/sim` stays green (host-only ticket; sanity check).
- [ ] `tests/unit` is green, including new tests for the translator
      dispatch (rump vs. binary) and `--decode`.

## Implementation Plan

### Approach

1. Extend `cmd_send`/`_print_binary_reply` in `host/robot_radio/io/cli.py`:
   parse the typed verb, look it up against a small table mapping verb ->
   {rump (send as text) | M4 translation function}.
2. For a binary-mapped verb, tokenize the remaining text-v2 arguments the
   same way `CommandProcessor::parseTokens`/`parseKV` do firmware-side
   (positional tokens + `key=value` pairs), feed them to the matching M4
   translator function, build the envelope, and send via
   `SerialConnection.send_envelope()`.
3. Add the `--decode` argparse flag; when set, format the received
   `ReplyEnvelope`'s populated oneof body field-by-field instead of
   printing `str(reply)`'s raw protobuf repr.
4. Leave every `rogo binary <arm>` subcommand's own code path untouched.

### Files to modify

- `host/robot_radio/io/cli.py` — `cmd_send`, `_print_binary_reply`, new
  `--decode` flag, new verb-dispatch table.

### Testing plan

- New CLI-level tests (or unit tests against the underlying translation/
  dispatch function, if `cli.py`'s existing test coverage is structured
  that way) covering: a binary-mapped verb producing the correct
  envelope, a rump verb sending plain text unchanged, and `--decode`'s
  output format.
- Manual smoke exercise against the sim harness (or bench, if available)
  for at least one binary-mapped verb and one rump verb, per SUC-004's
  own acceptance criteria.
- Run `tests/unit` (host suite) and `tests/sim` (sanity — unaffected).

### Documentation updates

- `rogo send --help`'s own usage text (list the rump vs. translated
  verbs). `docs/protocol-v3.md` (ticket 009) references `rogo send` as
  the recommended human entry point.
