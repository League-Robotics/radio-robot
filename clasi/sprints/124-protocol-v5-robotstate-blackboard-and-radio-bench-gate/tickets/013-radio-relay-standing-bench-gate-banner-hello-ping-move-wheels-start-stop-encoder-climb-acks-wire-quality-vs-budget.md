---
id: '013'
title: 'Radio-relay standing bench gate: banner, HELLO/PING, move_wheels start/stop,
  encoder climb, acks, wire-quality vs budget'
status: open
use-cases: [SUC-007, SUC-008]
depends-on: ['005', '006', '008', '009', '010', '011', '012']
github-issue: ''
issue:
- protocol-v5-one-line-packets-command-prefix-and-newline-cobs.md
- relay-handshake-trips-comms-malformed.md
- telemetry-physical-layer-corruption-and-move-ack-observability.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Radio-relay standing bench gate: banner, HELLO/PING, move_wheels start/stop, encoder climb, acks, wire-quality vs budget

## Description

**This ticket is the sprint's acceptance gate.** Run the full standing
bench gate over the **`!GO` radio-relay data plane — NOT USB** (per
stakeholder directive; see `.claude/rules/hardware-bench-testing.md` and
`.clasi/knowledge/` for the relay `!GO` data-plane protocol: host opens
the relay with DTR asserted, sends `!GO` to enter the data plane, then
plain commands with no `>` prefix). A sprint is not done on tests alone
— this must be SEEN working on the stand.

The acceptance criteria below are the stakeholder's list, verbatim, all
of them over the relay:

## Acceptance Criteria

- [ ] Banner on connect: `DEVICE:` observed at boot on a fresh relay
      connect with no `HELLO` sent, and `connect()` completes without
      entering the `_HELLO_CLASSIFY_TIMEOUT_S` fallback.
- [ ] `HELLO`/`PING` (and `ID`/`VER`) all answer over the relay.
- [ ] `move_wheels` starts the wheels; `stop` stops them — over the
      relay.
- [ ] Encoder positions climb in telemetry while the move runs, over
      the relay.
- [ ] Enqueue ack AND completion ack are both observed, over the relay
      (via the packed `acks` ring from ticket 008; confirmed generally
      fixed by ticket 011).
- [ ] A `wire_truth`-equivalent quality measurement (ticket 012's
      promoted script) runs through the relay, checked against ticket
      012's stated loss budget for the relay path.
- [ ] `kFaultCommsMalformed` stays clear throughout (confirming ticket
      010's fix holds under the full gate, not just its isolated repro).
- [ ] All of the above pass over the radio relay specifically. USB may
      additionally be run for comparison but does NOT satisfy this
      gate on its own.
- [ ] The 123-006 hardware repro (`move_wheels` embedding a literal
      `0x0A`) executes 10/10 over the relay (ticket 005 confirmed this
      over USB; this ticket confirms it over the actual acceptance
      transport).

## Testing

- **Existing tests to run**: N/A — this ticket IS the test. All prior
  unit/sim/property tests from tickets 001-012 must already be green
  before this bench session is attempted.
- **New tests to write**: none beyond the bench session itself; results
  are recorded as this ticket's own closing evidence (PASS/FAIL per
  criterion, per `.claude/rules/hardware-bench-testing.md`'s stated
  format).
- **Verification command**: a single bench session on the stand,
  executing the full sequence above over the radio relay, with results
  recorded in this ticket's closing notes.
