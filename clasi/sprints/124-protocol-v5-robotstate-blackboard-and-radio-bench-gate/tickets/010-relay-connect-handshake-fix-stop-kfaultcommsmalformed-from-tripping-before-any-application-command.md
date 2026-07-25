---
id: '010'
title: 'Relay connect-handshake fix: stop kFaultCommsMalformed from tripping before
  any application command'
status: open
use-cases: [SUC-008]
depends-on: []
github-issue: ''
issue: relay-handshake-trips-comms-malformed.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Relay connect-handshake fix: stop kFaultCommsMalformed from tripping before any application command

## Description

Independent, largely self-contained fix — needed clean before ticket
013's bench gate can pass. `kFaultCommsMalformed` trips on a fresh,
clean `!GO` relay connect with ZERO application commands sent, only over
the relay, never over direct USB (confirmed reproducible per the linked
issue's isolated test). Root-cause and fix it.

Per sprint 124 architecture Decision 5, the framing grammar itself
(tickets 003/005) needs **no** relay firmware change — the relay's `!GO`
data-plane mode is a transparent RAW250 byte pass-through, content-
agnostic. This ticket's root cause is therefore something else: most
likely leaked `!ECHO OFF`/`!MODE RAW250`/`!GO` control-plane bytes
momentarily reaching the robot's parser before the relay fully commits
to transparent pass-through, or a partial/fragment line at the exact
moment of the RAW250 mode transition. Investigate via a targeted
pyOCD/gdb session or byte-level capture on the relay-robot leg (per
`.claude/rules/debugging.md`) if needed; the issue's own "Direction"
section is the starting point.

## Acceptance Criteria

- [ ] Reproduces the fix against the exact isolated-test sequence the
      issue itself records: fresh clean-boot firmware, relay-only
      connect, zero application commands sent, ~1 s settle, inspect
      `fault_bits` — confirm bit 3 (`kFaultCommsMalformed`) stays clear.
- [ ] `kFaultCommsMalformed` stays clear through a fresh relay connect
      with zero application commands, across multiple trials (matching
      the "reproduced across multiple fresh-boot trials" repro rigor
      the issue itself used).
- [ ] Root cause (leaked control-plane bytes vs. a host-side
      connect-timing race) is stated explicitly in this ticket's closing
      notes, whichever it turns out to be.
- [ ] If the fix requires a relay dongle firmware change (contradicting
      Decision 5's expectation that it wouldn't), that is flagged
      explicitly here and in the sprint's Migration Concerns — not
      silently absorbed.

## Testing

- **Existing tests to run**: existing relay-connect tests/bench scripts
  (`src/tests/bench/relay_telemetry_rate.py`, `dev_exercise.py`).
- **New tests to write**: the isolated repro sequence above, promoted
  into an automated bench/integration test that can catch a regression.
- **Verification command**: bench run per
  `.claude/rules/hardware-bench-testing.md` (relay path specifically).
