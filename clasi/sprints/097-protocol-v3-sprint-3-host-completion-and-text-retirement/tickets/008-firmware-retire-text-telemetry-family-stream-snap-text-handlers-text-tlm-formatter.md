---
id: '008'
title: 'Firmware: retire text telemetry family (STREAM/SNAP text handlers + text TLM
  formatter)'
status: open
use-cases: [SUC-008]
depends-on: ['005']
github-issue: ''
issue: protocol-v3-schema-driven-binary-command-plane-protobuf.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Firmware: retire text telemetry family (STREAM/SNAP text handlers + text TLM formatter)

## Description

**REVISED SCOPE — see `architecture-update-r1.md` Decision 8.** The
original plan (delete `handleStream`/`handleSnap`/`kStreamSchema`/
`Telemetry::buildTlmFrame()` now that ticket 003 converted
`NezhaProtocol.stream()`/`.snap()` to binary) assumed `NezhaProtocol` was
the only host path that arms text telemetry. Ticket 003's own
implementation found two live, production consumers that are NOT
`SerialConnection`-reachable and therefore could not be swept onto the
binary plane this sprint (documented in ticket 003's Resolution section
and `architecture-update-r1.md` Decision 8):

- **`calibration/linear.py`**: raw pyserial (`RelaySerial`/`DirectSerial`),
  sends `"SNAP"` directly and reads the text `TLM ...` reply.
- **`calibration/angular.py`**: same raw-pyserial transport, sends
  `"STREAM 20"`/`"STREAM 0"` and `"SNAP"` directly.

The team-lead's own follow-up sweep additionally found:

- **TestGUI** (`testgui/__main__.py`) sends a hardcoded `"STREAM 50"` on
  EVERY connect, for any transport including real hardware
  (`SerialTransport`/`RelayTransport`) — a connect-critical, first-run
  dependency on the text `STREAM` verb.

Per the issue's own rule and the sprint's own "TestGUI... change zero call
sites" success criterion, **the text `STREAM`/`SNAP` handlers and
`Telemetry::buildTlmFrame()` are NOT deleted this sprint.** They stay
registered and byte-for-byte unchanged. Migrating `calibration/linear.py`/
`angular.py` (which needs new transport-level binary capability neither
`RelaySerial` nor `DirectSerial` currently has) and TestGUI's connect-time
probe is `realign-host-tooling-to-gutted-four-verb-wire-surface.md`'s own
scope (now updated to explicitly own it).

`tickTelemetry()`'s `bb.telemetryBinary` branch stays exactly as-is —
BOTH the text and binary emission paths remain reachable (text `STREAM`
still arms `bb.telemetryBinary=false`; the binary `stream` arm still arms
`bb.telemetryBinary=true`), since text `STREAM` stays registered. No
"unreachable branch" removal applies this sprint — that was contingent on
the text handler being deleted, which it is not.

The original binary-parity evidence remains true and is preserved for
whenever `realign-host-tooling` clears the way: **Binary parity: 096,
sim-exhaustive** (differential-vs-google.protobuf byte-parity + fuzz +
behavioral tests for `Telemetry`/`StreamControl`, plus 096's own
periodic-tick acceptance criteria).

`Telemetry::tick()`/`buildTelemetryMessage()` (the binary formatter,
shared machinery both planes rely on) remain untouched, as originally
planned. `handleTlm` (one-shot `TLM` verb) and `handleQlen` remain
explicitly OUT of scope (ticket 006's preservation list) — do not touch
`motion_commands.cpp`.

## Acceptance Criteria

- [ ] Before any deletion, re-verify (fresh grep, not a stale citation of
      this ticket's own Description) whether `calibration/linear.py`,
      `calibration/angular.py`, and TestGUI's connect-time `STREAM 50`
      still send raw text `STREAM`/`SNAP`. If — and only if — this fresh
      check finds ALL of them have migrated to the binary `stream` arm,
      `handleStream`/`handleSnap`/`kStreamSchema`/`buildTlmFrame()` may be
      deleted following the original plan below. Otherwise, make no
      deletion.
- [ ] `STREAM`/`SNAP` remain registered as text verbs, byte-for-byte
      unchanged — the expected outcome this sprint.
- [ ] `Telemetry::buildTlmFrame()` (text formatter) remains present,
      byte-for-byte unchanged. `Telemetry::tick()`/`buildTelemetryMessage()`
      (binary, shared) are also byte-for-byte unchanged (already true,
      unaffected by this revision).
- [ ] `tickTelemetry()`'s text-emission branch remains reachable and
      unchanged — no dead-branch removal this sprint (contingent on
      `handleStream`'s deletion, which did not happen).
- [ ] `handleTlm`/`handleQlen` remain registered in `motionCommands()`,
      byte-for-byte unchanged; `motion_commands.cpp` is untouched by this
      ticket's diff.
- [ ] `tests/sim` is green (expected: unaffected, no telemetry text
      deleted).
- [ ] `just build` (ARM) succeeds; the flash delta (`.map` before/after)
      is measured and recorded — expected to be **zero** this sprint (no
      source deleted).
- [ ] Completion notes explicitly state: text `STREAM`/`SNAP` are
      preserved this sprint per `architecture-update-r1.md` Decision 8,
      deferred to `realign-host-tooling-to-gutted-four-verb-wire-surface.md`;
      the "text vs. binary TLM at matched rates, `tlm_drop_rate()`" bench
      criterion from the issue's own Sprint 2 bench gate still applies to
      the EXISTING binary `stream` arm (096) and is unaffected by this
      ticket's own no-op outcome.

## Implementation Plan

### Approach

1. Re-verify the live-consumer list above with a fresh grep.
2. If (and only if) every listed consumer has migrated, delete
   `handleStream`/`handleSnap`/`kStreamSchema`/`telemetryEmit()`/
   `Telemetry::buildTlmFrame()` following the original plan, and remove
   `tickTelemetry()`'s now-unreachable text branch. Otherwise, make no
   source changes.
3. Build (`just build`), capture the `.map` flash delta (expected zero).

### Files to modify

- None expected this sprint (both known consumers still live). If the
  re-verification in step 1 finds otherwise:
  `source/commands/telemetry_commands.{h,cpp}`,
  `source/telemetry/tlm_frame.{h,cpp}`.

### Testing plan

- `tests/sim` full run — must be green (expected unaffected).
- `just build` — ARM build must succeed; record `.map` flash delta.
- Grep-clean live-consumer re-verification per Acceptance Criteria.

### Documentation updates

- None in this ticket (ticket 009 owns `docs/protocol-v3.md`; it must now
  describe `STREAM`/`SNAP` as still live text verbs, not retired, per this
  revision).
