---
id: '008'
title: 'Firmware: gut the text telemetry family'
status: open
use-cases: [SUC-008]
depends-on: []
github-issue: ''
issue: protocol-v3-schema-driven-binary-command-plane-protobuf.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Firmware: gut the text telemetry family

## Description

**REWRITTEN — see `architecture-update-r2.md` Decision 9 (supersedes r1's
Decision 8 and this ticket's own r1-conservative no-op scope in full).**
Eric's 2026-07-10 redirect: gut the firmware text plane unconditionally.
**DELETE**: `handleStream`/`handleSnap` and their `telemetryCommands()`
registrations, `kStreamSchema`, `telemetryEmit()`, and
`Telemetry::buildTlmFrame()` (the text-only formatter). ALSO DELETE
`handleTlm` (the one-shot text `TLM` verb, `motion_commands.cpp`) —
r1's Decision 7 preserved it as a bench-diagnostic rump with "no proven
binary substitute"; Decision 9 explicitly overrides that (its
bench-diagnostic fields — `acc`/`active`/`conn`/`glitch`/`ts` — are
already present in binary `Telemetry` per 096 Decision 6's union, and "no
proven substitute" is no longer, on its own, a preservation reason).
`handleTlm`'s deletion is this ticket's, not ticket 006's, since it lives
in the telemetry surface conceptually even though its source is in
`motion_commands.cpp` — coordinate the exact file edit with ticket 006 if
both land close together (same file, `motion_commands.cpp`, different
functions) to avoid a merge collision; note the coordination in
completion notes either way.

**Binary parity: 096, sim-exhaustive** (differential-vs-google.protobuf
byte-parity + fuzz + behavioral tests for `Telemetry`/`StreamControl`,
plus 096's own periodic-tick acceptance criteria: monotonic `seq=`,
correct on/off behavior). This ticket no longer depends on ticket 005's
verification (`depends-on: []`) — deletion is unconditional under
Decision 9.

`tickTelemetry()`'s `bb.telemetryBinary` branch: since only the binary
`stream` arm can set `bb.telemetryPeriod`/`.telemetryBinary` once
`handleStream` is gone, the text-emission branch becomes unreachable —
remove it. `Telemetry::tick()`/`buildTelemetryMessage()` (the binary
formatter, shared machinery) are UNTOUCHED.

**Every live text `STREAM`/`SNAP`/`TLM` sender r1 found —
`calibration/linear.py`, `calibration/angular.py`, TestGUI's connect-time
`"STREAM 50"`, and any ad-hoc `tests/bench/*.py` script sending raw
`"TLM"` — BREAKS against this firmware once this ticket lands, until
rewired to the `rogo` translator proxy (ticket 004).** This is an
accepted, stakeholder-approved consequence of Decision 9, not a
regression to fix here. State it plainly in completion notes.

## Acceptance Criteria

- [ ] `STREAM`/`SNAP` are no longer registered as text verbs.
- [ ] `Telemetry::buildTlmFrame()` (text formatter,
      `source/telemetry/tlm_frame.{h,cpp}`) is deleted.
      `Telemetry::tick()`/`buildTelemetryMessage()` (binary, shared) are
      byte-for-byte unchanged.
- [ ] `tickTelemetry()`'s now-unreachable text-emission branch is removed.
- [ ] `handleTlm`/`TLM` (one-shot verb, `motion_commands.cpp`) is deleted
      and unregistered — coordinate this specific edit with ticket 006
      (same file).
- [ ] `handleQlen`/`QLEN` is deleted per ticket 006's own scope, NOT this
      ticket's — confirm no duplicate/conflicting edit.
- [ ] `tests/sim/unit/*` tests exercising text STREAM/SNAP/one-shot TLM
      are re-pointed at the binary `stream` arm (including a case
      exercising the host's `snap()`-equivalent arm-wait-disarm sequence,
      or the firmware-side portion of it) — coverage maintained.
- [ ] `tests/sim` is green.
- [ ] `just build` (ARM) succeeds; the flash delta (`.map` before/after)
      is measured and recorded — expected to be a real, meaningful
      reduction (the text formatter + both handlers + the schema table).
- [ ] Completion notes state plainly which live text `STREAM`/`SNAP`
      senders (listed in Description) now break against this firmware,
      and that rewiring them to the `rogo` proxy (ticket 004) is deferred
      to `realign-host-tooling-to-gutted-four-verb-wire-surface.md`.

## Implementation Plan

### Approach

1. Delete `handleStream`/`handleSnap`/`kStreamSchema`/`telemetryEmit()`
   and their `telemetryCommands()` registrations from
   `telemetry_commands.{h,cpp}`.
2. Delete `Telemetry::buildTlmFrame()` from `tlm_frame.{h,cpp}`.
3. Remove `tickTelemetry()`'s now-unreachable text branch.
4. Delete `handleTlm`/`TLM`'s registration from `motion_commands.cpp`
   (coordinate with ticket 006's own edits to the same file — confirm no
   overlap/conflict with `QLEN`'s deletion, which is ticket 006's).
5. Update `tests/sim/unit/*` per Acceptance Criteria.
6. Build (`just build`), capture the `.map` flash delta.

### Files to modify

- `source/commands/telemetry_commands.{h,cpp}`
- `source/telemetry/tlm_frame.{h,cpp}`
- `source/commands/motion_commands.cpp` (`handleTlm`/`TLM` only — shared
  file with ticket 006, coordinate)
- Affected `tests/sim/unit/*` test files

### Testing plan

- `tests/sim` full run — must be green.
- `just build` — ARM build must succeed; record `.map` flash delta.
- Grep-clean checks listed in Acceptance Criteria.

### Documentation updates

- None in this ticket (ticket 009 owns `docs/protocol-v3.md`; it must now
  describe telemetry as binary-only, with the `rogo` proxy as the
  text-compatibility path).
