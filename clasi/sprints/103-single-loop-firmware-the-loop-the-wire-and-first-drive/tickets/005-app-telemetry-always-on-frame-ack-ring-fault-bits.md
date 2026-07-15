---
id: '005'
title: "app/Telemetry — always-on frame, ack ring, fault bits"
status: open
use-cases: [SUC-005]
depends-on: ['001', '004']
github-issue: ''
issue: single-loop-firmware-p3-p7-continuation.md
completes_issue:
  single-loop-firmware-p3-p7-continuation.md: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# app/Telemetry — always-on frame, ack ring, fault bits

## Description

Build `source/app/telemetry.{h,cpp}`: assembles and emits the primary
telemetry frame (via `Comms`'s send path, ticket 004) every cycle,
carrying the depth-3 ack ring and fault/event bits (ticket 001's schema),
plus the `TelemetrySecondary` slow frame on whatever framing/cadence
ticket 001 decided (Decision 3). This is the highest-risk piece of this
sprint — the primary frame's measured margin against the 186B ceiling is
only 7B (spike-003's number; ticket 001 re-derives it for real), so field
additions here are zero-slack.

Depends on ticket 001 (the `Telemetry`/`TelemetrySecondary`/`AckEntry`
types) and ticket 004 (`Comms::sendReply()` is the actual send path this
module calls).

## Acceptance Criteria

- [ ] `Telemetry::emit()` builds one primary frame per call
      (`now`/`mode`/`seq`/`enc`/`vel`/`pose`/`otos`+`otos_connected`/
      `twist`/`active`/`conn_left`/`conn_right`/`fault_bits`/`event_bits`,
      or ticket 001's own final field list if it differs) and sends it via
      `Comms`.
- [ ] `Telemetry::ack(corrId, status, errCode)` pushes one entry into the
      ring; the ring holds exactly the last 3 entries (oldest evicted);
      every `emit()` call includes the current ring contents in full (not
      just new entries) so a single dropped frame cannot lose an ack.
- [ ] A host-buildable unit test proves the ring survives one dropped
      frame: push 4 sequential acks, build 2 successive frames (simulating
      one "lost" frame in between by not reading the first), confirm the
      newest 3 acks are present in the frame that IS read.
- [ ] `fault_bits`/`event_bits` bit layout is decided and documented (this
      ticket's own decision — ticket 001 declared the fields, not the
      layout). At minimum: one bit for the `I2CBus` `readyAt` safety-net
      trip (ticket 002) and one bit for a `Deadman` expiry (ticket 004) are
      wired to real call sites, not left as dead bit positions.
- [ ] `TelemetrySecondary` is emitted on the framing/cadence ticket 001
      decided; its own emission does not starve or delay the primary
      frame's cadence.
- [ ] Measured emission cadence (both frame types, real timestamps from a
      host-buildable or bench test) is recorded against the 25 Hz/40ms
      target from spike-001 — this ticket does not need to HIT 25 Hz
      exactly, but must report its own actual number rather than assuming
      it matches spike-001's pre-rewrite baseline (spike-001's own
      ~30.3 Hz armed vs. ~26.8 Hz actual gap was never root-caused).

## Implementation Plan

**Approach**: Build the frame assembly first against a fixed, fake data
source (host-buildable), verify the ring/budget logic in isolation, THEN
wire real data sources (motor leaves, `Otos`, `Deadman`, `I2CBus`) once
tickets 002/003/006/007 exist — this ticket's own acceptance criteria are
satisfiable with `Telemetry` as a standalone, testable class before the
full loop (ticket 008) wires it to live devices.

**Files to create/modify**:
- `source/app/telemetry.h`, `source/app/telemetry.cpp` (new)

**Testing plan**:
- Existing tests to run: ticket 001's rewritten wire test suite (confirms
  the underlying `Telemetry`/`AckEntry` encode path this module calls is
  itself correct).
- New tests to write: the ack-ring-survives-a-drop test (Acceptance
  Criteria above); a frame-size test confirming the assembled frame's
  encoded size matches ticket 001's recorded worst case (or is smaller,
  for a partially-populated frame); a fault-bit test confirming each wired
  bit flips when its trigger condition is simulated.
- Verification command: `uv run python -m pytest tests/sim/unit/ -k telemetry`
  (once the test file exists) plus a host-side timing measurement script
  under `tests/bench/` or `tests/sim/` for the cadence acceptance
  criterion (can run host-buildable with a scripted clock, does not
  require hardware for this ticket).

**Documentation updates**: record the `fault_bits`/`event_bits` layout in
a comment block at the top of `telemetry.h` (the single place a future
reader looks to decode a fault bit) and in this ticket's completion notes.
