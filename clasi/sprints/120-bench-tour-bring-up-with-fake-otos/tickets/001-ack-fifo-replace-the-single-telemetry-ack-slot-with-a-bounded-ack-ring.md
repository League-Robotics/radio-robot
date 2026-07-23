---
id: '001'
title: 'Ack FIFO: replace the single telemetry ack slot with a bounded ack ring'
status: in-progress
use-cases:
- SUC-069
depends-on: []
github-issue: ''
issue: bench-single-ack-slot-observability-collapses-at-40ms.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Ack FIFO: replace the single telemetry ack slot with a bounded ack ring

## Description

`Telemetry`'s single ack slot (`ack_corr`/`ack_err`, valid iff `flags`
bit 5) loses transient enqueue/CONFIG/STOP acks at the real 40ms cycle
against a ~15Hz host read rate: a second command's ack inside the same
primary period silently overwrites the first. On real hardware,
`move_protocol_bench.py` is 31/43 (every FAIL a missed transient ack,
every functional behavior — moves, completions, timeout, drain,
preempt, STOP-flush — correct); `twist_drive.py` misses its `stop()`
ack. This actively corrupts bench MEASUREMENT harnesses, not just the
acceptance gate (a team-lead turn-accuracy capture lost 3/6 enqueue
acks, each miss cascading into a garbage heading reading).

This ticket replaces the single slot with a small, bounded ack ring
(depth 4) so a host reading at the current rate still drains every ack,
including rapid-fire bursts. This is FIRST in this sprint's dependency
order — ticket 002's own bench-tour acceptance criteria depend on
reliable enqueue-ack observability to confirm each leg of a tour was
actually accepted, not silently `ERR_FULL`'d.

See sprint.md's Architecture (Step 5, Step 6 Decision 1) for the full
rationale, and the Design Overlay's `src/firm/app/DESIGN.md` overlay
("Telemetry's ack ring") for the updated contract this ticket ships.

## Acceptance Criteria

- [ ] `Telemetry`'s ack storage is a bounded ring (depth 4), not a
      single `ackCorr_`/`ackErr_` pair; `ack(corrId, errCode)` pushes an
      entry; a push past depth 4 evicts the oldest.
- [ ] `ack_corr`/`ack_err` (existing scalar fields) and `flags` bit 5
      (`kFlagAckFresh`) keep their exact prior meaning — "the freshest
      ack" — unchanged for any existing reader; the wire change is
      additive only (new field added, nothing renumbered or removed).
- [ ] `telemetry.proto` gains a new repeated ack-entry field; the wire
      shape is documented in `docs/protocol-v4.md` (§7's ack section
      and the frame field table), following the same append-only
      convention 119 used for `flags` bit 16.
- [ ] Host decode (`src/host/robot_radio`) reads the new repeated field;
      `wait_for_ack()`'s matching core (`io/serial_conn.py`) scans the
      ring for a matching `corr_id` instead of one scalar pair — document
      which matching policy was chosen (Architecture Step 7's open
      question).
- [ ] `move_protocol_bench.py` reaches 43/43 **on hardware** (robot on
      the stand, real serial link).
- [ ] A new rapid-fire N-enqueue test (N up to the `MoveQueue`'s 5-deep
      `ERR_FULL` ceiling) surfaces all N acks over the real link.
- [ ] `twist_drive.py`'s `stop()` ack lands on every run.
- [ ] `src/firm/app/DESIGN.md`'s "Telemetry's ack ring" paragraph
      (already drafted in this sprint's design overlay,
      `clasi/sprints/120-bench-tour-bring-up-with-fake-otos/design/DESIGN.md`)
      is verified/refined against the shipped code and applied to the
      canonical doc at sprint close.
- [ ] `src/host/robot_radio/DESIGN.md` gets a direct edit describing the
      ring-aware `wait_for_ack()` behavior (per this sprint's Design
      Overlay — not overlaid, ticket-direct-edit).

## Implementation Plan

### Approach

1. Add a small, purpose-built ack ring type inside `app/telemetry.{h,cpp}`
   (push/drain, no interpolation — deliberately NOT
   `Devices::MeasurementRing<T>`, which is built for continuous,
   interpolatable samples; see sprint.md Architecture Decision 1 for the
   full rejection rationale). Depth 4, oldest-evicted-first.
2. `Telemetry::ack(corrId, errCode)` pushes onto the ring (in addition to
   still updating the existing `ackCorr_`/`ackErr_` "freshest" pair and
   pulsing `kFlagAckFresh` exactly as today — no change to that existing
   behavior).
3. `Telemetry::emit()`/`emitPrimary()` serializes the ring's current
   contents into the new wire field.
4. Bump `src/protos/telemetry.proto`: add a new `AckEntry`-shaped
   repeated field to `Telemetry` at the next free field number (14).
   Regenerate host + firmware bindings (`src/scripts/` codegen).
5. Update `docs/protocol-v4.md`: the ack section (§7) and the frame field
   table, following the append-only convention already used for `flags`
   bit 16 (119).
6. Host: extend the `Telemetry`/`TLMFrame` decode path and
   `wait_for_ack()`'s matcher (`io/serial_conn.py`,
   `robot/protocol.py`'s `NezhaProtocol`) to scan the new ring.
7. Build + flash (`just build-clean` then `mbdeploy deploy <robot-UID>
   --hex MICROBIT.hex`; robot UID
   `9906360200052820a8fdb5e413abb276000000006e052820`; APPROTECT
   auto-mass-erase is expected/normal; reflash once more if comms look
   malformed post-mass-erase). Run `move_protocol_bench.py`,
   `twist_drive.py`, and a new rapid-fire N-enqueue scenario against the
   real robot on the stand.

### Files to Create/Modify

- `src/firm/app/telemetry.h` / `.cpp` — ack ring storage, `ack()`,
  `emit()`/`emitPrimary()`.
- `src/protos/telemetry.proto` — new repeated ack-entry field.
- `docs/protocol-v4.md` — §7 ack description, frame field table.
- `src/host/robot_radio/io/serial_conn.py` — decode + `wait_for_ack()`
  ring scan.
- `src/host/robot_radio/robot/protocol.py` — `NezhaProtocol`'s ack
  surface, if it needs updating to expose more than one match.
- `src/tests/bench/move_protocol_bench.py` — new rapid-fire N-enqueue
  scenario.
- `src/firm/app/DESIGN.md` (canonical) — apply this sprint's overlay
  edit at close.
- `src/host/robot_radio/DESIGN.md` — direct edit (per Design Overlay).

### Testing Plan

- Unit/sim: ack ring push/drain logic (eviction at depth 4, ordering),
  host decode of the new field.
- Hardware (required, this sprint IS the bench session): `just
  build-clean`, `mbdeploy deploy`, then `move_protocol_bench.py` (target
  43/43), `twist_drive.py`, and the new rapid-fire N-enqueue test, all
  over `/dev/cu.usbmodem2121102`. Record pass/fail output in this
  ticket.

### Documentation Updates

- `src/firm/app/DESIGN.md` — apply this sprint's overlay diff.
- `src/host/robot_radio/DESIGN.md` — direct edit describing the
  ring-aware matcher.
- `docs/protocol-v4.md` — wire-level ack section + field table.
