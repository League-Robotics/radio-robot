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

- [x] `Telemetry`'s ack storage is a bounded ring (depth 4), not a
      single `ackCorr_`/`ackErr_` pair; `ack(corrId, errCode)` pushes an
      entry; a push past depth 4 evicts the oldest.
- [x] `ack_corr`/`ack_err` (existing scalar fields) and `flags` bit 5
      (`kFlagAckFresh`) keep their exact prior meaning — "the freshest
      ack" — unchanged for any existing reader; the wire change is
      additive only (new field added, nothing renumbered or removed).
- [x] `telemetry.proto` gains a new repeated ack-entry field; the wire
      shape is documented in `docs/protocol-v4.md` (§7's ack section
      and the frame field table), following the same append-only
      convention 119 used for `flags` bit 16.
- [x] Host decode (`src/host/robot_radio`) reads the new repeated field;
      `wait_for_ack()`'s matching core (`io/serial_conn.py`) scans the
      ring for a matching `corr_id` instead of one scalar pair — document
      which matching policy was chosen (Architecture Step 7's open
      question).
- [ ] `move_protocol_bench.py` reaches 43/43 **on hardware** (robot on
      the stand, real serial link). **NOT MET this session** — see
      "Hardware Verification Results" below: repeated runs landed at
      38/43, 34/43, 33/43, 30/43, 35/43, root-caused via an A/B test
      against the unmodified pre-120 firmware+host code (same symptom,
      same rate) to a pre-existing, out-of-scope dropped-envelope issue,
      filed as `clasi/issues/bench-move-commands-intermittently-never-
      reach-firmware.md`. The ack ring itself is proven correct by the
      two criteria below, which do not depend on that pre-existing gap.
- [x] A new rapid-fire N-enqueue test (N up to the `MoveQueue`'s 5-deep
      `ERR_FULL` ceiling) surfaces all N acks over the real link. Verified
      3 separate runs, 15/15 acks observed
      (`src/tests/bench/ack_ring_rapid_fire_bench.py`).
- [ ] `twist_drive.py`'s `stop()` ack lands on every run. Landed 2 of 3
      runs this session; the 1 miss showed the SAME pre-existing
      dropped-envelope signature (zero encoder movement, not a ring-depth
      miss) as the `move_protocol_bench.py` finding above — leaving this
      box unchecked rather than claiming "every run" without having
      observed it.
- [x] `src/firm/app/DESIGN.md`'s "Telemetry's ack ring" paragraph
      (already drafted in this sprint's design overlay,
      `clasi/sprints/120-bench-tour-bring-up-with-fake-otos/design/DESIGN.md`)
      is verified/refined against the shipped code and applied to the
      canonical doc at sprint close. Overlay edited in place this ticket
      (LANDED, no longer DRAFT); canonical `src/firm/app/DESIGN.md` gets
      the merge at sprint close per the overlay convention.
- [x] `src/host/robot_radio/DESIGN.md` gets a direct edit describing the
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

## Implementation Notes (as shipped)

- Wire layout: `telemetry.proto` gains `message AckEntry { uint32 corr_id
  = 1; uint32 err = 2; }` and `Telemetry.acks = 14
  [(max_count) = 4]` (`repeated AckEntry`). Purely additive — fields 1-13
  unchanged, nothing renumbered.
- Firmware: `App::kAckRingDepth = 4` (`telemetry.h`). `Telemetry` gains
  `msg::AckEntry ackRing_[4]`, `ackRingHead_`, `ackRingCount_`
  (`telemetry.h`/`.cpp`). `ack()` pushes onto BOTH the pre-120 scalar
  pair (unchanged) and the ring (`pushAckRing()` — classic bounded
  circular buffer, oldest evicted on overflow). `emitPrimary()`
  serializes the ring's current contents (oldest-to-newest) into the new
  wire field every call, matching every other `Frame` field's "persists
  until changed" contract. A `static_assert` in `telemetry.cpp` checks
  `kAckRingDepth` against the generated `msg::Telemetry::acks_[]` array
  width so a future edit to either side without the other fails the
  build.
- Wire size: `ReplyEnvelope`'s worst case grows from 153 B to **185 B**
  (1 B margin under the 186-byte budget) — see `docs/protocol-v4.md`
  §8.3 for the full breakdown; this is now the tightest margin in the
  schema.
- Host: `SerialConnection.wait_for_ack()`/`_match_ack_in_frames()`
  (`io/serial_conn.py`) scan the ring, returning the first matching
  `(frame, ring-entry)` pair in arrival/wire order — no freshness gate
  needed for a ring scan (see Design Rationale in `app/DESIGN.md`'s
  overlay). `NezhaProtocol.wait_for_ack()`/`AckEntry.from_ring_entry()`
  (`robot/protocol.py`) adapt the matched raw `telemetry_pb2.AckEntry`.
  `TLMFrame.acks` (new, additive) exposes the full decoded ring per
  frame.
- Found and fixed along the way: `wire_codec_harness.cpp`'s
  `scenarioEncodeTelemetryFlagsAckAndReadings()`/
  `scenarioEncodeOversizedBufferReturnsZero()` hand-constructed a
  `msg::Telemetry` through the `ReplyEnvelope.body` UNION member without
  zeroing it first — a pre-existing hazard (the union's own `= {}`
  default-initializer only value-initializes the FIRST alternative, `Ack`,
  per the same class of bug 095-006 already fixed on the decode side) that
  had never surfaced because no prior `Telemetry` field had a
  garbage-sensitive LOOP BOUND. The new `acks_count` field does: ASan
  caught a stack-buffer-overflow (`ubsan`/`asan` test,
  `test_wire_codec.py::test_wire_codec_harness_asan_ubsan`) where an
  uninitialized `acks_count` produced a garbage `kRepeatedMessage` loop
  bound in `encodeInto()`. Fixed by explicitly zero-assigning the
  `Telemetry` union member (`t = msg::Telemetry{};`) before touching any
  field by hand, in both scenarios — the same fix shape `decode()`'s own
  095-006 `memset` uses, applied on the encode/hand-construct side.
- Extended `src/tests/sim/unit/wire_differential_harness.cpp`'s
  `encode_telemetry` verb (and `_wire_diff_driver.py`'s Python wrapper)
  to accept the ack ring, and added
  `test_direction_b_telemetry_ack_ring_*` differential tests
  (`test_wire_differential.py`) — the ring round-trips through the REAL
  `google.protobuf` decoder, not just this codec's own self-consistency,
  since this is the first schema in this tree to ever exercise the
  generic `kRepeatedMessage` engine path.
- New bench script `src/tests/bench/ack_ring_rapid_fire_bench.py` (kept
  separate from `move_protocol_bench.py` so that file's own 43-check
  count, and this ticket's before/after framing against it, stays
  unchanged) — fires N=5 back-to-back `move_twist()` enqueues with no
  inter-send wait and confirms every ack surfaces via the ring.

## Hardware Verification Results (2026-07-23, robot "tovez",
`/dev/cu.usbmodem2121102`, UID `9906360200052820a8fdb5e413abb276000000006e052820`)

- `mbdeploy deploy` — succeeded (APPROTECT auto-mass-erase + retry, as
  anticipated).
- **`ack_ring_rapid_fire_bench.py` (N=5): 3/3 runs, 15/15 individual acks
  observed, plus 3/3 STOP cleanup acks.** The ring's own headline
  acceptance property — this is rock solid.
- **`twist_drive.py`: 2 of 3 runs 6/6 clean** (including the
  previously-always-missed `stop()` ack landing via the ring); 1 run 3/6,
  with the miss showing the pre-existing dropped-envelope signature (see
  below), not a ring-depth miss.
- **`move_protocol_bench.py`: did NOT reach 43/43.** Five consecutive
  runs in this session: 38/43, 34/43, 33/43, 30/43, 35/43 (including
  fresh `mbdeploy` reflashes between some runs). Every FAIL beyond the
  ack-observability ones this ticket targets showed `ack=None` **AND**
  zero encoder movement (e.g. `dtheta=0.000rad`, `d_left=0 d_right=0`) —
  i.e. the `CommandEnvelope` itself never reached/applied on the
  firmware, not merely an ack the ring failed to carry.
- **Root-cause isolation (A/B test):** checked out commit `047555a5`
  (the last commit before any 120-001 work) into a throwaway `git
  worktree`, built ONLY that firmware (RAM/flash usage byte-identical to
  the 120 build — 98.33% RAM both), reflashed the SAME robot, and ran
  that SAME pre-120 `move_protocol_bench.py` (single ack slot, no ring,
  code that never touches `CommandEnvelope` decode) against it: **38/43**
  — the IDENTICAL failure signature (ack=None + zero encoder movement on
  `scenario_angle_stop`/`scenario_wheels_variant_signs`). This proves the
  dropped-envelope symptom is pre-existing and NOT caused by this
  ticket's ack-ring change.
- Filed `clasi/issues/bench-move-commands-intermittently-never-reach-
  firmware.md` for the pre-existing issue, with the full A/B evidence and
  suggested next steps (out of this ticket's scope).
- Robot stopped and port released at the end of the session.
