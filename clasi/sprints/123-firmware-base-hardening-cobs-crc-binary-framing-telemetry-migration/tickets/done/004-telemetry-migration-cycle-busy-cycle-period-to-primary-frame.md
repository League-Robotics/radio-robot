---
id: '004'
title: Telemetry migration - cycle_busy/cycle_period to primary frame
status: done
use-cases:
- SUC-005
depends-on:
- '002'
- '003'
github-issue: ''
issue: cobs-crc-binary-framing-replace-base64-armor.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Telemetry migration - cycle_busy/cycle_period to primary frame

## Description

Move `cycle_busy`/`cycle_period` (added to
`TelemetrySecondary` as an interim placement by sprint 122 ticket 003)
onto the primary per-cycle `Telemetry` frame, now that COBS+CRC has
restored primary-frame headroom. Resolve Open Question 4 (remove from
secondary, or keep on both) with the stakeholder before finishing this
ticket.

## Acceptance Criteria

- [x] `cycle_busy`/`cycle_period` present on the primary `Telemetry`
      frame every cycle (proto field addition, `Frame` staging using the
      existing `previousCycleStartUs_`/`everCycled_` bookkeeping —
      unchanged mechanism, only destination frame moves).
- [x] Sim test asserts exact per-cycle values under the virtual clock
      (mirroring sprint 122 ticket 003's own test, relocated).
- [x] Host `TLMFrame` and the TestGUI telemetry panel read the fields
      from the primary frame.
- [x] `TelemetrySecondary`'s copy is removed (or explicitly retained
      with a stated reason) per Open Question 4's resolution — not left
      ambiguous.

## Implementation Plan

- **Approach:** Straightforward field relocation now that ticket 002 has
  freed the headroom; reuses sprint 122's existing bookkeeping without
  change.
- **Files:** `src/firm/protos/telemetry.proto`, `src/firm/app/telemetry.{h,cpp}`,
  regenerated `messages/telemetry.h`/`wire.cpp`, host `TLMFrame` decode,
  TestGUI telemetry panel display line.
- **Testing:** Sim unit test (exact virtual-clock values); TestGUI manual
  check that the panel line still displays.
- **Documentation:** `app/DESIGN.md`'s "122-003" interim-placement note
  updated to reflect the completed migration, ride ticket 005.

## Completion Notes

- **Open Question 4 resolved: REMOVED from `TelemetrySecondary`**, not kept
  on both (locked stakeholder decision, 2026-07-25). Fields 11/12 are now
  `reserved` in `telemetry.proto`'s `TelemetrySecondary` message (not
  reused, matching this project's standing wire-stability discipline for a
  field that already shipped). `cycle_busy`/`cycle_period` land on
  `Telemetry` as new fields 15/16 (next free after 120's `acks`=14).
- **Primary-frame worst-case size vs the 240B budget:** `gen_messages.py`
  recomputes `kReplyEnvelopeMaxEncodedSize` to **194B** (up from 185B pre-
  migration, +9B for the two new bounded `uint32` fields) — 46B of margin
  under the 240B envelope budget, comfortably clear.
- **Armored-serial size vs the 254-byte CODAL ceiling:** `kMaxEnvelopeBytes`
  = 194B -> `kMaxCrcPayloadBytes` (+2B CRC-16) = 196B -> COBS worst-case
  encode (`cobsEncodedMaxLength(196) = 196 + 196/254 + 1`) = **197B**, plus
  the transport's own trailing `0x00` delimiter = **198B on the wire** —
  56B of margin under the 254-byte ceiling. `App::kFramedMaxBytes`
  (`src/firm/app/comms.h`) recomputed from 192 to 200 (3B headroom over the
  197B requirement) to match; its own `static_assert` against
  `cobsEncodedMaxLength(kMaxCrcPayloadBytes)` passes.
- **`TelemetrySecondary` worst-case size** dropped from 60B to 52B (two
  fewer `uint32` fields), still far under the 240B budget.
- **Firmware mechanism unchanged:** `RobotLoop::cycle()`'s
  `previousCycleStartUs_`/`everCycled_` bookkeeping is untouched — only the
  destination struct moved (`Telemetry::SecondaryFrame` ->
  `Telemetry::Frame`), staged via a second `tlm_.setFrame(frame_)` call
  right after `updateTlm()` so the values ride the SAME `emitPrimary()`
  call that finalizes the rest of that cycle's frame. `RobotLoop`'s own
  now-fully-inert `secondaryFrame_` member (every other `SecondaryFrame`
  field stays permanently unwired from this class) was removed along with
  the now-pointless `tlm_.setSecondaryFrame()` call — this was a required
  consequence of the field move (the struct that member's type points at
  lost the only fields RobotLoop ever populated on it), not scope creep.
- **Host/TestGUI:** `TLMFrame` (protocol.py) gained `cycle_busy`/
  `cycle_period`, always populated from the primary frame in `from_pb2()`.
  `telemetry_panel.py`'s "loop" row now reads them via `update_frame()`;
  `update_secondary()` and its `__main__.py`-side plumbing
  (`_pending_secondary`, `secondary_ready` signal, `on_secondary_ready`
  slot, `_on_secondary_thread_v2`, the `transport.on_telemetry_secondary`
  wiring) were removed — that whole path existed solely to feed this one
  row. `Transport.on_telemetry_secondary` itself (the general callback
  surface, `transport.py`) is untouched for a future caller wanting
  `TelemetrySecondary`'s other fields (`cmd_vel`/`acc_*`/`glitch_*`/
  `ts_*`).
- **Sim test relocated:** `app_robot_loop_harness.cpp`'s
  `scenarioSecondaryFrameCarriesExactLoopTimingFields` ->
  `scenarioPrimaryFrameCarriesExactLoopTimingFields`, now asserting
  `decoded.telemetry.cycle_busy`/`cycle_period` (via
  `TestSupport::DecodedKind::kTelemetry`) instead of
  `decoded.secondary.*`. The differential/fuzz suite
  (`test_wire_differential.py`, `_wire_diff_driver.py`,
  `wire_differential_harness.cpp`, `wire_test_codec.cpp`) moved the same
  two fields from its `TelemetrySecondary` full-shape corpus to its
  `Telemetry` full-shape corpus and field-number cross-check.
- **Suite numbers, before/after:** unchanged at **1428 passed / 2 skipped /
  9 xfailed / 2 xpassed** (`uv run python -m pytest -q`) — the migrated
  sim scenario replaces the secondary-frame one 1:1 (same C++ harness
  binary, same pytest-level test), and the differential-suite edits
  modify existing test bodies rather than adding/removing test functions,
  so the net pytest count does not shift. `uv run python3 build.py
  --clean` is green (ARM firmware + host-sim lib both build; all three
  `wire.h` envelope `static_assert`s pass at the 240B budget).
