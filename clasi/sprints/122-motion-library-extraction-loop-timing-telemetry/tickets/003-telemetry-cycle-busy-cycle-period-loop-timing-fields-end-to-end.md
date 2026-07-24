---
id: '003'
title: 'Telemetry: cycle_busy/cycle_period loop-timing fields end to end'
status: open
use-cases: [SUC-003]
depends-on: ["002"]
github-issue: ''
issue: telemetry-report-loop-cycle-duration.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Telemetry: cycle_busy/cycle_period loop-timing fields end to end

## Description

Add two `uint32 [us]` diagnostic fields to every telemetry frame:
`cycle_busy` (elapsed from this cycle's `cycleStart` to the end of the
cycle's work, measured at frame staging) and `cycle_period` (this cycle's
`cycleStart` minus the previous cycle's `cycleStart`). Per
`clasi/issues/telemetry-report-loop-cycle-duration.md`: I2C stalls, OTOS
retries, and comms bursts show up directly in `cycle_busy`; scheduling
jitter and overruns vs. the nominal `kCycle` (40 ms) show up in
`cycle_period`, and it is the true dt a host-side rate consumer should
use instead of assuming 40 ms.

`App::RobotLoop::cycle()` already computes `uint32_t cycleStart =
markTime();` (`robot_loop.cpp` line ~532) at the top of every cycle — this
ticket threads that value (and the previous cycle's) through to
`App::Telemetry::Frame` staging. This ticket is independent of the
motion/base split's module boundary (it touches `telemetry.*` and the
wire schema only) but is sequenced after ticket 002 so it lands against
the final post-extraction `robot_loop.cpp`/`telemetry.cpp` rather than an
intermediate shape.

## Acceptance Criteria

- [ ] `src/protos/telemetry.proto`'s `Telemetry` message gains two new
      `uint32` fields (next free field numbers — currently 15 and 16 as
      of this writing; re-verify against the live `.proto` at
      implementation time since other work may have claimed a number
      meanwhile): `cycle_busy` and `cycle_period`, both `// [us]`,
      additive only (no renumbering of existing fields).
- [ ] `App::Telemetry::Frame` (`src/firm/app/telemetry.h`) gains matching
      `uint32_t cycleBusy` / `uint32_t cyclePeriod` members (`// [us]`,
      no units in the identifier per project convention).
- [ ] `App::RobotLoop::cycle()` computes both values from its existing
      `cycleStart` bookkeeping (tracking the previous cycle's
      `cycleStart` across calls) and stages them onto the frame before
      `tlm_.emit()`. Comment states explicitly where `cycle_busy` is
      measured (at frame staging, per the issue's own field-comment
      requirement).
- [ ] Codecs regenerated (`src/scripts/` codegen for both the C++ wire
      structs and the Python `pb2` bindings); `messages/telemetry.h`'s
      generated struct and `wire.cpp`'s encode/decode paths carry the new
      fields.
- [ ] Host `TLMFrame` (`src/host/robot_radio/`) exposes both fields
      decoded from the wire.
- [ ] TestGUI telemetry panel displays one line showing both, e.g.
      `loop 3.2ms / 40.0ms` (busy/period, converted from `us` to a
      human-readable `ms` display).
- [ ] A sim unit test asserts EXACT `cycle_busy`/`cycle_period` values
      under `TestSim`'s deterministic virtual clock across at least two
      consecutive cycles (not a range/tolerance check) — proving both
      the "busy" and "period" computations, not just that the fields are
      present.
- [ ] The wire change is backward-compatible: an old host decoder that
      does not know about the two new fields still decodes the rest of
      the frame correctly (verify via the existing wire round-trip/codec
      tests — no existing decode path breaks).
- [ ] No measurable emit-cost regression (two extra `uint32` fields;
      this should be a non-issue, but confirm no new per-cycle work
      beyond the two subtractions).

## Testing

- **Existing tests to run**: `uv run pytest` (wire codec/differential
  tests — `wire_codec_harness`, `wire_differential_harness` — plus
  `app_telemetry_harness`, `app_robot_loop_harness`); host-side
  `test_tlm_log.py` and any other test decoding `TLMFrame`.
- **New tests to write**: one sim unit test (new or extending
  `app_telemetry_harness.cpp`/`app_robot_loop_harness.cpp`) that steps the
  virtual clock a known, deliberately non-uniform number of `us` across
  two+ cycles and asserts the exact `cycle_busy`/`cycle_period` values
  staged onto the frame each time.
- **Verification command**: `uv run pytest`.

## Implementation Plan

**Approach**: thread `cycleStart` (already computed) and a newly-added
`previousCycleStart_` member through `RobotLoop` to `Telemetry::Frame`
staging; extend the proto, regenerate codecs, wire through the host
decoder and GUI display; add the deterministic sim assertion.

**Files to modify**:
- `src/protos/telemetry.proto` (two new fields on `Telemetry`)
- `src/firm/app/telemetry.h` / `telemetry.cpp` (`Frame` struct + staging)
- `src/firm/app/robot_loop.h` / `robot_loop.cpp` (track previous
  `cycleStart`, compute both values, stage before `tlm_.emit()`)
- `src/firm/messages/telemetry.h` (generated — regenerate, do not hand-edit)
- `src/firm/messages/wire.cpp` (generated encode/decode — regenerate)
- Host: `src/host/robot_radio/robot/pb2/*_pb2.py` (regenerated),
  wherever `TLMFrame` decodes/exposes `Telemetry` fields
- TestGUI telemetry panel source (one new display line)

**Testing plan**: extend or add a sim unit test harness that drives
`RobotLoop` two or more cycles under `TestSim::SimClock` with a
deliberately non-uniform step (so `cycle_period` differs cycle to cycle
and is distinguishable from a constant), and assert the staged frame's
`cycleBusy`/`cyclePeriod` match hand-computed expected values exactly.

**Documentation updates**: `src/firm/app/DESIGN.md`'s telemetry section
gains a line noting the two new fields (folded into ticket 004's broader
reconciliation pass if that lands after this ticket in execution, or
added directly here if simpler — either is acceptable, just don't leave
the frame's documented field list stale).
