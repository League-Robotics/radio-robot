---
id: '003'
title: 'Telemetry: cycle_busy/cycle_period loop-timing fields end to end'
status: in-progress
use-cases:
- SUC-003
depends-on:
- '002'
github-issue: ''
issue: telemetry-report-loop-cycle-duration.md
completes_issue: true
exception:
  thrown_by: programmer
  thrown_at: '2026-07-24T21:10:03.882773+00:00'
  attempted: 'Recovered the ticket from `exception` to `in-progress` per the stakeholder''s
    approved resolution (raise the self-imposed 186-byte envelope budget so cycle_busy/cycle_period
    fit on the PER-CYCLE primary Telemetry frame; radio MTU=247 has real headroom).
    Before touching any code, I ran the MANDATORY pre-commit transport verification
    the resolution itself required (confirm no other transport caps below the new
    budget). Radio path (src/firm/com/radio.h): confirmed safe -- MTU=247 per fragment,
    FRAME_HEADER=3, REASM_MAX=512-byte reassembly buffer, Radio::send() already fragments
    messages over MTU across multiple RAW250 frames; real headroom exists exactly
    as the resolution assumed. Host-side read path (src/host/robot_radio/io/serial_conn.py,
    testgui/transport.py): uses pyserial''s ser.readline(), which grows dynamically
    with no fixed-size cap -- confirmed no host-side truncation risk. Repo-wide grep
    for ''186''/''188''/kReplyEnvelopeMaxEncodedSize/kArmoredBufSize found only comment-level
    mirrors of the generated wire.h constant (fit_sim_error_model.py, sim_loop.py,
    _legacy_tlm_text.py, sim_ctypes.cpp, test files) -- none are independent hard
    caps; all update automatically when wire.h is regenerated. Then I traced the ACTUAL
    outbound path for Comms::sendReply() (src/firm/app/comms.cpp lines 126-151): it
    armors the reply ("*B" + base64, no CRLF yet) and calls serialLink_.send(armored)
    UNCONDITIONALLY every cycle (not gated by transport), which reaches SerialPort::send()
    (src/firm/com/serial_port.cpp:37-42), which appends "\r\n" and calls the underlying
    CODAL `_serial.send(ManagedString, ASYNC)`. I traced that call into codal-core''s
    Serial::send()/setTxInterrupt() (src/libraries/codal-core/source/driver-models/Serial.cpp
    lines 83-114, 341-396) and Serial::setTxBufferSize() (lines 1023-1041): the TX
    ring buffer size is set via `SerialPort::begin()`''s `_serial.setTxBufferSize(255)`
    (serial_port.cpp:15) -- 255 is the literal maximum representable by the `uint8_t
    size` parameter, and setTxBufferSize''s own special-case (`if (size != 255) size++;`)
    confirms 255 is already the ceiling, not a headroom choice. A standard circular
    buffer with `txBuffSize=255` slots can hold at most `txBuffSize-1=254` bytes before
    `nextHead == txBuffTail` triggers full (Serial.cpp:89-91); in ASYNC mode (comms.cpp/serial_port.cpp
    never uses SYNC_* for telemetry) the copy loop does `if (mode == ASYNC) break;`
    on full (Serial.cpp:101-102), silently discarding every remaining byte of the
    line -- this is the exact "drop-on-full" truncation the code''s own comments warn
    about, but at the byte level mid-line, not merely a dropped frame. I computed
    the worst-case armored+CRLF wire-line length at the CURRENT budget: base64EncodedLength(185)
    = ((185+2)/3)*4 = 248 (wire_runtime.cpp:277); armored = 2("*B") + 248 = 250; +
    "\r\n" = 252 bytes actually written to send(). Usable TX capacity is 254 bytes
    -- only 2 bytes of headroom, not the 1-byte headroom DESIGN.md records at the
    proto-encoding layer, but a SEPARATE, tighter headroom at the wire/serial-transport
    layer that no existing comment (comms.h''s kArmoredBufSize note only discusses
    the RX side, `_rxBuf[256]`) documents. I then computed the ceiling for any raise:
    base64 output grows in fixed +4-byte jumps at every 3-byte boundary of the raw
    envelope, so raising the raw budget from 186 to 187 already requires ceil(187/3)=63
    groups (252 b64 bytes, +4 for "*B"+CRLF = 256) against a 254-byte hard capacity
    -- a guaranteed 2-byte overflow/truncation on literally every over-186-byte frame
    emitted over serial, not an occasional-backpressure risk. There is no raw envelope
    size above 186 that fits the real CODAL TX ring buffer at all, so the two new
    uint32 fields (worst case ~198B, or ~194B even with the tightest honest (max)
    proto bounds per the prior exception''s own arithmetic) cannot be added to the
    primary per-cycle Telemetry frame without corrupting every serial-transport telemetry
    emission. I did not write any source changes -- ticket remains unimplemented,
    exactly as found on recovery, pending stakeholder guidance.'
  conflict: 'The stakeholder''s 2026-07-24 resolution for 122-003 rests on the premise
    that the 186-byte envelope budget is "conservative, NOT the physical limit" because
    "the micro:bit radio MTU is 247" -- true for the radio transport, but this repo''s
    Telemetry primary frame is NOT radio-only: Comms::sendReply() (src/firm/app/comms.cpp:126-151)
    unconditionally emits the SAME armored reply on BOTH serialLink_ and radioLink_
    every cycle, and src/.claude/rules/hardware-bench-testing.md''s Standing Verification
    Gate explicitly requires "Round-trip over the real link" including "serial at
    the bench" as a first-class, mandatory transport, not an optional/legacy one.
    The serial/CDC transport''s outbound capacity is bounded by codal-core''s `Serial`
    driver (src/libraries/codal-core/source/driver-models/Serial.cpp / inc/driver-models/Serial.h),
    a vendored SDK component this project does not own or modify (same vendor-boundary
    category as the already-excluded `system_timer_current_time_us()` per .claude/rules/coding-standards.md''s
    "External/vendor function names are excluded" section) -- `setTxBufferSize(uint8_t
    size)` caps the TX ring buffer at 255 slots / 254 usable bytes, a type-level ceiling,
    not a tunable one. `SerialPort::begin()` (src/firm/com/serial_port.cpp:15) already
    configures this at the maximum (255). At the CURRENT 185B/186-byte-budget worst
    case, the actual armored+CRLF wire line written to this buffer is 252 bytes --
    only 2 bytes below the 254-byte hard ceiling. Any raise of kReplyEnvelopeMaxEncodedSize
    above 186 (the stakeholder proposed ~200) crosses at least one base64 3-byte encoding
    boundary, adding a minimum of +4 encoded bytes, and guarantees the resulting wire
    line exceeds 254 bytes -- CODAL''s ASYNC-mode Serial::setTxInterrupt() (Serial.cpp:83-114)
    silently truncates mid-line when the ring buffer fills (`if (mode == ASYNC) break;`),
    corrupting the base64 payload and causing every over-budget primary Telemetry
    frame sent over serial to fail to decode host-side, on every single cycle, not
    intermittently. This is a genuinely different (and more binding) constraint than
    the one the prior exception (2026-07-24T20:42:02Z) identified: that exception
    flagged comms.h''s kArmoredBufSize=256 / SerialPort''s `_rxBuf[256]` as the blocker
    -- both are firmware-owned RAM scratch buffers that COULD in principle be resized
    by this project. The constraint found here is different and stricter: it sits
    one layer downstream, inside the vendored CODAL Serial driver''s TX ring buffer,
    and cannot be resolved by resizing any firmware-owned buffer at all (kArmoredBufSize,
    _rxBuf, or a hypothetical larger TX buffer request) because `setTxBufferSize`''s
    own parameter type (`uint8_t`) makes 255 slots an absolute ceiling regardless
    of what this project asks for. Raising the reply-envelope budget past 186 is therefore
    not implementable on the primary per-cycle frame over the serial transport without
    either (a) forking/patching the vendored CODAL Serial driver to use a wider buffer-size
    type (explicitly out of scope -- vendor SDK, same exclusion class as system_timer_current_time_us()),
    or (b) dropping serial as a per-cycle carrier of the primary Telemetry frame (a
    real architecture change to Comms::sendReply()''s "broadcast on BOTH transports
    every call" contract, cutting against hardware-bench-testing.md''s mandatory serial
    round-trip gate), or (c) moving the two new fields off the primary per-cycle frame
    entirely (contradicts the stakeholder''s explicit resolution that they "fit on
    the PER-CYCLE primary frame"). None of these is a discretionary implementer call
    -- each overrides either a vendor-SDK boundary, the bench-testing charter''s mandatory
    serial gate, or the stakeholder''s own explicit resolution text.'
  surface: internal
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

**2026-07-24 stakeholder resolution (recovery from the ticket's second
exception): the two fields land on `TelemetrySecondary` (fields 11/12),
NOT the primary `Telemetry` message the criteria below were originally
written against** — the primary frame has no budget left (serial TX-ring
ceiling; see both exception blocks above and telemetry.proto's own
`TelemetrySecondary` doc comment for the full derivation). This is
INTERIM, pending a future COBS+CRC framing rework. Every criterion below
is satisfied against this substitution; the substitution itself is called
out explicitly wherever it changes which message/struct is touched.

- [x] `src/protos/telemetry.proto`'s **`TelemetrySecondary`** message (not
      `Telemetry` — see resolution note above) gains two new `uint32`
      fields at the next free numbers (11 and 12 — `TelemetrySecondary`
      had no field 11/12 in flight elsewhere): `cycle_busy` and
      `cycle_period`, both `// [us]` with `(max) = 200000` bounds,
      additive only (no renumbering of existing fields, `Telemetry`
      untouched).
- [x] `App::Telemetry::SecondaryFrame` (`src/firm/app/telemetry.h`, not
      `Frame` — see resolution note) gains matching `uint32_t cycleBusy` /
      `uint32_t cyclePeriod` members (`// [us]`, no units in the
      identifier per project convention).
- [x] `App::RobotLoop::cycle()` computes both values from its existing
      `cycleStart` bookkeeping (a new `previousCycleStartUs_`/
      `everCycled_` pair tracks the previous cycle's high-resolution
      `clock_.nowMicros()` reading across calls — `markTime()`'s own
      `[ms]`-truncated `cycleStart` lacks the resolution these
      diagnostics need) and stages them onto `secondaryFrame_` before
      `tlm_.emit()`. Comment states explicitly where `cycle_busy` is
      measured (at frame staging, immediately after `updateTlm()`, same
      instant the primary frame's own snapshot is finalized).
- [x] Codecs regenerated (`gen_messages.py` for the C++ wire structs,
      `gen_pb2.py` for the Python `pb2` bindings, both run via `build.py`);
      `messages/telemetry.h`'s generated `TelemetrySecondary` struct and
      `wire.cpp`'s encode path carry the new fields;
      `kTelemetrySecondaryMaxEncodedSize` regenerated 52 -> 60 B (well
      under the 186-byte budget); `kReplyEnvelopeMaxEncodedSize` (the
      primary frame) UNCHANGED at 185 B.
- [x] Host exposes both fields: `drain_binary_secondary_tlm()`
      (`src/host/robot_radio/io/serial_conn.py`) already returns raw
      `telemetry_pb2.TelemetrySecondary` objects (regenerated pb2 —
      `cycle_busy`/`cycle_period` are automatically present, no adapter
      needed, unlike `TLMFrame` which wraps the primary frame only).
- [x] TestGUI telemetry panel displays one line showing both — new `loop`
      row (`telemetry_panel.py`'s `fmt_loop_timing()`), e.g.
      `3.2ms / 40.0ms` — wired end to end via a new
      `Transport.on_telemetry_secondary` callback
      (`testgui/transport.py`, `_HardwareTransport._reader_loop()` now
      also drains `drain_binary_secondary_tlm()`) and a new
      `_TelemetryBridge.secondary_ready` Qt signal/slot
      (`testgui/__main__.py`) calling
      `telemetry_ctrl.update_secondary()`. Hardware transports only —
      `SimTransport` does not decode `TelemetrySecondary` at all yet (a
      pre-existing gap shared by every other secondary-frame field, not a
      regression this ticket introduces).
- [x] A sim unit test (`app_robot_loop_harness.cpp`'s
      `scenarioSecondaryFrameCarriesExactLoopTimingFields()`) asserts
      EXACT `cycle_busy`/`cycle_period` values under `TestSim`'s
      deterministic virtual clock across (non-adjacent, cadence-selected)
      cycles — `cycle_busy` is exactly 0 (no simulated time passes within
      one synchronous `cycle()` call against a test-driven clock — the
      correct, exact, deterministic value here, not a dodge) and
      `cycle_period` exactly matches each checked cycle's own
      deliberately-non-uniform clock advance, decoded off the real wire
      via `TestSupport::decodeOutboundLine()`.
- [x] The wire change is backward-compatible: additive `TelemetrySecondary`
      fields only; the existing wire round-trip/codec/differential-fuzz
      tests (`wire_codec_harness`, `wire_differential_harness` — extended
      this ticket to also fuzz-cover the two new fields, not just
      default-zero them) all pass; no existing decode path breaks.
- [x] No measurable emit-cost regression: two extra `uint32` field writes
      plus two subtractions per cycle, no new bus/sleep work — confirmed
      by the full sim suite's unchanged pass count (1407 before and
      after — see ticket completion notes for the pytest-count nuance).

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
