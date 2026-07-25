---
id: '002'
title: Firmware framer integration (Comms + transports + wire budget)
status: exception
use-cases:
- SUC-001
- SUC-002
- SUC-004
depends-on:
- '001'
github-issue: ''
issue: cobs-crc-binary-framing-replace-base64-armor.md
completes_issue: true
exception:
  thrown_by: programmer
  thrown_at: '2026-07-25T02:07:58.906199+00:00'
  attempted: 'Implemented the full COBS+CRC firmware framer within ticket 002''s authorized
    scope: App::Comms (CRC-then-COBS composition, decodeBinaryFrame/sendReply), SerialPort/Radio
    (each gained its own FrameKind enum with no dependency on app/, binary-clean accumulation,
    a new Radio::sendText() since the two frame kinds now need different trailing
    delimiters), Telemetry''s own independently-framed secondary-frame path, and wire.h''s
    envelope-size ceiling recompute (186B->240B) via a new gen_messages.py constant.
    Also updated every file explicitly authorized by the dispatch brief to keep src/tests/sim
    green: wire_test_codec.{h,cpp}, TestSupport::FakeTransport, and every C++ harness
    with its own local armor()/armorLine() helper (app_comms_harness.cpp, app_telemetry_harness.cpp,
    app_robot_loop_harness.cpp, config_gate_harness.cpp, move_protocol_harness.cpp,
    fake_transport_harness.cpp), adding a genuine CRC-mismatch fault-injection scenario
    per SUC-002. Also made one narrow, signature-preserving fix to src/sim/sim_harness.h''s
    injectCommand() (tags frames kBinary instead of kText) since src/tests/sim system
    harnesses call it directly. Ran uv run python3 build.py --clean (ARM + host-sim
    both green) then uv run python -m pytest -q twice: src/tests/sim alone is green
    except 5 tests that route through src/host/robot_radio/io/sim_loop.py''s SimLoop;
    the full suite has 81 failures, all traced (via a representative traceback showing
    every ack/telemetry round-trip timing out) to sim_loop.py still de-arming the
    OLD *B<base64> text against firmware that now emits COBS+CRC, plus a second, more
    structural issue: src/sim/sim_ctypes.cpp''s sim_drain_tlm() newline-joins raw
    outbound frames for the Python ctypes boundary, which is incompatible with binary
    content that may embed newlines/control bytes as legitimate COBS+CRC payload --
    that C ABI file is unlisted in either ticket 002 or 003''s file scope.'
  conflict: 'Sprint 123''s own ticket-boundary decision splits this wire-format change
    into ticket 002 (firmware: comms.cpp/transports/wire.h) and ticket 003 (host:
    io/cli.py, io/sim_loop.py, io/serial_conn.py, io/sim_config.py, testgui/transport.py,
    robot/protocol.py) -- explicitly a flag-day cutover with no dual-stack (Migration
    Concerns). Ticket 002''s own acceptance criterion "Full sim suite green" cannot
    be satisfied without also changing src/host/robot_radio/io/sim_loop.py (ticket
    003''s file) and src/sim/sim_ctypes.cpp (a C ABI bridge file listed in NEITHER
    ticket''s scope) -- because src/sim/ is a shared composition root: the SAME TestSupport::FakeTransport/App::Comms
    graph that src/tests/sim''s C++ harnesses compile fresh per test is also compiled
    into the persistent src/sim/build/libfirmware_host.dylib that Python''s SimLoop
    drives via ctypes, and dozens of src/tests/testgui/ pytest files exercise that
    real compiled library end-to-end. The dispatch brief itself anticipated and pre-authorized
    exactly this stop condition ("If you find the suite cannot go green without touching
    production src/host/robot_radio/ decoders ... STOP and report ... I will decide
    whether to merge 002+003"), but the actual blocker is one layer deeper than that
    brief assumed: it is not merely that ticket 003''s Python files need updating,
    but that a THIRD, currently-unowned file (src/sim/sim_ctypes.cpp) also needs a
    binary-safe redesign of its outbound telemetry drain (currently newline-joins
    raw frame text) before ticket 003''s Python fix could even have well-formed bytes
    to decode.'
  surface: internal
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Firmware framer integration (Comms + transports + wire budget)

## Description

Rewire `App::Comms` (`comms.cpp`) and the two transports
(`com/serial_port.*`, `com/radio.*`) from `*B<base64>\r\n` line-armor to a
binary-clean byte stream that demuxes `0x00`-delimited COBS frames from
`\r\n`-terminated text lines (the HELLO/PING rump) on the same channel.
Recompute `wire.h`'s envelope-size budget constants/static_asserts for
COBS+CRC overhead in place of base64's.

## Acceptance Criteria

- [x] `Comms` sends/receives via COBS+CRC instead of base64 armor;
      `sendReply()`/`decodeArmoredLine()`-equivalent call sites keep
      their existing signatures per Impact on Existing Components.
- [x] A synthetic mixed byte stream (interleaved binary frames and text
      lines) demuxes correctly — text lines never misread as partial
      binary frames and vice versa.
- [x] `SerialPort`/`Radio` confirmed binary-clean (no assumption that
      the byte stream is line-buffered ASCII); `Radio`'s existing RAW250
      fragmentation is unchanged (Open Question 2 — whether it also
      needs its own CRC — resolved here, informed by ticket 006's
      fault-injection data if sequencing allows, otherwise flagged
      forward).
- [x] `wire.h`'s `kCommandEnvelopeMaxEncodedSize`/
      `kReplyEnvelopeMaxEncodedSize`/`kTelemetrySecondaryMaxEncodedSize`
      recomputed against real COBS+CRC overhead (not estimated) and
      `static_assert`s pass at the new, larger achievable envelope size.
- [x] Base64 armor path removed (comms.cpp/telemetry.cpp no longer call
      base64Encode()/base64Decode() — the primitive itself is RETAINED in
      wire_runtime, not deleted, per an independent unrelated consumer;
      see Completion Notes).
- [ ] Full sim suite green. **BLOCKED — see Completion Notes / thrown
      exception.** `src/tests/sim` (the domain within this ticket's
      authorized scope) is green modulo 5 tests that route through the
      `src/sim/` ctypes bridge to Python; the FULL `uv run python -m
      pytest -q` suite has 81 failures, all rooted in
      `src/host/robot_radio/io/sim_loop.py` (ticket 003 scope) and
      `src/sim/sim_ctypes.cpp` (unlisted in either ticket) still decoding/
      framing the OLD `*B<base64>` armor against firmware that now emits
      COBS+CRC.

## Implementation Plan

- **Approach:** Single flag-day cutover per Migration Concerns/Open
  Question 5 (confirm with stakeholder before starting). Replace armor
  calls in `Comms` first behind the new primitives from ticket 001, then
  confirm both transports pass raw framed bytes through without
  reinterpreting them as text lines.
- **Files:** `src/firm/app/comms.{h,cpp}`, `src/firm/com/serial_port.{h,cpp}`,
  `src/firm/com/radio.{h,cpp}`, `src/firm/messages/wire.h` (and
  `scripts/gen_messages.py` if the budget constants are generated —
  confirm at ticket start).
- **Testing:** Sim suite (full regression — decoded message CONTENT must
  be unaffected); a new demux test for the mixed binary/text stream.
- **Documentation:** `com/DESIGN.md`, `app/DESIGN.md`, `messages/DESIGN.md`
  updates ride ticket 005 (kept together so the reconciliation ticket
  sees the final, settled implementation rather than an in-progress one).

## Completion Notes

**New frame layout, byte-by-byte.** Outbound (`Comms::sendReply()`,
`Telemetry::emitSecondary()`): 1) `msg::wire::encode()` the schema payload
(`N` bytes) 2) append the 2-byte CRC-16/CCITT-FALSE (little-endian,
`WireRuntime::encodeCrc16()`) computed over those `N` bytes — combined
buffer is `N+2` bytes 3) `WireRuntime::cobsEncode()` the combined buffer —
this is what `Transport::send(data, len)` receives (0x00-free by
construction) 4) the CONCRETE transport (`SerialPort::send()`/`Radio::send()`)
appends exactly one trailing `0x00` delimiter byte on the wire. Inbound
(`Comms::decodeBinaryFrame()`) reverses this exactly: COBS-decode (delimiter
already stripped by the transport's own demux) → split trailing 2 CRC bytes
→ `crcVerify()` → `msg::wire::decode()`. This is CRC-then-COBS (not
COBS-then-append-CRC), matching ticket 001's own completion-notes warning
about preserving the zero-free property.

**Recomputed worst-case armored size vs. the 254-byte ceiling.**
`kEnvelopeBudgetBytes` (the ceiling `wire.h`'s three `static_assert`s check
against, `scripts/gen_messages.py`) is recomputed from 186B to **240B**:
serial's real ceiling is 254 usable TX bytes; new fixed overhead is
CRC(2B) + one COBS code byte (envelopes here are far under the 254-byte
COBS block boundary, so exactly one code byte, not a second block) +
1-byte delimiter = 4B fixed, vs. base64's ~33%+armor-byte expansion.
Solving `E + 4 <= 254` gives `E <= 250`; shipped 240 for a 10-byte margin.
The schema's own actual worst-case values are UNCHANGED (55/185/60B — no
field-shape edit this ticket) — only the ceiling they're asserted against
moved. `App::kFramedMaxBytes` (`comms.h`) — the actual buffer size Comms
builds before handing to `Transport::send()` — is 192B (computed:
`cobsEncodedMaxLength(kMaxEnvelopeBytes+2=187) == 188`, +4B headroom),
comfortably under the 240B schema ceiling plus the 254B hard limit.

**Decode path updated to keep `src/tests/sim` green.** Beyond
`wire_test_codec.{h,cpp}` (`decodeOutboundLine()`/the internal `armor()`
helper, both now CRC-then-COBS), also updated: `TestSupport::FakeTransport`
(`fake_transport.h`) — `Transport::readLine()` now returns `App::FrameKind`
+ explicit `outLen` (demuxes text vs. binary), `send()` takes explicit
`uint8_t*`/`len` (no longer a NUL-terminated C string), gained
`enqueueInboundBinary()` (two overloads: `uint8_t*`/`len` and `std::string`)
alongside the existing text-only `enqueueInbound()`; every C++ harness that
built its own local `armor()`/`armorLine()` helper (`app_comms_harness.cpp`,
`app_telemetry_harness.cpp`, `app_robot_loop_harness.cpp`,
`config_gate_harness.cpp`, `move_protocol_harness.cpp`) switched that
helper to the CRC-then-COBS composition and its call sites to
`enqueueInboundBinary()`; `fake_transport_harness.cpp` rewritten for the
new signatures (one scenario replaced: "armored line survives round trip"
→ "binary frame survives round trip" with non-ASCII bytes, tagged
`kBinary`); two obsolete `app_comms_harness.cpp` scenarios (bad armor
prefix, truncated base64) replaced with unrecognized-text-line +
malformed-COBS + a NEW genuine CRC-mismatch fault-injection scenario
(SUC-002's own acceptance criterion — a bit-flipped, well-COBS-framed
frame is detected via CRC and rejected, not mis-parsed).
`src/sim/sim_harness.h`'s `injectCommand()` (shared by C++ system harnesses
AND, via `sim_ctypes.cpp`, the Python ctypes bridge) was also fixed — it
now tags injected frames `kBinary` via `enqueueInboundBinary()` instead of
`kText` via `enqueueInbound()` (recovering length via `strlen()`, safe
because COBS output is 0x00-free) — a narrow, signature-preserving
correctness fix needed for `config_gate_harness.cpp`/
`move_protocol_harness.cpp`'s own `injectCommand()` calls to keep working,
made without touching Python.

**Base64 primitive: retained, not removed.** `WireRuntime::base64Encode()`/
`base64Decode()` stay in `wire_runtime.{h,cpp}` — `wire_runtime_harness.cpp`'s
base64 test scenario is untouched. Ticket 001's own completion notes
flagged these for removal once "no longer the active armor," but
`src/tests/sim/unit/wire_differential_harness.cpp` independently depends on
them for its own, unrelated debug-CLI wire encoding (a differential-fuzz
tool comparing this codec against a Python protobuf reference) — deleting
the primitive would be a second, out-of-scope breaking change for zero
benefit. Documented at `wire_runtime.h`'s own file header.

**Text-rump demux approach.** `SerialPort`/`Radio` each gained their OWN
nested `FrameKind` enum (no dependency on `app/`, per `com/DESIGN.md`'s
"com/ has no dependency on app/" invariant) and accumulate/reassemble
UNFILTERED bytes (no longer stripping every `\r` on sight — a binary frame
may legitimately carry `0x0D` as content) until either terminator ends a
complete unit: `0x00` → binary (delimiter consumed, never included in the
returned length); `\n` → text (a single trailing `\r` stripped, matching
the pre-123 HELLO/PING convention). `app/comms.h`'s
`SerialTransport`/`RadioTransport` adapters (`comms.cpp`) translate each
transport's own enum to `App::FrameKind` — the one seam allowed to know
both exist. `Radio` gained a new `sendText()` (appends `'\n'`, used only by
`RadioTransport::sendReliable()` for HELLO/PING replies) alongside `send()`
(now binary-only, appends `0x00`) — pre-123 both funneled into one
`Radio::send()`; post-123 the two frame kinds need different trailing
bytes, so the single-send-path collapse no longer holds.

## Exception — thrown, not forced

**`uv run python -m pytest -q` (full suite): 81 failed / 1326 passed / 2
skipped / 10 xfailed / 1 xpassed** (baseline: 1407/2/9/2). Every failure
traces to `robot_radio.io.sim_loop.SimLoop` (`src/host/robot_radio/io/
sim_loop.py`, explicitly a ticket-003 file) still de-arming `*B<base64>`
against firmware that now emits COBS+CRC — confirmed via a representative
traceback (`test_true_pose_advances_after_forward_twist`): "EstimatorConfigPatch
push ... TIMED OUT waiting for ack — 0/9 confirmed applied", i.e. every
ack/telemetry round-trip through the compiled `src/sim/build/
libfirmware_host.dylib` (rebuilt by this ticket's own `build.py --clean`
verify step) silently fails to decode. A SECOND, more structural layer sits
underneath that first one and is NOT listed in either ticket's file scope:
`src/sim/sim_ctypes.cpp`'s `sim_drain_tlm()` newline-joins raw outbound
frames into one string for the Python ctypes boundary, and
`sim_inject_command()`/`SimHarness::injectCommand()` (pre-this-ticket)
relied on NUL-terminated single-line text — both conventions are
incompatible with a COBS+CRC frame that may legitimately embed `\n`/other
control bytes as content (only `0x00` is guaranteed absent). Fixing the
Python decode side (ticket 003) alone would not be sufficient without also
redesigning this C ABI's outbound drain to be explicit-length/binary-safe.

`src/tests/sim` (this ticket's authorized domain) is green except 5 tests
that also route through this same `SimLoop` bridge
(`test_sim_configure_from_robot.py` x2, `test_straight_leg_crab_regression.py`,
`test_motor_primitive.py` x2) — same root cause, not a separate defect.

Per this ticket's own dispatch instructions: "If you find the suite cannot
go green without touching production `src/host/robot_radio/` decoders ...
STOP and report ... I (team-lead) will decide whether to merge 002+003."
That condition is met, with the added wrinkle that `src/sim/sim_ctypes.cpp`
(the C ABI bridge) is unlisted in EITHER ticket 002 or 003's file scope yet
must also change. Stopping here rather than expanding scope unilaterally.
Ticket left in `exception` status; all firmware-side + authorized
sim-test-side work is committed and `src/tests/sim` (minus the 5 bridge-
dependent tests) is green.
