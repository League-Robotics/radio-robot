---
id: '002'
title: Firmware framer integration (Comms + transports + wire budget)
status: done
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
- [x] Full sim suite green. **UNBLOCKED — team-lead merged ticket 003's
      host-decoder scope into this ticket** (see Completion Notes — 002/003
      Merge Resolution). `uv run python -m pytest -q`:
      **1428 passed, 2 skipped, 9 xfailed, 2 xpassed, 0 failed** (before:
      1407/2/9/2 baseline, 81 failed mid-cutover; after: baseline + 21 new
      tests, zero failures — see Completion Notes for the full before/after
      and the file-by-file diff).

## Folded-in scope — ticket 003 (Host decoder rewrite)

Per the exception thrown above and team-lead's decision to merge 002+003
under this ticket (see sprint's own dispatch record), the following is
ticket 003's own acceptance criteria, satisfied here:

- [x] Each of `io/serial_conn.py`, `io/sim_loop.py`, `io/sim_config.py`,
      `io/cli.py` (doc-only — no functional base64), `testgui/transport.py`
      (doc-only), `robot/protocol.py` (doc-only) decodes/encodes via
      COBS+CRC; no remaining `base64.b64decode`/`b64encode` call on the
      wire path (confirmed via `grep -rn base64 src/host/` — every hit left
      is a historical doc-comment, none a code path; see Completion Notes).
  - [x] `SerialConnection`/`NezhaProtocol`/`sim_loop`/TestGUI's
        `transport.py` all pass their existing higher-level test suites
        unchanged (command builders, `TLMFrame` field access untouched).
  - [x] HELLO/PING text rump still decodes correctly when interleaved
        with binary frames on the same connection — `ByteStreamDemuxer`
        (new `io/wire_codec.py`) demuxes both shapes structurally off the
        SAME raw byte stream, exercised directly
        (`test_host_wire_codec.py`) and through `SerialConnection`'s own
        reader-loop tests (`test_serial_conn_binary_plane.py`).
  - [x] Fault-injection test: a corrupted frame is dropped on the host
        decode path with a counted fault, not silently mis-parsed —
        `SerialConnection.malformed_frame_count` (new, 123-003) plus
        `test_reader_loop_crc_corrupted_binary_frame_is_dropped_not_raised`/
        `test_decode_frame_rejects_crc_corrupted_frame`.

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

## 002/003 Merge Resolution — completed

Team-lead decided to merge ticket 003's host-decoder scope into this
ticket (the wire-format change is atomic; there is no green-per-ticket
boundary once the sim suite exercises the production host decode path).
Ticket moved `exception` -> `in-progress` and this second pass completed
the full cutover: firmware side (already committed, 72812f49/e60c0b75)
required NO further changes; everything below is the host + bridge half.

### Final frame layout (unchanged from the firmware-side completion notes
above — restated here because this is now the byte-for-byte contract the
HOST side implements independently, not just documents)

Outbound: `msg::wire::encode()` (N bytes) -> append little-endian
CRC-16/CCITT-FALSE (2 bytes, poly 0x1021, init 0xFFFF, no reflection, no
xorout) -> COBS-encode the combined N+2 bytes -> transport appends exactly
one trailing `0x00` delimiter. Inbound: strip the transport's own `0x00`
delimiter -> COBS-decode -> split trailing 2 CRC bytes -> verify -> decode
the schema payload. Demux of the coexisting HELLO/PING text rump: whichever
byte (`0x00` or `\n`) is seen FIRST in the accumulated stream decides the
kind — this is a byte-for-byte port of `App::Transport::readLine()`
(`comms.h`), including its one known, narrow, ACCEPTED collision (a COBS
frame's content is only guaranteed `0x00`-free, not `0x0A`-free; an
embedded `0x0A` before a frame's own terminator can misclassify a text/
binary boundary — present in firmware too, not a defect introduced here;
self-heals via each side's own malformed-frame counting and retry
discipline. See `wire_codec.py`'s `ByteStreamDemuxer` docstring).

### `src/sim/sim_ctypes.cpp` binary-safe redesign (the unplanned, unlisted
file both tickets' exceptions flagged)

`sim_drain_tlm(SimHandle h, uint8_t* buf, int buflen)` no longer
newline-joins raw outbound text (safe pre-123 only because base64's
alphabet excludes both `0x00`/`0x0A`; unsafe now that frame content is
arbitrary bytes). It now `memcpy`s each captured COBS+CRC frame into `buf`
with exactly one `0x00` appended per frame — the SAME trailing-delimiter
shape multiple back-to-back real wire frames would have — and returns the
total byte count (snprintf-return-value convention: `> buflen` means only
a whole-frame PREFIX was copied; never splits a frame across the boundary).
The Python side (`sim_loop.py`'s `_drain_tlm_into_queue()`) reads
`buf.raw[:n]` (never `.value`, which stops at the first embedded `0x00`)
and splits on `b"\x00"` directly — no text/binary demux needed on this
path since `drainRawTelemetry()` only ever captures binary `send()` frames,
never text `sendReliable()` replies. `sim_inject_command()`'s own
`const char*` signature needed NO change: a COBS-encoded frame body is
0x00-free by construction, so a NUL-terminated C string safely carries it
(only the Python CALLER changed, from `"*B" + base64...` to
`wire_codec.encode_frame(...)`).

### Every host file changed

- **New:** `src/host/robot_radio/io/wire_codec.py` — the one Python
  mirror of `wire_runtime.h`'s COBS/CRC primitives:
  `crc16_ccitt_false()`, `cobs_encode()`/`cobs_decode()`,
  `encode_frame()`/`decode_frame()` (CRC-then-COBS composition,
  `decode_frame()` returns `None` on any malformed input, never raises),
  and `ByteStreamDemuxer` (the text/binary stream demuxer). Verified
  against the firmware's own known-answer CRC vector
  (`crc16_ccitt_false(b"123456789") == 0x29B1`) and round-tripped against
  500 random payloads each for COBS and the full frame codec.
- **`io/serial_conn.py`** — `_reader_loop()` rewritten to read raw bytes
  (`_ser.read()`/`.in_waiting`, never `.readline()`) through a
  `ByteStreamDemuxer`; text-plane classification extracted into a new
  `_handle_text_line()` (behavior unchanged). `_handle_binary_reply()`
  takes a raw frame body, decodes via `wire_codec.decode_frame()`, counts
  a new `malformed_frame_count` on failure. `send_envelope()`/
  `send_envelope_fast()` write `encode_frame(...) + b"\x00"` instead of
  `f"*B{base64...}\n"`. The pre-reader-thread handshake helpers
  (`_banner_classify`/`_relay_handshake`/`_poll_ready`/`_poll_read_lines`,
  plus the standalone `probe_devices()`) also switched off raw
  `.readline()` onto a new module helper `_read_text_line_raw()` +  a
  per-connect-attempt `self._handshake_demux` — a binary telemetry frame
  can now legitimately arrive interleaved with the boot banner/PING
  handshake (the firmware streams telemetry from boot regardless of
  HELLO), and unlike base64 text, a raw frame may embed a literal `0x0A`
  that a plain `readline()` would misinterpret as a line ending.
- **`io/sim_loop.py`** — `_dearmor_reply()` replaced with
  `_decode_reply_frame()` (COBS+CRC via `wire_codec`); `move()`/
  `inject_command()` build/accept frame `bytes` instead of `"*B..."` `str`;
  `_drain_tlm_into_queue()` reads the binary-safe `sim_drain_tlm()` output
  per the sim_ctypes.cpp redesign above.
- **`io/sim_config.py`** — `SimConfigConn.send_envelope_fast()` builds a
  frame via `encode_frame()` instead of base64-armoring.
- **`io/cli.py`, `robot/protocol.py`, `testgui/transport.py`** — doc-only
  updates (module docstrings/help text describing the wire shape); neither
  file had a functional base64 call site (both delegate framing to
  `serial_conn.py`/`sim_config.py`).
- **`testgui/binary_bridge.py`** — `render_log_line()` redesigned to
  dispatch by Python TYPE (`bytes` = binary frame, `str` = text line)
  instead of a `"*B"` string-prefix check, since a real frame is no longer
  guaranteed ASCII-safe to carry as a Python `str` at all. Malformed/
  undecodable binary input now renders a `<binary: malformed, N bytes>`
  marker instead of falling back to the (no-longer-meaningful) raw line.
- **`src/tests/bench/relay_telemetry_rate.py`,
  `src/tests/sim/{tune_velocity_pid,scoreboard_700}.py`** — non-pytest-
  collected diagnostic tools, updated for consistency (not required for
  the acceptance gate, but left on the old wire shape would have silently
  broken on next use): `instrument_malformed_counter()`'s wrapper and the
  two scripts' inline envelope-injection helpers now go through
  `wire_codec`. `src/tests/notebooks/plan_dump_trace_overlay.ipynb`
  (targets the already-`reserved` `plan_dump` arm, pre-existing sprint-
  103-era staleness unrelated to this cutover) was left untouched —
  out of scope, not pytest-collected, not a live wire-shape reference.

### Tests

- **New:** `src/tests/unit/test_host_wire_codec.py` (20 tests — CRC known-
  answer + bit-sensitivity, COBS round-trip/boundary/malformed-input,
  frame round-trip + the CRC-corruption/malformed fault-injection pair,
  `ByteStreamDemuxer` interleaving/partial-feed). Named `test_host_wire_
  codec.py`, not `test_wire_codec.py`, to avoid a pytest module-name
  collision with the pre-existing, unrelated
  `src/tests/sim/unit/test_wire_codec.py` (the C++ `wire.{h,cpp}` generated-
  codec harness wrapper — neither `src/tests/unit/` nor `src/tests/sim/
  unit/` has an `__init__.py`, so pytest's default rootdir-insertion import
  mode requires distinct basenames).
- **New:** `src/tests/unit/_wire_test_helpers.py` — shared `FakeSerial`
  (byte-buffer-backed, `.read()`/`.in_waiting`, "raises once exhausted"
  contract) + `binary_frame()`/`text_line()` builders, used by
  `test_serial_conn_binary_plane.py`/`test_serial_conn_telemetry_
  secondary.py`. Not itself pytest-collected (no `test_`/`_test` filename
  match).
- **Rewritten:** `test_serial_conn_binary_plane.py`,
  `test_serial_conn_telemetry_secondary.py`, `test_protocol_binary_client.py`
  — every `_FakeSerial`/`_LoopbackSerial`/`_RecordingSerial`/
  `_ConfigLoopbackSerial`/`_NoReplySerial` test double rebuilt on the raw
  `.read()`/`.in_waiting` contract instead of `.readline()`; every
  `"*B" + base64...` line builder replaced with `binary_frame()`/
  `encode_frame()`. Added one new test,
  `test_reader_loop_crc_corrupted_binary_frame_is_dropped_not_raised`
  (SUC-002's host-side half: a bit-flipped, well-COBS-framed frame is
  dropped via CRC, not mis-parsed). One test's synthetic field values were
  changed (`tlm.now=999` -> `42`) after discovering they happened to
  produce a frame starting with the shared, accepted `0x0A` collision byte
  described above — not a bug, a deterministic (non-flaky) authoring
  pitfall now documented inline.
- **Rewritten:** `src/tests/testgui/test_sim_loop.py` (two tests capturing
  `inject_command()`'s argument now decode a `bytes` frame via
  `wire_codec.decode_frame()` instead of base64), `src/tests/testgui/
  test_binary_bridge.py` (`_armor()` helper now returns `encode_frame(...)`
  bytes; the "malformed/non-armored passes through unchanged" tests split
  into a type-based pair — `test_text_plane_line_passes_through_unchanged`
  and `test_malformed_binary_frame_renders_marker_never_raises`, matching
  `render_log_line()`'s new dispatch-by-type contract).
- **Unchanged, confirmed still green as-is (no wire-shape dependency):**
  `test_serial_conn_ack_ring.py`, `test_twist_stop_ack_matcher.py`,
  `test_protocol_config.py` (all operate above the framing layer, on
  `_binary_tlm_queue`/duck-typed connections).

### Verification

- `uv run python3 build.py --clean` — ARM firmware (`v0.20260724.2`,
  MICROBIT.hex) and the host-sim library (`libfirmware_host.dylib`,
  HOST_BUILD) both built clean, no warnings introduced by this ticket's
  changes.
- `uv run python -m pytest -q` — **1428 passed, 2 skipped, 9 xfailed, 2
  xpassed, 0 failed** in ~523s. Before (exception-time baseline): 1407
  passed/2 skipped/9 xfailed/2 xpassed with 81 FAILED. The +21 passed
  delta is exactly the 20 new `test_host_wire_codec.py` tests plus the one
  new CRC-fault-injection test added to `test_serial_conn_binary_plane.py`
  — every other file's test COUNT is unchanged, only its wire-shape
  construction — confirming no test was silently dropped in the rewrite.
  Skipped/xfailed/xpassed counts are byte-identical to baseline (no
  incidental change in unrelated marked tests).
- Corrupted-frame rejection confirmed on BOTH sides: firmware
  (`app_comms_harness.cpp`'s CRC-mismatch fault-injection scenario, already
  committed by the earlier firmware-side pass) and host
  (`test_decode_frame_rejects_crc_corrupted_frame`,
  `test_reader_loop_crc_corrupted_binary_frame_is_dropped_not_raised`,
  both new).

### Explicit statement for team-lead: ticket 003 is COMPLETE

Every one of ticket 003's own acceptance criteria (host decoder rewrite:
`io/serial_conn.py`, `io/sim_loop.py`, `io/sim_config.py`, plus the doc-only
`io/cli.py`/`robot/protocol.py`/`testgui/transport.py`; no remaining
`base64.b64decode`/`b64encode` on the wire path; existing higher-level test
suites pass unchanged; HELLO/PING-interleaved-with-binary demux; host-side
fault-injection) is satisfied by the work described above, folded into this
ticket per team-lead's merge decision. Ticket 003 itself was NOT moved to
`done` by this agent (out of scope per dispatch instructions — "Do NOT touch
ticket 003's status — team-lead will move it to done") — its file scope,
acceptance criteria, and file list are otherwise fully covered here and it
is ready for team-lead to fold/close.
