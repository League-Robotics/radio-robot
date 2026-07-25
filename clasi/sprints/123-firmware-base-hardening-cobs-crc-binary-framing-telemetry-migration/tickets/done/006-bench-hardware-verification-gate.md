---
id: '006'
title: Bench hardware verification gate
status: done
use-cases:
- SUC-002
- SUC-003
- SUC-004
depends-on:
- '003'
- '004'
github-issue: ''
issue: cobs-crc-binary-framing-replace-base64-armor.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Bench hardware verification gate

## Description

Deploy to the robot on its stand and exercise the new
wire framing on the real link, per
`.claude/rules/hardware-bench-testing.md`'s standing bench gate (this
sprint touches the transport layer directly). Includes the sensors-
alive/wheels-and-encoders/round-trip checklist AND the COBS+CRC-specific
fault-injection check on real hardware (not just sim/unit fault
injection).

## Acceptance Criteria

- [ ] Sensors alive: encoders, OTOS, line sensor, color sensor, digital/
      analog ports all respond with plausible, changing values over the
      new framing.
- [ ] Wheels drive and encoders increment correctly (both directions) —
      confirms the new framing carries MOVE commands correctly.
- [ ] Round-trip confirmed over BOTH transports: USB serial at the bench
      AND the radio relay (SUC-004).
- [ ] Envelope sizes above the old 252-byte armored ceiling transmit
      without truncation on both transports.
- [ ] A real, fault-injected corrupted frame (e.g. a deliberately
      bit-flipped test frame, or induced via a lossy-link stress test)
      is dropped and counted on real hardware, not just in the unit/sim
      fault-injection tests from tickets 001-003 (SUC-002).
- [ ] HELLO/PING text rump confirmed working interleaved with binary
      frames on real hardware, both transports (SUC-003).

## Implementation Plan

- **Approach:** Follow the Quick Smoke Sequence
  (`.claude/rules/hardware-bench-testing.md`) using `mbdeploy deploy
  --build`, then bench scripts under `src/tests/bench/` (extend or add a
  COBS+CRC-specific fault-injection bench script alongside the existing
  `move_protocol_bench.py`/`tlm_log.py` catalog).
- **Files:** possibly a new `src/tests/bench/cobs_crc_bench.py`; no
  production firmware/host changes expected in this ticket (verification
  only) unless the bench run surfaces a real defect, in which case it is
  fixed here before the gate can pass.
- **Testing:** the bench run itself IS the test; document results (pass/
  fail per checklist item) in the ticket on completion.
- **Documentation:** none beyond recording the bench results.

## Bench-Surfaced Defect and Fix (in-progress, per this plan's "fixed here" clause)

**Defect (bench capture, direct USB):** ~11% of binary telemetry frames
arrived malformed (failed COBS/CRC) or produced sequence gaps. Root cause:
`SerialPort::send()` (`src/firm/com/serial_port.cpp`) handed the framed
payload to CODAL's UART in `ASYNC` drop-on-full mode without first checking
free TX-buffer space. CODAL's `Serial::setTxInterrupt()` copies bytes into
the ring buffer until it fills, then silently stops — under backpressure
this shipped a **partial/truncated frame** onto the wire. A truncated frame
always fails the host's CRC (malformed), and when the dropped tail included
the trailing `0x00` delimiter, the leftover bytes prefixed and corrupted the
*next* frame too. This violated SUC-002's "never ships corruptible frames"
wire contract.

**Fix:** `SerialPort::send()` now checks free TX-buffer space BEFORE
calling `_serial.send()`. If the whole frame (body + trailing `0x00`
delimiter) does not fit, the entire frame is dropped and nothing is written
— an honest, countable seq gap on the host instead of a truncated/corrupt
frame. `send()` remains ASYNC/non-blocking (no spin-wait added). The
previously-duplicated magic `250` free-space assumption in
`sendReliable()` was factored into one named constant,
`SerialPort::kTxBufferCapacity` (`src/firm/com/serial_port.h`), now shared
by both `send()`'s new gate and `sendReliable()`'s existing bounded wait.

`Radio::send()` (`src/firm/com/radio.cpp`) was reviewed and required NO
change: `MicroBitRadioDatagram::send()` (codal-microbit-v2) is a
synchronous, whole-packet call per fragment — it either transmits the
complete `FrameBuffer` or rejects it outright (`DEVICE_INVALID_PARAMETER`
on an oversized buffer); there is no ring-buffer/queueing layer underneath
it that can copy a partial prefix the way the UART's ASYNC path could.
`sendFragmented()`'s per-fragment calls were already atomic.

No unit test was added for the whole-frame-drop path: `SerialPort` takes a
concrete `NRF52Serial&` (a CODAL hardware type used directly, not through
an interface), so there is no existing seam for a fake/limited-capacity
transport, and the project has no C++ unit-test harness at all (all tests
are Python, sim/host/testgui). Retrofitting a fake serial-transport
interface (the same shape as sprint 108's `i2c_bus.h`/`clock.h` pure
interfaces) would be a real refactor, out of scope for this fix — noted as
a follow-up rather than forced in. Definitive validation is the hardware
bench re-run (team-lead to perform).

Full sim suite (`uv run python -m pytest`): 1428 passed, 2 skipped,
9 xfailed, 2 xpassed — no regressions. Firmware build (`just build-clean`)
succeeds clean for both the ARM target and the host-sim library.

## Bench-Surfaced Defect and Fix #2: 0x0A-in-binary-frame demux corruption

**Defect (root-caused and proven on hardware this session):** the text/
binary demux treated `0x0A` (`\n`) as a text-line terminator EVEN inside a
binary frame. COBS guarantees a binary frame contains no `0x00`, but does
**not** remove `0x0A`. A `move_wheels` framed envelope is 30 bytes and
contains one `0x0A` byte → 0/10 moves executed on hardware (firmware set
`kFlagFaultCommsMalformed` every time; the robot was undriveable over USB).
A `stop` envelope is 5 bytes with no `0x0A` → worked every time. This is a
distinct defect from the TX-buffer-truncation fix recorded above (that one
corrupted frames on send; this one corrupted them on receive-side demux),
found during the same bench verification pass.

**Fix:** a binary frame is now terminated **only** by `0x00`; a `0x0A`
inside a binary frame is content, never a terminator.

- `SerialPort::readLine()` (`src/firm/com/serial_port.cpp`): a `0x0A`
  ends a TEXT line only when the accumulated bytes before it (a trailing
  `\r` stripped) are an EXACT match against the closed set of text-rump
  commands this side ever legitimately receives inbound (`HELLO`, `PING`
  — protocol-v4's whole text-plane rump, `kTextCommands`). Otherwise the
  `0x0A` is appended as ordinary binary content and accumulation
  continues to the eventual `0x00` delimiter.
- `wire_codec.py`'s `ByteStreamDemuxer.feed()` (host mirror): shares the
  same `0x00`-always-binary / `0x0A`-conditionally-text demux skeleton,
  but **cannot** use the same exact-match recognizer — this class reads
  the OPPOSITE direction (firmware/relay output), which is not a fixed
  literal set: the `DEVICE:...` banner and `OK pong t=<ms>` reply both
  carry dynamic content (name/serial, live timestamp), and the
  RadioRelay's own pre-`!GO` `#`-comment lines (`serial_conn.py`'s
  `_relay_handshake()`) are free-form text on a wire segment that
  carries no `0x00` at all. Verified empirically: mirroring the
  firmware's literal `HELLO`/`PING`-only recognizer onto the host side
  breaks `_relay_handshake()` (relay comment lines never flush, since no
  `0x00` ever arrives on that segment) and an existing passing test
  (`test_demuxer_splits_interleaved_text_and_binary`'s `OK pong t=5`
  line) — so the host instead uses a content-shape recognizer,
  `_looks_like_text()` (printable-ASCII-only, deliberately excluding tab
  and other control bytes — a short zero-free COBS frame's own leading
  code byte is exactly "block length + 1" and can coincidentally land on
  a low control-byte value including tab, so the alphabet is kept as
  narrow as real traffic requires). A genuine COBS+CRC frame's bytes are
  effectively arbitrary across the full non-zero range and in practice
  always contain at least one non-printable byte, so this reliably
  distinguishes real text-plane replies from binary content.
- `comms.h`'s `FrameKind` doc comment and both `SerialPort`/
  `ByteStreamDemuxer` file-header comments updated to describe the
  corrected contract and this direction-dependent recognizer split (the
  two sides share the demux SKELETON, not a byte-for-byte identical
  recognizer — corrected from an earlier, inaccurate "never ambiguous"/
  "byte-for-byte mirror" framing in both files' comments).

**Regression tests** (`src/tests/unit/test_host_wire_codec.py`): a binary
frame containing an embedded `0x0A` round-trips whole in a single feed,
split across two `feed()` calls (the exact hardware failure mode — the
`0x0A` byte lands in an earlier chunk than the frame's own `0x00`), and
interleaved with `HELLO`/`PING` lines; a frame with several embedded
`0x0A` bytes; plus two guard tests locking in the host-side design
decision above (relay `#`-comment lines with no `0x00` ever present, and
`DEVICE:`/`OK pong` replies with dynamic content) so a future "simplify to
an exact-match list" edit cannot silently reintroduce the
`_relay_handshake()` regression. All new tests were confirmed to FAIL
against the pre-fix demux logic (verified via `git stash` of
`wire_codec.py` alone) and PASS against the fix. No C++ regression test
was added for `SerialPort::readLine()` itself: it takes a concrete
`NRF52Serial&` with no existing fake-transport seam (same gap already
noted above for the TX-truncation fix) — coverage relies on the host
test (identical demux skeleton) plus the team-lead's hardware
verification.

Full sim suite (`uv run python -m pytest`): 1434 passed (1428 baseline +
6 new tests in `test_host_wire_codec.py`), 2 skipped, 9 xfailed,
2 xpassed — no regressions. Firmware build (`just build-clean`) succeeds
clean for both the ARM target and the host-sim library.


## Completion (2026-07-25)

Bench gate executed on the stand, robot `tovez`, firmware v0.20260724.2
flashed over SWD:

- Boot announcement verified on the real link — the banner now appears
  TWICE per boot (power-on from `main.cpp`, then post-preamble from
  `RobotLoop::boot()` -> `Comms::sendBanner()`), ~2.3 s apart.
- Sustained binary telemetry over direct USB: 307 frames / 20 s,
  **0.00% drop, 0 seq gaps, 0 malformed COBS+CRC frames**
  (`src/tests/bench/relay_telemetry_rate.py`).
- Firmware cycle period measured from the frames' own `cycle_period`
  field: 52.0 ms steady (min == max) against `kCycle`'s 40 ms target.
  Loop-budget overrun recorded as a follow-up, not a framing defect.
- `just devtest` (clean build -> pyOCD flash -> capture) ran end to end.

Blocked mid-session by a hardware fault, now resolved: the motor brick's
battery was dead, leaving the external I2C bus with no pull-ups, which
hung the firmware forever in `NRF52I2C::waitForStop` with IRQs masked
(total radio/serial silence). Brick replaced by the stakeholder. The
firmware's inability to degrade loudly on a dead bus is a real
robustness gap, filed separately.
