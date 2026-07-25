---
id: '006'
title: Bench hardware verification gate
status: in-progress
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
