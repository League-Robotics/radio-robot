---
status: pending
filed: 2026-07-24
filed_by: team-lead (stakeholder question, sprint 122)
related:
- firmware-base-hardening-bounded-wheel-moves-and-wheel-observer.md
- telemetry-report-loop-cycle-duration.md
sprint: '123'
---

# Replace base64 line-armor with COBS + CRC binary framing

## Problem (surfaced during sprint 122, stakeholder question 2026-07-24)

The wire framing today is `*B` + `base64(protobuf envelope)` + `\r\n`,
line-framed so the host can `readline()` it (`Comms::sendReply()` /
`decodeArmoredLine()`, `src/firm/app/comms.cpp`; host decoders in
`src/host/robot_radio/io/{cli,sim_loop}.py`, `testgui/transport.py`,
`robot/protocol.py`). Two costs:

1. **Base64 expands payload by 33%.** A 185 B envelope becomes 248 B of
   base64 → 250 B armored → 252 B with `\r\n`. The USB-CDC/serial path
   has a HARD ceiling: `SerialPort::begin()` calls CODAL
   `setTxBufferSize(255)` — 255 is the `uint8_t` type maximum (not a
   tunable), giving 254 usable bytes, and ASYNC mode **silently truncates
   mid-line** on overflow. So the primary telemetry frame already sits 2
   bytes under a real vendor ceiling, and the "186-byte envelope budget"
   (`wire.h` `kReplyEnvelopeMaxEncodedSize` static_asserts) is really
   "whatever base64-expands to just under 254." This is why sprint 122's
   loop-timing fields could not go on the per-cycle primary frame and
   landed on `TelemetrySecondary` as an interim (see
   `telemetry-report-loop-cycle-duration.md`).
2. **There is no integrity check anywhere on the wire** — no CRC, no
   checksum. Base64 detects nothing. On the documented-lossy radio relay,
   a corrupted envelope either fails protobuf parse and vanishes silently
   or mis-parses. This is a real gap for a base whose contract is "never
   lies."

## Fix

Replace the base64 line-armor with **COBS framing + a CRC**:

- **COBS** (Consistent Overhead Byte Stuffing) delimits frames with a
  single reserved `0x00` byte at ~0.4% overhead (≈1 byte/frame) instead of
  33%. This frees ~60 B on the serial path — the envelope-budget crunch
  disappears, and loop-timing (and future telemetry) fits on the per-cycle
  primary frame with room to spare.
- **CRC** (e.g. CRC-16/CCITT or CRC-32) gives the wire the integrity it
  has never had; corrupt frames are detected and dropped, not mis-parsed.

## The one real design constraint (do not rediscover painfully)

Base64 was chosen for **text-channel coexistence**: the HELLO/PING **text
safety rump** shares the same channel — plain `\r\n` text lines interleaved
with `*B` binary lines, both `readline()`-parseable, printable-ASCII so no
delimiter collision. COBS delimits with `0x00` (binary), so the host reader
AND the relay must demux `0x00`-delimited binary frames from the text rump
and be fully binary-clean. Solvable (text lines carry no `0x00`; COBS
frames never contain `0x00` except the delimiter), but it is the actual
work: a framer that recognizes both a `0x00`-delimited binary frame and a
`\r\n` text line on the same byte stream.

## Scope (a base-layer wire change — touches both sides + relay)

- **Firmware:** `src/firm/app/comms.cpp` (armor/dearmor → COBS encode/decode
  + CRC), `src/firm/com/serial_port.*` (line-send → framed byte-stream send),
  `src/firm/com/radio.*` (already length-prefixed datagrams with reassembly
  — decide whether CRC is added there too or the datagram framing suffices),
  `src/firm/messages/wire.*` (the envelope-size budget asserts get recomputed
  against COBS+CRC overhead instead of base64), and the HELLO/PING text-rump
  coexistence.
- **Host:** every decoder — `io/cli.py`, `io/sim_loop.py`, `io/serial_conn.py`,
  `testgui/transport.py`, `robot/protocol.py` — plus wherever `*B<base64>` is
  produced/consumed (`io/sim_config.py`).
- **Relay:** confirm the radio-relay path is binary-clean and passes COBS
  frames + CRC intact.
- **Docs:** `docs/protocol-v4.md` §2 (framing), `wire.h` budget comments.

## Acceptance

- Serial + radio-relay round-trips carry envelopes larger than today's
  252-byte armored ceiling without truncation; a corrupted frame is
  detected by CRC and dropped (add a fault-injection test).
- The `kReplyEnvelope`/`kTelemetrySecondary` size asserts recomputed for
  COBS+CRC overhead; primary-frame headroom restored.
- Text rump (HELLO/PING) still works interleaved with binary frames on the
  same channel, both transports.
- **Bench-verified on the real link** (serial at the bench + radio relay) —
  this changes the actual wire, so the standing bench gate applies.
- Full sim suite green; host + TestGUI decode the new framing.

## Follow-on

Once this lands, **migrate `cycle_busy`/`cycle_period` from
`TelemetrySecondary` back to the per-cycle primary `Telemetry` frame** (the
interim placement from sprint 122's ticket 003), restoring true per-cycle
loop-timing visibility.

## Sequencing

Base-hardening work (the base owns the wire; "never lies, never hides
latency, never ships corruptible frames"). Belongs in the firmware-base-
hardening plan (sprint 2), a strong candidate for an early/leadoff ticket
since it unblocks per-cycle telemetry and adds integrity the whole base
relies on.
