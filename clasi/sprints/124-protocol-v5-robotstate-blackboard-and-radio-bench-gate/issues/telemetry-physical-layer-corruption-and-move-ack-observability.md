---
sprint: '124'
status: in-progress
tickets:
- 124-006
- 124-011
- 124-012
- 124-013
---
# Telemetry physical-layer corruption + move-enqueue ack observability

**Filed:** 2026-07-25 (sprint 123 overnight bench work, ticket 006 follow-up)
**Robot:** tovez, firmware v0.20260724.2 (+ commit 9b4ea538), direct USB
`/dev/cu.usbmodem2121102`. Relay dongle was NOT connected during this work.

## ★ CONFIRMED ROOT CAUSE (added after full diagnosis): the 0x0A demux bug

The dominant, fixable cause of the "frame drops" is NOT physical corruption —
it is a **framing bug in the 123 cutover, present on BOTH sides**:

- Firmware `SerialPort::readLine()` (`src/firm/com/serial_port.cpp:19-53`) and
  host `ByteStreamDemuxer.feed()` (`src/host/robot_radio/io/wire_codec.py:242-268`)
  both split the stream on **whichever of `0x00` or `0x0A` (`\n`) comes first**.
- COBS guarantees a binary frame contains no `0x00`, but does **NOT** remove
  `0x0A`. So **any binary frame that embeds a `0x0A` byte is split at it** and
  corrupted (the piece before → mis-delivered as "text"/dropped, the piece
  after → fails CRC). The "never ambiguous" claim in both code comments is
  wrong.
- **Proven:** a `move_wheels` framed envelope is 30 bytes and contains one
  `0x0A` → fails 100% (`0/10` moves activate; `kFlagFaultCommsMalformed` sets on
  every send) → **robot is undriveable over USB**. A `stop` framed envelope is
  5 bytes with no `0x0A` → works every time. This asymmetry is the fingerprint.
- Also explains much of the ~5% baseline TELEMETRY "corruption" below:
  telemetry frames that happen to contain a `0x0A` are split by the host
  demuxer the same way. (The separate multi-second 100%-fail episodes are NOT
  this — see §1.)
- Sim never caught it: the `sim_ctypes` bridge (123-002) doesn't exercise the
  byte-level `readLine`/demux path.

**Fix (in scope for sprint 123 — this is the cutover's own defect):** the demux
must treat `0x0A` as a terminator ONLY for the text-plane rump (HELLO/PING),
never inside a binary frame. Options: recognize the two known text commands by
content, or gate text-vs-binary by a handshake/mode rather than by
first-terminator-wins. **Firmware `readLine()` and host `ByteStreamDemuxer`
MUST be fixed together and kept byte-for-byte mirrors** (they document each
other as such). Add a regression test with a binary frame containing `0x0A`.

---

Original framing of the two residual problems (the physical-layer §1 is real
and separate; §2 is now explained by the root cause above):

## 1. Physical-layer telemetry byte corruption (~5–11%, CRC-caught)

After fixing the `SerialPort::send()` partial-frame truncation bug (commit
9b4ea538, which took steady-state 20 s USB drop from ~11% to ~1.3%), a residual
corruption remains on direct USB, proven to be REAL wire corruption (not the
capture harness) by `wire_truth.py` (single-threaded raw pyserial + demux +
decode, no queue/threads):

- Steady baseline ~5% of primary frames fail COBS/CRC.
- PLUS irregular multi-second episodes (up to ~16 s observed) where **100% of
  frames fail CRC** while the **byte rate stays constant (~1300 B/s)** and the
  delimiter rate stays normal (~19/s).
- Constant bytes + intact framing + 100% content corruption ⇒ **byte-level bit
  corruption with framing intact** — i.e. the nRF UART ↔ KL27 DAPLink ↔ USB CDC
  physical path, NOT truncation, NOT queue overflow, NOT the host.
- CODAL verified: `Serial::setTxInterrupt` copies into the ring (no stack-buffer
  reuse); `Serial::send` `txInUse()` early-returns cleanly. No firmware path
  produces content corruption post-fix.

**Crucially `unparseable=0` on every run** — nothing ever mis-parses; COBS+CRC
detects and drops 100% of corruption. This is **SUC-002 validated on real
hardware by the link itself**. The cutover REVEALED pre-existing physical
corruption the old base64 armor hid/mis-handled; it did not create it.

**Next steps (need hardware / not a framing bug):**
- Compare the radio-relay path (different physical link) — plug in the relay
  dongle and run `wire_truth.py`-equivalent through the `!GO` data plane.
- Try a different USB port/cable/hub; test another micro:bit.
- Correlate corruption episodes with robot I2C/CPU activity (OTOS bursts, flash
  writes) and with the ~5 Hz secondary-telemetry emits.
- Consider lower UART baud or reduced telemetry volume if link is marginal.

## 2. CRITICAL: inbound MOVE commands systematically fail decode (robot undriveable over USB)

**Upgraded from "ack observability" to a critical command-reception bug after
direct hardware testing (scratchpad `move_reliability.py`, `fault_dump.py`).**

On direct USB, `move_wheels`/`move_twist` commands are **never received**: over
10 consecutive moves, **0/10 activated and 0/10 acked**. Sending a move flips
the telemetry `kFlagFaultCommsMalformed` bit False→True — i.e. the firmware's
inbound COBS/CRC decode **rejects the move frame**, so `handleMove()` never
runs (mode stays IDLE, `active` stays false, encoders stay (0,0), no ack). STOP
commands (tiny `CommandEnvelope{stop:Stop{}}` envelope) DO get through and ack.

**This is systematic (100%), not the physical link corruption** in §1: at the
~5% per-frame link rate a smaller move frame would fail <5% of the time, not
100%. It is a real regression from the 123 COBS+CRC cutover — the robot cannot
be driven over USB at all. Sim did NOT catch it (the `sim_ctypes` bridge,
123-002, passes envelopes without exercising the real COBS+CRC wire framing).
The bench gate (ticket 006) is exactly what surfaced it — validating Eric's
insistence on the standing bench gate.

**Asymmetry clue:** STOP (tiny) decodes fine, MOVE (larger, ~30-40 B) fails
100% → suspect an inbound wire-decode issue that's size- or content-dependent:
inbound COBS-decode / CRC / demux buffer sizing in the command RX path
(`App::Comms` pump → `App::Transport::readLine()` demux → `wire_runtime`
COBS/CRC decode → `processMessage()` dispatch), or a text/binary demux
misclassification of a binary move frame (the demux must split `\r\n` text from
`0x00`-delimited binary; a move frame's bytes may trip it). Localize by
decoding what the host actually sends for a move vs. what the firmware's RX
demux/decoder accepts. **This should be fixed within sprint 123** (it is the
cutover's own defect) before ticket 006 can pass.

### (historical framing) move/config enqueue ack observability (8/43)

`src/tests/bench/move_protocol_bench.py` scores **8/43, reproducibly** (identical
across two runs — deterministic, so NOT the intermittent corruption above).
Pattern: `move_wheels`/`move_twist`/`config` **enqueue** acks return `ack=None`,
while **STOP** acks are observed fine (real `AckEntry` returned) and motion
executes (moves run, stop, empty-queue drain all behave). So the command plane
works and the ack *mechanism* works for STOP — but move/config enqueue acks are
not observed via `wait_for_ack()`.

Sharpened by direct raw-frame inspection (`scratchpad/ack_probe.py`): after a
single `move_wheels` (returned corr_id=1) on a fresh connection, over 45 clean
telemetry frames in 3 s (no corruption episode) the firmware emitted **NO ack
at all** for it: the `acks[]` ring stayed **empty**, `ack_fresh` was **never**
set, and `ack_corr` stayed at its default **0** (never 1). Yet STOP acks are
observed and moves execute (motors run). So `handleMove()`'s
`tlm_.ack(result.corrId, ...)` (robot_loop.cpp:279) is either receiving
`env.corr_id == 0` (host `send_envelope_fast()` not writing the corr_id it
returns into the envelope) or the enqueue-ack is not reaching the primary
frame. Localized further: `move_wheels()`, `move_twist()`, and `stop()` all send via
the IDENTICAL `SerialConnection.send_envelope_fast(envelope)` path (same
`_corr_counter` corr_id assignment) — and STOP acks are observed while MOVE
acks are not. So the host send/corr-id path is NOT the difference; the defect
is firmware-side: either `RobotLoop::processMessage()` dispatch of the `move`
oneof arm, or `handleMove()`/`MoveQueue::enqueue()`'s ack emission
(robot_loop.cpp:277-279 calls `tlm_.ack(result.corrId, result.err)` — verify
it runs and that `result.corrId` is nonzero for a normal enqueue). Needs
firmware-side tracing (e.g. temporary counter/telemetry, or gdb) to see whether
`handleMove` runs and whether `tlm_.ack()` pushes to the ring for a move.
Deterministic — not corruption-related. NOTE: also confirm the move actually
executes (motors spin) in the no-ack case — ack_probe did not verify motion.

## Not blocked, for reference
- COBS+CRC framing correctness: PROVEN good (unparseable=0).
- `SerialPort::send()` truncation: FIXED (9b4ea538).
- Bench script `relay_telemetry_rate.py`: fixed to the P4 always-on-push model
  this session (was calling P4-pruned `ping()`/`stream()`); it is accurate.
- Scratchpad probes this session: `wire_truth.py` (authoritative wire quality),
  `raw_capture.py` (byte/delimiter flow).
