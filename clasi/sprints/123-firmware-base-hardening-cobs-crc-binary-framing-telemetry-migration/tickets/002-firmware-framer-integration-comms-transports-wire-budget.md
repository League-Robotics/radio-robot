---
id: '002'
title: Firmware framer integration (Comms + transports + wire budget)
status: open
use-cases: [SUC-001, SUC-002, SUC-004]
depends-on: ['001']
github-issue: ''
issue: cobs-crc-binary-framing-replace-base64-armor.md
completes_issue: true
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

- [ ] `Comms` sends/receives via COBS+CRC instead of base64 armor;
      `sendReply()`/`decodeArmoredLine()`-equivalent call sites keep
      their existing signatures per Impact on Existing Components.
- [ ] A synthetic mixed byte stream (interleaved binary frames and text
      lines) demuxes correctly — text lines never misread as partial
      binary frames and vice versa.
- [ ] `SerialPort`/`Radio` confirmed binary-clean (no assumption that
      the byte stream is line-buffered ASCII); `Radio`'s existing RAW250
      fragmentation is unchanged (Open Question 2 — whether it also
      needs its own CRC — resolved here, informed by ticket 006's
      fault-injection data if sequencing allows, otherwise flagged
      forward).
- [ ] `wire.h`'s `kCommandEnvelopeMaxEncodedSize`/
      `kReplyEnvelopeMaxEncodedSize`/`kTelemetrySecondaryMaxEncodedSize`
      recomputed against real COBS+CRC overhead (not estimated) and
      `static_assert`s pass at the new, larger achievable envelope size.
- [ ] Base64 armor path removed once the cutover is confirmed working
      (no dual-stack left behind — Migration Concerns' flag-day
      cutover).
- [ ] Full sim suite green.

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
