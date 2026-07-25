---
id: '001'
title: COBS + CRC wire-runtime primitives
status: done
use-cases:
- SUC-001
- SUC-002
depends-on: []
github-issue: ''
issue: cobs-crc-binary-framing-replace-base64-armor.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# COBS + CRC wire-runtime primitives

## Description

Add COBS frame encode/decode and CRC compute/verify as
new, schema-agnostic byte primitives in `src/firm/messages/wire_runtime.{h,cpp}`
— the same layer that already holds varint/zigzag/fixed32/base64 today.
Decide the CRC width (Open Question 1: recommend CRC-16/CCITT) based on
measured overhead vs. detection strength for this frame size range.

## Acceptance Criteria

- [x] `wire_runtime.h`/`.cpp` gain `cobsEncode()`/`cobsDecode()` and
      `crcCompute()`/`crcVerify()` (or equivalent names following the
      project's lowerCamelCase convention), with the same
      never-partially-write-or-read contract every other `wire_runtime`
      primitive already has (`.claude/rules/coding-standards.md`; no
      `#include` of any other `messages/*.h`, no naming of a `msg::`
      type).
  - [x] COBS round-trip test: encode then decode recovers the original
        payload exactly, including payloads containing `0x00` bytes and
        payloads at/near the COBS block-size boundary (254 bytes).
  - [x] CRC test: detects single-bit flip, byte truncation, and
        over-length corruption; a clean frame passes.
  - [x] CRC width decision recorded with its rationale (measured
        overhead vs. detection strength), not left as an assumption.
  - [x] Existing `wire_runtime` tests (varint/zigzag/base64 — base64
        stays present for now, removed in ticket 002 once no longer the
        active armor) remain green.

## Completion Notes

- **CRC variant (pinned, byte-for-byte spec for tickets 002/003):**
  CRC-16/CCITT-FALSE — `poly=0x1021`, `init=0xFFFF`, `refin=false`,
  `refout=false`, `xorout=0x0000`, MSB-first, no reflection. Known-answer
  vector asserted in the test: `crcCompute("123456789", 9) == 0x29B1`
  (RevEng catalogue check value). Wire placement helpers `encodeCrc16()`/
  `decodeCrc16()` write/read the 2-byte result **little-endian**, using
  the same buf/cap/pos cursor discipline as `encodeFixed32()`/
  `decodeFixed32()` — ticket 002/003 must match this exact byte order,
  not just the polynomial.
  - *Rationale (16 vs. 32 bits):* frames on this wire are small (well
    under 256 B even before this change) — a 16-bit CRC gives a 2^-16
    miss rate on random corruption at half the per-frame overhead (2
    bytes vs. 4) of a CRC-32, on a link whose whole point this sprint is
    shedding overhead rather than re-spending most of it on the
    integrity check that replaces base64's expansion.
- **COBS overhead formula:** `cobsEncodedMaxLength(rawLen) = rawLen +
  rawLen/254 + 1` (integer division) — a worst-case bound, exact only
  when the payload has no embedded zero bytes (a zero-dense payload can
  encode smaller, never larger). Measured/asserted exactly in the test
  at the 254-byte and 508-byte (2x) block boundaries: a 254-byte
  zero-free payload encodes to exactly 256 bytes; a 508-byte zero-free
  payload (two full blocks landing exactly on the boundary) encodes to
  exactly 511 bytes. `cobsDecodedMaxLength(encodedLen) == encodedLen`
  (decoding only removes overhead, never adds). The delimiting `0x00`
  itself is NOT appended by `cobsEncode()` — that is left to the framing
  layer (ticket 002's `Comms`), consistent with this ticket's
  primitives-only scope (no transport, no I2C).
  - *Composition note for ticket 002:* the sprint.md prose ("COBS-encodes
    the payload and appends the CRC") reads as CRC-after-COBS, but the
    correct composition to preserve the zero-free wire property is
    CRC-then-COBS: compute `crcCompute()` over the schema-encoded
    payload, append the 2-byte CRC to it (`encodeCrc16`), THEN
    `cobsEncode()` the combined (payload+CRC) bytes, then append the
    trailing `0x00` delimiter. Appending the CRC after COBS-encoding
    would risk emitting a literal `0x00` if the CRC bytes happen to
    contain one, breaking the delimiter property. Flagging this for
    ticket 002, not deviating from ticket 001's own acceptance criteria
    (which do not mandate an order).
- **Where they live:** `src/firm/messages/wire_runtime.h`/`.cpp`, items 8
  (COBS) and 9 (CRC) — alongside the existing varint/zigzag/fixed32/
  base64 primitives, same schema-agnostic layer, no `#include` of any
  other `messages/*.h`, no `msg::` type named. Base64 left in place per
  the plan (ticket 002 cuts it over).
- **How tested:** extended the existing
  `src/tests/sim/unit/wire_runtime_harness.cpp` (new scenario in an
  existing binary, not a new pytest test function) with 4 new scenarios
  — `scenarioCobsRoundTrip` (empty/short/embedded-zero/all-zeros/
  no-zeros-300/254-byte-boundary/255-byte/508-byte-2x-boundary payloads,
  plus asserting the encoded output never contains a `0x00`),
  `scenarioCobsMalformedInput` (literal `0x00` in place of a code byte,
  truncated block, zero-length input, literal `0x00` inside a data
  block), `scenarioCrcKnownVectorAndCleanFrame` (known-answer vector,
  clean-frame pass, `encodeCrc16`/`decodeCrc16` round-trip), and
  `scenarioCrcDetectsCorruption` (single-bit flip, byte truncation,
  over-length corruption). `test_wire_runtime.py`'s two existing pytest
  tests (normal build + ASan/UBSan rebuild) both compile and run this
  extended harness — both pass, including under ASan/UBSan (proves the
  new COBS bounds-checking, not just the returned bool). Full suite:
  `uv run python -m pytest -q` → 1407 passed / 2 skipped / 9 xfailed / 2
  xpassed — unchanged from baseline (new coverage landed inside an
  existing binary/pytest test, not a new pytest test function). ARM
  firmware + HOST_BUILD sim both build green via `uv run python3
  build.py --clean`.

## Implementation Plan

- **Approach:** Add the two primitive pairs alongside the existing
  varint/base64 functions; keep base64 in place until ticket 002 cuts
  over (avoids a half-migrated intermediate state within this ticket).
- **Files:** `src/firm/messages/wire_runtime.h`, `wire_runtime.cpp`, new
  unit test file(s) under the existing `src/tests/` tree mirroring how
  `wire_runtime`'s existing primitives are tested today.
- **Testing:** Host-buildable unit tests (no hardware needed for this
  ticket — pure byte-manipulation logic).
- **Docs:** None yet — `messages/DESIGN.md` update rides ticket 005.
