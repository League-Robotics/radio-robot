---
id: '003'
title: COBS delimiter parameterization (0x0A) and CRC scope extension to the command
  name
status: open
use-cases: [SUC-001]
depends-on: []
github-issue: ''
issue: protocol-v5-one-line-packets-command-prefix-and-newline-cobs.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# COBS delimiter parameterization (0x0A) and CRC scope extension to the command name

## Description

Core codec change everything downstream frames against. COBS guarantees
`0x00`-freedom but not `0x0A`-freedom, which is exactly what let a
`move_wheels` envelope embedding a literal `0x0A` corrupt on the wire
(fixed narrowly in 123-006). Fix the class, not the instance: key COBS
on `0x0A` instead of `0x00` (an XOR over the existing encoder's output,
per the issue's own worked example and verified adversarial-input table),
and extend the CRC's input to cover `COMMAND ':'` plus the payload, not
just the payload — closing the "bit-flip lands on another valid verb"
gap.

Prefer a delimiter parameter on `WireRuntime::cobsEncode()`/
`cobsDecode()` (default `0x00`) over a separate XOR at every call site,
so the delimiter constant lives in exactly one place per side. Mirror
both changes in host `wire_codec.py`. Update `wire_runtime.h`'s header
comment, currently written entirely in `0x00` terms.

## Acceptance Criteria

- [ ] `cobsEncode()`/`cobsDecode()` take a delimiter parameter; existing
      `0x00` call sites are unaffected by the default.
- [ ] `crcCompute("123456789") == 0x29B1` still passes — the CRC variant
      itself is unchanged, only its input range extends.
- [ ] CRC-scope vector: identical payload under two different command
      names produces two different CRCs; a frame whose command byte is
      mutated in transit fails verification rather than dispatching.
- [ ] Round-trip verified against the adversarial inputs the issue
      already worked out (all-`0x0A`, all-`0x00`, `0x00..0xFF` sweep).
- [ ] Host `wire_codec.py` mirrors both changes exactly (delimiter
      parameter, extended CRC input).
- [ ] `encode_frame()`/`decode_frame()` (host) and `Comms::sendReply()`/
      `decodeBinaryFrame()` (firmware) take the command bytes as a
      separate CRC-scope argument — not concatenated-and-sliced.

## Testing

- **Existing tests to run**: existing `wire_runtime` C++ unit tests,
  `test_wire_codec.py`.
- **New tests to write**: known-answer vectors (all-`0x0A`, all-`0x00`,
  `0x00..0xFF` sweep, empty payload); a property test that random
  payloads up to 251 bytes never contain `0x0A` post-encode and
  round-trip; the CRC-scope differential vector above.
- **Verification command**: `uv run pytest` plus the C++ sim-tests build.
