---
id: '004'
title: 'wire_runtime.{h,cpp}: hand-written varint/zigzag/fixed32/length-delimited/base64
  primitives'
status: open
use-cases: [SUC-003]
depends-on: ['001']
github-issue: ''
issue: protocol-v3-schema-driven-binary-command-plane-protobuf.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# wire_runtime.{h,cpp}: hand-written varint/zigzag/fixed32/length-delimited/base64 primitives

## Description

Hand-write the ONE schema-agnostic byte-level codec toolkit in the whole
stack — never regenerated, never touching `CommandEnvelope` or any
specific message type. Ticket 005's generated `wire.{h,cpp}` builds on
this; `BinaryChannel` (ticket 007) uses its base64 functions directly for
the `*B<base64>` armor layer.

Implement in `source/messages/wire_runtime.{h,cpp}` (new files):

1. **Varint encode/decode** (protobuf's base-128 varint, unsigned).
2. **Zigzag encode/decode** (signed integer mapping for `sint32`/`sint64`
   wire types, if the schema uses them — otherwise implement it anyway
   since `min`/`max`/`abs_max`-bounded signed fields may benefit from it;
   confirm against ticket 001's actual field types before deciding it's
   unused).
3. **Fixed32 encode/decode** (protobuf `float`/`fixed32`/`sfixed32` wire
   type — this schema's `float` fields, which are most of them, use
   this).
4. **Length-delimited framing** with a depth-bounded recursion guard
   (nested message decode calls itself; cap recursion depth at a small
   constant, e.g. 8, matching the schema's actual max nesting depth with
   headroom — reject deeper input cleanly rather than overflow the
   stack).
5. **Packed-repeated reader** that clamps at a caller-supplied
   `max_count` (mirrors `gen_messages.py`'s existing `(max_count)`
   convention — silently drop entries beyond the cap, per the issue's
   "packed-repeated with max_count clamp").
6. **Unknown-field skip** (read past a field with an unrecognized number
   using its own wire-type-implied length, without erroring — this is
   what lets the declared-only oneof arms/future schema growth stay
   forward-compatible).
7. **Base64 encode/decode** — pick ONE alphabet (Open Question 2:
   standard `+/` vs. URL-safe `-_`; either works since both exclude
   `\0 \r \n #`) and document the choice as the FIRST line of
   `wire_runtime.h`'s doc comment, in bold or equivalently unmissable
   formatting, so `host/robot_radio/io/serial_conn.py`'s `send_envelope()`
   (ticket 002) cannot silently pick the other one. Recommend standard
   base64 (`+/`) since it's what Python's `base64.b64encode` defaults to
   (`base64.urlsafe_b64encode` requires an explicit call) — minimizes the
   chance of an accidental host-side alphabet mismatch.

Constraints (verify each explicitly, don't just assume the compiler will
catch a violation): CODAL C++11, no heap allocation anywhere in this file
(every function operates on caller-owned buffers passed by pointer+size),
`-fno-exceptions -fno-rtti` compiles clean, newlib-nano (no `%f`/float
`snprintf` — these are pure binary encode/decode functions, so this
should be naturally satisfied, but double-check no debug/error-path code
sneaks in a `snprintf("%f", ...)`).

## Acceptance Criteria

- [ ] Varint encode/decode round-trips correctly for `0`, `1`, small
      positive values, `UINT32_MAX`, and multi-byte boundary values (127,
      128, 16383, 16384).
- [ ] Zigzag encode/decode round-trips correctly for `0`, small positive/
      negative values, `INT32_MIN`, `INT32_MAX`.
- [ ] Fixed32 encode/decode round-trips correctly for `0.0f`, negative
      values, the smallest/largest representable `float` magnitudes this
      schema's bounds actually use (e.g. ±31.416 for angle fields,
      ±10000 for distance).
- [ ] Base64 encode/decode round-trips correctly for empty input, a
      single byte, and a full 186-byte envelope-sized buffer; the chosen
      alphabet is documented as the first line of `wire_runtime.h`'s doc
      comment.
- [ ] A truncated buffer (varint missing its continuation byte,
      length-delimited field claiming more bytes than remain, base64
      string with invalid padding) is rejected with a clean failure
      return — never reads past the buffer end (verify under ASan on the
      host build), never crashes.
- [ ] Length-delimited recursion has an enforced depth bound (documented
      constant); a maliciously/accidentally over-nested input is rejected
      cleanly rather than overflowing the stack (verify with a
      synthetic-nesting test case).
- [ ] The packed-repeated reader clamps at the caller-supplied
      `max_count` and does not overflow a fixed-size output array when
      fed more elements than the cap.
- [ ] The unknown-field skip correctly advances past an unrecognized
      field number of each wire type (varint, fixed32, fixed64,
      length-delimited) without corrupting the read position for
      subsequent known fields.
- [ ] No heap allocation (verify by inspection — no `new`/`malloc`
      anywhere in `wire_runtime.{h,cpp}`); compiles clean under
      `-fno-exceptions -fno-rtti`; no `%f`/float `snprintf`.
- [ ] `just build` (ARM) and `just build-sim` both succeed; the full
      existing sim suite stays green.

## Testing

- **Existing tests to run**: `just build`, `just build-sim`, `uv run
  python -m pytest tests/sim -q`.
- **New tests to write**: a host-compiled C++ harness
  (`tests/sim/unit/wire_runtime_harness.cpp`, following
  `runtime_blackboard_harness.cpp`'s exact pattern — compiled by the
  system C++ compiler, no ARM toolchain needed) exercising every
  acceptance criterion above as a battery of round-trip/boundary/
  malformed-input assertions, driven by a
  `tests/sim/unit/test_wire_runtime.py` pytest wrapper (compile + run +
  assert exit 0, matching `test_runtime_blackboard.py`'s shape). Include
  an ASan/UBSan-built variant of the harness (or a documented separate
  invocation) specifically for the truncated/malformed-input cases, since
  those are exactly the out-of-bounds-read risk this ticket's acceptance
  criteria call out.
- **Verification command**: `uv run python -m pytest
  tests/sim/unit/test_wire_runtime.py -q` plus the full
  `uv run python -m pytest tests/sim -q`.
