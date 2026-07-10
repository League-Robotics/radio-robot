---
id: '004'
title: 'wire_runtime.{h,cpp}: hand-written varint/zigzag/fixed32/length-delimited/base64
  primitives'
status: done
use-cases:
- SUC-003
depends-on:
- '001'
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

- [x] Varint encode/decode round-trips correctly for `0`, `1`, small
      positive values, `UINT32_MAX`, and multi-byte boundary values (127,
      128, 16383, 16384).
- [x] Zigzag encode/decode round-trips correctly for `0`, small positive/
      negative values, `INT32_MIN`, `INT32_MAX`.
- [x] Fixed32 encode/decode round-trips correctly for `0.0f`, negative
      values, the smallest/largest representable `float` magnitudes this
      schema's bounds actually use (e.g. ±31.416 for angle fields,
      ±10000 for distance).
- [x] Base64 encode/decode round-trips correctly for empty input, a
      single byte, and a full 186-byte envelope-sized buffer; the chosen
      alphabet is documented as the first line of `wire_runtime.h`'s doc
      comment.
- [x] A truncated buffer (varint missing its continuation byte,
      length-delimited field claiming more bytes than remain, base64
      string with invalid padding) is rejected with a clean failure
      return — never reads past the buffer end (verify under ASan on the
      host build), never crashes.
- [x] Length-delimited recursion has an enforced depth bound (documented
      constant); a maliciously/accidentally over-nested input is rejected
      cleanly rather than overflowing the stack (verify with a
      synthetic-nesting test case).
- [x] The packed-repeated reader clamps at the caller-supplied
      `max_count` and does not overflow a fixed-size output array when
      fed more elements than the cap.
- [x] The unknown-field skip correctly advances past an unrecognized
      field number of each wire type (varint, fixed32, fixed64,
      length-delimited) without corrupting the read position for
      subsequent known fields.
- [x] No heap allocation (verify by inspection — no `new`/`malloc`
      anywhere in `wire_runtime.{h,cpp}`); compiles clean under
      `-fno-exceptions -fno-rtti`; no `%f`/float `snprintf`.
- [x] `just build` (ARM) and `just build-sim` both succeed; the full
      existing sim suite stays green.

## Completion Notes (2026-07-10)

**Files**: `source/messages/wire_runtime.{h,cpp}` (new, hand-written, the
ONE never-regenerated file in the codec stack), `tests/sim/unit/
wire_runtime_harness.cpp` + `tests/sim/unit/test_wire_runtime.py` (new,
mirror `runtime_blackboard_harness.cpp`/`test_runtime_blackboard.py`'s
exact pattern), `tests/_infra/sim/CMakeLists.txt` (one-line addition of
`messages/wire_runtime.cpp` to `FIRMWARE_SOURCES`, same TU-anchor
treatment `messages/layout_checks.cpp` already gets, so `just build-sim`
proves this file compiles clean under that build's own flags too — the ARM
build needed no CMakeLists edit since the root `CMakeLists.txt` discovers
`source/**/*.cpp` via `RECURSIVE_FIND_FILE`, so `wire_runtime.cpp` was
picked up automatically).

**API shape**: a flat `namespace WireRuntime` of free functions, all
`(buf, len/cap, size_t* pos, ...)` cursor-style (encode and decode
symmetric) — no classes, no heap, no exceptions. Beyond the ticket's 7
named primitives, two small supporting functions are exposed:
`encodeTag`/`decodeTag` (field_number<<3|wire_type, varint-encoded) --
needed by both "length-delimited framing" and "unknown-field skip" to
learn a field's wire type, so factored out once rather than duplicated.

**Zigzag (item 2)**: confirmed unused by this sprint's actual schema —
read every field in `protos/motion.proto`/`protos/envelope.proto`
directly; every signed/bounded field is a protobuf `float` (fixed32 wire
type), not `sint32`/`sint64`. Implemented anyway per the ticket's own
instruction, for both 32- and 64-bit widths (protobuf's sint32 AND
sint64), at negligible cost.

**Base64 alphabet (item 7 / Open Question 2)**: **standard `+/`**, as
recommended. Confirmed against ticket 002's actual shipped host code, not
assumed: `grep -n "base64" host/robot_radio/io/serial_conn.py` shows
`import base64` and calls to `base64.b64encode(...)`/
`base64.b64decode(...)` (both `send_envelope()`'s encode path and the
`*B<base64>` reply-decode path) — Python stdlib defaults, i.e. the
standard alphabet. No mismatch to flag; firmware and host agree. Pinned as
the bold first line of `wire_runtime.h`'s doc comment per the ticket's
instruction, and the harness's malformed-base64 scenario explicitly proves
url-safe `-`/`_` characters are REJECTED (not silently accepted as a second
valid alphabet).

**Length-delimited depth bound (item 4)**: `kMaxNestingDepth = 8`. This
schema's actual max nesting is shallow (`CommandEnvelope` -> e.g.
`DrivetrainCommand` -> `WheelTargets` -> repeated `WheelTarget` is the
deepest chain today, 3 levels) — 8 is small-constant headroom over that,
matching the ticket's own "e.g. 8" guidance. `beginLengthDelimited(buf,
len, pos, depth, payloadLen)` checks the bound before parsing anything and
leaves `*pos` unchanged on rejection; the depth-increment contract (only
increment when recursing into a NESTED MESSAGE, not for leaf bytes/
string/packed payloads) is documented in `wire_runtime.h` for ticket 005's
generated decoder to follow.

**Packed-repeated clamp (item 5)**: two concrete variants,
`decodePackedVarint`/`decodePackedFixed32`, covering the only two packable
scalar wire shapes this tree's generated arrays actually use (uint32_t and
float, per `messages/common.h`'s `command_modes_[8]`/`args_[4]`). Every
element in the payload is parsed (so a malformed trailing element past
`max_count` is still caught) but only the first `maxCount` are written;
the harness's clamp scenario sizes the output array to EXACTLY `maxCount`
and runs under ASan so a real overflow would abort, not just fail an
equality check.

**Verification performed**: standalone host compile
(`c++ -std=c++20 -Wall -Wextra -fno-exceptions -fno-rtti`) → clean, zero
warnings. `uv run python -m pytest tests/sim/unit/test_wire_runtime.py -q`
→ **2 passed** (normal build + a second full recompile/rerun under
`-fsanitize=address,undefined`, both green — the ASan/UBSan run covers
every scenario including all malformed-input and packed-clamp cases, not
a separate subset). `just build` (ARM) → green;
`source/messages/wire_runtime.cpp.obj` compiles with no warnings under the
project's real `-fno-exceptions -fno-rtti -std=gnu++20` flags (confirmed
by grepping `build/compile_commands.json` for `-fno-rtti`/`-fno-exceptions`
before relying on it); flash 83.67% used (comfortably within budget), RAM
98.33% (expected/by-design on this target, not a regression signal — see
project knowledge on CODAL RAM headroom). `just build-sim` → green,
`wire_runtime.cpp` links into `libfirmware_host`. `uv run python -m
pytest tests/sim -q` → **60 passed** (the pre-existing 58 plus this
ticket's 2 new tests, zero regressions).

**No heap / no `%f`**: verified by inspection — `wire_runtime.{h,cpp}`
contain no `new`, no `malloc`, no `std::vector`/`std::string`, and no
`snprintf`/format-string call of any kind (pure binary encode/decode, no
text formatting).

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
