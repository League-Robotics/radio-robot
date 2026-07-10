---
id: '005'
title: 'gen_messages.py: FieldDesc tables + wire.{h,cpp} generation + kMaxEncodedSize<=186
  asserts'
status: open
use-cases: [SUC-004]
depends-on: ['003', '004']
github-issue: ''
issue: protocol-v3-schema-driven-binary-command-plane-protobuf.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# gen_messages.py: FieldDesc tables + wire.{h,cpp} generation + kMaxEncodedSize<=186 asserts

## Description

Extend `scripts/gen_messages.py` to emit the generic, schema-driven
decode/encode/validate engine — the expensive generator work ticket 003's
gate exists to de-risk BEFORE it's written. Consumes ticket 003's gate
result and ticket 004's `wire_runtime` primitives.

1. **Check ticket 003's gate result first.** If it passed for every
   struct this ticket needs, proceed with the generic `offsetof`-based
   `FieldDesc` table approach below. If it failed for specific structs,
   implement the documented fallback (generated per-message UNROLLED
   decode/encode functions) for those structs, behind the identical
   `msg::wire::decode`/`encode` API, so callers (ticket 007's
   `BinaryChannel`) never know which path a given message took.
2. **Emit, per message type, a `FieldDesc{number, wireType, kind, offset,
   aux, min, max}` table in `.rodata`** (flash, not RAM):
   - `number`: proto field number (the wire tag source).
   - `wireType`: varint/fixed32/fixed64/length-delimited, derived from
     the proto field's type.
   - `kind`: scalar kind (float/double/int32/.../message/enum), or a
     marker for repeated/oneof-member/`Opt<T>` fields — each needs
     slightly different decode handling (repeated: clamp at `max_count`;
     oneof member: also set the `_kind` discriminator; `Opt<T>`: set
     `.has = true`).
   - `offset`: `offsetof(Struct, field)` for scalar/message/`Opt<T>`
     fields; for a oneof-union member, the union member's own offset
     within the struct.
   - `aux`: context-dependent — for repeated fields, the count-field's
     offset + `max_count` capacity; for oneof members, the `_kind`
     discriminator field's offset + the enum value selecting that arm;
     for message-typed fields, an index into a generated table-of-tables
     (`kMessageTables[]`) so the generic walker can recurse into the
     nested message's own `FieldDesc` table.
   - `min`/`max`: from ticket 001's `(min)`/`(max)`/`(abs_max)` options —
     see Open Question 1 below for storage width.
3. **Emit `source/messages/wire.{h,cpp}`**:
   ```cpp
   namespace msg { namespace wire {
   struct Result { bool ok; uint16_t field; ErrCode code; };
   Result   decode(CommandEnvelope& out, const uint8_t* buf, uint16_t len);
   uint16_t encode(const ReplyEnvelope& in, uint8_t* buf, uint16_t cap);
   }}
   ```
   `decode()` walks the target message's `FieldDesc` table per incoming
   wire tag, validates `min`/`max`/`abs_max`/`req` INLINE during the same
   pass (single pass, no second validation walk), and returns
   `{false, fieldNumber, ErrCode}` on the first violation (missing `req`
   field, out-of-bound value, or a structurally malformed buffer —
   delegate malformed-buffer detection to ticket 004's primitives).
   Unknown field numbers are skipped (ticket 004's unknown-field-skip),
   never rejected. `encode()` returns `0` (not a truncated buffer) when
   the caller's buffer is smaller than the required output.
4. **Emit `static_assert(kMaxEncodedSize<=186)` per top-level envelope
   type** (`CommandEnvelope`, `ReplyEnvelope`) — computed from each
   field's worst-case encoded size (varint max width, fixed sizes,
   nested-message worst case). This is a BUILD-TIME check: if a schema
   change (this sprint's or a future one) pushes an envelope over budget,
   the build fails loudly here, not at runtime on a truncated wire line.
   Per Open Question 3, `ReplyEnvelope{DeviceId}` is the closest to the
   ceiling this sprint (two `char[64]` string fields) — explicitly
   compute and report its worst-case size in this ticket's completion
   notes; if it's over 186 bytes, shrink `DeviceId`'s string field sizes
   in ticket 001's schema (revisit, don't silently let the assert fail
   the build with no explanation) rather than solve it here.

**Open Question 1 (resolve and document in this ticket's completion
notes)**: store `min`/`max`/`abs_max` in the generated `FieldDesc` table
as `float` (4 bytes, halves the ~4 KB table budget, matches every
generated scalar field's own `float` type) or `double` (8 bytes, matches
the proto option's own declared width, simpler generator code — no
narrowing-cast step). Recommendation: `float` — the flash budget pressure
(issue's own +12-15 KB dual-stack estimate) argues for it, and no
schema field this sprint needs more than `float` precision for a bound.
Document whichever is chosen in `wire.h`'s own header comment so a future
schema author knows what precision their `(min)`/`(max)`/`(abs_max)`
declaration actually gets.

## Acceptance Criteria

- [ ] Ticket 003's gate result is consulted; if any struct failed the
      standard-layout check, this ticket's completion notes confirm the
      unrolled-codegen fallback was used for those specific structs and
      name them (should be "none" for a clean pass, but the criterion is
      about explicit handling either way, not a specific outcome).
- [ ] `decode()` correctly rejects a message with a missing `req` field,
      an out-of-`min`/`max`/`abs_max` field, with a `{fieldNumber,
      ErrCode}` result — verified for at least one field of each
      validated kind (`min`, `max`, `abs_max`, `req`) in this sprint's
      schema.
- [ ] `decode()` correctly SKIPS an unknown field number rather than
      rejecting the message (forward-compatibility with future schema
      growth).
- [ ] `encode()` returns `0` (not a truncated/corrupt buffer) when the
      caller's buffer is smaller than the required output.
- [ ] `kMaxEncodedSize` for `CommandEnvelope` and `ReplyEnvelope` is
      `<= 186` bytes, enforced by a generated `static_assert` that fails
      the BUILD (not a runtime check) if violated. `ReplyEnvelope{DeviceId}`'s
      worst-case size is explicitly computed and reported in this
      ticket's completion notes.
- [ ] Repeated fields decode up to their `max_count` and silently clamp
      (never overflow the fixed backing array) — verified for at least
      one repeated field this schema uses.
- [ ] Open Question 1 (float vs. double table storage) is resolved and
      documented in `wire.h`'s header comment.
- [ ] `just build` (ARM) and `just build-sim` succeed; the full existing
      sim suite stays green.

## Testing

- **Existing tests to run**: `just build`, `just build-sim`, `uv run
  python -m pytest tests/sim -q`.
- **New tests to write**: a host-compiled harness
  (`tests/sim/unit/wire_codec_harness.cpp` or folded into ticket 004's
  harness if that proves cleaner) exercising `msg::wire::decode`/`encode`
  directly against every implemented `CommandEnvelope`/`ReplyEnvelope`
  arm this sprint declares (not just the ones `BinaryChannel` wires up in
  ticket 007 — this ticket's own scope is the generic engine, testable
  independent of the dispatcher), covering the required/bound/unknown-
  field/oversized-buffer acceptance criteria above. This is DISTINCT from
  ticket 006's differential-against-`google.protobuf` suite — this
  ticket's tests check the generic engine's OWN behavior in isolation;
  ticket 006 checks it AGREES with the reference implementation.
- **Verification command**: `uv run python -m pytest
  tests/sim/unit/test_wire_codec.py -q` (or wherever this ticket's driver
  lands) plus `uv run python -m pytest tests/sim -q`.
