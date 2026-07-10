---
id: '005'
title: 'gen_messages.py: FieldDesc tables + wire.{h,cpp} generation + kMaxEncodedSize<=186
  asserts'
status: done
use-cases:
- SUC-004
depends-on:
- '003'
- '004'
github-issue: ''
issue: protocol-v3-schema-driven-binary-command-plane-protobuf.md
completes_issue: false
exception:
  thrown_by: programmer
  thrown_at: '2026-07-10T08:31:45.529352+00:00'
  attempted: Read ticket 005, architecture-update.md's M4 section, wire_runtime.{h,cpp}
    (004), the generated envelope.h/motion.h (001), layout_checks.h (003, the 30-struct
    reachable-from-CommandEnvelope/ReplyEnvelope set), and every proto file that set
    (drivetrain/motion/planner/odometer/common) touches, plus gen_messages.py's existing
    _emit_message/_classify_oneofs machinery. Fully designed the FieldDesc/MessageTable/FieldKind
    generic offsetof-based decode/encode/validate engine (per ticket 003's clean standard-layout
    gate result -- no unrolled fallback needed) including oneof-discriminator handling,
    Opt<T> has-flag handling, repeated-message clamping, bytes/string length-delimited
    clamping, and the single-wire-pass req-completeness check. Before writing any
    gen_messages.py or wire.{h,cpp} code, hand-computed kMaxEncodedSize for every
    CommandEnvelope/ReplyEnvelope oneof arm using the ticket's own literal rule ("computed
    from each field's worst-case encoded size... nested-message worst case", MAX (not
    sum) across mutually-exclusive oneof arms, matching nanopb's own PB_SIZE_MAX convention
    this codebase already cites) -- the same due-diligence step Open Question 3 already
    mandates for DeviceId, extended to every arm rather than just the one the ticket
    flagged. Confirmed the 7 implemented arms (drive=56B, segment/replace=64B, ping=2B,
    echo=68B, stop=2B) plus DeviceId-based id (210B unshrunk, 171B after shrinking
    DeviceId's 3 char[64] strings to char[48] per OQ3's own sanctioned latitude) all
    fit comfortably under 186B for both CommandEnvelope and ReplyEnvelope -- DeviceId
    is a clean, isolated, already-sanctioned fix with no other blocker.
  conflict: 'CommandEnvelope''s `motion` arm (field 5, PlannerCommand, declared-only/ERR_UNIMPLEMENTED
    this sprint per ticket 001/architecture-update.md) makes ticket 005''s own acceptance
    criterion ("kMaxEncodedSize for CommandEnvelope... <=186 bytes, enforced by a
    generated static_assert that FAILS THE BUILD") structurally unsatisfiable, independent
    of and in addition to the already-anticipated DeviceId fix. PlannerCommand''s
    own worst-case encoded size is 327 bytes: `repeated StopCondition stops = 9 [(max_count)
    = 4]` alone costs 160B (4 x (tag+len+38B StopCondition body), each occurrence
    separately tagged since repeated-message is never packed), the `goal` oneof''s
    worst arm (VelocityGoal) costs 23B, and the two `string` fields `corr_id`=12/`verb`=13
    cost 66B each at the generator''s fixed char[64] width -- 327B total, wrapped
    inside CommandEnvelope''s own oneof (+3B tag/len) exceeds 186 on its own, before
    corr_id or any other arm is even counted. Unlike DeviceId (a brand-new envelope.proto
    type, ticket-001-owned, no other consumer, narrow "shrink 3 strings" fix explicitly
    sanctioned by Open Question 3), PlannerCommand is a pre-existing `protos/planner.proto`
    message with a LIVE non-wire consumer: `source/commands/motion_commands.cpp`''s
    text-plane R/TURN/G handlers construct `msg::PlannerCommand` today (`copyCorrId()`
    at line 39 explicitly sizes into `corr_id[64]`; a comment at line 29 cites `stops_[4]`''s
    capacity) -- so shrinking `stops`''s max_count or corr_id/verb''s width to fit
    the wire budget would also require re-verifying/touching that text-plane code,
    well beyond gen_messages.py/protos/envelope.proto. The only fix that stays inside
    a single message''s own shape (matching architecture-update.md Decision 2''s own
    precedent: MotionSegment was deliberately made a NEW, wire-specific message rather
    than exposing the internal Motion::Segment type directly, for exactly this class
    of reason) is to give `motion` its own new, deliberately-bounded wire payload
    type instead of exposing PlannerCommand as-is -- a schema/architecture decision,
    not a codegen-ticket judgment call. I do not have the authority to either (a)
    silently narrow kMaxEncodedSize''s computation to exclude declared-only arms (this
    would defeat the exact "catches a future/unnoticed schema violation at build time,
    not runtime" safety purpose ticket 005''s own text states for this static_assert),
    or (b) redesign the `motion` arm''s wire shape unilaterally (that is squarely
    ticket-001/architecture territory, the same level Decision 2 was made at).'
  surface: internal
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# gen_messages.py: FieldDesc tables + wire.{h,cpp} generation + kMaxEncodedSize<=186 asserts

## Description

**Revised per `architecture-update-r1.md` (Decision 6), in response to
this ticket's own thrown exception — read that document first.** The
original design ("declare all arms now") made `CommandEnvelope`'s
declared-only `motion` arm (`PlannerCommand`) structurally unable to fit
the 186-byte envelope budget (327B worst-case, dominated by
`repeated StopCondition stops[4]` = 160B, never packed since it's
repeated-message). Decision 6 revises the schema to declare oneof arms
incrementally (non-breaking in protobuf — a later-added field number is
simply skipped by older builds via ticket 004's unknown-field-skip) and
removes `motion` from this sprint's schema rather than either exempting
it from the size check (defeats the check's purpose) or shrinking
`PlannerCommand` (out of scope — it has a live text-plane consumer,
`motion_commands.cpp`'s R/TURN/G handlers).

**Step 0 below is new and must be done FIRST, before any codegen work.**
Steps 1-4 are the original codec-generation scope, unchanged.

### Step 0: Make the schema fit the wire budget

Edit `protos/envelope.proto` and `protos/motion.proto`/wherever
`DeviceId` lives (per ticket 001's actual file layout):

0a. **Remove `PlannerCommand motion = 5;` from `CommandEnvelope.cmd`.**
    Field number 5 stays reserved/skipped — do not renumber any other
    arm, do not reassign 5 to a different field. When a future sprint
    un-parks `Subsystems::Planner` and implements the `motion` arm, it
    defines a new, deliberately-bounded wire payload type for it first
    (the same move Decision 2 already made for `segment`/`replace`:
    `MotionSegment` rather than exposing `Motion::Segment` directly), the
    same way this step is now doing for `motion`'s removal.
0b. **Shrink `DeviceId`'s `model`/`name`/`fw_version` fields from
    `char[64]` to `char[48]`** (the generator's fixed string width — add
    a minimal per-field string-width mechanism to `gen_messages.py` if
    one doesn't already exist, scoped to this one case; do not
    over-engineer a general string-width option beyond what this fix
    needs). This drops `ReplyEnvelope{DeviceId}`'s worst-case from ~210B
    to ~171B, comfortably under 186B. `DeviceId` is also
    `CommandEnvelope`'s `id` request arm (empty payload, unaffected by
    this shrink either way).
0c. **Verify every remaining declared arm independently fits.** Implemented
    arms (`drive`/`segment`/`replace`/`stop`/`ping`/`echo`/`id`) were
    already hand-verified comfortably under budget (56B/64B/64B/2B/2B/68B/
    171B respectively, per the exception's own due-diligence numbers) —
    re-confirm via the generated `static_assert` once it exists, don't
    just trust the hand computation. Declared-only arms this sprint keeps
    (`config`/`ConfigDelta`, `pose`/`SetPose`, `otos`/`OdometerCommand`,
    `get`/`ConfigGet`, `stream`/`StreamControl`) have NOT been individually
    sized yet — size each one (same worst-case-encoding method used for
    `motion`/`DeviceId` above) BEFORE proceeding to Step 1; if any is also
    over budget, stop and escalate the same way this ticket's first
    attempt correctly did (do not silently narrow the check or redesign a
    pre-existing message's shape unilaterally) rather than guessing it's
    fine.

### Steps 1-4 (original scope, unchanged)

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

- [x] **(New, Step 0)** The `motion` arm (`PlannerCommand`, field 5) is
      removed from `CommandEnvelope.cmd` in `protos/envelope.proto`; its
      removal (and field-number 5 reservation, not reuse) is noted in this
      ticket's completion notes, referencing `architecture-update-r1.md`
      Decision 6.
- [x] **(New, Step 0)** `DeviceId`'s `model`/`name`/`fw_version` fields
      are `char[48]` (shrunk from `char[64]`); `ReplyEnvelope{DeviceId}`'s
      resulting worst-case `kMaxEncodedSize` is reported in completion
      notes (expected ~171B).
- [x] **(New, Step 0)** Every remaining declared arm's worst-case
      `kMaxEncodedSize` is individually computed and reported in
      completion notes BEFORE Step 1 proceeds — implemented arms
      (drive/segment/replace/stop/ping/echo/id) and declared-only arms
      (config/pose/otos/get/stream) alike — and every one is `<= 186`
      bytes. If any is over budget, this is treated as a NEW exception
      (escalate, do not silently fix by narrowing the check or editing an
      out-of-scope message), per the same judgment this ticket's original
      attempt correctly exercised for `motion`.
- [x] Ticket 003's gate result is consulted; if any struct failed the
      standard-layout check, this ticket's completion notes confirm the
      unrolled-codegen fallback was used for those specific structs and
      name them (should be "none" for a clean pass, but the criterion is
      about explicit handling either way, not a specific outcome).
- [x] `decode()` correctly rejects a message with a missing `req` field,
      an out-of-`min`/`max`/`abs_max` field, with a `{fieldNumber,
      ErrCode}` result — verified for at least one field of each
      validated kind (`min`, `max`, `abs_max`, `req`) in this sprint's
      schema.
- [x] `decode()` correctly SKIPS an unknown field number rather than
      rejecting the message (forward-compatibility with future schema
      growth).
- [x] `encode()` returns `0` (not a truncated/corrupt buffer) when the
      caller's buffer is smaller than the required output.
- [x] `kMaxEncodedSize` for `CommandEnvelope` and `ReplyEnvelope` is
      `<= 186` bytes, enforced by a generated `static_assert` that fails
      the BUILD (not a runtime check) if violated. `ReplyEnvelope{DeviceId}`'s
      worst-case size is explicitly computed and reported in this
      ticket's completion notes.
- [x] Repeated fields decode up to their `max_count` and silently clamp
      (never overflow the fixed backing array) — verified for at least
      one repeated field this schema uses.
- [x] Open Question 1 (float vs. double table storage) is resolved and
      documented in `wire.h`'s header comment.
- [x] `just build` (ARM) and `just build-sim` succeed; the full existing
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

## Completion Notes (2026-07-10)

**Files**: `protos/envelope.proto` (Step 0: `motion` removed + reserved,
`DeviceId` shrunk, `ConfigGet.target` made `optional`+`(req)`),
`protos/options.proto` (new `(str_len)` extension, field 50006),
`scripts/gen_messages.py` (the `(str_len)` mechanism; the `bytes`-field
emission fix; the whole new FieldDesc/MessageTable/`kMaxEncodedSize`
generator section), `source/messages/envelope.h` + `layout_checks.h`
(regenerated — `CommandEnvelope`'s `motion` arm and `PlannerCommand` +
its 7 goal-variant messages + `StopCondition` drop out of the
reachable-from-envelope set entirely), `source/messages/wire.{h,cpp}`
(new, generated), `tests/_infra/sim/CMakeLists.txt` (added
`messages/wire.cpp` to `FIRMWARE_SOURCES`; the ARM build needed no edit —
`RECURSIVE_FIND_FILE` picks it up automatically),
`tests/sim/unit/wire_codec_harness.cpp` + `tests/sim/unit/test_wire_codec.py`
(new). Also regenerated as a mechanical downstream consequence of the
proto edits (M7's own build step, ticket 002): `host/robot_radio/robot/pb2/envelope_pb2.py`
and `options_pb2.py` — included in this ticket's commit so the host
mirror never skews from the schema this ticket also changed.

### Step 0 — schema-fit results

**0a.** `PlannerCommand motion = 5` removed from `CommandEnvelope.cmd`;
`reserved 5;` added at the `CommandEnvelope` message level (not reused).
`protos/envelope.proto`'s header comment and the oneof's own inline
comment both cite `architecture-update-r1.md` Decision 6. The now-unused
`import "planner.proto";` was also removed (protoc had started warning
"Import planner.proto is unused"); the stale
"Cross-file message references... motion->PlannerCommand" paragraph
(pre-dating ticket 003's actual fix) was corrected in the same pass.
Regenerating confirms `PlannerCommand` and its 7 goal-variant messages
(`VelocityGoal`/`GotoGoal`/`TurnGoal`/`DistanceGoal`/`TimedGoal`/
`RotationGoal`/`StreamGoal`) plus `StopCondition` all drop out of
`layout_checks.h`'s reachable-from-`CommandEnvelope`/`ReplyEnvelope` set
(31 structs -> 22) — `git diff source/messages/layout_checks.h` shows
exactly those 9 `static_assert`s removed, nothing else.

**0b.** `DeviceId.model`/`.name`/`.fw_version` now carry
`[(str_len) = 48]`; `gen_messages.py` gained the minimal per-field
string-width mechanism the ticket asked for — a new `(str_len)` extension
option (`options.proto`, field 50006) plus `_read_str_len(field)`, wired
into all three string-emitting sites in `_emit_message()` (plain string,
optional string, oneof-union string arm) with a `_DEFAULT_STR_LEN = 64`
fallback so every OTHER string field in the schema is byte-identical to
before (verified: `git diff --stat source/messages/` touches only
`envelope.h` and `layout_checks.h`, nothing else). `ReplyEnvelope{DeviceId}`
worst-case: see the full report below — **162B wrapped** (159B standalone),
comfortably under 186B.

**0c.** Full worst-case report, computed by a new pure-Python calculator
in `gen_messages.py` (varint max width incl. protobuf's own int32
sign-extension gotcha, fixed sizes, nested-message worst case via a
memoized recursive walk, MAX — not sum — across each oneof's mutually
exclusive arms) and baked into `wire.h` as `constexpr` literals +
`static_assert`s (re-verified below by actually building):

```
CommandEnvelope: drive=56B, segment=64B, replace=64B, config=2B, pose=17B,
  otos=19B, ping=2B, echo=68B, get=8B, stream=10B, stop=2B, id=162B
  (worst=id=162B) + non-oneof(corr_id)=6B => total=168B
ReplyEnvelope:   ok=13B, err=10B, tlm=2B, cfg=2B, evt=2B, id=162B
  (worst=id=162B) + non-oneof(corr_id)=6B => total=168B
```

Every declared arm (implemented AND declared-only alike) is `<= 186`
bytes — no second exception needed. `id` (`DeviceId`) is the ceiling in
both envelopes, as Open Question 3 anticipated, with 18B of headroom
under the 186B budget in each direction. Hand-cross-checked several of
these by hand (e.g. `drive`: `DrivetrainCommand`'s own worst-case oneof
arm is `wheels` — `WheelTargets.w` at `max_count=4`, each `WheelTarget`
element `10B` standalone (two `Opt<float>` fields) + 2B tag/len =
12B/element × 4 = 48B, +2B tag/len for the `wheels` arm itself = 50B,
+`seed`+`standby` (2B each) = 54B standalone, +2B tag/len wrapping it as
`CommandEnvelope.cmd.drive` = 56B — matches the generator's own number
exactly) — the formula is sound, not just internally self-consistent.

**Discrepancy flagged, not silently reconciled** (per this project's own
established convention — see `protos/motion.proto`'s `yaw_accel_max`/
`yaw_jerk_max` comments for the precedent): the ticket's own Step 0b text
and `architecture-update-r1.md` both say `ReplyEnvelope{DeviceId}`'s
worst case is "expected ~171B" post-shrink. The generated, authoritative
number is **162B** (wrapped) / **159B** (`DeviceId` standalone). The ~9B
gap is explained by two precision differences in the original hand
estimate versus this generator's exact accounting: (1) a `char[48]`
string's WORST-CASE *content* is 47 bytes, not 48 — one byte is reserved
so `decodeInto()`'s `kString` case can always null-terminate, which the
original estimate likely didn't subtract; (2) the original estimate was
explicitly marked "~" (approximate). 162B is what the generated
`static_assert` actually enforces on every future build, so it is the
number that governs, not the ticket's own approximation — flagged here
per instruction rather than the ticket's estimate being silently treated
as correct.

### Steps 1-4

**Step 1 (ticket 003's gate)**: consulted — **003's own completion notes
report all 31 (now 22, post Step-0a) structs PASS `std::is_standard_layout`
on both toolchains, no fallback needed.** This ticket used the generic
`offsetof`-based `FieldDesc` table approach for every one of the 22
reachable structs; the unrolled-codegen fallback was not needed and is
not present anywhere in `wire.cpp`.

**Step 2 (FieldDesc/MessageTable tables)**: `FieldDesc{number, wireType,
kind, scalarType, offset, offset2, oneofKindValue, cap, tableIndex,
elemStride, flags, minVal, maxVal, absMaxVal}` — wider than the ticket's
own 7-tuple sketch because the generic walker needs a few more
context-dependent slots (a `FieldKind` enum of 9 members: `kScalar`,
`kOpt`, `kOneofScalar`, `kMessage`, `kOneofMessage`, `kString`, `kBytes`,
`kRepeatedScalar`, `kRepeatedMessage` — `kMessage`/`kRepeatedScalar` are
implemented but unreached by this sprint's actual schema, kept for engine
completeness/future growth and clearly commented as such in `wire.cpp`).
`kMessageTables[]` is a flat `constexpr MessageTable[]`, one entry per
reachable struct, indexed by the SAME BFS order ticket 003's
`_compute_layout_check_structs()` already produces (reused directly, not
re-derived) — no forward declarations needed anywhere: the generator
emits the fixed engine's TYPE definitions and scalar helpers first, then
every `kFields_Xxx[]`/`kTable_Xxx`, then `kMessageTables[]`, and only
THEN the recursive `decodeInto`/`encodeInto` walkers (which reference
`kMessageTables[]`, already fully defined above them by that point).

**Step 3 (`wire.{h,cpp}`)**: `msg::wire::decode(CommandEnvelope&, buf,
len)` walks the table with inline `min`/`max`/`abs_max` validation
(returns `ErrCode::ERR_RANGE`) and a single-pass `(req)`-completeness
check via a per-decode "seen" bitmask checked once at the end of each
message's own field loop (`ErrCode::ERR_BADARG` if a `(req)` field's tag
never appeared) — no second walk over the bytes. Unknown field numbers
are skipped via `WireRuntime::skipField`. `msg::wire::encode(const
ReplyEnvelope&, buf, cap)` implements real proto3 implicit presence for
plain (non-oneof, non-`Opt`) scalar/string fields (a zero/empty value is
omitted from the wire, exactly like a real protobuf encoder — verified in
the harness: an `Error` with `corr_id=0` encodes with the `err` oneof arm
as the FIRST byte on the wire, `corr_id`'s tag correctly absent) and
returns `0` (not a truncated buffer) on any encode-side failure,
including a too-small `cap`.

**Step 4 (`kMaxEncodedSize` static_asserts)**: `kCommandEnvelopeMaxEncodedSize`
/ `kReplyEnvelopeMaxEncodedSize` are `constexpr uint16_t` in `wire.h`
(public, so `BinaryChannel`/ticket 007 can also use them for buffer
sizing), each guarded by its own `static_assert(... <= 186, ...)`.
Recomputed by `gen_messages.py` on every regeneration, which every `just
build`/`just build-sim` already runs — a future schema change that grows
an envelope past 186B fails the BUILD, not a runtime check.

### Echo.payload bytes fix (Step 4 / ticket 001's flagged deviation)

Fixed as described: `bytes payload = 1 [(max_count) = 64]` previously fell
through `_emit_message()`'s generic scalar branch and emitted a ONE-byte
`uint8_t payload = 0;`. Added an explicit `field.type == _TYPE_BYTES`
branch (mirroring the existing repeated-field array+count shape) that now
emits `uint8_t payload_[64] = {}; uint8_t payload_count = 0;` plus the
matching `payload()`/`payload_count_val()` accessor pair. `wire.cpp`'s new
`FieldKind::kBytes` handles the length-delimited decode (copy up to
`min(payloadLen, cap)` bytes, clamp `count`, still consume the FULL
declared payload length so a subsequent field's position is never
corrupted) and encode (omit if `count==0`, matching implicit presence).
Verified round-trip in the harness (`scenarioRoundTripStopPingEcho`, a
5-byte `"hello"` payload).

### Open Questions

**OQ1 (resolved)**: `float`, as recommended — documented as the first
substantive paragraph of `wire.h`'s header comment (flash-budget
reasoning restated there).

**OQ3**: `ReplyEnvelope{DeviceId}` worst-case = **162B** (see the
discrepancy note above for why this differs from the ticket's own "~171B"
estimate) — comfortably under 186B, 18B of headroom.

### Verification performed

- `python scripts/gen_messages.py --dry-run` -> clean (protoc emitted a
  now-resolved "unused import" warning until `import "planner.proto"` was
  removed; no other proto-syntax issues).
- Real `python scripts/gen_messages.py` -> `envelope.h`/`layout_checks.h`
  change exactly as described above; every other pre-existing header
  byte-identical; `wire.{h,cpp}` newly written.
- Standalone host compile (`c++ -std=c++20 -Wall -Wextra -fno-exceptions
  -fno-rtti -I source -c source/messages/wire.cpp`) -> clean, zero
  warnings.
- `uv run python -m pytest tests/sim/unit/test_wire_codec.py -q` ->
  **2 passed** (normal build + a full ASan/UBSan recompile/rerun of every
  scenario, including the repeated-message max_count-clamp scenario whose
  output array is sized to EXACTLY `max_count` — any real overflow would
  abort under ASan).
- `just build` (ARM, `arm-none-eabi-g++` 15.2.1) -> green. **FLASH
  83.67% (311868 B / 364 KB) — UNCHANGED from ticket 004's own reported
  83.67%.** This is expected, not a miscount: the ARM build links with
  `-ffunction-sections -fdata-sections` + `--gc-sections` (confirmed in
  `libraries/codal-microbit-v2/target.json`), and nothing in `source/`
  calls `msg::wire::decode`/`encode` yet this sprint (`BinaryChannel`,
  ticket 007, is what wires a live call site) — so the linker strips
  `wire.cpp`'s new `FieldDesc` tables and walker functions as dead code,
  the same way `layout_checks.cpp`'s asserts show up in neither FLASH nor
  RAM despite compiling. The issue's own "+12-15KB dual-stack" flash
  estimate is expected to land once ticket 007 gives this code a live
  caller, not this ticket. RAM 98.33% (by design on this target, not a
  regression signal — `.clasi/knowledge/codal-ram-always-near-full.md`).
- `just build-sim` -> green, `wire.cpp.o` links into `libfirmware_host`
  cleanly.
- `uv run python -m pytest tests/sim -q` -> **62 passed** (the
  004-established 60 plus this ticket's 2 new `test_wire_codec.py`
  tests).
- `uv run python -m pytest tests/unit -q` -> **12 passed** (unaffected —
  includes the getter-regression guard).

## Testing (verification commands run)

- `uv run python -m pytest tests/sim/unit/test_wire_codec.py -q`
- `uv run python -m pytest tests/sim -q`
- `uv run python -m pytest tests/unit -q`
- `just build`
- `just build-sim`
