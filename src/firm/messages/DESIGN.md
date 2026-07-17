---
root: ../DESIGN.md
---

# Messages (`src/firm/messages`)

**Owner:** Eric Busboom · **Last reviewed:** 2026-07-17 · **Status:** in-flux

---

## 1. Purpose

`messages/` is the firmware's wire schema: the C++ shape of every message
that crosses the host/robot boundary, plus the codec that turns those shapes
into and out of bytes on the armored serial/radio link. It exists as its own
directory because it is a **leaf library with no project dependencies of its
own** (see root [DESIGN.md](../DESIGN.md) §2 dependency diagram) — `app/`
depends on it to talk to the host, `config/` depends on it for boot-config
shapes, but it depends on neither, and it must never depend on `devices/`
(the isolation invariant, root doc §3). No other directory owns "what a
`msg::Twist` looks like on the wire" or "how a `CommandEnvelope` is
decoded" — that ownership lives here alone.

## 2. Orientation

Three layers, in dependency order:

1. **Generated message structs** — one header per `protos/*.proto` file
   (`common.h`, `motion.h`, `motor.h`, `drivetrain.h`, `planner.h`,
   `gripper.h`, `sensors.h`, `ports.h`, `communicator.h`, `config.h`,
   `telemetry.h`, `envelope.h`, `odometer.h`). Each declares plain
   standard-layout `msg::*` structs with default member initializers — no
   heap, no STL containers, no virtual functions. `envelope.h` is the root:
   it declares `CommandEnvelope`/`ReplyEnvelope`, the two message types
   every wire line actually carries.
2. **Generated table-driven codec** — `wire.h`/`wire.cpp`. Declares/defines
   `msg::wire::decode(CommandEnvelope&, ...)` and
   `msg::wire::encode(ReplyEnvelope|TelemetrySecondary, ...)`, which walk
   per-message `FieldDesc` tables (field number, wire type, byte offset,
   bounds) emitted into `wire.cpp` to decode, encode, and validate
   (`(min)`/`(max)`/`(abs_max)`/`(req)`, see `protos/options.proto`) every
   message reachable from `CommandEnvelope`/`ReplyEnvelope`/
   `TelemetrySecondary`.
3. **Hand-written, schema-agnostic byte primitives** — `wire_runtime.h`/
   `wire_runtime.cpp`. The one hand-authored file pair in this directory:
   raw protobuf-wire-format primitives (varint, zigzag, fixed32/float,
   length-delimited framing, packed-repeated arrays, unknown-field skip,
   base64) that know nothing about field numbers, message shapes, or `msg::`
   types at all. Layer 2 is built on top of these; layer 2 owns the schema
   knowledge, layer 3 owns the bytes.

`layout_checks.h`/`layout_checks.cpp` sit alongside these as a **generated**
build-time gate (see §3) rather than a fourth layer: they exist to prove the
precondition layer 2's offsetof-based tables depend on, not to do any work
of their own.

The generation pipeline: `scripts/gen_messages.py` reads `protos/*.proto`
(plus `protos/options.proto`'s custom field-option extensions —
`units`/`max_count`/`min`/`max`/`abs_max`/`req`/`str_len`) via `grpcio-tools`
on the host, and emits one header per proto into this directory, plus
`wire.h`/`wire.cpp` and `layout_checks.{h,cpp}`. It runs as a codegen step
before every `just build`/`just build-sim` (via `build.py`) — never at
firmware runtime; the device itself never sees protobuf. It also emits
`docs/design/message-inventory.md` (traceability table) with
`--emit-inventory`.

## 3. Constraints and Invariants

- **Generated files are never hand-edited.** Every file in this directory
  carries an `// AUTO-GENERATED — do not edit by hand.` header banner
  **except** `wire_runtime.h` and `wire_runtime.cpp` — confirmed by reading
  every file's header. This includes `layout_checks.h`/`layout_checks.cpp`,
  which are also generated (despite the name suggesting a hand-maintained
  gate). A hand edit to any generated file is silently destroyed the next
  time `scripts/gen_messages.py` runs (every build). Any fix belongs in the
  generator, never in the emitted header.
- **Wire schema changes go through `protos/` + the generator, not this
  directory.** Adding, removing, or reshaping a field means editing the
  relevant `protos/*.proto` (and, for a new bound/width/requiredness,
  `protos/options.proto`'s extension), then regenerating — never editing a
  generated `.h`/`.cpp` to "just add the field."
- **`wire_runtime.*` must stay schema-agnostic.** No `#include` of any other
  `messages/*.h` header and no naming of a `msg::` type, anywhere in
  `wire_runtime.h`/`wire_runtime.cpp`. This is what lets it be the one
  layer that never regenerates: schema knowledge (field numbers, offsets,
  bounds) belongs entirely to the generated `wire.{h,cpp}` layer built on
  top of it.
- **Encode/decode never partially write or read on failure.** Every
  `encode*` function in `wire_runtime` either fully writes its value inside
  `[*pos, cap)` and advances `*pos`, or returns `false` and leaves `*pos`
  unchanged. Every `decode*` function either fully reads a valid value and
  advances `*pos`, or returns `false` and leaves `*pos` unchanged, never
  reading at or past `buf[len]`. This property is what the malformed-input
  acceptance tests (truncated varint, over-claiming length-delimited field,
  bad base64 padding) verify under ASan/UBSan — breaking it turns a
  malformed wire line into an out-of-bounds read/write instead of a clean
  rejection.
- **Base64 alphabet is pinned to standard (RFC 4648 `+/`), not URL-safe.**
  Both sides of the `*B<base64>` armor must agree; the host's
  `base64.b64encode`/`base64.b64decode` default to this same alphabet. There
  is no negotiation and no version byte — whichever alphabet
  `wire_runtime.cpp` encodes/decodes with **is** the wire format. Changing
  it on only one side breaks every armored line silently (garbage decode,
  not an error).
- **Struct layout must stay standard-layout.** Every `msg::*` struct
  reachable from `CommandEnvelope`/`ReplyEnvelope`/`TelemetrySecondary` must
  satisfy `std::is_standard_layout` — this is what makes the generated
  `wire.cpp` field tables' `offsetof()` calls well-defined. `layout_checks.h`
  is the generated build-time gate that proves this for the current schema;
  it emits no field table and no `offsetof` call itself, only the
  `static_assert`s. A schema change that breaks standard-layout (e.g. adding
  a virtual function, a non-standard-layout member, or multiple access
  specifiers in a way that violates the rule) fails the build here rather
  than corrupting offsets silently at runtime. Every struct these tables
  index into is standard-layout but not *trivial* (every field carries a
  default member initializer); `offsetof` on such a type is
  conditionally-supported under strict C++11/C++14 wording but
  unconditionally well-defined from C++17 onward, and this project's actual
  compiled standard is `-std=gnu++20` (the CMake targets override the
  vendored CODAL target's nominal C++11 pin) — so this is standard-guaranteed
  here, not merely "GCC/Clang define it in practice."
- **Envelope size is bounded and checked at compile time.** `wire.h`
  declares `kCommandEnvelopeMaxEncodedSize`/`kReplyEnvelopeMaxEncodedSize`/
  `kTelemetrySecondaryMaxEncodedSize` — the worst-case encoded size of the
  largest oneof arm in each envelope, computed by the generator from the
  schema's own field widths (max, not sum, across mutually exclusive oneof
  arms) — each checked at build time against a 186-byte envelope budget.
  A schema change that pushes an envelope over budget fails a
  `static_assert` at build time, not silently at runtime on a truncated wire
  line. As of 109-003: `ReplyEnvelope` is 178B (`Move` alone added `Move`
  as a NEW `CommandEnvelope` oneof arm, `CommandEnvelope` now 115B).
- **A `(max)`/`(abs_max)` bound now narrows a VARINT field's worst-case wire
  width, not just a `float` field's semantic range** (109-003 —
  `gen_messages.py`'s `_worst_case_scalar_size()`; previously this docstring
  said "a future bounded VARINT field would need this revisited" — this
  ticket is that future). `AckEntry.err_code`'s `(max) = 7` (its real
  domain — `ErrCode`'s own highest enumerator) and `Telemetry.queue_depth`'s
  `(max) = 8` (the ring's own real depth) are both ACCURATE bounds, not
  artificial shrinks — narrowing them bought back the wire budget the
  three new `Telemetry` fields (`queue_depth`/`active_id`/`exec_state`)
  spent. This is a size-estimation optimization only: the runtime encoder
  never clamps or rejects a value exceeding its declared bound, it just
  costs more bytes than the worst-case table assumed for that one frame,
  and `msg::wire::encode()`'s own capacity check means that rare case
  safely skips sending the frame rather than corrupting a buffer.
- **Bounds are stored as `float`, not `double`.** `FieldDesc.minVal`/
  `maxVal`/`absMaxVal` in the generated `wire.cpp` tables are `float` (4
  bytes) even though `protos/options.proto`'s `(min)`/`(max)`/`(abs_max)`
  extensions are declared `double` — this halves the flash cost of the
  field tables and matches the type every generated scalar field itself
  uses; no schema field needs more than `float` precision for a bound. This
  was a deliberate day-one decision, not an oversight — do not "fix" it back
  to `double` without re-justifying the flash cost.
- **No `sint32`/`sint64` (zigzag) fields exist in the schema today** — every
  signed/bounded quantity is a protobuf `float` (fixed32 wire type), not a
  zigzag-mapped integer. `wire_runtime`'s zigzag functions are implemented
  anyway as a cheap, standard primitive for a future schema addition; they
  are currently unused. Confirm this is still true before assuming
  zigzag is dead code to delete.
- **Length-delimited nesting is capped at depth 8** (`kMaxNestingDepth`) —
  small-constant headroom over the schema's actual deepest chain today
  (`CommandEnvelope → *Command → WheelTargets → repeated WheelTarget`, 3
  levels). Guards against unbounded recursion in the generated decoder on a
  malformed or maliciously over-nested input.
- **Generated `get_*` accessors are non-conforming and slated for
  generator-side removal**, per stakeholder decision
  (`clasi/issues/remove-generated-get-accessors.md`, referenced from root
  [DESIGN.md](../DESIGN.md) §6) — the trivial protobuf-style `get_kind()`/
  `get_ax()` style accessors are unused and violate the no-uppercase-start,
  lowerCamelCase function naming rule as `get_`-prefixed snake_case. As of
  this review, no such `get_*`-prefixed accessor appears in the currently
  generated headers in this directory (the "array / optional-string
  accessors" section instead emits bare-name accessors like
  `stops()`/`stops_count_val()` in `planner.h`) — either this was already
  addressed in the generator, or the issue predates the current schema. Any
  future generator change reintroducing `get_`-prefixed accessors must not
  ship; fixes go in `scripts/gen_messages.py`, never in a generated header.

## 4. Design

**Why a codec built on primitives, not a direct protobuf library.** The
firmware target is CODAL/`-fno-exceptions -fno-rtti`, newlib-nano, no heap —
incompatible with a general-purpose protobuf runtime. `wire_runtime` supplies
exactly the wire-format primitives this schema needs (varint, zigzag,
fixed32, length-delimited framing, packed-repeated, unknown-field skip,
base64) with caller-owned buffers and no allocation; the generated `wire.cpp`
then supplies only the schema-specific knowledge (field numbers, struct
offsets, bounds) as data tables that a small generic walker interprets. This
split is why `wire_runtime` never regenerates and never needs to.

**Two structurally different oneof shapes.** `CommandEnvelope`/
`ReplyEnvelope` model a proto3 `oneof` as a tagged union: a `*Kind` enum
discriminant (`cmd_kind`/`body_kind`) plus a C `union` of the arm structs.
The generated codec picks the union member to decode/encode from the
discriminant tag on the wire, not from a `oneof`-aware runtime type. `msg::
wire::encode()` for `ReplyEnvelope` walks only the currently-selected `body`
arm — proto3 implicit presence means a plain scalar field equal to its zero
default is omitted from the wire entirely, matching a real protobuf
encoder's byte-for-byte output (verified against `google.protobuf` by a
differential fuzz suite).

**`TelemetrySecondary` rides its own armored line.** Unlike `ReplyEnvelope`'s
oneof arms, `TelemetrySecondary` is encode-only and never a `ReplyEnvelope`
oneof arm — it is the slower diagnostic frame, firmware-emitted only, never
host-decoded on the robot side, framed as its own independently-armored `*B`
line (see `telemetry.h`'s own doc comment and root
[DESIGN.md](../DESIGN.md) §4 "Command plane").

**Unknown fields are forward-compatible by design.** `skipField()`
advances past an unrecognized field number's value without interpreting it,
letting an older decoder round-trip a newer schema's added field (or a
declared-but-unused oneof arm) without erroring. It never recurses into a
length-delimited payload's structure — an opaque byte-range skip — so it
cannot trip the nesting-depth guard regardless of how deep the caller
already is.

**`main.cpp` is the one place `msg::*` types meet `Devices::*` types.** Per
the devices-isolation invariant (root doc §3), `messages/` types never reach
`devices/`; `main.cpp` converts wire-plane `msg::MotorConfig` to
`Devices::MotorConfig` at construction time. This directory has no part in
that conversion — it only defines the wire-side shape.

## 5. Interfaces

### Exposes

- **`msg::wire::decode(CommandEnvelope& out, const uint8_t* buf, uint16_t
  len) -> Result`:** decodes and validates one `CommandEnvelope` from a raw
  (already base64-decoded) byte buffer. `Result{ok, field, code}`: `ok` is
  false on the first violation encountered (missing `(req)` field,
  out-of-bound value, or malformed wire bytes), `field` names which field
  number, `code` is an `ErrCode` (see `envelope.proto`'s doc comment for
  which code means which). Never partially decodes into `out` on failure in
  a way the caller should trust.
- **`msg::wire::encode(const ReplyEnvelope& in, uint8_t* buf, uint16_t cap)
  -> uint16_t`** and the `TelemetrySecondary` overload: encode into `buf`,
  return the number of bytes written, or `0` if `cap` is smaller than the
  required output (never a truncated/corrupt partial buffer).
- **`WireRuntime::*` primitives** (`wire_runtime.h`): the encode/decode
  never-partial contract described in §3, for any future hand-written
  caller needing raw protobuf-wire-format bytes without going through the
  generated schema layer.
- **Generated `msg::*` structs:** the wire-schema shape itself — every
  field, enum, and nested oneof union other subsystems construct, read, and
  pass to `App::Comms`/`App::Telemetry`. Authoritative source for these
  shapes is the corresponding `protos/*.proto` file, not this document.

### Consumes

- **`protos/*.proto` and `protos/options.proto`** (via `scripts/
  gen_messages.py`, host-only, `grpcio-tools`) — the schema source of
  truth; see root [DESIGN.md](../DESIGN.md) §5 "Build-time generators."
- **`app/` (via `App::Comms`/`App::Telemetry`):** the only consumer of the
  decode/encode entry points at runtime — see
  [app/DESIGN.md](../app/DESIGN.md) for how a decoded `CommandEnvelope`
  reaches the loop's dispatch and how a `ReplyEnvelope`/
  `TelemetrySecondary` gets armored and sent.
- **`config/`:** consumes generated `msg::*Config`/`msg::*ConfigPatch`
  shapes declared here for baked boot configuration — see
  [config/DESIGN.md](../config/DESIGN.md).

## 6. Open Questions / Known Limitations

- **`event.h` (`msg::Event`) is hand-written but not generated, and is not
  referenced anywhere in the live `src/firm` tree** — the only other
  references found are in `protos/planner.proto`/`planner.h` (a distinct,
  generated type, not `msg::Event`) and in archived/parked source
  (`src/archive/source_parked/094/subsystems/planner.cpp`,
  `src/tests/sim/parked-094/...`). Its header comment describes a role
  ("lets both a subsystem producer and `CommandProcessor` depend on the same
  type") that predates the single-loop rebuild — `CommandProcessor` and
  `Subsystems::Planner` no longer exist in this tree (deleted sprints
  102-107, root [DESIGN.md](../DESIGN.md) §2). This looks like dead code
  left over from the pre-rebuild architecture; confirm before either wiring
  it to a live producer or deleting it. Not touched by this review beyond
  comment trimming (see report).
  **Resolved for the 109-003 use case specifically** (sprint 109 `sprint.md`
  Open Question 3: should `Motion::Executor`'s new per-command completion
  events — `DONE`/`TRIVIAL`/`SUPERSEDED`/`FLUSHED`/`TIMEOUT`/`SOLVE_FAIL` —
  ride `event.h`, or the existing reply/TLM path?) — **the existing ack ring
  wins**: `telemetry.proto`'s `AckStatus` enum gained the six completion
  values above, riding the SAME depth-3 `Telemetry.acks` ring every
  `TWIST`/`CONFIG`/`STOP` ack already uses, rather than reviving a second,
  parallel, hand-written notification channel with no other live consumer.
  `event.h` itself remains untouched, unreferenced dead code — this
  resolution does not un-orphan it, it just answers "where do 109-003's own
  new events go" without reopening that separate question.
- **`get_*` accessor removal issue may already be moot** — see §3's note;
  worth confirming against `clasi/issues/remove-generated-get-accessors.md`
  whether that issue is stale/already resolved or still pending against a
  future generator change.
- **`docs/design/message-inventory.md`** (the `--emit-inventory` output) was
  not verified as current against today's `protos/*.proto` set as part of
  this review.
