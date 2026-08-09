---
root: ../../../docs/design/design.md
---

# Messages (`src/firm/messages`)

**Owner:** Eric Busboom · **Last reviewed:** 2026-07-25 · **Status:** in-flux

---

## 1. Purpose

`messages/` is the firmware's wire schema: the C++ shape of every message
that crosses the host/robot boundary, plus the codec that turns those shapes
into and out of bytes on the COBS+CRC-framed serial/radio link (sprint 123 —
replaces the pre-123 base64 line armor, see §3/§4 below). It exists as its own
directory because it is a **leaf library with no project dependencies of its
own** (see the system doc's dependency diagram, `docs/design/design.md`
§5) — `app/` depends on it to talk to the host, `config/` depends on it
for boot-config shapes, but it depends on neither, and it must never
depend on `devices/` (the isolation invariant, `docs/design/design.md`
§5). No other directory owns "what a
`msg::Move` looks like on the wire" or "how a `CommandEnvelope` is
decoded" — that ownership lives here alone.

## 2. Orientation

Three layers, in dependency order:

1. **Generated message structs** — one header per `protos/*.proto` file
   (`common.h`, `motor.h`, `drivetrain.h`, `gripper.h`, `sensors.h`,
   `ports.h`, `communicator.h`, `config.h`, `telemetry.h`, `envelope.h`,
   `odometer.h`). `motion.proto`/`motion.h` and `planner.proto`/`planner.h`
   no longer exist (115-002/115-003, gut-to-minimal-firmware S1
   motion-stack excision deleted their only consumers — the motion stack
   and `Core::Pilot`'s `PlannerConfig`). Each remaining header declares plain
   standard-layout `msg::*` structs with default member initializers — no
   heap, no STL containers, no virtual functions. `envelope.h` is the root:
   it declares `CommandEnvelope`/`ReplyEnvelope`, the two message types
   every wire line actually carries.
2. **Generated table-driven codec** — `wire.h`/`wire.cpp`. Declares/defines
   `msg::wire::decode(CommandEnvelope&, ...)` and
   `msg::wire::encode(ReplyEnvelope&, ...)`, which walk per-message
   `FieldDesc` tables (field number, wire type, byte offset, bounds)
   emitted into `wire.cpp` to decode, encode, and validate
   (`(min)`/`(max)`/`(abs_max)`/`(req)`, see `protos/options.proto`) every
   message reachable from `CommandEnvelope`/`ReplyEnvelope`. (Through
   sprint 123, a THIRD top-level message, `TelemetrySecondary`, had its
   own `encode()` overload here too — sprint 124 ticket 009 deleted
   `TelemetrySecondary` outright, so `ReplyEnvelope` is the only outbound
   top-level message now; see §3/§4 below.)
3. **Hand-written, schema-agnostic byte primitives** — `wire_runtime.h`/
   `wire_runtime.cpp`. The one hand-authored file pair in this directory:
   raw protobuf-wire-format primitives (varint, zigzag, fixed32/float,
   length-delimited framing, packed-repeated arrays, unknown-field skip,
   base64) PLUS, as of sprint 123, two more schema-agnostic byte
   primitives at this same layer: COBS frame encode/decode
   (`cobsEncode()`/`cobsDecode()`) and CRC-16/CCITT-FALSE compute/verify
   (`crcCompute()`/`crcVerify()`/`encodeCrc16()`/`decodeCrc16()`) — the
   wire's current binary framing + integrity check (§3/§4). None of these
   primitives know anything about field numbers, message shapes, or
   `msg::` types at all. Layer 2 is built on top of these; layer 2 owns
   the schema knowledge, layer 3 owns the bytes.

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
- **HISTORICAL/SUPERSEDED (pre-123): base64 alphabet was pinned to standard
  (RFC 4648 `+/`), not URL-safe, because base64 was the wire's armor.**
  Sprint 123 replaced base64 armor with COBS+CRC framing (see the next
  bullet) — `Comms`/`Telemetry` no longer encode or decode base64 at all.
  `WireRuntime::base64Encode()`/`base64Decode()` themselves are RETAINED,
  not removed, purely because `wire_differential_harness.cpp`
  (`src/tests/sim/unit/`) independently depends on the primitive for its
  own, unrelated debug-CLI wire encoding — a consumer with no connection
  to the armor scheme. This invariant no longer governs the live wire;
  kept here as the historical record of what the pre-123 armor required.
- **CRC-16/CCITT-FALSE variant is pinned exactly, with no negotiation and
  no version byte (sprint 123, the CURRENT wire integrity check).** Both
  sides of every COBS+CRC frame must agree byte-for-byte: poly `0x1021`,
  init `0xFFFF`, no input/output reflection, no final XOR —
  `wire_runtime.h`'s file header names this the CRC RevEng catalogue's
  "CRC-16/CCITT-FALSE," with a known-answer test vector
  (`crcCompute("123456789", 9) == 0x29B1`). `src/host/robot_radio/
  io/wire_codec.py` ports the identical variant to Python. Changing the
  variant on only one side breaks every frame's CRC check silently
  (every frame appears corrupt and is dropped, not decoded).
  **124-003 (CRC scope extension):** the *variant* above is unchanged —
  `crcCompute()` still computes exactly the same value it always did —
  but the CRC's *input range* now extends to `COMMAND ':' payload`
  instead of `payload` alone, closing the "a bit-flip inside the command
  name lands on another valid verb and dispatches with a passing CRC"
  gap. `WireRuntime` gained an incremental `crcInit()`/`crcUpdate()` pair
  (`crcCompute(data, len) == crcUpdate(crcInit(), data, len)`, same loop
  body) so a caller can fold a command-name prefix and a payload — two
  byte ranges that are not adjacent in memory — into one CRC without
  concatenating them into a scratch buffer first. `WireRuntime` itself
  still knows nothing about a command name or a `':'` separator — that
  composition is `Comms`'s boundary (`comms.cpp`'s `crcOverScope()`) /
  `wire_codec.py`'s (`_crc_over_scope()`), not `WireRuntime`'s, per this
  file's own three-layer split. **124-005 (grammar cutover — landed):**
  every call site now passes the real, parsed `<COMMAND>` ASCII bytes as
  the scope — `Comms::crcOverScope()` (`comms.cpp`) composes
  `crcUpdate(crcInit(), command, commandLen)` then folds in the `':'`
  separator and the payload via a second `crcUpdate()` call, on both the
  encode side (`sendReply()`, deriving the command name from `reply.
  body_kind`) and the decode side (`decodeBinaryFrame()`, handed the
  command bytes `dispatchLine()` already parsed off the line);
  `wire_codec.py`'s `_crc_over_scope()` mirrors this exactly in Python.
  The 124-003-era "every call site passes an empty scope" state above is
  now historical — every CRC on the live wire is scoped to its own
  command name, closing the "a bit-flip inside the command name lands on
  another valid verb and dispatches with a passing CRC" gap this bullet's
  own opening sentence describes.
- **COBS is the wire's current framing** (sprint 123, replacing base64
  armor). A COBS-encoded frame body never contains an embedded byte equal
  to the delimiter it was encoded against, by construction — that byte is
  reserved exclusively as the trailing frame delimiter the transport
  appends. **AS OF 124-005 (current):** the live delimiter is `0x0A`
  (`'\n'`, see the "124-005 (grammar cutover)" note below), so a
  COBS-encoded body is `0x0A`-free but MAY legitimately contain an
  embedded `0x00` byte — the inverse of the sprint-123-era statement this
  bullet originally made (`0x00`-free, `0x0A` reserved as the two-terminator
  demux's OWN split byte). `Core::FrameKind` (`src/firm/core/comms.h`,
  cited by the pre-124-005 wording here) no longer exists — see
  [`../app/DESIGN.md`](../app/DESIGN.md) §1 and
  [`../com/DESIGN.md`](../com/DESIGN.md) §2 for the uniform-grammar
  replacement. See `docs/protocol-v5.md` §2 for the full frame layout and
  byte-budget derivation.
  **124-003 (delimiter parameterization):** `cobsEncode()`/`cobsDecode()`
  gained a trailing `delimiter` byte parameter (default `0x00`, every
  pre-124 call site unaffected). The mechanism is an XOR of every output
  byte (encode) / every input byte at the point of reading (decode) by
  `delimiter` — sound because `cobsEncode()`'s pre-XOR output is
  guaranteed `0x00`-free by construction, and `b ^ delimiter ==
  delimiter` iff `b == 0`, which never occurs, so the XOR-ed stream can
  never contain `delimiter` either. 124-003 itself only landed the
  parameterized primitive — every call site still keyed on `0x00`.
  **124-005 (grammar cutover — landed):** `Core::kCobsDelimiter` (`comms.h`)
  is now `0x0A`, and every `cobsEncode()`/`cobsDecode()` call site in
  `Comms` (`sendReply()`, `decodeBinaryFrame()`, `Telemetry::
  emitSecondary()`) and in `wire_codec.py` (`encode_frame()`/
  `decode_frame()`, hard-coded `delimiter=0x0A`) passes it explicitly —
  the live wire delimiter and the `\n` grammar terminator (issue §1/§2)
  are now the SAME byte, which is what makes `\n` a genuine,
  unconditional line terminator for `com/`'s transports (see
  [`../com/DESIGN.md`](../com/DESIGN.md) §2): a COBS-encoded frame body
  can never contain a literal `0x0A` by construction, so it can never be
  mistaken for a line boundary. `wire_runtime.h`'s own primitive-level
  default parameter stays `0x00` (unchanged — some non-live callers, e.g.
  `wire_differential_harness.cpp`'s unrelated debug-CLI encoding, still
  want that default); only the *live wire's* callers pass `0x0A`.
  (`Telemetry::emitSecondary()`, the third call site this bullet used to
  list alongside `sendReply()`/`decodeBinaryFrame()`, no longer exists —
  124-009 deleted `TelemetrySecondary` outright, see §4 below.)
- **Struct layout must stay standard-layout.** Every `msg::*` struct
  reachable from `CommandEnvelope`/`ReplyEnvelope` (through sprint 123,
  also `TelemetrySecondary` — deleted 124-009, see §4) must
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
  declares `kCommandEnvelopeMaxEncodedSize`/`kReplyEnvelopeMaxEncodedSize`
  (through sprint 123, also `kTelemetrySecondaryMaxEncodedSize` — that
  constant and the message it sized are DELETED by 124-009, see §4) — the
  worst-case encoded size of the largest oneof arm in each envelope, computed by the generator from the
  schema's own field widths (max, not sum, across mutually exclusive oneof
  arms) — each checked at build time against a **240-byte** envelope
  budget (sprint 123 recompute — see below; **186 bytes pre-123**).
  A schema change that pushes an envelope over budget fails a
  `static_assert` at build time, not silently at runtime on a truncated wire
  line.

  **123-002 (COBS+CRC budget recompute).** The 186-byte ceiling was sized
  against the pre-123 base64 armor's ~33% expansion, keeping an armored
  `"*B" + base64(...) + "\r\n"` line under the CODAL serial TX ring's
  254-usable-byte hard ceiling (`SerialPort::begin()`'s
  `setTxBufferSize(255)`, a `uint8_t` max). COBS+CRC framing's overhead is
  fixed instead of proportional: CRC (2 B) + one COBS code byte (0 extra
  blocks below the 254-byte block boundary) + the trailing `0x00`
  delimiter = 4 B total, independent of payload content. Solving
  `E + 4 <= 254` gives `E <= 250`; the budget ships at **240**, 10 B of
  margin below that edge — comfortably above every schema value this
  sprint computes, including ticket 004's `cycle_busy`/`cycle_period`
  primary-frame migration (194 B largest, up from 185 B pre-migration —
  the whole reason this budget needed recomputing). See
  `docs/protocol-v5.md` §2 for the current derivation and size table (this bullet describes the sprint-123 recompute historically -- see the 124-008 bullet below for the current ≤130B gate).

  As of 116-001 (MOVE protocol cutover — see
  [envelope.proto](../../protos/envelope.proto)'s own header comment):
  `CommandEnvelope` is still 50B (`cmd` oneof = `{config, stop, move}` —
  `twist` (arm 19, the bare v_x/omega/duration shape 103-001 added) is
  DELETED and reserved this ticket, superseded by `move` (a fresh arm 21:
  `MoveTwist|MoveWheels` velocity oneof + `time|distance|angle` stop oneof +
  `timeout`/`replace`/`id` — see that message's own doc comment); the old
  arc-command `Move` sprint 109 added stays deleted, its field number `20`
  reserved, never reused, never the same shape as this `move`). Per-arm
  worst case: `config`=44B, `stop`=2B, `move`=38B — `config` (dominated by
  the drivetrain live-tuning message) stays the largest arm even though `move` is a
  structurally bigger message (two nested oneofs + `id`) than the `twist`
  it replaced, so the envelope's own total is UNCHANGED at 50B despite the
  schema growing. `ReplyEnvelope` is 153B (dominated by the `tlm` arm's
  `EncoderReading`×2/`OtosReading`/packed line+color payload — 33B margin
  under budget, 26B *smaller* than the pre-115 178B ack-ring/bool-flag
  frame despite carrying strictly more signal; untouched by this ticket,
  which only edits `CommandEnvelope`'s `cmd` oneof and `ConfigDelta`'s
  `patch` oneof — see `wire.h`'s own generated size-report comment), and
  `TelemetrySecondary` is 52B (also untouched). **117 ticket 003** added
  `ConfigDelta.estimator` (the estimator live-tuning message, `estimator = 6`, the next
  free number after `reserved 3, 4` and `otos = 5`) plus
  `ConfigTarget.CONFIG_ESTIMATOR = 6` (`config.proto`) — that message
  carries 3 optional floats (`weight_heading_otos`/`weight_omega_otos`/
  `staleness_ms`), smaller than the drivetrain live-tuning message's 8, so `ConfigDelta`'s
  own worst-case arm (and therefore `CommandEnvelope`'s `config`=44B and its
  50B total) is **unchanged** — re-measured against the regenerated
  `wire.h`'s size-report comment, not assumed.

  **SUPERSEDED, 132-013 (patch-surface retirement):** `ConfigDelta` and
  every curated per-target live-tuning message described in this whole
  passage (drivetrain/motor/otos/estimator, `config.proto`) are deleted
  outright — `CommandEnvelope.config` now carries `SetConfigGroup`
  (`robot_config.proto`, ~220B `body`), not `ConfigDelta`. This retires
  the entire "config is the small arm" framing this passage's history
  documents: `config` is now the LARGEST oneof arm in `CommandEnvelope` by
  far. Current, regenerated ground truth as of 132-013: `CommandEnvelope`
  total **234B** (`config`=228B, `move`=38B, `stop`=8B, `wheels`=24B,
  `estop`=3B, `get_config`=5B, `set_field`=16B), `ReplyEnvelope` total
  **232B** (`cfg`=228B is now the worst-case arm, just ahead of `tlm`) --
  see `wire.h`'s own generated size-report comment (never hand-calculated
  or assumed) and `app/comms.h`'s `kMaxEnvelopeBytes`/`kFramedMaxBytes`/
  `kMaxLineBytes` doc comments for the cascading COBS-framing/line-buffer
  consequences.

  **As of sprint 123 (current, regenerated ground truth — `wire.h`'s own
  header comment):** `CommandEnvelope`'s worst-case arm is `config`=49B
  (`stop`=2B, `move`=38B) for a **55B total** — 5B more than the 117-era
  44B/50B snapshot above from schema growth in an intervening sprint
  unrelated to wire framing (this ticket's own scope is the envelope
  BUDGET recompute and the loop-timing FRAME PLACEMENT, not
  `CommandEnvelope`'s own field shapes — see `wire.h` directly for
  exactly which field grew). `ReplyEnvelope`'s `tlm` arm now measures
  188B for a **194B total** — ticket 004's `cycle_busy`/`cycle_period`
  primary-frame migration, +9B over the 179B `tlm` arm above.
  `TelemetrySecondary` remained **52B** at that point in the sprint's own
  history — its former fields 11/12 were by then `reserved`, not
  populated (see `app/DESIGN.md` §4). Sprint 124 ticket 009 later deleted
  `TelemetrySecondary` outright (see below) — there is no third size to
  track any more. All totals sat comfortably under the 240-byte budget
  above.

  **124-008 (issue §B5, a NEW, tighter, explicit gate — `kReplyEnvelope
  MaxEncodedSize <= 130` bytes, separate from and stricter than the
  240-byte envelope budget above):** driven by the sint32+`(scale)`
  conversion (shrinks every position/velocity/pose/twist/OtOS field from a
  flat 4-byte `float` to a zigzag varint, usually 1-3 bytes at realistic
  magnitudes) and packed `acks` (one `uint32` per ring entry instead of a
  nested `AckEntry` sub-message's own tag+length+two-field overhead)
  bringing `ReplyEnvelope`'s `tlm` arm down from 188B to comfortably under
  130B — reached by iterating the REGENERATED `wire.h` size-report comment
  (never hand-calculated or assumed) and adding SIZING-ONLY bounds (this
  section's own "sizing-only" bullet above) to `EncoderReading.
  position_epoch` (127, not the field's raw 8-bit storage range), `now`
  (2097151), `seq` (127, and made genuinely wrapping in the real value —
  see the "sizing-only" bullet above), the packed `acks` word (1048575 ==
  `(65535 << 4) | 15`, the worst case for a 16-bit corr_id), and
  `ReplyEnvelope.corr_id` itself (65535 — required threading `import
  "options.proto"` into `envelope.proto`, which had never needed field
  options before this). Exact regenerated result at ticket-close: **130
  bytes** — confirm against `wire.h`'s own `kReplyEnvelopeMaxEncodedSize`
  before assuming this number is still current.
- **124-009: `TelemetrySecondary` — message, `encode()` overload,
  `kTelemetrySecondaryMaxEncodedSize`, and the wire arm/tie-break
  machinery that used to pick between it and the primary frame — is
  DELETED OUTRIGHT, not merely unused.** It emitted nothing but `now` in
  production (no firmware caller ever populated `has_cmd_vel`/`acc_*`/
  `glitch_*`/`ts_*`), and `RobotState ⊇ wire` (the blackboard issue's own
  framing, see [`../types/DESIGN.md`](../types/DESIGN.md)) means there is
  no wire-visibility invariant obligating any of its former fields to fold
  into the primary frame. `ReplyEnvelope`/`Telemetry` is the ONLY outbound
  top-level wire message now; every "reachable from
  `CommandEnvelope`/`ReplyEnvelope`/`TelemetrySecondary`" statement
  elsewhere in this document describes a PRE-124-009 state, called out
  inline where it appears.
- **A `(max)`/`(abs_max)` bound now narrows a VARINT field's worst-case wire
  width, not just a `float` field's semantic range** (109-003 —
  `gen_messages.py`'s `_worst_case_scalar_size()`; previously this docstring
  said "a future bounded VARINT field would need this revisited" — this
  ticket is that future). Two flavors of bound now coexist: an ACCURATE
  domain bound (e.g. `ack_err`'s pre-124-008 `(max) = 7`, `ErrCode`'s own
  highest enumerator at the time — now 8, `ERR_NOT_CONFIGURED`, and no
  longer a standalone field at all, folded into the packed ack word below)
  and a **SIZING-ONLY** bound (124-008, issue §B5's ≤130B gate): a bound
  that narrows the generator's worst-case estimate without the runtime
  ever enforcing it, added purely to make the size table's own assumption
  honest for a field `msg::wire::encode()` never validates on the way out
  (`Telemetry.now`, `.seq`, `.acks`, `ReplyEnvelope.corr_id` — see each
  field's own proto doc comment for why encode-only fields get this
  treatment; `seq_`, telemetry.cpp, is also made to genuinely wrap at its
  declared bound in the real running value, not just the wire table, so
  the declared bound and the true worst case never drift apart). Either
  way this is a size-ESTIMATION mechanic only: the runtime encoder never
  clamps or rejects a value exceeding its declared bound, it just costs
  more bytes than the worst-case table assumed for that one frame, and
  `msg::wire::encode()`'s own capacity check means that rare case safely
  skips sending the frame rather than corrupting a buffer.
- **Bounds are stored as `float`, not `double`.** `FieldDesc.minVal`/
  `maxVal`/`absMaxVal` in the generated `wire.cpp` tables are `float` (4
  bytes) even though `protos/options.proto`'s `(min)`/`(max)`/`(abs_max)`
  extensions are declared `double` — this halves the flash cost of the
  field tables and matches the type every generated scalar field itself
  uses; no schema field needs more than `float` precision for a bound. This
  was a deliberate day-one decision, not an oversight — do not "fix" it back
  to `double` without re-justifying the flash cost.
- **`sint32` (zigzag) fields exist in the schema as of 124-008** — every
  signed, bounded physical quantity (`Pose2D.x/y/h`, `BodyTwist3.v_x/v_y/
  omega`, `EncoderReading.position/velocity`, `OtosReading.x/y/heading/
  v_x/v_y/omega`) is now a protobuf `sint32` (zigzag varint wire type), not
  a `float` (fixed32) — `WireRuntime::zigzagEncode32()`/`zigzagDecode32()`
  (previously implemented but unused, per this bullet's pre-124-008
  wording) are now live, exercised by `ScalarType::kSint32`'s branches in
  `decodeScalarValue()`/`encodeScalarValue()`/`scalarIsDefault()`/
  `scalarAsDouble()` (`wire.cpp`'s generated engine). Motivation: a
  zigzag-mapped small-magnitude signed integer costs far fewer bytes than
  a `float`'s fixed 4 bytes (a value like `0` costs 1 byte; the full
  `(abs_max)`-bounded range still costs at most 3-5 bytes depending on the
  field, well under `float`'s flat 4) — this shrink is what made the
  124-008 ≤130B `ReplyEnvelope` gate reachable at all (see the size-budget
  update below) alongside packed acks. No `sint64` field exists; `int32`/
  `int64` (non-zigzag, sign-extending) fields also do not exist in the
  schema — every signed field uses `sint32` specifically. Confirm this is
  still accurate before assuming any of these three are still unused/dead.
- **A field's `(scale)` option (`protos/options.proto`, extension 50007,
  124-008) is a GENERATED conversion, not a documentation-only tag** —
  contrast `(units)`, which is purely informational and never generates
  code. Any non-oneof, non-repeated `sint32` field carrying `(scale)`
  causes `gen_messages.py`'s `_emit_message()` to additionally emit, on
  that field's own generated struct, a `static constexpr float k<Field>
  Scale`, a `static int32_t pack<Field>(float value)` (round-half-away-
  from-zero, not truncate — `value/scale + 0.5f` for positive, `- 0.5f`
  for negative), and a `static float unpack<Field>(int32_t raw)`
  (`raw * scale`). The wire codec ENGINE (`decodeScalarValue()`/
  `encodeScalarValue()` in `wire.cpp`) is completely scale-agnostic — it
  only ever moves a raw zigzag `int32_t` on and off the wire; `(scale)` is
  purely a header-generation-time concept, applied by every CALLER (e.g.
  `Core::RobotLoop::assembleFrame()`, `robot_loop.cpp`) via these generated
  pack/unpack methods, never inline division/multiplication by a
  hand-copied constant. `gen_messages.py` raises `GenMessagesError` if
  `(scale)` is declared on any non-`sint32` field — the option only makes
  sense paired with the zigzag integer representation it converts to/from.
  Struct MEMBERS stay `int32_t` (raw wire ticks) regardless — `(scale)`
  changes what the generator emits alongside a field, never the field's
  own storage type.
- **`Telemetry.acks` is a PACKED `repeated uint32`, not `repeated
  AckEntry`, as of 124-008 (issue §B4)** — the wire message type
  `msg::AckEntry` is DELETED (`telemetry.proto`'s own history: introduced
  120, deleted 124-008), along with `Telemetry.ack_corr`/`ack_err` (fields
  5/6, the pre-120 single "freshest ack" scalar slot these two fields
  duplicated) and `flags` bit 5 (`kFlagAckFresh`, its own freshness
  signal). Every entry is one wire `uint32`, packed
  `(corr_id << 4) | err` — 4 low bits for `err` (an `ErrCode`, whose real
  ceiling `ERR_NOT_CONFIGURED` = 8 needs exactly 4 bits), the remaining 28
  bits for `corr_id`/`Move.id` (in practice these need at most 16 bits on
  the wire, `envelope.proto`'s own `CommandEnvelope.corr_id`/`Move.id`
  doc comments — but see the caution below). `Core::Telemetry::
  pushAckRing()` (`telemetry.cpp`) is the one packer; `AckEntry::
  from_ring_entry()` (`src/host/robot_radio/robot/protocol.py`) is the
  one host-side unpacker — both duplicate the exact same `kAckErrBits=4`/
  `kAckErrMask=0xF` shift/mask locally (no shared header exports this
  formula across the C++/Python boundary; every other ring consumer in
  the test tree mirrors it the same way, by grep). This is NOT a special
  case in the generated codec engine — `FieldKind::kRepeatedScalar`'s
  packed encode/decode (`WireRuntime::decodePackedVarint()`/
  `decodePackedFixed32()`, plus a scratch-buffer packing loop in
  `encodeInto()`) already existed generically before this ticket; `acks`
  is simply its first real schema use. **Caution for any future caller
  choosing a `corr_id`/`Move.id` value:** the packed format's 28-bit
  budget for `corr_id` is a HARD ceiling — a value at or above `2**28`
  silently loses its high-order bits on `<< 4`, corrupting ack
  correlation with no error signal anywhere in this codec. This bit the
  project once already (124-008's own fix to
  `src/host/robot_radio/testgui/transport.py`'s `_MOVE_ID_BASE`, which
  picked `1 << 30` specifically to stay disjoint from small envelope
  `corr_id`s and overflowed the packed field on the very first Move a GUI
  session sent — fixed to `1 << 24`, still comfortably disjoint, still
  comfortably under `2**28`).
- **`Telemetry.flags`'s `(max)` bound was widened past 65535 (124-008,
  a real bug fix, not a cosmetic tidy)** — real usage already reaches bit
  16 (`kFlagFaultShapingDisabled`, 119-001) and this ticket adds bit 17
  (`kFlagFaultPositionClamped`), both well past the 16-bit ceiling a
  `(max) = 65535` sizing bound implied; the declared bound now reads
  `262143` (`(1 << 18) - 1`, covering both bits with headroom). Before
  this fix the sizing table UNDERESTIMATED `flags`'s own worst-case wire
  width relative to what the real running value could already produce —
  the opposite direction of every other sizing-bound fix in this ticket
  (those NARROW an honest overestimate; this one widens a bound that had
  gone stale relative to the real field).
- **The position-rebaseline policy (sprint 124 architecture Decision 6)
  lives in `app/robot_loop.cpp`, not here** — this directory's own stake
  in it is purely the WIRE SHAPE: `EncoderReading.position`'s `(abs_max) =
  32000` (mm) is the tight bound the policy exists to respect (storage
  side, `Devices::NezhaMotor::encOffset_`, is ~121km capacity and never
  needs it), and `EncoderReading.position_epoch` (ADDITIVE, field 4,
  `uint32` on the wire but 8 bits of real storage suffice per Decision
  6's own sizing call) is the counter `Core::RobotLoop` owns and increments
  each time it software-rebaselines a wheel via the EXISTING, unmodified
  `Devices::Motor::rebaseline()` — never `Motor::resetPosition()`, which
  can still choose a real bus-touching hard reset and is forbidden
  outright for this policy by stakeholder ruling. See
  [`../app/DESIGN.md`](../app/DESIGN.md) for the policy's own trigger/
  clamp mechanism. `EncoderReading.time` is RENAMED `age` (field 3,
  `(max) = 255` ms) in the same pass — an absolute robot-clock timestamp
  cannot be packed small (it grows for the whole session), so it could
  never fit this ticket's own ≤130B gate; `age` (the delta a sample's own
  collect time sits behind `Telemetry.now`) can. Production firmware
  emits `age = 0` as of 124-008 (a genuinely honest zero, not a stub —
  every reading is still stamped with the frame's own `now`); wiring real
  nonzero per-sample skew is a later ticket's work, not this one's.
- **`msg::wire::decode(Telemetry& out, const uint8_t* buf, uint16_t len)`
  is TEST-ONLY infrastructure, added 124-008** — production firmware only
  ever ENCODES a `Telemetry` (it is the outbound push, never something the
  robot itself decodes), but decoding one back is exactly what a test
  needs to exercise `validateBounds()` against `Telemetry`'s own bounded
  fields (`flags`, the packed `acks` word, …) the same way
  `decode(CommandEnvelope&, ...)` already does for the inbound side — see
  §5 "Exposes" below. Same `std::memset` + `decodeInto(&out, kTable_
  Telemetry, ...)` shape as every other generated `decode()` overload;
  nothing about it is hand-written or special-cased.
- **Length-delimited nesting is capped at depth 8** (`kMaxNestingDepth`) —
  small-constant headroom over the schema's actual deepest chain today
  (`CommandEnvelope → *Command → WheelTargets → repeated WheelTarget`, 3
  levels). Guards against unbounded recursion in the generated decoder on a
  malformed or maliciously over-nested input.
- **Generated `get_*` accessors are non-conforming and slated for
  generator-side removal**, per stakeholder decision
  (`clasi/issues/remove-generated-get-accessors.md`) — the trivial protobuf-style `get_kind()`/
  `get_ax()` style accessors are unused and violate the no-uppercase-start,
  lowerCamelCase function naming rule as `get_`-prefixed snake_case. As of
  this review, no such `get_*`-prefixed accessor appears in the currently
  generated headers in this directory — either this was already addressed
  in the generator, or the issue predates the current schema (the
  "array / optional-string accessors" bare-name-accessor example this
  bullet used to cite, `stops()`/`stops_count_val()` in `planner.h`, no
  longer applies: `planner.proto`/`planner.h` were deleted by 115-002/
  115-003's motion-stack excision). Any future generator change
  reintroducing `get_`-prefixed accessors must not ship; fixes go in
  `scripts/gen_messages.py`, never in a generated header.

## 4. Design

**Why a codec built on primitives, not a direct protobuf library.** The
firmware target is CODAL/`-fno-exceptions -fno-rtti`, newlib-nano, no heap —
incompatible with a general-purpose protobuf runtime. `wire_runtime` supplies
exactly the wire-format primitives this schema needs (varint, zigzag,
fixed32, length-delimited framing, packed-repeated, unknown-field skip,
base64 — retained but no longer the live armor, §3) plus, as of sprint 123,
the wire's current framing primitives (COBS encode/decode, CRC-16 compute/
verify) with caller-owned buffers and no allocation; the generated `wire.cpp`
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

**`TelemetrySecondary` — DELETED (sprint 124 ticket 009); this paragraph
is now historical.** Through sprint 123, `TelemetrySecondary` rode its
own, independently-COBS+CRC-framed line: encode-only, never a
`ReplyEnvelope` oneof arm, the slower diagnostic frame, firmware-emitted
only, never host-decoded on the robot side. Its former `cycle_busy`/
`cycle_period` fields (11/12, sprint 122's interim placement) became
`reserved` at sprint 123 ticket 004, migrated onto the primary
`Telemetry` frame instead (fields 15/16) once COBS+CRC framing restored
primary-frame headroom — see `app/DESIGN.md` §4. Sprint 124 ticket 009
deleted the message, its wire arm, and the tie-break/alternation cadence
machinery that used to choose between it and the primary frame outright
— there is only ever one outbound telemetry frame now. See
[`docs/protocol-v5.md`](../../../docs/protocol-v5.md) §8 for the current
field reference.

**Unknown fields are forward-compatible by design.** `skipField()`
advances past an unrecognized field number's value without interpreting it,
letting an older decoder round-trip a newer schema's added field (or a
declared-but-unused oneof arm) without erroring. It never recurses into a
length-delimited payload's structure — an opaque byte-range skip — so it
cannot trip the nesting-depth guard regardless of how deep the caller
already is.

**`main.cpp` is the one place `msg::*` types meet `Devices::*` types.** Per
the devices-isolation invariant (`docs/design/design.md` §5), `messages/` types never reach
`devices/`; `main.cpp` converts wire-plane `msg::MotorConfig` to
`Devices::MotorConfig` at construction time. This directory has no part in
that conversion — it only defines the wire-side shape.

## 5. Interfaces

### Exposes

- **`msg::wire::decode(CommandEnvelope& out, const uint8_t* buf, uint16_t
  len) -> Result`:** decodes and validates one `CommandEnvelope` from a raw
  (already COBS-decoded, CRC-verified — sprint 123) byte buffer.
  `Result{ok, field, code}`: `ok` is
  false on the first violation encountered (missing `(req)` field,
  out-of-bound value, or malformed wire bytes), `field` names which field
  number, `code` is an `ErrCode` (see `envelope.proto`'s doc comment for
  which code means which). Never partially decodes into `out` on failure in
  a way the caller should trust.
- **`msg::wire::encode(const ReplyEnvelope& in, uint8_t* buf, uint16_t cap)
  -> uint16_t`:** encode into `buf`, return the number of bytes written, or
  `0` if `cap` is smaller than the required output (never a
  truncated/corrupt partial buffer). (Through sprint 123 there was also a
  `TelemetrySecondary` overload — deleted outright by 124-009, see §3/§4.)
- **`msg::wire::decode(Telemetry& out, const uint8_t* buf, uint16_t len)
  -> Result`** (124-008, test-only — see §3's own bullet): the same
  decode contract as `decode(CommandEnvelope&, ...)` above, applied to a
  message production code only ever encodes. Exists so a test can exercise
  `validateBounds()` against `Telemetry`'s own bounded fields without a
  hand-rolled parallel decoder.
- **`WireRuntime::*` primitives** (`wire_runtime.h`): the encode/decode
  never-partial contract described in §3, for any future hand-written
  caller needing raw protobuf-wire-format bytes without going through the
  generated schema layer.
- **Generated `msg::*` structs:** the wire-schema shape itself — every
  field, enum, and nested oneof union other subsystems construct, read, and
  pass to `Core::Comms`/`Core::Telemetry`. Authoritative source for these
  shapes is the corresponding `protos/*.proto` file, not this document.

### Consumes

- **`protos/*.proto` and `protos/options.proto`** (via `scripts/
  gen_messages.py`, host-only, `grpcio-tools`) — the schema source of
  truth; see [`../../protos/DESIGN.md`](../../protos/DESIGN.md) and
  [`../../scripts/DESIGN.md`](../../scripts/DESIGN.md) "Build-time
  generators."
- **`app/` (via `Core::Comms`/`Core::Telemetry`):** the only consumer of the
  decode/encode entry points at runtime — see
  [app/DESIGN.md](../app/DESIGN.md) for how a decoded `CommandEnvelope`
  reaches the loop's dispatch and how a `ReplyEnvelope` gets
  CRC-then-COBS framed and sent (the only outbound top-level message —
  `TelemetrySecondary` is deleted, 124-009).
- **`config/`:** consumes generated `msg::*Config` shapes declared here for
  baked boot configuration, and (132-013) the generated `robot_config.h`
  group structs (`msg::Drive`/`WheelControl`/`Motors`/`Otos`/etc.) both as
  `Config::Robot`'s own field storage and as the persisted-tuning source
  of truth (`config/persisted_tuning.h`) — see
  [config/DESIGN.md](../config/DESIGN.md).

## 6. Open Questions / Known Limitations

- **`event.h` (`msg::Event`) is hand-written but not generated, and is not
  referenced anywhere in the live `src/firm` tree** — the only other
  references found (at the time of the original review) were in the
  now-deleted `protos/planner.proto`/`planner.h` (a distinct, generated
  type, not `msg::Event` — removed entirely by 115-002/115-003's
  motion-stack excision) and in archived/parked source
  (`src/archive/source_parked/094/subsystems/planner.cpp`,
  `src/tests/sim/parked-094/...`). Its header comment describes a role
  ("lets both a subsystem producer and `CommandProcessor` depend on the same
  type") that predates the single-loop rebuild — `CommandProcessor` and
  `Subsystems::Planner` no longer exist in this tree (deleted sprints
  102-107, `docs/design/design.md` §5). This looks like dead code
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
