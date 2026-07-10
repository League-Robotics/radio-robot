---
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 095 Use Cases

This sprint lands the *codec foundation* of the protocol-v3 program
(`clasi/issues/protocol-v3-schema-driven-binary-command-plane-protobuf.md`):
a schema-driven binary command plane that coexists with the untouched text
plane. Most use cases here are infrastructure (schema, codegen, hand-written
wire primitives, differential testing) with no direct parent in
`docs/usecases.md` — they exist to *serve* the transport-independent
UC-001/UC-002/UC-003/UC-004/UC-018 (drive/stop/device-discovery) use cases
over a second wire format, not to introduce new robot behavior. SUC-006 and
SUC-007 are the user/host-visible use cases that actually ride the new
plane this sprint; SUC-001..005 are the machinery underneath them.

## SUC-001: Schema author declares the binary wire contract
Parent: (infrastructure — schema foundation for UC-001/UC-002/UC-003/UC-004/UC-018)

- **Actor**: Firmware/protocol developer editing `protos/*.proto`.
- **Preconditions**: `protos/options.proto` has only `units`/`max_count`;
  no envelope or validation-bound schema exists; `source/motion/segment.h`'s
  `Motion::Segment` is explicitly NOT a generated proto type (094-005
  decision, predates the binary plane's existence).
- **Main Flow**:
  1. Extend `protos/options.proto` with `min`/`max`/`abs_max`/`req` field
     options (field numbers 50002-50005).
  2. Add `protos/envelope.proto`: `CommandEnvelope`/`ReplyEnvelope` (one
     oneof arm per `Rt::Blackboard` command-plane queue plus system verbs),
     `Ack`, `Error`, `ErrCode`, and the system-verb leaf messages
     (`Ping`/`Echo`/`ConfigGet`/`StreamControl`/`Stop`/`DeviceId`).
  3. Add `protos/motion.proto`: `MotionSegment`, a new proto message
     mirroring `Motion::Segment`'s fields 1:1 in the SAME native units
     (mm, rad, mm/s, ...) — not a generated replacement for
     `Motion::Segment` itself, which stays the executor's own hand-owned
     internal type.
  4. `python scripts/gen_messages.py` regenerates every header, including
     two new ones (`envelope.h`, `motion.h`), without touching the shape of
     any existing generated header.
- **Postconditions**: The full binary wire contract for this sprint's seven
  implemented arms (drive/segment/replace/stop/ping/echo/id) plus five
  declared-only arms (motion/config/pose/otos/get/stream) exists as proto
  source, with every validated field's bound traceable to an existing text
  handler's own numeric constant (no bound re-derived from scratch).
- **Acceptance Criteria**:
  - [ ] `protos/options.proto` declares `min`/`max`/`abs_max`/`req` at field
        numbers 50002-50005, alongside the existing `units`/`max_count`.
  - [ ] `protos/envelope.proto` declares `CommandEnvelope`/`ReplyEnvelope`
        with every oneof arm named in this sprint's scope (implemented and
        declared-only), plus `Ack{q,rem}`, `Error{code,field}`, `ErrCode`.
  - [ ] `protos/motion.proto`'s `MotionSegment` fields match
        `Motion::Segment`'s fields 1:1 (name, unit, sign convention); every
        bound mirrors the matching `parseMove`/`parseMover` constant in
        `motion_commands.cpp` converted to native units, not re-derived.
  - [ ] `python scripts/gen_messages.py --dry-run` succeeds with zero diff
        to any pre-existing header's emitted content beyond additive
        changes implied by the new options.
  - [ ] `just build-sim` and the full existing sim suite stay green.

## SUC-002: Build decides, on day one, whether generated structs support field tables
Parent: (infrastructure — de-risks SUC-004 before the expensive codegen work)

- **Actor**: Firmware developer running the build.
- **Preconditions**: SUC-001's schema exists; no field-descriptor table or
  `wire.{h,cpp}` codegen exists yet.
- **Main Flow**:
  1. `gen_messages.py` emits one `static_assert(std::is_standard_layout<
     msg::Xxx>::value, "...")` per generated struct that a future
     `offsetof`-based field table would need to reach into.
  2. The ARM and sim builds compile these asserts as part of the normal
     build (no new build step).
  3. The result (every struct passes, or a specific named subset fails) is
     recorded in the ticket that runs this check.
- **Postconditions**: Whichever generator strategy SUC-004 implements
  (offsetof-based generic field tables, or the documented unrolled-codegen
  fallback for any failing struct) is a decision made from evidence, not
  assumed.
- **Acceptance Criteria**:
  - [ ] Every struct the future field tables would need `offsetof` into
        (every message reachable from `CommandEnvelope`/`ReplyEnvelope`) has
        a generated `static_assert(std::is_standard_layout<...>)`.
  - [ ] `just build` (ARM) and `just build-sim` both compile the asserts
        successfully, or fail with a specific, named struct list.
  - [ ] The pass/fail outcome — and, if any struct fails, which one and the
        fallback decision — is written into this ticket's completion notes.

## SUC-003: Firmware can encode and decode raw protobuf wire bytes with no heap
Parent: (infrastructure — codec primitives underlying SUC-004/SUC-006)

- **Actor**: Firmware developer; indirectly, `BinaryChannel` (SUC-006) at
  runtime.
- **Preconditions**: CODAL C++11, `-fno-exceptions -fno-rtti`, no heap in
  the hot path, newlib-nano (no `%f`).
- **Main Flow**:
  1. `source/messages/wire_runtime.{h,cpp}` implements varint/zigzag
     encode+decode, fixed32 encode+decode, length-delimited framing with a
     depth-bounded recursion guard, a packed-repeated reader that clamps at
     a caller-supplied `max_count`, an unknown-field skip, and base64
     encode+decode.
  2. Every function operates on caller-owned static buffers; nothing
     allocates.
- **Postconditions**: A schema-agnostic byte-level toolkit exists that
  SUC-004's generated `wire.{h,cpp}` builds on; it knows nothing about
  `CommandEnvelope` or any specific message type.
- **Acceptance Criteria**:
  - [ ] Round-trip unit tests (varint, zigzag, fixed32, base64) pass for
        boundary values (0, negative, `INT32_MIN/MAX`, empty buffer).
  - [ ] A truncated or malformed buffer is rejected (returns a clean
        failure indicator), never reads past the buffer end, never crashes.
  - [ ] Length-delimited recursion has an enforced depth bound; a
        maliciously nested input is rejected rather than overflowing the
        stack.
  - [ ] No heap allocation, no exceptions, no RTTI (compiles under
        `-fno-exceptions -fno-rtti`); no `%f`/float `snprintf` anywhere.

## SUC-004: Generated per-message field tables drive a generic decode/encode/validate engine
Parent: (infrastructure — the schema-driven engine SUC-006 dispatches through)

- **Actor**: `gen_messages.py` (build-time); `BinaryChannel` (SUC-006, at
  runtime).
- **Preconditions**: SUC-001's schema, SUC-002's layout decision, and
  SUC-003's primitives all exist.
- **Main Flow**:
  1. `gen_messages.py` emits, per message type, a `FieldDesc{number,
     wireType, kind, offset, aux, min, max}` table in `.rodata`.
  2. `gen_messages.py` emits `source/messages/wire.{h,cpp}`:
     `msg::wire::decode(CommandEnvelope&, buf, len)` (validates every
     bounded field inline against its table entry) and
     `msg::wire::encode(const ReplyEnvelope&, buf, cap)` (returns 0 on
     overflow rather than truncating).
  3. `gen_messages.py` emits `static_assert(kMaxEncodedSize<=186)` per
     top-level envelope type.
- **Postconditions**: Decoding/encoding any of this sprint's implemented
  envelope arms requires zero hand-written per-message code — the same
  generic engine walks every message's table.
- **Acceptance Criteria**:
  - [ ] `decode()` rejects a message with a missing `req` field, an
        out-of-`min`/`max`/`abs_max` field, or an unknown field number
        (skipped, not rejected, per SUC-003's unknown-field skip) with a
        `{fieldNumber, ErrCode}` result.
  - [ ] `encode()` returns 0 (not a truncated buffer) when the caller's
        buffer is smaller than the required output.
  - [ ] `kMaxEncodedSize` for `CommandEnvelope` and `ReplyEnvelope` is
        `<= 186` bytes, enforced at compile time.
  - [ ] Repeated fields decode up to their `max_count` and silently clamp
        (drop) any excess, never overflow the fixed array.

## SUC-005: The firmware codec is proven correct against the host's reference protobuf implementation
Parent: (infrastructure — the correctness backbone for the whole program, issue Risk 1)

- **Actor**: Developer running the test suite; CI.
- **Preconditions**: SUC-003 and SUC-004 exist; `host/robot_radio/robot/
  pb2/` (SUC-007) provides the `google.protobuf`-backed reference codec for
  the same schema.
- **Main Flow**:
  1. A host-compiled C++ harness (`tests/sim/unit/*_harness.cpp`, the same
     pattern as `runtime_blackboard_harness.cpp`) links `wire_runtime.cpp` +
     `wire.cpp` and exposes encode/decode entry points to a Python driver.
  2. A pytest suite feeds matched inputs through both the firmware codec
     and `google.protobuf`'s generated Python bindings, in both directions
     (host-encode -> firmware-decode; firmware-encode -> host-decode), and
     asserts the decoded values agree field-for-field.
  3. A fuzz corpus (random, truncated, oversized, and unknown-field-salted
     byte strings) is fed to the firmware decoder; it must never crash and
     must reject cleanly.
  4. A boundary/range corpus exercises every `min`/`max`/`abs_max`/`req`
     validated field at, just inside, and just outside its bound.
- **Postconditions**: The self-written codec (this program's #1 ranked
  risk) has machine-checked evidence of byte-for-byte agreement with the
  reference implementation, not just "it compiled."
- **Acceptance Criteria**:
  - [ ] Differential round-trip passes in both directions for every
        implemented oneof arm (drive/segment/replace/stop/ping/echo/id).
  - [ ] The fuzz corpus (>= 200 generated cases) produces zero crashes and
        zero out-of-bounds reads (run under ASan/UBSan on the host build).
  - [ ] Every validated field's boundary case (min-1, min, max, max+1,
        abs_max, -abs_max) produces the expected accept/reject verdict.
  - [ ] The full pre-existing sim suite (58 tests, per the sprint's
        baseline) stays green alongside the new differential suite.

## SUC-006: Operator or host client drives the robot's highest-traffic verbs over the binary plane
Parent: UC-001 (Drive Robot at Continuous Speed) / UC-004 (Stop Robot
Immediately) / UC-018 (Device Discovery) — served over a second wire format

- **Actor**: `robot_radio` host software, `rogo`, or any binary-speaking
  client, over USB serial or the radio relay.
- **Preconditions**: SUC-004's codec and SUC-005's differential proof both
  exist; the text plane (`S`/`STOP`/`D`/`T`/`RT`/`MOVE`/`MOVER`/`PING`/
  `ECHO`/`ID`/...) is unmodified and still fully functional.
- **Main Flow**:
  1. Client sends `*B<base64(CommandEnvelope bytes)>\n` on the same serial/
     radio line the text plane already uses.
  2. `CommandProcessor::process()` sees `line[0] == '*'` before
     tokenizing and dispatches to `BinaryChannel::handle()` instead of the
     text table; the text table's own code path is untouched.
  3. `BinaryChannel` dearmors, decodes (`msg::wire::decode`), and — for
     `drive`/`segment`/`replace`/`stop` — posts to the SAME
     `Rt::Blackboard` queue (`driveIn`/`segmentIn`/`replaceIn`/`driveIn`
     respectively) the matching text handler already posts to; for
     `ping`/`echo`/`id` it replies inline, mirroring
     `system_commands.cpp`'s handlers.
  4. `BinaryChannel` encodes and armors a `ReplyEnvelope` (`Ack{q,rem}` on
     success, `Error{code,field}` on failure) and sends it back on the
     command's own reply channel.
  5. `motion`/`config`/`pose`/`otos`/`get`/`stream` arms are declared in the
     schema (SUC-001) but reply `Error{UNIMPLEMENTED}` this sprint — their
     Blackboard consumers are parked (`motion`) or scoped to a later sprint
     (096: config/telemetry; 098: pose/otos).
- **Postconditions**: A binary client can drive/stop the robot and query
  its identity exactly as a text client can, on a framing that a relay
  byte-pipe and a NUL-terminating line reader both pass through unmodified
  — subsystems below `Rt::Blackboard` never learn which plane a command
  arrived on.
- **Acceptance Criteria**:
  - [ ] A binary `drive`/`segment`/`replace`/`stop` command posts the exact
        same Blackboard payload shape the equivalent text verb posts (S/
        MOVE/MOVER/STOP).
  - [ ] A binary `ping`/`echo`/`id` reply matches its text counterpart's
        information content (not wire bytes — the reply envelope, not
        `OK pong t=...` text).
  - [ ] A malformed or out-of-range binary command yields a typed
        `Error{code,field}` reply, never a crash, never a silent drop.
  - [ ] The existing text-plane sim suite passes byte-for-byte unmodified
        (proves `CommandProcessor`'s text branch is untouched code, not
        just untouched behavior).
  - [ ] Confirmed by the team-lead on the stand (per
        `.claude/rules/hardware-bench-testing.md`, run after this sprint's
        tickets close, not during any single ticket's own dev-time tests):
        binary `MOVE`/`MOVER`/`STOP` drive the robot over BOTH USB serial
        and the radio relay, the text-protocol regression pass still
        works, and the flash footprint delta is recorded from
        `MICROBIT.map`.

## SUC-007: Host tooling can speak the binary plane over the same transport
Parent: UC-018 (Device Discovery) — host-side half of SUC-006's wire

- **Actor**: A developer running `rogo`, or any future host client built on
  `robot_radio`.
- **Preconditions**: SUC-001's schema exists; `SerialConnection`'s reader
  thread currently drops any `*`-prefixed line silently (falls through
  every existing classify branch — verified against
  `host/robot_radio/io/serial_conn.py`'s `_reader_loop`).
- **Main Flow**:
  1. `grpc_tools.protoc --python_out=host/robot_radio/robot/pb2` is wired
     into `justfile`/`build.py` beside the existing `gen_messages.py` step,
     so the host's Python bindings and the firmware's C++ structs are
     generated from the identical `protos/*.proto` source and can never
     skew independently.
  2. `SerialConnection._reader_loop` gains one branch: a line starting with
     `*` is dearmored, parsed via `ReplyEnvelope.FromString(...)`, and
     routed through the SAME corr-id-keyed `_reply_queues`/`_tlm_queue`
     machinery the text `OK`/`ERR`/`TLM` branches already use.
  3. `SerialConnection` gains `send_envelope()` alongside the existing text
     `send()`; `rogo` gains a binary send path that can drive
     drive/segment/replace/stop/ping/echo/id.
- **Postconditions**: The host can send and receive binary envelopes over
  the identical transport and demux machinery the text plane already uses,
  with zero change to `_reader_loop`'s existing TLM/EVT/OK/ERR/CFG/ID
  branches or drop rules.
- **Acceptance Criteria**:
  - [ ] `host/robot_radio/robot/pb2/` is generated from `protos/*.proto`
        and importable.
  - [ ] `SerialConnection`'s reader thread correctly classifies a `*`-line
        reply and delivers it to the corr-id-keyed caller exactly as an
        `OK #<id>` text reply would be delivered today.
  - [ ] `send_envelope()`/`rogo`'s binary path successfully round-trips a
        `Ping` against the differential test harness's host-side codec
        (SUC-005) without needing live hardware.
  - [ ] No existing `_reader_loop` branch (`TLM`/`EVT`/`OK`/`ERR`/`CFG`/
        `ID`/`#`-comment/keepalive) changes behavior.
