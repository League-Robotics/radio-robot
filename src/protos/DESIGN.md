# Protos (`src/protos`)

**Owner:** Eric Busboom · **Last reviewed:** 2026-07-21 · **Status:** in-flux

---

## 1. Purpose

`protos/` is the schema source of truth for every message that crosses the
host/robot wire boundary: one `.proto` file per subsystem's messages, plus
a shared `common.proto` and a `options.proto` extension file that adds the
bounds/units metadata neither firmware nor host codegen can express in
plain proto3. It is a subsystem of its own — not folded into
`src/firm/messages/` — because it has exactly one dependency direction:
two independent generators (`scripts/gen_messages.py` for the firmware,
`scripts/gen_pb2.py` for the host) each compile from these same `.proto`
files, and neither generator's output may become the thing a human edits
to change a message shape. Editing a field means editing here, then
regenerating both sides — never hand-patching a generated header or a
generated `_pb2.py`.

## 2. Orientation

Ten subsystem proto files (`common`, `communicator`, `config`,
`drivetrain`, `envelope`, `gripper`, `motor`, `odometer`, `ports`,
`sensors`, `telemetry`) plus `options.proto` (the custom field-option
extensions). `envelope.proto` is the root of the schema graph: it declares
`CommandEnvelope` (the one inbound frame shape, `oneof cmd { config, stop,
twist }`) and `ReplyEnvelope` (the one outbound frame shape, `oneof body {
ok, err, tlm }`), importing `config.proto` and `telemetry.proto`; every
other file is either imported transitively from there or stands alone as
a message a subsystem-specific consumer decodes directly (e.g.
`gripper.proto`, not yet wired into `envelope.proto`'s oneofs — see §6).

`options.proto` defines the extension fields both generators read off
`FieldOptions`: `(units)` (informational-only string), `(max_count)`
(repeated-field fixed capacity — the generators have no heap, so every
`repeated` field becomes a fixed-size C array/list), `(min)`/`(max)`/
`(abs_max)` (validated bounds, both generators enforce these at
decode/encode time as of the binary wire cutover), and `(req)` (field
must be present on the wire or decode fails). A field with no bound
option is unconstrained; a field the generator cannot fit a required
bound onto fails codegen loudly rather than silently shipping an
unbounded wire surface.

`motion.proto` and `planner.proto` — the pre-115 motion-stack schema
(the `Move` command, jerk-trajectory profile parameters, `PlannerConfig`)
— are **deleted**, not merely unused: 115-002/115-003
(gut-to-minimal-firmware S1) removed them along with their only
consumers (`Motion::Executor`, `App::Pilot`). `envelope.proto`'s
`CommandEnvelope.cmd` oneof `reserved`s field number `20` (the old `Move`
arm) rather than reusing it — sprint 116's planned MOVE protocol
reintroduces a `Move`-shaped arm at a fresh number, never 20.

## 3. Constraints and Invariants

- **This is the only place a wire message shape is authored.** Both
  `src/firm/messages/*.h` (C++ POD structs + codec) and
  `src/host/robot_radio/robot/pb2/*_pb2.py` (compiled Python bindings) are
  generated from these files and are never hand-edited (see each
  generator's own header comment). A schema change starts and ends here;
  regenerate both sides (`gen_messages.py`, `gen_pb2.py` — `build.py`'s
  codegen step runs both before every firmware build) before either
  generated tree can be trusted again.
- **Deleted field numbers are `reserved`, never reused, once a schema has
  shipped to real hardware.** `envelope.proto`'s `CommandEnvelope`/
  `ReplyEnvelope` carry `reserved` lists of every removed pre-102/pre-115
  oneof arm's field number specifically because those envelopes have
  shipped on real robots — reusing a retired number for an unrelated new
  field would let an old firmware image silently misinterpret a new
  command. `telemetry.proto`'s `Telemetry` message is the deliberate
  exception: it is a clean 115-003 renumber with NO `reserved` list,
  because (per that file's own header comment) `Telemetry` "has still
  never shipped to a real robot" at the time of that rewrite — there was
  no deployed client's field-number expectation to protect. Do not copy
  that exception onto a message that HAS shipped.
- **The 186-byte envelope budget is enforced at C++ build time, not
  here.** `options.proto`'s bounds feed `gen_messages.py`'s worst-case
  size estimator, but the actual `static_assert` against the budget lives
  in generated `wire.h` (see
  [`../firm/messages/DESIGN.md`](../firm/messages/DESIGN.md) §3) — a
  schema change that blows the budget fails the firmware build, not
  `protoc`/`grpcio-tools` compilation of this directory.
- **`(min)`/`(max)`/`(abs_max)`/`(req)` are the validated contract, not
  documentation.** Both generated decoders reject a wire value outside
  its declared bound or a missing `(req)` field — these are not merely
  informative comments the way `(units)` is.

## 4. Design

**Why a curated `ConfigDelta`, not the full generated config messages.**
`config.proto`'s header comment explains the shape decision directly:
the wire config plane exposes only the ~15 keys
`src/firm/app/robot_loop.cpp`'s `handleConfig` actually understands
(`MotorConfigPatch`/`OtosConfigPatch`/`DrivetrainConfigPatch`/
`WatchdogConfigPatch`), each field `optional` so presence signals "set
this," rather than the full generated `MotorConfig`/`DrivetrainConfig`
messages — those don't fit the envelope budget individually and marking
every one of their fields `optional` would ripple an `Opt<T>` wrapper
into every existing construction call site for no benefit (only a
fraction of those fields have a live wire-config verb at all). This is
the same "curate a Patch subset, don't wire-expose the internal struct"
pattern `envelope.proto`'s narrowing to `twist`/`config`/`stop` follows
at the top level.

**Two independent generators, one schema, by design.** `gen_messages.py`
and `gen_pb2.py` do not share a code path — one hand-emits C++11 POD
structs plus a table-driven codec for a no-heap/no-RTTI embedded target,
the other is a thin wrapper around `protoc --python_out`. Both read the
identical `.proto` set on every build so the two sides can never skew
independently; see [`../scripts/DESIGN.md`](../scripts/DESIGN.md) for
each generator's own shape.

## 5. Interfaces

### Exposes

- **The `.proto` schema itself** — every `message`/`enum`/`oneof` other
  subsystems' generated code is built from. Authoritative for wire shape;
  `src/firm/messages/*.h` and `src/host/robot_radio/robot/pb2/*_pb2.py`
  are derived artifacts, never the other way around.
- **`options.proto`'s extension fields** (`units`/`max_count`/`min`/
  `max`/`abs_max`/`req`) — the vocabulary every other `.proto` file in
  this directory uses to declare a field's wire-level contract.

### Consumes

- Nothing — this is the leaf of the schema dependency graph (root
  [`docs/design/design.md`](../../docs/design/design.md) subsystem map).
  It is read by, but does not depend on, `scripts/gen_messages.py`,
  `scripts/gen_pb2.py`, and (indirectly, via those generators' output)
  `src/firm/messages/` and `src/host/robot_radio/robot/pb2/`.

## 6. Open Questions / Known Limitations

- **`gripper.proto`, `sensors.proto` (line/color read-only state),
  `ports.proto`, and `odometer.proto` are declared but not all wired into
  `envelope.proto`'s `CommandEnvelope`/`ReplyEnvelope` oneofs today** —
  the current minimal firmware (S1) speaks `twist`/`config`/`stop` in and
  `ok`/`err`/`tlm` out only; `Telemetry`'s packed `line`/`color` fields
  cover line/color *sensing* without a dedicated `sensors.proto` message
  reaching the wire. Confirm each file's actual live consumer before
  assuming it is reachable from a real command.
- **`motion.proto`/`planner.proto` are deleted, not archived here** — if
  sprint 116's MOVE protocol work needs to reference the pre-gut schema
  shape, it lives in git history at the `pre-gut-motion-stack` tag, not
  as a parked file in this directory.
