---
status: pending
---

# Protocol v3: Schema-Driven Binary Command Plane (protobuf)

## Context

The command layer is ~4,900 lines in `source/commands/` (motion_commands.cpp alone is 1,162),
dominated by hand-rolled per-verb parsing (47 near-identical range-check reject sites),
five parallel `strcmp` config-key chains, per-handler `snprintf` reply assembly, and three
duplicated copies of integer-math float formatting. The host maintains a *second* hand-written
parser of the same text grammar (`host/robot_radio/robot/protocol.py`, 1,097 lines, plus a
pydantic config mirror kept in sync by a checker script). The stakeholder no longer needs the
protocol to be hand-typeable — a good host program can speak a machine format — and wants a
schema system that validates messages and drastically reduces parsing code on both sides.

**Key asset already in the tree:** a proto3 schema pipeline — `protos/*.proto` →
`scripts/gen_messages.py` (grpcio-tools, runs on every build via `justfile`/`build.py`) →
`source/messages/*.h` POD structs (`Opt<T>`, fixed arrays via `(max_count)`, oneof→Kind+union).
It generates the exact structs every subsystem consumes but no wire codec. This plan extends
that pipeline to put the same schema **on the wire**.

**Stakeholder decisions (2026-07-09):**
- Direction: straight to the binary program (~3 sprints), no interim text-parser rewrite.
- Format: **protobuf** (over MessagePack/JSON) — leverages the existing pipeline; host gets
  `google.protobuf` runtime for free; firmware codec is differentially testable against it.
- End-state text plane: minimal rump — **PING, ID, HELLO, HELP, STOP** stay hand-typeable;
  everything else is binary-only, with `rogo` as the human REPL translating typed v2 syntax.
- Migration style: dual stack — both planes live during migration; a text family is deleted
  only after its binary replacement is bench-proven. Robot drivable at every commit.

## Constraints (verified)

- Transports are line-oriented and NOT binary-safe at the app layer: serial `readLine()` splits
  on `\n`, NUL-terminates (`source/com/serial_port.cpp:21`, `_rxBuf[256]`); radio reassembly
  NUL-terminates + `strlen()` and appends `\n` for the relay byte-pipe (`source/com/radio.cpp:64-95`);
  the host relay reader skips `#`-prefixed lines; relay firmware cannot change.
  `Communicator` latches `char line_[256]` → effective ~250-char line budget.
- Firmware: CODAL C++11, no heap in hot path, no exceptions/RTTI, newlib-nano (no `%f`).
- Flash: image ends 0x684B8 → ~427 KB of 512 KB used, ~90 KB usable headroom. RAM ~98% is
  by design (never flag it); measure `.bss` deltas anyway.

## Design

### Framing: ASCII-armored binary inside the existing line world

Wire form: `*B<base64(envelope_bytes)>\n`. Base64 contains none of `\0 \r \n #`, so
**zero changes** to serial_port.cpp, radio.cpp, communicator, the relay, or the host reader's
line splitting. `*` (0x2A) cannot collide with text verbs (uppercase), replies (`OK/ERR/...`),
or relay `#` lines. Budget: ~250-char line → **186-byte max envelope payload** (enforced by
generated `static_assert` on computed max encoded sizes; config snapshots chunk one subsystem
slice per reply). Corr-id moves inside the envelope. Raw COBS framing: deferred indefinitely
(33% armor overhead is irrelevant at 115200 baud; binary TLM is still smaller than text TLM).

### Schema: `protos/envelope.proto` (new)

```proto
message CommandEnvelope {
  uint32 corr_id = 1;
  oneof cmd {                      // one arm per Blackboard input queue + system verbs
    DrivetrainCommand drive   = 2; // -> bb.driveIn
    MotionSegment     segment = 3; // -> bb.segmentIn   (MOVE)
    MotionSegment     replace = 4; // -> bb.replaceIn   (MOVER)
    PlannerCommand    motion  = 5; // -> bb.motionIn
    ConfigDelta       config  = 6; // -> bb.configIn
    SetPose           pose    = 7; // -> bb.poseResetIn
    OdometerCommand   otos    = 8; // -> bb.otosCommandIn
    Ping ping = 9;  Echo echo = 10;  ConfigGet get = 11;  StreamControl stream = 12;
    Stop stop = 13;
  }
}
message Ack   { uint32 q = 1; float rem = 2; }         // MOVE/MOVER flow control rides the ack
message Error { ErrCode code = 1; uint32 field = 2; }  // field number that failed; host maps to name
message ReplyEnvelope {
  uint32 corr_id = 1;
  oneof body { Ack ok = 2; Error err = 3; Telemetry tlm = 4;
               ConfigSnapshot cfg = 5; Event evt = 6; DeviceId id = 7; }
}
```

Rule: **one oneof arm per queue** — the firmware dispatcher becomes a switch, not a verb table.

### Validation: generated from custom options (replaces 47 hand-written range checks)

Extend `protos/options.proto`:

```proto
optional double min     = 50002;  // inclusive lower bound
optional double max     = 50003;  // inclusive upper bound
optional double abs_max = 50004;  // |v| <= abs_max (speed/twist idiom)
optional bool   req     = 50005;  // must be present on the wire
```

Usage: `float v = 1 [(abs_max) = 2000, (units) = "mm/s"];`. Validation executes during decode
(single pass over the same tables); a failed bound aborts with `{fieldNumber, ErrCode}` → binary
`Error` reply. Also emit standalone `validate()` for internally built messages.

### Generated codec (firmware)

- **Hand-written once:** `source/messages/wire_runtime.{h,cpp}` (~500 lines) — varint/zigzag/
  fixed32, length-delimited recursion (depth-bounded), packed repeated with `max_count` clamp,
  unknown-field skip, base64. No heap, static buffers.
- **Generated by gen_messages.py:** `source/messages/wire.{h,cpp}` — per-message
  `FieldDesc{number, wireType, kind, offset, aux, min, max}` tables in `.rodata` (~4 KB) plus:

```cpp
namespace msg { namespace wire {
struct Result { bool ok; uint16_t field; ErrCode code; };
Result   decode(CommandEnvelope& out, const uint8_t* buf, uint16_t len);  // validates inline
uint16_t encode(const ReplyEnvelope& in, uint8_t* buf, uint16_t cap);     // 0 on overflow
}}
```

- **Day-one decision gate:** `static_assert(std::is_standard_layout<...>)` on every generated
  struct the tables `offsetof` into. If any fail → fallback is generated per-message unrolled
  decode functions (+10–15 KB flash, same API). Secondary fallback if codec bugs pile up:
  swap the interpreter for nanopb's `pb_decode.c/pb_encode.c` behind the same API.

Consider using https://github.com/nanopb/nanopb_cpp 

### Dispatcher integration (firmware)

`CommandProcessor::process()` (source/commands/command_processor.cpp:421): if `line[0]=='*'` →
`BinaryChannel::handle(line, returnPath)`; else existing text table. New
`source/commands/binary_channel.{h,cpp}` (~250 lines): dearmor → `wire::decode` →
`switch (env.cmdKind)` → post to the matching Blackboard queue (same posts the text handlers
make today) → encode/armor/send `ReplyEnvelope`. Ping/echo/id/get/stream handled inline.
Subsystems never know which plane a command came from. Binary TLM: `StreamControl.binary`
selects the emitter; deletes the snprintf telemetry emitters at cutover.

Config plane: mark config fields `proto3 optional` (generator already maps to `Opt<T>`);
generated merge (apply-present-fields) replaces the five strcmp chains; generated slice encode
replaces the CFG snprintf emitters. Note: the currently *parked* text families (config/pose/
otos/dev — unregistered since 093/094) get their functionality back through binary arms; the
text versions stay parked and are then deleted.

### Host

1. `grpc_tools.protoc --python_out=host/robot_radio/robot/pb2` added beside gen_messages.py in
   justfile/build.py — device tables and host pb2 can never skew (unknown-field skip covers
   transient skew).
2. `io/serial_conn.py`: reader thread gains one branch — `*` lines → dearmor →
   `ReplyEnvelope.ParseFromString` → existing corr-id demux/`_reply_queues`/TLM queue machinery.
   `send_envelope()` alongside text `send()`. Pipelining semantics unchanged.
3. `robot/protocol.py`: **NezhaProtocol keeps its public API** (the compatibility shim) —
   method bodies become envelope builders; `TLMFrame` dataclass constructed from pb2 Telemetry —
   so TestGUI, gamepad teleop (reads q/rem from Ack), bench scripts, and the MCP server change
   zero call sites. `parse_response/parse_tlm/parse_cfg` deleted at text retirement (~500+ lines).
4. `robot_config.py`: retool `scripts/check_config_sync.py` to diff pydantic against pb2
   descriptors (later: generate the model).
5. `rogo send` becomes the REPL: text-v2→envelope translator + `--decode` pretty-printer.

## Sprint breakdown (3 sprints, HITL bench gate each per .claude/rules/hardware-bench-testing.md)

**Sprint 1 — Codec foundation + binary command plane (dual stack).**
options.proto validation extensions; gen_messages.py emits envelope structs, field tables,
wire.{h,cpp}; wire_runtime; standard-layout static_asserts (decision gate); host pb2 codegen;
BinaryChannel + `*` discriminator with drive/segment/replace/stop/ping/echo/id arms; binary
Ack/Error; `serial_conn.py` branch + `rogo` binary send.
*Tests:* differential round-trip pytest vs google.protobuf (both directions) + fuzz + range
cases; full existing sim suite stays green (text untouched).
*Bench:* deploy; drive on stand via binary MOVE/MOVER/STOP over **USB serial AND radio relay**
(proves armor survives the relay's `#`-skip and line pipe); text regression pass; flash delta
from MICROBIT.map recorded.

**Sprint 2 — Binary telemetry + config plane.**
Telemetry reply arm + `StreamControl.binary`; host TLMFrame-from-pb2; `optional`-ize config
fields; generated merge + ConfigSnapshot encode; generated validation replaces the hand range
checks on the binary path; host set/get config binary; check_config_sync retooled.
*Bench:* stream text vs binary TLM at matched rates, compare `tlm_drop_rate()`; gamepad teleop
session on binary TLM + Ack q/rem flow control; round-trip every config slice; change a PID
gain over binary and observe wheel behavior change on the stand.

**Sprint 3 — Host completion + text retirement.**
All NezhaProtocol methods binary; TestGUI/teleop/bench/MCP verified on the unchanged API;
rogo REPL translator complete; delete migrated text parseFns, SET/GET chains, snprintf TLM/CFG
emitters, stop-clause grammar, and host parse_tlm/parse_cfg; **retain text rump PING, ID,
HELLO, HELP, STOP** (~120 lines; STOP as the bare-terminal safety affordance); update
docs/protocol-v2.md → protocol-v3 doc.
*Bench:* full binary regression over serial + relay; typed `STOP` from a bare terminal halts
a moving robot; TestGUI + teleop session over radio; final flash report (expect net reduction
vs. pre-project baseline).

## What gets deleted (end state)

- Firmware: hand parseFns (~650 lines), five config strcmp chains (~370), snprintf TLM/CFG/reply
  emitters (most of 125 sites), stop-clause text grammar, duplicated formatFixed/formatTenths
  copies, vestigial `ParsedCommand` (source/types/command_types.h:197 — zero references).
  `source/commands/` ~4,900 → roughly 1,000–1,300 (rump + BinaryChannel + dispatch core).
- Host: `protocol.py` hand parsers (~500+ lines), config-sync divergence risk.

## Flash/RAM

Peak dual-stack: +12–15 KB flash against ~90 KB headroom (interpreter 2–3 KB, tables ~4 KB,
glue/base64/envelope structs, BinaryChannel). Sprint 3 reclaims an estimated 15–30 KB —
plausible net negative. RAM: ~1–1.5 KB static scratch (shared decode/encode buffers, reuse the
existing stack-buffer pattern); measure `.bss` per sprint.

## Risks (ranked)

1. **Self-written codec correctness** — mitigated by differential/fuzz tests against
   google.protobuf; fallback: nanopb runtime behind the same API.
2. **standard-layout/offsetof on generated oneof-union structs** — Sprint 1 day-one gate;
   fallback: unrolled per-message codegen.
3. **186-byte payload cap (radio path)** — generated `kMaxEncodedSize` static_asserts fail the
   build if an envelope outgrows the line; CFG chunked by design; Telemetry stays curated.
4. **Config presence semantics churn** (`Opt<T>` reaching `configure()` paths) — quarantined in
   Sprint 2.
5. **Relay-side filtering beyond `#`** — exercised by Sprint 1's relay bench gate before
   anything depends on it.
6. **Parked text families have no live regression tests** (their sim tests sit in
   tests/sim/parked-093/094) — binary arms for config/pose/otos need fresh sim coverage in
   Sprints 1–2.

## Critical files

- `scripts/gen_messages.py` — extend: field tables, envelope, validation, wire.{h,cpp} emit
- `protos/options.proto`, new `protos/envelope.proto`
- `source/messages/wire_runtime.{h,cpp}` (new), `source/commands/binary_channel.{h,cpp}` (new)
- `source/commands/command_processor.cpp` — `*` discriminator
- `host/robot_radio/io/serial_conn.py`, `host/robot_radio/robot/protocol.py` (shim),
  `host/robot_radio/io/cli.py` (rogo REPL)

## Verification summary

Per sprint: differential codec pytest (host reference), sim suite green, ARM + sim builds,
`mbdeploy probe && mbdeploy deploy` then the sprint's stand exercises above (sensors alive,
wheels drive both directions with encoders tracking, round-trip over serial AND relay).

## Process note

This repo runs CLASI: on approval this plan becomes an issue + sprint plans via the
sprint-planner (team-lead cannot create sprints/tickets directly — mcp-guard). Do not start
implementing directly from this approval.
