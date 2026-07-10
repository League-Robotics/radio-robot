---
id: 095
title: 'Protocol v3 Sprint 1: Codec foundation and binary command plane (dual stack)'
status: done
branch: sprint/095-protocol-v3-sprint-1-codec-foundation-and-binary-command-plane-dual-stack
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
- SUC-006
- SUC-007
issues:
- protocol-v3-schema-driven-binary-command-plane-protobuf.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 095: Protocol v3 Sprint 1: Codec foundation and binary command plane (dual stack)

## Goals

Land the schema-driven binary command plane end to end for the highest-traffic
verbs (drive/segment/replace/stop/ping/echo/id) while the existing text plane
stays fully intact — the robot must remain drivable, on the bench, at every
commit. This is Sprint 1 of the 3-sprint protocol-v3 program described in
`clasi/issues/protocol-v3-schema-driven-binary-command-plane-protobuf.md`.
Establishes the codec (schema, generator, wire runtime, dispatcher) that
Sprints 2 (096) and 3 (097) build on.

## Problem

`source/commands/` is ~4,900 lines dominated by hand-rolled per-verb parsing,
five parallel strcmp config-key chains, and per-handler snprintf reply
assembly; the host maintains a second hand-written parser of the same text
grammar. A proto3 schema pipeline (`protos/*.proto` → `scripts/gen_messages.py`
→ `source/messages/*.h`) already generates the structs every subsystem
consumes, but nothing puts that schema on the wire. The stakeholder decided
(2026-07-09) to go straight to a binary protobuf-based command plane over
~3 sprints, no interim text-parser rewrite.

## Solution

Extend `protos/options.proto` with validation options (`min`/`max`/`abs_max`/
`req`); add `protos/envelope.proto` (`CommandEnvelope`/`ReplyEnvelope`, one
oneof arm per Blackboard input queue plus system verbs); extend
`scripts/gen_messages.py` to emit envelope structs, per-message field-
descriptor tables, and `source/messages/wire.{h,cpp}`; hand-write
`source/messages/wire_runtime.{h,cpp}` (varint/zigzag/fixed32/length-delimited/
base64, no heap, static buffers). Framing is ASCII-armored binary inside the
existing line world: `*B<base64(envelope_bytes)>\n` — zero changes to
`serial_port.cpp`, `radio.cpp`, the communicator, the relay, or the host
reader's line splitting. Day-one decision gate:
`static_assert(std::is_standard_layout<...>)` on every generated struct the
field tables `offsetof` into; fallback is generated per-message unrolled
codec if the gate fails. New `source/commands/binary_channel.{h,cpp}` plus a
`*`-line discriminator in `source/commands/command_processor.cpp::process()`
dispatch decoded envelopes to the same Blackboard queues the text handlers
already post to (drive/segment/replace/stop/ping/echo/id arms this sprint) and
encode binary `Ack`/`Error` replies. Host gets `grpc_tools.protoc --python_out`
codegen alongside `gen_messages.py`, and `io/serial_conn.py` gains a `*`-line
branch plus `rogo` binary send. Dual stack throughout — the text plane is
untouched code, not just untouched behavior.

## Success Criteria

- Differential round-trip tests (encode/decode both directions) pass against
  `google.protobuf` as the reference implementation, plus fuzz and boundary/
  range cases for every validated field.
- The full existing sim test suite stays green (text plane provably
  unmodified).
- On the bench (robot on the stand): deploy the dual-stack firmware, drive the
  robot via binary MOVE/MOVER/STOP over both USB serial and radio relay, run
  the existing text-protocol regression pass unmodified, and record the flash
  footprint delta from `MICROBIT.map`.
- The standard-layout decision gate result (pass, or fallback engaged) is
  recorded in the ticket that resolves it.

## Scope

### In Scope

- `protos/options.proto` validation-option extensions (`min`/`max`/`abs_max`/
  `req`).
- New `protos/envelope.proto` (`CommandEnvelope`, `ReplyEnvelope`, `Ack`,
  `Error`).
- `scripts/gen_messages.py` extensions: envelope structs, field-descriptor
  tables, `source/messages/wire.{h,cpp}` emission.
- New `source/messages/wire_runtime.{h,cpp}` (hand-written wire codec
  primitives).
- New `source/commands/binary_channel.{h,cpp}` and the `*`-line discriminator
  in `command_processor.cpp::process()`.
- Binary oneof arms: drive, segment (MOVE), replace (MOVER), stop, ping, echo,
  id. Binary `Ack`/`Error` replies.
- Host: `grpc_tools.protoc` Python codegen wiring (justfile/build.py), a `*`
  branch in `io/serial_conn.py`, and `rogo` binary send.
- Differential/fuzz/range test suite for the codec (both directions, vs.
  `google.protobuf`).

### Out of Scope

- Binary telemetry, binary config plane, `StreamControl.binary` — Sprint 2
  (096).
- Deleting any text-plane code, retiring any text verb — Sprint 3 (097).
- `NezhaProtocol` public-API conversion to envelope builders — Sprint 3 (097).
- Camera-fix / pose-estimation work — Sprint 098 (D), which depends on this
  program having landed and must express its own binary surface in terms of
  what Sprints 1–3 establish here.
- Raw COBS framing (deferred indefinitely per the issue — base64 armor
  overhead is irrelevant at 115200 baud).

## Test Strategy

Differential pytest suite comparing the hand-written firmware-side codec
(exercised via a host-side harness or shared test vectors) against
`google.protobuf` encode/decode, in both directions, plus fuzz inputs and
boundary/invalid-range cases per validated field. Full existing sim suite
must stay green throughout (proves the text plane is untouched). Bench gate
per `.claude/rules/hardware-bench-testing.md`: deploy, drive on the stand via
binary MOVE/MOVER/STOP over USB serial AND the radio relay (proves the armor
survives the relay's `#`-skip and line-pipe reassembly), run the text
regression pass, and record the flash delta.

## Architecture Notes

Key constraint: transports are line-oriented and NOT binary-safe at the app
layer (NUL-terminated `readLine()`, `strlen()`-based radio reassembly, a
250-char-ish effective line budget) — this is why the design is ASCII-armored
base64 inside the existing line world rather than raw binary framing. The
`*` prefix cannot collide with uppercase text verbs, replies (`OK`/`ERR`/...),
or relay `#` lines. Budget: ~186-byte max envelope payload, enforced by a
generated `static_assert` on computed max encoded sizes. One oneof arm per
Blackboard queue — the firmware dispatcher is a switch, not a verb table;
subsystems never know which plane a command arrived on. See the issue's
"Design" section for the full schema and the day-one standard-layout decision
gate with its unrolled-codegen fallback, and its "Risks" section (self-
written codec correctness is risk #1, mitigated by the differential/fuzz
tests this sprint builds).

## GitHub Issues

(None — tracked via the CLASI issue file referenced above.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [x] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 095-001 | Wire schema: options.proto validation extensions + envelope.proto + motion.proto | — |
| 095-002 | Host codec mirror: grpc_tools.protoc pb2 codegen + serial_conn.py `*` branch + rogo binary send | 095-001 |
| 095-003 | Day-one decision gate: std::is_standard_layout static_asserts on generated structs | 095-001 |
| 095-004 | wire_runtime.{h,cpp}: hand-written varint/zigzag/fixed32/length-delimited/base64 primitives | 095-001 |
| 095-005 | gen_messages.py: FieldDesc tables + wire.{h,cpp} generation + kMaxEncodedSize<=186 asserts | 095-003, 095-004 |
| 095-006 | Differential/fuzz/range test harness vs google.protobuf | 095-002, 095-004, 095-005 |
| 095-007 | BinaryChannel + `*` discriminator + CommandProcessor/CommandRouter wiring (drive/segment/replace/stop/ping/echo/id) | 095-005, 095-006 |

Tickets execute serially in the order listed. Every ticket carries an
`issue:` back-reference to
`protocol-v3-schema-driven-binary-command-plane-protobuf.md` (see
`clasi/sprints/095-.../issues/`) with `completes_issue: false` — the issue
is a 3-sprint program (095/096/097, plus 098 depends on it) and is not
fully resolved by this sprint alone.
