---
id: '001'
title: 'Prune wire protocol: twist/config/stop + ack-ring telemetry'
status: open
use-cases: [SUC-001]
depends-on: []
github-issue: ''
issue: single-loop-firmware-p3-p7-continuation.md
completes_issue:
  single-loop-firmware-p3-p7-continuation.md: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Prune wire protocol: twist/config/stop + ack-ring telemetry

## Description

Prune `protos/envelope.proto` and `protos/telemetry.proto` to the P4 wire
surface the whole rest of this sprint builds on: `CommandEnvelope` narrows
to `corr_id` + `oneof { Twist; ConfigDelta; Stop }`; `ReplyEnvelope`
narrows to `ok`/`err`/`tlm`; `Telemetry` gains a depth-3 ack ring
(`repeated AckEntry acks`) and `fault_bits`/`event_bits`; the
bench-diagnostic fields move to a new `TelemetrySecondary` message. This is
a re-derivation, not a merge, of sprint 102's spike-003 dry run
(`scratch/102-003-frame-budget`, never merged) — re-run the measurement for
real against this sprint's own field list rather than inheriting the
scratch branch's judgment calls silently.

This ticket is the sprint's foundation: every `source/app/` module
(tickets 004-007) and the host slice (ticket 009) compile against the
types this ticket generates. It has no dependencies and should land first.

## Acceptance Criteria

- [ ] `CommandEnvelope.cmd` reduced to exactly `Twist twist`, `ConfigDelta
      config`, `Stop stop`; every other pre-102 arm (drive, segment,
      replace, pose_fix, otos, ping, echo, get, stream, id, hello, ver,
      help, plan_dump) removed; its field number `reserved`, not reused.
- [ ] `ReplyEnvelope.body` reduced to exactly `Ack ok`, `Error err`,
      `Telemetry tlm`; every other arm removed and reserved.
- [ ] `Twist{v_x, omega, duration}` defined (new message).
- [ ] `Telemetry` gains `repeated AckEntry acks` (ring depth 3;
      `AckEntry{corr_id, status, err_code}`) and `fault_bits`/`event_bits`
      (`uint32`, bit layout decided and documented in this ticket's
      completion notes — spike-003 left it undefined).
- [ ] `acc_left/acc_right/glitch_left/glitch_right/ts_left/ts_right/
      has_cmd_vel/cmd_vel_left/cmd_vel_right` move to a new
      `TelemetrySecondary` message.
- [ ] `TelemetrySecondary`'s wire framing (a second `*B`-armored line vs. a
      `ReplyEnvelope` oneof arm) is decided and documented (Decision 3,
      architecture-update.md) — not left implicit.
- [ ] `scripts/gen_messages.py` and `scripts/gen_pb2.py` run clean against
      the pruned protos; `source/messages/{envelope,telemetry,
      layout_checks,wire}.{h,cpp}` and host `envelope_pb2`/`telemetry_pb2`
      regenerated.
- [ ] `wire.h`'s `kCommandEnvelopeMaxEncodedSize`/`kReplyEnvelopeMaxEncodedSize`
      static_asserts pass; both worst-case sizes and their margin against
      186B are recorded in the ticket's completion notes (mirroring
      spike-003's own reporting style).
- [ ] `tests/sim/unit/wire_codec_harness.cpp`, `test_wire_codec.py`,
      `test_wire_differential.py`, `test_wire_fuzz.py` rewritten against
      the pruned schema; the protobuf differential oracle
      (`_wire_diff_driver.py`) still runs and passes.
- [ ] No hardware needed for this ticket (schema/codegen/host-compile
      only).

## Implementation Plan

**Approach**: Work schema-first. Draft the pruned `.proto` files, run the
generators, read the real `kMaxEncodedSize` numbers `gen_messages.py`
prints (not spike-003's numbers — re-derive), adjust the field set if the
budget doesn't fit, then rewrite the wire test harnesses last (they can
only be written against a schema that has already stabilized).
`scratch/102-003-frame-budget` (commit `10985ec1d4`) is a reference
starting point, not something to `git cherry-pick` verbatim — confirm each
of its judgment calls (keeping `active`/`conn_left`/`conn_right` in the
primary frame; `ReplyEnvelope` staying a wrapper type) against this
ticket's own measurement before relying on them.

**Files to create/modify**:
- `protos/envelope.proto` — prune `CommandEnvelope`/`ReplyEnvelope`, add
  `Twist`.
- `protos/telemetry.proto` — add ack ring + fault/event bits; split out
  `TelemetrySecondary`.
- `source/messages/{envelope,telemetry,layout_checks,wire}.{h,cpp}` —
  regenerated, not hand-edited.
- Host `envelope_pb2.py`/`telemetry_pb2.py` (wherever `gen_pb2.py` emits
  them) — regenerated.
- `tests/sim/unit/wire_codec_harness.cpp`,
  `tests/sim/unit/wire_differential_harness.cpp`,
  `tests/sim/unit/test_wire_codec.py`,
  `tests/sim/unit/test_wire_differential.py`,
  `tests/sim/unit/test_wire_fuzz.py` — rewritten against the pruned arm
  set.

**Testing plan**:
- Existing tests to run: `tests/sim/unit/test_wire_runtime.py` (armor/
  base64 primitives — unchanged, should stay green untouched).
- New/rewritten tests: the four wire test files above, targeting
  `twist`/`config`/`stop`/`tlm`/`ok`/`err` only.
- Verification command: `uv run python -m pytest tests/sim/unit/test_wire_codec.py
  tests/sim/unit/test_wire_differential.py tests/sim/unit/test_wire_fuzz.py
  tests/sim/unit/test_wire_runtime.py`; plus a direct compile check
  (`c++ -std=c++20 -Wall -Wextra -I source -c source/messages/wire.cpp`)
  matching spike-003's own verification method.

**Documentation updates**: record the final worst-case sizes/margins and
the `TelemetrySecondary` framing decision in this ticket's own completion
notes (architecture-update.md Decision 3 references this ticket's
resolution).
