---
id: '007'
title: Host binary telemetry and config client (TLMFrame-from-pb2, NezhaProtocol binary
  set/get)
status: open
use-cases: [SUC-006]
depends-on: ['001', '004', '005']
github-issue: ''
issue: protocol-v3-schema-driven-binary-command-plane-protobuf.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Host binary telemetry and config client (TLMFrame-from-pb2, NezhaProtocol binary set/get)

## Description

Give the host a binary telemetry/config client built on 095's already-
generic envelope demux, with zero call-site change to existing consumers
(TestGUI/teleop/bench/MCP). Depends on ticket 001 (pb2 schema must exist
and regenerate), and on tickets 004/005 (firmware behavior to test the
client against — host-side round-trip tests need real firmware/sim
behavior on the other end).

**Approach**:
1. `host/robot_radio/robot/protocol.py`: give `TLMFrame` an alternate
   constructor (e.g. `TLMFrame.from_pb2(telemetry: pb2.Telemetry)`)
   producing the SAME dataclass shape the existing text `parse_tlm()`
   produces — so `TestGUI`/teleop/bench/MCP call sites need zero changes.
   Do NOT change `TLMFrame`'s existing fields/shape to accommodate this;
   the alternate constructor adapts pb2's shape to the existing dataclass,
   not the other way around.
2. `NezhaProtocol` gains binary set/get config methods (building
   `CommandEnvelope{config: ConfigDelta}`/`{get: ConfigGet}` via
   `send_envelope()`, parsing the `Ack`/`ConfigSnapshot` reply) alongside
   its existing text `SET`/`GET` wrappers — same public-API-stability
   posture 095 established for drive/segment/replace (`NezhaProtocol`
   keeps its public API; only method bodies/new methods are envelope
   builders).
3. No change to `host/robot_radio/io/serial_conn.py` — 095's
   `ReplyEnvelope` reader-thread branch already demuxes by `corr_id`
   generically regardless of which `body` oneof arm arrives (`tlm`/`cfg`
   route through the SAME `_reply_queues`/`_tlm_queue` machinery `ok`/
   `err`/`id`/`echo` already use). Verify this rather than assuming it —
   confirm no new branch is needed as part of this ticket's own testing.

**Files to modify**: `host/robot_radio/robot/protocol.py`.

## Acceptance Criteria

- [ ] `TLMFrame.from_pb2(telemetry)` produces a `TLMFrame` field-for-field
      equal to what parsing the matching text TLM line would have
      produced, for every field both formats carry.
- [ ] `NezhaProtocol`'s binary config set/get round-trips against the
      differential test harness's host-side codec (ticket 006's
      machinery) without needing live hardware.
- [ ] No existing `NezhaProtocol`/`TestGUI`/teleop call site changes
      signature or behavior — verified by running the existing host test
      suite unmodified.
- [ ] `serial_conn.py`'s `ReplyEnvelope` demux correctly routes `tlm`/
      `cfg` body arms through the existing `_reply_queues`/`_tlm_queue`
      machinery with zero code changes to that file (confirmed, not
      assumed).

## Testing

- **Existing tests to run**: full host test suite (`uv run python -m
  pytest host/` or the project's established host test invocation);
  confirm zero regressions to existing `NezhaProtocol`/`TLMFrame`
  consumers.
- **New tests to write**: unit tests for `TLMFrame.from_pb2()` against a
  hand-constructed `pb2.Telemetry`; unit tests for the new binary config
  set/get methods against the differential harness's reference codec.
- **Verification command**: `uv run python -m pytest`
