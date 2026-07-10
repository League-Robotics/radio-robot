---
id: '007'
title: Host binary telemetry and config client (TLMFrame-from-pb2, NezhaProtocol binary
  set/get)
status: done
use-cases:
- SUC-006
depends-on:
- '001'
- '004'
- '005'
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

- [x] `TLMFrame.from_pb2(telemetry)` produces a `TLMFrame` field-for-field
      equal to what parsing the matching text TLM line would have
      produced, for every field both formats carry.
- [x] `NezhaProtocol`'s binary config set/get round-trips against the
      differential test harness's host-side codec (ticket 006's
      machinery) without needing live hardware.
- [x] No existing `NezhaProtocol`/`TestGUI`/teleop call site changes
      signature or behavior — verified by running the existing host test
      suite unmodified.
- [x] `serial_conn.py`'s `ReplyEnvelope` demux correctly routes `tlm`/
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

## Completion Notes

**Implementation**: `host/robot_radio/robot/protocol.py` gains a top-level
`from robot_radio.robot.pb2 import envelope_pb2, planner_pb2, telemetry_pb2`
(confirmed safe by direct experiment against all four import orderings —
unlike `serial_conn.py`, `robot_radio.robot.pb2` has no dependency back onto
`robot_radio.robot`/`robot_radio.io`, so no circular-import hazard exists
here); `TLMFrame.from_pb2(telemetry)` classmethod adapts a `pb2.Telemetry`
onto TLMFrame's existing dataclass shape (t/mode/seq always copied;
enc/vel/cmd_vel/pose/otos/twist gated by their `has_*` flag, matching
`parse_tlm()`'s own key-presence gating); `NezhaProtocol.set_config_binary()`/
`get_config_binary()` build `CommandEnvelope{config: ConfigDelta}`/
`{get: ConfigGet}`, call `send_envelope()`, and unwrap the `ok`/`cfg` reply
arm (returning `None` on timeout, not-connected, or an `Error` reply — same
lax failure posture the existing text `set_config()`/`get_config()` already
have). `serial_conn.py` is untouched (confirmed via `git diff`, and by two
new demux tests below).

**Field mapping / unshared fields** (from `from_pb2()`'s own doc comment):
shared (compared field-for-field in tests): `t`, `mode` (via a `modeChar()`-
mirroring dict, `IDLE/STREAMING/TIMED/DISTANCE/GO_TO` -> `I/S/T/D/G`,
`VELOCITY` falling back to `I` exactly like `modeChar()`'s own `default`
case), `seq`, `enc`, `vel`, `cmd_vel`, `pose` (heading converted rad->cdeg
via the same `kAngleScale`/`static_cast<int>` truncation `buildTlmFrame()`
applies), `otos` (same conversion), `twist` (differential 2-tuple only —
`v_y` dropped, matching the text plane's own 2-value `twist=%d,%d`).
TLMFrame-only fields with no `Telemetry` counterpart, left `None`: `wedge`,
`encpose` (trimmed at 096-001), `otos_health`. `Telemetry`-only fields with
no TLMFrame slot, silently dropped: `otos_connected` (parse_tlm() never
parsed the text plane's own `otosconn=` token either) and the nine
bench-diagnostic fields (`acc_left/acc_right/active/conn_left/conn_right/
glitch_left/glitch_right/ts_left/ts_right`) that `Telemetry` curates from
the SEPARATE one-shot `TLM` verb's `OK tlm ...` reply (`handleTlm()`,
motion_commands.cpp) — a different text wire shape than the STREAM/SNAP
`TLM t=... mode=...` line TLMFrame/`parse_tlm()` model.

**Binary config API added**: `set_config_binary(delta: ConfigDelta,
read_timeout=500) -> Ack | None` and `get_config_binary(target: int,
read_timeout=500) -> ConfigSnapshot | None`, alongside (not replacing) the
existing text `set_config()`/`get_config()`. Callers build the `ConfigDelta`/
target themselves via `envelope_pb2`/`config_pb2` — mirrors
`send_envelope()`'s own "caller builds the envelope" contract one layer up.

**Tests**: `tests/unit/test_protocol_binary_client.py` (new, 9 tests) —
`from_pb2()` vs `parse_tlm()` field-for-field agreement, absent-field/mode-
mapping/dropped-bench-diagnostic-field coverage, and `set_config_binary()`/
`get_config_binary()` round-tripped through a real `SerialConnection` +
reader thread against a synthetic loopback transport (no live hardware),
including a `watchdog` bare-uint32 oneof-arm case, a timeout case, and an
`Error`-reply case. `tests/unit/test_serial_conn_binary_plane.py` (095-002)
extended with two new `_reader_loop` demux tests exercising the `tlm`/`cfg`
body arms specifically (095's own tests only covered `ok`/`err`) — confirms
096's new arms route through the same corr-id-keyed `_reply_queues` with
zero `serial_conn.py` changes.

**Verification**: `tests/unit/test_protocol_binary_client.py` +
`tests/unit/test_serial_conn_binary_plane.py`: 25 passed. Full
`tests/unit`: 29 passed. `just build-sim` clean (regenerates
`host/robot_radio/robot/pb2/` from the same `protos/*.proto` — `git diff`
confirms zero drift). `uv run python -m pytest tests/sim`: 600 passed in
~103s. `git diff --stat -- host/robot_radio/io/serial_conn.py`: empty
(zero changes, confirmed not assumed). Full default collection
(`tests/sim` + `tests/unit` + `tests/testgui`, `uv run python -m pytest`):
993 passed, 16 failed -- all 16 failures are in `tests/testgui/` (a
real-sim-driven GUI HITL tier, not one of `tests/CLAUDE.md`'s three
sim/bench/playfield domains) and reproduce byte-for-byte (same 16 test
IDs, same "no moving TLMFrame observed" symptom) with this ticket's own
changes fully `git stash`ed out and `tests/testgui` re-run in isolation
(348 passed, same 16 failed) -- confirmed PRE-EXISTING, not a regression
this ticket introduced, before restoring the stash.
