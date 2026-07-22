---
id: '007'
title: Host protocol.py + state adapter rework for the new telemetry frame
status: done
use-cases:
- SUC-047
- SUC-048
- SUC-049
depends-on:
- '006'
github-issue: ''
issue: telemetry-frame-tightening-amendment-to-gut-s1.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Host protocol.py + state adapter rework for the new telemetry frame

## Description

The bench-toolchain-forced host edit: `pb2` regenerates from
`telemetry.proto`/`envelope.proto`/`config.proto` on every `python
build.py` run (ticket 003 already forced this), so `protocol.py`'s decode
logic must track the new shapes or the host package doesn't import at
all — this is not optional/deferred scope, unlike the dormant host
planner/tour code (sprint.md Architecture Decision 6, explicitly out of
scope for this sprint).

Implements the amendment issue's own "Host changes" section
(`telemetry-frame-tightening-amendment-to-gut-s1.md`): nested-reading
decode, single-ack gated on the `flags` bit, and — the design choice
that keeps this a small, contained change — presence/status/fault/event
exposed as **properties derived from `flags`**, so every existing
TestGUI panel / downstream consumer reading e.g. `.has_otos` or
`.conn_left` keeps working unchanged; only the decode internals move.

## Acceptance Criteria

- [x] `src/host/robot_radio/robot/protocol.py`: decodes the nested
      `EncoderReading`/`OtosReading` messages (`frame.enc_left.position`,
      `.velocity`, `.time`, etc.); single-ack decode gated on `flags`
      bit 5 (`ack_fresh`); `DriveMode` import repointed from
      `planner_pb2` to `telemetry_pb2`.
- [x] Presence/status/fault/event **properties** derived from `flags`
      (e.g. `otos_present`, `otos_connected`, `active`, `conn_left`,
      `conn_right`, the fault bits 6-9, the event bits 10-12, `line_present`
      bit 13, `color_present` bit 14) are exposed with the SAME names
      existing callers already use — verify by grepping
      `src/host/robot_radio/` for every attribute name the OLD
      `has_*`/bool fields exposed, and confirm each has a same-named
      replacement property here.
- [x] `line`/`color` packed-word decode: unpacks the 4-channel `line`
      word (1 byte/channel) and the RGBC `color` word (8 bits/channel)
      into whatever per-channel shape downstream consumers expect
      (check `nezha_state.py`/`robot_state.py`'s existing line/color
      attribute shape before inventing a new one).
- [x] `src/host/robot_radio/robot/nezha_state.py` /
      `robot_state.py`: adapter mapping updated so existing attribute
      names TestGUI panels read are populated from the new
      readings/flags, not the old flat fields.
- [x] `src/host/robot_radio/calibration/sim_boot_config.py`: the
      `planner_pb2.HeadingSourceMode` use (verified today at
      sim_boot_config.py:85-105) removed or repointed — confirm what,
      if anything, in `telemetry_pb2` or elsewhere now serves this
      role; if nothing does, remove the dead code path rather than
      leave a broken import.
- [x] `python build.py` (host package import/build) succeeds — this is
      the ticket where the host package first imports cleanly again
      after ticket 003 changed the `pb2` modules it depends on.

## Implementation Plan

**Approach**: `protocol.py`'s decode core first (the nested-reading and
flags-property logic, in isolation, testable against hand-constructed
`Telemetry` protobuf objects), then the two state adapters (which
consume `protocol.py`'s output, not the wire directly), then
`sim_boot_config.py` (independent, smallest edit). Grep for every
existing consumer of the old attribute names before starting, so the
property-rename risk (silently breaking a TestGUI panel) is caught by
inspection, not by a later runtime `AttributeError`.

**Files to modify**: `src/host/robot_radio/robot/protocol.py`,
`src/host/robot_radio/robot/nezha_state.py`,
`src/host/robot_radio/robot/robot_state.py`,
`src/host/robot_radio/calibration/sim_boot_config.py`.

**Testing plan**: Unit-test `protocol.py`'s decode against a
hand-constructed `telemetry_pb2.Telemetry` message covering: a frame
with `otos_present` set vs. clear, an ack-fresh vs. stale frame, line/color
words with known channel values decoding to the expected unpacked
values. `uv run python -m pytest src/tests/unit/` (the existing
`test_protocol_config.py`/`test_protocol_binary_client.py` suite) green.
Full sim-integration confirmation (decoding a frame actually emitted by
`SimHarness` through the wire codec) rides ticket 009's sweep, once
everything upstream is in place.

**Documentation updates**: none required — no `docs/architecture/` doc
models `protocol.py`'s internals at this granularity; sprint.md's own
Architecture section already documents this module's boundary.

## Completion Notes

- `protocol.py`: `DriveMode`/`_DRIVE_MODE_CHAR` repointed to `telemetry_pb2`
  (the module-level dict was the actual import-time blocker — it evaluated
  `planner_pb2.IDLE` etc. at module load). `AckEntry` reworked to
  `(corr_id, ok, err_code)` — the `status`/`AckStatus` field is gone
  (`AckStatus` deleted wholesale, ticket 003); its old
  `status: int = telemetry_pb2.ACK_STATUS_OK` default was ALSO an
  import-time blocker (dataclass field defaults evaluate at class-body
  execution). Added `AckEntry.from_telemetry()` (replaces `from_pb2()` —
  there is no wire `AckEntry` message any more to adapt from) and two new
  host-side reading dataclasses, `EncoderReading`/`OtosReading`, mirroring
  the wire's per-source messages. `TLMFrame` gained `flags`/`ack_corr`/
  `ack_err`/`ack`/`enc_left`/`enc_right`/`otos_reading` fields plus 15
  flags-derived `@property` accessors (`otos_present`, `otos_connected`,
  `conn_left`, `conn_right`, `ack_fresh`, four `fault_*`, three `event_*`,
  `line_present`, `color_present`) — `active` stays a plain settable field
  (many callers/tests construct `TLMFrame(active=...)` directly) rather
  than a property. `enc`/`vel`/`pose`/`twist`/`otos`/`line`/`color` keep
  their PRE-115 tuple shapes and names (`from_pb2()` derives them from the
  new nested readings) so `testgui/telemetry_panel.py` and every other
  downstream reader needed zero changes. `line`/`color` packed-word decode
  is genuinely new (previously declared on `TLMFrame` but never populated
  by the binary decode path — text-plane-only before this ticket).
  Deleted `move()`/`wait_for_move_terminal()` (built/polled the now-deleted
  `envelope_pb2.Move` message and `AckStatus` completion taxonomy — S1 has
  no MOVE command) and `_PLANNER_KEYS`/the `planner`-patch branches of
  `set_config()`/`config()` (targeted the now-deleted `PlannerConfigPatch`)
  — `minSpeed`/`headingKp`/`headingKd`/`distanceKp`/`arriveDwell` now
  behave like any other unknown key (`ValueError`/`None`, no wire traffic).
- `io/serial_conn.py` (touched beyond this ticket's own guessed file list,
  per the Approach's own "grep first" step): `_match_ack_in_frames()`/
  `SerialConnection.wait_for_ack()` reworked from the depth-3-ring scan to
  the single `ack_corr`/`ack_err` slot (gated on `flags` bit 5). Necessary
  because `NezhaProtocol.wait_for_ack()` delegates the entire match/timeout
  algorithm here (104-003) — this ticket's own "single-ack decode gated on
  the ack_fresh bit" criterion is not actually implemented without this
  file. `wait_for_ack()`'s return type changed from a raw
  `telemetry_pb2.AckEntry` to the matching raw `telemetry_pb2.Telemetry`
  frame (no `AckEntry` wire message exists any more).
- `calibration/sim_boot_config.py`: `_heading_source_wire_value()` deleted
  (its `planner_pb2` import target is gone). Went further than the named
  function per the ticket's own "if nothing does, remove the dead code
  path" reasoning: `planner_boot_config_for()` itself is deleted wholesale
  — EVERY `gen_boot_config.py` function it called (`motion_limits_for_config`,
  `profile_rot_limits_for_config`, `min_speed_for_config`,
  `heading_gains_for_config`, `arrive_dwell_for_config`,
  `heading_source_for_config`, `heading_dwell_for_config`,
  `lead_compensation_for_config`, `actuation_lag_for_config`,
  `distance_gains_for_config`, `model_tau_for_config`) was already deleted
  by ticket 003 alongside `msg::PlannerConfig`, so nothing short of
  restoring 10 deleted functions could keep it working. `motor_boot_config_for()`
  is untouched (depends only on still-live functions).
- `nezha_state.py`/`robot_state.py`: no functional changes needed —
  `NezhaState._apply_tlm()` reads `tlm.enc`/`.pose`/`.twist`/`.line`/
  `.color`/`.t`/`.ekf_rej`, every one of which kept its pre-115 name and
  shape by design (see `protocol.py` note above); `robot_state.py` is a
  plain dataclass with no direct wire dependency. Confirmed by grep (no
  `planner_pb2`/`AckStatus`/`fault_bits`/`queue_depth`/`.acks` references
  in either file) — this satisfies the ticket's own "adapter mapping
  updated" criterion by construction (nothing to update).
- Tests updated beyond the two named in the Testing plan (needed for
  `uv run python -m pytest src/tests/unit/` to be meaningfully green, per
  the plan's own "grep first" Approach): `test_serial_conn_ack_ring.py`
  (direct coverage of the promoted `serial_conn.py` matcher — rewritten
  for the single-slot design, "ring re-delivery"/"ring-wrap" scenarios
  replaced with "ignores non-fresh acks"/"slot-overwrite"),
  `test_twist_stop_ack_matcher.py` sections 2/3 (direct `TLMFrame`/
  `wait_for_ack()` coverage), `test_sim_boot_config.py` (its
  `planner_boot_config_for()` tests deleted along with the function;
  `motor_boot_config_for()` tests kept verbatim).
- **Verification**: `uv run python -m pytest src/tests/unit/` —
  `test_protocol_config.py` 26/26 passed, `test_protocol_binary_client.py`
  24/24 passed (the two suites this ticket's Testing plan names), plus
  `test_serial_conn_ack_ring.py` 13/13, `test_twist_stop_ack_matcher.py`
  11/11, `test_sim_boot_config.py` 10/10. Full `src/tests/unit/` directory:
  337 passed, 19 failed, 1 collection error — every failure is
  pre-existing/out-of-scope: `test_planner_executor.py` (17 failures) and
  `test_planner_tour.py` (1 collection error) are the dormant `planner/`
  package (explicitly out of scope per sprint.md Architecture Decision 6;
  their `TLMFrame(fault_bits=..., event_bits=...)` construction calls and
  `telemetry_pb2.ACK_STATUS_DONE` reference are ticket 009's sweep).
  `test_check_config_sync.py` (2 failures) hits `config_pb2.PlannerConfigPatch`
  — a residue of ticket 003's `config.proto` surgery in a CI lint script
  (`src/scripts/check_config_sync.py`) this ticket does not own; also
  ticket 009's "PlannerConfig" grep-sweep territory. `python build.py`
  (full firmware + host sim lib) built clean end to end, zero errors;
  regenerated `pb2`/`messages`/`boot_config.cpp` output matched the
  already-committed tree exactly (`git status` showed no diff).
