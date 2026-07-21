---
id: '007'
title: Host protocol.py + state adapter rework for the new telemetry frame
status: open
use-cases: [SUC-047, SUC-048, SUC-049]
depends-on: ["006"]
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

- [ ] `src/host/robot_radio/robot/protocol.py`: decodes the nested
      `EncoderReading`/`OtosReading` messages (`frame.enc_left.position`,
      `.velocity`, `.time`, etc.); single-ack decode gated on `flags`
      bit 5 (`ack_fresh`); `DriveMode` import repointed from
      `planner_pb2` to `telemetry_pb2`.
- [ ] Presence/status/fault/event **properties** derived from `flags`
      (e.g. `otos_present`, `otos_connected`, `active`, `conn_left`,
      `conn_right`, the fault bits 6-9, the event bits 10-12, `line_present`
      bit 13, `color_present` bit 14) are exposed with the SAME names
      existing callers already use — verify by grepping
      `src/host/robot_radio/` for every attribute name the OLD
      `has_*`/bool fields exposed, and confirm each has a same-named
      replacement property here.
- [ ] `line`/`color` packed-word decode: unpacks the 4-channel `line`
      word (1 byte/channel) and the RGBC `color` word (8 bits/channel)
      into whatever per-channel shape downstream consumers expect
      (check `nezha_state.py`/`robot_state.py`'s existing line/color
      attribute shape before inventing a new one).
- [ ] `src/host/robot_radio/robot/nezha_state.py` /
      `robot_state.py`: adapter mapping updated so existing attribute
      names TestGUI panels read are populated from the new
      readings/flags, not the old flat fields.
- [ ] `src/host/robot_radio/calibration/sim_boot_config.py`: the
      `planner_pb2.HeadingSourceMode` use (verified today at
      sim_boot_config.py:85-105) removed or repointed — confirm what,
      if anything, in `telemetry_pb2` or elsewhere now serves this
      role; if nothing does, remove the dead code path rather than
      leave a broken import.
- [ ] `python build.py` (host package import/build) succeeds — this is
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
