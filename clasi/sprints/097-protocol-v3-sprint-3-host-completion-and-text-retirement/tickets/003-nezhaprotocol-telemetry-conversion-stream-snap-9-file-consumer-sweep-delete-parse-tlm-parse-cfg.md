---
id: '003'
title: NezhaProtocol telemetry conversion (stream/snap) + 9-file consumer sweep +
  delete parse_tlm/parse_cfg
status: open
use-cases: [SUC-003]
depends-on: ['001']
github-issue: ''
issue: protocol-v3-schema-driven-binary-command-plane-protobuf.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# NezhaProtocol telemetry conversion (stream/snap) + 9-file consumer sweep + delete parse_tlm/parse_cfg

## Description

Convert `NezhaProtocol.stream(period)` to send
`CommandEnvelope{stream: StreamControl{period, binary: true}}`.

Convert `NezhaProtocol.snap()` by **synthesizing** its existing one-shot
`TLMFrame | None` contract host-side from the already-implemented binary
`stream` arm — no new firmware wire capability is added (architecture
Decision 4, honoring the sprint's "no new binary functionality" scope
boundary, and closing 096's own Open Question 2): drain
`_binary_tlm_queue` (ticket 001) of stale frames, arm a brief period
(`StreamControl{period: <floor, e.g. kStreamFloorMs>, binary: true}`),
wait for exactly one frame off `_binary_tlm_queue`, disarm
(`StreamControl{period: 0, binary: true}`), and return the resulting
`TLMFrame` (built via the already-existing `TLMFrame.from_pb2()`, 096-007)
or `None` on timeout.

Sweep every internal, non-test call site of the module-level `parse_tlm()`
off raw text `TLM ...` lines onto binary-native `TLMFrame` delivery
(sourced from ticket 001's `_binary_tlm_queue`): `host/robot_radio/robot/
nezha.py`, `nezha_state.py`, `host/robot_radio/testgui/transport.py`,
`host/robot_radio/calibration/linear.py`, `angular.py`,
`fit_sim_error_model.py`, `host/robot_radio/sensors/odom_tracker.py`,
`host/robot_radio/io/cli.py`, `tests/playfield/world_goto_chart.py` — nine
files, all found by direct grep during architecture research (none of
these are enumerated in `sprint.md`; see `architecture-update.md` Step 1
and Decision 3). Every one of these edits is the SAME mechanical change
(swap a text-parse call for an already-adapted `TLMFrame` object), for the
SAME reason — this is NOT nine unrelated changes, it is one conceptual
change applied nine times.

Once the sweep is complete and zero real call sites remain, delete the
module-level `parse_tlm()`/`parse_cfg()` functions and the
`NezhaProtocol.parse_tlm`/`.parse_cfg` static-method mirrors.
`parse_cfg()` has ZERO real call sites already (grep-confirmed during
architecture research) and is trivially deletable regardless of this
ticket's other work.

`stream_fields()` is explicitly OUT of scope (it sends a `fields=` kv the
current text `STREAM` handler has never accepted — pre-existing broken
method, tracked by the separate
`realign-host-tooling-to-gutted-four-verb-wire-surface.md` issue, not
re-scoped here).

## Acceptance Criteria

- [ ] `stream()` sends `*B<base64>` on the wire; return type (`None`) is
      unchanged.
- [ ] `snap()` sends/receives entirely over the binary plane via the
      arm-wait-disarm sequence described above; its return type/shape
      (`TLMFrame | None`) and public contract are unchanged. Its docstring
      is updated to describe the new implementation strategy.
- [ ] Every one of the nine listed internal consumer files is updated to
      source its `TLMFrame` from the binary plane (`_binary_tlm_queue` /
      `TLMFrame.from_pb2()`), not from `parse_tlm(line)` on a text line.
- [ ] `grep -rn "parse_tlm" host/` (excluding the deleted function's own
      former definition and test files exercising `TLMFrame.from_pb2`'s
      historical text/binary parity claim, e.g.
      `tests/unit/test_protocol_binary_client.py`) returns no hits.
- [ ] `parse_tlm`/`parse_cfg` (module-level functions) and
      `NezhaProtocol.parse_tlm`/`.parse_cfg` (static wrappers) are
      deleted.
- [ ] `stream_fields()` is byte-for-byte untouched by this ticket's diff.
- [ ] `tests/sim` stays green (host-only ticket; sanity check).
- [ ] `tests/unit` is green, including updated tests for every swept
      consumer file and for `stream()`/`snap()`.

## Implementation Plan

### Approach

1. Convert `stream()` (straightforward — its `period` argument maps 1:1
   onto `StreamControl.period`).
2. Implement `snap()`'s arm-wait-disarm synthesis, reusing ticket 001's
   `_binary_tlm_queue` drain accessor.
3. Sweep the nine consumer files one at a time: replace each
   `parse_tlm(line)` call site with the binary-native `TLMFrame` already
   delivered by the reader thread, preserving each file's own surrounding
   control flow (this is a targeted swap, not a rewrite of these files'
   business logic).
4. Grep-confirm zero remaining real call sites, then delete
   `parse_tlm`/`parse_cfg` (module-level) and their `NezhaProtocol`
   static-method mirrors.

### Files to modify

- `host/robot_radio/robot/protocol.py` — `stream()`/`snap()` bodies;
  deletion of `parse_tlm`/`parse_cfg`/static wrappers.
- `host/robot_radio/robot/nezha.py`
- `host/robot_radio/robot/nezha_state.py`
- `host/robot_radio/testgui/transport.py`
- `host/robot_radio/calibration/linear.py`
- `host/robot_radio/calibration/angular.py`
- `host/robot_radio/calibration/fit_sim_error_model.py`
- `host/robot_radio/sensors/odom_tracker.py`
- `host/robot_radio/io/cli.py`
- `tests/playfield/world_goto_chart.py`

### Testing plan

- New/updated host unit tests for `stream()`/`snap()` against a fake
  serial port, including a `snap()` timeout case (no frame arrives).
- For each of the nine swept files, run/update that file's own existing
  test coverage (where it exists) to confirm the swap didn't change
  observable behavior.
- `grep -rn "parse_tlm" host/` as an explicit, automatable acceptance
  check (not just a manual review).
- Run `tests/unit` (host suite) and `tests/sim` (sanity — unaffected, no
  firmware files touched).

### Documentation updates

- `snap()`'s docstring (implementation-strategy note, contract unchanged).
- None required for the nine swept files beyond inline comments explaining
  the new `TLMFrame` source, matching each file's existing comment style.
