---
id: '007'
title: Host protocol.py MOVE cutover
status: open
use-cases: [SUC-050, SUC-051]
depends-on: ['001']
github-issue: ''
issue:
- protocol-set-point-the-minimal-firmware-s-complete-command-surface.md
- gut-to-minimal-firmware-motion-stack-excision-move-protocol-minimal-telemetry.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Host protocol.py MOVE cutover

## Description

Only needs ticket 001's regenerated `envelope_pb2` — independent of the
firmware-side tickets 002-006, so it can run any time after 001. Adds
`NezhaProtocol.move_twist(v_x, v_y, omega, stop=..., timeout=...,
replace=..., id=...)` / `move_wheels(v_left, v_right, stop=..., timeout=...,
replace=..., id=...)` builders per the new arm-21 `Move` shape; deletes
`NezhaProtocol.twist()` (its wire arm, `Twist`, is gone). Also folds in
the 115 handoff item: `_DRIVE_MODE_CHAR` is missing `DriveMode.VELOCITY`
(decodes as `"I"` while driving — noted in
`docs/bench-checklists/sprint-115-gut-s1.md`). And removes `sTimeout`/
`watchdog` from `config()`/`set_config()`'s curated key vocabulary
(`_ALL_SET_KEYS`) — `ConfigDelta.watchdog` is deleted by ticket 001, so
both builders' existing `envelope_pb2.ConfigDelta(watchdog=...)` call
sites would otherwise break at the protobuf level the moment `envelope_pb2`
regenerates.

Confirmed by grep against the live (non-`archive/`) tree: `.twist(` is
called from `protocol.py` itself, `test_twist_stop_ack_matcher.py`,
`test_rig_dev.py`, and bench scripts (`twist_drive.py`, `rig_dev.py`,
`rig_soak.py`, `velocity_step_response.py`); `sTimeout`/`watchdog` is
tested directly in `test_protocol_config.py` and
`test_protocol_binary_client.py`. Per the parent gut issue's own scope
note ("host motion/tour code stays in place, dormant/broken; only
bench-toolchain-forced edits land here"), this ticket fixes `protocol.py`
itself and its direct unit tests, plus whichever bench script ticket 010's
own gate actually invokes — everything else stays dormant/broken and is
explicitly listed as out-of-scope in this ticket's own completion notes,
not silently left broken without a record.

## Acceptance Criteria

- [ ] `NezhaProtocol.move_twist(...)` and `move_wheels(...)` added,
      building `CommandEnvelope{move: Move{...}}` per the arm-21 shape;
      both fire-and-poll (matching `twist()`/`stop()`'s existing
      "telemetry-only return path" shape), returning the corr_id for
      `wait_for_ack()`.
- [ ] `NezhaProtocol.twist()` deleted.
- [ ] `_DRIVE_MODE_CHAR` gains `telemetry_pb2.VELOCITY: "V"` (or
      whichever character the implementer confirms doesn't collide with
      the existing I/S/T/D/G set).
- [ ] `sTimeout`/`watchdog` removed from `_ALL_SET_KEYS`; `config()`/
      `set_config()` no longer build `ConfigDelta(watchdog=...)` — a
      `sTimeout=` kwarg now raises the same "unknown key" `ValueError`
      any other bogus key does.
- [ ] `test_twist_stop_ack_matcher.py` rewritten (or split/renamed) to
      cover `move_twist()`/`move_wheels()` instead of `twist()`.
- [ ] Watchdog-specific test cases deleted/updated:
      `test_config_watchdog_key_builds_correct_envelope`,
      `test_config_spanning_drivetrain_and_watchdog_raises_value_error`
      (`test_protocol_config.py`); `test_set_config_binary_watchdog_arm`,
      `test_set_config_watchdog_key_sends_binary_and_returns_applied`
      (`test_protocol_binary_client.py`).
- [ ] Whichever bench script(s) ticket 010's own gate depends on for
      driving MOVEs (e.g. `twist_drive.py`) are ported to
      `move_twist`/`move_wheels`.
- [ ] Every other live `.twist(`/`sTimeout` caller not covered above
      (`rig_dev.py`, `test_rig_dev.py`, `rig_soak.py`,
      `velocity_step_response.py`, and any TestGUI/host motion-code
      caller) is explicitly listed in this ticket's completion notes as
      left dormant/broken, per the parent gut issue's scope — not
      silently discovered later.

## Testing

- **Existing tests to run**: `src/tests/unit/test_protocol_config.py`,
  `test_protocol_binary_client.py`, `test_twist_stop_ack_matcher.py`
  (post-rewrite).
- **New tests to write**: `move_twist()`/`move_wheels()` envelope-building
  tests mirroring the deleted `twist()` tests' shape (correct fields,
  fresh corr_id per call); a `_DRIVE_MODE_CHAR` regression test asserting
  `VELOCITY` no longer falls back to `"I"`.
- **Verification command**: `uv run python -m pytest src/tests/unit/`
