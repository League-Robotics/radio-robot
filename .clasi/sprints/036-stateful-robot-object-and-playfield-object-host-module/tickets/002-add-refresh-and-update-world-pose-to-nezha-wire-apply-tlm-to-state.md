---
id: '002'
title: Add refresh() and update_world_pose() to Nezha; wire _apply_tlm to state
status: done
use-cases:
- SUC-002
- SUC-007
depends-on:
- '001'
github-issue: ''
issue: plan-stateful-robot-object-playfield-object-for-the-host-module.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Add refresh() and update_world_pose() to Nezha; wire _apply_tlm to state

## Description

With `RobotState` extended and `_apply_tlm` wired (T001), this ticket adds two
new public methods to `Nezha`:

- `refresh() -> RobotState`: issues a one-shot `SNAP` via `self._proto.snap()`
  (protocol.py:676), passes the returned `TLMFrame` to `_apply_tlm`, and returns
  `self.state`. This is how a caller queries fresh state when no drive command is
  active, without starting continuous streaming.
- `update_world_pose(x_cm: float, y_cm: float, yaw_rad: float) -> None`:
  converts camera-native units to firmware units (`mm = cm * 10`, `cdeg =
  round(degrees(yaw_rad) * 100)`) and calls `self._proto.otos_set_position` (the
  `OV` / SI path at protocol.py:719). Also writes `(x_cm, y_cm, yaw_rad)` into
  `state.world_pose` (via a new frozen replacement of `self.state`).

Both methods are the "idle" half of the state model — no streaming, no callback
loop. The `test_robot_state.py` test file created in T001 is extended here with
`refresh()` and `update_world_pose()` coverage.

Also wire `_apply_tlm` into the existing `stream_drive` path in `Nezha` — the
existing `stream_drive` method already calls `_apply_tlm` (nezha.py:242–244), so
confirm this path continues to work after T001's rewrite of `_apply_tlm`.

## Acceptance Criteria

- [x] `Nezha.refresh()` calls `self._proto.snap()`, passes the result to
      `_apply_tlm`, and returns the updated `self.state`. Returns the prior
      `self.state` if `snap()` returns `None` (no TLM in response).
- [x] `Nezha.update_world_pose(x_cm, y_cm, yaw_rad)` calls `set_world_pose`
      (via `self._proto.otos_set_position`) with `x_mm = round(x_cm * 10)`,
      `y_mm = round(y_cm * 10)`, `h_cdeg = round(math.degrees(yaw_rad) * 100)`.
- [x] After `update_world_pose`, `self.state.world_pose == (x_cm, y_cm, yaw_rad)`.
- [x] `Nezha.snap()` (the existing low-level method at nezha.py:294) is NOT
      removed; `refresh()` is a new higher-level wrapper that also updates state.
- [x] `uv run --with pytest python -m pytest host/tests/test_robot_state.py -q`
      — all tests including the new ones below pass.

## Implementation Plan

### Approach

Add `refresh()` and `update_world_pose()` directly in `host/robot_radio/robot/nezha.py`
in the "Telemetry" and "OTOS sensor management" sections respectively. For
`update_world_pose`, the unit conversion is a one-liner; the state update uses
Python's `dataclasses.replace` to produce a new frozen instance with `world_pose`
set.

### Files to Modify

- `host/robot_radio/robot/nezha.py` — add `refresh()` below `snap()` in the
  Telemetry section; add `update_world_pose()` in the OTOS section.

### Testing Plan

Extend `host/tests/test_robot_state.py`:

1. `test_refresh_issues_snap_and_updates_state` — mock `_proto.snap()` to return
   a known `TLMFrame`; call `robot.refresh()`; assert returned `RobotState` has
   correct fields and `robot.state` is updated.
2. `test_refresh_when_snap_returns_none` — mock `_proto.snap()` returning `None`;
   call `refresh()`; assert prior state is returned unchanged.
3. `test_update_world_pose_unit_conversion` — mock `_proto.otos_set_position`;
   call `robot.update_world_pose(10.0, -5.0, math.pi/2)`; assert
   `otos_set_position` called with `(100, -50, 9000)`.
4. `test_update_world_pose_stores_world_pose` — after `update_world_pose`, assert
   `robot.state.world_pose == (10.0, -5.0, math.pi/2)`.

Verification: `uv run --with pytest python -m pytest host/tests/test_robot_state.py
host/tests/test_nezha_drive.py -q`

### Documentation Updates

Add docstrings to `refresh()` and `update_world_pose()` explaining unit conventions
(cm/rad in, mm/cdeg to wire, world_pose stored in cm/rad).
