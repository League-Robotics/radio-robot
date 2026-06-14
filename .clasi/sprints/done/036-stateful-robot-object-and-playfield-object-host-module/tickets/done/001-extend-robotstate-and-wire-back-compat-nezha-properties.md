---
id: '001'
title: Extend RobotState and wire back-compat Nezha properties
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-007
depends-on: []
github-issue: ''
issue: plan-stateful-robot-object-playfield-object-for-the-host-module.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Extend RobotState and wire back-compat Nezha properties

## Description

`RobotState` (at `host/robot_radio/robot/robot_state.py`) is currently a frozen
dataclass carrying only `pose`, `v`, `omega`, `accel`, `stamp`. `Nezha` exposes
sensor data as separate instance attributes (`encoders`, `otos_pose`,
`line_sensor`, `color`).

This ticket extends `RobotState` to carry the full TLM payload — `encoders`,
`twist`, `line`, `color`, `world_pose`, and all fields already present — and
wires `Nezha._apply_tlm` to populate the new unified `state` field. The existing
per-attribute reads on `Nezha` become thin properties over `state`, preserving
all existing callers without any import or usage change.

This is the state-model foundation that all later tickets (T002–T006) depend on.

The `Robot` ABC (`host/robot_radio/robot/robot.py`) also receives two minor
additions needed to keep `Cutebot` compliant:
- `go_to` gains `on_tick=None` and `timeout_s=15.0` keyword params (no body change).
- `turn` is added as an abstract stub raising `NotImplementedError`.

## Acceptance Criteria

- [x] `host/robot_radio/robot/robot_state.py` — `RobotState` is a frozen dataclass
      with fields: `pose` (`Pose` from `nav.pose`, retaining x_mm/y_mm/heading_rad
      semantics), `encoders` (tuple[int,int] | None, default None), `twist`
      (tuple[int,int] | None for v_mmps/omega_mradps, default None), `line`
      (tuple[int,int,int,int] | None, default None), `color`
      (tuple[int,int,int,int] | None, default None), `world_pose`
      (tuple[float,float,float] | None, default None), `v` (float, retained),
      `omega` (float, retained), `accel` (tuple[float,float] | None, retained),
      `stamp` (float, retained).
- [x] `host/robot_radio/robot/nezha.py` — `Nezha.__init__` initialises
      `self.state` as a default `RobotState` with all-None/zero fields; the bare
      `self.encoders/otos_pose/line_sensor/color` instance attribute assignments
      are removed from `__init__`.
- [x] `Nezha._apply_tlm(tlm)` constructs and assigns a new frozen `RobotState`
      from the incoming `TLMFrame` fields. Fields absent from the frame retain
      their previous value (partial-frame handling — copy unchanged fields from
      the prior `self.state`).
- [x] `Nezha.encoders` (property) returns `self.state.encoders or (0, 0)`.
- [x] `Nezha.otos_pose` (property) returns `(state.pose.x, state.pose.y,
      state.pose.heading)` or `(0.0, 0.0, 0.0)`.
- [x] `Nezha.line_sensor` (property) returns `self.state.line or (255,255,255,255)`.
- [x] `Nezha.color` (property) returns `self.state.color or (0,0,0,0)`.
- [x] `robot.py` `Robot` ABC: `go_to` gains `on_tick=None`, `timeout_s=15.0`
      keyword params; `turn` abstract stub added.
- [x] `uv run --with pytest python -m pytest host/tests/test_nezha_drive.py
      host/tests/test_protocol_v2.py -q` passes without changes to those files.

## Implementation Plan

### Approach

Extend `RobotState` first (pure data; no logic change). Then modify `_apply_tlm`
to write `self.state` using frozen dataclass reconstruction (copy prior state
fields for any absent TLM fields). Then add the four property wrappers. The ABC
change is minimal — two new param defaults, one new stub.

### Files to Modify

- `host/robot_radio/robot/robot_state.py` — add `encoders`, `twist`, `line`,
  `color`, `world_pose` fields with `None` defaults; keep `v`, `omega`, `accel`,
  `pose`, `stamp`.
- `host/robot_radio/robot/nezha.py` — remove bare attribute assignments from
  `__init__`; add `self.state = _default_state()`; rewrite `_apply_tlm` to build
  a new frozen `RobotState`; add the four properties.
- `host/robot_radio/robot/robot.py` — add `on_tick=None`, `timeout_s=15.0` to
  `go_to`; add `turn` abstract stub.

### Files to Create

- `host/tests/test_robot_state.py`

### Testing Plan

New file `host/tests/test_robot_state.py`. Drive `Nezha` via `SimConnection`
(at `host/robot_radio/io/sim_conn.py`). No hardware needed.

1. `test_apply_tlm_populates_state` — construct a `TLMFrame` with all fields set;
   call `robot._apply_tlm(tlm)`; assert `state.encoders`, `state.pose`,
   `state.twist`, `state.line`, `state.color` match input.
2. `test_apply_tlm_partial_frame` — `TLMFrame` with only `enc` set; assert
   `state.encoders` updated while `state.line/color` retain their prior values.
3. `test_back_compat_properties` — after `_apply_tlm`, assert `robot.encoders ==
   state.encoders` and `robot.otos_pose == (state.pose.x, state.pose.y,
   state.pose.heading)`.
4. `test_state_stamp_recent` — assert `state.stamp` is within 1 second of
   `time.monotonic()`.

Verification: `uv run --with pytest python -m pytest host/tests/test_robot_state.py
host/tests/test_nezha_drive.py host/tests/test_protocol_v2.py -q`

### Documentation Updates

Update `RobotState` docstring to describe new fields and unit conventions.
Update `Nezha` class docstring to mention `robot.state`.
