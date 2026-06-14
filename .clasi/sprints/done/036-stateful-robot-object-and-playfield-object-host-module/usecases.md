---
status: approved
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 036 Use Cases

## SUC-001: Query robot state after a drive tick

- **Actor**: Host program (or test harness)
- **Preconditions**: `Nezha` is constructed and connected; at least one TLM frame
  has arrived (via streaming drive or `refresh()`).
- **Main Flow**:
  1. The drive loop receives a TLM frame from the firmware.
  2. `_apply_tlm(tlm)` writes the new values into `robot.state`.
  3. Caller reads `robot.state.pose`, `robot.state.encoders`, `robot.state.twist`.
  4. Existing attribute reads (`robot.encoders`, `robot.otos_pose`, etc.) return the
     same values via thin property wrappers.
- **Postconditions**: `robot.state` reflects the most recent TLM frame; back-compat
  properties return identical values; no breaking change to Navigator/CLI callers.
- **Acceptance Criteria**:
  - [ ] `test_robot_state.py`: `_apply_tlm` populates `state.pose/encoders/twist/line/color`.
  - [ ] `robot.encoders`, `robot.otos_pose`, `robot.line_sensor`, `robot.color`
        return the same data as `robot.state` fields.
  - [ ] Existing `test_nezha_drive.py` passes without modification (regression guard).

## SUC-002: Query robot state when idle (one-shot SNAP)

- **Actor**: Host program between drive commands
- **Preconditions**: `Nezha` is constructed and connected; robot is idle (no active
  drive command).
- **Main Flow**:
  1. Caller calls `robot.refresh()`.
  2. `refresh()` sends `SNAP`; receives the TLM frame in the response.
  3. `_apply_tlm` updates `robot.state`; `refresh()` returns the new `RobotState`.
- **Postconditions**: `robot.state` is fresh; caller did not need to start streaming.
- **Acceptance Criteria**:
  - [ ] `test_robot_state.py`: `refresh()` issues SNAP and returns a populated
        `RobotState` without enabling continuous streaming.

## SUC-003: Drive to a target with a per-tick callback (G command)

- **Actor**: Host program (demo, MCP agent)
- **Preconditions**: `Nezha` connected; `on_tick` callable provided.
- **Main Flow**:
  1. Caller calls `robot.go_to(x_mm, y_mm, speed, on_tick=my_cb)`.
  2. `go_to` enables STREAM, issues `G`, then enters `_run_until_done`.
  3. Each TLM frame → `_apply_tlm` (state updated) → `my_cb(robot)`.
  4. `my_cb` may record `robot.state`, draw paths, check bounds.
  5. On `EVT done G` the loop returns `(enc_l, enc_r, "done")`.
- **Postconditions**: `robot.state` reflects final pose; `on_tick` was called once
  per telemetry tick; streaming is disabled on exit.
- **Acceptance Criteria**:
  - [ ] `test_robot_go_to_callback.py`: callback called per tick; state updated;
        loop exits on `EVT done G`; outcome is `"done"`.
  - [ ] `on_tick=None` (default) exercises the old blocking path with no change in
        return type (regression guard for Navigator callers).

## SUC-004: Abort a drive command from a callback

- **Actor**: Host program enforcing a safety boundary
- **Preconditions**: `go_to` or `turn` running with `on_tick` provided.
- **Main Flow**:
  1. Camera or odometry check inside `on_tick` determines robot is outside bounds.
  2. `on_tick` returns `False`.
  3. `_run_until_done` sends `X`, exits the tick loop, returns `"aborted"`.
- **Postconditions**: Robot has stopped; caller receives `"aborted"` outcome.
- **Acceptance Criteria**:
  - [ ] `test_robot_go_to_callback.py`: `on_tick` returning `False` sends `X` and
        produces outcome `"aborted"`.

## SUC-005: Turn to a heading with a per-tick callback

- **Actor**: Host program
- **Preconditions**: `Nezha` connected; `turn` method called with `on_tick`.
- **Main Flow**:
  1. Caller calls `robot.turn(heading_cdeg, on_tick=my_cb)`.
  2. Internally enables STREAM, issues `TURN`, enters `_run_until_done("TURN", ...)`.
  3. Each tick: state update + callback.
  4. On `EVT done TURN` returns `"done"`.
- **Postconditions**: Robot at target heading; state updated.
- **Acceptance Criteria**:
  - [ ] `test_robot_go_to_callback.py` or a dedicated test: `turn` with `on_tick`
        calls callback per tick and exits on `EVT done TURN`.
  - [ ] `turn(heading_cdeg)` with no callback blocks until done (backward-compat).

## SUC-006: Stream VW velocity and read state per tick

- **Actor**: Host program implementing open-ended body-velocity control
- **Preconditions**: `Nezha` connected.
- **Main Flow**:
  1. Caller iterates `for _ in robot.vw(v_mms, omega_mrads):`.
  2. Each iteration: VW is re-sent as keepalive; TLM frame arrives; state updated;
     loop body reads `robot.state`.
  3. Caller `break`s when done → `GeneratorExit` → STOP + STREAM 0 sent.
- **Postconditions**: Motor stopped; streaming disabled.
- **Acceptance Criteria**:
  - [ ] `test_robot_vw_generator.py`: `vw()` yields ticks; VW keepalive re-sent
        within watchdog window; `break` triggers STOP + STREAM 0 (GeneratorExit
        cleanup path).

## SUC-007: Push a camera pose fix into the robot

- **Actor**: Host program with a fresh camera tag read
- **Preconditions**: `Nezha` connected; camera has identified robot tag position.
- **Main Flow**:
  1. Caller has `(x_cm, y_cm, yaw_rad)` from `Playfield.get_tag(ROBOT_TAG)`.
  2. Caller calls `robot.update_world_pose(x_cm, y_cm, yaw_rad)`.
  3. Method converts to firmware units (mm, cdeg) and calls `set_world_pose`.
  4. Stores `(x_cm, y_cm, yaw_rad)` into `state.world_pose`.
- **Postconditions**: Firmware EKF is seeded with the camera fix; `state.world_pose`
  is updated.
- **Acceptance Criteria**:
  - [ ] `test_robot_state.py`: `update_world_pose(x_cm, y_cm, yaw_rad)` converts
        units correctly and calls `otos_set_position` with `(x_cm*10, y_cm*10,
        round(degrees(yaw_rad)*100))`.
  - [ ] `state.world_pose` reflects the passed values.

## SUC-008: Look up a live tag from the playfield

- **Actor**: Host program (demo, navigation agent)
- **Preconditions**: `Playfield` constructed; AprilCam daemon running; tag visible.
- **Main Flow**:
  1. Caller calls `field.get_tag(tag_id)`.
  2. `Playfield` calls `dc.get_tags(cam)`, finds the matching tag.
  3. Returns `Tag(id, x, y, yaw)` with x,y in cm, yaw in rad; `None` if absent.
- **Postconditions**: Caller has a fresh tag pose; no coupling to `Nezha`.
- **Acceptance Criteria**:
  - [ ] `test_playfield.py`: `get_tag` returns `Tag` with correct fields from a
        faked `TagRecord`; returns `None` for unknown id.

## SUC-009: Look up a static map feature

- **Actor**: Host program selecting a drive target
- **Preconditions**: `Playfield` constructed; `playfield.json` accessible.
- **Main Flow**:
  1. Caller calls `field.get_object("sq_A1")` (slug or name).
  2. `Playfield` resolves via `dc.where_is()` or direct `playfield.json` parse.
  3. Returns `Feature(slug, type, color, x, y)` or `None`.
- **Postconditions**: Caller has static feature coordinates (cm) to use as a drive
  target.
- **Acceptance Criteria**:
  - [ ] `test_playfield.py`: `get_object` returns `Feature` with correct fields
        from a fixture `playfield.json`.

## SUC-010: Convert world coordinates to pixel coordinates

- **Actor**: Host program drawing overlays or aligning camera frames
- **Preconditions**: `Playfield` constructed; homography available from daemon.
- **Main Flow**:
  1. Caller calls `field.world_to_pixel(x_cm, y_cm)`.
  2. `Playfield` applies the A1-centred y-up → raw corner-origin transform
     (`raw = [x+origin_x, origin_y-y, 1]`), then `pixel = inv(H) @ raw` (normalize).
  3. Returns `(u, v)` pixel coordinates, or `None` if no homography available.
- **Postconditions**: Pixel coordinates are consistent with the `display.py:411-447`
  convention.
- **Acceptance Criteria**:
  - [ ] `test_playfield.py`: `world_to_pixel` / `pixel_to_world` round-trip within
        floating-point tolerance for a known homography + origin. The y-flip and
        origin offset must be verified (not a naive `H` multiply).

## SUC-011: Draw a path on the live view

- **Actor**: Host program visualising robot trajectory
- **Preconditions**: `Playfield` constructed; daemon live view active.
- **Main Flow**:
  1. Caller accumulates world-frame waypoints from `robot.state` or `get_tag`.
  2. Calls `field.add_path("camera", points, symbol="filled_circle", color=...)`.
  3. `Playfield` writes `paths.json` atomically under the daemon data dir.
  4. Live view overlay refreshes on next frame.
- **Postconditions**: Path visible in live view; `paths.json` is valid JSON.
- **Acceptance Criteria**:
  - [ ] `test_playfield.py`: `add_path` writes a `paths.json` with the correct
        schema; `clear_paths` writes `[]`; file write is atomic (temp-rename).

## SUC-012: Port the playfield-tour demo to use Nezha and Playfield

- **Actor**: Developer / operator running the bench demo
- **Preconditions**: Robot on stand (safe to drive); AprilCam daemon with live view;
  `VER` confirms live firmware before trusting bench results.
- **Main Flow**:
  1. `make_robot()` constructs `Nezha` (relay mode, `!GO` data-plane).
  2. `Playfield.open()` opens the daemon connection.
  3. Per leg: `field.get_tag(ROBOT_TAG)` → `robot.update_world_pose(...)` →
     compute robot-relative target → `robot.go_to(..., on_tick=record_and_bounds_check)`.
  4. `on_tick` reads `field.get_tag`, updates camera/odometry tracks, calls
     `field.add_path`, returns `False` on bounds excursion.
  5. After all hops, demo exits cleanly; paths remain visible.
- **Postconditions**: Camera + odometry tracks drawn; robot reached each target
  (within `ARRIVE_CM`); any bounds excursion aborted cleanly.
- **Acceptance Criteria**:
  - [ ] Demo script `uv run --group calibrate python host_tests/playfield_tour/playfield_random_tour.py`
        runs end-to-end without raw serial, raw DaemonControl calls, or ad-hoc
        `!GO` / `send()` plumbing.
  - [ ] Bounds-abort (`on_tick` returning False) sends `X` and demo continues to
        next hop.
  - [ ] All raw `serial.Serial` usage is replaced by `make_robot()`.
