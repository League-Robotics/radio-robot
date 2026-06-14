---
status: done
sprint: '036'
tickets:
- 036-005
- 036-006
- 036-007
---

# Plan: Stateful Robot object + Playfield object for the host module

## Context

The host-side Python module (`robot_radio`, at `host/robot_radio/`) currently
splits robot control across several loosely-coupled pieces: `Nezha` (command
sender with cached sensor attrs), `NezhaState`/`NezhaKinematic` (separate state
managers), and ad-hoc bench scripts that open a raw `serial.Serial` to the relay
and talk to the camera daemon directly. Driving is a mix of blocking calls
(`go_to` → `wait_for_evt_done`) and a `stream_drive` generator, with no single
queryable "robot state" and no uniform callback model.

We want the robot to be a single object that **owns its own state**, is driven
through **callbacks** for definite-end commands (G, turn) and **generators** for
open-ended commands (VW, S), and updates that state from telemetry on every tick.
Separately, a **Playfield** object should own all camera/world-geometry concerns:
looking up live tags and static map features, converting world↔pixel, exposing
the homography, and drawing paths/symbols on the live view.

Decisions locked with the stakeholder:
1. **Evolve `Nezha` in place** (not a new facade) — add the state model and the
   callback/generator drive methods directly to `Nezha`, keeping existing
   signatures backwards-compatible so current callers (Navigator, CLI, MCP,
   tests) keep working.
2. **Playfield wraps the AprilCam daemon** (`DaemonControl` gRPC) — keeps the
   daemon's live view + path drawing; tags are live, static features come from
   `playfield.json`/`where`, pixel↔world is computed from the daemon homography.
3. **State updates during commands only** — drive-loop ticks refresh
   `robot.state`; idle queries call `refresh()` which issues a one-shot `SNAP`.
   No background telemetry thread.
4. **Scope** = the two classes + unit tests + port the open demo
   (`host_tests/playfield_tour/playfield_random_tour.py`). Leave
   `navigator.py`/`odometry.py`/other bench scripts untouched.

The Robot is constructed only on a serial connection; the Playfield only on the
daemon. They never reference each other — the caller bridges them (read a tag
from the Playfield, push it into the Robot via `update_world_pose`).

---

## Part A — Evolve `Nezha` into the stateful Robot
File: `host/robot_radio/robot/nezha.py` (and a small state type, see below).

### A1. A single queryable state object
Add a `RobotState`-style snapshot exposed as `robot.state`, populated by the
existing `_apply_tlm(tlm)` hook (nezha.py:246). Reuse the frozen dataclass in
`host/robot_radio/robot/robot_state.py` if it fits, otherwise add a small mutable
`RobotState` carrying the telemetry already parsed in `TLMFrame`
(protocol.py:40): `pose` (x_mm, y_mm, heading_rad), `encoders` (l,r mm),
`twist` (v_mmps, omega_mradps), `line`, `color`, `world_pose` (set via
`update_world_pose`), and a `stamp` (`time.monotonic()`). Keep the current
`self.encoders/otos_pose/line_sensor/color` attributes as thin properties over
`state` so existing readers don't break. `_apply_tlm` becomes the one place that
writes state.

### A2. Definite-end commands take a callback (G, turn)
Convert `go_to` (nezha.py:190) to a tick-loop form and add a new `turn` method
(the wire op already exists at protocol.py:563, `Nezha` just doesn't expose it):

```python
def go_to(self, x_mm, y_mm, speed_mms, on_tick=None, timeout_s=15.0) -> tuple[int,int,str]
def turn(self, heading_cdeg, on_tick=None, eps_cdeg=None, timeout_s=10.0) -> str
```

- When `on_tick is None`: preserve today's behaviour (issue the command, then
  `wait_for_evt_done`) so existing callers (Navigator, tests) are unaffected.
- When `on_tick` is given: enable `STREAM`, issue the command, then run a private
  `_run_until_done(verb, on_tick, timeout)` loop. Each iteration is one **tick**:
  - drain `read_lines`; for each line: a `TLM` → `_apply_tlm` (update state) then
    `on_tick(self)`; an `EVT done <verb>`/`safety_stop` → return outcome.
  - if no telemetry arrived and the keepalive interval elapsed → send a keepalive
    and `on_tick(self)`. (Note: G/turn have a firmware stop-net and the
    `SerialConnection` keepalive daemon already sends `+`; the explicit keepalive
    here only matters when the daemon is off — keep it cheap.)
  - **`on_tick` may return `False` to abort** → send `X`, return outcome
    `"aborted"`. This is what the demo's camera bounds-abort needs.
- Outcome strings stay `"done" | "safety_stop" | "timeout" | "aborted"`.

Reuse the existing reader/keepalive primitives in `protocol.py` (`read_lines`,
`send_fast`, `parse_response`, `wait_for_evt_done`); do **not** touch
`serial_conn.py`.

### A3. Open-ended commands are generators (VW, S)
- Add `vw(v_mms, omega_mrads, *, period_ms=40)` as a **generator** that yields
  each tick (after `_apply_tlm`), re-sending `VW` as its own keepalive within the
  watchdog window — model it on the existing `stream_drive` generator
  (protocol.py:846) which already does exactly this for `S`. Breaking out of the
  caller's `for` loop triggers `GeneratorExit` → send `STOP` + `STREAM 0`.
- Keep the existing `speed()` generator (nezha.py:130) as the S-streaming
  generator (rename/alias to `stream(left,right)` for symmetry if desired).
- Both update `robot.state` every tick, same as the callback path.

### A4. Idle state refresh + world-pose push
- `refresh() -> RobotState`: issue `SNAP` (protocol.py:676), `_apply_tlm` the
  frame, return `state`. This is how you query fresh state when not driving.
- `update_world_pose(x_cm, y_cm, yaw_rad)`: convert camera-native units to
  firmware units (`mm = cm*10`, `cdeg = round(degrees(yaw)*100)`) and call the
  SI/OV path (`set_world_pose`, nezha.py:313). Store into `state.world_pose`.
  This is the "push the tag's pose directly into the robot" method.

### A5. Keep the `Robot` ABC consistent
`host/robot_radio/robot/robot.py` defines the interface. Add the new optional
params/methods with safe defaults (or leave the new methods Nezha-specific) so
`Cutebot` still satisfies the ABC. No behavioural change to `Cutebot`.

---

## Part B — `Playfield` object (daemon wrapper)
New subpackage: `host/robot_radio/field/__init__.py` + `playfield.py`.
(Namespaced, so the name does not collide with `aprilcam.Playfield`.)

Construct on the daemon (the stakeholder's revised model — built on
`DaemonControl`, the same client the open demo uses):

```python
class Playfield:
    @classmethod
    def open(cls, cam_name=None) -> "Playfield"   # DaemonControl.connect_default(Config.load())
    def get_tag(self, tag_id) -> Tag | None        # live, from dc.get_tag/get_tags
    def tags(self) -> dict[int, Tag]               # all live tags
    def get_object(self, slug_or_query) -> Feature | None   # static map feature
    def world_to_pixel(self, x_cm, y_cm) -> tuple[float,float] | None
    def pixel_to_world(self, u, v) -> tuple[float,float] | None
    @property
    def homography(self) -> "np.ndarray"
    def add_path(self, path_id, points, *, symbol="filled_circle", color=(0,200,255), size_cm=1.2)
    def add_symbol(self, x_cm, y_cm, *, symbol="x", color=..., size_cm=1.0)
    def clear_paths(self)
    def close(self)
```

- **Tag** = small dataclass `Tag(id, x, y, yaw)` (x,y in cm, yaw in rad) mapped
  from the daemon `TagRecord` (`world_xy`, `orientation_yaw`; reuse the
  `aprilcam.Tag` `.wx/.wy/.orientation` convention already trusted in
  `odometry.py:175`). Return `None` when the tag is absent/stale.
- **Feature** (static) = `Feature(slug, type, color, x, y)` from `playfield.json`
  categories (`april_tags, aruco_tags, rectangles, dots`). Resolve via
  `dc.where_is(query, cam)` (returns matches with x,y) or by loading
  `playfield.json` directly (path from the daemon data dir, as the demo does at
  playfield_random_tour.py:42). `get_tag` = **live detection**; `get_object` =
  **static map**.
- **pixel↔world** — IMPORTANT convention (verified in
  `aprilcam/ui/display.py:411-447`): the daemon homography `H` maps
  pixel ↔ **raw corner-origin** cm, while tag world coords are **A1-centred,
  y-up**. So:
  - `world_to_pixel(x,y)`: `raw = [x+origin_x, origin_y-y, 1]`; `pixel = inv(H)@raw`.
  - `pixel_to_world(u,v)`: `raw = H@[u,v,1]` (normalize); `x = raw_x-origin_x`,
    `y = origin_y-raw_y`.
  - `origin_x/​origin_y` come from `playfield.json` `playfield` block
    (width_cm/height_cm, `origin`). Source `H` from the latest
    `TagFrame.homography` (a flat 9-float list → 3×3) cached on each tag read.
    Replicate display.py exactly; do not naively apply `H`.
- **Paths/symbols** — write the persistent `paths.json` under
  `<daemon-data>/cameras/<cam>/paths.json` using the schema the demo already uses
  (playfield_random_tour.py:119-139: `[{path_id, playfield_id, waypoints:[{x,y,
  size_cm,symbol,symbol_color,line_color}]}]`, atomic temp-rename). Optionally
  also expose live `dc.publish_overlay` for transient drawing (per
  `aprilcam/ROBOT_API_GUIDE.md:121`). `clear_paths` writes `[]`.

---

## Part C — Port the demo to the new API
File: `host_tests/playfield_tour/playfield_random_tour.py` (rewrite).

Replace the raw `serial.Serial(RELAY,...)` + `!GO` + manual `send()` plumbing
with the evolved `Nezha` (via `robot_radio.robot.connection.make_robot(...)`,
which already handles relay mode, the `!GO` data-plane, the reader thread and the
keepalive daemon), and replace the direct `DaemonControl` calls with `Playfield`.

Per leg the new flow becomes:
1. `loc = field.get_tag(ROBOT_TAG)` → robot world pose (cm, rad).
2. `robot.update_world_pose(loc.x, loc.y, loc.yaw)` (replaces the raw `SI`).
3. compute robot-relative `(fwd_mm, left_mm)` to the target rectangle
   (`field.get_object(slug)` gives the static target).
4. `robot.go_to(fwd, left, SPEED, on_tick=record_and_bounds_check)` — one smooth
   firmware G; the callback records `field.get_tag` (camera track) and
   `robot.state.pose` (odometry track) each tick, draws via `field.add_path`, and
   returns `False` to bounds-abort. This exercises: G-callback + state + Playfield
   tag/object + add_path + abort, end-to-end.

This is the working proof. Keep `SET sTimeout/turnGate/alphaYaw/yawRateMax` as
`robot.set_config(...)` and `STREAM` handled inside `go_to`.

---

## Part D — Unit tests
Dir: `host/tests/` (alongside `test_nezha_drive.py`, `test_protocol_v2.py`).

Drive the evolved `Nezha` against `robot_radio.io.sim_conn.SimConnection` (the
existing in-process firmware sim) — no hardware needed. Cover:
- `test_robot_state.py`: `_apply_tlm` populates `state.pose/encoders/twist/...`;
  `refresh()` issues SNAP and returns fresh state; back-compat properties
  (`robot.encoders` etc.) still read.
- `test_robot_go_to_callback.py`: `go_to(..., on_tick)` calls the callback per
  tick, updates state, terminates on `EVT done G`; `on_tick` returning `False`
  aborts with `"aborted"` and sends `X`; `on_tick=None` keeps the old blocking
  behaviour (regression guard for Navigator).
- `test_robot_vw_generator.py`: `vw(...)` yields ticks, re-sends VW keepalive,
  stops cleanly on `break` (GeneratorExit → STOP + STREAM 0).
- `test_playfield.py`: `world_to_pixel`/`pixel_to_world` round-trip against a
  known homography + origin (assert the display.py convention, incl. y-flip);
  `get_tag`/`get_object` mapping from faked `TagRecord`/`playfield.json`;
  `add_path`/`clear_paths` write the expected `paths.json`. Mock `DaemonControl`
  so the test needs no live daemon/camera.

Run: `uv run --with pytest python -m pytest host/tests/ -q`
(per project memory: bare `uv run pytest` fails on missing serial).

---

## Files to create / modify
- **Modify** `host/robot_radio/robot/nezha.py` — state object, `go_to` callback
  form, new `turn`, `vw` generator, `refresh`, `update_world_pose`,
  back-compat properties.
- **Modify** `host/robot_radio/robot/robot_state.py` — extend/confirm the state
  dataclass (or add a new lightweight one).
- **Modify** `host/robot_radio/robot/robot.py` — keep ABC consistent (optional
  params/defaults).
- **Create** `host/robot_radio/field/__init__.py`, `host/robot_radio/field/playfield.py`.
- **Modify** `host/robot_radio/__init__.py` / `robot/__init__.py` — export the
  Playfield and any new state type.
- **Rewrite** `host_tests/playfield_tour/playfield_random_tour.py`.
- **Create** the four test files under `host/tests/`.

## Reuse (do not reimplement)
- `NezhaProtocol.stream_drive` / `wait_for_evt_done` / `snap` / `read_lines` /
  `parse_tlm` — `host/robot_radio/robot/protocol.py`.
- `SerialConnection` reader thread + `+` keepalive daemon — `io/serial_conn.py`
  (untouched).
- `make_robot` for relay detection + `!GO` + construction — `robot/connection.py`.
- `SimConnection` for tests — `io/sim_conn.py`.
- Daemon client + `where_is` + `publish_overlay` — `aprilcam.client.control.DaemonControl`.
- pixel↔world convention — mirror `aprilcam/ui/display.py:411-447`.

## Defaults chosen (flag if you disagree)
- Drive methods keep firmware-native units (mm, mm/s, milli-rad/s, cdeg) for
  back-compat; only `update_world_pose` takes camera-native cm/rad.
- `on_tick` callback signature is `on_tick(robot)` and may return `False` to abort.
- `get_tag` = live detection, `get_object` = static map feature. (Live colored-
  object detection via `ObjectRecord`/`SquareDetector` is a later add-on, not in
  this pass.)

## Verification
1. `uv run --with pytest python -m pytest host/tests/ -q` — all new + existing
   host tests green (regression guard for Navigator/CLI paths).
2. `uv run --with pytest python -m pytest host/tests/test_protocol_v2.py host/tests/test_nezha_drive.py -q`
   — confirm no break to the wire layer.
3. Bench smoke (robot on the stand, safe to drive — per project memory): run the
   ported demo `uv run --group calibrate python host_tests/playfield_tour/playfield_random_tour.py`
   with the daemon live view open; confirm the camera + odometry tracks draw, the
   robot reaches each rectangle, and a bounds-excursion aborts cleanly (callback
   returns False → `X`). Verify `VER` shows the live firmware first.
4. Interactive sanity: construct `Nezha` via `make_robot`, call `refresh()` and
   print `robot.state`; open `Playfield`, `get_tag(100)`, round-trip
   `world_to_pixel`/`pixel_to_world`, `add_path`/`clear_paths`.
