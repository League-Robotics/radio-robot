---
status: in-progress
sprint: '062'
tickets:
- 062-003
- 062-004
- 062-005
- 062-006
- 062-007
- 062-008
- 062-009
- 062-010
---

# Plan: Robot Test GUI (PySide6)

## Context

The project has excellent low-level tooling — a Python protocol client (`robot_radio`),
a ctypes-loaded firmware **simulator** with ground-truth + realistically-degraded sensors,
an aprilcam daemon for camera ground truth, and a pile of one-off bench scripts — but no
single interactive cockpit for driving the robot and *watching* how the four pose estimates
(camera-truth, encoders, OTOS odometry, fused) diverge in real time.

This builds that cockpit: a PySide6 desktop app that can talk to the robot over **serial**,
**radio relay**, or the **simulator**, send every motion command with labeled parameter fields,
drive interactively with the cursor keys, and render live traces on a playfield image exactly
like the AprilCam overlay view. In simulation mode it exercises the already-built
`PhysicsWorld` plant so the traces show realistic encoder slip and OTOS/IMU drift with no
hardware.

**Toolkit:** PySide6 (Qt). **Process:** built as CLASI **sprint 062** — I (team-lead) will
write the architecture + sequenced tickets and dispatch programmer agents. This file is the
design the sprint executes against.

## Where it lives

New module `host/robot_radio/testgui/` (sits beside the existing `host/robot_radio/testkit/`),
launchable as `python -m robot_radio.testgui` (add a console entry point). PySide6 added to
a new `gui` dependency group in `host/pyproject.toml`.

## Reuse (do NOT reinvent)

- `host/robot_radio/robot/protocol.py` — `NezhaProtocol`, `parse_tlm`/`TLMFrame`, `Stop` builders.
- `host/robot_radio/robot/nezha.py` — `Nezha.drive/timed/distance/arc/turn/go_to` convenience wrappers.
- `host/robot_radio/io/serial_conn.py` — `SerialConnection` (handles both `mode="direct"` and `mode="relay"` `!GO` handshake + keepalive).
- `host/robot_radio/io/cli.py:268` `cmd_sync_pose` + `_daemon_read_pose` — the exact "reset odometry to camera pose" logic (reads aprilcam pose → firmware `P` setpose).
- `host/robot_radio/testkit/camera.py` `read_camera_pose` and `tests/bench/ccw_square_50.py` — the four-trace accumulation + `tw()` body→world transform pattern.
- `tests/_infra/sim/firmware.py` `Sim` — ctypes simulator (`send_command`, `tick`/`tick_for`, `get_async_evts`, ground-truth accessors `sim_get_true_pose_*`, error-injection setters).
- Playfield image `tests/old/playfield_tour/playfield.jpg` + `playfield_calibration.json` (field 134 cm × 89.3 cm) for world-cm → pixel mapping.

## Architecture

### 1. Transport abstraction — `testgui/transport.py`
An ABC that unifies the three backends so the UI never branches on backend:

```
class Transport(ABC):
    def send(self, line: str) -> None            # fire-and-forget (drive/keepalive)
    def command(self, line: str, read_ms=200) -> str  # send + collect reply
    telemetry: signal/callback delivering parsed TLMFrame
    truth: callback delivering (x_cm, y_cm, yaw_rad) ground-truth pose (or None)
```

- **SerialTransport** / **RelayTransport** — thin wrappers over `SerialConnection(port, mode="direct"|"relay")` + `NezhaProtocol`. Issues `STREAM 50`; a reader thread parses TLM. Ground-truth pose comes from a second thread polling the **aprilcam daemon** (`read_camera_pose`, tag 100) — this is the real robot's true position.
- **SimTransport** — owns a `Sim`; a background thread advances `sim.tick()` at wall-clock rate, forwards queued command lines via `sim.send_command`, drains `get_async_evts()` for TLM/EVT lines (fed through the same `parse_tlm`), and reads `sim_get_true_pose_*` for the ground-truth trace. Sim's degraded encoders/OTOS are enabled via a field profile so traces diverge realistically.

### 2. Pose/trace model — `testgui/traces.py`
Accumulates four world-cm polylines: **camera/truth** (green), **encoder** (orange),
**OTOS odometry** (cyan), **fused** (magenta). Firmware `enc/otos/pose` arrive as mm deltas
anchored at start; transform to world via the initial truth pose (`tw()` pattern from
`ccw_square_50.py`). Each has an on/off flag driven by the canvas checkboxes.

### 3. Main window — `testgui/app.py` (QMainWindow, layout matching the sketch)
- **Top-left — command rows** (one `QWidget` row per command, built from a schema table):
  - `S  left[mm/s] right[mm/s]` → `S l r`
  - `T  left right ms` → `T l r ms`
  - `D  left right mm` → `D l r mm`
  - `R  speed radius[mm]` → `R speed radius`
  - `TURN heading[°] eps[°]` → `TURN h*100 [eps=..]`
  - `G  x[mm] y[mm] speed` → `G x y speed`
  Each row: label, labeled `QLineEdit`/`QSpinBox` per parameter, and a **Send** button.
- **Mid-left — operations** (`QPushButton`s): **Sync pose from camera** (reuse `cmd_sync_pose` → `P` command), **Zero encoders**, **STOP** (`X`/`STOP`), **Clear traces**, **Refresh playfield from cam 3**, **STREAM on/off**.
- **Right — playfield canvas** (`QGraphicsView`): background playfield `QPixmap`, four trace `QPainterPath`s, and a **robot marker** — a rectangle drawn at the current fused pose with a **red front half / blue back half** (red = forward). Trace on/off **checkboxes** beside it.
- **Bottom — log pane** (`QPlainTextEdit`, read-only): every line sent and every reply/TLM/EVT, timestamped.
- **Transport selector** (`QComboBox` + port picker): Sim / Serial / Relay; connect/disconnect.

### 4. Interactive driving — `testgui/drive.py`
`keyPressEvent`/`keyReleaseEvent` on the main window (ignore Qt auto-repeat):
- **Up/Down** held → forward/back: repeatedly `send("VW ±v 0")` on a ~100 ms `QTimer` (doubles as keepalive).
- **Left/Right** held → slow rotate: `send("VW 0 ±omega")`.
- On release → `send("STOP")`.
Guard: don't drive while a queued `TURN/G/T/D` is active (firmware replies `OK vw busy` — surface it in the log). Default speeds are configurable constants.

### 5. Sim build integration
Sim mode requires `tests/_infra/sim/build/libfirmware_host.{dylib,so}`. On selecting Sim, if
the lib is missing/stale, prompt to run `python build.py` (reuse the `build_lib` logic
`tests/_infra/sim/firmware.py` already uses in the pytest fixture) rather than failing opaquely.

## Sprint 062 ticket breakdown (sequenced)

1. `gui` dep group + PySide6, `testgui/` package skeleton, `__main__` entry point, empty QMainWindow + transport selector.
2. `transport.py` ABC + Serial/Relay wrappers (reuse `SerialConnection`); reader thread → TLM signal; log pane wiring.
3. `SimTransport` (ctypes tick-thread, `get_async_evts` TLM, ground-truth accessor) + sim-lib build check.
4. Command-entry rows (schema-driven) + Send buttons.
5. Operations panel (sync-pose/zero/STOP/clear/refresh/stream).
6. `traces.py` + playfield `QGraphicsView` canvas: image, 4 traces, robot red/blue marker, checkboxes, world-cm→px mapping.
7. Interactive cursor-key driving + keepalive timer.
8. README + a headless smoke test (construct widgets, feed synthetic TLM, assert traces update) under `tests/`.

## Verification

- **Unit/smoke (CI-safe, no hardware):** pytest that builds the app with a fake Transport, pushes synthetic `TLMFrame`s, and asserts trace paths and the robot marker update; validates each command row emits the correct wire string.
- **Sim end-to-end (no hardware):** launch `python -m robot_radio.testgui`, pick **Sim**, send `D 200 200 500`, confirm the fused/encoder/OTOS traces advance and diverge (slip/drift) while the truth trace stays clean; drive with cursor keys.
- **Hardware (bench, opt-in):** with the relay + aprilcam daemon up, pick **Relay**, confirm the camera-truth trace tracks the robot and **Sync pose from camera** re-anchors odometry; drive interactively.
- Run `uv run python -m pytest tests/simulation` to confirm nothing regressed.

## Open defaults (chosen, not blocking)
- Playfield defaults to the checked-in `playfield.jpg`; "Refresh from cam 3" re-grabs a frame.
- Traces render **in-GUI** (unified across sim/hardware); not mirrored to the external aprilcam overlay.
- Interactive drive uses `VW`; drive speeds are named constants at the top of `drive.py`.
