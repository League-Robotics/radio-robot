# Robot Test GUI

Interactive PySide6 cockpit for robot control and pose-trace visualization.
Connects to the robot via three transport backends (Sim, Serial, Relay), renders
four live pose traces on a playfield image, and exposes every firmware motion
command through labeled spin-box rows.

---

## Prerequisites

Install PySide6 into the project venv with the `gui` dependency group:

```
uv sync --group gui
```

This adds PySide6 to the `.venv` without affecting the default `dev` group or
the simulation gate.

---

## Launch

```
uv run python -m robot_radio.testgui
```

The main window opens.  Select a transport from the **Transport** drop-down,
fill in the port if required, and click **Connect**.

---

## Transport selection

### Sim — ctypes firmware simulator

Runs the firmware in-process via the ctypes sim library.  No hardware required.

**Before connecting**, build the sim library:

```
python build.py
```

Run `build.py` from the **repository root** (the directory that contains
`source/`, `host/`, and `tests/`).  The library is built into
`tests/_infra/sim/build/libfirmware_host.{dylib,so}`.

If the library is missing when you click Connect, a warning dialog appears with
the build command.

After connecting, the tick-thread advances the simulator at ~20 ms / step and
streams TLM automatically.  Ground-truth pose is delivered from the simulator
directly (no aprilcam daemon needed).

### Serial — direct USB connection

Wraps `SerialConnection(port, mode="direct")`.

1. Select **Serial** from the Transport drop-down.
2. Enter the serial port in the **Port** field
   (e.g. `/dev/cu.usbmodem21431202`).
   The first detected USB-modem port is pre-filled automatically.
3. Click **Connect**.  `STREAM 50` is sent automatically to start
   50 ms telemetry streaming.

### Relay — radio relay connection

Wraps `SerialConnection(port, mode="relay")`.  The relay handshake
(`!ECHO OFF` / `!MODE RAW250` / `!GO`) is performed automatically by
`SerialConnection.connect()`.

1. Select **Relay** from the Transport drop-down.
2. Enter the relay dongle's serial port in the **Port** field
   (e.g. `/dev/cu.usbmodem21421201`).
3. Click **Connect**.

For the **camera-truth trace** (green polyline) to appear with Serial or Relay,
the aprilcam daemon must be running and a camera must be open with tag 100
visible.  If the daemon is unavailable, the camera trace is simply absent; the
other three traces continue to update.

---

## Command rows

The left panel contains six schema-driven command rows, one per firmware motion
verb:

| Row  | Wire format                         | Field units             |
|------|-------------------------------------|-------------------------|
| S    | `S <left> <right>`                  | mm/s (speed)            |
| T    | `T <left> <right> <ms>`             | mm/s, ms                |
| D    | `D <left> <right> <mm>`             | mm/s, mm                |
| R    | `R <speed> <radius>`                | mm/s, mm                |
| TURN | `TURN <heading_cdeg>` or `TURN <h> eps=<e>` | centidegrees (1 cdeg = 0.01°) |
| G    | `G <x> <y> <speed>`                 | mm, mm, mm/s            |

Each row has a **Send** button (disabled until a transport connects).  Clicking
Send assembles the wire string from the current spin-box values and calls
`transport.command(line)`.

**TURN notes:**
- `heading` and `eps` are in **centidegrees** (90° = 9000 cdeg).
- When `eps` is 0 it is omitted from the wire string; non-zero `eps` appears
  as `eps=<cdeg>`.

---

## Tour buttons

Below the command rows, one button per named tour (`commands.TOURS`) runs a
pre-programmed motion sequence.  Clicking **Tour 1**:

1. Resets the robot to the origin (same as **Set Robot @ 0,0**: `ZERO enc`,
   `OZ`, `SI 0 0 0` + display reset).
2. On a background thread, sends each move in the tour one at a time, **waiting
   for the previous move to finish** before the next.

Completion is detected by polling `SNAP` and reading its `mode` field
(`mode=I` = idle) — the async `EVT done` event is *not* used because the radio
relay drops asynchronous events but answers `SNAP` reliably.  Each poll waits
up to 30 s per move before aborting the tour.

Tour 1 (heading 0 after the reset) traces: `RT 45°`, drive 420 mm, turn to
absolute heading 180°, drive 700 mm, then `RT 90°` with drives of 500 / 700 /
500 mm between the turns.  The sequence lives in `commands.TOUR_1` as plain
wire strings.

The tour is aborted (and the thread joined) on Disconnect and on app quit.

---

## Operations panel

Below the command rows, the **Operations** group provides six one-click actions:

| Button          | Action                                                  |
|-----------------|---------------------------------------------------------|
| Sync Pose       | Read tag-100 from aprilcam; send `SI x_mm y_mm h_cdeg` |
| Zero Encoders   | Send `ZERO enc`                                         |
| STOP            | Send `STOP` (hard motor stop)                          |
| Clear Traces    | Clear all four trace polylines                          |
| Refresh Playfield | Capture a new playfield image from camera 3            |
| STREAM: off/on  | Toggle `STREAM 50` / `STREAM 0`                        |

**Sync Pose** requires the aprilcam daemon to be running.  It is disabled in
Sim mode (ground-truth is delivered automatically by the sim tick-thread).

---

## Four traces

The right panel shows a QGraphicsView of the playfield with four colour-coded
polylines:

| Trace         | Colour  | Source                                        |
|---------------|---------|-----------------------------------------------|
| Camera (truth)| Green   | aprilcam daemon (hardware) or sim true pose   |
| Encoder       | Orange  | Wheel-encoder odometry integrated host-side   |
| OTOS          | Cyan    | Raw OTOS sensor pose from TLM                 |
| Fused         | Magenta | Firmware EKF fused pose from TLM              |

Checkboxes to the right of the canvas toggle each trace's visibility
independently.  All four traces continue to accumulate data regardless of
whether they are visible.

The **playfield image** and calibration data are loaded from
`tests/old/playfield_tour/playfield.jpg` and
`tests/old/playfield_tour/playfield_calibration.json` relative to the repo
root.  If the files are not found, the canvas falls back to a grey rectangle
at the default field size (134 × 89.3 cm).

---

## Robot marker

The current fused-pose position is shown as a rectangle split into a
**red front half** and a **blue back half**, rotated to the pose heading so
the red half always faces the robot's forward direction.  The marker appears
as soon as the first TLM frame with a `pose` field arrives.

---

## Interactive driving (cursor keys)

While a transport is connected, cursor-arrow keys drive the robot in real time:

| Key         | Command sent        |
|-------------|---------------------|
| Up arrow    | `VW 200 0`          |
| Down arrow  | `VW -200 0`         |
| Left arrow  | `VW 0 500`          |
| Right arrow | `VW 0 -500`         |
| Release     | `STOP`              |

A 100 ms QTimer resends the current `VW` command while a key is held, acting
as a firmware watchdog keepalive.  Qt key auto-repeat is suppressed; the timer
handles re-transmission.

---

## Log pane

Every sent command (`TX …`) and received line (`RX …`) appears in the
timestamped log pane at the bottom-right of the window.

---

## Running the headless CI tests

The `tests/testgui/` suite runs without a display server or hardware using the
offscreen Qt platform.  The `gui` dependency group must be installed first:

```
uv sync --group gui
QT_QPA_PLATFORM=offscreen uv run python -m pytest tests/testgui/ -v
```

These tests are **not** included in the default simulation gate
(`uv run python -m pytest tests/simulation`), which remains PySide6-free.

---

## Package importability without PySide6

`import robot_radio.testgui` succeeds without PySide6 installed.  All PySide6
imports inside the package are deferred to the functions and classes that
actually use them.  The `commands.py` and `traces.py` modules are fully
testable without a display server.
