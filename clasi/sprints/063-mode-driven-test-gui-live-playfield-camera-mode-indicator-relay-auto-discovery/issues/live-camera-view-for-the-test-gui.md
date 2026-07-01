---
status: in-progress
sprint: '063'
tickets:
- 063-001
- 063-002
- 063-003
---

# Live Camera View for the Test GUI

## Problem / intent

The Test GUI's playfield canvas should reflect **what kind of run you're doing**, and
in a real playfield run it should show the **live camera** with the robot avatar over
the **real tag**, not a static image driven by fused telemetry.

The stakeholder wants three connected things:

### 1. A mode indicator near the top of the window
The transport you pick defines the mode; show it prominently near the top:
- **Simulation** transport → **SIM MODE**
- **Serial** transport → **BENCH MODE**
- **Relay** transport → **PLAYFIELD MODE**

### 2. The background follows the mode
- **SIM / BENCH mode** → show the **simulated / static playfield** (the current
  deskewed static image + avatar driven by telemetry, as today).
- **PLAYFIELD mode** (relay) → show the **live camera view**: a continuously-updated
  deskewed frame from the aprilcam daemon, with the red/blue avatar placed **directly
  over the real robot tag** (tag id 100) using its world position from the camera.
  This shows where the robot *really* is.

So the live view is **driven by mode**, not a manual toggle button. Entering playfield
mode (relay) starts the live camera background; leaving it restores the static playfield.

### 3. Relay auto-discovery — no manual port entry
When the user clicks Connect in **Relay** mode, the GUI should **find the relay itself**:
enumerate serial ports, open the candidates, read each device's announcement/banner
line, and identify the relay automatically. The user should not have to type a port.
(Serial/bench mode may keep manual port entry, or reuse discovery where sensible.)

## Implementation notes / guidance (for planning — not prescriptive)

- **Deskew is already correct.** `operations._deskew_bgr_with_tag_frame` now deskews
  from the daemon's `playfield_corners` onto the full output rectangle and returns an
  origin such that world (0,0)/tag 1 lands correctly (fixed in commit 69ade16). The live
  view should reuse this, refactored so the off-GUI-thread worker returns a **BGR ndarray**
  (Qt requires `QPixmap` to be built on the GUI thread).
- **Threading:** follow the existing telemetry-bridge pattern in `__main__.py`
  (`_TelemetryBridge`, `trigger_live_grab`): heavy daemon IO + `cv2` deskew off-thread,
  marshal results to the Qt main thread via a `QObject` signal (`QueuedConnection`), then
  `set_background(...)` + place the avatar. A ~10–15 Hz refresh loop is fine.
- **Avatar source in playfield mode:** the avatar should follow the camera tag
  (`tag.world_xy` + `heading_rad`/`yaw`, tag id 100 as in `robot.sync_pose.daemon_read_pose`),
  taking priority over fused telemetry. `CanvasController` will likely need a live-mode
  flag + a `set_avatar_pose(...)` method, and a `restore_static_background()` for mode exit.
- **Relay discovery:** relay transport lives in `testgui/transport.py`
  (`RelayTransport`, `SerialConnection(port, mode="relay")`) with `list_ports()` for
  enumeration. The relay/`!GO` data-plane protocol and banner behavior are documented in
  `.clasi/knowledge/` and at https://robots.jointheleague.org/ — consult before deriving
  banner parsing. Discovery must not disrupt non-relay devices it probes.
- **Camera-only:** the live view uses the aprilcam daemon, which is independent of the
  robot radio link. aprilcam + opencv are now in the host `gui` dependency group.

## Acceptance (behavioural)
- Selecting Sim/Serial/Relay shows SIM/BENCH/PLAYFIELD mode near the top.
- Relay Connect auto-finds the relay with no manual port entry (clear log on success/failure).
- In playfield mode the canvas shows the live deskewed camera and the avatar tracks the
  real robot tag; on the centre tag the avatar overlays tag 1.
- In sim/bench mode the canvas shows the simulated/static playfield as today.
- Headless tests cover the Qt-free pieces (mode→label mapping, deskew ndarray, avatar
  pose/gating, relay-discovery selection logic with a fake port list) and pass via
  `QT_QPA_PLATFORM=offscreen uv run python -m pytest tests/testgui -q`.
