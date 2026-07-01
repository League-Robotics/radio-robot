---
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 063 Use Cases

## SUC-001: View current operating mode in the GUI

- **Actor**: Developer or robot operator
- **Preconditions**: Test GUI is open; a transport type has been selected in the
  transport combo.
- **Main Flow**:
  1. User opens the Test GUI.
  2. A mode indicator label is visible near the top of the right panel.
  3. With "Sim" selected, the label reads **SIM MODE**.
  4. User changes the combo to "Serial"; the label updates immediately to **BENCH MODE**.
  5. User changes the combo to "Relay"; the label updates immediately to **PLAYFIELD MODE**.
- **Postconditions**: The label always reflects the current transport selection, even
  before a connection is established.
- **Acceptance Criteria**:
  - [ ] Mode indicator label is visible at all times, not hidden by any panel.
  - [ ] Label text is exactly "SIM MODE", "BENCH MODE", or "PLAYFIELD MODE" for Sim,
        Serial, and Relay respectively.
  - [ ] Label updates immediately on combo change (no connect required).
  - [ ] `transport_name_to_mode_label()` passes headless unit tests for all three values.

## SUC-002: Connect to the relay without typing a port

- **Actor**: Developer or robot operator
- **Preconditions**: A relay dongle is plugged in; "Relay" is selected in the transport
  combo; the user clicks Connect.
- **Main Flow**:
  1. User selects "Relay" in the transport combo.
  2. User clicks "Connect" (port field may be empty).
  3. The GUI enumerates available serial ports.
  4. Each candidate port is probed: opened briefly, banner line read.
  5. The port whose banner contains `RADIOBRIDGE` is identified as the relay.
  6. The GUI logs: "[INFO] Relay found on /dev/cu.usbmodemXXX".
  7. Connection proceeds normally using the discovered port.
- **Postconditions**: The relay is connected; the log shows which port was used.
- **Alternate Flow (no relay found)**:
  3a. No candidate port produces a `RADIOBRIDGE` banner.
  3b. GUI logs: "[WARN] No relay found on any serial port".
  3c. Connection is aborted; the user can try again after plugging in the relay.
- **Acceptance Criteria**:
  - [ ] `find_relay_port(port_list, probe_fn)` returns the correct port when the fake
        probe returns `RADIOBRIDGE` for one candidate.
  - [ ] `find_relay_port` returns `None` when no candidate matches.
  - [ ] `find_relay_port` is Qt-free and fully testable with a fake `probe_fn`.
  - [ ] Discovery does not hang or raise on ports that time out or raise I/O errors.
  - [ ] GUI log clearly states success (port name) or failure ("no relay found").

## SUC-003: See live camera view in PLAYFIELD MODE

- **Actor**: Developer or robot operator running a playfield trial
- **Preconditions**: Relay transport connected (PLAYFIELD MODE); aprilcam daemon is
  running with the playfield camera calibrated; robot with tag 100 is on the playfield.
- **Main Flow**:
  1. User connects via Relay. GUI enters PLAYFIELD MODE.
  2. Canvas background switches from the static/grey placeholder to a continuously-
     updated deskewed camera view of the actual playfield (~10–15 Hz).
  3. The red/blue avatar is positioned over the robot's real location as seen by the
     camera (tag id 100 world_xy + heading from the camera tag).
  4. As the robot moves, the avatar tracks its real position in the camera view.
  5. User disconnects. The canvas reverts to the static/placeholder background.
     The avatar returns to tracking fused telemetry.
- **Postconditions**: After disconnect, the GUI behaves exactly as in SIM/BENCH MODE.
- **Alternate Flow (daemon unavailable)**:
  2a. aprilcam daemon is not running.
  2b. GUI logs a warning; canvas stays on grey placeholder.
  2c. No crash; GUI remains usable for sending commands.
- **Alternate Flow (tag 100 not visible)**:
  3a. Robot is off the playfield or tag not detected.
  3b. Avatar stays at its last known position; background continues updating.
- **Acceptance Criteria**:
  - [ ] On relay connect, a live-view worker starts and begins emitting frames.
  - [ ] Canvas background updates at ~10–15 Hz in PLAYFIELD MODE.
  - [ ] Avatar in PLAYFIELD MODE follows camera tag 100 world_xy + heading (not fused
        TLM).
  - [ ] On relay disconnect, `restore_static_background()` is called and avatar reverts
        to fused telemetry path.
  - [ ] `_deskew_bgr_ndarray()` is a Qt-free helper testable with a fake TagFrame.
  - [ ] Live-view worker gracefully handles daemon unavailability (logs, no crash).
  - [ ] `CanvasController.set_avatar_pose(x_cm, y_cm, yaw_rad)` passes headless tests.
  - [ ] `CanvasController.restore_static_background()` passes headless tests.

## SUC-004: SIM/BENCH MODE background unchanged

- **Actor**: Developer using Sim or Serial transport
- **Preconditions**: GUI has Sim or Serial transport selected/connected.
- **Main Flow**:
  1. User connects via Sim or Serial.
  2. Canvas shows grey placeholder (or static playfield if TESTGUI_LOAD_STATIC_PLAYFIELD=1).
  3. Avatar follows fused telemetry as before.
  4. No live-view worker is started.
  5. "Refresh Playfield" button still works to do a one-shot camera grab.
- **Postconditions**: Existing behavior is fully preserved for Sim and Serial transports.
- **Acceptance Criteria**:
  - [ ] Sim transport: no live-view worker is started; background stays on placeholder.
  - [ ] Serial transport: no live-view worker is started; background stays on
        placeholder/static.
  - [ ] "Refresh Playfield" one-shot grab still works in Serial mode.
  - [ ] Existing `tests/testgui/` tests continue to pass unchanged.
