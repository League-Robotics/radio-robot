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

## SUC-005: Reset robot pose to origin from the GUI

- **Actor**: Developer or robot operator
- **Preconditions**: Robot is on the playfield (or bench stand); Test GUI is open;
  any transport may be selected.
- **Main Flow**:
  1. Operator physically places the robot at the playfield origin (0, 0).
  2. Operator clicks "Set Robot @ 0,0" in the operations panel.
  3. GUI sends `ZERO enc` to reset wheel encoder counters to zero.
  4. GUI sends `SI 0 0 0` to update the firmware's internal pose estimate to
     (x=0 mm, y=0 mm, heading=0 centidegrees).
  5. GUI resets the canvas display: avatar moves to field centre, heading 0;
     trace polylines are cleared; trace model is re-anchored at (0, 0, 0).
  6. Log shows the two commands sent.
- **Postconditions**: Robot firmware reports pose (0, 0, 0); GUI displays the
  robot at the field centre; subsequent telemetry-driven motion starts from the
  correct origin.
- **Alternate Flow (no transport connected)**:
  2a. `_state["transport"]` is `None`.
  2b. Wire commands are skipped; GUI logs "[WARN] Set Robot @ 0,0: no robot
      connected — display only".
  2c. Display reset still runs (avatar to centre, traces cleared).
- **Acceptance Criteria**:
  - [ ] `ZERO enc` is sent before `SI 0 0 0` when a transport is connected.
  - [ ] In Sim mode both commands are sent.
  - [ ] No transport connected: warning logged, display reset still runs.
  - [ ] Headless tests verify the command sequence with a fake transport.

## SUC-006: Record a Test GUI session to a file

- **Actor**: Developer running a playfield or bench trial
- **Preconditions**: Test GUI is open; Record, Pause, Stop controls are visible.
- **Main Flow**:
  1. User clicks **Record**. A timestamped `recordings/<timestamp>.jsonl` file
     is created; appending begins immediately.
  2. Every TX line (command sent to robot) and every RX line (response /
     telemetry) is written to the file with monotonic + wall-clock timestamps
     and a direction tag (`TX` or `RX`).
  3. User clicks **Pause**. Appending suspends; file remains open.
  4. User clicks **Record** (Resume). Appending resumes into the same file.
  5. User clicks **Stop**. File is finalized and closed; log shows the saved
     path. Controls return to idle state.
- **Postconditions**: The JSONL file on disk contains a complete, ordered record
  of all TX/RX lines during the non-paused recording window.
- **Alternate Flow (stop without recording)**:
  - Record was never clicked or Stop was already clicked; clicking Stop again is
    a no-op.
- **Acceptance Criteria**:
  - [ ] Record / Pause / Stop buttons exist with correct enable/disable states.
  - [ ] After Record, both TX and RX lines appear in the file with timestamps.
  - [ ] Pausing drops all entries until Resume; no gap in file (file stays open).
  - [ ] Stop writes and closes the file; log shows the path.
  - [ ] Works across Sim, Serial, and Relay transports.
  - [ ] `SessionRecorder` is Qt-free; headless tests cover append, pause gating,
        and JSONL serialization.

## SUC-007: Test the OTOS heading-reset bug in Sim mode

- **Actor**: Developer validating the EKF/OTOS fusion path or the "Set Robot @ 0,0"
  heading reset (SUC-005) without hardware.
- **Preconditions**: Test GUI (or a `tests/simulation` test) is connected via the Sim
  transport.
- **Main Flow**:
  1. Developer drives the sim robot to a non-zero heading (e.g. `VW 0 300` then `S`).
  2. Developer sends `SI 0 0 0` alone (no `OZ`).
  3. Over the next several ticks, the fused heading drifts back toward the sim OTOS's
     retained absolute heading — reproducing the exact hardware bug documented in
     `.clasi/knowledge/2026-07-01-heading-reset-needs-oz-not-just-si.md`.
  4. Developer repeats from a non-zero heading, this time sending `ZERO enc`, then
     `OZ`, then `SI 0 0 0` (the SUC-005 sequence).
  5. The fused heading resets to 0 and **holds** at 0 across many subsequent ticks
     (no drift-back), because `OZ` re-referenced the sim OTOS's absolute heading.
  6. `OZ`, `OI`, `OR`, `OV` all reply `OK` in Sim mode (never `ERR nodev`).
- **Postconditions**: Sim mode reproduces hardware OTOS-fusion behaviour closely enough
  that the heading-reset bug and its fix are both observable and regression-tested
  without a physical robot.
- **Acceptance Criteria**:
  - [ ] In Sim mode, `OZ`/`OI`/`OR`/`OV` never return `ERR nodev`.
  - [ ] `SI 0 0 0` alone (no `OZ`) leaves the fused heading drifting back toward the
        sim OTOS's retained heading within a few ticks.
  - [ ] `ZERO enc` + `OZ` + `SI 0 0 0` resets the fused heading to 0 and holds it there.
  - [ ] Test GUI Sim mode: clicking "Set Robot @ 0,0" after a turn drives the on-screen
        avatar heading to 0 and it stays there (no visible drift-back).
  - [ ] `tests/simulation` regression test covers both the bug-reproduction and the
        fix-verification cases described above.
  - [ ] Existing golden-TLM / OTOS-fusion sim tests (`test_golden_tlm.py`,
        `test_ekf_dual_source.py`, `test_dbg_otos_commands.py`, `test_ekf.py`) pass
        unchanged.

## SUC-008: Stop a running Tour and have controls reactivate

- **Actor**: Developer or robot operator running a Tour in PLAYFIELD or BENCH mode
- **Preconditions**: A transport is connected; a Tour is currently running (Tour 1
  button was clicked and has not yet finished).
- **Main Flow**:
  1. Operator clicks the dedicated **Stop Tour** button (or the shared STOP button).
  2. The tour worker is signalled to stop and its thread is joined synchronously.
  3. The Tour 1 button and the Stop Tour button both return to their idle
     enable/disable state **immediately** (Tour 1 enabled, Stop Tour disabled) —
     without waiting for, or depending on, the worker's `finished` signal.
  4. Operator can immediately click Tour 1 again to start a fresh run.
- **Postconditions**: Tour buttons are never left permanently disabled after a stop.
  The same reactivation guarantee applies to the GOTO button after a GOTO stop.
- **Alternate Flow (tour finishes naturally, no explicit stop)**:
  3a. The tour completes all steps; `_on_tour_finished` still re-enables the
      buttons via the worker's `finished` signal (this path is unaffected by
      the fix).
- **Acceptance Criteria**:
  - [ ] A visible "Stop Tour" control exists, disabled when no tour is running,
        enabled while a tour runs.
  - [ ] Clicking Stop Tour (or shared STOP) re-enables the Tour 1 button
        synchronously inside `_stop_tour()`, not only via `_on_tour_finished`.
  - [ ] The same synchronous re-enable is applied to `_stop_goto()` for the
        GOTO button.
  - [ ] A tour that finishes naturally (not stopped) still re-enables buttons
        via `_on_tour_finished` — no regression.
  - [ ] Headless test reproduces the bug scenario (stop while "running") and
        asserts buttons re-enable without depending on a `finished` signal
        delivery.

## SUC-009: Tour is gated to a connected transport and warns in Sim mode

- **Actor**: Developer or robot operator
- **Preconditions**: Test GUI is open.
- **Main Flow**:
  1. Before Connect, all tour buttons are disabled (cannot be clicked).
  2. Operator selects "Sim" and clicks Connect. Tour buttons enable (Sim is a
     valid dry-run target).
  3. Operator clicks Tour 1. The log shows a clear
     `[TOUR] running in SIM mode` line before the tour's own step-by-step log
     lines.
  4. The tour runs against the simulator exactly as it would against real
     hardware (dry run).
- **Postconditions**: Tours remain runnable in Sim mode; the operator is never
  confused about whether the tour targeted real hardware, because the log says so.
- **Acceptance Criteria**:
  - [ ] Tour buttons are disabled before Connect (regression test — extends the
        existing `test_tour_button_present_and_disabled` coverage).
  - [ ] Starting a tour with a `SimTransport` logs `[TOUR] running in SIM mode`.
  - [ ] Starting a tour with a hardware transport (Serial/Relay) does NOT log
        that line.
  - [ ] Tours are NOT blocked in Sim mode (stakeholder decision: Sim remains a
        valid dry-run target).

## SUC-010: Camera selection is consistent and user-configurable

- **Actor**: Developer operating the playfield with more than one camera
  attached to the aprilcam daemon
- **Preconditions**: aprilcam daemon is running with one or more cameras open;
  Test GUI is open.
- **Main Flow**:
  1. A camera pull-down (combo box) in the GUI lists the cameras the aprilcam
     daemon knows about, defaulting to the persisted selection from a prior
     session (falling back to camera index 3, "Arducam OV9782 USB Camera",
     if nothing is persisted or the persisted camera is no longer available).
  2. Operator picks a different camera from the pull-down.
  3. The choice is persisted (survives a GUI restart).
  4. A fresh one-shot playfield grab runs immediately, updating the canvas
     background from the newly-selected camera.
  5. All three camera-consuming code paths — one-shot grab
     (`_capture_playfield_frame_and_calib`), the live-view worker
     (`live_view.py::_capture_and_emit`), and pose read (`_read_daemon_pose`) —
     use the same selected camera from that point on.
- **Postconditions**: No code path silently reads a different (possibly
  uncalibrated) camera than the one shown in the GUI.
- **Acceptance Criteria**:
  - [ ] A `QComboBox` lists cameras known to the aprilcam daemon.
  - [ ] The selected camera is persisted to a small config file and reloaded on
        next GUI start.
  - [ ] Default falls back to camera index 3 ("Arducam OV9782 USB Camera") when
        no persisted/available selection matches.
  - [ ] Changing the pull-down selection triggers an immediate one-shot grab
        using the newly selected camera.
  - [ ] `_capture_playfield_frame_and_calib`, `_read_daemon_pose`, and
        `live_view.py::_capture_and_emit` all resolve the camera through the
        same shared selection helper (no more `cams[0]` or
        name-contains-digit heuristics duplicated three ways).
  - [ ] Headless tests cover the selection/persistence helper and the
        replacement of all three `cams[0]` call sites, with the aprilcam
        daemon mocked/faked (no hardware in CI).

## SUC-011: Live playfield background actually updates while connected via Relay

- **Actor**: Developer or robot operator running a playfield trial
- **Preconditions**: Relay transport connected (PLAYFIELD MODE); aprilcam daemon
  running with the playfield camera calibrated.
- **Main Flow**:
  1. User connects via Relay. The live-view worker starts (as delivered by
     ticket 003) and begins looping at ~9–12 Hz.
  2. Unlike the as-shipped ticket 003 behavior — where `frame_ready` is
     connected directly to a bare function `_on_live_frame` with
     `Qt.ConnectionType.QueuedConnection`, which this PySide build delivers on
     the **worker** thread (never processed, because the worker thread never
     re-enters its event loop) — frames are routed through a main-thread
     `QObject` bridge (matching the existing `_RXBridge` / `_WorkerBridge`
     pattern), so they are actually delivered and painted.
  3. The canvas background image visibly updates at a throttled ~3–4 fps (a
     new frame is converted to `QPixmap` and set roughly every 3rd worker tick).
  4. The avatar (red/blue marker) updates at the full worker rate (~9–12 Hz),
     independent of the background throttle — it does not visibly lag behind
     robot motion even though the image behind it refreshes more slowly.
  5. User disconnects; canvas reverts exactly as in SUC-003.
- **Postconditions**: The operator can watch the robot move on the live camera
  image in near real time, exactly as it worked in the earlier camera-work
  milestone that this ticket restores.
- **Acceptance Criteria**:
  - [ ] `frame_ready` is connected to a main-thread `QObject` bridge slot (not a
        bare function) so delivery happens on the GUI thread regardless of
        PySide connection-type quirks.
  - [ ] The avatar pose (`canvas_ctrl.set_avatar_pose`) is updated on every
        received frame (full worker rate, ~9–12 Hz).
  - [ ] The background `QPixmap` conversion + `canvas_ctrl.set_background`
        call is throttled to ~3–4 fps (every Nth frame, or time-gated).
  - [ ] A lower background update rate than 3–4 fps is acceptable; avatar
        smoothness is the priority.
  - [ ] Headless test simulates N `frame_ready` emissions and asserts
        `set_avatar_pose` is called N times while `set_background` is called
        a throttled subset of N times.
  - [ ] No regression to SUC-003's disconnect/restore-static-background
        behavior.

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
