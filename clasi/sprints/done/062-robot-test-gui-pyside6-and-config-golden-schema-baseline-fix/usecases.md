---
sprint: '062'
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Use Cases — Sprint 062

## SUC-001: Launch the Test GUI

**Actor:** Developer / operator
**Goal:** Start the interactive cockpit from the command line.

**Main flow:**
1. Operator runs `python -m robot_radio.testgui`.
2. A `QMainWindow` appears with: a transport selector, command-entry rows, an operations panel, a playfield canvas on the right, and a log pane at the bottom.
3. The window is usable before any transport is connected.

**Acceptance:** Application launches without error; all panels are visible; no hardware or simulator is required to open the window.

---

## SUC-002: Connect via Simulator (no hardware)

**Actor:** Developer
**Goal:** Exercise the full GUI against the physics simulator so traces diverge realistically without a robot.

**Main flow:**
1. Operator selects **Sim** in the transport selector.
2. The GUI checks whether `tests/_infra/sim/build/libfirmware_host.{dylib,so}` is present.
   - If missing or stale, a dialog prompts to run `python build.py`; connection is blocked until the lib is built.
3. On **Connect**, `SimTransport` loads the lib, starts a tick-thread, and issues `STREAM 50`.
4. Telemetry (TLM frames from `get_async_evts`) flows into the trace model; ground-truth positions come from `sim_get_true_pose_*`.

**Acceptance:** After Connect, the log pane shows `STREAM 50` being sent; telemetry appears; the robot marker is visible on the canvas at the origin.

---

## SUC-003: Connect via Serial (direct USB)

**Actor:** Developer / operator
**Goal:** Talk to a physically connected robot over USB serial.

**Main flow:**
1. Operator selects **Serial** in the transport selector and picks the port.
2. On **Connect**, `SerialTransport` wraps `SerialConnection(port, mode="direct")` and issues `STREAM 50`.
3. A reader thread parses incoming `TLM` lines via `parse_tlm`; a second thread polls the aprilcam daemon for camera ground truth.

**Acceptance:** Log shows STREAM command sent; TLM lines appear; camera-truth trace updates when the aprilcam daemon is available.

---

## SUC-004: Connect via Radio Relay

**Actor:** Developer / operator
**Goal:** Control the robot wirelessly through the relay dongle.

**Main flow:**
1. Operator selects **Relay** in the transport selector and picks the relay port.
2. On **Connect**, `RelayTransport` wraps `SerialConnection(port, mode="relay")`, which performs the `!GO` handshake and keepalive automatically.
3. TLM parsing and camera-truth polling are identical to SUC-003.

**Acceptance:** Connection completes without `!GO` handshake appearing in the operator-facing log (it is an internal detail of `SerialConnection`); TLM flows normally.

---

## SUC-005: Send a Motion Command via the Command Panel

**Actor:** Developer / operator
**Goal:** Issue any supported firmware motion command through labeled parameter fields.

**Supported commands:**

| Row label | Wire format |
|-----------|-------------|
| S (stream drive) | `S <left_mms> <right_mms>` |
| T (timed drive) | `T <left_mms> <right_mms> <ms>` |
| D (distance) | `D <left_mms> <right_mms> <mm>` |
| R (arc) | `R <speed> <radius_mm>` |
| TURN | `TURN <heading_cdeg> [eps=<cdeg>]` |
| G (go-to) | `G <x_mm> <y_mm> <speed>` |

**Main flow:**
1. Operator fills in the labeled `QLineEdit`/`QSpinBox` fields on a row.
2. Operator clicks **Send**.
3. The GUI assembles the wire string and calls `transport.command(line)`.
4. The sent string and the firmware reply appear timestamped in the log pane.

**Acceptance:** Each command row emits exactly the wire string documented above; the reply appears in the log; an error surfaces in the log if the transport returns an error (e.g., `OK vw busy`).

---

## SUC-006: Interactive Cursor-Key Driving

**Actor:** Developer / operator
**Goal:** Drive the robot with the keyboard while watching traces update in real time.

**Main flow:**
1. Transport is connected.
2. Operator presses and holds **Up** or **Down**: GUI sends `VW ±v 0` and a ~100 ms `QTimer` re-sends the command as a keepalive while the key is held.
3. Operator presses and holds **Left** or **Right**: GUI sends `VW 0 ±omega`.
4. On key release, GUI sends `STOP`.
5. If a queued TURN/G/T/D is active, the GUI detects `OK vw busy` in the response and surfaces it in the log without crashing.

**Acceptance:** Robot (or simulator) moves while key is held; stops promptly on release; no duplicate `STOP` spam; `vw busy` is logged clearly.

---

## SUC-007: Watch Four Pose Traces Diverge on the Playfield Canvas

**Actor:** Developer / operator
**Goal:** Visually observe how encoder odometry, OTOS odometry, and fused pose diverge from camera ground truth over a run.

**Main flow:**
1. Transport is connected; STREAM is active.
2. Operator drives or issues commands.
3. Four colored polylines update on the playfield canvas:
   - **Green** — camera/truth
   - **Orange** — encoder odometry
   - **Cyan** — OTOS odometry
   - **Magenta** — fused pose
4. A robot marker (red front half / blue back half) tracks the current fused pose.
5. Per-trace checkboxes let the operator toggle individual traces on/off.

**Acceptance (sim mode):** With `sim_set_motor_slip` applied, the encoder trace diverges from truth; fused and OTOS traces stay closer; the camera trace tracks `sim_get_true_pose_*` exactly.

---

## SUC-008: Sync Pose from Camera

**Actor:** Developer / operator
**Goal:** Re-anchor the firmware odometry to the current camera pose so subsequent traces start from a known position.

**Main flow:**
1. Operator clicks **Sync pose from camera** in the operations panel.
2. The GUI reuses the `cmd_sync_pose` / `_daemon_read_pose` logic from `host/robot_radio/io/cli.py:268`: reads the aprilcam daemon pose for tag 100, sends firmware `P <x> <y> <h_cdeg>`.
3. The log shows the pose values sent and the firmware `OK` reply.
4. All traces reset to the new anchored origin.

**Acceptance:** Firmware acknowledges the `P` command; trace model resets; subsequent traces start from the camera pose.

---

## SUC-009: Zero Encoders

**Actor:** Developer / operator
**Goal:** Zero the firmware encoder counters (not a full pose reset).

**Main flow:**
1. Operator clicks **Zero encoders**.
2. GUI sends the zero-encoder command.
3. Log shows the command and reply.

**Acceptance:** Encoder-based trace resets from the current position.

---

## SUC-010: Emergency Stop

**Actor:** Developer / operator
**Goal:** Immediately stop all robot motion.

**Main flow:**
1. Operator clicks **STOP** in the operations panel.
2. GUI sends `X`/`STOP`.
3. Robot halts; log shows the command.

**Acceptance:** Stop command appears in the log within one frame; robot halts.

---

## SUC-011: Clear Traces

**Actor:** Developer / operator
**Goal:** Reset the trace polylines to start a clean run without disconnecting.

**Main flow:**
1. Operator clicks **Clear traces**.
2. All four polylines are cleared; robot marker remains at the current fused pose.

**Acceptance:** Canvas clears immediately; no transport command is sent.

---

## SUC-012: Refresh Playfield Image from Camera

**Actor:** Developer / operator
**Goal:** Capture a fresh playfield background image to account for lighting or layout changes.

**Main flow:**
1. Operator clicks **Refresh playfield from cam 3**.
2. GUI captures a frame from the aprilcam daemon; replaces the `QPixmap` background on the canvas.

**Acceptance:** Canvas updates with the new image without restarting the transport.

---

## SUC-013: Toggle STREAM On/Off

**Actor:** Developer / operator
**Goal:** Start or stop the continuous telemetry stream.

**Main flow:**
1. Operator clicks **STREAM on/off** toggle.
2. When turning on: GUI sends `STREAM 50`; TLM starts flowing.
3. When turning off: GUI sends `STREAM 0`; TLM stops.

**Acceptance:** Log reflects the STREAM command; traces stop updating when stream is off.

---

## SUC-014: Switch Transport Without Restarting the Application

**Actor:** Developer
**Goal:** Disconnect one backend and connect another without relaunching the app.

**Main flow:**
1. Operator clicks **Disconnect**.
2. Current transport is torn down cleanly (threads joined, port closed).
3. Operator selects a different transport type and connects.

**Acceptance:** No dangling threads or port locks after disconnect; the new transport works normally.

---

## SUC-015: Run Headless CI Smoke Test (no display)

**Actor:** CI / developer
**Goal:** Validate GUI logic — trace accumulation, robot marker position, command wire strings — without a real display or transport.

**Main flow:**
1. `pytest tests/testgui/` runs with `QT_QPA_PLATFORM=offscreen`.
2. A fake Transport is injected; synthetic `TLMFrame` objects are pushed.
3. Tests assert trace polylines grow, the robot marker moves to the correct pixel, and each command row emits the right wire string.

**Acceptance:** All headless smoke tests pass in CI; `uv run python -m pytest tests/simulation` gate is not broken; no display or hardware is required.

---

## SUC-016: Schema accepts `tag_offset_mm.z` (config baseline fix)

**Actor:** Developer running the host test suite
**Goal:** The `test_tovez_validates_against_schema` test passes without modifying robot config files.

**Main flow:**
1. `robot_config.schema.json` `$defs.OffsetXYYaw` gains a `z: {type: number}` property.
2. `additionalProperties: false` is retained; the schema remains strict against other unknown keys.
3. `tovez.json`'s `vision.tag_offset_mm` carries `z: 120.0`; it now validates cleanly.
4. `uv run python -m pytest tests/simulation` passes this test.

**Acceptance:** `TestSchemaValidation::test_tovez_validates_against_schema` passes; no existing robot config files require changes.

---

## SUC-017: DefaultConfig golden matches SSOT values (stakeholder-confirmed)

**Actor:** Developer / stakeholder
**Goal:** The `test_default_robot_config_unchanged` pin test passes, with the golden snapshot reflecting confirmed-correct SSOT values for `odomOffY` and `yawRateMax`.

**Precondition:** Stakeholder has confirmed whether `yawRateMax=70` and `odomOffY=3.5` are the intended SSOT values (vs. the stale golden values of 35.0 and 4.0).

**Main flow:**
1. Stakeholder reviews the SSOT (schema defaults in `data/robots/robot_config.schema.json` or `source/robot/DefaultConfig.cpp` as generated by `scripts/gen_default_config.py`).
2. Stakeholder confirms the intended values.
3. Developer refreshes the golden file using the project's pin-update procedure.
4. `uv run python -m pytest tests/simulation` passes `test_default_robot_config_unchanged`.

**Acceptance:** Pin test passes; the golden file change is reviewed and confirmed by the stakeholder before commit; a commit message records the stakeholder decision.
