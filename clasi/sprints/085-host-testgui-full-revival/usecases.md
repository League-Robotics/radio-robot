---
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 085 Use Cases

Parent issue: [`clasi/issues/host-testgui-full-revival.md`](../../issues/host-testgui-full-revival.md).
Completes the program epic
[`clasi/issues/plan-revive-testgui-against-the-new-tree-simulator.md`](../../issues/plan-revive-testgui-against-the-new-tree-simulator.md).

These use cases scope the final TestGUI-revival sprint: reconnecting tours,
camera GOTO, the Operations panel, connect-time calibration push, sim-error
injection, live camera view, and device selection onto sprint 084's motion/
config/pose-set/OTOS verb surface, and porting the historical test coverage
for all of it. A pre-ticketing code read (recorded in `architecture-update.md`'s
Grounding section) found that most of this reconnection was never actually
broken host-side code — `host/robot_radio/testgui/__main__.py`,
`operations.py`, and `calibration/push.py` already send the exact wire verbs
084 now implements (they predate the greenfield rebuild and were written
against the old firmware's identical wire shapes). This sprint's job is
therefore mostly verification, gap-filling, and test-porting rather than net-new
host implementation — see the architecture document for the one exception
(camera GOTO has no historical test at all) and the two concrete wire-range
bugs found in `commands.py`.

## SUC-001: Operator runs a pre-programmed tour to completion

- **Actor**: Stakeholder / developer running TestGUI against the sim (and,
  where available, the bench/relay).
- **Preconditions**: Connected (Sim, Serial, or Relay); no tour or GOTO
  already running.
- **Main Flow**:
  1. Operator clicks "Tour 1" or "Tour 2".
  2. The GUI resets the robot to world origin (`_set_origin`), then
     `_TourRunner` sends each tour step (`D`/`RT` wire strings from
     `commands.TOURS`) in order via `transport.command()`.
  3. After each step, the runner polls completion via a fire-and-forget
     `SNAP` and reads `mode=I` from the cached `TLMFrame` (not
     `command("SNAP")`, whose corr-id-less reply never reaches the reply
     queue) before advancing to the next step.
  4. On the last step's completion, the runner logs "complete" and the
     robot is near world origin again (the tour is a closed loop).
- **Postconditions**: Tour buttons re-enable; the robot's fused pose is
  close to (0, 0, 0) within the tour's own geometric tolerance.
- **Acceptance Criteria**:
  - [ ] Tour 1 and Tour 2 both run to completion against the sim with no
        step timing out.
  - [ ] Stopping a tour mid-run re-enables the buttons synchronously (no
        dependence on a signal delivered during a blocking `thread.wait()`).
  - [ ] `_wait_for_idle` rejects a stale pre-move idle frame (timestamped
        before the current step began).

## SUC-002: Operator sends an interactive command-row verb within firmware's valid ranges

- **Actor**: Stakeholder / developer.
- **Preconditions**: Connected.
- **Main Flow**:
  1. Operator fills in an `S`/`T`/`D`/`R`/`TURN`/`RT`/`G` row's fields and
     clicks Send.
  2. `build_wire_string` formats the wire string per `commands.COMMANDS`'
     spec (degree→centidegree conversion and wrap for `TURN`/`RT`, optional
     `eps` omission).
  3. The GUI sends the string and logs the reply.
- **Postconditions**: Every value the UI accepts is inside the range the
  firmware itself validates — no silently-generated `ERR range …` from a
  UI field that let the operator type an out-of-firmware-range value.
- **Acceptance Criteria**:
  - [ ] Each row's declared `min`/`max` (post centidegree-conversion for
        `TURN`/`RT`) is within `docs/protocol-v2.md` §10's documented range
        for that verb.
  - [ ] `TURN`'s `eps` field cannot be set above the firmware's 1800 cdeg
        (18°) ceiling (found during planning: the UI previously allowed up
        to 180°/18000 cdeg).
  - [ ] `RT`'s `deg` field cannot be set beyond the firmware's ±180000 cdeg
        (±1800°) ceiling (found during planning: the UI previously allowed
        up to ±3600°/±360000 cdeg).

## SUC-003: Operator drives to a camera-observed world point (GOTO)

- **Actor**: Stakeholder / developer with a Relay/PLAYFIELD-mode camera feed
  (or a sim run supplying synthetic truth into `_state["last_truth"]`).
- **Preconditions**: Connected; a fresh camera truth pose is available
  (age ≤ `_GotoRunner.TRUTH_MAX_AGE_S`).
- **Main Flow**:
  1. Operator enters a target `(x, y, eps, speed)` and clicks GOTO.
  2. `_GotoRunner` repeatedly: reads the freshest cached truth pose, checks
     `goto_reached`, and if not yet arrived, sends `SI` (re-anchor pose to
     truth) then `G <x> <y> <speed>` (re-aim at the fixed target) —
     a camera-in-the-loop pure-pursuit loop throttled to `POLL_S`.
  3. On arrival (or explicit stop, or timeout), the runner sends `STOP`
     (the top-level verb — confirmed in this sprint's grounding pass to be
     a real, implemented, Planner-clearing verb as of sprint 084, not the
     dead command sprint 083 found).
- **Postconditions**: The robot is within `eps` mm of the target, stopped.
- **Acceptance Criteria**:
  - [ ] A new sim-driven integration test (no historical equivalent exists)
        exercises `_GotoRunner.run()` end to end: synthetic truth poses feed
        `_state["last_truth"]`, the runner issues `SI`/`G`, and the loop
        terminates within `eps` of the target.
  - [ ] Stopping a running GOTO re-enables the button synchronously (same
        pattern as tour-stop).
  - [ ] A stale/missing truth pose does not crash the loop — it logs and
        waits.

## SUC-004: Operator anchors robot pose via one-click Operations actions

- **Actor**: Stakeholder / developer.
- **Preconditions**: Connected (Sync Pose additionally requires a non-Sim
  transport with a working aprilcam daemon).
- **Main Flow** (three independent actions):
  - **Sync Pose**: reads tag-100 world pose from the aprilcam daemon, sends
    `SI x y h` (mm, mm, cdeg) via `build_setpose_command`.
  - **Zero Encoders**: sends `ZERO enc`.
  - **Set Robot @ 0,0**: sends `STOP` (halt + clear any in-flight goal),
    then (Sim only) teleports the plant ground truth via
    `transport.set_true_pose(0,0,0)`, then `ZERO enc`, `OZ` (re-anchor the
    OTOS heading reference), and `SI 0 0 0` — in that order — then resets
    the `TraceModel`/avatar display.
- **Postconditions**: The firmware's believed pose (and, for Set-Origin in
  Sim, the plant's true pose) matches the requested anchor.
- **Acceptance Criteria**:
  - [ ] Set-Origin's five-step sequence (`STOP`, sim-teleport, `ZERO enc`,
        `OZ`, `SI 0 0 0`) fires in that exact order.
  - [ ] Sync Pose is disabled (with an explanatory tooltip) when the active
        transport is `SimTransport`.
  - [ ] With no transport connected, Set-Origin skips the wire commands
        (logs a `[WARN]`) but still runs the display-only reset.

## SUC-005: Operator toggles telemetry streaming

- **Actor**: Stakeholder / developer.
- **Preconditions**: Connected.
- **Main Flow**: Operator toggles the STREAM button; the GUI sends
  `STREAM 50` (on) or `STREAM 0` (off) and relabels the button.
- **Postconditions**: The firmware's periodic `TLM` emission matches the
  toggle state.
- **Acceptance Criteria**:
  - [ ] Toggle-on sends `STREAM 50`; toggle-off sends `STREAM 0`.
  - [ ] A failed toggle reverts the button's visual state.
  - [ ] Disconnect resets the toggle to "off" for the next connect.

## SUC-006: System pushes robot calibration to firmware at connect and on robot-change

- **Actor**: The system (no direct operator action beyond Connect / robot
  selection).
- **Preconditions**: A robot config is active (`get_robot_config()`
  returns non-`None`).
- **Main Flow**:
  1. On Connect (and again on every robot-combo change while connected),
     `_push_robot_calibration` builds `calibration.push.calibration_commands(cfg)`
     (`SET ml=/mr=/tw=/rotSlip=`, `OI`, `OL`, `OA`, optional
     `SET odomOffX/Y/Yaw=`) and sends each in order.
  2. An uncalibrated robot ("tovez nocal") pushes the documented neutral
     sentinels (`rotSlip=0` → firmware's `effectiveSlip()` maps it to 1.0;
     `OL`/`OA` scale 1.0), so a nocal robot always runs geometry-pure
     regardless of what `DefaultConfig.cpp` bakes in at compile time.
  3. A reply containing `NODEV` (expected for `OI`/`OL`/`OA` against real
     hardware, which has no OTOS driver) is counted and logged, not treated
     as a failure.
- **Postconditions**: The firmware's live config matches the selected
  robot's calibration (or the neutral sentinel set, if uncalibrated).
- **Acceptance Criteria**:
  - [ ] Connecting with an uncalibrated robot config results in
        `GET rotSlip` reading back `0.000` (the sentinel), confirmed via
        a real headless-GUI + real ctypes-sim round trip.
  - [ ] Connecting with a calibrated robot config pushes and reads back
        that robot's actual `ml`/`mr`/`tw`/`rotSlip` values.
  - [ ] `OI`/`OL`/`OA`'s `NODEV`-tolerant handling is exercised (or at
        minimum documented as verified) against a transport that returns
        `ERR nodev`, so the push path never treats it as a hard failure.

## SUC-007: Operator applies a simulated sensor-error profile, optionally derived from calibration

- **Actor**: Stakeholder / developer running against the sim.
- **Preconditions**: Connected via `SimTransport`.
- **Main Flow**:
  1. Operator edits the Sim Errors panel's spin boxes and clicks Apply —
     the profile is persisted (`sim_prefs.save_sim_error_profile`) and
     live-applied to the running sim (`SimTransport.apply_error_profile`).
  2. Operator instead clicks "From Calibration" — the panel is populated
     with the inverse of the active robot's calibration (so the firmware's
     baked-in correction and the ideal sim plant's lack of scrub cancel),
     then Apply's same save/apply path runs once.
- **Postconditions**: The sim's live error-injection state matches the
  panel; the three no-ctypes-backing noise fields are left untouched by
  "From Calibration."
- **Acceptance Criteria**:
  - [ ] "From Calibration" with an uncalibrated robot yields the all-neutral
        (zero-error) panel.
  - [ ] "From Calibration" with a calibrated robot yields the documented
        inverse mapping.
  - [ ] "From Calibration" reuses `_on_sim_errors_apply`'s save/apply path
        exactly once each — no second, independently-written apply path.
  - [ ] A missing/partial robot config falls back to neutral per-field with
        a logged `[WARN]`, never raising.

## SUC-008: Operator views a live overhead camera feed in PLAYFIELD mode

- **Actor**: Stakeholder / developer connected via Relay.
- **Preconditions**: Connected via `RelayTransport`; an aprilcam daemon with
  an open, calibrated playfield camera.
- **Main Flow**:
  1. On a Relay connect, the GUI starts `_LiveViewWorker` on its own thread,
     which repeatedly captures, deskews (via the daemon's live homography),
     and delivers frames through `_LiveFrameBridge` (a main-thread bridge —
     a bare-function `QueuedConnection` callback would run on the emitting
     thread and never process, per the established PySide gotcha).
  2. While live view is active, the camera (not `TLM`-driven telemetry)
     owns the avatar marker: `_TelemetryBridge.on_frame_ready` calls
     `canvas_ctrl.refresh(update_marker=False)` so trace paths still redraw
     at TLM rate without fighting the camera-driven marker position.
  3. Disconnect (or leaving Relay mode) stops the worker and restores the
     static background.
- **Postconditions**: The canvas shows a live, deskewed playfield image with
  the avatar positioned from camera truth, not fused telemetry.
- **Acceptance Criteria**:
  - [ ] The live-view worker starts only for Relay connections, never
        Sim/Serial.
  - [ ] `on_frame_ready`'s live-view gate (`update_marker=False`) and
        `on_truth_ready`'s full gate (skip `refresh()` entirely when
        inactive) both hold.
  - [ ] Stopping live view (disconnect) restores the static background with
        no lingering worker thread.

## SUC-009: Operator selects a camera/relay port and sees the active transport mode

- **Actor**: Stakeholder / developer.
- **Preconditions**: None (works pre-connect).
- **Main Flow**:
  1. The camera combo lists cameras from the aprilcam daemon
     (`DaemonControl.list_cameras()`); selecting one persists the
     preference (`camera_prefs.save_camera_pref`) for future sessions.
  2. `find_relay_port`/`_relay_probe_banner` (in `transport.py`) probe
     serial ports for a relay's `!HELLO`-class banner to auto-select the
     right port.
  3. The mode label (`transport_name_to_mode_label`) shows "SIM MODE" /
     "BENCH MODE" / "PLAYFIELD MODE" based on the connected transport name.
- **Postconditions**: The operator can tell, at a glance, which transport
  is active and which camera/port will be used.
- **Acceptance Criteria**:
  - [ ] Camera-combo population and preference persistence/fallback
        (persisted pref → name-heuristic → first available) all hold.
  - [ ] Relay-port discovery correctly classifies a relay's `!HELLO`-style
        banner and rejects non-relay serial devices.
  - [ ] The mode label maps each known transport name to the correct text
        and style, and handles an unknown name gracefully.

## SUC-010: Operator records a session's wire traffic for later review

- **Actor**: Stakeholder / developer.
- **Preconditions**: None.
- **Main Flow**: Operator starts a recording; every logged TX/RX line is
  appended as a JSONL record (`t_wall`, direction, text) via
  `SessionRecorder`; pausing suppresses appends without ending the session;
  stopping returns the file path.
- **Postconditions**: A JSONL file exists with one valid-JSON line per
  recorded traffic line, in session order.
- **Acceptance Criteria**:
  - [ ] Start/pause/stop state transitions behave as documented (including
        the raise-on-start-when-already-recording/paused guards).
  - [ ] Every appended line is valid, newline-free JSON (CRLF/LF stripped).
  - [ ] `_append_log`'s TX/RX-vs-status-line direction classification
        (`direction_from_marker`) feeds the recorder correctly, and a paused
        session does not record.

## SUC-011: The full TestGUI test suite runs headless and green

- **Actor**: CI / the developer running `just testgui`'s test target.
- **Preconditions**: All of SUC-001 through SUC-010's tickets are done.
- **Main Flow**: `QT_QPA_PLATFORM=offscreen uv run pytest tests/testgui` runs
  every ported test (the pre-existing ~136 plus this sprint's ~16 newly
  ported/added files) and passes.
- **Postconditions**: `pyproject.toml`'s `testpaths` already includes
  `"tests/testgui"` (confirmed present pre-sprint); no `tests_old/testgui/`
  file remains un-ported without an explicit, recorded reason.
- **Acceptance Criteria**:
  - [ ] `QT_QPA_PLATFORM=offscreen uv run pytest tests/testgui -q` is fully
        green.
  - [ ] A scripted sim smoke pass (launch → Sim connect → run a tour to
        completion → camera GOTO drives to a point via synthetic truth →
        Sync-Pose/Set-Origin anchor pose → calibration push observed)
        succeeds, closing the sprint's (and the program's) acceptance bar.
  - [ ] The parent issue and the program-epic issue are both closeable.
