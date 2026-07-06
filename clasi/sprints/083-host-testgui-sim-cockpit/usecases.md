---
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 083 Use Cases

Parent issue: [`clasi/issues/host-testgui-sim-cockpit.md`](../../issues/host-testgui-sim-cockpit.md).

These use cases scope the earliest usable TestGUI cockpit: launch, connect to
the ctypes simulator, drive with arrow keys, watch pose traces, and inject
simulated sensor/actuator error profiles. No firmware work; no camera/tour/
GOTO features (those need motion/config verbs that do not exist yet — see
sprint 084/085).

## SUC-001: Operator connects TestGUI to the simulator

- **Actor**: Stakeholder / developer running the TestGUI locally.
- **Preconditions**: `libfirmware_host.{dylib,so}` is built
  (`tests/_infra/sim/build/`); `uv sync --group gui` has been run.
- **Main Flow**:
  1. Operator runs `python -m robot_radio.testgui`.
  2. Operator selects "Sim" in the transport combo and clicks Connect.
  3. `SimTransport` constructs a `SimConnection` on its tick-thread, starts
     telemetry streaming, and applies the persisted sim error profile.
  4. The mode label shows "SIM MODE"; the log pane shows the connect
     sequence; Send/drive controls enable.
- **Postconditions**: The tick-thread is advancing sim time at wall-clock
  rate; TLM frames are flowing to the GUI.
- **Acceptance Criteria**:
  - [ ] Connect succeeds against the current `libfirmware_host` build with no
        reference to a nonexistent `Sim`/`SimConnection` method.
  - [ ] Disconnect cleanly joins the tick-thread with no lingering handle.
  - [ ] A missing library build still shows the existing "Build required"
        warning path unchanged.

## SUC-002: Operator drives the simulated robot with arrow keys

- **Actor**: Stakeholder / developer.
- **Preconditions**: SUC-001 complete (Sim connected).
- **Main Flow**:
  1. Operator gives the main window keyboard focus and presses an arrow key.
  2. `KeyboardDriver` binds the drivetrain (`DEV DT PORTS 1 2`) once per
     session, then sends `DEV DT VW <v_x> 0 <omega>` (mm/s, mm/s, rad/s) for
     the held direction, resending on a keepalive timer while held.
  3. The simulated plant accelerates; the canvas avatar moves/rotates.
  4. Operator releases the key (or the window loses focus); `DEV DT STOP` is
     sent a bounded number of times (deadman resend) and the keepalive
     disarms.
- **Postconditions**: The robot is stationary after release; no wire string
  sent by the driver references a verb the current firmware does not
  implement.
- **Acceptance Criteria**:
  - [ ] Up/Down/Left/Right each produce the correct `DEV DT VW` line (forward,
        back, CCW, CW) with omega in rad/s, not milli-rad/s.
  - [ ] Release (or focus loss) reliably stops the plant within the bounded
        resend window.
  - [ ] `vw_line_for_key`/`vw_line_for_key_set` remain Qt-free, pure, unit
        testable functions.

## SUC-003: Operator observes live pose traces while driving

- **Actor**: Stakeholder / developer.
- **Preconditions**: SUC-001 and SUC-002 (or direct wire drive) producing TLM
  traffic and ground truth.
- **Main Flow**:
  1. `TraceModel.feed()` ingests each `TLMFrame`'s `encpose=`/`otos=`/`pose=`
     fields; `feed_truth()` ingests the sim's ground-truth pose delivered via
     `on_truth`.
  2. `CanvasController.refresh()` redraws the four polylines (camera/encoder/
     otos/fused) and the robot marker over the playfield background.
- **Postconditions**: All four traces render and grow as the robot moves; the
  canvas shows the correct playfield background image (not a broken-path
  fallback grey box, unless genuinely running with no camera assets).
- **Acceptance Criteria**:
  - [ ] The bundled playfield calibration/image assets resolve to their
        actual on-disk location under the rebuilt `tests_old/` tree.
  - [ ] Encoder/OTOS/fused traces visibly track the green camera-truth trace
        during a straight drive and a turn.

## SUC-004: Operator injects a simulated error profile and observes divergence

- **Actor**: Stakeholder / developer.
- **Preconditions**: SUC-001 (Sim connected).
- **Main Flow**:
  1. Operator adjusts a Sim Errors field (e.g. encoder noise, encoder scale
     error) and clicks Apply.
  2. The profile is persisted (`sim_prefs.save_sim_error_profile`) and, if
     connected, applied live to the running sim via the ctypes `sim_set_*`
     setters — no `SIMSET` wire command is sent (the firmware does not
     register that verb).
  3. Operator drives; the encoder/OTOS trace visibly diverges from the green
     camera-truth trace by an amount consistent with the injected error.
- **Postconditions**: The applied profile survives reconnect (persisted to
  `data/testgui/sim_error_profile.json`).
- **Acceptance Criteria**:
  - [ ] Every profile field that has a ctypes setter in the sprint-081 ABI is
        applied through that setter, not a wire command.
  - [ ] Profile fields with no ctypes backing (see architecture-update.md
        Design Rationale) degrade gracefully — logged, never crash the Apply
        action.
  - [ ] A nonzero encoder-noise or encoder-scale-error profile produces a
        measurable encoder-vs-truth trace separation in a scripted test.

## SUC-005: Operator stops the robot and disconnects cleanly

- **Actor**: Stakeholder / developer.
- **Preconditions**: Robot driving (SUC-002) or idle, Sim connected.
- **Main Flow**:
  1. Operator clicks the Operations panel's STOP button (or releases the
     drive keys).
  2. The GUI cancels any running background worker, sends a wire verb the
     firmware actually implements (`DEV DT STOP`), and stops telemetry
     streaming.
- **Postconditions**: The plant is stationary; no background thread keeps
  driving it.
- **Acceptance Criteria**:
  - [ ] The Operations panel STOP button sends a verb the current firmware
        recognizes (not the legacy bare `STOP`).
  - [ ] STOP works from both the keyboard-release path and the explicit
        button.

## SUC-006: Developer runs the TestGUI test suite headlessly

- **Actor**: Developer / CI.
- **Preconditions**: `uv sync --group gui` has installed PySide6; the sim
  library is built.
- **Main Flow**:
  1. Developer runs
     `QT_QPA_PLATFORM=offscreen uv run pytest tests/testgui`.
  2. Ported unit/integration tests exercise transport reconciliation, drive
     key mapping, trace accumulation, sim-prefs mapping, and canvas asset
     resolution without a real display.
- **Postconditions**: Tests are green; `tests/testgui` is collected by
  `pyproject.toml`'s `testpaths` alongside `tests/sim`/`tests/unit`.
- **Acceptance Criteria**:
  - [ ] `robot_radio.testgui` and its submodules import without PySide6
        installed (lazy-import discipline preserved).
  - [ ] The ported test files run green under `QT_QPA_PLATFORM=offscreen`.
  - [ ] `pyproject.toml`'s `testpaths` includes `tests/testgui`.
