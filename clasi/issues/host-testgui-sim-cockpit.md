---
status: pending
sprint: 083
---

# Host TestGUI sim cockpit — drive + traces + sim-error injection against the new sim

## Context

TestGUI (`host/robot_radio/testgui/`) is stranded: it points at old paths and
speaks the pre-greenfield protocol. But after sprints 081 (host ctypes sim) and
082 (pose estimation + `TLM`/`STREAM`/`SNAP`), **everything a basic sim cockpit
needs already exists**: the built `tests/_infra/sim/build/libfirmware_host.dylib`,
the `Sim` wrapper (`tests/_infra/sim/firmware.py`), body-twist driving
(`DEV DT VW`), and streamed pose telemetry (`TLM enc/pose/encpose/otos/twist/mode`).
This issue reconciles TestGUI to that surface so the stakeholder can **turn on the
GUI, connect to the simulator, drive with the arrow keys, and watch the pose
traces** — the fastest path to "running things," on the current firmware, with no
new firmware work.

## Scope (this issue — the minimal usable cockpit)

- **Repath + reconcile the sim transport.** `SimTransport` (`testgui/transport.py`)
  already resolves `tests/_infra/sim/build/libfirmware_host.*` (now present).
  Reconcile its `Sim`/`sim_conn` calls against sprint 081's final ABI; prefer
  routing through `host/robot_radio/io/sim_conn.py`.
- **Driving → `DEV DT VW`.** `testgui/drive.py`'s keyboard driver sends the old
  `VW <v> <omega_mrads>`; map it to `DEV DT VW <v_x> 0 <omega_rads>` (convert
  milli-rad/s → rad/s). Ensure `DEV DT PORTS` binding + `DEV DT STOP` on release.
- **Sim-error injection → ctypes.** Replace the 15 `SIMSET` wire references in
  `SimTransport._apply_profile_to_sim` with the sim's ctypes error setters
  (081's `sim_set_*`); update `testgui/sim_prefs.PROFILE_TO_SIMSET_KEY` → a
  field→setter map.
- **Traces.** Feed `TraceModel` from 082's `TLM` frames (`enc/encpose/pose/otos`)
  plus ground-truth via `sim.get_true_pose()`. Fix stale playfield asset paths in
  `testgui/canvas.py`/`traces.py` (`tests/old/...` → surviving location).
- **Runability.** `uv sync --group gui` (PySide6), a `justfile` launch recipe, and
  port the core headless GUI tests (`tests_old/testgui/{test_transport,test_drive,
  test_traces,test_sim_prefs,...}`) to `tests/testgui/`.

## Out of scope (deferred to the full-revival issue)

Tours, camera GOTO, Sync-Pose, Set-Origin, calibration-push, live camera view —
all need firmware motion/config verbs (separate issues). This issue delivers a
drive-and-observe cockpit only.

## Acceptance (sketch)

Launch the GUI (`uv run python -m robot_radio.testgui`), select Sim, Connect
succeeds; arrow keys spin the wheels and move the avatar; the encoder/OTOS/truth
traces render and update; injecting a slip/encoder-error profile visibly
separates the encoder trace from truth; headless GUI tests green under
`QT_QPA_PLATFORM=offscreen`.

## Dependencies

Depends on 081 (sim) + 082 (telemetry) — both done. No firmware work.
Related: [[host-testgui-full-revival]] builds on this.
