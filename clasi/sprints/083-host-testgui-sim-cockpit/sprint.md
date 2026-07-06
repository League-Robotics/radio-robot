---
id: 083
title: Host TestGUI sim cockpit
status: roadmap
branch: sprint/083-host-testgui-sim-cockpit
use-cases: []
issues:
- host-testgui-sim-cockpit.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 083: Host TestGUI sim cockpit

## Goals

Reconcile the stranded TestGUI (`host/robot_radio/testgui/`) to the sim +
telemetry surface delivered in sprints 081 (host ctypes sim) and 082 (pose
estimation + `TLM`/`STREAM`/`SNAP`) so the stakeholder can launch the GUI,
connect to the simulator, drive with the arrow keys, and watch live pose
traces. This is the earliest usable cockpit in the TestGUI-revival program —
it runs on 081+082 alone, with **no new firmware work**.

**Delivers:** a drive-and-observe TestGUI cockpit against the simulator — the
fastest path to "running things."

**Dependency:** none new — depends only on 081 (sim) and 082 (telemetry),
both already done. Can start immediately.

## Problem

TestGUI still points at old paths and speaks the pre-greenfield wire
protocol, so it cannot connect to anything in the new `source/` tree even
though everything a basic cockpit needs (sim library, body-twist driving,
streamed pose telemetry) now exists.

## Solution

Repath and reconcile `SimTransport`/`drive.py`/`sim_prefs.py`/`canvas.py` to
the current sim ABI and `TLM` wire format, dropping the old `SIMSET`/`VW`
wire calls in favor of ctypes and `DEV DT VW`.

## Success Criteria

Launch the GUI, select Sim, Connect succeeds; arrow keys spin the wheels and
move the avatar; encoder/OTOS/truth traces render and update; injecting a
slip/encoder-error profile visibly separates the encoder trace from truth;
headless GUI tests green under `QT_QPA_PLATFORM=offscreen`.

## Scope

### In Scope

- Repath + reconcile the sim transport: `SimTransport` (`testgui/transport.py`)
  against sprint 081's final ctypes ABI, preferring `host/robot_radio/io/sim_conn.py`.
- Driving → `DEV DT VW`: map `drive.py`'s keyboard driver (old `VW <v> <omega_mrads>`)
  to `DEV DT VW <v_x> 0 <omega_rads>` (mrad/s → rad/s); `DEV DT PORTS` binding +
  `DEV DT STOP` on release.
- Sim-error injection → ctypes: replace the 15 `SIMSET` wire references in
  `SimTransport._apply_profile_to_sim` with 081's `sim_set_*` ctypes setters;
  update `sim_prefs.PROFILE_TO_SIMSET_KEY` to a field→setter map.
- Traces: feed `TraceModel` from 082's `TLM` frames (`enc/encpose/pose/otos`)
  plus ground truth via `sim.get_true_pose()`; fix stale playfield asset paths
  in `canvas.py`/`traces.py`.
- Runability: `uv sync --group gui` (PySide6), a `justfile` launch recipe, and
  port the core headless GUI tests (`test_transport`, `test_drive`,
  `test_traces`, `test_sim_prefs`, ...) to `tests/testgui/`.

### Out of Scope

Tours, camera GOTO, Sync-Pose, Set-Origin, calibration-push, live camera view
— all need firmware motion/config verbs that don't exist yet (sprint 084).
Deferred to sprint 085 (full revival).

## Test Strategy

Headless GUI tests under `QT_QPA_PLATFORM=offscreen`, exercised against the
sim library. Detail-planning phase will size out the specific test files.

## Architecture Notes

No new firmware or architecture changes — this sprint is host-side
reconciliation only, wiring existing TestGUI modules to the existing
081/082 surface. Full architecture-update.md follows at detail-planning time.

## GitHub Issues

(GitHub issues linked to this sprint's tickets. Format: `owner/repo#N`.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [ ] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [ ] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|

Tickets execute serially in the order listed.
