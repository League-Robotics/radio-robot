---
status: in-progress
sprint: 082
tickets:
- 082-001
- 082-002
- 082-003
- 082-004
- 082-005
---

# Plan: Revive TestGUI against the new-tree simulator

## Context

TestGUI (`host/robot_radio/testgui/`) is the interactive PySide6 "cockpit" — live
keyboard driving, four colour-coded pose traces (camera / encoder / OTOS / fused),
pre-programmed tours, camera-in-the-loop GOTO, a sim-error injection panel, and a
TX/RX log. The **greenfield rebuild (sprint 077) parked the old firmware and test
trees** (`source_old/`, `tests_old/`) and stood up a new, minimal `source/` tree.
As a result TestGUI is stranded:

- Its sim backend (`SimTransport`) points at `tests/_infra/sim/…`, which sprint 077
  moved to `tests_old/`. **Sim Connect always fails today.**
- It speaks the **old production wire protocol** (`S/T/D/R/TURN/RT/G`, `STREAM/SNAP/TLM`,
  `SET/GET`, `SI/OZ/ZERO`, and `SIMSET` for sim knobs). The new `source/` firmware
  implements **only** `PING/VER/HELP/ECHO/ID` + the `DEV` family (`DEV M`, `DEV DT VW/WHEELS/STOP`,
  `DEV STATE`, `DEV STOP`, `DEV WD`). It has **no** telemetry stream, **no** closed-loop
  motion, **no** pose estimation, and sim knobs are **ctypes-only** (no `SIMSET`).

**Sprint 081** (in progress — only ticket 001 started) rebuilds the in-process ctypes
simulator (`libfirmware_host` + `tests/_infra/sim/{sim_api.cpp,firmware.py}` + a
reconciled `host/robot_radio/io/sim_conn.py`). It explicitly scopes **TestGUI revival
OUT** as "separate, later work." **This plan is that later work.**

**Intended outcome (stakeholder decisions, this session):** a *full* TestGUI revival —
live drive, all four traces, sim-error injection, tours, GOTO, camera live-view — with
the closed-loop motion (`drive N mm` / `turn N deg` / `goto XY`) implemented as **real
firmware verbs in the new `source/` tree**, not as host-side control. That is a
multi-sprint program; this document is the roadmap.

## Hard dependency gate

Nothing here can be implemented until **sprint 081 lands** and produces:
`tests/_infra/sim/build/libfirmware_host.{dylib,so}`, `tests/_infra/sim/firmware.py`
(the `Sim` context-manager class), and the reconciled `host/robot_radio/io/sim_conn.py`
with its final ctypes ABI. Phase 3 (host) builds directly on those artifacts. The
firmware phases (1–2) can begin in parallel with the tail of 081 since they touch
`source/`, but they must be verified through the 081 sim harness.

**"Drive frame" question — resolved:** the new firmware *already* has a body-frame
twist command: `DEV DT VW <v_x> <v_y> <omega>` (mm/s, mm/s, rad/s). It is differential,
so `v_y` (strafe) is parsed and carried through `msg::BodyTwist3` but ignored
(`source/subsystems/drivetrain.cpp` `commandedWheelTargets()`, `holonomic=false`).
Forward+rotate works today. TestGUI's keyboard driver (`VW <v> <omega_mrads>`) maps
onto it directly (convert milli-rad/s → rad/s). No new command is needed for live
driving — the gap is closed-loop *motion verbs* and *telemetry/estimation*, below.

## The program — three phases (each a CLASI sprint)

### Phase 1 — Firmware: pose estimation + telemetry spine

Give the new tree a pose it can report and a stream to report it on. This closes the
"OTOS gap" (SimOdometer exists but nothing consumes it) and feeds TestGUI's four traces.

- Add an **odometry + fusion** consumer of `Hal::Odometer`: encoder dead-reckoning
  (→ `encpose`) plus an OTOS-corrected fused estimate (→ `pose`), with the raw OTOS
  reading exposed as `otos`. Port from `source_old/control/Odometry.{cpp,h}`,
  `source_old/state/{EKFTiny,PhysicalStateEstimate,PoseEstimate}.*`, adapted to the
  new `Hal::Odometer`/`Subsystems::Hardware` interfaces and Google-style naming.
- Add a **telemetry surface**: `STREAM <hz>` / `SNAP` emitting `TLM` frames carrying
  `t= mode= seq= enc= vel= pose= encpose= otos= twist=`. Port from
  `source_old/robot/RobotTelemetry.cpp`. Wire it into `source/dev_loop.cpp` so both
  ARM (`main.cpp`) and the host sim emit identical frames. TestGUI's `TraceModel.feed()`
  already consumes exactly these fields (`frame.encpose/otos/pose`), so matching the
  field vocabulary is the acceptance bar.
- **Verification:** on the 081 sim, `TLM` `pose`/`encpose` track ctypes ground-truth
  (`sim_get_true_pose_*`) within tolerance; `otos` diverges by the configured error
  knob and re-converges when knobs are zeroed. Then the hardware bench gate
  (`.claude/rules/hardware-bench-testing.md`): sensors alive, encoders run, round-trip.

### Phase 2 — Firmware: closed-loop motion verbs + config/pose-set

Add the motion executor above `Subsystems::Drivetrain` and the config surface TestGUI's
connect sequence needs.

- **Motion verbs** (closed-loop, with the `mode=` state machine TestGUI's tour runner
  polls for idle): `D` (distance), `T` (timed), `R` (arc), `TURN` (absolute heading),
  `RT` (relative turn), `G` (goto XY), `S` (streaming watchdog drive), `STOP`, and the
  `stop=<kind>:<args>` clauses. Port `source_old/superstructure/{Planner,Superstructure,
  PlannerConfig}.*`, `source_old/control/{BodyVelocityController,HaltController,
  MotorController,VelocityController}.*`, `source_old/commands/MotionCommands.*` and
  `messages/planner.h`. Each handler **stages into the `DevLoopState` outbox**
  (command-plane discipline from sprint 079) — `devLoopTick` remains the sole drainer.
- **Config + pose-set**: `SET/GET` (trackwidth, wheel calibration, PID, slip) via a
  port of `source_old/commands/ConfigCommands.*` + `source_old/robot/ConfigRegistry.*`;
  `SI` (set internal pose), `ZERO enc`, and OTOS verbs (`OZ`/`OI`/`OL`/`OA`) via
  `source_old/commands/OtosCommands.*`. These back TestGUI's connect-time calibration
  push (`robot_radio.calibration.push.calibration_commands`) and Sync-Pose / Set-Origin.
- **Key design decision (resolve in sprint architecture, recommendation below):**
  restore these as the **top-level production verbs** (`D`, `TURN`, `G`, `STREAM`, …)
  rather than under the `DEV` namespace. Rationale: `docs/protocol-v2.md` already
  specifies them, and the *entire* host stack (`robot_radio.robot.protocol`,
  calibration push, sync_pose, and TestGUI's `commands.py`) already speaks them — so
  the host churn in Phase 3 is minimal. The `DEV` family stays as the low-level bench
  surface underneath. (Alternative: keep motion under `DEV DT …` and rename the host
  schema instead — more host rework, less aligned with protocol-v2.)
- **Verification:** sim geometry tests (a `D 200 200 500` moves true pose ~500 mm; a
  `RT 9000` rotates ~90°, within plant tolerance), `stop=` clause behavior, `mode=`
  returns to `I` at completion; then the hardware bench gate (drive both directions,
  encoders increment proportionally, round-trip over serial).

### Phase 3 — Host: TestGUI reconciliation + revival

With the firmware surface restored and 081's sim in place, bring TestGUI back to life.

- **Repath to the new sim.** `SimTransport._sim_lib_path()` already targets
  `tests/_infra/sim/build/` (correct once 081 lands); reconcile its `from firmware
  import Sim` usage and method calls against 081's final `Sim` surface — prefer routing
  through the reconciled `host/robot_radio/io/sim_conn.py` (`SimConnection`) rather than
  importing `firmware.Sim` directly, so there is one ctypes contract.
- **Replace `SIMSET` error injection with ctypes.** `SimTransport._apply_profile_to_sim`
  currently chunks `SIMSET key=val` wire lines (`_SIMSET_MAX_PAIRS_PER_LINE`); rewrite it
  to call the sim's ctypes error setters (the `sim_set_*` knobs exposed by 081's C ABI,
  surfaced on `SimConnection`). Update `sim_prefs.PROFILE_TO_SIMSET_KEY` → a
  profile-field → ctypes-setter mapping.
- **Fix stale asset paths:** `canvas.py:107-109` and `traces.py:66-75` reference
  `tests/old/playfield_tour/…` (now under `tests_old/`); repoint to the surviving
  playfield image/calibration.
- **Restore tours / GOTO / live-view** on the reconciled command surface (unchanged
  verbs if Phase 2 restores the top-level forms). GOTO stays host-side (it is already a
  synthetic host loop: read camera truth → `SI` → `G`).
- **Runability:** `uv sync --group gui` (PySide6 is an optional `gui` group and is *not*
  installed); add a `justfile` recipe to launch (`uv run python -m robot_radio.testgui`);
  unskip `build.py`'s host-sim build once `tests/_infra/sim/` exists.
- **Tests:** port `tests_old/testgui/` (26 files: transport, commands, drive, traces,
  canvas, operations, tours, sim-errors, mode-indicator, …) to `tests/testgui/`; add
  `tests/testgui` to `pyproject.toml` `testpaths`. Run headless with
  `QT_QPA_PLATFORM=offscreen`.
- **Verification (end-to-end):** launch the GUI against the sim; Connect succeeds; arrow
  keys spin the wheels and the avatar/traces move; a tour runs to completion and returns
  near origin; injecting a slip/encoder-error profile visibly separates the encoder
  trace from the truth trace; the four traces render.

## Concrete file inventory

**Port `source_old/` → `source/` (Phases 1–2), reworking to the new HAL/Drivetrain
interfaces + Google C++ naming (CamelCase, no units in identifiers):**
`superstructure/{Planner,Superstructure,PlannerConfig}.*`,
`control/{Odometry,BodyVelocityController,HaltController,MotorController,VelocityController}.*`,
`state/{EKFTiny,PhysicalStateEstimate,PoseEstimate}.*`,
`robot/{RobotTelemetry,ConfigRegistry}.*`,
`commands/{MotionCommands,ConfigCommands,OtosCommands,MotionCommand}.*`,
`messages/planner.h`. Register new verbs via the `makeCmd`/`makeSchemaCmd` pattern and
concatenate into `source/main.cpp`; emit `TLM`/drain motion outbox in `source/dev_loop.cpp`.

**Modify in host (Phase 3):**
`host/robot_radio/testgui/transport.py` (`SimTransport`: `Sim`/`sim_conn` reconcile,
`_apply_profile_to_sim` → ctypes), `testgui/sim_prefs.py` (profile→setter map),
`testgui/canvas.py` + `testgui/traces.py` (asset paths), `host/robot_radio/io/sim_conn.py`
(consume 081's ABI; likely already done by 081 ticket 005), `justfile` (launch recipe +
`build-sim`), `build.py` (unskip host-sim), `pyproject.toml` (`testpaths`). Port
`tests_old/testgui/` → `tests/testgui/`. `testgui/commands.py`/`drive.py`/`operations.py`
change little **if** Phase 2 restores top-level verbs; otherwise they get remapped to the
`DEV` family.

## How this maps to CLASI

This repo runs the CLASI process and I am the team-lead. On approval, I will convert this
roadmap into CLASI issues and a **sequence of sprints (082 estimation+telemetry, 083
motion+config, 084 TestGUI revival)** via the normal issue → sprint-plan → ticket flow —
sequenced after sprint 081 closes. The phase boundaries above are the sprint boundaries.
Each firmware sprint carries the standing hardware-bench gate; the host sprint carries
the headless GUI test suite + a live end-to-end run against the sim.

## Open decisions for sprint architecture (recommendations noted)

1. **Verb namespace** — restore top-level production verbs (recommended, minimizes host
   churn, matches protocol-v2) vs. expose motion under `DEV`.
2. **Fused-pose trace** — Phase 1 EKF fusion feeds it; if fusion slips a sprint, alias
   the "fused" trace to `encpose` temporarily (drop nothing from the GUI).
3. **Telemetry model** — streamed `TLM` (recommended; TestGUI already parses it) vs. a
   host polling loop over `DEV STATE`. Streaming is closer to the old contract and the
   `mode=` idle signal tours rely on.
