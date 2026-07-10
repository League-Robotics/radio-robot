---
status: pending
---

# Realign host tooling (TestGUI / robot_radio) + testgui tests to the current wire surface

> Refreshed 2026-07-09 (stakeholder triage). Originally filed against sprint
> 093's four-verb surface; the surface has since grown back under sprint 094
> and the teleop work — the fallout and the ask are updated below. The
> filename keeps its original (now historical) "four-verb" wording so
> existing references from archived sprint docs stay valid.

## Context

Sprint 093 gutted the firmware wire surface
(`simplify-the-main-loop-strip-it-to-bare-wheel-driving.md`, archived at
`clasi/sprints/done/093-simplify-the-main-loop-bare-wheel-driving-executive/issues/done/`);
sprint 094 + the teleop OOP work rebuilt it around the
segment-executing Drivetrain. The **current live surface** (see
`buildTable()` in `source/runtime/command_router.cpp` — only the `system`
and `motion` families are wired) is:

- **System:** `PING` `VER` `HELP` `ECHO` `ID` `HELLO`
- **Motion:** `S` `STOP` `D` `T` `RT` (re-parsed into `Motion::Segment`s),
  `MOVE`, `MOVER` (REPLACE/deadman teleop segment), `TLM` (one-shot pull),
  `QLEN`

Still **unregistered** (files intact on disk, families un-wired):
`SET`/`GET`, `STREAM`/`SNAP`, all `DEV *`, the OTOS verbs
(`OI`/`OZ`/`OR`/`OP`/`OV`/`OL`/`OA`), and `SI`/`ZERO`. `R` (arc), absolute
`TURN`, and `G` (GOTO) are off pending the pose-stack restoration
([`restore-goto-pursuit-with-pose-estimator.md`](restore-goto-pursuit-with-pose-estimator.md)).

Sprint 093 deliberately did not touch host-side code, and that fallout
stands: TestGUI / `robot_radio` still drive removed verbs — the
calibration-`SET` push on connect, `GET`, `STREAM`/`SNAP` telemetry polling,
GOTO, tours, OTOS — and `tests/testgui/` dropped from 364/364 green
(pre-093) to 16 failures across 7 files
(`test_calibration_push_on_connect.py`, `test_error_divergence.py`,
`test_goto.py`, `test_set_origin.py`, `test_sim_errors_panel.py`,
`test_tour1_geometry.py`, `test_traces.py`, `test_transport.py`).
`tests/sim` and `tests/unit` are green; the sim close-gate is honest.

## Scope (to be decided in planning)

- **TestGUI / robot_radio**: target the surface above — stop sending
  removed verbs on connect (calibration-`SET` push, GOTO, tours,
  `STREAM`/`SNAP` polling), or gate them on a capability/verb-probe so they
  degrade gracefully. Adopt the new verbs where they replace old ones:
  `MOVE`/`MOVER` for motion, pull-based `TLM` for telemetry (the teleop
  tool already speaks `MOVER` — reuse its pattern).
- **`SET`/`GET` decision**: the config family is unwired, so nothing
  host-side can push calibration. Decide with the stakeholder whether the
  config family comes back on the wire (e.g. for `jmax`/`yawjmax`, the
  jerk knobs sprint 094's drivetrain issue wanted live) or the host stops
  assuming it — this issue should not unilaterally re-wire firmware
  families.
- **tests/testgui/**: park the removed-surface failures (a `parked-NNN/`
  leaf + `norecursedirs`, as sprint 093 did for `tests/sim`) or update them
  to the new surface — whichever matches the host realignment above.
- **Gate hygiene**: `pyproject.toml` `testpaths` still includes
  `tests/testgui`, but `tests/CLAUDE.md` claims the collected gate is
  `tests/sim` only — reconcile the stale doc with the actual gate,
  alongside the testgui triage.

## Not blocking

The sim close-gate (`tests/sim` + `tests/unit`) is green and the bench gate
is firmware-only; this host realignment is deliberately deferred. Best done
once the wire surface stabilizes after sprint 094 closes.
