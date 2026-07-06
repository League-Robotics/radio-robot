---
status: in-progress
sprint: 085
tickets:
- 085-001
- 085-002
- 085-003
- 085-004
- 085-005
- 085-006
- 085-007
- 085-008
- 085-009
---

# Host TestGUI full revival — tours, GOTO, calibration, live view

## Context

Once the sim cockpit ([[host-testgui-sim-cockpit]]) is live and the firmware
motion ([[firmware-closed-loop-motion-verbs]]) and config/pose-set
([[firmware-config-and-pose-set-surface]]) surfaces exist, this issue wires
TestGUI's remaining features onto them so the cockpit is fully restored — the end
state of the TestGUI-revival program ([[plan-revive-testgui-against-the-new-tree-simulator]]).

## Scope

- **Command rows** (`testgui/commands.py`): point `S/T/D/R/TURN/RT/G` at the new
  top-level motion verbs; keep the centidegree/wrap conventions.
- **Tours** (`_TourRunner`): run `D`/`RT` sequences with `SNAP`-polled `mode=I`
  completion detection (now that 084 provides the `mode=` state machine).
- **Camera GOTO** (`_GotoRunner`): the host-side pure-pursuit loop —
  read camera truth → `SI` (set pose) → `G` (go-to) — restored on the new verbs.
- **Operations panel** (`testgui/operations.py`): Sync-Pose (`SI`),
  Zero-Encoders (`ZERO enc`), Set-Origin (`STOP`+plant-teleport+`ZERO`+`OZ`+`SI`),
  STREAM toggle.
- **Connect-time calibration push** (`calibration.push.calibration_commands`) via
  the new `SET`/OTOS verbs.
- **Live camera view** (Relay/PLAYFIELD mode) and remaining GUI test port
  (`tests_old/testgui/` → `tests/testgui/`), added to `pyproject.toml` `testpaths`.

## Acceptance (sketch)

Full cockpit against the sim (and, where applicable, the bench/relay): a tour runs
to completion and returns near origin; camera GOTO drives to a world point;
Sync-Pose/Set-Origin anchor pose; calibration pushes on connect; the full GUI
test suite is green.

## Dependencies

Depends on [[host-testgui-sim-cockpit]], [[firmware-closed-loop-motion-verbs]],
[[firmware-config-and-pose-set-surface]]. Completes the program epic
[[plan-revive-testgui-against-the-new-tree-simulator]].
