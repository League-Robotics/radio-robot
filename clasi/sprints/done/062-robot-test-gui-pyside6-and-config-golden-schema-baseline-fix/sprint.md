---
id: '062'
title: Robot Test GUI (PySide6) and config-golden/schema baseline fix
status: done
branch: sprint/062-robot-test-gui-pyside6-and-config-golden-schema-baseline-fix
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
- SUC-006
- SUC-007
- SUC-008
- SUC-009
- SUC-010
- SUC-011
- SUC-012
- SUC-013
- SUC-014
- SUC-015
- SUC-016
- SUC-017
issues:
- plan-robot-test-gui-pyside6.md
- tag-offset-mm-z-field-schema-mismatch.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 062: Robot Test GUI (PySide6) and config-golden/schema baseline fix

## Goals

1. Build a PySide6 interactive cockpit (`python -m robot_radio.testgui`) for
   driving the robot and watching how four pose estimates diverge in real time
   — camera truth, encoder odometry, OTOS odometry, and fused EKF — against a
   playfield image background.
2. Fix the two pre-existing baseline test failures: the `tag_offset_mm.z` schema
   gap and the stale DefaultConfig golden snapshot (with stakeholder SSOT
   confirmation for the golden refresh).

## Problem

The project has excellent low-level tooling — a Python protocol client, a ctypes
firmware simulator with realistic sensor degradation, an aprilcam daemon, and a
collection of one-off bench scripts — but no unified interactive cockpit. Developers
must read log lines to detect pose drift, which makes sensor calibration and
algorithm tuning slow. Additionally, two baseline tests have been failing since
sprint 054 (`test_tovez_validates_against_schema` and
`test_default_robot_config_unchanged`), creating noise in the CI gate.

## Solution

**GUI:** A PySide6 `QMainWindow` in a new `host/robot_radio/testgui/` package,
launchable as `python -m robot_radio.testgui`. Three backends behind a `Transport`
ABC: `SerialTransport`, `RelayTransport`, and `SimTransport` (ctypes). Command
rows for every motion command (S/T/D/R/TURN/G), an operations panel
(sync-pose/stop/clear/stream), a right-side `QGraphicsView` canvas with four
colored trace polylines and a robot marker, and cursor-key interactive driving.

**Schema fix (ticket 001):** Add `z: {type: number}` to `OffsetXYYaw` in
`data/robots/robot_config.schema.json`. One-line change; fixes `test_tovez_validates_against_schema`.

**Golden refresh (ticket 002):** Refresh the DefaultConfig pin after stakeholder
confirms SSOT values for `odomOffY` (3.5 vs golden 4.0) and `yawRateMax` (70 vs
golden 35.0). Programmer must surface the SSOT values and wait for stakeholder
confirmation before committing.

## Success Criteria

- `python -m robot_radio.testgui` launches in Sim mode; sending `D 200 200 500`
  shows four traces that start together and diverge as the sim's slip/noise
  applies.
- `python -m robot_radio.testgui` with relay transport: camera-truth trace tracks
  the real robot; Sync Pose re-anchors odometry.
- `uv run python -m pytest tests/testgui/` passes (headless, no display).
- `uv run python -m pytest tests/simulation` passes — both baseline failures are
  resolved.

## Scope

### In Scope

- New `host/robot_radio/testgui/` package: `transport.py`, `traces.py`, `app.py`,
  `drive.py`, `__main__.py`, `README.md`.
- PySide6 `gui` dependency group in `host/pyproject.toml`.
- `tests/testgui/` headless smoke test suite.
- `data/robots/robot_config.schema.json` — `OffsetXYYaw.z` addition.
- DefaultConfig golden refresh (gated on stakeholder SSOT confirmation).

### Out of Scope

- Firmware changes of any kind.
- Changes to `rogo` CLI or `robot_radio` runtime modules (read-only reuse only).
- Mirroring traces to the external aprilcam live overlay.
- `NezhaKinematic.go_to_world` (G4) host path.
- Automatic camera correction during a traversal.

## Test Strategy

- **Simulation gate (`tests/simulation`):** must pass; no GUI code introduced
  into this path.
- **Headless GUI smoke tests (`tests/testgui/`):** `QT_QPA_PLATFORM=offscreen`;
  fake Transport + synthetic TLM; validates trace model, robot marker, and
  command wire strings. Requires `uv sync --group gui`.
- **Manual sim end-to-end:** `python -m robot_radio.testgui` → Sim → D command
  → trace divergence visible.
- **Manual hardware/relay (opt-in):** relay + aprilcam daemon → camera-truth
  trace tracks robot; Sync Pose works.

## Architecture Notes

See `architecture-update.md`. Key decisions:
- `Transport` ABC isolates `app.py` from backend details.
- `SimTransport` owns the tick-thread and a lock for thread-safe command sends.
- `cmd_sync_pose` reuse from `cli.py` — OQ-1: verify importability; extract if
  tightly coupled to Click registration.
- Playfield image default from `tests/old/playfield_tour/` — OQ-2: path
  resolution documented in ticket 008 commit.
- Schema `z` addition uses the existing `OffsetXYYaw` def (not a new type) —
  also allows `z` on `odometry_offset_mm`, which is intentional.

## Stakeholder Decision Required

**Ticket 002 (golden refresh) is GATED on stakeholder confirmation.** Before
the programmer commits the golden update, the stakeholder must confirm:
- Is `yawRateMax = 70` the intended SSOT value? (Golden has 35.0.)
- Is `odomOffY = 3.5` the intended SSOT value? (Golden has 4.0.)

The SSOT is in `source/robot/DefaultConfig.cpp` (generated by `scripts/gen_default_config.py`
from `data/robots/robot_config.schema.json`). The programmer will surface these
values to the team-lead for stakeholder confirmation before committing.

## GitHub Issues

(None yet.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [x] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Add z property to OffsetXYYaw schema definition | — |
| 002 | Refresh DefaultConfig golden after stakeholder SSOT confirmation | 001 |
| 003 | Add gui dependency group and testgui package skeleton | — |
| 004 | Transport ABC and Serial/Relay wrappers with TLM reader thread | 003 |
| 005 | SimTransport: ctypes tick-thread, TLM from get_async_evts, sim-lib build check | 004 |
| 006 | Schema-driven command-entry rows and Send buttons | 004 |
| 007 | Operations panel: sync-pose, zero-encoders, STOP, clear-traces, refresh-playfield, STREAM toggle | 004, 005 |
| 008 | TraceModel and playfield QGraphicsView canvas with robot marker | 005, 006, 007 |
| 009 | Interactive cursor-key driving with keepalive timer | 008 |
| 010 | testgui README and headless CI smoke test | 009 |

Tickets execute serially in the order listed.
