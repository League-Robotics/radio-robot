---
status: pending
---

# Realign host tooling (TestGUI / robot_radio) + testgui tests to the gutted wire surface

## Context

Sprint 093 gutted the firmware to a four-verb live wire surface
(`PING`/`HELLO`/`S`/`STOP`) — see
[`simplify-the-main-loop-strip-it-to-bare-wheel-driving.md`](simplify-the-main-loop-strip-it-to-bare-wheel-driving.md).
Every other verb (`DEV*`, `SET`/`GET`, `STREAM`/`SNAP`/`TLM`, pose/OTOS, and the
`T`/`D`/`R`/`TURN`/`RT`/`G` motion verbs) now replies `ERR unknown`. Sprint 093
deliberately **did not touch host-side code** (architecture-update.md Step 5:
host-tooling fallout is accepted for the bench-bring-up phase).

Fallout confirmed during sprint 093 (ticket 003): `tests/testgui/` drops from
**364/364 green (pre-093)** to **16 failures** across 7 files
(`test_calibration_push_on_connect.py`, `test_error_divergence.py`,
`test_goto.py`, `test_set_origin.py`, `test_sim_errors_panel.py`,
`test_tour1_geometry.py`, `test_traces.py`, `test_transport.py`) — all because
TestGUI / `robot_radio` drive now-removed verbs (calibration `SET` on connect,
`GET`, telemetry, GOTO, OTOS, tours). `tests/sim` (37/37) and `tests/unit`
(4/4) are green; the sim close-gate is honest.

This issue tracks bringing the host side back into agreement with whatever wire
surface the robot actually exposes — **best done together with the "commands"
phase** (drivetrain motion command from the communicator,
[`communicator-drivetrain-motion-command-segment.md`](communicator-drivetrain-motion-command-segment.md),
+ [`drivetrain-becomes-the-motion-planner-segment-executing-subsystem.md`](drivetrain-becomes-the-motion-planner-segment-executing-subsystem.md)),
since that phase defines the real command surface the host should target.

## Scope (to be decided in planning)

- **TestGUI / robot_radio**: stop sending removed verbs on connect (the
  calibration-`SET`-on-connect push, GOTO, tours, telemetry polling) — or gate
  them on a capability/verb-probe so they degrade gracefully against a
  minimal-surface robot.
- **tests/testgui/**: either park the 16 removed-surface failures the same way
  sprint 093 parked `tests/sim` (a `parked-NNN/` leaf + `norecursedirs`), or
  update them to the new surface — whichever matches the host realignment above.
- **Gate hygiene**: `pyproject.toml`'s `testpaths` currently includes
  `tests/testgui`, but `tests/CLAUDE.md` claims the collected gate is
  `tests/sim` only — that doc is **stale**. Reconcile: either update
  `tests/CLAUDE.md` to match `testpaths`, or narrow `testpaths` to the intended
  gate. Decide alongside the testgui triage.

## Not blocking

Sprint 093's sim close-gate (`tests/sim` + `tests/unit`) is green and the
bench gate is firmware-only; this host realignment is deliberately deferred and
does not block sprint 093 closure.
