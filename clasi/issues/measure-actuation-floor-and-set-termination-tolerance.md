---
status: pending
---

# Measure the actuation floor and replace the provisional TERMINATION_TOLERANCE

## Description

`src/host/robot_radio/pathplan/planner.py:108` still carries
`TERMINATION_TOLERANCE = 100.0` marked `# PROVISIONAL -- pending ticket 007`.
Sprint 127 closed without taking the measurement that was supposed to replace it.

**The provisional value is demonstrably too loose.** In the goto-mode playfield
run, corner 1 reported `success=True, arrived within 100 mm (distance=96.8 mm)`
against a target 250 mm away — a 39% shortfall counted as arrival.

## Cause

The number needed is the **actuation floor**: the smallest commanded distance and
turn angle the drivetrain reliably executes. Below it, commands fall under the
motor write-suppression deadband and the robot stalls rather than creeping — the
failure already fixed once with the `copysign(deadband)` boost.

It cannot be measured on a stand. Unloaded wheels have no friction, no robot
weight and no drivetrain load, so the smallest command that produces motion there
is optimistically low and wrong for the field.

## Proposed fix

A fenced playfield sweep of decreasing commanded distances and angles, scoring
whether the robot actually moves and how repeatably, until it stops responding.
`src/tests/bench/square_tour.py --mode actuation-floor` already exists for this
and refuses to run without a camera geofence.

Then write the measured value into `planner.py:108`, drop the PROVISIONAL marker,
and re-run the goto-mode convergence gate against it.

Note the tolerance is bounded on both sides for path following, and the walls are
close together — see
[`path-following-hardware-gaps.md`](path-following-hardware-gaps.md). Too small
and the outer loop cannot resolve arrival (it overshoots every waypoint); too
large and it "arrives" at waypoints it never approached.

## Verification

- The sweep's raw data recorded, not just the chosen number.
- `planner.py:108` carries a measured value with its provenance.
- The goto-mode playfield gate re-run and passing against it.

## Related

- Sprint 127 ticket 007 (`clasi/sprints/done/127-*/tickets/`) — its exception
  block records exactly what was and was not done.
- `docs/bench-reports/2026-07-30-square-tour-dead-time.md` — why the bench tier
  cannot substitute for the field here.
