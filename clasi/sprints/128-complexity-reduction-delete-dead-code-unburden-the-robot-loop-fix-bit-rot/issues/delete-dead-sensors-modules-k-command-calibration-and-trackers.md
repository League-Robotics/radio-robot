---
status: in-progress
sprint: '128'
tickets:
- 128-006
---

# Delete sensors/ dead modules: the retired K-command calibration.py and the unreferenced trackers

**Source:** code review 2026-07-30, `03-host-core.md` MAJOR §8.
**Priority:** P1 — pure deletion; zero live callers for all three targets.
**Goal served:** `sensors/` currently offers a future reader TWO calibration
modules (one dead, sitting beside the live `robot_radio.calibration` package
under a near-identical name) and THREE pose-tracking classes (two dead, one
of which reproduces a known stale-pose hazard). Every one is a false lead in
a bug hunt.

## What is wrong

- `sensors/calibration.py` — its own docstring says the `K`-command wire
  family it sends "has been retired"; zero live callers; the real
  `robot_radio.calibration` package sits right next to it.
- `sensors/cam_tracker.py` (`CamTracker`) and `sensors/odom_tracker.py`
  (`OdomTracker`) — zero live callers repo-wide, yet exported from
  `sensors/__init__.py` alongside their replacement,
  `sensors/odometry.py::Odometry`. `CamTracker.update()` also reproduces the
  frozen-anchor hazard: on a miss it keeps last-good `pos`/`yaw` with no
  `is_valid` staleness flag — the exact bug class `Odometry` was built to
  fix.

## What to do

Delete all three files and their `sensors/__init__.py` exports (including
the `CamTracker` lazy-load slot). `Odometry` (with its `STALE_AGE`/
`is_valid`/`source` contract) is the one pose-tracking module `sensors/`
keeps — it is the model the review holds up as correct.

If anyone believes `CamTracker`/`OdomTracker` capability is still wanted,
the bar is: wire it to a real caller WITH `Odometry`'s staleness contract in
the same PR — otherwise it goes. Half-retired is the one disallowed state.

## Acceptance

- `grep -rn "cam_tracker\|odom_tracker\|from .calibration import\|sensors.calibration" src/ --include=*.py`
  returns nothing outside `src/archive`.
- `python -c "import robot_radio.sensors"` works; full pytest suite passes.
- `src/host/robot_radio/DESIGN.md`'s sensors row updated in the same commit.
