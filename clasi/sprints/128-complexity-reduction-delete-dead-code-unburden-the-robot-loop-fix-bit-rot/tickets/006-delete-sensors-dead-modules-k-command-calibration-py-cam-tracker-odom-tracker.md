---
id: '006'
title: 'Delete sensors/ dead modules: K-command calibration.py, cam_tracker, odom_tracker'
status: open
use-cases: [SUC-006]
depends-on: []
github-issue: ''
issue: delete-dead-sensors-modules-k-command-calibration-and-trackers.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Delete sensors/ dead modules: K-command calibration.py, cam_tracker, odom_tracker

**Worktree note**: implement in the git worktree checked out for
`sprint/128-complexity-reduction-delete-dead-code-unburden-the-robot-loop-fix-bit-rot`.
All paths below are relative to that worktree.

## Description

`sensors/calibration.py`'s own docstring says the `K`-command wire family
it sends "has been retired"; zero live callers; the real
`robot_radio.calibration` package sits right next to it under a
near-identical name. `sensors/cam_tracker.py` (`CamTracker`) and
`sensors/odom_tracker.py` (`OdomTracker`) have zero live callers
repo-wide, yet are exported from `sensors/__init__.py` alongside their
replacement, `sensors/odometry.py::Odometry`. `CamTracker.update()` also
reproduces the frozen-anchor hazard: on a miss it keeps last-good
`pos`/`yaw` with no `is_valid` staleness flag — the exact bug class
`Odometry` was built to fix.

## Acceptance Criteria

- [ ] `sensors/calibration.py`, `sensors/cam_tracker.py`,
      `sensors/odom_tracker.py` are deleted.
- [ ] Their `sensors/__init__.py` exports are removed, including the
      `CamTracker` lazy-load slot.
- [ ] `Odometry` (with its `STALE_AGE`/`is_valid`/`source` contract)
      remains as the one pose-tracking module `sensors/` keeps.
- [ ] `grep -rn "cam_tracker\|odom_tracker\|from .calibration import\|sensors.calibration" src/ --include=*.py`
      returns nothing outside `src/archive`.
- [ ] `python -c "import robot_radio.sensors"` works; full pytest suite
      passes.
- [ ] `src/host/robot_radio/DESIGN.md`'s `sensors/` row is updated in the
      same commit to drop the three deleted modules.

## Testing

- **Existing tests to run**: full `uv run python -m pytest`.
- **New tests to write**: none required for a pure deletion; if any
  existing test imports the deleted modules directly, update it to use
  `Odometry` or remove it.
- **Verification command**: `uv run python -m pytest -q` (full suite),
  plus `python -c "import robot_radio.sensors"`.

## Implementation Notes

- **Approach**: delete the three files and their exports in one commit;
  grep for any stray reference before declaring done.
- **Files to modify**: delete
  `src/host/robot_radio/sensors/calibration.py`,
  `src/host/robot_radio/sensors/cam_tracker.py`,
  `src/host/robot_radio/sensors/odom_tracker.py`; edit
  `src/host/robot_radio/sensors/__init__.py`.
- **Documentation updates**: `src/host/robot_radio/DESIGN.md`'s
  `sensors/` row (this ticket's own acceptance criterion).
