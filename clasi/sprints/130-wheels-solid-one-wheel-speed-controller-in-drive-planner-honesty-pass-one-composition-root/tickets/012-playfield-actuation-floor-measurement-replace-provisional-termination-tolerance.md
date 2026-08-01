---
id: '012'
title: Playfield actuation-floor measurement; replace provisional TERMINATION_TOLERANCE
status: open
use-cases: [SUC-006]
depends-on: ['006']
github-issue: ''
issue: measure-actuation-floor-and-set-termination-tolerance.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Playfield actuation-floor measurement; replace provisional TERMINATION_TOLERANCE

## Description

Per `measure-actuation-floor-and-set-termination-tolerance.md`: run the
camera-fenced `src/tests/bench/square_tour.py --mode actuation-floor`
sweep of decreasing commanded distances/angles to find the smallest the
drivetrain reliably executes — now that the new controller's speed-
floor policy (ticket 004's Open Question 2) is implemented and bench-
proven (ticket 006), so the measured floor reflects the system this
sprint ships, not the old open-loop path. This cannot be measured on
the stand — unloaded wheels have no friction/robot weight/drivetrain
load, so the smallest command that produces motion there is
optimistically low and wrong for the field.

Replace `src/host/robot_radio/pathplan/planner.py:108`'s
`TERMINATION_TOLERANCE = 100.0  # PROVISIONAL -- pending ticket 007`
with the measured value and its provenance; re-run the goto-mode
playfield convergence gate against it. Note the tolerance is bounded on
BOTH sides for path following (`path-following-hardware-gaps.md`): too
small and the outer loop cannot resolve arrival; too large and it
"arrives" at waypoints it never approached.

## Acceptance Criteria

- [ ] The sweep's raw data recorded, not just the chosen number.
- [ ] `planner.py:108` carries a measured value with its provenance;
      the `# PROVISIONAL` marker removed.
- [ ] The goto-mode playfield gate re-run and passing against the new
      value.
- [ ] The bounded-both-sides concern from `path-following-hardware-
      gaps.md` explicitly acknowledged in the provenance comment (too
      small = never resolves arrival; too large = false arrival).

## Testing

- **Existing tests to run**: the goto-mode playfield convergence gate
  (pre-change, to confirm today's provisional-value failure mode:
  corner 1 arriving at 96.8 mm against a 250 mm target).
- **New tests to write**: none beyond the sweep script's own scoring
  logic, if not already present in `square_tour.py --mode
  actuation-floor`.
- **Verification command**: camera-fenced playfield run per
  `.claude/rules/playfield-testing.md`; `uv run pytest` for the
  goto-mode gate re-run.

## Implementation Plan

**Approach**: playfield/camera-fenced measurement per
`.claude/rules/playfield-testing.md` (geofenced, room lights checked
first, camera pose captured at rest) — no new production code beyond
the one constant + provenance comment + `PROVISIONAL` marker removal.

**Files to create/modify**:
- `src/host/robot_radio/pathplan/planner.py` (`TERMINATION_TOLERANCE`
  + provenance comment)
- `src/tests/bench/square_tour.py` (`--mode actuation-floor` already
  exists — use as-is unless the sweep reveals a gap)

**Testing plan**: the camera-fenced sweep itself; the goto-mode
playfield convergence gate re-run against the new value.

**Documentation updates**: provenance comment at `planner.py:108`
recording the measured value's source and date; cross-reference to
`path-following-hardware-gaps.md`'s bounded-tolerance concern.
