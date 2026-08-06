---
id: '007'
title: 'Host shrink: pathplan gotoWorld/gotoRobot/followPath become thin GO_TO senders'
status: open
use-cases:
- SUC-002
depends-on:
- '004'
github-issue: ''
issue: replaceable-go-to-moves-in-the-motion-library.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Host shrink: pathplan gotoWorld/gotoRobot/followPath become thin GO_TO senders

## Sequencing — this is the sprint's ONE breaking change, run it last

This ticket depends technically only on ticket 004 (the wire arm existing
and working), but it should be executed LAST among this sprint's tickets
in practice — after ticket 005 (sim tests) and ticket 006 (hardware smoke)
have both proven `GO_TO` solid end-to-end. Deleting the host-side loop
before `GO_TO` is provably working would leave no way to drive a goto at
all if something regresses. Per this sprint's own "breaking changes go
last" sequencing rule (sprint.md Migration Concerns), do not start this
ticket until 005 and 006 are done, even though the dependency graph alone
would permit it right after 004.

## Description

Once `GO_TO` exists and works (tickets 004-006), the host's own
arc-solving and replace-throttling logic in
`src/host/robot_radio/pathplan/` becomes dead weight: `gotoWorld`/
`gotoRobot` (`planner.py:678`/`879`) shrink to thin `GO_TO` senders, and
`followPath` (`planner.py:993`) keeps `pursuitTarget()` (waypoint
lookahead — firmware doesn't do this, it stays host-side) but streams
`GO_TO` targets instead of driving arcs itself. `solveArcToPoint`
(`solver.py:256`), `ReplaceThreshold` (`planner.py:321`), and the
curvature-slew-clamp logic (`solver.py:237`, `_clampOmegaStep`) all
become dead code and are deleted.

**Keep**: `pursuitTarget()` (`solver.py:523`, waypoint lookahead — solves
a problem firmware-internal goto does not, "which point on the path is
next"), `MoveIdAllocator` (`planner.py:464`), and the ack-verified
retry/dedup machinery (`AckRetry`, `_recordAcks`, `_readFrames`,
`GiveUpLimits`) — link loss is real regardless of where arc-solving runs,
and ticket 001's measurement sizes this retry policy honestly rather than
against folklore.

## Callers to check before deleting anything — verified, not assumed

The linked issue names only `gotoWorld`/`gotoRobot`/`followPath` as
callers of the doomed functions. Grepping the actual repo at
sprint-planning time (2026-08-06) found MORE callers of
`robot_radio.pathplan` than the issue lists — **all of these must be
updated or explicitly retired, not silently left broken**:

```
src/tests/bench/square_tour.py
src/tests/unit/test_world_pose.py
src/tests/unit/test_planner.py
src/tests/unit/test_solver.py
src/tests/sim/test_pathplan_goto_convergence.py
```

Re-run this grep at ticket time — the list above is a floor, not a
ceiling:

```
grep -rln "from robot_radio.pathplan\|import pathplan\|pathplan\.planner\|pathplan import" --include="*.py" src/
```

At minimum: `test_solver.py` and `test_planner.py` almost certainly unit-
test `solveArcToPoint`/`ReplaceThreshold` directly — these tests must be
DELETED alongside the functions they test (not left red), and their
parity-testing value is already captured by ticket 002's C++ arc-solver
ctest, which was built BY PORTING this same `solver.py` surface — confirm
ticket 002's ctest actually covers what these Python tests covered before
deleting the Python originals, so no test coverage is silently lost in
the port, only relocated. `square_tour.py` and
`test_pathplan_goto_convergence.py` likely call `gotoWorld`/`gotoRobot`/
`followPath` themselves and should keep working once those functions are
reshaped to thin `GO_TO` senders — verify each one still passes/runs
after the shrink rather than assuming the reshape is behavior-preserving
for its callers.

## Acceptance Criteria

- [ ] `gotoWorld`/`gotoRobot` (`planner.py`) send `GO_TO` commands
      directly (frame WORLD/ROBOT respectively) instead of running their
      own solve-and-replace loop; ack-verified retry against the
      accepted-id ring is KEPT.
- [ ] `followPath` keeps `pursuitTarget()`'s own lookahead-point picking
      unchanged, and streams each picked point as a `GO_TO` instead of
      calling `_sendVerifiedTwist`'s own arc-driving logic.
- [ ] `solveArcToPoint`, `ReplaceThreshold`, and the curvature-slew-clamp
      logic (`_clampOmegaStep`) are DELETED, not just unreferenced —
      confirm with `grep -rn "solveArcToPoint\|ReplaceThreshold\|_clampOmegaStep" src/`
      returning nothing outside version-control history.
- [ ] `pursuitTarget()`, `MoveIdAllocator`, and the ack-verified
      retry/dedup machinery (`AckRetry`/`GiveUpLimits`/`_recordAcks`/
      `_readFrames`) are UNCHANGED and still exercised.
- [ ] Every caller found by the grep above (the five files listed, plus
      whatever a fresh re-run of that grep finds at ticket time) is
      checked: either still passes/runs against the reshaped functions,
      or is explicitly deleted/updated with a stated reason in Completion
      Notes — none is left silently broken.
- [ ] `test_solver.py`/`test_planner.py` coverage of the deleted
      functions is confirmed to already exist in ticket 002's C++
      arc-solver ctest before the Python tests are deleted (a coverage
      relocation, not a coverage loss) — state this confirmation
      explicitly in Completion Notes.
- [ ] `clasi/issues/later/otos-sampled-only-at-rest-not-integrated-
      during-motion.md` is marked superseded (sprint.md Scope's own
      instruction, issue's Open Point 2) once this ticket lands and the
      sprint's shipped behavior supersedes it in practice.

## Testing

- **Existing tests to run**: `pyproject.toml`'s `testpaths` is
  `["src/tests/sim", "src/tests/unit", "src/tests/testgui"]` (verified
  directly, not assumed) — a bare, repo-root `uv run python -m pytest`
  already collects ALL FIVE caller files from the grep above
  (`test_world_pose.py`/`test_planner.py`/`test_solver.py` live under
  `src/tests/unit`, `test_pathplan_goto_convergence.py` under
  `src/tests/sim`; `square_tour.py` is a bench script, not
  pytest-collected — exercise it manually or via whatever smoke path it
  already has). No narrower `--rootdir`/path is needed or more correct
  than the plain invocation:
  ```
  uv run python -m pytest
  ```
  plus an explicit, verbose run of whichever of the four pytest-collected
  caller files survive the shrink (e.g. `uv run python -m pytest src/tests/sim/test_pathplan_goto_convergence.py -v`).
- **New tests to write**: none required beyond what tickets 002/005
  already added — this ticket is a deletion/reshape, not new behavior.
- **Verification command**:
  ```
  uv run python -m pytest
  grep -rn "solveArcToPoint\|ReplaceThreshold\|_clampOmegaStep" src/
  ```
  (the grep should return nothing).
