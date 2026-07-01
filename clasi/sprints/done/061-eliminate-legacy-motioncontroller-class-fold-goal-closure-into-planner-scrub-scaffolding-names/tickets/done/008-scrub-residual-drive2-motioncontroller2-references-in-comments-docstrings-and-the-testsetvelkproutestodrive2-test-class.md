---
id: 008
title: Scrub residual Drive2/MotionController2 references in comments, docstrings,
  and the TestSetVelKpRoutesToDrive2 test class
status: done
use-cases: []
depends-on: []
github-issue: ''
issue: internalize-legacy-motioncontroller-into-planner.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Scrub residual Drive2/MotionController2 references in comments, docstrings, and the TestSetVelKpRoutesToDrive2 test class

## Description

Sprint 061 eliminated the legacy `MotionController` class and renamed `drive2`/`mc2`
scaffolding identifiers, but stale references to the old names (`Drive2`,
`MotionController2`, `bvc2`) linger in comments, docstrings, and one real Python
test-class identifier.  This ticket scrubs every remaining occurrence in `.cpp`,
`.h`, `.py`, and `CMakeLists.txt` files so that
`grep -rIn "Drive2\|MotionController2\|bvc2\b" source/ tests/` returns nothing
except the two intentional migration-history bench-checklist `.md` files.

No behavior change.  Comment and docstring rewording only, plus one test-class
rename (`TestSetVelKpRoutesToDrive2` → `TestSetVelKpRoutesToDrive`).

## Acceptance Criteria

- [x] Python test class `TestSetVelKpRoutesToDrive2` renamed to
  `TestSetVelKpRoutesToDrive` in `tests/simulation/unit/test_059_config_routing.py`
  (class name + docstring updated).
- [x] Stale comments in `source/robot/LoopTickOnce.cpp` (lines ~49–163) reworded
  to current names (`Drive`, `Planner`, `bvc`); literal old names removed.
- [x] `source/subsystems/drive/Drive.cpp:1` and `Drive.h:10` provenance comments
  reworded so the literal `Drive2` string is gone.
- [x] Comments/docstrings in `tests/simulation/unit/test_059_bus_drain.py`,
  `tests/simulation/unit/test_planner_subsystem.py`,
  `tests/_infra/sim/planner_api.cpp`, `tests/_infra/sim/sim_api.cpp`,
  `tests/_infra/sim/config_routing_api.cpp`, and
  `tests/_infra/sim/CMakeLists.txt` reworded to current names.
- [x] `grep -rIn "Drive2\|MotionController2\|bvc2\b" source/ tests/` returns only
  lines in the two bench-checklist `.md` files (intentional migration history).
- [x] Full test suite passes (all pass except 2 known baseline failures:
  `test_tovez_validates_against_schema`, `test_default_robot_config_unchanged`).

## Testing

- **Existing tests to run**: full suite — `uv run python -m pytest`
- **New tests to write**: none (comment/docstring + identifier rename only)
- **Verification command**: `uv run python -m pytest`
