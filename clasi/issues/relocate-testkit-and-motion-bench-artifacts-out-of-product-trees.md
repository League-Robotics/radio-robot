---
status: pending
---

# Relocate testkit/ and src/motion/planner's bench scripts + measurement captures out of the product trees

**Source:** code review 2026-07-30, `05-testgui-testkit.md` MAJOR §6;
`02-motion.md` MINOR §6.
**Priority:** P2 — no runtime effect, but both violate the project's own
"test code belongs in src/tests/" rule, and the committed multi-MB captures
inflate the library tree indefinitely.
**Goal served:** a reader auditing the importable product surface should
never have to decide whether a SIGINT-handling `SafeRun` harness or a 2.4 MB
CSV is load-bearing.

## What is wrong

- `src/host/robot_radio/testkit/` (`make_target`/`TestRobot`, `SafeRun`/
  `BenchRun`, `Dashboard`, `PoseSource`) is bench harness code inside the
  importable host package. Non-archived consumers: exactly one —
  `testgui/transport.py:1285` imports `read_camera_pose` from
  `testkit/camera.py`. Everything else is referenced only from
  `src/archive/tests_old/`.
- `src/motion/planner/bench/` (6 scripts, 2,229 lines) and
  `src/motion/planner/py/planner_harness.py` sit inside the C++ library
  tree, alongside ~9 MB of committed measurement output
  (`square_tour_velocity_*.csv` ×4, four PNGs).

## What to do

1. `testkit/`: move `read_camera_pose` into `testgui/transport.py` (its one
   real caller) or a small `testgui/camera_pose.py`; move the rest to
   `src/tests/tools/` **or delete it** — zero live callers means deletion is
   the honest default; confirm with the stakeholder before keeping.
2. `src/motion/planner/bench/` + `py/` → `src/tests/bench/` (join
   `move_protocol_bench.py` etc.), fixing their imports/paths.
3. The committed CSV/PNG captures: move to a bench-results location outside
   the source tree, or delete and regenerate on demand (they are output, not
   source). If history matters, note the generating script + date in the
   bench README instead.

## Acceptance

- `src/host/robot_radio/testkit/` no longer exists (or contains only what a
  stakeholder explicitly kept, with a stated reason).
- `git ls-files src/motion/planner | grep -E "\.py$|\.csv$|\.png$"` returns
  nothing.
- testgui still imports its camera-pose helper; relocated bench scripts run
  from their new home (spot-check one HIL script against the stand and one
  sim script).
