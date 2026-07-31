---
status: in-progress
sprint: '128'
tickets:
- 128-014
---

# Decide the firm↔motion boundary: promote RobotState wheel targets, delete the MoveQueue generation, fix the docs

**Source:** code review 2026-07-30, `01-firm.md` MAJOR §2, §3;
`02-motion.md` MAJOR §1, §5. One root cause, four findings: the
`Motion::Planner` cutover never walked back to the boundary contract or the
design docs.
**Priority:** P1 — nothing is broken at runtime, but the documented
architecture is false, and ~1,500 lines of dead-but-compiled motion code
(`MoveQueue`/`WheelSink`/`StopCondition`/`VelocityShaper`) sit exactly where
the next bug hunt will look first.
**Goal served:** a reader of `CLAUDE.md`, `docs/design/design.md` §5, or
`src/motion/DESIGN.md` today builds a completely wrong model of how the robot
decides motion. Wrong maps are worse than no maps when hunting bugs.

## The decision (recommended: option a)

**(a) Promote `Types::RobotState::Wheel::cmdVelocity` to be the documented
actuation boundary** — it already IS the real one: `Motion::Planner::update()`
writes it (`planner.cpp:1230-1233`), `RobotLoop::cycle()` consumes it
(`robot_loop.cpp:499`), and `Motion::WheelSink` is called by nothing
(`drive.h:125-132`'s own comment says so). Then delete the dead generation.

**(b)** Restore `WheelSink` as the seam and route `Planner` through it —
only if there is a concrete reason to keep an interface object between two
trees that already share `RobotState`. Absent that reason, (a) is less code
and matches reality.

## What to do (under option a)

1. **Delete, decisively:** `src/motion/wheel_sink.h`, `move_queue.{h,cpp}`,
   `stop_condition.{h,cpp}`, `velocity_shaper.{h,cpp}`, `App::Drive`'s
   `WheelSink` overrides (`drive.h:125-132`, `drive.cpp:31-36`), the
   `#include "motion/move_queue.h"` in `main.cpp:29`, their `motion_tests`
   targets and `test_app_move_queue.py`. Remember the four build source
   lists (CMake ×2 + pytest `_APP_SOURCES`) or this fails at link, not
   compile.
2. **Preserve the history, not the code:** `move_queue.cpp`'s 250-line
   land-at-zero margin derivation moves to a dated design-history doc (e.g.
   `docs/design/history/land-at-zero-margin-derivation.md`) before deletion.
3. **Document the boundary where it lives** — on the field itself:

```cpp
// --- Wheel command targets: THE base<->motion actuation boundary ---
// writer: exactly ONE of Motion::Planner::update() (when a Move owns
//   motion) or App::Drive::update() (WHEELS teleop) -- ownership arbitrated
//   by RobotLoop's ordering (robot_loop.cpp "ordering" comment).
// consumer: RobotLoop::cycle() -> drive_.tick(cmdVelocity, ...).
// Motion::WheelSink -- the interface that used to be this seam -- was
// deleted 2026-XX-XX (craftsmanship review 01 SS2): it had no callers.
float cmdVelocity;  // [mm/s]
```

4. **Rewrite the three design docs to match the code:**
   `docs/design/design.md` §5 (delete the `MoveQueue`/`setDuty()` story,
   describe `Motion::Planner` + the `RobotState` boundary),
   `src/firm/app/DESIGN.md` (Drive's real second lifecycle:
   `command()`/`estop()`/`owns()`/`takeCompletion()`/`update()`), and
   `src/motion/DESIGN.md` (add `src/motion/planner/` — the larger, live
   half: its own CMake project, ctypes harness, 9 ctest executables — or
   give it its own `DESIGN.md`). Fix `CLAUDE.md`'s stale "the velocity PID
   is part of the frozen base" claim in the same pass.
5. `StopCondition`/`VelocityShaper` are clean code — if anyone wants to keep
   one warm for a future cutover, that intent must be WRITTEN in DESIGN.md
   with a named owner, else they go. Default is delete.

## Acceptance

- `grep -rn "WheelSink\|MoveQueue\|move_queue\|stop_condition\|velocity_shaper" src/firm src/motion` returns only the design-history doc reference.
- Full clean build (`just build-clean`) + `motion_tests` + planner ctest
  suite + firmware pytest tiers pass.
- `docs/design/design.md` §5, `src/firm/app/DESIGN.md`, and
  `src/motion/DESIGN.md` each describe the planner-era architecture; no doc
  names `MoveQueue` as current.
- Deploy and run the standing bench smoke (`twist_drive.py`) — behavior
  unchanged (the deleted code was unwired, so any behavior change means the
  deletion cut something live: stop and investigate).
