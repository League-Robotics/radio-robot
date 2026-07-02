---
status: pending
review: docs/code_review/2026-07-01-full-codebase-review.md
findings: CR-01
severity: critical
sprint: '065'
---

# Stop-clause overflow aborts the process: D/T with ≥2 stop=/sensor= clauses hits assert(false)

## Problem

`MotionCommand::kMaxStopConds` is 4 and `addStop()` handles overflow with
`assert(false && "addStop overflow")`
([MotionCommand.cpp:53-61](../../source/commands/MotionCommand.cpp)).
The D command path double-books stops:

1. `beginDistance()` installs DISTANCE + TIME internally (2 stops)
   ([PlannerBegin.cpp:292-295](../../source/control/PlannerBegin.cpp)).
2. `Superstructure::requestGoal(DISTANCE)` then re-adds `gr.stops[]`
   ([Superstructure.cpp:51-61](../../source/superstructure/Superstructure.cpp)),
   which starts with a **duplicate** DISTANCE stop
   ([MotionCommands.cpp:759](../../source/commands/MotionCommands.cpp)) plus any
   `stop=` / `sensor=` clauses from the wire command.

Totals: plain `D` = 3 (including one wasted duplicate); `D … stop=… sensor=…`
(2 clauses) = 5 → the assert fires. The T path is analogous (internal TIME +
duplicate TIME + clauses).

The sim build (`tests/_infra/sim/CMakeLists.txt`) sets no
`CMAKE_BUILD_TYPE`/`NDEBUG`, so the assert is live: it **aborts the whole
Python process** hosting the sim (pytest run or the TestGUI). This is the
strongest identified candidate for "the simulation is crashing." On real
firmware the CODAL assert path panics the micro:bit mid-drive.

## Fix direction

- Stop the double-add: either `requestGoal` must not re-add the goal's own
  primary stop, or `begin*()` must not install stops the caller will supply.
- Make `addStop` overflow a recoverable error (reply `ERR` to the host)
  instead of asserting.
- Regression test: `D 150 150 300 stop=time:9000 sensor=line0>500` must not
  crash the sim and must honor the clauses.

## Acceptance

- The command above runs in sim without process abort and stops on the
  earliest-firing condition.
- No duplicate stop conditions on plain `D`/`T`.
- Overflow (if still reachable) produces a wire-visible ERR, never an assert.
