---
id: '005'
title: Delete MotionController source files; fix all includes; verify grep clean
status: done
use-cases:
- SUC-005
depends-on:
- '004'
github-issue: ''
issue: internalize-legacy-motioncontroller-into-planner.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 005 — Delete MotionController source files; fix all includes; verify grep clean

## Description

After ticket 004, `MotionController.h/.cpp` and `MotionControllerBegin.cpp`
are no longer compiled as part of the absorb — but they still exist on disk and
may still be `#include`d in some files transitively. This ticket deletes all
three legacy files, removes every remaining `#include "superstructure/MotionController.h"`
directive in `source/`, updates `CMakeLists.txt` to drop the old TU, and
verifies that `grep -rIn "MotionController\b" source/` produces no meaningful
hits (only provenance comments, which must be reworded or removed).

### Changes

1. **Delete legacy files** (git rm):
   - `source/superstructure/MotionController.h`
   - `source/superstructure/MotionController.cpp`
   - `source/control/MotionControllerBegin.cpp`

2. **Update `CMakeLists.txt`** (firmware and/or sim targets):
   - Remove `MotionController.cpp` and `MotionControllerBegin.cpp` from
     source file lists.
   - Confirm `PlannerBegin.cpp` is already present (added in ticket 004).

3. **Audit and fix remaining `#include "superstructure/MotionController.h"`**:
   The following files may still include it directly or transitively:
   - `source/superstructure/Planner.h` (should have been removed in 004 — verify)
   - `source/superstructure/Superstructure.cpp` (may have included it)
   - `source/robot/Robot.cpp` (may still include it)
   - `source/commands/MotionCommands.cpp`
   - `source/commands/SystemCommands.cpp`
   - `tests/_infra/sim/planner_api.cpp` (removed in 004 — verify)
   Run a grep to find any remaining: `grep -rn "MotionController.h" source/ tests/`

4. **Update / remove provenance comments** in remaining source files:
   Comments like `// renamed from MotionController...` or references to
   `MotionController` class by name in comments should be reworded or removed.
   The target: `grep -rIn "MotionController\b" source/` returns zero hits.

5. **Update doc file `source/COMMANDS.md`** to replace references to
   `MotionController` with `Planner`.

6. **Update `source/commands/MotionCommand.h/.cpp` comments** that reference
   `MotionController::...` to `Planner::...`.

7. **Verify `source/control/MotionEventSink.h`** does not pull in
   `MotionController.h` in its include chain.

## Acceptance Criteria

- [x] `source/superstructure/MotionController.h` does not exist.
- [x] `source/superstructure/MotionController.cpp` does not exist.
- [x] `source/control/MotionControllerBegin.cpp` does not exist.
- [x] `grep -rn "MotionController.h" source/ tests/` returns zero hits.
- [x] `grep -rIn "MotionController\b" source/` returns zero hits (after
      comment updates; pure comment hits are acceptable only if they describe
      provenance in past-tense — programmer judgment on each hit).
- [x] `cmake --build build_sim` succeeds with zero errors.
- [x] `uv run python -m pytest tests/simulation/unit/test_golden_tlm.py
      tests/simulation/unit/test_059_ordered_tick_parity.py
      tests/simulation/unit/test_planner_subsystem.py` all pass.

## Implementation Plan

### Approach

1. Run `grep -rn "MotionController.h" source/ tests/` to get the full include
   list before deleting.
2. Delete the three files.
3. Try to build — the compiler will report every remaining missing include.
4. Fix each include error.
5. Re-run grep for `MotionController\b` and clean up comments.
6. Final build + tests.

### Files to modify

- CMakeLists.txt (remove old TU entries)
- Any files that include `"superstructure/MotionController.h"` directly
- `source/COMMANDS.md`
- `source/commands/MotionCommand.h/.cpp` (comment updates)

### Testing plan

```
cmake --build build_sim && uv run python -m pytest \
  tests/simulation/unit/test_golden_tlm.py \
  tests/simulation/unit/test_059_ordered_tick_parity.py \
  tests/simulation/unit/test_planner_subsystem.py
```

### Documentation updates

- `source/COMMANDS.md`: replace `MotionController` with `Planner`.
- `source/commands/MotionCommand.h/.cpp`: update method name references
  in comments.
