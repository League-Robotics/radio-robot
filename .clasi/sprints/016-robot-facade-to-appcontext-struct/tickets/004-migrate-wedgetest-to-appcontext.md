---
id: '004'
title: Migrate WedgeTest to AppContext
status: open
use-cases:
  - SUC-006
depends-on:
  - '003'
github-issue: ''
issue: ''
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Migrate WedgeTest to AppContext

## Description

Migrate `WedgeTest.h` and `WedgeTest.cpp` from `Robot*` to `AppContext*`.
This is a small change (~5 sites) and can be done before `CommandProcessor`
because `WedgeTest` is invoked through `CommandProcessor` (`DBG WEDGE` verb),
but the `DBG WEDGE` dispatch still uses `_sched->robot()` to get the robot
reference, which is now an `AppContext&` after T003.

After T003, `_sched->robot()` returns `AppContext&`. The `DBG WEDGE` dispatch
in `CommandProcessor.cpp` calls `runWedgeTest(..., &_sched->robot())`. Since
`_sched->robot()` is now `AppContext&`, the `runWedgeTest` function signature
must accept `AppContext*` instead of `Robot*`. If `CommandProcessor` still
uses `Robot&` at this point (it does — T005 migrates it), the `DBG WEDGE` call
site in `CommandProcessor.cpp` must be updated to cast or take the reference
from the scheduler rather than from `_robot`.

**Resolution**: In `CommandProcessor.cpp`, the `DBG WEDGE` dispatch currently
calls `runWedgeTest(..., &_sched->robot())`. After T003, `_sched->robot()`
returns `AppContext&`. Update the `WedgeTest` signature to `AppContext*` and
update the single call site in `CommandProcessor.cpp` from
`&_sched->robot()` (type `AppContext&`) accordingly. This is the one
`CommandProcessor.cpp` site that must be touched in this ticket (not the full
`CommandProcessor` migration, just this one call).

### WedgeTest.h changes

```cpp
// Old:
class Robot;
void runWedgeTest(MicroBit& uBit, ..., Robot* robot = nullptr);

// New:
struct AppContext;
void runWedgeTest(MicroBit& uBit, ..., AppContext* robot = nullptr);
```

### WedgeTest.cpp changes

1. Change `#include "Robot.h"` to `#include "AppContext.h"`.
2. Change the function signature parameter from `Robot* robot` to
   `AppContext* robot`.
3. In the `useReal` path inside `runWedgeTest`, locate every use of
   `robot->` and apply substitutions. The known sites from reading the source:
   - `robot->motor()` where `MotorController` methods are accessed →
     `robot->motorController`
   - Verify by reading WedgeTest.cpp lines 150+ for all `robot->` dereferences.
   - Typical pattern: `robot->motor().setTarget(...)` →
     `robot->motorController.setTarget(...)`

### CommandProcessor.cpp change (targeted — DBG WEDGE only)

In the `DBG WEDGE` dispatch block, the call:
```cpp
runWedgeTest(_sched->uBit(), wrate, wwrite, wbus, wdith, wreg, wsens,
             wreal, &_sched->robot());
```
After T003, `_sched->robot()` returns `AppContext&`, so `&_sched->robot()`
is `AppContext*`. With the WedgeTest signature now accepting `AppContext*`, this
call compiles correctly without further changes. Verify this compiles; no
additional changes to CommandProcessor.cpp should be needed.

## Acceptance Criteria

- [ ] `WedgeTest.h` uses `struct AppContext;` forward declaration and
      `AppContext* robot = nullptr` parameter.
- [ ] `WedgeTest.cpp` includes `AppContext.h` (not `Robot.h`).
- [ ] All `robot->` dereferences inside `runWedgeTest` compile against
      `AppContext` (direct member access, no accessor methods).
- [ ] The `DBG WEDGE` dispatch in `CommandProcessor.cpp` compiles correctly
      with the updated signature (no type mismatch).
- [ ] Clean build: `python3 build.py` passes.
- [ ] Host unit tests pass: `uv run --with pytest python -m pytest`.

## Implementation Plan

**Approach**: Type swap in WedgeTest; verify the one call site in
CommandProcessor compiles without changes.

**Files to modify**:
- `source/app/WedgeTest.h` — forward decl + parameter type swap
- `source/app/WedgeTest.cpp` — include swap + `robot->` substitutions
- `source/app/CommandProcessor.cpp` — verify DBG WEDGE call site compiles;
  edit only if a type mismatch requires it (should not be needed)

**Files NOT to touch**: `Robot.h`, `Robot.cpp`, `AppContext.h/.cpp`
(already correct), `LoopScheduler.h/.cpp` (already migrated in T003).

**Testing plan**:
- `python3 build.py` — clean build.
- `uv run --with pytest python -m pytest` — no regressions.
- Optional bench: `DBG WEDGE 50 40 100 3` — confirm WedgeTest starts and
  reports encoder readings (robot is on stand, safe to drive).

**Documentation updates**: None required.
