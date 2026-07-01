---
id: '001'
title: Expose MotionController API on Planner via delegation (additive)
status: done
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: internalize-legacy-motioncontroller-into-planner.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 001 — Expose MotionController API on Planner via delegation (additive)

## Description

`Planner` currently exposes only the 4-verb message contract (`apply`, `tick`,
`state`, `configure`) plus `syncWireContext`. Call sites that need goal-start,
mode-query, or safety methods reach `robot.motionController` directly. This
ticket adds all the missing public methods to `Planner`, implemented by
delegating to the wrapped `_mc` (the `MotionController&` it holds).

This is purely additive: no existing call sites are changed. After this ticket
`Planner` exposes the full API that subsequent rerouting tickets need, and all
call sites continue to work exactly as before (against `motionController`
directly). The build and test suite must remain green.

Methods to add on `Planner` (all delegating to `_mc`):

- `mode() const` — delegates to `_mc.mode()`
- `beginStream(leftMms, rightMms, now_ms, target, fn, ctx)`
- `beginVelocity(v_mms, omega_rads, now_ms, target, fn, ctx, corr_id, seedImmediate)`
- `beginTimed(leftMms, rightMms, durationMs, now_ms, target, fn, ctx, corr_id)`
- `beginDistance(leftMms, rightMms, targetMm, now_ms, target, fn, ctx, corr_id)`
- `beginGoTo(tx, ty, speedMms, now_ms, target, fn, ctx, corr_id)`
- `beginTurn(headingCdeg, epsCdeg, now_ms, target, fn, ctx, corr_id)`
- `beginRotation(relCdeg, now_ms, target, fn, ctx, corr_id)`
- `stop(now_ms, fn, ctx)`
- `cancel(now_ms, fn, ctx)`
- `softStop(now_ms)`
- `beginRawVelocity(v_mms, omega_rads)`
- `disableSafetyOneShot()`
- `hasActiveCommand() const`
- `emitToActiveChannel(evt, target)`
- `activeCmd()` — returns `MotionCommand&` via `_mc.activeCmd()`
- `setHardwareState(HardwareState* s)` — delegates to `_mc.setHardwareState(s)`
- `setRobotCtx(Robot* r)` — delegates to `_mc.setRobotCtx(r)`
- `setBvcStateRef(DesiredState* ds)` — delegates to `_mc.setBvcStateRef(ds)`
- `hardwareState() const` — delegates to `_mc.hardwareState()`

All signatures must exactly match the corresponding `MotionController` methods
so that later tickets can change a call site's receiver from `motionController`
to `planner` without altering any argument lists.

## Acceptance Criteria

- [x] All methods listed above are declared in `source/superstructure/Planner.h`
      and defined (inline or in `.cpp`) with delegation to `_mc`.
- [x] Each method delegates to `_mc.<same-method>(same-args)` without any
      logic change.
- [x] `source/superstructure/Planner.h` still includes `"MotionController.h"`
      (no include changes in this ticket).
- [x] `cmake --build build_sim` succeeds with zero errors.
- [x] `uv run python -m pytest tests/simulation/unit/test_golden_tlm.py
      tests/simulation/unit/test_059_ordered_tick_parity.py
      tests/simulation/unit/test_planner_subsystem.py` all pass.
- [x] No call site outside `Planner.h/.cpp` is touched.

## Implementation Plan

### Approach

Edit `source/superstructure/Planner.h` to add the new public method
declarations in a clearly-delimited block (comment: `// MotionController
compatibility API — delegated to _mc (ticket 061-001); becomes native in 061-004`).
Implement trivial one-liners inline in the header; add non-trivial forwarding
bodies to `Planner.cpp`.

### Files to modify

- `source/superstructure/Planner.h` — add public method declarations
- `source/superstructure/Planner.cpp` — add any non-trivial delegation bodies

### Files NOT to touch

- All call sites (`Robot.cpp`, `RobotTelemetry.cpp`, `SystemCommands.cpp`,
  `MotionCommands.h`, `Superstructure.h/.cpp`, `LoopScheduler.h`, tests)

### Testing plan

After changes:
```
cmake --build build_sim && uv run python -m pytest \
  tests/simulation/unit/test_golden_tlm.py \
  tests/simulation/unit/test_059_ordered_tick_parity.py \
  tests/simulation/unit/test_planner_subsystem.py
```

### Documentation updates

Add a comment block above the delegation methods in `Planner.h` explaining
they are the compatibility surface for 061 call-site rerouting.
