---
id: '001'
title: 'Extend GoalRequest: stops[], streamSeed, doneLabel'
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
depends-on: []
issue: stop-conditions-as-a-first-class-system-primitive.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 053-001: Extend GoalRequest: stops[], streamSeed, doneLabel

## Description

Add four fields to `GoalRequest` in `source/superstructure/Superstructure.h`
that will carry per-command stop conditions, the stream-seed flag, and the
wire-label string from verb handlers to `Superstructure::requestGoal`. These
fields are the foundation that tickets 003–005 rely on to eliminate the
stringify/re-parse round-trip and migrate S onto MotionCommand.

This ticket is purely additive: existing callers use aggregate initialization
(`GoalRequest gr{};`) which zero-initializes all new fields, so no callsite
changes are required this ticket.

## Acceptance Criteria

- [x] `GoalRequest` in `Superstructure.h` contains exactly these new fields
  (after the existing `radiusMm` field):
  ```cpp
  StopCondition stops[4];   // stop conditions to apply after begin
  uint8_t       nStops;     // number of valid entries in stops[]
  bool          streamSeed; // true → seed BVC immediately (S-command semantics)
  const char*   doneLabel;  // EVT label for setDoneEvt; nullptr = use default
  ```
- [x] `StopCondition.h` is included in `Superstructure.h` (it may already be
  transitively included; verify and add a direct include if needed).
- [x] `Superstructure::requestGoal` in `Superstructure.cpp` reads `gr.nStops`
  and `gr.stops[]` and calls `mc.addStop(gr.stops[i])` for `i` in
  `0..nStops-1` after each begin call that uses an active MotionCommand
  (VELOCITY and DISTANCE cases). Reads `gr.doneLabel` and calls
  `_mc.activeCmd().setDoneEvt(gr.doneLabel)` when `gr.doneLabel != nullptr`
  and an active command exists. `gr.streamSeed` is stored but not yet acted
  upon (used by ticket 003).
- [x] All existing callers of `requestGoal` compile without modification
  (verified by `uv run --with pytest python -m pytest tests/simulation -q`
  passing with exactly 2 known failures).
- [ ] `python build.py --clean` exits 0 (ARM build).

## Implementation Plan

### Approach

Purely additive header change + requestGoal body extension. No callsite edits
needed this ticket because aggregate zero-init handles the new fields.

### Files to Modify

- `source/superstructure/Superstructure.h`
  - Add `#include "StopCondition.h"` if not already present (check current
    includes; `StopCondition.h` is in `source/control/`).
  - Add the four new fields to `GoalRequest` after `radiusMm`.

- `source/superstructure/Superstructure.cpp`
  - In `requestGoal`, the VELOCITY case (currently `beginVelocity`): after the
    `_mc.beginVelocity(...)` call, add:
    ```cpp
    if (gr.doneLabel) _mc.activeCmd().setDoneEvt(gr.doneLabel);
    for (uint8_t i = 0; i < gr.nStops; ++i) _mc.activeCmd().addStop(gr.stops[i]);
    ```
  - In the DISTANCE case (currently `robot->distanceDrive(...)`): after the
    call, add the same two blocks (activeCmd() is active after distanceDrive
    sets up the MotionCommand via beginDistance).
  - In the ARC case (currently `beginArc`): same pattern.
  - In TIMED case (currently `beginTimed`): same pattern.
  - STREAM case: leave unchanged for now (ticket 003 migrates it).
  - Note: GOTO, TURN, ROTATE do not use the new fields (closed-loop; keep
    as-is).

### Testing Plan

- Run `uv run --with pytest python -m pytest tests/simulation -q`.
  Expect exactly 2 failures (the two known pre-existing ones). No new failures.
- `python build.py --clean` exits 0.
- Manual inspection: confirm `GoalRequest` struct size change does not cause
  stack overflow concerns (4 StopConditions + 1 byte + 1 bool + 1 pointer =
  ~50 bytes on ARM; GoalRequest is stack-local in each handler; safe on M4).

### Documentation Updates

None required this ticket.
