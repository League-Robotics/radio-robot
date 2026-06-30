---
id: '003'
title: 'Close gap 3: unify sensor lag schedule under sensors.tick()'
status: open
use-cases:
- SUC-003
depends-on:
- '002'
github-issue: ''
issue: make-ordered-tick-the-default-close-parity-gaps.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Close gap 3: unify sensor lag schedule under sensors.tick()

## Description

Parity gap #3: the ordered-tick path calls `robot.sensors.tick(now)` (step 7), which
drives line and color reads through the `Sensors` facade's own lag timers
(`_lastLineTick`, `_lastColorTick`). However, the `LoopTickOnce.cpp` comment block
(lines 257-263) notes that `lineSensor.periodic` / `colorSensor_.periodic` are
"still called for ports (Ports is not yet wrapped) and to keep LoopTickState.lastLine
/ lastColor in sync." In practice, the ordered-tick path in sprint 059 does NOT call
`lineSensor.periodic` or `colorSensor_.periodic` (only `ports.periodic` is called).
Verify this by reading the current ordered-tick branch.

If `lineSensor.periodic` and `colorSensor_.periodic` are already absent from the
ordered-tick branch, this ticket's work is:
- Confirm in the code that they are absent.
- Add a comment documenting that `sensors.tick()` is the sole schedule authority.
- Confirm `LoopTickState.lastLine`/`.lastColor` are NOT read or written in the
  ordered-tick path.

If they are present (i.e., the comment was written prospectively), remove them.

`ports.periodic(ts, now)` STAYS — this is documented as remaining scaffolding (no
`Ports2` facade). Do not remove it.

After this ticket: the sensor lag schedule is entirely owned by `sensors.tick()`.
The `Sensors` facade's `_lastLineTick`/`_lastColorTick` are the authoritative timers.
`LoopTickState.lastLine`/`.lastColor` are not used in the ordered-tick path.

## Acceptance Criteria

- [ ] `robot.lineSensor.periodic(ts, now)` is NOT called in the ordered-tick branch of `LoopTickOnce.cpp`.
- [ ] `robot.colorSensor_.periodic(ts, now)` is NOT called in the ordered-tick branch.
- [ ] `robot.ports.periodic(ts, now)` IS still called in the ordered-tick branch.
- [ ] A comment in `LoopTickOnce.cpp` documents that `sensors.tick()` is the sole sensor schedule authority.
- [ ] `uv run python -m pytest` — green except the 2 known-baseline failures.
- [ ] `test_golden_tlm.py` remains green.

## Implementation Plan

### Approach

Primarily a verification + documentation task. Read the current ordered-tick branch
of `LoopTickOnce.cpp` (lines 161-281) to determine whether the legacy sensor periodic
calls are present or absent. Then add/confirm the cleanup and documentation comment.

### Files to modify

- `source/robot/LoopTickOnce.cpp` — remove legacy sensor periodic calls if present;
  add documentation comment confirming `sensors.tick()` schedule authority.

### Files to read first

- `source/robot/LoopTickOnce.cpp:161-281` — ordered-tick branch, step 7 and ports block.
- `source/subsystems/sensors/Sensors.h` — confirm `_lastLineTick`/`_lastColorTick`.

### Testing plan

1. `uv run python -m pytest tests/simulation/unit/test_golden_tlm.py -v`
2. `uv run python -m pytest` — green except 2 known-baseline failures.

### Documentation updates

Add an inline comment in `LoopTickOnce.cpp` step 7 block confirming the schedule
authority. No changes to architecture docs.
