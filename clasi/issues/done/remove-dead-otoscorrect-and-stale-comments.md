---
status: done
tickets:
- NONE
---

# Remove dead Robot::otosCorrect and fix stale OTOS-fusion comments

## Problem

`Robot::otosCorrect()` ([Robot.cpp:176](source/robot/Robot.cpp#L176)) has
no callers since the sprint-060 ordered-tick cutover — the live fusion
path is `Drive::tickUpdate` STEP 5. The dead function and several stale
comments actively mislead debugging:

- [Planner.cpp:139](source/superstructure/Planner.cpp#L139): claims
  "Robot::otosCorrect() called at the slow cadence in LoopScheduler" —
  it is called nowhere.
- `DBG OTOS`'s reply labels `state.actual.optical.pose` as `fused=`
  ([DebugCommands.cpp](source/commands/DebugCommands.cpp#L456)) — it is
  the raw-OTOS (pre-EKF) pose, not the EKF-fused pose; the comment block
  above it says "EKF-fused pose written by otosCorrect()".
- Several Drive.cpp comments describe behaviour as "mirrors
  Robot::otosCorrect()" — Drive is now the only implementation.

## Proposed fix

Delete `Robot::otosCorrect` (and its `_otosInvalidStartMs` /
`_otosLostEmitted` state if unused elsewhere), fix the `DBG OTOS` label
(emit both `optical=` and true `fused=` from `state.actual.fused.pose` —
having both would have shortened today's diagnosis), and update the
stale comments to point at `Drive::tickUpdate` STEP 5.
