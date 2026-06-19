---
id: '001'
title: Repoint TLM and MotionController pose reads to PhysicalStateEstimate seam
status: done
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: migrate-radio-robot-c-to-the-frc-elite-architecture-c-codal-adaptation.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 044-001: Repoint TLM and MotionController pose reads to PhysicalStateEstimate seam

## Description

Phase C (`architecture-update-041.md`) explicitly deferred this read-path
repoint to Phase F: "keep publishing the fused pose back into HardwareState so
existing readers work byte-identically; repoint readers to `getPose()` in the
cleanup phase."

This ticket performs the READ-PATH repoint. There is no write-path change:
`Odometry::predict` and `correctEKF` continue to write `poseX/Y/poseHrad/
fusedV/fusedOmega` into `HardwareState` as the primary store. The only change
is that two callers now call through the `PhysicalStateEstimate` static methods
instead of calling `Odometry::getPose` or reading fields directly.

The values produced are byte-identical because `PhysicalStateEstimate::getPose`
and `getVelocity` are thin forwarders to the same `HardwareState` fields.

## Files to Change

**`source/robot/RobotTelemetry.cpp`:**
- Line ~27: `Odometry::getPose(state.inputs, pose_x, pose_y, pose_h)`
  → `estimate.getPose(state.inputs, pose_x, pose_y, pose_h)`
- Lines 83-84: `state.inputs.fusedV` / `state.inputs.fusedOmega`
  → call `estimate.getVelocity(state.inputs, fV, fOmega)` and use the local vars.
- Remove `#include "Odometry.h"` from `RobotTelemetry.cpp` if `Odometry::getPose`
  was the only reason it was included (the include may come via `Robot.h` which
  already includes everything needed).

**`source/superstructure/MotionController.cpp`** (around line 353-360):
- `getPoseFloat` currently calls `Odometry::getPose(*_hwState, xi, yi, hi)`.
  Change to `PhysicalStateEstimate::getPose(*_hwState, xi, yi, hi)`.
- Add `#include "state/PhysicalStateEstimate.h"` to `MotionController.cpp` if
  not already transitively included.

**What does NOT change in this ticket:**
- `MotionCommand.cpp` lines 101-103 (baseline capture `poseHrad/poseX/poseY`)
  — these are internal motion-control reads, not TLM, and remain direct reads.
- `MotionControllerBegin.cpp` line 461 (`_hwState->poseHrad`) — same.
- `StopCondition.cpp` direct reads of `poseHrad/poseX/poseY` — same.
- `Robot::otosCorrect` in `Robot.cpp` line 209 (`state.inputs.poseHrad`) — same.
- `WorldView.cpp` direct reads — test infrastructure, not seam output.

## Acceptance Criteria

- [x] `buildTlmFrame` calls `estimate.getPose(state.inputs, ...)` (not `Odometry::getPose`).
- [x] `buildTlmFrame` reads `fusedV`/`fusedOmega` via `estimate.getVelocity(state.inputs, ...)`.
- [x] `MotionController::getPoseFloat` calls `PhysicalStateEstimate::getPose(*_hwState, ...)`.
- [x] `Odometry::getPose` is not called from `RobotTelemetry.cpp` or `MotionController.cpp`.
- [x] Golden-TLM canary passes byte-exact (`uv run --with pytest python -m pytest tests/simulation/unit/test_golden_tlm.py -q`).
- [x] Full simulation tier green: `uv run --with pytest python -m pytest -q` >= 2001 passed, 0 errors.
- [x] ARM firmware build green: `python3 build.py --fw-only` → 0 errors. Then `git checkout -- source/robot/DefaultConfig.cpp`.
- [x] Behavior-preservation fences green: `test_ekf*.py`, `test_otos_fusion.py`, `test_goto_bounds.py`.

## Implementation Plan

1. Edit `source/robot/RobotTelemetry.cpp`:
   - In `buildTlmFrame`, replace the `Odometry::getPose` call with
     `estimate.getPose(state.inputs, pose_x, pose_y, pose_h)`.
   - For the `twist=` block, introduce `float fV, fOmega; estimate.getVelocity(state.inputs, fV, fOmega);`
     before the `snprintf` and use `(int)fV` and `(int)(fOmega * 1000.0f)`.
   - Check whether `#include "Odometry.h"` is still needed in `RobotTelemetry.cpp`;
     if only needed for `getPose`, it can be removed (Robot.h pulls it transitively).

2. Edit `source/superstructure/MotionController.cpp`:
   - In `getPoseFloat`, change `Odometry::getPose(*_hwState, xi, yi, hi)` to
     `PhysicalStateEstimate::getPose(*_hwState, xi, yi, hi)`.
   - Add `#include "state/PhysicalStateEstimate.h"` if not already present
     (check what `MotionController.h` includes transitively).

3. Run `uv run --with pytest python -m pytest tests/simulation/unit/test_golden_tlm.py -v`
   first to confirm byte-exact.

4. Run full simulation tier: `uv run --with pytest python -m pytest -q`.

5. Run `python3 build.py --fw-only` (ARM gate). Then `git checkout -- source/robot/DefaultConfig.cpp`.

## Testing Plan

- **Primary gate:** `test_golden_tlm.py` — must be byte-exact.
- **Behavior fences:** `test_ekf*.py`, `test_otos_fusion.py`, `test_goto_bounds.py`,
  `test_watchdog_exemption.py`, `test_incident_scenarios.py`.
- **Full suite:** `uv run --with pytest python -m pytest -q`.
- **ARM build:** `python3 build.py --fw-only`.

## Notes

- The `Robot` struct has a member `estimate` of type `PhysicalStateEstimate`
  (added in Phase C). `buildTlmFrame` is a `Robot::` member, so it can call
  `estimate.getPose(...)` directly. `MotionController` holds `const HardwareState*
  _hwState`; `PhysicalStateEstimate::getPose` is a static method taking the
  `HardwareState` by const ref, so it can be called as
  `PhysicalStateEstimate::getPose(*_hwState, xi, yi, hi)` without a reference
  to the `estimate` object.
- No `HardwareState` fields are added or removed. No struct layout changes.
- This ticket MUST be done before T2 (the RobotState.h rename) because the
  rename touches many files and golden-TLM should be confirmed green before
  that mechanical churn.
