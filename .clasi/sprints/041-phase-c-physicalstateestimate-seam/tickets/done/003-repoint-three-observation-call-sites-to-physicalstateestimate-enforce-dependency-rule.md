---
id: '003'
title: Repoint three observation call-sites to PhysicalStateEstimate; enforce dependency
  rule
status: done
use-cases:
- SUC-001
- SUC-003
- SUC-004
- SUC-005
depends-on:
- 041-002
github-issue: ''
issue: migrate-radio-robot-c-to-the-frc-elite-architecture-c-codal-adaptation.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Repoint three observation call-sites to PhysicalStateEstimate; enforce dependency rule

## Description

Rename `Robot::odometry` to `Robot::estimate` (type `PhysicalStateEstimate`) and
repoint the three external observation call sites to the estimate's API methods.
Also repoint all accessors (TLM, wedge push, encoder rebaseline) so there are zero
remaining external references to `robot.odometry.*`. After this ticket, Phase C is
behaviorally complete: the seam exists, the dependency rule holds, and the full suite
is green.

`HardwareState.pose*`/`fused*` fields continue to be populated by the estimate's
delegating methods — unchanged by this ticket.

### Three observation call-sites to repoint

| Location | Old call | New call |
|---|---|---|
| `source/control/LoopTickOnce.cpp` line ~199 | `robot.odometry.predict(robot.state.inputs, cfg.trackwidthMm, cfg.rotationalSlip, now)` | `robot.estimate.addOdometryObservation(robot.state.inputs, cfg.trackwidthMm, cfg.rotationalSlip, now)` |
| `source/robot/Robot.cpp` (`otosCorrect`) line ~233 | `odometry.correctEKF(state.inputs, p.x, p.y, p.h, vel.v_mmps, vel.omega_rads)` | `estimate.addOtosObservation(state.inputs, p.x, p.y, p.h, vel.v_mmps, vel.omega_rads)` |
| `source/app/SystemCommands.cpp` (`handleSI`) line ~725 | `robot->odometry.setPose(robot->state.inputs, x_mm, y_mm, h_cdeg)` | `robot->estimate.resetPose(robot->state.inputs, x_mm, y_mm, h_cdeg)` |

### Additional call-sites to repoint (accessors and wiring)

All of the following reference `robot.odometry` (or `odometry.`) and must become
`robot.estimate` (or `estimate.`) equivalents:

**In `source/control/LoopTickOnce.cpp`:**
- `robot.odometry.setWedgeActive(anyWedged)` → `robot.estimate.setWedgeActive(anyWedged)`
- `robot.odometry.setEncOmegaHealthy(false)` → `robot.estimate.setEncOmegaHealthy(false)`
- `robot.odometry.setEncOmegaHealthy(true)` → `robot.estimate.setEncOmegaHealthy(true)`

**In `source/robot/Robot.cpp`:**
- `odometry.setCtx(&otos, &state.inputs)` → `estimate.setCtx(&otos, &state.inputs)` (constructor)
- `odometry.initEKF(...)` → `estimate.initEKF(...)` (constructor)
- `odometry.rebaselinePrev(0.0f, 0.0f)` → `estimate.rebaselinePrev(0.0f, 0.0f)` (`resetEncoders`)

**In `source/robot/Robot.h`:**
- `Odometry odometry;` → `PhysicalStateEstimate estimate;`
- Add `#include "PhysicalStateEstimate.h"` (or adjust include path)
- Remove `#include "Odometry.h"` from `Robot.h` if `PhysicalStateEstimate.h`
  provides the needed transitives. (Check: `Robot.h` uses `Odometry` type only
  for the `odometry` member; after rename, it uses `PhysicalStateEstimate` only.
  `Odometry.h` is included from `PhysicalStateEstimate.h` already.)

**In `source/robot/RobotTelemetry.cpp` (if applicable):**
- `robot.odometry.ekfRejectCount()` → `robot.estimate.ekfRejectCount()`
- `robot.odometry.otosRejectedCount()` → `robot.estimate.otosRejectedCount()` (if used)
- Any `robot.odometry.ekfPDiag(...)` → `robot.estimate.ekfPDiag(...)`

**In `source/control/MotionController.cpp` (if it references odometry):**
- Grep `odometry` in `MotionController.cpp`; if found, repoint. (Expected: none —
  `MotionController` reads `HardwareState` not `Odometry` directly.)

**In `tests/_infra/sim/sim_api.cpp` (if applicable):**
- Grep `robot.odometry` / `robot->odometry`; repoint any found. (Expected: unlikely —
  sim_api accesses state fields directly, not the estimator object.)

### Step-by-step plan

1. Rename `odometry` to `estimate` in `source/robot/Robot.h` (member declaration and
   type). Update include from `Odometry.h` to `PhysicalStateEstimate.h` (keep
   `Odometry.h` if still needed by other members — check after the rename).

2. Update `source/robot/Robot.cpp` constructor: rename `odometry(...)` initializer
   to `estimate(...)` (it is a default-constructed value member — the initializer
   list entry name must match the member name; `PhysicalStateEstimate` has a
   default constructor, so no arguments needed).

3. Update all call sites in `Robot.cpp` body: `odometry.` → `estimate.`

4. Update `source/control/LoopTickOnce.cpp`: `robot.odometry.` → `robot.estimate.`

5. Update `source/app/SystemCommands.cpp` (`handleSI`): `robot->odometry.` →
   `robot->estimate.`

6. Update `source/robot/RobotTelemetry.cpp`: any `robot.odometry.` →
   `robot.estimate.`

7. Full grep audit: `grep -rn "\.odometry\." source/` and `grep -rn "odometry\."
   source/` — zero hits expected outside `Odometry.{h,cpp}` and `PhysicalStateEstimate.{h,cpp}`.

8. Build and test:
   ```
   python3 build.py --fw-only
   git checkout -- source/robot/DefaultConfig.cpp
   uv run --with pytest python -m pytest -q
   ```

9. Run the golden-TLM canary explicitly and confirm byte-exact:
   ```
   uv run --with pytest python -m pytest tests/simulation/unit/test_golden_tlm.py -v
   ```

## Acceptance Criteria

- [x] `Robot::odometry` renamed to `Robot::estimate` of type `PhysicalStateEstimate`.
- [x] Zero external calls to `robot.odometry.*` or `odometry.*` outside `Odometry.{h,cpp}` and `PhysicalStateEstimate.{h,cpp}`. (Also repointed the 5 `robot.odometry.*` accessor call-sites in `tests/_infra/sim/sim_api.cpp` to `estimate.*`.)
- [x] `loopTickOnce` calls `robot.estimate.addOdometryObservation(...)`.
- [x] `Robot::otosCorrect` calls `estimate.addOtosObservation(...)`.
- [x] `handleSI` calls `robot->estimate.resetPose(...)`.
- [x] All wedge/omega-health/rebaseline accessors routed through `robot.estimate.*` (incl. ZERO-pose path → `robot->estimate.zero(...)`).
- [x] `PhysicalStateEstimate` has no `CommandTypes.h`, `Commandable`, `MicroBit.h`, or `Protocol.h`-as-a-command-surface in its include graph. New fence `test_estimate_dependency_rule.py` walks the actual transitive `#include` graph and asserts no command-dispatch surface (`CommandTypes.h`/`CommandProcessor.h`/`Commandable`/`CommandDescriptor`), no CODAL (`MicroBit.h`), no device handle (`I2CBus`/`NezhaHAL`); reachable header set is a subset of the documented allowed set. NOTE per architecture-update.md §Module Definitions: `Protocol.h` IS reachable transitively via `RobotState.h` as a pure types header (reply tags + `ReplyFn` + `KVPair`; no `Commandable`/`CommandDescriptor`) — this is the documented allowed set; the fence asserts `Protocol.h` carries no command-dispatch surface rather than asserting its total absence.
- [x] `python3 build.py --fw-only` → 0 errors; `git checkout -- source/robot/DefaultConfig.cpp`.
- [x] `uv run --with pytest python -m pytest -q` → ≥ 1997 passed, 0 errors. (2001 passed.)
- [x] Golden-TLM canary byte-exact (`test_golden_tlm.py` passes — frame-by-frame string equality against committed capture).
- [x] Field-pin diff empty (`test_default_config_pin.py` passes).
- [x] Vendor-confinement grep gate passes (`test_vendor_confinement.py`).
- [x] Phase B fences green: `test_ekf*.py`, `test_otos_fusion.py`, `test_estimator_isolation.py`, `test_estimator_command_paths.py`, `test_observation_models.py`, `test_incident_scenarios.py`, `test_watchdog_exemption.py`.

## Testing

- **Existing tests to run**: full simulation tier — `uv run --with pytest python -m pytest -q`
- **Key fences**: `test_golden_tlm.py` (byte-exact TLM after observation-site repoint), `test_ekf*.py` + `test_otos_fusion.py` (estimator behavior unchanged), `test_estimator_isolation.py` + `test_observation_models.py` (Phase B seam guards), `test_estimator_command_paths.py` (OtosCommands verbs still work)
- **New tests to write**: none — existing fences provide complete coverage of the behavioral invariants
- **Verification command**: `uv run --with pytest python -m pytest -q` (≥ 1997 passed, 0 errors)
