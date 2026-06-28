---
id: '005'
title: 'Tests cleanup and final verification: delete mecanum integration tests, verify
  green'
status: in-progress
use-cases:
- SUC-048-004
- SUC-048-001
depends-on:
- '001'
- '002'
- '003'
- '004'
issue: eliminate-ifdef-robot-drivetrain-mecanum-everywhere.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

## Description

Final cleanup and verification gate. Delete the three mecanum-integration test files
and `WheelTestMain.cpp` that exercised the now-deleted integrated mecanum code path.
Retain `test_mecanum_kinematics.py` (pure math, no `#ifdef` dependency). Verify
bench scripts are compatible with the differential robot. Run the full success gate.

This ticket is last — it depends on all prior tickets because:
- The deleted tests exercised code that no longer exists (compilation-safe only if
  the source was already stripped).
- The final `grep` gate confirms zero residual macro references across the full
  codebase (meaningful only after tickets 001–004 are done).
- The `uv run pytest` green gate must reflect the final state of all deletions.

## Acceptance Criteria

- [ ] `tests/simulation/unit/test_046_007_mecanum_tlm_format.py` deleted.
- [ ] `tests/simulation/unit/test_046_006_otos_lateral_vy.py` deleted.
- [ ] `tests/simulation/unit/test_mecanum_vw_bvc.py` deleted.
- [ ] `tests/WheelTestMain.cpp` deleted.
- [ ] `tests/simulation/unit/test_mecanum_kinematics.py` still present and passes.
- [ ] `tests/bench/wheel_test.py` runs against the differential robot without error
  (verify it does not hard-require mecanum; adjust if needed).
- [ ] `tests/bench/teleop.py` runs against the differential robot without error.
- [ ] `tests/bench/playfield_camera_run.py` runs against the differential robot
  without error.
- [ ] **Final success gate:**
  - [ ] `grep -rn ROBOT_DRIVETRAIN_MECANUM source tests CMakeLists.txt build.py`
    returns **zero** matches.
  - [ ] `python build.py --clean` compiles clean, no errors or warnings.
  - [ ] `uv run pytest` passes, all green (differential single-config).
  - [ ] `test_mecanum_kinematics.py` is included in the passing run.

## Implementation Plan

### Approach

**Step 1 — Delete mecanum-integration tests:**

```
tests/simulation/unit/test_046_007_mecanum_tlm_format.py  → delete
tests/simulation/unit/test_046_006_otos_lateral_vy.py     → delete
tests/simulation/unit/test_mecanum_vw_bvc.py              → delete
tests/WheelTestMain.cpp                                   → delete
```

These tests exercised:
- `test_046_007`: Mecanum telemetry format (rear-motor fields now deleted from
  `RobotTelemetry.cpp` in ticket 003).
- `test_046_006`: OTOS lateral `vy` fusion (`setOtosAlphaVy` / `_fusedVy` now
  deleted from `Odometry` in ticket 003).
- `test_mecanum_vw_bvc`: BVC 3-arg `setTarget(v, omega, vy)` (deleted in ticket 002).
- `WheelTestMain.cpp`: Per-wheel mecanum motor diagnostic (`WHEEL_TEST_MAIN` block
  deleted from `main.cpp` in ticket 001).

**Step 2 — Audit bench scripts:**

Read each of the three bench scripts and check if they branch on drivetrain type.
Expected behavior:
- If they use `robot_config.drivetrain_type` to choose a code path: confirm the
  differential path is correct and the mecanum path is either unreachable or
  guarded cleanly. No edits expected.
- If any hard-requires mecanum (e.g. calls `setTarget(v, omega, vy)` directly or
  references rear-motor addresses): adjust to differential-only.

**Step 3 — Run success gate:**

```bash
grep -rn ROBOT_DRIVETRAIN_MECANUM source tests CMakeLists.txt build.py
python build.py --clean
uv run pytest
```

All three must pass. The `grep` returning zero is the definitive gate.

### Files to Delete

- `tests/simulation/unit/test_046_007_mecanum_tlm_format.py`
- `tests/simulation/unit/test_046_006_otos_lateral_vy.py`
- `tests/simulation/unit/test_mecanum_vw_bvc.py`
- `tests/WheelTestMain.cpp`

### Files to Audit (modify only if needed)

- `tests/bench/wheel_test.py`
- `tests/bench/teleop.py`
- `tests/bench/playfield_camera_run.py`

### Files to Retain Unchanged

- `tests/simulation/unit/test_mecanum_kinematics.py`
- `source/kinematics/MecanumKinematics.h`
- `source/kinematics/MecanumKinematics.cpp`
- `source/io/real/MecanumHAL.cpp`

### Testing Plan

The testing plan IS the acceptance criteria gate:

1. `grep -rn ROBOT_DRIVETRAIN_MECANUM source tests CMakeLists.txt build.py` → zero.
2. `uv run pytest` → all green.
3. Confirm `test_mecanum_kinematics.py` is in the passing run (not silently skipped).
4. `python build.py --clean` → clean firmware build.

If any bench script requires adjustment, verify it against the differential robot
configuration (not against a mecanum robot JSON) before marking this ticket done.

### Documentation Updates

None. The sprint closure will trigger consolidate-architecture to fold this sprint's
update into the main architecture document.
