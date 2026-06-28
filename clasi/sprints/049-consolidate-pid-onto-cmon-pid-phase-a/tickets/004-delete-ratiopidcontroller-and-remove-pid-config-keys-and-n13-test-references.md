---
id: '004'
title: Delete RatioPidController and remove pid.* config keys and N13 test references
status: done
use-cases:
- SUC-003
depends-on:
- '003'
github-issue: ''
issue: consolidate-control-code-onto-vendored-libraries-cmon-pid-tinyekf.md
completes_issue: false
---

# Delete RatioPidController and remove pid.* config keys and N13 test references

## Description

Remove all dead code associated with the deprecated `RatioPidController` and
its `pid.*` config key set, which were officially declared dead in sprint 013
(N13 note) and retained only for host-side test compatibility.

This ticket has three sub-tasks that must be completed together so the tree
remains green at exit:

1. **Delete source files**: `source/control/RatioPidController.h` and
   `source/control/RatioPidController.cpp`.
2. **Delete config struct fields**: `ratioPidKp`, `ratioPidKi`, `ratioPidKd`,
   `ratioPidMax` from `source/types/Config.h` and the four `pid.*` entries
   from `source/robot/ConfigRegistry.cpp`; delete the four initializer lines
   from `source/robot/DefaultConfig.cpp`.
3. **Update tests**: remove all `pid.*` references from the sim test suite and
   delete `tests/simulation/unit/test_ratio_pid.py`.

The N13 note in `MotorController.h` (which explains the dead-code retention)
must be updated or removed once the fields are gone.

## Acceptance Criteria

- [x] `source/control/RatioPidController.h` does not exist.
- [x] `source/control/RatioPidController.cpp` does not exist.
- [x] `source/types/Config.h` contains no `ratioPidKp`, `ratioPidKi`,
      `ratioPidKd`, or `ratioPidMax` fields.
- [x] `source/robot/ConfigRegistry.cpp` `kRegistry[]` contains no
      `pid.kp`, `pid.ki`, `pid.kd`, or `pid.max` entries.
- [x] `source/robot/DefaultConfig.cpp` contains no `ratioPid` assignments.
- [x] `source/control/MotorController.h` N13 note is updated or removed.
- [x] `tests/simulation/unit/test_ratio_pid.py` does not exist.
- [x] No remaining `pid.kp`, `pid.ki`, `pid.kd`, `pid.max` strings appear in
      any `tests/simulation/` Python file as live config key assertions (grep confirms;
      remaining occurrences are deletion-verification tests or comments).
- [x] `uv run --with pytest python -m pytest tests/simulation -q` exits with
      exactly the 2 pre-existing failures (no new failures introduced).

## Implementation Plan

### Approach

**Step 1 — Delete source files**

```
git rm source/control/RatioPidController.h
git rm source/control/RatioPidController.cpp
```

These files are picked up by `file(GLOB CONTROL_SOURCES ...)` in the sim
CMakeLists.txt, so deletion automatically removes them from both builds. No
CMake edits needed for this step.

**Step 2 — Delete Config.h fields**

In `source/types/Config.h`, remove the four-line block:
```cpp
    // Ratio PID gains
    float ratioPidKp;
    float ratioPidKi;
    float ratioPidKd;
    float ratioPidMax;
```

**Step 3 — Delete ConfigRegistry.cpp entries**

In `source/robot/ConfigRegistry.cpp` `kRegistry[]`, remove:
```cpp
    // Ratio PID gains
    CFG_F("pid.kp",       ratioPidKp),
    CFG_F("pid.ki",       ratioPidKi),
    CFG_F("pid.kd",       ratioPidKd),
    CFG_F("pid.max",      ratioPidMax),
```
Also delete the N13 comment block at lines ~520-524 (the note that explains why
`pid.*` keys were retained even though the controller was removed).

**Step 4 — Delete DefaultConfig.cpp initializers**

In `source/robot/DefaultConfig.cpp`, remove:
```cpp
    p.ratioPidKp      = 300.0f;
    p.ratioPidKi      = 0.0f;
    p.ratioPidKd      = 0.0f;
    p.ratioPidMax     = 30.0f;
```

**Step 5 — Update MotorController.h**

In `source/control/MotorController.h`, find and remove or update the N13 block
in the class doc comment (approximately lines 17-22) that reads "N13
(030-010): RatioPidController removed — its update() was never called..."
since the fields are now gone entirely.

**Step 6 — Delete test_ratio_pid.py**

```
git rm tests/simulation/unit/test_ratio_pid.py
```

**Step 7 — Update affected test files**

The following files contain `pid.*` assertions that will break once the keys
are removed. Each must be updated before the ticket exits:

- `tests/simulation/unit/test_config_registry.py`:
  Remove `("pid.kp", "float")`, `("pid.ki", "float")`, `("pid.kd", "float")`,
  `("pid.max", "float")` from the parametrize list (lines 79-82). Remove
  `pid.kp=300.000` from the expected GET dump string (lines 125-126). Remove
  the `test_n13_pid_key_retention` class entirely (lines 152-180).

- `tests/simulation/unit/test_config_set.py`:
  Remove or update tests that use `pid.kp`/`pid.ki` as the test key (lines
  78-120). Remove the class `TestPidKeyRetentionN13` if present.

- `tests/simulation/unit/test_n12_n13_get_chunking_dead_code.py`:
  Remove the `TestN13DeadCodeRemoval` class and any `pid.*` key assertions
  (lines 95-180). Remove `"pid.kp"`, `"pid.ki"`, `"pid.kd"`, `"pid.max"` from
  the expected key list (lines 105).

- `tests/simulation/unit/test_protocol_v2.py`:
  Search for `pid\.` and remove any assertion that `pid.*` keys appear in GET
  output (line 761-769 area).

- `tests/simulation/unit/test_imports_smoke.py`:
  Check for `pid.` references and remove if present.

- `tests/simulation/unit/test_motor_controller_coverage.py`:
  Check for N13-related `pid.*` assertions and remove.

**Verification after each file update**: run
`uv run --with pytest python -m pytest tests/simulation/unit/<modified_file> -q`
and confirm no new failures in that file before moving to the next.

### Files to delete

- `/Volumes/Proj/proj/RobotProjects/radio-robot-elite/source/control/RatioPidController.h`
- `/Volumes/Proj/proj/RobotProjects/radio-robot-elite/source/control/RatioPidController.cpp`
- `/Volumes/Proj/proj/RobotProjects/radio-robot-elite/tests/simulation/unit/test_ratio_pid.py`

### Files to modify

- `/Volumes/Proj/proj/RobotProjects/radio-robot-elite/source/types/Config.h`
- `/Volumes/Proj/proj/RobotProjects/radio-robot-elite/source/robot/ConfigRegistry.cpp`
- `/Volumes/Proj/proj/RobotProjects/radio-robot-elite/source/robot/DefaultConfig.cpp`
- `/Volumes/Proj/proj/RobotProjects/radio-robot-elite/source/control/MotorController.h`
- `/Volumes/Proj/proj/RobotProjects/radio-robot-elite/tests/simulation/unit/test_config_registry.py`
- `/Volumes/Proj/proj/RobotProjects/radio-robot-elite/tests/simulation/unit/test_config_set.py`
- `/Volumes/Proj/proj/RobotProjects/radio-robot-elite/tests/simulation/unit/test_n12_n13_get_chunking_dead_code.py`
- `/Volumes/Proj/proj/RobotProjects/radio-robot-elite/tests/simulation/unit/test_protocol_v2.py`
- `/Volumes/Proj/proj/RobotProjects/radio-robot-elite/tests/simulation/unit/test_imports_smoke.py` (check)
- `/Volumes/Proj/proj/RobotProjects/radio-robot-elite/tests/simulation/unit/test_motor_controller_coverage.py` (check)

### Testing plan

After all changes, run the full canonical suite:

```
uv run --with pytest python -m pytest tests/simulation -q
```

Expected:
- Exactly 2 failures: `test_default_config_pin.py::test_default_robot_config_unchanged`
  and `test_robot_config.py::TestSchemaValidation::test_tovez_validates_against_schema`.
  These are pre-existing config-schema drift failures unrelated to this sprint.
- `test_ratio_pid.py` is absent — its prior pass count is simply gone.
- `test_config_registry.py`, `test_config_set.py`, `test_n12_n13_get_chunking_dead_code.py`
  all pass with `pid.*` references removed.
- `test_vendor_confinement.py` passes (no source-layer changes in this direction).

Also confirm with grep that no `pid\.kp|pid\.ki|pid\.kd|pid\.max` strings remain
in `tests/simulation/`:

```
grep -r 'pid\.\(kp\|ki\|kd\|max\)' tests/simulation/ --include="*.py"
```

Expected: zero matches.

### Documentation

No doc changes beyond the inline comments removed above. The N13 note in
`MotorController.h` is the only externally-visible documentation of the
dead-code decision; removing it is correct once the fields are gone.
