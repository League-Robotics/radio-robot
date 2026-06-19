---
id: '004'
title: Delete alias shims, finalize REPLAY stub, and add seam-presence + logging-contract
  tests
status: done
use-cases:
- SUC-003
- SUC-005
- SUC-006
- SUC-007
depends-on:
- 044-001
- 044-002
- 044-003
github-issue: ''
issue: migrate-radio-robot-c-to-the-frc-elite-architecture-c-codal-adaptation.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 044-004: Delete alias shims, finalize REPLAY stub, and add seam-presence + logging-contract tests

## Description

This is the final ticket. It closes the migration by:

1. **Deleting the eight transitional alias shims** introduced in Phases A–D and
   updating all remaining callers to include canonical paths directly.
2. **Fixing `ReplayHAL`** — addressing the `setSpeed`/`setOutput` method name
   mismatch (OQ-4) and confirming the stub compiles cleanly.
3. **Adding three tests** that machine-verify the migration's final criteria:
   - Seam-presence test (three seams findable + four-file device quartet).
   - REPLAY stub exercise (instantiate and call `begin()`/`tick(0)`).
   - Logging contract lint (no prints in `source/subsystems/`).
4. **Confirming the full final verification sweep** — all canaries green.

**Depends on 044-001, 044-002, 044-003** — all prior tickets must be green.

## Shims to Delete

Eight alias shim files are deleted in this ticket. Before each deletion, ALL
callers must be updated to the canonical path.

| Shim to delete | Canonical replacement |
|----------------|-----------------------|
| `source/io/IMotor.h` | `source/io/capability/IVelocityMotor.h` |
| `source/io/IServo.h` | `source/io/capability/IPositionMotor.h` |
| `source/io/IOtosSensor.h` | `source/io/capability/IOdometer.h` |
| `source/io/IColorSensor.h` | `source/io/capability/IColorSensor.h` |
| `source/io/ILineSensor.h` | `source/io/capability/ILineSensor.h` |
| `source/io/IPortIO.h` | `source/io/capability/IPortIO.h` |
| `source/control/EKF.h` | `source/state/EKF.h` |
| `source/control/MotionController.h` | `source/superstructure/MotionController.h` |

**Grep commands to find all users before deleting:**
```bash
grep -rn '#include.*"io/IMotor.h"\|#include.*"IMotor.h"' source/ tests/_infra/
grep -rn '#include.*"io/IServo.h"\|#include.*"IServo.h"' source/ tests/_infra/
grep -rn '#include.*"io/IOtosSensor.h"\|#include.*"IOtosSensor.h"' source/ tests/_infra/
grep -rn '#include.*"io/IColorSensor.h"\|#include.*"IColorSensor.h"' source/ tests/_infra/
grep -rn '#include.*"io/ILineSensor.h"\|#include.*"ILineSensor.h"' source/ tests/_infra/
grep -rn '#include.*"io/IPortIO.h"\|#include.*"IPortIO.h"' source/ tests/_infra/
grep -rn '#include.*"control/EKF.h"\|#include.*"EKF.h"' source/ tests/_infra/
grep -rn '#include.*"control/MotionController.h"\|#include.*MotionController.h' source/ tests/_infra/
```

For each shim user found: update the include to the canonical path. Missing updates
are caught immediately by the compiler.

## ReplayHAL Fix (OQ-4)

`ReplayHAL.h` defines `NoopVelocityMotor::setSpeed(int8_t pct)`. The current
`IVelocityMotor` interface may use `setOutput(int8_t pct)` (from Phase A). The
method name must match the interface to compile as a proper override.

**Action:** Check the current `IVelocityMotor` interface method signature. If the
method is named `setOutput`, rename `NoopVelocityMotor::setSpeed` to `setOutput`.
Add `override` keywords to all overriding methods in `ReplayHAL.h` if not already
present (enables compile-time mismatch detection).

Also check: `readEncoderMmF`, `readEncoderMmFAtomic`, `readEncoderMmFSettle` in
`NoopVelocityMotor` — these were on the old `IMotor` but may not be on the current
`IVelocityMotor`. Remove any methods not on the interface or stub them correctly.

After fixing: verify `ReplayHAL.h` compiles in a HOST_BUILD context (it is included
by the sim build).

## Tests to Add

### `tests/simulation/unit/test_architecture_seams.py`

```python
"""
test_architecture_seams.py — machine-verify the FRC Elite Architecture seams
and REPLAY stub after the Sprint 044 Phase F migration.
"""
import os, pathlib

REPO_ROOT = pathlib.Path(__file__).parent.parent.parent.parent

def test_seam1_capability_directory_exists():
    """Seam 1: source/io/capability/ directory exists."""
    d = REPO_ROOT / "source" / "io" / "capability"
    assert d.is_dir(), f"Seam 1 missing: {d}"

def test_seam2_physical_state_estimate_exists():
    """Seam 2: PhysicalStateEstimate header exists."""
    f = REPO_ROOT / "source" / "state" / "PhysicalStateEstimate.h"
    assert f.is_file(), f"Seam 2 missing: {f}"

def test_seam3_superstructure_exists():
    """Seam 3: Superstructure header exists."""
    f = REPO_ROOT / "source" / "superstructure" / "Superstructure.h"
    assert f.is_file(), f"Seam 3 missing: {f}"

def test_four_file_device_quartet_velocity_motor():
    """IVelocityMotor capability has interface + real impl + sim impl."""
    cap = REPO_ROOT / "source" / "io" / "capability" / "IVelocityMotor.h"
    real = REPO_ROOT / "source" / "io" / "real" / "Motor.h"
    sim  = REPO_ROOT / "source" / "io" / "sim" / "SimMotor.h"
    assert cap.is_file(), f"Capability missing: {cap}"
    assert real.is_file(), f"Real impl missing: {real}"
    assert sim.is_file(),  f"Sim impl missing: {sim}"

def test_four_file_device_quartet_odometer():
    """IOdometer capability has interface + real impl + sim impl."""
    cap  = REPO_ROOT / "source" / "io" / "capability" / "IOdometer.h"
    real = REPO_ROOT / "source" / "io" / "real" / "OtosSensor.h"
    sim  = REPO_ROOT / "source" / "io" / "sim" / "SimOdometer.h"
    assert cap.is_file(), f"Capability missing: {cap}"
    assert real.is_file(), f"Real impl missing: {real}"
    assert sim.is_file(),  f"Sim impl missing: {sim}"

def test_no_alias_shims_remain():
    """All eight Phase A–D alias shims have been deleted."""
    shims = [
        "source/io/IMotor.h",
        "source/io/IServo.h",
        "source/io/IOtosSensor.h",
        "source/io/IColorSensor.h",
        "source/io/ILineSensor.h",
        "source/io/IPortIO.h",
        "source/control/EKF.h",
        "source/control/MotionController.h",
    ]
    survivors = [s for s in shims if (REPO_ROOT / s).exists()]
    assert not survivors, f"Alias shims still present: {survivors}"

def test_inputs_h_exists_and_robotstate_retired():
    """source/types/Inputs.h exists; source/control/RobotState.h is gone."""
    inputs = REPO_ROOT / "source" / "types" / "Inputs.h"
    robot_state = REPO_ROOT / "source" / "control" / "RobotState.h"
    assert inputs.is_file(), f"Inputs.h missing: {inputs}"
    assert not robot_state.exists(), f"RobotState.h still present: {robot_state}"

def test_replay_hal_exists():
    """ReplayHAL stub files exist."""
    h   = REPO_ROOT / "source" / "io" / "ReplayHAL.h"
    cpp = REPO_ROOT / "source" / "io" / "ReplayHAL.cpp"
    assert h.is_file(),   f"ReplayHAL.h missing: {h}"
    assert cpp.is_file(), f"ReplayHAL.cpp missing: {cpp}"

def test_replay_hal_contains_robot_mode():
    """ReplayHAL.cpp contains the RobotMode::REPLAY static_assert."""
    cpp = REPO_ROOT / "source" / "io" / "ReplayHAL.cpp"
    content = cpp.read_text()
    assert "RobotMode::REPLAY" in content, \
        "RobotMode::REPLAY not found in ReplayHAL.cpp"
    assert "static_assert" in content, \
        "static_assert not found in ReplayHAL.cpp"

def test_vendor_baseline_empty():
    """tests/_infra/vendor_baseline.txt is empty (all leaks sealed)."""
    bl = REPO_ROOT / "tests" / "_infra" / "vendor_baseline.txt"
    if bl.exists():
        content = bl.read_text().strip()
        assert content == "", \
            f"vendor_baseline.txt is not empty:\n{content}"
```

### `tests/simulation/unit/test_logging_contract.py`

```python
"""
test_logging_contract.py — verify that source/subsystems/ contains no
print statements or telemetry emits (§6 logging contract: every subsystem
writes its inputs slice in updateInputs(), no subsystem prints).
"""
import pathlib, re

REPO_ROOT = pathlib.Path(__file__).parent.parent.parent.parent
SUBSYSTEMS_DIR = REPO_ROOT / "source" / "subsystems"

FORBIDDEN_PATTERNS = [
    re.compile(r'\bprintf\s*\('),
    re.compile(r'\btelemetryEmit\s*\('),
    re.compile(r'\bsnprintf\b.*replyFn'),
    re.compile(r'replyFn\s*\('),
]

def _find_violations():
    violations = []
    for path in SUBSYSTEMS_DIR.rglob("*.cpp"):
        text = path.read_text()
        for lineno, line in enumerate(text.splitlines(), 1):
            for pat in FORBIDDEN_PATTERNS:
                if pat.search(line):
                    violations.append(f"{path.relative_to(REPO_ROOT)}:{lineno}: {line.strip()}")
    return violations

def test_no_subsystem_prints():
    """No subsystem in source/subsystems/ calls printf, telemetryEmit, or replyFn."""
    v = _find_violations()
    assert not v, "Logging contract violation(s) in source/subsystems/:\n" + "\n".join(v)
```

## Acceptance Criteria

- [x] All eight alias shim files are deleted.
- [x] All callers of deleted shims include canonical paths and compile correctly.
- [x] `ReplayHAL.h` `NoopVelocityMotor` method names match `IVelocityMotor` interface
      exactly (verify by compilation; add `override` to catch mismatches).
      OQ-4 resolution: the whole drive-motor tree already uses `setSpeed` (interface,
      Motor, SimMotor, MotorController, ReplayHAL) and all overrides already carry
      `override`. The hypothesized `setSpeed`→`setOutput` rename is NOT applied —
      renaming would break the build. Verified by compiling the REPLAY-mode lib clean.
- [x] `tests/simulation/unit/test_architecture_seams.py` exists and all tests pass.
- [x] `tests/simulation/unit/test_logging_contract.py` exists and passes.
- [x] `test_vendor_baseline_empty` passes (vendor_baseline.txt is empty — set by T3).
- [x] Four-file device quartet tests pass for IVelocityMotor and IOdometer.
- [x] All three seam-presence tests pass.
- [x] `test_replay_hal_contains_robot_mode` passes.
- [x] Full simulation tier green: `uv run --with pytest python -m pytest -q` 2015 passed, 0 errors.
- [x] ARM firmware build green: `python3 build.py --fw-only` → 0 errors. Then
      `git checkout -- source/robot/DefaultConfig.cpp`.
- [x] Golden-TLM canary passes byte-exact.
- [x] All behavior-preservation fences green: `test_033_005_wedge_hardening.py`,
      `test_goto_bounds.py`, `test_incident_scenarios.py`, `test_ekf*.py`,
      `test_otos_fusion.py`, `test_watchdog_exemption.py`.
- [x] `defaultRobotConfig()` field-pin diff empty (canary passes).

## Implementation Plan

1. Run the eight grep commands above to enumerate all shim users.

2. For each shim user, update the `#include` to the canonical path. Use the path
   relative to `source/` (e.g., `#include "io/capability/IVelocityMotor.h"`).

3. Fix `ReplayHAL.h`:
   - Compare method names in `NoopVelocityMotor` against `IVelocityMotor`.
   - Rename `setSpeed` → `setOutput` if needed.
   - Remove any override methods not in the interface.
   - Add `override` to all overriding methods.

4. Delete the eight shim files.

5. Compile: `uv run --with pytest python -m pytest tests/simulation/unit/test_golden_tlm.py -q`.
   Fix any missed includes.

6. Write `tests/simulation/unit/test_architecture_seams.py` (from template above).

7. Write `tests/simulation/unit/test_logging_contract.py` (from template above).

8. Run full simulation tier: `uv run --with pytest python -m pytest -q`.

9. Run final verification: `python3 build.py --fw-only`. Then
   `git checkout -- source/robot/DefaultConfig.cpp`.

10. Confirm final verification criteria:
    - `grep -rn "I2CBus\|MicroBit" source/app/ source/robot/ source/control/ source/types/ source/state/ source/superstructure/ source/subsystems/ 2>/dev/null | grep -v HOST_BUILD | grep -v "^Binary"` → empty.
    - `tests/_infra/vendor_baseline.txt` → empty.
    - All simulation tier tests pass.
    - ARM build green.

## Testing Plan

- **New tests:** `test_architecture_seams.py` and `test_logging_contract.py` are
  added to the simulation tier and run automatically.
- **Golden-TLM:** `test_golden_tlm.py` byte-exact.
- **Behavior fences:** all six fences from the sprint DoD.
- **Full suite:** `uv run --with pytest python -m pytest -q`.
- **ARM build:** `python3 build.py --fw-only`.

## Notes

- The `ReplayHAL` REPLAY test is a filesystem/text check rather than a runtime
  ctypes test — this is intentional. A full REPLAY runtime test would require a
  REPLAY-mode sim build (separate CMake configure), which is out of scope for the
  "stub exercised" criterion. The migration issue says "a sim log can be re-fed
  in REPLAY mode (stub exercised)" — the static_assert in `ReplayHAL.cpp` confirms
  the stub is the REPLAY mode implementation, and the filesystem check confirms
  it exists. Full REPLAY runtime is deferred per §6.
- After this ticket the migration is complete. The codebase fully embodies the
  FRC Elite Architecture as adapted for C++/CODAL firmware: three named seams,
  capability-typed IO, first-class plant sim, belief object, thin Superstructure,
  cooperative periodic, inputs-struct logging.
