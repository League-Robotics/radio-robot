---
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 038 Use Cases

## SUC-001: Reorganize tests/ into tier-separated directories

- **Actor**: Developer / CI
- **Preconditions**: `tests/` exists with the Sprint 037 flat layout (`sim/`, `unit/`,
  `bench/`, `calibrate/`, `tools/`, `system/`, `old/`). All 1954 simulation tests pass.
- **Main Flow**:
  1. Developer runs the canonical test command before any changes; it reports ≥ 1954 passed.
  2. The `tests/sim/` build infrastructure is moved to `tests/_infra/sim/` (CMakeLists.txt,
     sim_api.cpp, firmware.py). Paths in conftest.py, build.py, and CMakeLists.txt are updated.
  3. `tests/unit/` is split into `tests/simulation/unit/` (pure-logic unit tests that run
     against the sim lib) and `tests/simulation/system/` (whole-robot scenario tests).
  4. `tests/bench/` scripts move to `tests/bench/` (preserved at top level, renamed parent).
  5. `tests/system/` (goto_world, tours) moves to `tests/field/system/`.
  6. `tests/calibrate/` and `tests/tools/` move into `tests/_infra/`.
  7. `pyproject.toml` `testpaths` and `norecursedirs` are repointed so only
     `simulation/` is collected by default; `bench/` and `field/` are excluded.
  8. Developer runs the canonical command again; it still reports ≥ 1954 passed with
     no path errors.
- **Postconditions**: Tests live in the §7 tier layout. `bench/` and `field/` are not
  collected by the default run. `source/` has zero modifications.
- **Acceptance Criteria**:
  - [ ] `tests/` contains `simulation/unit/`, `simulation/system/`, `bench/unit/`,
        `bench/system/`, `field/unit/`, `field/system/`, and `_infra/`.
  - [ ] `uv run --with pytest python -m pytest -q` still collects and passes ≥ 1954 tests.
  - [ ] `bench/` and `field/` are not collected by default.
  - [ ] `tests/old/` remains excluded.
  - [ ] `source/` has zero modifications (verified by `git diff source/`).

---

## SUC-002: Sim build infrastructure remains functional after move

- **Actor**: Developer / CI
- **Preconditions**: `tests/_infra/sim/CMakeLists.txt` has been moved from `tests/sim/`.
- **Main Flow**:
  1. Developer runs `python3 build.py` (or the `build_lib` pytest fixture fires).
  2. CMake locates `source/` correctly from its new directory depth (`tests/_infra/sim/`
     is three levels below the repo root; `REPO_ROOT = ${CMAKE_SOURCE_DIR}/../../..`).
  3. `libfirmware_host` builds successfully into `tests/_infra/sim/build/`.
  4. The `sim` fixture in `tests/conftest.py` resolves `_SIM_DIR` and `_BUILD_DIR`
     to the new paths and `from firmware import Sim` succeeds.
- **Postconditions**: The simulation build works identically to before the move.
- **Acceptance Criteria**:
  - [ ] `python3 build.py` produces `tests/_infra/sim/build/libfirmware_host.*` without errors.
  - [ ] `from firmware import Sim` succeeds in any simulation test.
  - [ ] `build_lib` fixture operates at the new `_BUILD_DIR` path.

---

## SUC-003: Vendor-confinement grep gate catches regressions

- **Actor**: Developer / CI
- **Preconditions**: The simulation tier is green; `source/` is at its current state (Phase 0
  baseline — known leaks in `MotorController`, `DebugCommandable`).
- **Main Flow**:
  1. CI (or developer) runs the simulation tier pytest suite.
  2. The vendor-confinement test greps `source/` for forbidden tokens above `source/hal/`
     (Phase 0 boundary: `MicroBit.h`, `I2CBus`, `microbit_random`, OTOS `*Raw` int16 register
     access, Nezha split-phase patterns).
  3. The test compares the current hit-set against a committed allowlist/baseline.
  4. If the hit-set has GROWN (new files or lines above baseline), the test fails.
  5. If the hit-set is unchanged or smaller, the test passes.
- **Postconditions**: Any new vendor leak above `source/hal/` is caught automatically in
  the simulation tier before merge.
- **Acceptance Criteria**:
  - [ ] The canary test passes with the Phase 0 baseline committed alongside it.
  - [ ] Introducing a synthetic new leak (e.g. `#include "MicroBit.h"` in a control file)
        causes the test to fail.
  - [ ] The baseline reflects actual current hits (MotorController, DebugCommandable) so
        existing known leaks do not block Phase 0 green.

---

## SUC-004: defaultRobotConfig() field-pin prevents calibration drift

- **Actor**: Developer / CI
- **Preconditions**: `data/robots/tovez.json`, `scripts/gen_default_config.py`, and
  `source/robot/DefaultConfig.cpp` are at their current state; the host lib is built.
- **Main Flow**:
  1. CI (or developer) runs the simulation tier.
  2. The field-pin test builds the host lib and calls `defaultRobotConfig()` via the
     ctypes Sim wrapper.
  3. It compares the returned config fields against a committed golden snapshot.
  4. Any field that has changed causes the test to fail with a diff.
- **Postconditions**: Every subsequent migration phase must produce a field-pin diff of
  zero, or explicitly update the snapshot with documented rationale.
- **Acceptance Criteria**:
  - [ ] The field-pin test passes when `tovez.json` / `DefaultConfig.cpp` are unchanged.
  - [ ] Mutating a calibration field in `tovez.json` and rebuilding causes the test to fail.
  - [ ] The golden snapshot is committed in the repo alongside the test.

---

## SUC-005: Golden-TLM canary preserves behavior across migration phases

- **Actor**: Developer / CI
- **Preconditions**: The host lib is built; the `Sim` ctypes wrapper works; deterministic
  time stepping is available.
- **Main Flow**:
  1. CI (or developer) runs the simulation tier.
  2. The golden-TLM test drives the sim through a fixed, deterministic command sequence
     (fixed seed, stepped time — no wall-clock dependency).
  3. It captures the resulting TLM frame(s) and compares them byte-for-byte against a
     committed golden capture.
  4. Any difference in the TLM frame causes the test to fail with a diff.
- **Postconditions**: The golden-TLM canary is the behavior-preservation oracle. Passing
  it means no migration phase has silently changed motion output, calibration, or the
  telemetry format.
- **Acceptance Criteria**:
  - [ ] The canary passes against the committed golden capture.
  - [ ] Mutating a motion parameter or TLM field in the sim causes the test to fail.
  - [ ] The canary uses deterministic stepped time (no `time.sleep`, no wall-clock).
  - [ ] The golden capture is committed in the repo alongside the test.

---

## SUC-006: Coverage harness makes simulation-tier line coverage measurable

- **Actor**: Developer
- **Preconditions**: CMake ≥ 3.16, gcovr ≥ 8.6 available via uv, the simulation tier is green.
- **Main Flow**:
  1. Developer (or CI) invokes the coverage harness (a justfile target or script).
  2. The harness configures a coverage-instrumented build dir, builds
     `libfirmware_host` with `--coverage -O0 -g`, runs the simulation tier against it,
     then runs `gcovr --root source --print-summary <covdir>` to report line coverage.
  3. A single overall line-coverage percentage for `source/` is printed.
- **Postconditions**: Line coverage of the simulatable code is measurable and
  reproducible from a single command. No hard threshold is enforced yet.
- **Acceptance Criteria**:
  - [ ] The coverage harness produces a line-coverage percentage without errors.
  - [ ] Coverage numbers are reproducible (same test set → same percentage, modulo
        non-determinism that does not exist in the sim).
  - [ ] The harness script/target is committed in `tests/_infra/`.
