---
status: ready
---

# Sprint 049 Use Cases

## SUC-001: Vendor cmon-pid into the project as a float-adapted header

- **Actor**: Firmware build system (CODAL, ARM Cortex-M4F) and host simulation build system (CMake / gcc / x86)
- **Preconditions**: `libraries/` directory exists; both `CMakeLists.txt` (root CODAL build) and `tests/_infra/sim/CMakeLists.txt` (host sim build) are operational.
- **Main Flow**:
  1. The `libraries/cmon-pid/` directory is created containing `cmon-pid.h` (BSD-2-Clause, with all `double` occurrences mechanically converted to `float`) and `LICENSE`.
  2. The root `CMakeLists.txt` adds `libraries/cmon-pid/` to the firmware include search path.
  3. The sim `CMakeLists.txt` adds `libraries/cmon-pid/` to the host-build include search path.
  4. The vendored header compiles cleanly in both build environments with `-fno-exceptions -fno-rtti`, no STL, no heap allocation.
- **Postconditions**: Any source file in either build can `#include "cmon-pid.h"` and use `cmon_pid::ParallelPid` / `cmon_pid::backcalculation_t` without further include-path changes.
- **Acceptance Criteria**:
  - [ ] `libraries/cmon-pid/cmon-pid.h` and `libraries/cmon-pid/LICENSE` exist.
  - [ ] The vendored header contains no `double` occurrences; all floating-point types are `float`.
  - [ ] No STL, heap (`new`/`malloc`), exceptions, or RTTI in the vendored header.
  - [ ] `test_vendor_confinement.py` still passes (vendored code is in `libraries/`, outside the scanned `source/` directories).
  - [ ] Sim build compiles with the new include path wired in.

## SUC-002: VelocityController uses cmon-pid for its integral/derivative/anti-windup core

- **Actor**: MotorController (firmware and host simulation) invoking VelocityController::update() per control tick
- **Preconditions**: SUC-001 complete; cmon-pid is available on the include path.
- **Main Flow**:
  1. `VelocityController` holds a `cmon_pid::backcalculation_t<cmon_pid::pid_bwe>` instance constructed from `velKp`, `velKi`, `velKaw`, and `velIMax`.
  2. Each call to `VelocityController::update(setpoint, measured, dt_s)` computes `error = setpoint - measured`, then delegates integral accumulation and anti-windup to cmon-pid's `Update(error, dt_s)`.
  3. The thin wrapper computes feed-forward (`kFF * |setpoint|`), applies the sign of `setpoint`, and applies the deadband gate (integrator not driven when `|setpoint| < minWheelMms`).
  4. The final PWM output is clamped to `[-100, +100]`.
  5. `VelocityController::reset()` resets cmon-pid's internal state.
- **Postconditions**: Per-wheel velocity control behaviour is numerically equivalent to the previous hand-rolled PI loop for the same gain set; `velKp`, `velKi`, `velIMax`, `velKaw`, `velKff`, `minWheelMms` all retain their current semantics and wire keys.
- **Acceptance Criteria**:
  - [ ] `VelocityController.cpp` includes `cmon-pid.h` and contains no hand-rolled integrator arithmetic.
  - [ ] `test_velocity_controller.py` passes with no new failures.
  - [ ] `test_motor_controller.py` passes with no new failures.
  - [ ] `test_body_velocity_controller.py` passes with no new failures.
  - [ ] Public API of `VelocityController` (constructor signature, `update()`, `reset()`, public fields) is unchanged.

## SUC-003: Dead code RatioPidController and pid.* config keys are removed

- **Actor**: Developer / code maintainer
- **Preconditions**: `RatioPidController` is confirmed dead (its `update()` is never called; only `pid.*` config fields and SET/GET registry entries reference it).
- **Main Flow**:
  1. `source/control/RatioPidController.h` and `source/control/RatioPidController.cpp` are deleted.
  2. `source/control/RatioPidController.cpp` is removed from any CMake source lists that enumerate it explicitly (it is currently picked up by glob, so deletion removes it automatically).
  3. `ratioPidKp`, `ratioPidKi`, `ratioPidKd`, `ratioPidMax` fields are deleted from `source/types/Config.h`.
  4. The four `pid.*` entries (`pid.kp`, `pid.ki`, `pid.kd`, `pid.max`) are removed from the `kRegistry[]` table in `source/robot/ConfigRegistry.cpp`.
  5. `source/robot/DefaultConfig.cpp` ratioPid initializer lines are deleted.
  6. `tests/simulation/unit/test_ratio_pid.py` is deleted (it is a bench script, not a pytest test, but it lives in the sim unit directory and references dead controller code).
  7. Any other test files that reference `pid.*` keys purely to verify the dead-code retention note (N13) are updated to remove or comment out those assertions, since the keys no longer exist.
- **Postconditions**: No reference to `RatioPidController` or `pid.*` config keys remains in `source/`; sim test suite runs clean.
- **Acceptance Criteria**:
  - [ ] `source/control/RatioPidController.{h,cpp}` are absent from the repo.
  - [ ] `source/types/Config.h` contains no `ratioPid*` fields.
  - [ ] `source/robot/ConfigRegistry.cpp` contains no `pid.kp`, `pid.ki`, `pid.kd`, `pid.max` entries.
  - [ ] `source/robot/DefaultConfig.cpp` contains no `ratioPid` initializers.
  - [ ] `tests/simulation/unit/test_ratio_pid.py` is deleted.
  - [ ] All N13-referencing test assertions that verify `pid.*` key retention are removed.
  - [ ] `uv run --with pytest python -m pytest tests/simulation -q` introduces no new failures beyond the 2 pre-existing baseline failures.

## SUC-004: Full sim suite validates Phase A with no regressions

- **Actor**: CI / developer running the simulation test suite
- **Preconditions**: SUC-001, SUC-002, and SUC-003 are complete; tree is compilable with both the ARM firmware build and the host sim build.
- **Main Flow**:
  1. Developer runs `uv run --with pytest python -m pytest tests/simulation -q`.
  2. All velocity controller, motor controller, body velocity controller, and vendor confinement tests pass.
  3. Exactly the 2 pre-existing failures remain (config-schema drift in `test_default_config_pin.py` and `test_robot_config.py`); no new failures appear.
  4. `test_vendor_confinement.py` reports zero hits above `source/io/`.
- **Postconditions**: The sprint is validated. The firmware tree is ready for Phase B (Sprint 050).
- **Acceptance Criteria**:
  - [ ] `uv run --with pytest python -m pytest tests/simulation -q` exits with exactly the 2 pre-existing failures.
  - [ ] `test_vendor_confinement_zero_hits_empty_baseline` passes.
  - [ ] No `RatioPidController` references remain in `source/`.
  - [ ] `test_velocity_controller.py`, `test_motor_controller.py`, and `test_body_velocity_controller.py` are all green.
