---
id: '004'
title: 'Parity gate: verify EKFTiny passes test_ekf.py in full with no new suite failures'
status: done
use-cases:
- SUC-002
depends-on:
- 050-003
github-issue: ''
issue: consolidate-control-code-onto-vendored-libraries-cmon-pid-tinyekf.md
completes_issue: false
---

# Parity gate: verify EKFTiny passes test_ekf.py in full with no new suite failures

## Description

This is the critical parity gate for Phase B. Wire `EKFTiny` into the simulation
layer's test harness so that `tests/simulation/unit/test_ekf.py` exercises it
instead of (or alongside) the old `EKF`, and confirm that every test in that file
passes. The Python mirror in `test_ekf.py` is the numerical oracle — all state
transitions, covariance updates, gating thresholds, and D3 recovery must produce
results within float tolerance.

**The parity gate is the only acceptance criterion that matters for this ticket.**
If any test in `test_ekf.py` fails, it must be fixed in `EKFTiny.cpp` before this
ticket is accepted. Do NOT accept this ticket with failing EKF parity tests.

### How to wire EKFTiny into the sim layer

The sim build exposes a C API (`tests/_infra/sim/sim_api.cpp`) that the Python
`firmware.py` wrapper calls via ctypes. The EKF tests in `test_ekf.py` call
`sim.sim_ekf_*` functions. These functions currently construct an `EKF` object.

The approach: add parallel `sim_ekftiny_*` C API functions in `sim_api.cpp` that
create and exercise an `EKFTiny` object, then extend `test_ekf.py` with a
parametrized fixture that runs the same test cases against both `EKF` (existing)
and `EKFTiny` (new). Alternatively, if the sim API can be switched to `EKFTiny`
cleanly, replace the existing `sim_ekf_*` functions.

The simpler approach: add a second set of `sim_ekftiny_*` functions and add a
pytest parametrize or duplicate fixture. The key requirement is that **all tests in
`test_ekf.py` pass for both the old EKF and EKFTiny** (or for EKFTiny alone if the
sim API is switched).

**If numerical parity fails** (a test asserts float equality and the Cholesky-based
`invert()` in `ekf_update` produces a different result from the analytic 2x2 inverse
for the position channel), resolve it in `EKFTiny.cpp` by computing S⁻¹ analytically
(as the current code does) and applying the Kalman gain manually without calling
`ekf_update`, then calling only the `_mulmat`-based state/P update logic. Document
the resolution in a comment in `EKFTiny.cpp`.

### Known pre-existing baseline

Running `uv run --with pytest python -m pytest tests/simulation -q` on master
produces exactly 2 failures:
- `tests/simulation/unit/test_default_config_pin.py::test_default_robot_config_unchanged`
- `tests/simulation/unit/test_robot_config.py::TestSchemaValidation::test_tovez_validates_against_schema`

These are config-schema drift failures unrelated to EKF. Do NOT fix them. The gate
for this ticket is: test_ekf.py passes in full AND the overall suite shows no new
failures beyond those 2.

## Acceptance Criteria

- [x] `tests/simulation/unit/test_ekf.py` passes in full — zero failures in that file.
- [x] `uv run --with pytest python -m pytest tests/simulation -q` shows exactly 2 failures (the pre-existing config-schema tests) and no others.
- [x] If numerical parity required a fix in `EKFTiny.cpp` (e.g., analytic S⁻¹ for position channel), the fix is commented with a note explaining why `ekf_update` was bypassed for that channel.
- [x] `source/state/EKF.{h,cpp}` remain untouched.
- [x] `source/control/Odometry.{h,cpp}` remain untouched.

## Implementation Plan

### Approach

1. Inspect `sim_api.cpp` to understand the existing `sim_ekf_*` function pattern.
2. Add `sim_ekftiny_*` functions (or replace `sim_ekf_*`) that exercise `EKFTiny`.
3. Update `firmware.py` ctypes wrapper if needed for new function names.
4. Run `test_ekf.py` and compare outputs.
5. If any test fails due to numerical difference, diagnose the delta, fix in `EKFTiny.cpp`, repeat.

### Files likely modified

- `tests/_infra/sim/sim_api.cpp` — add or replace sim_ekf* functions for EKFTiny
- `tests/simulation/unit/test_ekf.py` — extend if needed for EKFTiny fixture
- `tests/_infra/sim/firmware.py` (the ctypes wrapper) — if new sim_api functions are added
- `source/state/EKFTiny.cpp` — bug fixes if parity failures surface

### Testing plan

**Primary verification:** `uv run --with pytest python -m pytest tests/simulation/unit/test_ekf.py -v`

This must show all tests passing (0 failures). Then run the full suite:

**Full suite:** `uv run --with pytest python -m pytest tests/simulation -q`

Expected: exactly 2 failures (config-schema), 0 new.

### Documentation updates

None required beyond in-code comments if a Cholesky bypass is applied.
