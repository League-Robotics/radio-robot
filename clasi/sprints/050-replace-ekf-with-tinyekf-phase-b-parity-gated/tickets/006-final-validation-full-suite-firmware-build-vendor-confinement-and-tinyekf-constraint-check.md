---
id: '006'
title: 'Final validation: full suite, firmware build, vendor confinement, and TinyEKF constraint check'
status: open
use-cases: ["SUC-004"]
depends-on: ["050-005"]
github-issue: ''
issue: consolidate-control-code-onto-vendored-libraries-cmon-pid-tinyekf.md
completes_issue: true
---

# Final validation: full suite, firmware build, vendor confinement, and TinyEKF constraint check

## Description

Sprint 050 completion gate. Run all validation checks after the swap is live and
the old EKF is deleted. This ticket is the confirmation that Phase B is complete
and the `consolidate-control-code-onto-vendored-libraries-cmon-pid-tinyekf.md`
issue is fully resolved (Phase A done in 049, Phase B done in 050).

This ticket writes no code and makes no file changes. It is purely a validation
and sign-off ticket.

### Checks to run in order

**1. Full simulation test suite:**
```
uv run --with pytest python -m pytest tests/simulation -q
```
Expected: exactly 2 failures (the pre-existing config-schema tests below), 0 new.
Pre-existing failures that are NOT regressions:
- `tests/simulation/unit/test_default_config_pin.py::test_default_robot_config_unchanged`
- `tests/simulation/unit/test_robot_config.py::TestSchemaValidation::test_tovez_validates_against_schema`

**2. test_ekf.py explicitly:**
```
uv run --with pytest python -m pytest tests/simulation/unit/test_ekf.py -v
```
Expected: all pass, 0 failures.

**3. test_vendor_confinement.py explicitly:**
```
uv run --with pytest python -m pytest tests/simulation/unit/test_vendor_confinement.py -v
```
Expected: both `test_vendor_confinement_no_new_leaks` and
`test_vendor_confinement_zero_hits_empty_baseline` pass.

**4. Firmware build:**
```
python build.py --clean
```
Expected: exits 0; both firmware (MICROBIT.hex) and host-sim artifacts produced.

**5. TinyEKF constraint manual check** — verify in `libraries/tinyekf/tinyekf.h`
that none of the following appear in the file body (below the preamble):
- `#include <vector>`, `#include <string>`, `#include <iostream>` or any STL header
- `new `, `delete `, `throw `, `typeid`, `dynamic_cast`
- `#include "MicroBit.h"`, `#include "I2CBus.h"`, `#include "microbit_random.h"`

Confirm: tinyekf.h includes only `<math.h>`, `<stdbool.h>`, `<string.h>`.

**6. EKFTiny constraint manual check** — verify in `source/state/EKFTiny.{h,cpp}`:
- None of the forbidden tokens from check 5.
- `#define EKF_N 5` and `#define EKF_M 2` appear before `#include <tinyekf.h>` in EKFTiny.cpp.

**7. Old EKF deletion check:**
- `source/state/EKF.h` does not exist.
- `source/state/EKF.cpp` does not exist.
- `grep -rn "EKF.h" source/` returns zero hits.

## Acceptance Criteria

- [ ] `uv run --with pytest python -m pytest tests/simulation -q` — exactly 2 failures (pre-existing config-schema), 0 new.
- [ ] `uv run --with pytest python -m pytest tests/simulation/unit/test_ekf.py -v` — all pass, 0 failures.
- [ ] `uv run --with pytest python -m pytest tests/simulation/unit/test_vendor_confinement.py -v` — both tests pass.
- [ ] `python build.py --clean` exits 0.
- [ ] `libraries/tinyekf/tinyekf.h` contains no STL, heap, exception, RTTI, or CODAL headers.
- [ ] `source/state/EKFTiny.{h,cpp}` contain no STL, heap, exception, RTTI, or CODAL headers.
- [ ] `#define EKF_N 5` and `#define EKF_M 2` are present in `EKFTiny.cpp` before `#include <tinyekf.h>`.
- [ ] `source/state/EKF.h` and `source/state/EKF.cpp` do not exist.
- [ ] `grep -rn '"state/EKF.h"' source/` returns zero hits.

## Implementation Plan

### Approach

Run all checks in order. If any check fails, fix the underlying cause (in a prior
ticket's code if needed) and re-run from the beginning of the check list. This
ticket has no code changes of its own.

### Testing plan

All checks listed above. No new test code is written.

**Primary command:** `uv run --with pytest python -m pytest tests/simulation -q`

### Documentation updates

None. The sprint itself is the documentation of Phase B completion.

## Issue Back-Reference

This ticket resolves `consolidate-control-code-onto-vendored-libraries-cmon-pid-tinyekf.md`.
Phase A (cmon-pid/PID) was completed in Sprint 049. Phase B (TinyEKF/EKF) is
completed by this sprint. With both phases done, the umbrella issue is fully resolved.
