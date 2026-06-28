---
status: ready
---

# Sprint 050 Use Cases

## SUC-001: Vendor TinyEKF into the repository

- **Actor**: Build system / programmer
- **Preconditions**: `libraries/cmon-pid/` pattern exists from Sprint 049; local TinyEKF clone is at the scratchpad path.
- **Main Flow**:
  1. Programmer copies `tinyekf.h` and `LICENSE` from the local upstream clone into `libraries/tinyekf/`.
  2. The file receives a provenance preamble documenting upstream URL, commit hash, and date.
  3. `EKF_N=5` and `EKF_M=2` are defined before the include in every consumer; tinyekf.h uses these compile-time constants.
  4. The include directory `libraries/tinyekf` is added to both root `CMakeLists.txt` and `tests/_infra/sim/CMakeLists.txt` following the cmon-pid pattern from Sprint 049.
- **Postconditions**: `libraries/tinyekf/tinyekf.h` is present; both build paths resolve the header without modification to tinyekf.h itself.
- **Acceptance Criteria**:
  - [ ] `libraries/tinyekf/tinyekf.h` exists with provenance preamble (URL, commit `ba0d2a90e22f2f4e4120a91cb05ca2ca6a8e8da3`, MIT).
  - [ ] `libraries/tinyekf/LICENSE` exists.
  - [ ] Root `CMakeLists.txt` includes `libraries/tinyekf` via `include_directories`, mirroring the cmon-pid entry.
  - [ ] `tests/_infra/sim/CMakeLists.txt` adds `${REPO_ROOT}/libraries/tinyekf` to `target_include_directories(firmware_host PRIVATE ...)`.
  - [ ] `uv run --with pytest python -m pytest tests/simulation -q` still passes (zero new failures beyond the 2 pre-existing baseline failures).

## SUC-002: Build EKF as a thin layer over ekf_t (parity-gated, behind old implementation)

- **Actor**: Programmer / CI
- **Preconditions**: TinyEKF is vendored (SUC-001 complete); existing `EKF.{h,cpp}` in `source/state/` and `test_ekf.py` are untouched.
- **Main Flow**:
  1. Programmer adds `EKFTiny.h` and `EKFTiny.cpp` to `source/state/` implementing the same public API as `EKF` but backed by `ekf_t` from TinyEKF for the predict/update linear algebra.
  2. All robustness layers are preserved: arc-segment motion model (fx, F Jacobian), three measurement update channels (position M=2, heading M=1, velocity sequential scalars), per-channel Mahalanobis chi-squared gating (innovation y and S = H P Hᵀ + R computed before calling `ekf_update`), D3 gate-recovery via direct `ekf.P[]` writes, and wedge-aware omega suppression.
  3. A Python parity oracle test is added (or the existing `test_ekf.py` is extended) that exercises `EKFTiny` via the simulation build and confirms numerical agreement with the Python EKF mirror.
  4. The old `EKF.{h,cpp}` files remain untouched; no production call-sites are changed in this ticket.
- **Postconditions**: `EKFTiny` compiles in both the ARM firmware build and the host-sim build; `tests/simulation/unit/test_ekf.py` passes in full (zero failures in that file); overall suite shows no new failures beyond the 2 pre-existing baseline.
- **Acceptance Criteria**:
  - [ ] `source/state/EKFTiny.h` and `source/state/EKFTiny.cpp` exist with the same public API as `EKF`.
  - [ ] `EKFTiny` uses `ekf_t` fields (`x[]`, `P[]`) and calls `ekf_initialize`, `ekf_predict`, `ekf_update` from tinyekf.h for all LA operations.
  - [ ] Hand-unrolled matrix arithmetic is deleted from `EKFTiny.cpp`; robustness layers (gating, D3, wedge suppression) are present.
  - [ ] `EKF_N=5` and `EKF_M=2` are defined at the top of `EKFTiny.cpp` before the tinyekf.h include.
  - [ ] `tests/simulation/unit/test_ekf.py` passes in full against a build that includes `EKFTiny`.
  - [ ] `uv run --with pytest python -m pytest tests/simulation -q` shows no new failures beyond the 2 pre-existing baseline.

## SUC-003: Swap PhysicalStateEstimate and Odometry to use EKFTiny; delete old EKF internals

- **Actor**: Programmer / CI
- **Preconditions**: EKFTiny passes parity (SUC-002 accepted); old `EKF.{h,cpp}` still exist.
- **Main Flow**:
  1. `Odometry.h` changes `EKF _ekf` field to `EKFTiny _ekf`; `#include "state/EKF.h"` becomes `#include "state/EKFTiny.h"`.
  2. No call-site changes needed — `EKFTiny` has identical public API.
  3. Old `source/state/EKF.h` and `source/state/EKF.cpp` are deleted.
  4. Full test suite passes; firmware builds clean.
- **Postconditions**: Production code uses TinyEKF-backed EKF exclusively; hand-unrolled matrix arithmetic is gone from the codebase.
- **Acceptance Criteria**:
  - [ ] `source/state/EKF.h` and `source/state/EKF.cpp` no longer exist.
  - [ ] `Odometry.h` includes `state/EKFTiny.h` and declares `EKFTiny _ekf`.
  - [ ] `python build.py --clean` produces firmware and host-sim binaries without error.
  - [ ] `uv run --with pytest python -m pytest tests/simulation -q` shows no new failures beyond the 2 pre-existing baseline.
  - [ ] `test_vendor_confinement.py` remains green.

## SUC-004: Final validation — confinement, builds, and full suite green

- **Actor**: CI / programmer
- **Preconditions**: SUC-003 complete; all prior tickets accepted.
- **Main Flow**:
  1. Full simulation test suite runs; only the 2 pre-existing baseline failures appear.
  2. Firmware build (`python build.py --clean`) completes cleanly.
  3. Vendor-confinement test passes (zero new hits above `source/io/`).
  4. TinyEKF confinement satisfied: `libraries/tinyekf/tinyekf.h` uses no heap, no STL, no exceptions, no RTTI, float arithmetic only, no CODAL headers.
- **Postconditions**: Sprint 050 is ready to close; the consolidate-libs issue is complete (Phase A done in 049, Phase B done in 050).
- **Acceptance Criteria**:
  - [ ] `uv run --with pytest python -m pytest tests/simulation -q` — exactly 2 failures (the pre-existing config-schema ones), no others.
  - [ ] `python build.py --clean` exits 0.
  - [ ] `test_vendor_confinement.py` passes (zero hits in both assertions).
  - [ ] `libraries/tinyekf/tinyekf.h` contains no `#include <vector>`, `#include <string>`, `new`, `delete`, `throw`, `typeid`, `dynamic_cast`, or CODAL headers.
