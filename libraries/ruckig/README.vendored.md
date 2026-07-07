# Vendored: Ruckig (community version)

Jerk-limited online trajectory generation. Used by the Planner to produce motion
plans that decelerate to **rest at the target** and never cross zero velocity
into a reverse spin — see
[`clasi/issues/planner-motion-planning-via-vendored-ruckig.md`](../../clasi/issues/planner-motion-planning-via-vendored-ruckig.md).

- **Upstream:** <https://github.com/pantor/ruckig>
- **License:** MIT (see `LICENSE`) — community version.
- **Vendored commit:** `2249d57ffaa19ecdadeaab62daf97857813629ff` (2026-07-07).

## What was vendored (and what was left out)

Kept the minimal C++ library:

- `include/ruckig/*.hpp` — all 16 headers.
- `src/*.cpp` — the **11 core** state-to-state solver sources (the
  `add_library(ruckig …)` list in upstream's `CMakeLists.txt`): `brake.cpp`,
  `position_first/second/third_step{1,2}.cpp`, `velocity_second/third_step{1,2}.cpp`.

Deliberately **excluded**:

- `src/ruckig/cloud_client.cpp` and `third_party/` (httplib, nlohmann/json) — the
  **cloud** waypoint client (upstream `BUILD_CLOUD_CLIENT`). We do **not** define
  `WITH_CLOUD_CLIENT` and use **state-to-state** trajectories only, so the local
  solve is fully offline (community waypoint planning would need the cloud/Pro).
- `src/wrapper/` (Rust/Python bindings), `examples/`, `test/`, `doc/`,
  `Cargo.*`, `pyproject.toml`.

## Build constraints (verified)

Compiles and runs under the firmware's **exact** flags — **C++20**,
`-fno-exceptions`, `-fno-rtti` — using the compile-time-DoF form `Ruckig<N>`
(`std::array`, **no heap**). Ruckig's `throw_error` template parameter defaults
to `false`, so input validation returns `Result` **codes** instead of throwing
(required under `-fno-exceptions`). Proven by
[`tests/sim/unit/test_ruckig_smoke.py`](../../tests/sim/unit/test_ruckig_smoke.py)
(+ `ruckig_smoke_harness.cpp`).

The firmware builds at C++20 via a `-std=c++20` override in the repo-root
`CMakeLists.txt` (last `-std` wins over the vendored codal target's c++11); the
host sim uses `CMAKE_CXX_STANDARD 20`.

## Updating

Re-copy `include/ruckig/` and the 11 `src/ruckig/*.cpp` above from a new upstream
tag, keep this file's commit hash current, and re-run the smoke test. Do not pull
in the cloud client / third_party unless we move to Ruckig Pro.
