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

## Build integration (sprint 089 ticket 001)

`src/*.cpp` is compiled into **both** real build targets — appended to each
build's existing flat source list, not a separate CMake target (no upstream
build file was vendored to build one from):

- The **ARM firmware image** (repo-root `CMakeLists.txt`): a bare
  `include_directories()` for `libraries/ruckig/include` (mirroring the
  `cmon-pid`/`tinyekf` header-only pattern already there) plus a
  `file(GLOB RUCKIG_SOURCES "libraries/ruckig/src/*.cpp")` /
  `list(APPEND SOURCE_FILES ${RUCKIG_SOURCES})` before the
  "no application to build" guard.
- The **host-sim `firmware_host` target**
  (`tests/_infra/sim/CMakeLists.txt`): the same glob appended to
  `FIRMWARE_SOURCES`, plus `libraries/ruckig/include` added to
  `target_include_directories(firmware_host PRIVATE ...)`.

**Flash/RAM footprint (measured 2026-07-07, `just build-clean`, nRF52833 —
512 KB flash / 128 KB RAM shared with CODAL):**

- As integrated by ticket 001 (compiled in, but **no call site anywhere in
  `source/`** — ticket 002 is the first consumer): **zero** flash/RAM delta.
  `-Wl,--gc-sections` (already in the vendored codal `target.json`'s linker
  flags) discards every Ruckig object entirely since nothing references it;
  confirmed byte-identical `arm-none-eabi-size` output before/after this
  ticket's CMake change (FLASH 177764 B / 47.69%, RAM 120768 B / 98.33%).
- **Worst case once something calls it** (measured via a temporary,
  uncommitted scratch probe — a single `Ruckig<1>::calculate()` call wired
  into `main()`, then reverted): **+~151.5 KB flash** (177764 B -> 329276 B,
  47.69% -> 88.34% of the 364 KB FLASH region), **~0 RAM delta** (Ruckig's
  working state is fully stack-local, no heap, no added statics). The jump
  is real and load-bearing for planning: linking in *any* call to
  `Ruckig<1>::calculate()` pulls in the whole quartic/quintic
  position/velocity step-solver code (`position_third_step2.cpp` alone is
  55 KB of source), not just the code path a given input happens to take —
  and this is a **one-time fixed cost**, paid once for the first call site
  regardless of how many Planner call sites subsequently reuse the same
  `Ruckig<1>` instantiation. **This is flagged as a real concern**: it
  leaves only ~43 KB (11.7%) FLASH headroom once ticket 002 lands, which
  every later ticket in this sprint must budget against.

## Updating

Re-copy `include/ruckig/` and the 11 `src/ruckig/*.cpp` above from a new upstream
tag, keep this file's commit hash current, and re-run the smoke test. Do not pull
in the cloud client / third_party unless we move to Ruckig Pro.
