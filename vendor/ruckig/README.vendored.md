# Vendored: Ruckig (community version)

Jerk-limited online trajectory generation. Used by `Motion::JerkTrajectory`
(`src/firm/motion/`) to produce motion plans that decelerate to **rest at the
target** (or a nonzero target velocity, `solveToState()`) and never cross zero
velocity into a reverse spin — see
[`clasi/sprints/109-firmware-jerk-limited-motion-ruckig-return-arc-command-queue/`](../../clasi/sprints/109-firmware-jerk-limited-motion-ruckig-return-arc-command-queue/).

**Restored 2026-07-17 (sprint 109 ticket 001)**, unchanged in content, from
`git show c63ec6c:libraries/ruckig` — the pre-102 vendored tree that sprints
102-107's greenfield single-loop rebuild deleted wholesale (no call site
survived the rebuild, so it was removed with its only consumer). Moved from
the old `libraries/ruckig/` path (gitignored, dependency-fetch destination)
to this repo's own repo-root `vendor/ruckig/` (a REAL, git-tracked
directory) as part of the restore; no other content changed from the
original vendoring below.

**Not under `src/vendor/`:** this repo's `src/vendor` is a tracked symlink
to a shared reference pool in an unrelated sibling checkout (see
`clasi/issues/vendor-symlink-not-reproducible-fresh-clone.md`) — fine for
reference-only material (PurePursuit, PythonRobotics, pxt SDKs) that
nothing compiles against, but not reproducible from this repo's own git
history alone, which a compiled-in firmware dependency requires. Ruckig
lives at the repo root instead, precisely so `git clone` + this repo's own
history is sufficient to build it.

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
[`src/tests/sim/unit/test_jerk_trajectory.py`](../../src/tests/sim/unit/test_jerk_trajectory.py)
(+ `jerk_trajectory_harness.cpp`), which compiles this vendored library
together with `Motion::JerkTrajectory`.

The firmware builds at C++20 via a `-std=c++20` override in the repo-root
`CMakeLists.txt` (last `-std` wins over the vendored codal target's c++11); the
host sim uses `CMAKE_CXX_STANDARD 20`.

## Build integration (originally sprint 089 ticket 001; restored sprint 109
## ticket 001)

`src/*.cpp` is compiled into **both** real build targets — appended to each
build's existing flat source list, not a separate CMake target (no upstream
build file was vendored to build one from):

- The **ARM firmware image** (repo-root `CMakeLists.txt`): a bare
  `include_directories()` for `vendor/ruckig/include` plus a
  `file(GLOB RUCKIG_SOURCES "vendor/ruckig/src/*.cpp")` /
  `list(APPEND SOURCE_FILES ${RUCKIG_SOURCES})` before the
  "no application to build" guard.
- The **host-sim `firmware_host` target** (`src/sim/CMakeLists.txt`): an
  explicit `RUCKIG_SOURCES` list (this build's convention is an explicit
  list, not a glob — see that file's own header comment) plus
  `vendor/ruckig/include` added to
  `target_include_directories(firmware_host PRIVATE ...)`.

**Flash/RAM footprint, original vendoring (measured 2026-07-07, `just
build-clean`, nRF52833 — 512 KB flash / 128 KB RAM shared with CODAL):**

- As integrated by the original ticket (compiled in, but **no call site
  anywhere in `source/`** yet): **zero** flash/RAM delta.
  `-Wl,--gc-sections` (already in the vendored codal `target.json`'s linker
  flags) discards every Ruckig object entirely since nothing references it;
  confirmed byte-identical `arm-none-eabi-size` output before/after that
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
  regardless of how many call sites subsequently reuse the same
  `Ruckig<1>` instantiation.

**Restore (2026-07-17, sprint 109 ticket 001):** this vendoring is
byte-identical to the original; see this ticket's completion notes for the
current `arm-none-eabi-size` before/after baseline (`Motion::JerkTrajectory`
is a real call site this time, so the worst-case delta above is expected to
actually land, not stay dead-code-eliminated).

## Updating

Re-copy `include/ruckig/` and the 11 `src/ruckig/*.cpp` above from a new upstream
tag, keep this file's commit hash current, and re-run the smoke test. Do not pull
in the cloud client / third_party unless we move to Ruckig Pro.
