---
id: '002'
title: 'Landmine defusal batch: Planner::apply timestamp, OdomTracker convention test,
  sim-ABI clock isolation, SimConnection buffer'
status: open
use-cases: [SUC-003, SUC-004, SUC-005, SUC-006]
depends-on: []
github-issue: ''
issue: landmine-cleanups-planner-apply-now0-sim-abi-buffers.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Landmine defusal batch: Planner::apply timestamp, OdomTracker convention test, sim-ABI clock isolation, SimConnection buffer

## Description

Four independent, medium-severity latent defects
(`clasi/issues/landmine-cleanups-planner-apply-now0-sim-abi-buffers.md`,
CR-11..14), none reachable by any test today (confirmed by reading every
call site, not assumed) but each a landmine for whoever next touches the
surrounding code:

- (a) `Planner::apply()` hard-codes `now=0`, which becomes every subsequent
  `MotionCommand`'s `t0Ms` baseline — a future `PlannerCommand`-path TIME
  stop would fire instantly.
- (b) `OdomTracker`'s TLM→world-frame transform has never been anchored to
  the aprilcam world convention by test.
- (c) The sim C-ABI's `g_sim_now_ms` is a process-global clock; a second
  `SimHandle` on a different thread (e.g. `SimTransport` reconnect racing a
  slow-exiting prior tick-thread) can yank it backwards for the still-live
  instance.
- (d) `SimConnection._raw_command` truncates replies at 512 bytes against a
  2048-byte C-side buffer.

See `architecture-update.md` §"Landmine defusal batch (CR-11/12/13/14)" and
Design Rationale Decision 4 (thread_local clock, not a threaded parameter)
for the full design.

## Acceptance Criteria

- [ ] `Planner::apply()` takes a `uint32_t now_ms` parameter and uses it in
      place of the hard-coded `now=0` when calling every `begin*()`.
- [ ] `tests/_infra/sim/planner_api.cpp`'s seven `apply_*` shim functions
      (`apply_velocity/stop/turn/timed/goto/distance/rotation`) thread a
      `now_ms` parameter through to `planner.apply(cmd, now_ms)`.
- [ ] New guard test: a `PlannerCommand`-path TIMED goal staged via
      `planner_api_apply_timed(h, vx, omega, duration_ms)` with a realistic
      nonzero `now_ms`, ticked forward, runs its full duration before the
      TIME stop fires (not on the next tick).
- [ ] New `OdomTracker` convention test: anchor at a known pose, feed a
      synthetic straight-ahead TLM track, assert `world_pos`/`world_yaw`
      match the expected aprilcam-frame (A1-centred, +x east, +y north)
      coordinates.
- [ ] `tests/_infra/sim/sim_api.cpp`'s `g_sim_now_ms` becomes
      `thread_local uint32_t g_sim_now_ms = 0;`.
- [ ] `SimHandle` records its constructing `std::thread::id`;
      `sim_tick()`/`sim_command()` assert the calling thread matches it.
- [ ] New test (or documented manual verification, programmer's call):
      two sequential `Sim()` instances on different threads do not corrupt
      each other's clock.
- [ ] `host/robot_radio/io/sim_conn.py`'s `SimConnection._raw_command`
      buffer: `ctypes.create_string_buffer(512)` → `2048`.
- [ ] `GET CFG` via `SimConnection` returns complete, untruncated output
      (test or manual verification).
- [ ] Full default test suite green.

## Implementation Plan

**Approach:** Four independent sub-fixes; implement and test each in
sequence within this one ticket (no shared state between them, but grouped
per the sprint's own bundling decision — see architecture-update.md Step
1-2).

1. **Planner::apply timestamp**: add the `now_ms` parameter, remove the
   hard-coded local. Update the seven `planner_api.cpp` shims. Write the
   guard test using the existing `test_059_bus_drain.py`/
   `test_059_config_routing.py` pattern (ctypes handle + tick loop) as a
   model — neither of those files currently exercises `apply_timed`, so this
   is new coverage, not a modification of theirs.
2. **OdomTracker convention test**: add to
   `tests/simulation/unit/test_sensors_v2.py` (the one existing consumer) or
   a new sibling test file — programmer's call. No production code change to
   `odom_tracker.py` unless the test reveals a genuine bug (not expected —
   the issue calls this "untested," not "wrong").
3. **Sim clock isolation**: `thread_local` on `g_sim_now_ms`; add
   `std::thread::id _ownerThread` to `SimHandle`, set in its constructor,
   asserted in `sim_tick()`/`sim_command()`. Requires `#include <thread>` in
   `sim_api.cpp` (host-test-only file, no ARM-target impact).
4. **SimConnection buffer**: one-line size change in `sim_conn.py`.

**Files to modify:**
- `source/superstructure/Planner.h`, `source/superstructure/Planner.cpp`
- `tests/_infra/sim/planner_api.cpp`
- `tests/_infra/sim/sim_api.cpp`
- `host/robot_radio/io/sim_conn.py`

**Files to create:**
- A new `OdomTracker` convention test (new file or addition to
  `test_sensors_v2.py`)
- A new `Planner::apply` TIME-stop guard test (new file or addition to
  `test_059_bus_drain.py`/`test_059_config_routing.py`)

**Testing plan:**
- Existing tests to run: `tests/simulation/unit/test_059_config_routing.py`,
  `test_059_bus_drain.py`, `test_sensors_v2.py`, `test_queue_invariant.py`
  (touches `g_sim_now_ms`/`sim_create` — confirm no regression), full
  default suite.
- New tests: as listed in Acceptance Criteria.
- Verification command: `uv run --with pytest python -m pytest -q`.

**Documentation updates:** None beyond this ticket and
`architecture-update.md`.
