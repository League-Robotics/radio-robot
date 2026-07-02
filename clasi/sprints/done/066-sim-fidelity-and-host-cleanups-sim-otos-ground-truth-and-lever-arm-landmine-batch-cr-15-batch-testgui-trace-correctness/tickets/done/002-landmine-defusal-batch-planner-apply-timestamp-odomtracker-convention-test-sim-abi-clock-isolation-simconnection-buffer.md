---
id: '002'
title: 'Landmine defusal batch: Planner::apply timestamp, OdomTracker convention test,
  sim-ABI clock isolation, SimConnection buffer'
status: done
use-cases:
- SUC-003
- SUC-004
- SUC-005
- SUC-006
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

- [x] `Planner::apply()` takes a `uint32_t now_ms` parameter and uses it in
      place of the hard-coded `now=0` when calling every `begin*()`.
- [x] `tests/_infra/sim/planner_api.cpp`'s seven `apply_*` shim functions
      (`apply_velocity/stop/turn/timed/goto/distance/rotation`) thread a
      `now_ms` parameter through to `planner.apply(cmd, now_ms)`.
- [x] New guard test: a `PlannerCommand`-path TIMED goal staged via
      `planner_api_apply_timed(h, vx, omega, duration_ms)` with a realistic
      nonzero `now_ms`, ticked forward, runs its full duration before the
      TIME stop fires (not on the next tick).
- [x] New `OdomTracker` convention test: anchor at a known pose, feed a
      synthetic straight-ahead TLM track, assert `world_pos`/`world_yaw`
      match the expected aprilcam-frame (A1-centred, +x east, +y north)
      coordinates.
- [x] `tests/_infra/sim/sim_api.cpp`'s `g_sim_now_ms` becomes
      `thread_local uint32_t g_sim_now_ms = 0;`.
- [x] `SimHandle` records its constructing `std::thread::id`;
      `sim_tick()`/`sim_command()` assert the calling thread matches it.
- [x] New test (or documented manual verification, programmer's call):
      two sequential `Sim()` instances on different threads do not corrupt
      each other's clock.
- [x] `host/robot_radio/io/sim_conn.py`'s `SimConnection._raw_command`
      buffer: `ctypes.create_string_buffer(512)` → `2048`.
- [x] `GET CFG` via `SimConnection` returns complete, untruncated output
      (test or manual verification).
- [x] Full default test suite green.

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

## Implementation Notes (post-execution)

- **(b) genuine bug found and fixed, per the plan's own contingency.** The
  convention test revealed that `OdomTracker._to_world_mm()`/`world_yaw`
  implemented a CW-positive world transform, while (1) firmware TLM pose is
  already a proper Cartesian pose in firmware's own fixed frame — CCW-positive,
  0 = firmware +X, confirmed by reading `Odometry.cpp`'s
  `pose.x += d*cos(theta); pose.y += d*sin(theta)` integration — and (2)
  aprilcam's world frame is independently documented as CCW-positive,
  0 = east (`odometry.py`'s `_apply()`: "aprilcam.Tag.orientation is now
  world-CCW-positive — verified empirically 2026-05-28"; `testgui/canvas.py`:
  "In world space, yaw=0 is east... CCW world"). Empirically verified with a
  throwaway probe script before touching code: a robot anchored facing north
  (`world_yaw=+90°`) driving straight ahead moved *south* under the old code.
  `OdomTracker.world_yaw`/`.world_pos` have zero production consumers
  (confirmed by repo-wide grep — only `odom_tracker.py` itself and the test
  file reference them), so this is a zero-blast-radius fix. Per the plan's own
  authorization ("no production code change... unless the test reveals a
  genuine bug — not expected"), `_to_world_mm()` and `world_yaw` were corrected
  to a pure CCW rotation + translation; both the sign flip and the bogus
  "TLM x=right,y=forward body-frame" comment (contradicted by `Odometry.cpp`)
  were fixed. Verified by reverting to the old formula and re-running the new
  test — it fails as expected.
- **Existing-test fallout from the `apply_*` shim signature change.**
  `tests/simulation/unit/test_planner_subsystem.py` and
  `test_planner_subsystem_smoke.py` call the `planner_api_apply_*` shims
  directly via ctypes with explicit `argtypes`; both needed their `argtypes`
  lists and call sites updated to pass the new trailing `now_ms` argument
  (0, matching their own tick loops which all start at `t=0`). Without this,
  ctypes would call the new 5-arg `planner_api_apply_timed` (etc.) with only
  4 arguments, leaving `now_ms` as ABI-undefined — this was caught by the
  full-suite run (`test_timed_goal_twist_profile` failed) before being fixed;
  not called out explicitly in the architecture doc's Migration Concerns
  (which only tracked `planner_api.cpp`'s own C++ call site), but required to
  keep "full default test suite green."
- Every one of the four sub-fixes' new/updated tests was verified to
  genuinely fail against a temporarily-reintroduced copy of its bug (CR-11,
  CR-13, CR-14) or against the wrong sign convention (CR-12) before being
  confirmed passing against the fix — not just written to pass by
  construction.
