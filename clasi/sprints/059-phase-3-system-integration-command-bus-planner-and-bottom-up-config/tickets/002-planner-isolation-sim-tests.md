---
id: '002'
title: Planner-isolation sim tests
status: open
use-cases:
- SUC-001
- SUC-002
depends-on:
- 059-001
github-issue: ''
issue: message-based-subsystem-architecture.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Planner-isolation sim tests

## Description

Write the planner-isolation sim test suite in
`tests/simulation/unit/test_planner_subsystem.py`. This is the key
stakeholder-requested test: construct `MotionController2` with an injected
`Drive2` (on `SimHardware`), feed user goals via `apply()`, call `tick()` N
times, and **assert the RETURNED `CommandBatch` of `DrivetrainCommand{twist}`**.
No robot, no comms — the RETURN model makes this a pure function of goal + injected
pose.

This test proves the Planner's guidance logic independently of the loop wiring and
is the automated acceptance gate for ticket 001.

The test also needs a C-ABI shim (`planner_api.cpp`) in `tests/_infra/sim/` to
expose `MotionController2` to the Python test harness, following the same pattern as
`drive2_api.cpp` (sprint 057-004) and `sensors_api.cpp` (sprint 057-005).

## Acceptance Criteria

- [ ] `tests/_infra/sim/planner_api.cpp` exposes `extern "C"` functions:
  - `void* planner_api_create(void* drive2_handle)` — construct `MotionController2`
    backed by the given `Drive2` instance
  - `void  planner_api_destroy(void* h)`
  - `void  planner_api_apply_timed(void* h, float vx_mmps, float omega_rads, uint32_t dur_ms)`
  - `void  planner_api_apply_turn(void* h, float heading_rad, float eps_rad)`
  - `void  planner_api_apply_distance(void* h, float dist_mm, float speed_mmps)`
  - `void  planner_api_apply_stop(void* h)`
  - `int   planner_api_tick(void* h, uint32_t now_ms, float* out_vx, float* out_vy, float* out_omega, int* out_has_cmd)` — calls `tick(now)`, writes the first `DrivetrainCommand.twist` fields; sets `*out_has_cmd = 1` if batch non-empty; returns batch count
  - `int   planner_api_is_active(void* h)` — 1 if `state().active`, else 0
- [ ] `tests/simulation/unit/test_planner_subsystem.py` contains:
  - `test_timed_goal_twist_profile` — apply timed goal (200 mm/s for 500 ms); tick
    50 times; assert early ticks have increasing vx (ramp-up), mid ticks cruise near
    target, late ticks decrease (decel), and after deadline `is_active` returns 0.
    Assert omega stays near 0 for a straight-forward timed goal.
  - `test_turn_goal_convergence` — apply turn goal (e.g. π/2 rad); tick up to 200
    times; assert omega non-zero while heading error exceeds tolerance; `is_active`
    becomes 0 when converged; vx stays near 0 (turn-in-place).
  - `test_distance_goal_profile` — apply distance goal (300 mm at 150 mm/s); tick
    until `is_active` returns 0 (or max 300 ticks); assert vx > 0 during motion and
    `CommandBatch` contains a `DrivetrainCommand` each active tick.
  - `test_stop_command_clears_active` — apply timed goal then apply stop; assert
    `is_active` becomes 0 and returned vx, omega are 0.
  - `test_planner_returns_empty_batch_when_idle` — tick with no goal; assert
    `out_has_cmd == 0`.
- [ ] All new tests pass: `uv run python -m pytest tests/simulation/unit/test_planner_subsystem.py -v`
- [ ] Existing suite stays at 2380/2: `uv run python -m pytest -x --tb=short -q`
- [ ] `python build.py --clean` zero errors (planner_api.cpp compiles).

## Implementation Plan

### Approach

Follow the `drive2_api.cpp` / `test_drive2_subsystem.py` pattern exactly.

`planner_api.cpp` constructs a `MotionController2` by:
1. Casting `drive2_handle` to the `Drive2Handle` type used in `drive2_api.cpp`.
2. Constructing `MotionController2` with refs to the internal `MotorController`,
   `BodyVelocityController`, etc., plus the `Drive2` instance for pose reads.
3. Storing the result in a heap-allocated (host-only) wrapper struct.

`planner_api_tick()` sequence:
1. Call the Drive2 tickUpdate function on the drive handle to advance pose.
2. Call `planner.tick(now_ms)`.
3. Inspect the returned `CommandBatch`; write the first command's twist fields.

For isolation tests, `Drive2::tickUpdate` must be called before `planner.tick()`
each iteration so the planner reads a freshly updated pose. The default `SimHardware`
with zero physics produces a stationary pose — sufficient for testing goal convergence
logic (planner sees static heading/distance). For distance goals, the planner ticks
to its deadline regardless of physical progress (the timed deadline bounds it).

### Files to Create

- `tests/_infra/sim/planner_api.cpp` — C-ABI shim
- `tests/simulation/unit/test_planner_subsystem.py` — pytest test file

### Files to Modify

- `tests/_infra/sim/CMakeLists.txt` — add `planner_api.cpp` to the shim library

### Testing Plan

```bash
python build.py --clean
uv run python -m pytest tests/simulation/unit/test_planner_subsystem.py -v
uv run python -m pytest -x --tb=short -q
```

### Documentation Updates

Add a docstring at the top of `test_planner_subsystem.py` describing the
planner-isolation fixture and how to extend it for new goal types.
