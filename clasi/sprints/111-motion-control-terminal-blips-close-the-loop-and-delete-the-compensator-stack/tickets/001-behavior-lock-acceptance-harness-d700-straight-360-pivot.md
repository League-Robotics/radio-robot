---
id: '001'
title: "Behavior-lock acceptance harness (D700 straight + 360\xB0 pivot)"
status: open
use-cases: [SUC-001]
depends-on: []
github-issue: ''
issue: motion-control-terminal-blips-reconciled-fix-plan.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Behavior-lock acceptance harness (D700 straight + 360° pivot)

## Description

Build a numeric acceptance harness that drives a D700 straight and a 360°
pivot through the REAL `App::RobotLoop` / `App::Pilot` / `Motion::Executor`
stack, captures the per-cycle wheel-velocity trace, numerically
differentiates it for acceleration/jerk, and asserts limits and
single-lobe shape. This is Step 0 of the driving issue
(`clasi/issues/motion-control-terminal-blips-reconciled-fix-plan.md`) —
"land a numeric jerk / single-lobe acceptance test first so every
subsequent deletion is guarded." Today's patch stack (straight-lead
padding, terminal top-up, pivot overshoot lead, the min-speed floor —
none of which this ticket touches) cannot satisfy most of these
assertions; mark those `xfail(strict=False)` citing the driving issue so
sprint 2 has a byte-for-byte regression fence to flip green. This ticket
also adds a same-boot repeated-move scenario targeting the driving
issue's §1.8/F7 stale-executor-state reliability finding.

`completes_issue: false` — this ticket is one of four implementing the
same sprint issue; only the last ticket to land should trigger archival
(set on ticket 004, the final ticket in dependency order — see that
ticket's own frontmatter).

## Acceptance Criteria

- [ ] A new harness (`src/tests/sim/system/behavior_lock_harness.cpp`)
      compiles and links against the real `App::RobotLoop`/`App::Pilot`/
      `Motion::Executor` graph via `TestSim::SimHarness`, mirroring
      `move_queue_harness.cpp`'s own source list and compile flags
      (`test_move_queue.py`'s `_all_sources()`/`_compile_harness()` shape).
- [ ] The harness commands a D700 `kArc` straight (`injectMove(distance=700,
      deltaHeading=0, ...)`) and a 360° `kPivot`
      (`injectMove(distance=0, deltaHeading=2*pi, ...)`), each to
      completion, capturing `Telemetry::Frame.velLeft`/`velRight` every
      cycle via `drainTelemetry()`.
- [ ] Each trace is numerically differentiated (finite difference) into
      per-cycle acceleration and jerk, for both wheels.
- [ ] Assertions exist for: velocity/accel/jerk within `PlannerConfig`'s
      own configured limits at both the ramp-up and the terminal decel;
      exactly one velocity lobe per wheel for the straight; one +lobe and
      one -lobe per wheel for the pivot.
- [ ] A separately-named, separately-marked assertion exists: "no nonzero
      command survives past the terminal zero" (once both wheels first
      reach 0 after the command's own completion event, no cycle before a
      NEW command sees a nonzero `velLeft`/`velRight`). This is the one
      ticket 003 flips from xfail to passing — keep it distinguishable
      from the hump/tail-shape assertions in the harness's own output.
- [ ] Every assertion the CURRENT code cannot satisfy is marked
      `xfail(strict=False)` with a comment/reason string citing
      `motion-control-terminal-blips-reconciled-fix-plan.md`. Do not use
      `strict=True` — the fence tolerates today's known-bad numbers
      without pinning exact values; a marker flipping to green is the
      signal sprint 2 needs, not a byte-exact match.
- [ ] A same-boot scenario reuses ONE `SimHarness` instance across dozens
      (30-50) of consecutive alternating straight/pivot Move commands
      (no reboot between them, unlike `turn_windage_sweep.py`'s
      deliberate per-run isolation) and asserts every one reaches
      `ACK_STATUS_DONE` — its outcome (pass, or an honest, cited finding
      if it reproduces the §1.8/F7 stale-executor-state bug) is recorded
      in this ticket's own completion notes.
- [ ] `uv run pytest` collects and runs the new harness with no collection
      errors; the overall suite stays green (pass or intentional,
      cited xfail).

## Implementation Plan

**Approach**: new harness module, following the exact established shape
of `move_queue_harness.cpp`/`test_move_queue.py` and
`heading_source_harness.cpp`/`test_heading_source.py` (both under
`src/tests/sim/system/`) — a `TestSim::SimHarness`-driven C++ binary
compiled per-test via `subprocess` from a Python pytest file, printing a
human-readable trace and using the same `beginScenario()`/`fail()`/
`checkTrue()` helper idiom those files already use.

1. `TestSim::SimHarness::injectMove(distance, deltaHeading, vMax, omega,
   timeMs, replace, id, corrId)` — use `timeMs=0`/`replace=false` for a
   fresh DISTANCE-mode command in each case. Read `PlannerConfig`'s
   actual configured limits (`a_max`/`a_decel`/`v_body_max`/`j_max` for
   the straight; `yaw_acc_max`/`yaw_rate_max`/`yaw_jerk_max` for the
   pivot) from the SAME default config the harness boots with — do not
   hand-duplicate numeric limits in the test; read them from the
   `msg::PlannerConfig` the harness constructs (avoids drift if a
   default ever changes).
2. Step the sim (`sim.step(1)` per cycle) until the injected Move's own
   completion event drains via `sim.drainTelemetry()` (mirror
   `move_queue_harness.cpp`'s `findAck()` helper), recording
   `frame.velLeft`/`velRight` from each decoded `Telemetry::Frame` along
   the way.
3. Differentiate: `accel[i] = (v[i]-v[i-1]) / dt`, `jerk[i] =
   (accel[i]-accel[i-1]) / dt`, `dt` = the real elapsed cycle time (the
   harness's own virtual clock step, matching `robot_loop.cpp`'s current
   `kCycle=20ms` — read it from the same constant/harness accessor other
   sibling harnesses use for timing assertions, e.g. `test_sim_api.py`'s
   own timing scenario, rather than hardcoding 20).
4. Assert bounds at both ends of the trace (first few cycles after
   activation; last few cycles before/at the completion event) and
   lobe-count over the whole trace (a lobe = a maximal run of
   same-signed nonzero velocity bounded by near-zero samples).
5. Implement "no command survives past the terminal zero" as its own,
   independently reportable check — structure the harness so ticket 003
   can flip exactly this one assertion without touching any other.
6. Give each named check independent pass/xfail visibility — either by
   having the Python driver expose one `pytest` function per named
   scenario (each with its own `@pytest.mark.xfail(strict=False,
   reason=...)` where needed), or by having the C++ harness itself print
   a machine-parseable PASS/FAIL/XFAIL line per check that the Python
   side asserts against selectively. Either mechanism is acceptable; the
   acceptance criterion is independent visibility per named check, not a
   specific implementation.
7. Same-boot scenario: a separate `main()` scenario block (or a second
   harness binary/argument mode) that constructs ONE `SimHarness`,
   `boot()`s once, then loops N times alternating a D700 straight and a
   360° pivot back-to-back on the SAME instance, asserting
   `ACK_STATUS_DONE` each time.

**Files to create**:
- `src/tests/sim/system/behavior_lock_harness.cpp`
- `src/tests/sim/system/test_behavior_lock.py`

**Files to modify**: none required — this is a pure addition. If
`src/tests/sim/system/README.md` catalogs sibling harnesses (it exists
and does describe the tier's own testing pattern), add one entry for
this harness to keep that catalog current.

## Testing

- **Existing tests to run**: `uv run pytest` (confirm no collection
  errors introduced and no pre-existing test's own behavior changes —
  this ticket is additive only).
- **New tests to write**: `src/tests/sim/system/test_behavior_lock.py`
  (see Acceptance Criteria above for required scenarios).
- **Verification command**: `uv run python -m pytest
  src/tests/sim/system/test_behavior_lock.py -v -s`, then `uv run pytest`
  for the full suite.
