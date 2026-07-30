---
id: '001'
title: Replace-preemption characterization (five cases)
status: open
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: sprint-127-host-side-path-planner-goto-path-following.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Replace-preemption characterization (five cases)

## Description

Foundation ticket for the whole sprint. Characterize the firmware's
`Move` `replace=True` preemption path against the five cases the design
issue's investigation identified, at both the sim unit tier and the bench
tier. Ticket 005 (the goto solver) cannot size its curvature slew limit
until this ticket reports its Edge-B measurement — that is a hard,
not cosmetic, dependency (sprint.md Architecture Decision 4).

**Hard constraint**: this ticket characterizes existing firmware behavior
by writing tests against it. It must not modify any file under `src/firm`,
any wire message, or any `.proto` definition. `RobotLoop::handleMove()`'s
axis-carry logic (`src/motion/planner/planner.cpp:723-729`), `axisOf()`
(`:835-843`), and the `axisPerLambda` conversion (`:1075`) are read-only
references for this ticket, not edit targets. If a case reveals behavior
that looks like it needs a firmware fix, stop and flag it (a new issue),
do not fix it here.

**The five cases** (bench: `src/tests/bench/move_protocol_bench.py`, new
scenario functions alongside the existing `scenario_replace_preempts`;
sim: a new unit-tier harness under `src/tests/sim/unit/`, following
`test_app_robot_loop.py`'s existing `*_harness.cpp` + `test_*.py` pattern
— **not** a new `src/tests/sim/system/` file):

1. **Same-curvature-at-speed** (baseline). Replace a Linear move with
   another Linear move at the same `unitLeft`/`unitRight` ratio while at
   speed. Expect profile continuity — no discontinuity beyond measurement
   noise.
2. **Large-curvature-step-at-speed** (Edge B). Replace a straight
   (`axisPerLambda = 1.0`) with a tight arc (e.g. `unitLeft=-0.5,
   unitRight=1.0`, `axisPerLambda = 0.25`) while at speed. Measure the
   per-wheel commanded-velocity discontinuity at the instant of replace,
   in mm/s — this is the number ticket 005 consumes.
3. **Axis-change-at-speed** (Edge A). Replace a Distance-stopped (Linear
   axis) move with an Angle-stopped (Angular axis) move while at speed.
   `profileVelocity_`/`profileAccel_` zero on the axis change but the PID
   integrator does not reset (`replace` ≠ `estop()`) — measure the
   transient wheel-speed error (mm/s, over N control cycles immediately
   after the replace) this produces and give it an explicit
   benign/hazardous verdict backed by that number.
4. **Axis-change-from-rest** (sanity). Same as case 3 but from rest — the
   sanctioned path for a terminal in-place turn. Expect zero surprises;
   confirm the discontinuity is exactly zero (or within noise).
5. **High-rate replacement** (~20 Hz for ≥5 s). Issue `replace=True`
   twists faster than the ~1-tick activation latency. Measure: (a) the
   largest step between consecutive commanded wheel velocities across the
   run (mm/s), and (b) the planner queue depth, which must never exceed 1
   (no runaway growth from replacements arriving faster than they drain).

Also include a **duplicate-id sanity check** in the sim harness (resend an
already-accepted `Move.id`; confirm no additional plan change results).
This is a precondition smoke check only — the **full** four-rule dedup
verification contract (id-0 exemption, window-outlives-completion,
`ERR_FULL` non-recording, and the hardware retry capture) is ticket 002's
scope, not this ticket's; do not duplicate that work here.

**Files**:
- Modify: `src/tests/bench/move_protocol_bench.py` (add 5 scenario
  functions; reuse `_next_move_id()`, `Result`, `_watch()`, `_drain()`
  from the existing file — do not fork a second copy of that
  infrastructure).
- New: `src/tests/sim/unit/test_app_robot_loop_replace.py` +
  `test_app_robot_loop_replace_harness.cpp` (or extend the existing
  `test_app_robot_loop.py`/harness pair if that reads more naturally once
  you're in the file — either is acceptable, prefer extending if the
  existing harness already exposes what you need).

**Coding standards**: no units in any new identifier — a measured
discontinuity is `discontinuity` with a `# [mm/s]` comment tag, never
`discontinuity_mmps`. lowerCamelCase functions/variables, UpperCamelCase
types, matching the rest of `move_protocol_bench.py`.

**Transport**: the robot is currently on battery, reachable only through
the RADIOBRIDGE relay (`/dev/cu.usbmodem2121302`, dongle `zavaz`) — NOT its
own USB port. Confirm the current ROLE via `mbdeploy list` before running;
do not assume the port above is still current. `SerialConnection` detects
the RADIOBRIDGE role and performs the `!GO` handshake itself.

**Safety obligations** (`.claude/rules/playfield-testing.md`,
`.claude/rules/hardware-bench-testing.md`): this ticket runs on the
**stand** (wheels off the ground), not the camera-covered playfield, so
the playfield-specific lights-check/geofence/per-boundary-camera-fix
obligations do not apply (no camera is in this ticket's loop). The
obligation that **does** carry over unconditionally: every connection
path calls `estop()` (never `stop()`) in a `finally` block on exit —
`stop()` is a planned stop that waits behind whatever is already queued;
`estop()` clears Drive targets and the planner queue in the same cycle.
The relay's own known sporadic ack loss is real transport behavior on
this path, not a fault to work around — leave it in the loop.

## Acceptance Criteria

- [ ] Case 1 (same-curvature-at-speed): PASS, measured per-wheel command
      discontinuity at replace ≤ stated noise floor (report the actual
      mm/s value, not just PASS/FAIL).
- [ ] Case 2 (Edge B, large-curvature-step-at-speed): the per-wheel
      command discontinuity at replace is measured and printed in mm/s;
      that value is recorded in this ticket's Completion Notes in a form
      ticket 005 can read directly (a labeled number, not buried in log
      output).
- [ ] Case 3 (Edge A, axis-change-at-speed): transient wheel-speed error
      after replace is measured in mm/s over a stated number of control
      cycles, and given an explicit benign/hazardous verdict backed by
      that number.
- [ ] Case 4 (axis-change-from-rest): discontinuity is zero (or within the
      same noise floor as case 1).
- [ ] Case 5 (high-rate ~20 Hz replacement, ≥5 s): largest inter-command
      step (mm/s) is reported; planner queue depth is confirmed to never
      exceed 1 throughout the run.
- [ ] Sim duplicate-id sanity check: a resent already-accepted `Move.id`
      produces no additional plan change (queue depth unchanged).
- [ ] No file under `src/firm`, no `.proto` file, and no wire message
      changed anywhere in this ticket's diff.
- [ ] Bench cases run on the stand (wheels free); results included in the
      ticket's Completion Notes with raw printed output.

## Testing

- **Existing tests to run**: `uv run python -m pytest src/tests/sim -q`
  (confirm no regression from the new unit harness); existing
  `move_protocol_bench.py` scenarios still pass unmodified
  (`scenario_replace_preempts` and friends).
- **New tests to write**: `src/tests/sim/unit/test_app_robot_loop_replace.py`
  (or extension of the existing `test_app_robot_loop.py`) covering cases
  1, 3, 4, and the duplicate-id sanity check at the sim/unit tier; cases
  2 and 5 are primarily bench-measured (Edge B's discontinuity and the
  20 Hz stress case need real transport timing) but should have a sim-tier
  smoke version too where feasible.
- **Verification command**:
  `uv run python -m pytest src/tests/sim/unit/test_app_robot_loop_replace.py -q`
  and, on the stand (RADIOBRIDGE relay — confirm with `mbdeploy list` first):
  `uv run python src/tests/bench/move_protocol_bench.py --port /dev/cu.usbmodem2121302`
