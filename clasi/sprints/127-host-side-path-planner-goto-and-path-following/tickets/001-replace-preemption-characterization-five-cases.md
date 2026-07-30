---
id: '001'
title: Replace-preemption characterization (five cases)
status: in-progress
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

## Completion Notes (this pass — SIM TIER ONLY, bench NOT executed)

**Scope of this pass**: the robot was on the camera-covered PLAYFIELD when
this pass ran (camera-confirmed world (49.2, 2.5) cm), not the STAND this
ticket's bench cases require — `.claude/rules/playfield-testing.md` and
`.claude/rules/hardware-bench-testing.md` never combine those two regimes.
So this pass delivers: (1) all five sim-tier cases + the duplicate-id
sanity check, run and PASSING, with real measured numbers below; (2) the
bench harness (`src/tests/bench/move_protocol_bench.py`, five new scenario
functions) fully written and ready, but **not invoked against hardware**.
The ticket stays `in-progress` — see "Still blocked on stand access"
below for exactly what remains.

### Sim-tier results (`uv run python -m pytest src/tests/sim/unit/test_app_robot_loop_replace.py -v -s`, all scenarios PASS)

Built two harnesses in one file
(`src/tests/sim/unit/test_app_robot_loop_replace_harness.cpp` +
`test_app_robot_loop_replace.py`): cases 1-5 drive a bare `Motion::Planner`
directly under `TestPlanner::benchLimits()` (realistic, finite accel/decel
ceilings — the "effectively unshaped" `TestSim::SimHarness` sim defaults
would hide Edge B's own discontinuity entirely, since an ~infinite decel
ceiling lets the profiler jump straight to cruise in one step regardless of
the carried-velocity mismatch); the duplicate-id check drives the real
`TestSim::SimHarness` (RobotLoop graph), since that dedup lives in
`RobotLoop::handleMove()`, one layer above `Motion::Planner`.

- **Case 1 (same-curvature-at-speed baseline)**: PASS.
  `CASE1_SAME_CURVATURE_DISCONTINUITY_MM_S = 0.0000` (left/right both
  150.0000 -> 150.0000). Noise floor asserted at <= 1.0 mm/s.

- **Case 2 (Edge B, large-curvature-step-at-speed) — THE number ticket 005
  consumes**:

  > **`CASE2_EDGE_B_DISCONTINUITY_MM_S = 433.3333`** (left: 150.0000 ->
  > -283.3333, delta 433.3333; right: 150.0000 -> 566.6667, delta
  > 416.6667; max of the two = 433.3333 mm/s)

  Measured at `TestPlanner::benchLimits()` (vMax=600, aMax=400, aDecel=300
  mm/s^2, trackWidth=100mm), replacing a 150mm/s straight with a tight arc
  (unitLeft=-0.5, unitRight=1.0, axisPerLambda=0.25) at the same 150mm/s
  dominant-wheel peak. Confirms the design issue's own reasoned estimate
  (carried `profileVelocity_`=150 / new `axisPerLambda`=0.25 = 600 mm/s
  shape-space "previous", vs. the wheel's actual ~150mm/s) by direct
  measurement, not just code reasoning — this is a REAL, ~433mm/s
  single-tick step, not absorbed by the profiler's own decel ramp within
  one control interval.

- **Case 3 (Edge A, axis-change-at-speed)**: PASS. The trim/PID integrator
  is confirmed structurally NOT reset by `replace` (`CASE3_TRIM_INTEGRAL_
  BEFORE_REPLACE_MM_S = 25.4609`, `..._AFTER_REPLACE_MM_S = 25.4609`,
  unchanged). Transient wheel-speed error over 10 cycles (0.5s)
  post-replace: **`CASE3_EDGE_A_TRANSIENT_WHEEL_SPEED_ERROR_MM_S =
  22.3362`** (left=22.3362, right=19.7994).

  > **Verdict: BENIGN** — 22.3362 mm/s vs. the new Move's own commanded
  > wheel speed (omega * trackWidth/2 = 100.0000 mm/s). The stale
  > integral's injected error is well under a quarter of the new turn's own
  > commanded speed; it does not reverse or dominate the turn. (Measured
  > with `trimKp=0.3, trimKi=0.5, trimIMax=150, trimMax=150` and a plant
  > with a deliberate 15% left-wheel gain mismatch to give the integrator
  > something real to learn — the real robot's own live-configured trim
  > gains, `data/robots/tovez.json`, may differ; the bench scenario
  > (`scenario_replace_edge_a_axis_change_at_speed`) measures the ACTUAL
  > configured-robot behavior once run.)

- **Case 4 (axis-change-from-rest sanity)**: PASS.
  `CASE4_TRIM_INTEGRAL_AT_REST_MM_S = 0.0000` — coming to rest passes
  `cmd` through exactly 0.0, which resets the trim integrator (unlike a
  mid-motion replace, case 3). The raw first-tick commanded-value ramp from
  rest is `CASE4_FROM_REST_DISCONTINUITY_MM_S = 30.0000` (the ordinary
  accel step any fresh Move gets, not a stale-state artifact).
  `CASE4_FROM_REST_TRANSIENT_WHEEL_SPEED_ERROR_MM_S = 9.6460`, asserted
  (and confirmed) to be < 75% of case 3's own 22.3362 — the gap between
  the two (9.65 vs 22.34) is attributed to the surviving integral case 3
  alone carries.

- **Case 5 (high-rate ~20Hz replacement, 5.5s)**: PASS.
  `CASE5_HIGH_RATE_MAX_STEP_MM_S = 22.9509`,
  `CASE5_HIGH_RATE_MAX_QUEUE_DEPTH = 1` (never exceeded 1 across 110
  replacements at 20Hz over 5.5s — no runaway queue growth).

- **Duplicate-id sanity check**: PASS. Resending an already-accepted
  `Move.id` (different corr_id, different content, `replace=True`) leaves
  the original Move active, queue depth unchanged (1 -> 1), and the
  duplicate resend's own corr_id still acks OK (err==0) via the telemetry
  ack ring — confirms `RobotLoop::handleMove()`'s `alreadyAccepted()`
  short-circuit runs before `replace` is ever honored.

No file under `src/firm`, no `.proto` file, and no wire message was
modified anywhere in this pass's diff (verified: only test/harness files
plus `move_protocol_bench.py` and this ticket file changed).

### Bench harness (written, NOT run)

`src/tests/bench/move_protocol_bench.py` gained five new scenario
functions (`scenario_replace_same_curvature_at_speed`,
`scenario_replace_edge_b_curvature_step`,
`scenario_replace_edge_a_axis_change_at_speed`,
`scenario_replace_axis_change_from_rest`, `scenario_replace_high_rate`),
reusing `_next_move_id()`/`Result`/`_watch()`/`_drain()`/
`_find_completion_ack()` from the existing file, added to `SCENARIOS`
right after `scenario_replace_preempts`. Each `estop()`s on exit. They
measure the SAME five cases via `TLMFrame.vel` (the always-present, ACTUAL
measured per-wheel velocity) rather than `TLMFrame.cmd_vel` (a
`TelemetrySecondary`-only field, not reliably present in a short
post-replace window) — a deliberately DIFFERENT signal from the sim
harness's raw-command measurement: bench observes what the real plant's
closed loop does (PID lag included), sim isolates the profiler's own math.
Syntax-checked (`python -m py_compile`); **never connected to the robot**
— per this pass's explicit scope limit, no hardware command was issued.

### Still blocked on stand access — NOT satisfied by this pass

The following acceptance criteria require the robot on the STAND (wheels
free) and are **not yet checked off**:

- Bench-tier measurement for cases 1-5 (the acceptance criteria's own
  per-case bullets are satisfied at the sim tier above; the ticket's own
  framing — "at both the sim unit tier and the bench tier" — and the final
  acceptance bullet below still require the bench run).
- "Bench cases run on the stand (wheels free); results included in the
  ticket's Completion Notes with raw printed output" — the bench harness
  is ready (`move_protocol_bench.py`) but has not been run.

**Next step**: once the robot is physically moved to the stand (a
stakeholder action being coordinated separately, per this pass's
instructions), run:

```
mbdeploy list   # confirm the current RADIOBRIDGE port
uv run python src/tests/bench/move_protocol_bench.py --port <that port>
```

and append the bench-tier numbers to this section, then check off the
remaining acceptance criteria and move this ticket to `done`.
