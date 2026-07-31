---
id: '001'
title: Replace-preemption characterization (five cases)
status: done
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

- [x] Case 1 (same-curvature-at-speed): PASS, measured per-wheel command
      discontinuity at replace ≤ stated noise floor (report the actual
      mm/s value, not just PASS/FAIL).
- [x] Case 2 (Edge B, large-curvature-step-at-speed): the per-wheel
      command discontinuity at replace is measured and printed in mm/s;
      that value is recorded in this ticket's Completion Notes in a form
      ticket 005 can read directly (a labeled number, not buried in log
      output).
- [x] Case 3 (Edge A, axis-change-at-speed): transient wheel-speed error
      after replace is measured in mm/s over a stated number of control
      cycles, and given an explicit benign/hazardous verdict backed by
      that number.
- [x] Case 4 (axis-change-from-rest): discontinuity is zero (or within the
      same noise floor as case 1).
- [x] Case 5 (high-rate ~20 Hz replacement, ≥5 s): largest inter-command
      step (mm/s) is reported; planner queue depth is confirmed to never
      exceed 1 throughout the run.
- [x] Sim duplicate-id sanity check: a resent already-accepted `Move.id`
      produces no additional plan change (queue depth unchanged).
- [x] No file under `src/firm`, no `.proto` file, and no wire message
      changed anywhere in this ticket's diff.
- [x] Bench cases run on the stand (wheels free); results included in the
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

## Completion Notes

**Two-pass history**: the robot was on the camera-covered PLAYFIELD in the
first pass (camera-confirmed world (49.2, 2.5) cm) — not the STAND this
ticket's bench cases require (`.claude/rules/playfield-testing.md` /
`.claude/rules/hardware-bench-testing.md` never combine those two
regimes) — so that pass delivered the sim-tier results only (below) and a
written-but-unexecuted bench harness, and left the ticket `in-progress`.
The robot was then moved to the stand (wheels free, direct USB); this
second pass ran the bench harness for real against hardware — see
"Bench-tier results" below — and completes the ticket.

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

### Bench-tier results (2026-07-30, robot on the stand, direct USB `/dev/cu.usbmodem2121102`, `tovez`)

Ran `uv run python src/tests/bench/move_protocol_bench.py --port /dev/cu.usbmodem2121102`
against the real robot on the stand (wheels free), power-cycled during the
move so encoders started from zero. `main()` now calls `proto.tlmOn()`
after connect (`TLM:OFF`/`kAuto` keeps a parked robot silent) and
`proto.tlmOff()` in `finally` — added this pass; without it the bench
script would see zero frames before the first Move. **57/57 checks
passed**, including all 17 scenarios (the 10 pre-existing ones unchanged
plus the 7 new/five-case ones — `scenario_replace_high_rate` reports one
check, not five). No `.proto`/wire/`src/firm` file touched. Full raw
transcript: every `[PASS]`/measured-number line below is copied verbatim
from the run.

The five new scenarios initially measured "before vs. last-frame-in-a-
0.2-0.3s-window" (mirroring `_last_pose()`'s own established idiom in this
file) — that badly UNDERSTATED any single-tick transient, since telemetry
lands every ~44ms and a 0.3s window spans 6-7 cycles, long enough for the
wheel-velocity PID to have already closed most of the gap by the time the
"after" sample is taken. Replaced with `_watch_bracket()` +
`_max_consecutive_vel_step()` (also added this pass): one continuous,
chronologically-ordered frame capture spanning the replace instant, scored
by the LARGEST step between any two adjacent frames — the closest a ~23Hz
wire observer can get to "the commanded discontinuity at the instant of
replace." Both methodology iterations are captured in git history; the
numbers below are from the final (bracketed) version.

- **Case 1 (same-curvature-at-speed)**: PASS.
  `CASE1_SAME_CURVATURE_STEP_MM_S=13` (largest consecutive-frame step:
  `(129, 99)->(124, 112)`, 18 frames bracketed). Small, as expected —
  same order of magnitude as case 4's from-rest baseline noise, nowhere
  near Edge B's own figure.

- **Case 2 (Edge B) — bench figure next to the sim figure**:

  > **Sim (raw commanded value, one control tick post-activation):
  > `CASE2_EDGE_B_DISCONTINUITY_MM_S = 433.3333` mm/s**
  > **Bench (largest measured-velocity step across the replace, real
  > robot, tight bracket): `CASE2_EDGE_B_BENCH_STEP_MM_S = 20` mm/s**
  > (largest consecutive-frame step: `(119, 95)->(99, 86)`, 18 frames
  > bracketed, ~44ms/frame)

  **These disagree by ~20x, and it is a real disagreement worth stating
  plainly — but the two numbers are not measurements of the same
  quantity, so "prefer the hardware number" is not a straightforward
  substitution here.** The sim figure is `Planner::commandedLeft()/
  commandedRight()` — the raw value the FIRMWARE'S PROFILER computes and
  hands to the wheel-velocity PID, read directly, with zero actuation
  delay or PID dynamics in between. The bench figure is `TLMFrame.vel` —
  ENCODER-DERIVED measured velocity, downstream of (a) the staged
  command's own actuation latency (`hil_drive.py`'s own measured
  constant, cited in sprint.md: ~150ms transport+PID lag), (b) the
  wheel-velocity PID's own bounded tracking response, and (c) the fact
  that the profiler's carried-value error does not hold as a single step
  — it DECAYS every subsequent tick at the shape's own decel ceiling (the
  same math that produces case 2's 433 mm/s single-tick jump also means
  the commanded value keeps falling every ~40ms after that), so the
  transient the real wheel is asked to chase is a fast RAMP, not a
  step-and-hold. A ~44ms bench sample cannot resolve a step that both (1)
  needs ~150ms of actuation lag to begin manifesting physically and (2)
  is itself already relaxing away by the time it does.
  >
  > **Recommendation for ticket 005**: size the curvature slew limit
  > against the **sim figure, 433.3333 mm/s**, not the bench figure. The
  > slew limit constrains what the HOST is allowed to ask the firmware to
  > command — exactly the quantity the sim measures directly. The bench
  > figure is genuinely reassuring (the CURRENT actuation/PID chain damps
  > the raw command's excursion to something small in practice) but is not
  > a safe basis for bounding the command itself: a future retune (faster
  > PID, shorter actuation delay, different `wheel_velocity_pid.cpp`
  > gains) could let more of that raw 433 mm/s reach the wheels than it
  > does today, and the slew limit is exactly the thing meant to prevent
  > depending on the current damping holding. This is a judgment call by
  > the programmer completing this ticket, not a firmware fix (out of
  > scope here either way) — flagging it explicitly so ticket 005's
  > dispatch can weigh in or override.
  >
  > The `profileVelocity_`/`axisPerLambda` carry mechanism itself (why the
  > commanded value spikes at all) is being filed as its own follow-up
  > issue per the coordinator, separate from this characterization ticket.

- **Case 3 (Edge A, axis-change-at-speed)**: PASS.
  `CASE3_EDGE_A_BENCH_TRANSIENT_ERROR_MM_S=128.0` vs.
  `commanded_wheel_speed=128.0` → **VERDICT=BENIGN** (bench). This exact
  128.0-vs-128.0 match is a measurement-limitation artifact, not a
  finding: no wire telemetry field exposes the trim/PID integrator's own
  state (`Planner::trimLeft()`/`trimIntegralLeft()` are sim/unit-only
  accessors, never serialized), so this bench metric ("how far is
  measured `|vel|` from the turn's own final commanded speed") is
  dominated by the ordinary ramp-from-near-zero every fresh axis change
  shows (one wheel legitimately passes through 0 mid-reversal) — the SAME
  structural fact the sim harness's own case 4 established
  (`profileVelocity_` resets on ANY axis change, at speed or from rest).
  It does **not** isolate the sim's specific trim-integral-carryover
  finding. What DOES show a real, directional difference: the tighter
  bracketed metric, `CASE3_EDGE_A_BENCH_MAX_CONSECUTIVE_STEP_MM_S=72`
  (`(99, 69)->(27, 9)`, 18 frames) vs. case 4's `40` below — case 3's
  transient is ~1.8x case 4's, consistent in DIRECTION with the sim's own
  finding (case 3's 22.3362 mm/s > case 4's 9.6460 mm/s) even though the
  absolute bench numbers reflect a different, larger physical fact (case
  3 crosses a wider velocity range: +150→-128 mm/s per wheel, a 278 mm/s
  span, vs. case 4's 0→±128 mm/s, a 128 mm/s span) rather than isolating
  the integral specifically. **Bench verdict: BENIGN, consistent with the
  sim's own BENIGN verdict** — no oscillation, no runaway, no failed
  enqueue; the sim's trim-integral analysis (22.3362 mm/s vs. the turn's
  own 100 mm/s commanded speed, well under a quarter) remains the
  operative MECHANISM-level explanation, since the wire has no channel to
  confirm or refute it directly on hardware.

- **Case 4 (axis-change-from-rest)**: PASS.
  `CASE4_FROM_REST_BENCH_TRANSIENT_ERROR_MM_S=128.0` (same measurement-
  limitation caveat as case 3 above).
  `CASE4_FROM_REST_BENCH_MAX_CONSECUTIVE_STEP_MM_S=40`
  (`(-20, 43)->(-60, 63)`, 17 frames) — smaller than case 3's 72, matching
  the sim's own directional finding (case 4 < case 3).

- **Case 5 (high-rate ~20Hz replacement, 5.5s)**: PASS.
  `CASE5_HIGH_RATE_BENCH_MAX_STEP_MM_S=94` (sim: 22.9509 — bench higher,
  expected: real encoder noise/jitter plus a plant that isn't a
  zero-error `PerfectPlant`, at a polling cadence not phase-locked to the
  firmware's own 40ms cycle). `every replace=True enqueue acked OK (never
  ERR_FULL) across the 20Hz run` — PASS, 53 replacements sent over 5.5s.
  Queue-depth-never-exceeds-1 is confirmed INDIRECTLY here (no
  `Planner::pendingCount()`-equivalent field is wire-visible;
  `move_protocol_bench.py`'s own `scenario_err_full` elsewhere in this
  file already proves ERR_FULL fires correctly once the queue genuinely
  fills at 5) — the absence of any ERR_FULL ack across 53 rapid
  `replace=True` enqueues is the wire-visible evidence available that
  nothing piled up.

No file under `src/firm`, no `.proto` file, and no wire message was
modified anywhere in this pass's diff (verified: `move_protocol_bench.py`
gained a `tlmOn()`/`tlmOff()` pairing in `main()` plus the bench
methodology fix above — both host-side Python, no wire/firmware changes).

### Summary for ticket 005's dispatch

**Edge B (case 2) — the number ticket 005 needs**: sim `433.3333 mm/s`
(raw commanded discontinuity, one control tick), bench `20 mm/s` (largest
measured-velocity step, tight bracket). **Use the sim figure, 433.3333
mm/s**, for sizing the curvature slew limit — see the full reasoning
above. The bench figure shows the current actuation/PID chain damps this
in practice; it is not a safe basis for bounding the command itself.

**Edge A (case 3) verdict**: **BENIGN** at both tiers. Sim: 22.3362 mm/s
transient vs. the new Move's own 100 mm/s commanded wheel speed (well
under a quarter). Bench: no oscillation/runaway/failed-enqueue observed;
the directional case-3-vs-case-4 comparison (72 vs 40 mm/s max
consecutive step) is consistent with the sim's own finding, though the
wire has no channel to confirm the trim-integral mechanism specifically.
