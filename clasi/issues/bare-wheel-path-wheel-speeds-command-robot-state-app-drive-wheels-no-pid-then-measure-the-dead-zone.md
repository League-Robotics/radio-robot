---
status: pending
---

# Bare wheel path: wheel-speeds command → robot state → App::Drive → wheels (no PID), then measure the dead zone

## Part 0 — Finish the loop hygiene pass (immediate, stakeholder-directed)

`robot_loop.cpp` was already gutted (1022 → ~550 lines, publish blobs →
methods, duty staging inline, `cycleStart` local removed). Two remaining
corrections per the stakeholder's rule that **every device tick is a
schedule event and must be visible in `cycle()` itself** — publish
methods only copy device state into the robot state, never tick:

- Hoist `otos_.tick(nowUs)` out of `publishOtos()` into the pace block;
  `publishOtos()` becomes publish-only.
- Hoist the line/color alternation
  (`if (lineTurnNext_) line_.tick(nowUs); else color_.tick(nowUs);
  lineTurnNext_ = !lineTurnNext_;`) out of `publishLineColor()`;
  the publish helper takes which leaf was ticked so the untouched leaf's
  fresh flag stays false.
- Hoist `planner_.tick(state_)` / `planner_.update(state_)` out of
  `runPlannerTick()` into the loop; the helper keeps only the
  move-fault flags + completion ack (rename to `publishMoveResult()`).

Verify: host sim build + ARM build clean, sim domain suite green
(pure refactor, no behavior change).

## Context

Stakeholder-directed reset (2026-07-27). No PID anywhere in the wheel
path — the motor-level PID was already removed (125-003), and the
planner's duty stage comes OUT of the path too. The architecture, in the
stakeholder's words: a wheel-speeds wire command puts wheel speeds
directly into the robot state; the robot state wheel speeds go directly
into App::Drive, which drives the wheels. That's the whole path. All
planning/shaping is a separate object (idle for now). Then run the duty
sweep to measure the real dead zone with encoders.

Work in `/Volumes/Proj/proj/RobotProjects/radio-robot-elite-pidfree`
(branch `pid-removal`, OOP bypass active, robot on the bench stand).

## Part 1 — Rewire the wheel-speeds path (small changes, mostly deletions)

1. **Dispatch** (`src/firm/app/robot_loop.cpp` `processMessage()`,
   `handleMove()`): a `MoveWheels` command **never touches the planner**.
   It writes the two speeds into the robot state
   (`state_.wheelLeft/Right.cmdVelocity`) plus a timeout deadline;
   expiry or `STOP` zeroes them. Receipt ack + expiry completion ack keep
   the existing corr_id/ack-ring plumbing so bench scripts still work.
2. **`App::Drive`** (`src/firm/app/drive.{h,cpp}`): each cycle, take the
   state's commanded wheel speeds and drive the wheels **open loop**:
   `duty = cmdVelocity × kDutyPerSpeed` per wheel (initial scale
   1/1370 duty per mm/s from the plant measurement; per-wheel/per-
   direction calibration comes from Part 2's sweep). Delete the dead
   interim `WheelVelocityPid` path and its lead-comp machinery; keep
   quiet-at-zero. No PID, no shaping, no compensations.
3. **Planner** (`src/motion/planner/`): out of the wheels path entirely —
   RobotLoop stops feeding its duty outputs to Drive; its duty stage and
   the pile of compensations (accel FF, lag comp, ramp gate, braking
   lead, rest damping, breakaway kick, duty floor, settle creep) are
   removed from the loop's path by this rewiring. (Deeper deletion of the
   now-dead planner code is a follow-up, not blocking tonight — nothing
   calls it once the loop stops.) Twist/planned moves are simply not sent
   in this phase.
4. **`src/firm/main.cpp`**: drop the planner-tuning block from the loop
   wiring accordingly; Drive gets the open-loop scale.
5. Mirrors/tests: sim harness accessors read the state hand-off; suites
   re-run, re-baselining (with comments) the scenarios that encoded the
   removed mechanisms.

## Part 2 — Dead-zone sweep (the real experiment)

Host script `src/tests/bench/duty_sweep.py` steps the wheel-speeds
command through the open-loop map — equivalent to stepping duty, since
the map is a known linear scale — and reads encoders back from telemetry
(open-loop characterization; no control crosses the serial link):

- From rest: command a level, **hold 500 ms**, record encoder delta and
  end velocity, command 0, wait for full stop + 300 ms so every trial
  starts stuck.
- Sweep 0 → 0.30 duty-equivalent in 0.01 steps, per wheel, both
  directions, 3 repeats.

Output: duty-vs-speed curve per wheel per direction (a clean gain
re-measurement at the same time) and the dead-zone edge — four numbers
with repeat spread — reported before any code consumes them.

## Part 3 — On/off check

`move_wheels(±150, ±150)` then stop through the new path. Chart
commanded speed, measured speed, and duty for both wheels — the sim rig
first (bench plant model), then the robot. Present the charts and stop.

Non-goals: no PID, no shaping, no tours, no dead-zone compensation —
the bare path and the measurement only.

## Verification

- Bench: `twist_drive.py` smoke (wheels arm), the Part-3 charts, the
  Part-2 sweep numbers/plot as deliverables.
- Suites: full pytest (sim/testgui/unit) after re-baselines; planner
  ctest still green (library untouched by the rewiring).
