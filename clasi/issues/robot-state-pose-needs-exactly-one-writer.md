---
status: pending
---

# RobotState::pose needs exactly one writer; retire the unconsumed estimators

**Source:** code review 2026-07-30, `02-motion.md` MAJOR §2, §3;
`01-firm.md` MINOR §7 (stale writer comment).
**Priority:** P1 — today's correct telemetry is a **coincidence of call
order**, not a contract; and the loop computes a full estimator result every
cycle that nobody reads.
**Goal served:** "which pose is the robot's pose?" must have a one-line
answer. Three independent ZOH/OTOS-blend implementations (`Odometry`,
`StateEstimator`, planner `PoseTracker`) are the exact "three pose
estimators, no owner" failure our guidelines were written against — recurring
inside one tree.

## What is wrong

- `robot_state.h:164-169` documents `pose`'s writer as
  `Motion::Odometry::integrate()`, "never OTOS-blended". But
  `Planner::update()` (`planner.cpp:1247-1252`) ALSO writes `state.pose`,
  from its own `PoseTracker` — which blends OTOS heading whenever
  `limits_.headingOtosWeight > 0`. Telemetry happens to read the odom value
  only because `tlm_.update()` runs before `planner_.tick()` in today's
  cycle order; reorder the loop or set the weight nonzero and telemetry
  silently alternates between two disagreeing pose sources.
- `StateEstimator::update()` runs every cycle (`robot_loop.cpp:536`)
  computing a value with **no consumer** — its own header says so.
- `robot_state.h:103-104` still names `App::Drive::tick()` as
  `cmdVelocity`'s writer — stale since the planner integration.

## What to do

1. **One writer for `state.pose`.** Recommended split, matching what each
   estimator is actually for:
   - `Odometry::integrate()` (via `publishPose()`) stays the sole writer of
     `state.pose` — it is the encoder-only trip odometer telemetry reports.
   - **Delete the `state.pose` write block in `planner.cpp:1247-1252`.**
     `PoseTracker` is the planner's *internal* working estimate; if the wire
     ever needs it, publish it through a distinctly-named field (the
     existing `state.estimate` section), never by overwriting `pose`.

```cpp
// planner.cpp -- Planner::update(), DELETE:
//   state.pose.x = ...; state.pose.y = ...; state.pose.heading = ...;
//   state.pose.v_x = ...; state.pose.v_y = ...; state.pose.omega = ...;
// PoseTracker remains planner-internal. RobotState::pose has ONE writer:
// Odometry::integrate() via RobotLoop::publishPose().
```

2. **Retire `StateEstimator` from the loop.** Remove the every-cycle
   `stateEstimator_.update(state_, ...)` call. Then decide: delete
   `state_estimator.{h,cpp}` (recommended — `PoseTracker` is the live
   fusion, `estimator-v2-otos-fusion-sim-first.md` can start fresh from the
   planner's math), or park it with an explicit DESIGN.md note naming why it
   is kept and who owns reviving it. Compiled-and-computed-but-unread is the
   one state it may not stay in.

3. **Fix the field contracts in `robot_state.h`:** the `pose` comment
   (writer + "never OTOS-blended" stays true once step 1 lands) and the
   `cmdVelocity` writer comment (name both writers and the ownership
   switch, mirroring the accurate `Command.mode`/`moveActive` treatment at
   lines 241-256).

4. **Name the estimator roster in `src/motion/DESIGN.md`:** Odometry =
   telemetry trip odometer; PoseTracker = the planner's working estimate
   (the one that drives the robot); anything else must not exist.

## Acceptance

- `grep -n "state.pose" src/motion/planner/planner.cpp` shows reads only.
- `grep -rn "stateEstimator_\." src/firm/app/` returns nothing (or the
  parked class has a DESIGN.md entry naming its owner).
- A firmware test asserts a full cycle leaves `state.pose` equal to
  Odometry's integration even with `headingOtosWeight > 0` configured.
- Bench: telemetry `pose` on the stand matches pre-change values for the
  standard smoke moves (the change removed an unread write, so drift here
  means something was live: stop and investigate).
