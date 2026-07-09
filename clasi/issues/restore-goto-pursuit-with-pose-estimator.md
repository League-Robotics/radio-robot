---
status: pending
---

# Restore GOTO/pursuit + absolute-heading TURN, once PoseEstimator is back

## Status: parked, not lost — `Subsystems::Planner` relocated intact by ticket 094-002

Sprint 094 ticket 094-002 relocated `Subsystems::Planner`
(`source/subsystems/planner.{h,cpp}`) out of `source/` verbatim, to
`source_parked/094/subsystems/planner.{h,cpp}` — codal.json's
`"application": "source"` setting compiles every `.cpp` recursively under
`source/`, so once `Motion::VelocityRamp` (a real compile dependency of
`planner.cpp`) was deleted, `Planner` could not be left in place
"unregistered," the way sprint 093 left other unused subsystems
(`Rt::Configurator`, the `dev`/`telemetry` command families). See
`clasi/sprints/094-drivetrain-becomes-the-motion-planner-segment-executor-move-command/architecture-update.md`
Decisions 3 and 4 for the full reasoning.

This means GOTO_GOAL (relative-XY go-to, the `G` verb) and absolute-heading
`TURN` — both of which need a fused pose, not just wheel-frame dead
reckoning — are off the live build entirely, on top of `Subsystems::
PoseEstimator` already being parked by sprint 093.

## What has to come back, in order

1. **`Subsystems::PoseEstimator` restored live** into `Rt::MainLoop::tick()`
   (parked by sprint 093 — the class itself is intact on disk, just
   unwired).
2. **`source_parked/094/subsystems/planner.{h,cpp}` moved back** into
   `source/subsystems/` (reversing ticket 094-002's `git mv`).
3. **GOTO_GOAL's `PRE_ROTATE`/`PURSUE` sub-phases re-profiled onto
   `Motion::JerkTrajectory`** (`source/motion/jerk_trajectory.h`) — every
   other goal kind already migrated off the ramp-based profiler in sprint
   089; do **not** resurrect `Motion::VelocityRamp` (deleted outright by
   094-002, per the issue's own locked "consolidate on Ruckig" decision) to
   unblock this — that would reintroduce the two-motion-generation-mechanism
   duplication the locked decision closed off project-wide.

## What else is parked alongside Planner (094-002)

- `tests/sim/parked-094/unit/test_planner.py` +
  `planner_harness.cpp` — Planner's own isolated acceptance coverage.
- `tests/sim/parked-094/unit/test_velocity_ramp.py` +
  `velocity_ramp_harness.cpp` — VelocityRamp's coverage, kept as a
  historical record only (its source is deleted, not parked); port this
  coverage onto `Motion::JerkTrajectory` rather than reviving the file.
- `tests/sim/parked-094/unit/test_main_loop_order_independence.py` +
  `main_loop_order_independence_harness.cpp` — hand-drives a stale
  4-subsystem (Hardware/Drivetrain/PoseEstimator/Planner) pipeline that
  already predates sprint 093's 2-subsystem `MainLoop` gut; needs a fresh
  decision on whether to re-prove order-independence against the CURRENT
  `Rt::MainLoop::tick()` shape rather than resurrecting this file verbatim.

See `tests/sim/parked-094/README.md` for the full inventory and rationale.
