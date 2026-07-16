# src/tests/sim/parked-094/ — sprint-094-era parking leaf

This leaf holds one set of parked tests, stemming from ticket 094-002's
Planner/VelocityRamp relocation. Per this project's greenfield-rebuild
precedent (`source_old/`, `src/tests/sim/parked-093/`): **parked, not deleted.**
`pyproject.toml`'s `norecursedirs` excludes the whole `parked-094/` leaf from
pytest collection (bare name `parked-094`, matching `parked-093`'s
basename-fnmatch behavior).

## [RESTORED] Set A — drive-severed tests (from 093's queue teardown)

Prep work toward "Drivetrain owns its motors" removed the motor/hardware
inbound message queues from `Rt::Blackboard` (`motorIn[]`, `motorResetIn[]`,
`hardwareBroadcastIn`), so a Drivetrain's commanded wheel targets no longer
reached `Subsystems::Hardware`/the simulated plant for a while. Ticket
094-004 gave `Subsystems::Drivetrain` its own `Hardware&` and ticket 094-005
wired `hardware.tick(now)` -> `drivetrain.tick(now, bb.segmentIn,
bb.driveIn)` -> commit into both composition roots, so `S`/`STOP` reach the
plant again. The plant-motion assertions (`unit/test_bare_loop_drive_severed.py`)
have been folded back into `src/tests/sim/unit/test_bare_loop_commands.py`
(094-005) — this set is EMPTY now, kept as a historical marker only.

## Set B — Planner / VelocityRamp isolation tests (ticket 094-002)

Ticket 094-002 relocated `Subsystems::Planner` out of `src/firm/` (to
`source_parked/094/subsystems/planner.{h,cpp}`) and deleted
`Motion::VelocityRamp` outright — forced by `codal.json`'s `"application":
"source"` recursive glob (`planner.cpp` `#include`s `velocity_ramp.h`, so
deleting the ramp while leaving `planner.cpp` in `src/firm/` would break the
firmware build). See `clasi/sprints/094-.../architecture-update.md`.

- **`test_planner.py` + `planner_harness.cpp`** — isolated coverage for
  `Subsystems::Planner`. Returns alongside `planner.{h,cpp}` moving back into
  `src/firm/subsystems/`.
- **`test_velocity_ramp.py` + `velocity_ramp_harness.cpp`** — isolated
  coverage for `Motion::VelocityRamp`, whose source is DELETED (the locked
  "consolidate on Ruckig" decision). Kept only as a historical record; a
  revival should port the coverage onto `Motion::JerkTrajectory`, not
  resurrect `VelocityRamp`.
- **`test_main_loop_order_independence.py` +
  `main_loop_order_independence_harness.cpp`** — hand-drives a stale
  FOUR-subsystem pipeline (Hardware, Drivetrain, PoseEstimator, Planner) that
  predates 093's MainLoop gut; `Planner` is central to the property under
  test, so it cannot be un-parked by dropping Planner alone. A revival needs
  a fresh decision on re-proving order-independence against the current loop.

**Comes back:** `Subsystems::PoseEstimator` restored live, **plus**
`Subsystems::Planner` moved back into `src/firm/subsystems/` and re-profiled
onto `Motion::JerkTrajectory`. Tracked by
`clasi/issues/restore-goto-pursuit-with-pose-estimator.md`.

Not parked (stay live, unaffected): `configurator_harness.cpp`/
`test_configurator.py` (`Rt::Configurator` dropped its `Subsystems::Planner&`
dependency in this same ticket); `jerk_trajectory_harness.cpp`,
`runtime_blackboard_harness.cpp`, `tlm_frame_harness.cpp`,
`motor_policy_harness.cpp` (reference `msg::Planner*` wire types or
`Rt::ConfigDelta::kPlanner` only, never the `Subsystems::Planner` class);
`test_segment_executor.py` + `segment_executor_harness.cpp` (ticket 094-001's
replacement coverage — the whole point of this sprint).
