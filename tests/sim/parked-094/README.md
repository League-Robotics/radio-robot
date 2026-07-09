# tests/sim/parked-094/ — sprint-094-era parking leaf

This leaf holds two sets of parked tests, both stemming from the sprint
093→094 transition (Drivetrain becoming the motion planner that owns its
motors). Per this project's greenfield-rebuild precedent (`source_old/`,
`tests/sim/parked-093/`): **parked, not deleted.** `pyproject.toml`'s
`norecursedirs` excludes the whole `parked-094/` leaf from pytest collection
(bare name `parked-094`, matching `parked-093`'s basename-fnmatch behavior).

## Set A — drive-severed tests (from 093's queue teardown)

Prep work toward "Drivetrain owns its motors" removed the motor/hardware
inbound message queues from `Rt::Blackboard` (`motorIn[]`, `motorResetIn[]`,
`hardwareBroadcastIn`), so a Drivetrain's commanded wheel targets no longer
reach `Subsystems::Hardware`/the simulated plant. `S`/`STOP` still parse and
reply (they post to `bb.driveIn`), but the plant-motion half of the
four-verb suite tests a severed path.

- `unit/test_bare_loop_drive_severed.py` — the plant-motion assertions split
  out of `tests/sim/unit/test_bare_loop_commands.py` (093-003's four-verb
  suite). The command-reply-only tests (`PING`, `HELLO`, `ERR unknown`) stay
  live in `test_bare_loop_commands.py`.

**Comes back:** once sprint 094 gives the Drivetrain its own motors and the
S/STOP path drives the plant again (tickets 094-004/005/006), move this back
to `tests/sim/unit/` (folding into `test_bare_loop_commands.py` if cleaner).

## Set B — Planner / VelocityRamp isolation tests (ticket 094-002)

Ticket 094-002 relocated `Subsystems::Planner` out of `source/` (to
`source_parked/094/subsystems/planner.{h,cpp}`) and deleted
`Motion::VelocityRamp` outright — forced by `codal.json`'s `"application":
"source"` recursive glob (`planner.cpp` `#include`s `velocity_ramp.h`, so
deleting the ramp while leaving `planner.cpp` in `source/` would break the
firmware build). See `clasi/sprints/094-.../architecture-update.md`.

- **`test_planner.py` + `planner_harness.cpp`** — isolated coverage for
  `Subsystems::Planner`. Returns alongside `planner.{h,cpp}` moving back into
  `source/subsystems/`.
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
`Subsystems::Planner` moved back into `source/subsystems/` and re-profiled
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
