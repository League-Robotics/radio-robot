---
id: "094-002"
title: "Delete VelocityRamp, park Planner"
status: done
use-cases: []
depends-on: ["094-001"]
issue: drivetrain-becomes-the-motion-planner-segment-executing-subsystem.md
---

# 094-002: Delete VelocityRamp, park Planner

## Description

Delete `Motion::VelocityRamp` outright (per the issue's own locked
stakeholder decision: "consolidate on Ruckig... retire
`Motion::VelocityRamp`") and physically relocate `Subsystems::Planner` out
of `source/` into a parked tree, mirroring the existing `source_old`/
`tests/sim/parked-093` precedent.

This is forced, not optional, by a structural fact: `codal.json` sets
`"application": "source"` (codal.json:14), so CODAL compiles every file
under `source/` recursively regardless of whether anything constructs the
class it defines. `planner.cpp` `#include`s `velocity_ramp.h` and calls
`ramp_.setTarget()`/`ramp_.advance()`/`ramp_.reset()` — deleting
`velocity_ramp.{h,cpp}` while leaving `planner.{h,cpp}` in `source/` breaks
the build the moment this ticket's first half lands. Both halves must land
together. See architecture-update.md Design Rationale Decisions 3 and 4 for
the full reasoning — this ticket does not re-litigate it, only executes it.

File a new `clasi/issues/` follow-up tracking GOTO/pursuit's eventual
revival (needs `PoseEstimator` restored, per 093's own parking, plus this
sprint's relocated `Planner` moved back into `source/` and re-profiled onto
`Motion::JerkTrajectory` rather than a resurrected `VelocityRamp`).

## Acceptance Criteria

- [x] `source/motion/velocity_ramp.{h,cpp}` are deleted.
- [x] `source/subsystems/planner.{h,cpp}` are moved (not copied, not
      deleted) to a parked location outside `source/` — pick a location
      consistent with the existing `source_old`/`tests/sim/parked-093`
      precedent (e.g. alongside `source_old/`, or a new dedicated
      `source_parked/` — ticket executor's call, documented in the ticket's
      own completion notes).
- [x] The relocated `planner.h`'s file header gains a short note (per
      architecture-update.md Decision 4's Consequences) that a future
      revival should re-profile GOTO's PRE_ROTATE/PURSUE onto
      `Motion::JerkTrajectory`, not resurrect `Motion::VelocityRamp` —
      doc-comment only, no logic change to the parked file.
- [x] No other file in `source/` references `Subsystems::Planner`,
      `Motion::VelocityRamp`, or their headers after this ticket (grep
      confirms zero remaining `#include "subsystems/planner.h"` /
      `#include "motion/velocity_ramp.h"` under `source/`).
- [x] Firmware build (`just build`) and sim build (`just build-sim`) both
      succeed with `Planner` and `VelocityRamp` absent from the compiled
      `source/` tree.
- [x] `uv run python -m pytest` stays green — any test that directly
      exercised the now-relocated `Planner`/`VelocityRamp` in isolation
      (e.g. `planner_harness.cpp`, `velocity_ramp_harness.cpp` if either
      exists) is itself relocated alongside the class it tests, following
      093 Decision 3's own parking-triage precedent (parked test directory,
      added to `pyproject.toml`'s `norecursedirs`), with a short header note
      on what must return before it can be un-parked.
- [x] A new `clasi/issues/` file exists tracking GOTO/pursuit + absolute
      TURN's revival (needs `PoseEstimator` + the relocated `Planner`
      re-profiled onto `Motion::JerkTrajectory`).

## Implementation Plan

**Approach**: This is a mechanical move-and-delete ticket, not a rewrite.
Use `git mv` for `planner.{h,cpp}` (preserves history) and `git rm` for
`velocity_ramp.{h,cpp}`. Grep the whole `source/` and `tests/` trees for
any remaining reference to either before considering the ticket done.

**Files to modify/delete**:
- Delete: `source/motion/velocity_ramp.{h,cpp}`.
- Move: `source/subsystems/planner.{h,cpp}` → parked location.
- Any test harness file that references either class directly — relocate
  alongside, per 093's own parking precedent.

**Files to create**:
- `clasi/issues/goto-pursuit-absolute-turn-revival.md` (or similar slug) —
  the follow-up issue.

**Testing plan**: `just build` (firmware) and `just build-sim` (sim) both
compile clean with zero references to the deleted/relocated files. `uv run
python -m pytest` stays green. No new behavioral tests are needed — this
ticket changes no runtime behavior of anything still wired.

**Documentation updates**: the new follow-up issue file; a doc-comment-only
edit to the relocated `planner.h`'s header (see AC above) — this is inside
the parked, non-compiled file, so it carries no build risk.

## Completion Notes

- **`Motion::VelocityRamp` deleted**: `git rm source/motion/velocity_ramp.{h,cpp}`.
- **`Subsystems::Planner` parked**: `git mv` to
  `source_parked/094/subsystems/planner.{h,cpp}` — a new dedicated
  `source_parked/094/` leaf (not `source_old/`, which predates Planner
  entirely and holds a different, pre-060 tree), mirroring
  `tests/sim/parked-093`'s own sprint-scoped naming. Header note added per
  AC3 (revival must re-profile GOTO onto `Motion::JerkTrajectory`, not
  resurrect `VelocityRamp`).
- **Real finding beyond the ticket's literal text**: `source/runtime/
  configurator.{h,cpp}` (`Rt::Configurator`) held a genuine, currently-
  compiling `Subsystems::Planner&` constructor parameter/member and called
  `planner_.configure(...)` from its `kPlanner` `ConfigDelta` case —
  architecture-update.md's Decision 3 context note ("unlike ... Rt::
  Configurator, which 093 left unregistered ... because nothing about
  deleting some other file broke its compilation") did not anticipate this:
  Configurator directly depends on the `Subsystems::Planner` class itself
  (not just some unrelated file), so relocating Planner would have broken
  `Configurator`'s own compilation — a real, additional `source/` reference
  the AC's own "no other file in source/ references Subsystems::Planner"
  bar required fixing. `Rt::Configurator` is itself fully unwired already
  (nothing in `main.cpp` constructs it, per 093's `SET`/`GET` unregistration
  — see `main.cpp`'s own header comment), so this was a minimal, surgical
  cut: removed the `Planner&` ctor param/member and the `#include
  "subsystems/planner.h"`; the `kPlanner` delta target still folds onto
  `msg::PlannerConfig` (a wire message type, untouched by the class
  relocation) and still publishes `bb.plannerConfig` — it just no longer
  calls a (now-nonexistent) live subsystem's `configure()`. Configurator's
  other three fold targets (`kDrivetrain`/`kMotor`/`kOdometer`) are
  untouched. Updated `configurator_harness.cpp`/`test_configurator.py` to
  match (dropped the `Planner` local/ctor-arg and the now-unneeded
  `velocity_ramp.cpp`/`planner.cpp`/`stop_condition.cpp`/
  `jerk_trajectory.cpp`/vendored-Ruckig compile inputs and their
  gnu++20/-fno-exceptions/-fno-rtti flags, reverting to plain `c++20` per
  `test_drivetrain.py`/`test_pose_estimator.py`'s own precedent).
- **Sim build fix**: `tests/_infra/sim/CMakeLists.txt` explicitly listed
  `source/subsystems/planner.cpp` in `FIRMWARE_SOURCES` (not a glob) — removed
  that line and updated the file's own "Present"/"Absent" doc-comment
  inventories.
- **Tests parked** to `tests/sim/parked-094/unit/` (new leaf, added to
  `pyproject.toml`'s `norecursedirs`; see `tests/sim/parked-094/README.md`
  for the full inventory/rationale):
  - `test_planner.py` + `planner_harness.cpp` (Planner's own isolated
    coverage).
  - `test_velocity_ramp.py` + `velocity_ramp_harness.cpp` (VelocityRamp's
    own isolated coverage — kept as a historical record only, since its
    source is deleted, not parked).
  - `test_main_loop_order_independence.py` +
    `main_loop_order_independence_harness.cpp` (087-009's stale
    4-subsystem hand-rolled pipeline — predates 093's 2-subsystem
    `MainLoop` gut; Planner's DISTANCE-goal anticipation is central to the
    property under test, so it can't be un-parked by dropping Planner
    alone).
  - NOT parked (verified no actual `Subsystems::Planner`/`VelocityRamp`
    class dependency, comments/`msg::Planner*` wire types only):
    `jerk_trajectory_harness.cpp`, `runtime_blackboard_harness.cpp`,
    `tlm_frame_harness.cpp`, `motor_policy_harness.cpp`,
    `test_segment_executor.py`/`segment_executor_harness.cpp` (094-001's
    replacement coverage, stays live).
- **Follow-up issue**: `clasi/issues/restore-goto-pursuit-with-pose-estimator.md`.
- **Build results**: `just build` (ARM) succeeds — `v0.20260709.3`, FLASH
  137940 B / 37.01% used, no errors (only pre-existing vendor CODAL
  warnings). `just build-sim` succeeds — `libfirmware_host.dylib` links
  clean.
- **Test results**: `uv run python -m pytest tests/sim tests/unit` — 39
  passed, 0 failed (includes `test_configurator.py` and
  `test_segment_executor.py`, both green). Full `uv run python -m pytest`
  (incl. `tests/testgui`) — 10 failed / 90 errors, all pre-existing
  `ModuleNotFoundError: No module named 'PySide6'` (verified by spot-check),
  unrelated to this ticket's changes — per this ticket's own testing plan
  and this project's standing instruction to not chase testgui/PySide6
  failures.
