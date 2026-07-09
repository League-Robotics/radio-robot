---
id: "094-002"
title: "Delete VelocityRamp, park Planner"
status: open
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

- [ ] `source/motion/velocity_ramp.{h,cpp}` are deleted.
- [ ] `source/subsystems/planner.{h,cpp}` are moved (not copied, not
      deleted) to a parked location outside `source/` — pick a location
      consistent with the existing `source_old`/`tests/sim/parked-093`
      precedent (e.g. alongside `source_old/`, or a new dedicated
      `source_parked/` — ticket executor's call, documented in the ticket's
      own completion notes).
- [ ] The relocated `planner.h`'s file header gains a short note (per
      architecture-update.md Decision 4's Consequences) that a future
      revival should re-profile GOTO's PRE_ROTATE/PURSUE onto
      `Motion::JerkTrajectory`, not resurrect `Motion::VelocityRamp` —
      doc-comment only, no logic change to the parked file.
- [ ] No other file in `source/` references `Subsystems::Planner`,
      `Motion::VelocityRamp`, or their headers after this ticket (grep
      confirms zero remaining `#include "subsystems/planner.h"` /
      `#include "motion/velocity_ramp.h"` under `source/`).
- [ ] Firmware build (`just build`) and sim build (`just build-sim`) both
      succeed with `Planner` and `VelocityRamp` absent from the compiled
      `source/` tree.
- [ ] `uv run python -m pytest` stays green — any test that directly
      exercised the now-relocated `Planner`/`VelocityRamp` in isolation
      (e.g. `planner_harness.cpp`, `velocity_ramp_harness.cpp` if either
      exists) is itself relocated alongside the class it tests, following
      093 Decision 3's own parking-triage precedent (parked test directory,
      added to `pyproject.toml`'s `norecursedirs`), with a short header note
      on what must return before it can be un-parked.
- [ ] A new `clasi/issues/` file exists tracking GOTO/pursuit + absolute
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
