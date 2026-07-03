---
id: '005'
title: Estimation, motion, and goal-closure identifier sweep
status: open
use-cases: [SUC-002, SUC-004]
depends-on: ['002', '003', '004']
github-issue: ''
issue: remove-units-from-identifier-names.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Estimation, motion, and goal-closure identifier sweep

## Description

Rename remaining unit-suffixed identifiers (parameters, members, locals)
across the pose-estimation, motor-control-inner-loop, and goal-closure
call chain. This ticket is sequenced after tickets 002, 003, and 004
because its files are the **consumers** of all three renamed surfaces
(`Planner` reads `RobotConfig` via `_cfg` per sprint 067, reads
`DesiredState` per sprint 047, and reads `msg::PlannerConfig`) — renaming
its own locals before its inputs are renamed would mean touching every
call site twice (`architecture-update.md` Step 5 "Why").

Scope:
- `source/control/Odometry.{h,cpp}`, `source/state/EKFTiny.{h,cpp}`,
  `source/state/PhysicalStateEstimate.{h,cpp}`: `encLeftMm`/`encRightMm`
  → `encLeft`/`encRight`, `trackwidthMm` (parameter, mirrors ticket 002's
  field rename) → `trackwidth`, `v_otos_mmps` → `vOtos`, `omega_otos_rads`
  → `omegaOtos`, `vy_otos_mmps` → `vyOtos`, `x_mm`/`y_mm`/`h_cdeg` →
  `x`/`y`/`h` (each with a `// [unit]` tag), `now_ms` → `now` `// [ms]`.
  These are the parameter names introduced fresh by sprint 070's
  de-threading ticket (070-003) — renaming them here, not in 070, is
  deliberate (this issue was explicitly deferred out of 070's scope).
- `source/control/MotorController.{h,cpp}`, `VelocityController.{h,cpp}`:
  `velLMms`/`velRMms` → `velLeft`/`velRight`, `trueVelLMms`/`trueVelRMms`,
  `kAtRestVelEpsilonMms`, and peers renamed with `// [mm/s]` tags.
- `source/control/BodyKinematics.{h,cpp}`, `BodyVelocityController.{h,cpp}`:
  remaining `Mms`-suffixed locals renamed.
- `source/superstructure/Planner.{h,cpp}`, `source/control/
  PlannerBegin.cpp`: `arcMm` → `arc`, `rateDps` → `rate`,
  `currentAngleDeg`/`setAngleDeg` → `currentAngle`/`setAngle`,
  `kRtRateDps` → `kRtRate`, `kRtCoastArcMm` → `kRtCoastArc`, each
  `// [unit]` tagged.
- `source/control/StopCondition.{h,cpp}`, `MotionCommand.{h,cpp}`:
  remaining `Mm`/`Ms`-suffixed fields renamed.

**Ambiguity-resolution watch point** (per `architecture-update.md`'s
Comment Convention section and Open Question 4): `Odometry` is one of the
two places (with `Motor`, ticket 006) flagged as most likely to have a
raw-ticks vs. mm-scaled sibling pair that would collide under a naive
suffix strip. If such a pair is found, apply the ambiguity-resolution rule
from `docs/coding-standards.md` (ticket 001) — choose a descriptive name
for the *kind* of quantity rather than a bare strip.

`tests/_infra/golden_tlm_capture.json` requires no regeneration — no
`TLM`/`DBG EST` wire field or format changes (these are already
unit-free tokens, confirmed in `architecture-update.md` Step 1).

See `architecture-update.md` Step 5 ("005 — Estimation/motion/goal-closure
sweep"), the ambiguity-resolution rule, Decision 5; `usecases.md` SUC-002,
SUC-004.

## Acceptance Criteria

- [ ] `Odometry`, `EKFTiny`, `PhysicalStateEstimate`: all identifiers
      listed above renamed; each carries a `// [unit]` comment; no
      unit-suffixed parameter/member/local remains.
- [ ] `MotorController`, `VelocityController`: `velLMms`/`velRMms`/
      `trueVelLMms`/`trueVelRMms`/`kAtRestVelEpsilonMms` and peers renamed
      with `// [mm/s]` tags.
- [ ] `BodyKinematics`, `BodyVelocityController`: no remaining
      `Mms`-suffixed local.
- [ ] `Planner`, `PlannerBegin.cpp`: `arcMm`→`arc`, `rateDps`→`rate`,
      `currentAngleDeg`/`setAngleDeg`→`currentAngle`/`setAngle`,
      `kRtRateDps`→`kRtRate`, `kRtCoastArcMm`→`kRtCoastArc`, each tagged.
- [ ] `StopCondition`, `MotionCommand`: no remaining `Mm`/`Ms`-suffixed
      field.
- [ ] Any raw-ticks vs. mm-scaled sibling-pair collision found in
      `Odometry` is resolved per the ambiguity-resolution rule (descriptive
      replacement, not a bare strip) — documented inline if applied.
- [ ] `tests/_infra/golden_tlm_capture.json` requires no regeneration;
      `TLM`/`DBG EST` output byte-identical for a fixed command sequence
      (spot-checked before/after).
- [ ] `tests/simulation/unit/test_070_003_physicalstateestimate_dethreading.py`,
      the EKF/Odometry unit-test tiers, `test_pursuit_arc_steering.py`,
      `test_planner_subsystem_smoke.py`, `test_rt_slip.py` pass with
      unchanged numeric assertions.
- [ ] Full test suite green (`uv run python -m pytest`).
- [ ] `--clean` sim build performed before running tests.

## Testing

- **Existing tests to run**: `test_070_003_physicalstateestimate_dethreading.py`,
  EKF/Odometry unit-test tiers, `test_pursuit_arc_steering.py`,
  `test_planner_subsystem_smoke.py`, `test_rt_slip.py`, full default
  suite.
- **New tests to write**: none required for the rename itself. If the
  ambiguity-resolution rule is applied to an `Odometry` sibling pair, add/
  update a test asserting both renamed identifiers are read/written
  independently (no accidental merge of two previously-distinct
  quantities).
- **Verification command**: `uv run python -m pytest`

## Implementation Plan

**Approach**: Work file-family by file-family (estimation → motor-control
inner loop → goal-closure), grepping for each old identifier after each
family's edit to confirm no stray reference remains before moving on.
Watch for the ambiguity-resolution case in `Odometry` specifically per
Open Question 4.

**Files to modify**:
- `source/control/Odometry.h`, `Odometry.cpp`
- `source/state/EKFTiny.h`, `EKFTiny.cpp`
- `source/state/PhysicalStateEstimate.h`, `PhysicalStateEstimate.cpp`
- `source/control/MotorController.h`, `MotorController.cpp`
- `source/control/VelocityController.h`, `VelocityController.cpp`
- `source/control/BodyKinematics.h`, `BodyKinematics.cpp`
- `source/control/BodyVelocityController.h`, `BodyVelocityController.cpp`
- `source/superstructure/Planner.h`, `Planner.cpp`
- `source/control/PlannerBegin.cpp`
- `source/control/StopCondition.h`, `StopCondition.cpp`
- `source/control/MotionCommand.h`, `MotionCommand.cpp`
- corresponding `tests/simulation/unit/` fixtures that mirror any of
  these identifiers by name

**Testing plan**: `--clean` sim build, then the estimation/EKF test tier
and the arc/pursuit test tier in isolation, then the full suite.

**Documentation updates**: none in this ticket (ticket 008's final sweep
covers prose docs).
