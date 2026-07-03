---
id: '005'
title: Estimation, motion, and goal-closure identifier sweep
status: done
use-cases:
- SUC-002
- SUC-004
depends-on:
- '002'
- '003'
- '004'
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

- [x] `Odometry`, `EKFTiny`, `PhysicalStateEstimate`: all identifiers
      listed above renamed; each carries a `// [unit]` comment; no
      unit-suffixed parameter/member/local remains.
      (`EKFTiny` had no unit-suffixed identifiers to begin with — confirmed
      by direct read; it takes descriptive, already-unit-free noise-param
      names (`q_xy`, `r_otos_v`, etc.), not raw encoder/OTOS readings.)
- [x] `MotorController`, `VelocityController`: `velLMms`/`velRMms`/
      `trueVelLMms`/`trueVelRMms`/`kAtRestVelEpsilonMms` and peers renamed
      with `// [mm/s]` tags.
      (`velLMms`/`velRMms`/`trueVelLMms`/`trueVelRMms` do not literally
      exist in these two files — those names live in `BenchOtosSensor`
      (ticket 006) and `PhysicsWorld`/`WorldView` (ticket 007) and are out
      of this ticket's scope. The actual `Mm`/`Ms`/`Mms`-suffixed
      identifiers found by direct read of `MotorController`/
      `VelocityController` — `kAtRestVelEpsilonMms`, `leftMms`/`rightMms`,
      `_lastVelMmsL/R`, `_prevTimeMsL/R`, `_lastPidMs`, `encLMm`/`encRMm`
      locals, `getEncoderPositions(leftMm, rightMm)`,
      `VelocityController::minWheelMms`, `kMaxPlausibleMmps` — are all
      renamed with `// [unit]` tags; `minWheelMms` renamed to
      `minWheelSpeed` to match `RobotConfig::minWheelSpeed` (ticket 002).)
- [x] `BodyKinematics`, `BodyVelocityController`: no remaining
      `Mms`-suffixed local.
      (`BodyKinematics` had none — its `vy_mmps`/`vx_mmps`/`omega_rads`
      hits are `BodyTwist3` struct-field accesses owned by `Pose2D.h`, out
      of scope. `BodyVelocityController`'s `v_mms`/`omega_rads` params
      renamed to `v`/`omega`; internal `yawRateMax_rad`/`yawAccMax_rad`/
      `yawJerkMaxRad` locals renamed to `yawRateLimit`/`yawAccLimit`/
      `yawJerkLimit` — descriptive rename, not a bare strip, because a bare
      strip would collide in meaning with `_cfg.yawRateMax` (same name,
      different unit: deg/s config vs. rad/s local) — ambiguity-resolution
      rule applied here, the one place in ticket 005's scope where a
      same-name/different-unit collision actually existed.)
- [x] `Planner`, `PlannerBegin.cpp`: `arcMm`→`arc`, `rateDps`→`rate`,
      `currentAngleDeg`/`setAngleDeg`→`currentAngle`/`setAngle`,
      `kRtRateDps`→`kRtRate`, `kRtCoastArcMm`→`kRtCoastArc`, each tagged.
      (`kRtRateDps`→`kRtRate`, `kRtCoastArcMm`→`kRtCoastArc`, and the
      `rateDps`→`rate` local in `beginRotation` all done. `arcMm` does not
      exist as a separate identifier — `arc`/`stopArc` were already
      unit-free locals in `beginRotation`; tagged with `// [mm]` for
      clarity. `currentAngleDeg`/`setAngleDeg` do not exist anywhere in
      `Planner`/`PlannerBegin.cpp` — confirmed by full-file read and a
      codebase-wide grep; those exact names belong to the unrelated
      `IPositionMotor` servo/motor-position interface (`hal/capability/`,
      `hal/real/Motor.h`, `hal/real/Servo.h`, `hal/sim/SimServo.*`,
      `subsystems/gripper/`), which is out of every ticket's declared scope
      in this sprint and was left untouched. Given the ticket's opening
      framing ("remaining unit-suffixed identifiers... across the...
      goal-closure call chain") and that no other ticket touches
      `Planner`/`PlannerBegin.cpp`, the full unit-suffix sweep was
      completed for these two files beyond the four literal examples:
      `leftMms`/`rightMms`→`left`/`right`, `now_ms`→`now`,
      `v_mms`/`omega_rads`→`v`/`omega`, `durationMs`→`duration`,
      `targetMm`→`targetDistance`, `speedMms`→`speed`,
      `headingCdeg`/`epsCdeg`→`heading`/`eps`, `relCdeg`→`relAngle`,
      `h_rad`→`h`, `bearingRad`→`bearing`, `gateRad`→`gate`,
      `distanceMm`→`distance`, `pursueTimeoutMs`→`pursueTimeout`,
      `nominalMs`/`timeoutMs`→`nominal`/`timeout`,
      `_lastTickMs`/`_currentTimeMs`→`_lastTick`/`_currentTime`,
      `_lastVelocityRefreshMs`→`_lastVelocityRefresh` (public accessor
      `lastVelocityRefreshMs()`→`lastVelocityRefresh()`, with its one
      external call site in `Superstructure.cpp` updated to match).)
- [x] `StopCondition`, `MotionCommand`: no remaining `Mm`/`Ms`-suffixed
      field.
      (`MotionBaseline`'s `t0Ms`/`enc0Mm`/`encDiff0Mm`/`heading0Rad` fields
      renamed to `t0`/`enc0`/`encDiff0`/`heading0`; `MotionCommand`'s
      `_softDeadlineMs`/`kSoftDeadlineMs` renamed to
      `_softDeadline`/`kSoftDeadline`; factory-function params in
      `StopCondition.h` (`makeTimeStop`, `makeDistanceStop`,
      `makeRotationStop`, `makeHeadingStop`, `makePositionStop`) and
      `MotionCommand`'s `configure`/`setTarget`/`start`/`tick`/`softStop`
      params also renamed. `HaltController.{h,cpp}` (not a ticket-005 file)
      directly consumes `MotionBaseline` as a struct field
      (`StopEntry::base`) and was updated minimally — only the
      `.base.t0`/`.base.enc0` field-access sites — to keep the build green;
      its own `now_ms`/`enc_avg_mm` parameter names were left untouched as
      out of this ticket's declared scope.)
- [x] Any raw-ticks vs. mm-scaled sibling-pair collision found in
      `Odometry` is resolved per the ambiguity-resolution rule (descriptive
      replacement, not a bare strip) — documented inline if applied.
      (None found: every encoder-related identifier reachable from
      `Odometry`/`PhysicalStateEstimate`/`MotorController` in this ticket's
      scope is already mm-scaled — `HardwareState::encMm[]` — with no
      parallel raw-ticks field anywhere in these files; ticks-to-mm
      conversion happens in `hal/real/Motor.cpp`, ticket 006's scope. The
      one genuine same-name/different-unit ambiguity found in this ticket's
      scope was in `BodyVelocityController` (see above), not `Odometry`.)
- [x] `tests/_infra/golden_tlm_capture.json` requires no regeneration;
      `TLM`/`DBG EST` output byte-identical for a fixed command sequence
      (spot-checked before/after).
      (File untouched; `tests/simulation/unit/test_golden_tlm.py::
      test_golden_tlm_unchanged` passes.)
- [x] `tests/simulation/unit/test_070_003_physicalstateestimate_dethreading.py`,
      the EKF/Odometry unit-test tiers, `test_pursuit_arc_steering.py`,
      `test_planner_subsystem_smoke.py`, `test_rt_slip.py` pass with
      unchanged numeric assertions.
- [x] Full test suite green (`uv run python -m pytest`).
      (2621 passed, 0 failed — matches the pre-ticket baseline exactly.)
- [x] `--clean` sim build performed before running tests.

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
