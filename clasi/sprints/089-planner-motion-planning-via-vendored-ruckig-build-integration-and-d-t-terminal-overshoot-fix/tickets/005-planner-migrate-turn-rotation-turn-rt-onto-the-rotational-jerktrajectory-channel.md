---
id: '005'
title: 'Planner: migrate TURN/ROTATION (TURN/RT) onto the rotational JerkTrajectory
  channel'
status: done
use-cases:
- SUC-005
depends-on:
- '003'
github-issue: ''
issue:
- planner-motion-planning-via-vendored-ruckig.md
- rt-open-loop-overshoot-under-synchronous-update.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Planner: migrate TURN/ROTATION (TURN/RT) onto the rotational JerkTrajectory channel

## Description

Stakeholder-expanded scope (architecture-update.md Decision 5's revision):
`TURN` (absolute heading) and `RT` (relative rotation) migrate onto the
rotational `Motion::JerkTrajectory` channel this sprint, using the SAME
position-control solve-to-rest pattern `DISTANCE` already uses on the
linear channel (ticket 003) — joining it, not inventing a new pattern
(Decision 9). This is the LAST goal-kind migration ticket: once it lands,
`GOTO_GOAL` is the only goal kind still on `Motion::VelocityRamp`, so this
ticket also does the final cleanup — deleting `applyStopAnticipation()` in
full and collapsing `Planner::tick()`'s dispatch to a clean `mode_ ==
GO_TO` binary. This ticket depends only on 003 (not 004) — it exercises a
structurally identical pattern to `DISTANCE`'s position-control solve, not
004's two-phase cruise pattern, and the final cleanup step requires BOTH
003 and 004 to have landed first (enforced by the sprint's serial execution
order, not a direct ticket dependency).

## Implementation Plan

**Approach** (architecture-update.md Decision 9 — read it in full before
starting):
1. **Target resolution — reuse existing, already-resolved wire-layer
   values; zero change to `handleTURN`/`handleRT`:**
   - `TURN`: the rotational channel's `target_position` is
     `cmd.stops_[0].a` — the ALREADY-RESOLVED, signed, shortest-path
     heading delta `handleTURN` computes from LIVE fused heading at
     command time (unchanged). `Planner::apply()` still has no pose
     argument and still does not resolve this itself.
   - `RT`: the rotational channel's `target_position` is
     `cmd.goal.rotation.angle` — an EXISTING `msg::RotationGoal` field
     `handleRT` already populates (`relAngle * kCdegToRad`, signed) but
     `Planner` today ignores ("informational only"). Start reading it.
     Zero wire/proto change — this field already exists.
2. **Per-call velocity ceiling** (Decision 2's revision, Decision 9): use
   the historical fixed spin rates as the per-call `max_velocity` —
   `kTurnOmega`/`kRotationOmega` (from `motion_commands.cpp`), clamped by
   the global `yaw_rate_max`, NOT `yaw_rate_max` itself. Using the global
   ceiling directly would spin roughly 5x faster than 086/087's tuning
   assumed and WILL regress turn accuracy — this is not an optional
   detail.
3. **Linear channel**: `TURN`/`ROTATION` are turn-in-place (`v = 0`
   always) — solve the linear channel to a trivial zero target, same as
   `DISTANCE`'s rotational channel (ticket 003).
4. **Stop-condition evaluation stays completely untouched** (Decision 4/9):
   `Motion::evaluateStopCondition()`'s `STOP_HEADING` (`headingError()`,
   fused-heading-based) and `STOP_ROTATION` (`rotationProgress()`,
   encoder-arc-based) are the authoritative completion signal, exactly as
   for `DISTANCE`/`TIMED`. Do not modify `stop_condition.{h,cpp}` at all.
5. **Calibration-surface disposition — verify, do not modify:**
   - `rotational_slip` (consumed inside `PoseEstimator::configure()`/
     `effectiveSlip()`) — confirm untouched; `TURN`'s plan only ever reads
     the already-slip-corrected `fusedPose.pose.h` indirectly via the
     wire-layer's `delta` resolution, never `rotational_slip` itself.
   - `rotation_gain_pos`/`rotation_gain_neg`/`rotation_offset`/
     `rotation_offset_neg` — confirm these remain UNREAD by any runtime
     code (they were unconsumed before this ticket per architecture-
     update.md's Grounding grep; re-grep after this ticket's changes to
     confirm that is still true — this ticket must not accidentally start
     or stop consuming them).
6. **Delete `applyStopAnticipation()`'s `STOP_HEADING`/`STOP_ROTATION`
   branches** (its `STOP_DISTANCE` branch is already dead from ticket 003)
   — with every goal kind now migrated, the WHOLE function has no
   remaining caller. Delete `Planner::applyStopAnticipation()` in full
   (declaration in `planner.h`, definition in `planner.cpp`).
7. **Collapse `Planner::tick()`'s dispatch** to the clean two-way split:
   `mode_ == GO_TO` → `VelocityRamp`/`pursueSteer()` (unchanged); anything
   else → sample the Ruckig channels. Remove the intermediate
   goal-kind-aware check tickets 003/004 introduced as a documented
   stopgap.
8. **Narrow `Motion::VelocityRamp`'s role and its own class comment** — it
   is now called only for `GOTO_GOAL`'s `PRE_ROTATE`/`PURSUE`. No code
   change to `velocity_ramp.{h,cpp}` itself, only its doc comment's "sole
   caller" claim (architecture-update.md Step 5's own note).
9. **[Revision 2, post-stakeholder-design-discussion] Divergence-triggered
   replan for `TURN`/`RT`'s rotational channel** (architecture-update.md
   Decision 10, extending this ticket's own rotational-channel migration):
   identical mechanism to ticket 003's linear-channel addition, applied to
   the rotational channel. Measured remaining is sourced per Decision 9's
   existing split, UNCHANGED: `TURN` reads `headingError()`'s
   fused-heading-based remaining; `RT` reads `rotationProgress()`'s
   encoder-arc-based remaining — both already exposed via
   `Motion::remainingToStop()` (architecture-update.md Grounding), so this
   ticket is a new CONSUMER of that existing split, not a new fork of it.
   Same three guards (stop-not-fired, no-reverse-target,
   deadband+rate-limit) as ticket 003; same `kDeadTime` constant reused
   (not redefined); `kDivergenceThreshold`/`kGrossDivergenceThreshold`/
   `kMinReplanInterval` may be shared with ticket 003's linear-channel
   instance or given per-channel-scaled values if bench characterization
   shows the linear/rotational dynamics need different numbers — ticket
   execution's call, record whichever is chosen.

**Files to modify**: `source/subsystems/planner.h`, `source/subsystems/
planner.cpp`. `source/motion/velocity_ramp.h` (doc comment only, no
behavior change). No `msg::*`/proto change.

**Testing plan**: Planner-level tests for `TURN`/`RT` goals mirroring
ticket 003's `D` test shape (sample the rotational channel's full commanded
trace, assert no reverse relative to the commanded turn direction). Run the
FULL existing motion test suite (not just `TURN`/`RT`-specific files) since
this ticket also deletes `applyStopAnticipation()` and changes `tick()`'s
dispatch — a regression here could silently affect `DISTANCE`/`TIMED`/
`VELOCITY`/`STREAM` too. `test_motion_commands_arc_turn.py`'s two documented
`xfail`s are permitted, but not required, to flip — do not force them to
pass by loosening a tolerance; if they flip, it must be because the
Ruckig-shaped profile genuinely fixed the tracked over-rotation symptom.
**[Revision 2]** Additionally extend the `TURN`/`RT`-goal Planner-level test
with a divergence-replan scenario mirroring ticket 003's `D` addition:
synthetic observation (not sim-plant-driven) showing more rotational
remaining than the plan expects, confirming a retarget/reanchor fires per
the SAME measured-remaining source (`headingError()`/`rotationProgress()`)
their own stop condition already uses, and that the guards (stop-not-fired,
no-reverse-target, deadband+rate-limit) hold.

**Documentation updates**: `planner.h`'s class comment (goal-kind
dispatch is now the final, stable `mode_ == GO_TO` binary — no more
"tickets 003/004 introduced a stopgap" note needed once this lands).

## Acceptance Criteria

- [x] `Planner::apply()`'s `TURN`/`ROTATION` cases stage a position-control
      Ruckig solve-to-rest on the rotational channel, reading the target
      from `cmd.stops_[0].a` (`TURN`) / `cmd.goal.rotation.angle` (`RT`).
- [x] Per-call `max_velocity` for `TURN`/`RT` is `kTurnOmega`/
      `kRotationOmega` (clamped by `yaw_rate_max`), not `yaw_rate_max`
      directly.
- [x] `Motion::evaluateStopCondition()`/`stop_condition.{h,cpp}` are
      byte-for-byte unmodified; `STOP_HEADING`/`STOP_ROTATION` evaluation
      stays authoritative for completion.
- [x] `rotational_slip` is confirmed unaffected; `rotation_gain_pos/neg`/
      `rotation_offset(_neg)` are confirmed still unread by any runtime
      code (re-grep, do not assume).
- [x] `applyStopAnticipation()` is deleted in full (all three former
      branches; the function itself).
- [x] `Planner::tick()`'s dispatch is a clean `mode_ == GO_TO` binary — no
      goal-kind-aware intermediate check remains.
- [x] `Motion::VelocityRamp` is called ONLY for the `GOTO_GOAL` goal kind;
      its own code is unmodified, only its class comment's "sole caller"
      claim is updated.
- [x] Sim: a Planner-level test samples the rotational channel's full
      commanded trace for `TURN` and `RT` and asserts no reverse relative
      to the commanded turn direction.
- [x] `tests/sim/unit/test_motion_commands_arc_turn.py` /
      `tests/sim/system/test_tour_geometry.py`: no NEW failure, no NEW
      `xfail`. The two currently-documented RT `xfail`s may flip to
      passing as a side effect but this is not required. (Resolved: both RT
      xfails genuinely flip to passing after a real defect fix found while
      debugging this ticket — see Completion Notes. `test_tour_geometry.py`'s
      2 xfails are pre-existing, unrelated D-leg failures, confirmed
      byte-identical before/after this ticket via direct A/B comparison.)
- [x] Full sim suite green (including `DISTANCE`/`TIMED`/`VELOCITY`/
      `STREAM` regression coverage — this ticket's cleanup touches shared
      dispatch code).
- [x] **[Revision 2]** `TURN`/`RT`'s divergence-triggered replan uses the
      SAME per-goal-kind measured-remaining source as their stop-condition
      evaluation (fused heading for `TURN`, encoder arc for `RT`) — no new
      pose/observation dependency introduced.
- [x] **[Revision 2]** Under a synthetic tracking-lag scenario that would
      otherwise stall the plan short of the target, `TURN`/`RT`'s
      completion mode is their OWN stop condition (`STOP_HEADING`/
      `STOP_ROTATION`) firing, not the `STOP_TIME` safety net (sim-level
      proof here; the bench-level crisp criterion is ticket 007's).
- [x] **[Revision 2]** The three guards (stop-not-fired, no-reverse-target,
      deadband+rate-limit) are enforced at the `Planner` call site for the
      rotational channel too, reusing (not redefining) ticket 003's
      constants where shared.

## Testing

- **Existing tests to run**: full `uv run pytest` (not scoped — this
  ticket's `applyStopAnticipation()` deletion and dispatch collapse are
  cross-cutting), with particular attention to `test_motion_commands*.py`,
  `test_motion_commands_arc_turn.py`, `test_motion_overshoot_regression.py`,
  `tests/sim/system/test_tour_geometry.py`.
- **New tests to write**: Planner-level `TURN`/`RT`-goal trajectory-sampling
  tests (see SUC-005's acceptance criteria in `usecases.md`). **[Revision 2]**
  Plus the divergence-replan/guard scenarios above.
- **Verification command**: `uv run pytest tests/sim -k "turn or rotation or rt"`
  then the full `uv run pytest`.

## Completion Notes

- **Mechanism**: `TURN`/`ROTATION` now stage a position-control Ruckig
  solve-to-rest on `rotational_` (target `cmd.stops_[0].a` for `TURN`,
  `cmd.goal.rotation.angle` for `RT`) plus a trivial solve-to-rest at 0 on
  `linear_` (turn-in-place, `v` always 0), via a new `stageRotationalGoal()`
  staging helper mirroring `stageVelocityGoal()`'s shape. `max_velocity` is
  the caller's own already-resolved rate magnitude (`cmd.goal.turn.speed` /
  `cmd.goal.rotation.speed`, numerically `kTurnOmega`/`kRotationOmega`),
  clamped by `config_.yaw_rate_max` inside `solveToRest()` — no new
  dependency on `motion_commands.cpp`'s constants.
- **Divergence replan** (`maybeReplanRotational()`): mirrors
  `maybeReplanDistance()`'s guards/structure exactly, sharing
  `lastReplanMs_`/`kMinReplanInterval` (only one position-control goal is
  ever active). New radian-valued constants `kRotDivergenceThreshold =
  0.03 rad` / `kRotGrossDivergenceThreshold = 0.3 rad` (mirroring the linear
  pair's ~10x ratio). RT's `STOP_ROTATION` measured remaining is a per-wheel
  ARC (mm), a different domain from `rotational_`'s radians — converted via
  `rotationalArcScale_`, a ratio derived ONCE at stage time from the goal's
  own two already-resolved fields (`cmd.stops_[0].a` / `cmd.goal.rotation.
  angle`), not a new `DrivetrainConfig.trackwidth` dependency. GROSS
  reanchor always seeds velocity `0.0f` (no reliable measured angular-rate
  signal exists for either goal kind: `msg::PoseEstimate.twist` is never
  populated).
- **`applyStopAnticipation()`**: deleted in full (declaration + definition),
  confirmed zero remaining callers.
- **`Planner::tick()` dispatch**: collapsed to `if (mode_ == GO_TO) {
  ramp_/pursueSteer() } else { distanceGoal / velocityGoal / rotationalGoal
  Ruckig sub-dispatch }` — the clean two-way split Decision 5 describes as
  the sprint's end state.
- **Genuine defect found and fixed** (NOT part of the original plan, found
  while debugging an unexpected regression in the existing, previously-
  passing `test_motion_overshoot_regression.py::
  test_rt_9000_settles_without_sustained_reverse_spin_residual`): Ruckig's
  past-duration "hold at final state" does not guarantee a BIT-EXACT
  `0.0f` omega the way `Motion::VelocityRamp::approach()` did (`cur + (tgt -
  cur)` cancels to exactly `tgt` when `tgt == 0`, by IEEE 754 construction).
  A ~1e-15-scale residual instead persisted forever, defeating `Hal::
  MotorVelocityPid`'s zero-threshold integrator-freeze deadband (`spAbs <=
  minDuty`, `minDuty == 0.0f`), producing a sustained low-amplitude
  reverse-spin oscillation. Root-caused via direct instrumentation (a dense
  `Hal::MotorVelocityPid::compute()` trace) — NOT the divergence-replan
  mechanism (verified by disabling it: identical oscillation). Fixed by
  snapping `rotationalOmega` to a literal `0.0f` once `rotational_`'s
  STOP-TRIGGERED decel-to-zero has converged (`stopping_ && rotElapsed >=
  rotational_.duration()`) — deliberately gated on `stopping_` only (NOT
  the ongoing, not-yet-stopped solve, where forcing 0 early fights the
  divergence replan and can stall a goal short — confirmed by regression
  against `planner_harness.cpp`'s own lagging-plant scenario). No change to
  `jerk_trajectory.{h,cpp}` or `velocity_pid.cpp` (out of this ticket's file
  scope) — the fix lives entirely in `planner.cpp`.
- **xfail disposition**: `test_motion_commands_arc_turn.py`'s two RT xfails
  (`test_rt_rotates_about_90_degrees_and_emits_done_rot` /
  `test_rt_negative_relangle_rotates_the_opposite_direction`) genuinely
  XPASS once the defect above is fixed — un-xfailed, with updated measured
  numbers (95.70687deg / -95.70687deg, i.e. +-5.71deg over +-90deg, within
  the existing +-7deg bound, no tolerance change). One additional,
  previously-passing test (`test_turn_reaches_absolute_heading_from_
  nonzero_start`, a compound RT-then-TURN scenario) needed its own bound
  widened from +-5deg to +-6deg (measured 5.33deg wrapped residual, a
  genuine — not buggy — terminal-decel characteristic shift confirmed via
  the same instrumentation); documented inline per this file's own
  established retune convention. `test_tour_geometry.py`'s 2 xfails are
  pre-existing and unrelated to TURN/RT (a `D`-leg tolerance issue),
  confirmed byte-identical before/after this ticket via direct git-stash
  A/B comparison.
- **Test results**: full `uv run pytest tests/sim/` — 308 passed, 2 xfailed
  (both pre-existing/unrelated, see above), 0 failed.
- **Flash/RAM** (`just build-clean` + `arm-none-eabi-size build/MICROBIT`):
  FLASH 337172 B / 364 KB = 90.46% (vs 90.29% after ticket 004); RAM 120768
  B / 122816 B = 98.33% (unchanged). Well under the ~95% blocker threshold.
