---
id: '005'
title: 'Planner: migrate TURN/ROTATION (TURN/RT) onto the rotational JerkTrajectory
  channel'
status: open
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

- [ ] `Planner::apply()`'s `TURN`/`ROTATION` cases stage a position-control
      Ruckig solve-to-rest on the rotational channel, reading the target
      from `cmd.stops_[0].a` (`TURN`) / `cmd.goal.rotation.angle` (`RT`).
- [ ] Per-call `max_velocity` for `TURN`/`RT` is `kTurnOmega`/
      `kRotationOmega` (clamped by `yaw_rate_max`), not `yaw_rate_max`
      directly.
- [ ] `Motion::evaluateStopCondition()`/`stop_condition.{h,cpp}` are
      byte-for-byte unmodified; `STOP_HEADING`/`STOP_ROTATION` evaluation
      stays authoritative for completion.
- [ ] `rotational_slip` is confirmed unaffected; `rotation_gain_pos/neg`/
      `rotation_offset(_neg)` are confirmed still unread by any runtime
      code (re-grep, do not assume).
- [ ] `applyStopAnticipation()` is deleted in full (all three former
      branches; the function itself).
- [ ] `Planner::tick()`'s dispatch is a clean `mode_ == GO_TO` binary — no
      goal-kind-aware intermediate check remains.
- [ ] `Motion::VelocityRamp` is called ONLY for the `GOTO_GOAL` goal kind;
      its own code is unmodified, only its class comment's "sole caller"
      claim is updated.
- [ ] Sim: a Planner-level test samples the rotational channel's full
      commanded trace for `TURN` and `RT` and asserts no reverse relative
      to the commanded turn direction.
- [ ] `tests/sim/unit/test_motion_commands_arc_turn.py` /
      `tests/sim/system/test_tour_geometry.py`: no NEW failure, no NEW
      `xfail`. The two currently-documented RT `xfail`s may flip to
      passing as a side effect but this is not required.
- [ ] Full sim suite green (including `DISTANCE`/`TIMED`/`VELOCITY`/
      `STREAM` regression coverage — this ticket's cleanup touches shared
      dispatch code).
- [ ] **[Revision 2]** `TURN`/`RT`'s divergence-triggered replan uses the
      SAME per-goal-kind measured-remaining source as their stop-condition
      evaluation (fused heading for `TURN`, encoder arc for `RT`) — no new
      pose/observation dependency introduced.
- [ ] **[Revision 2]** Under a synthetic tracking-lag scenario that would
      otherwise stall the plan short of the target, `TURN`/`RT`'s
      completion mode is their OWN stop condition (`STOP_HEADING`/
      `STOP_ROTATION`) firing, not the `STOP_TIME` safety net (sim-level
      proof here; the bench-level crisp criterion is ticket 007's).
- [ ] **[Revision 2]** The three guards (stop-not-fired, no-reverse-target,
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
