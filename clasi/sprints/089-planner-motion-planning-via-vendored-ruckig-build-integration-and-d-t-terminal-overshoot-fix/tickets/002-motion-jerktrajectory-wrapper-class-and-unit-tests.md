---
id: '002'
title: Motion::JerkTrajectory wrapper class and unit tests
status: done
use-cases:
- SUC-002
- SUC-003
- SUC-004
- SUC-005
depends-on:
- '001'
github-issue: ''
issue: planner-motion-planning-via-vendored-ruckig.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Motion::JerkTrajectory wrapper class and unit tests

## Description

A new, host-safe class (`source/motion/jerk_trajectory.{h,cpp}`) wrapping
one `ruckig::Ruckig<1>`/`ruckig::Trajectory<1>` pair — one instance is a
single 1-DoF channel; `Subsystems::Planner` will hold two (linear,
rotational — architecture-update.md Decision 1). This ticket builds and
unit-tests the wrapper in isolation, with NO `Planner` integration yet
(tickets 003-005 are the consumers). Mirrors `Motion::VelocityRamp`'s
existing boundary discipline: this class knows nothing about goal kinds,
wire verbs, or `msg::*` types beyond `msg::PlannerConfig` (for
`configure()`).

**[Revision 2, post-stakeholder-design-discussion]** Scope grows to include
two divergence-replan entry points (architecture-update.md Decision 10):
`retarget()` (re-solve position-control-to-rest from an externally supplied
new remaining, seeded from the channel's own last sampled velocity/
acceleration) and `reanchor()` (full external re-anchor, seeded from
caller-supplied position/velocity with acceleration forced to 0). Both
entry points still just solve whatever they are told to solve — they do
NOT decide *whether* a replan should happen (that is `Planner`'s job,
tickets 003/005) — see Approach items 7-8 below.

## Implementation Plan

**Approach** (architecture-update.md Decisions 2, 6, 8):
1. Two solve entry points:
   - **Position-control solve-to-rest**: `current_{position,velocity,
     acceleration}` → `target_position`, `target_velocity = 0`,
     `target_acceleration = 0`. Used by `DISTANCE`/`TURN`/`ROTATION`
     (tickets 003, 005).
   - **Velocity-control solve-to-a-velocity** (`ControlInterface::
     Velocity`, open-ended, no target position): `current_{velocity,
     acceleration}` → `target_velocity`. Used for cruise ramp-up
     (`TIMED`/`VELOCITY`/`STREAM`, ticket 004) AND, with `target_velocity
     = 0`, for every stop-triggered terminal decel across every migrated
     goal kind (tickets 003-005).
   - **[Decision 2's revision]** Both entry points take `max_velocity` as
     a PER-CALL ARGUMENT, not a value read once from `configure()` — the
     caller passes `min(commandedSpeed, globalCeiling)` (e.g. `D`'s own
     `speed` argument clamped by `v_body_max`; `kTurnOmega`/
     `kRotationOmega` clamped by `yaw_rate_max` for `TURN`/`RT`).
     `configure()` still owns the genuinely global, non-per-command limits:
     `a_max`/`a_decel`/`j_max` (linear), `yaw_acc_max`/`yaw_jerk_max`
     (rotational), plus the global ceilings themselves for callers to
     clamp against.
2. A sampling accessor: given an elapsed time, return
   `(position, velocity, acceleration)` via `Trajectory::at_time()`. Rely
   directly on Ruckig's own past-duration "hold at final state"
   extrapolation (confirmed in `trajectory.hpp`'s
   `state_to_integrate_from()`) — do not add separate Planner-side
   "cruise sustain" or "stay at rest" bookkeeping.
3. `configure(const msg::PlannerConfig&, bool isRotational)` (or two
   separate `configure()` overloads/call sites — one instance per
   channel): maps `a_max`/`a_decel`/`v_body_max`/`j_max` (linear) or
   `yaw_rate_max`/`yaw_acc_max`/`yaw_jerk_max`/`yaw_rate_max` (rotational)
   onto Ruckig's `max_velocity`/`max_acceleration`/`max_jerk`. The
   rotational channel's `max_acceleration`/`min_acceleration` needs no
   direction-mirroring (already symmetric, `yaw_acc_max` both ways,
   confirmed against `VelocityRamp::advance()`'s yaw branch). The linear
   channel DOES need direction-mirrored `max_acceleration`/
   `min_acceleration` for a negative-direction command (Open Question 2 —
   `max_acceleration = a_decel, min_acceleration = -a_max` when the
   command's direction is negative; `a_max`/`-a_decel` when positive).
4. Jerk sentinel mapping (Decision 6): `j_max == 0.0f`/`yaw_jerk_max ==
   0.0f` maps to Ruckig's `max_jerk = std::numeric_limits<double>::
   infinity()` (matching Ruckig's own `InputParameter` default), NOT a
   literal `max_jerk = 0` (which would forbid any acceleration change). A
   positive value passes straight through.
5. Current-state seeding (Decision 8): the class remembers its own last
   sampled `(position, velocity, acceleration)` internally; each solve
   call reads that back as its `current_*` input UNLESS the caller
   explicitly seeds a different starting state (needed for `DISTANCE`'s
   very first solve, where there is no prior trajectory). Never accept
   `leftObs`/`rightObs` (measured wheel state) as a seed source — this is
   the direct generalization of `VelocityRamp::seedCurrent()`'s existing
   role and the lesson `applyStopAnticipation()`'s prior limit-cycle bug
   (087-009) already taught this codebase; do not reproduce it.
6. Build the object with `Ruckig<1>` (compile-time DoF, `std::array`, no
   heap), matching `ruckig_smoke_harness.cpp`'s existing usage exactly.
7. **[Revision 2] `retarget(newRemaining)`** — re-solves a
   position-control-to-rest trajectory (the SAME mode as item 1, not a new
   mode) using the channel's OWN remembered velocity/acceleration as the
   seed (item 5, unchanged) but with `newRemaining` (a plain float, supplied
   by the caller) as the new target — i.e. this call re-baselines the
   channel's internal position frame to 0 and solves to `newRemaining`
   ahead, exactly like the very first solve for a fresh `DISTANCE`/`TURN`/
   `ROTATION` goal. This is what `Planner` calls on a NORMAL divergence
   (architecture-update.md Decision 10) — `newRemaining` is the caller's
   dead-time-projected, measured-derived value; this class has no idea
   where that number came from and does not need to.
8. **[Revision 2] `reanchor(position, velocity)`** — re-solves a
   position-control-to-rest trajectory seeded from the CALLER-SUPPLIED
   `position`/`velocity` (plain floats) with `acceleration` forced to `0`,
   NOT from the channel's own remembered state. This is what `Planner`
   calls on a GROSS divergence (Decision 10) — a deliberate, narrow
   exception to item 5's seeding rule, accepted because past the gross
   threshold the channel's own remembered state is known to be wrong.
   `Planner`, not this class, decides when gross vs. normal divergence
   applies and what `position`/`velocity` to pass.
9. **[Revision 2] The never-solves-backward guard is enforced by the
   caller (`Planner`), not this class.** `JerkTrajectory` has no concept of
   "commanded direction of travel" or live measurement — it solves
   whatever target/seed it is given. Rejecting/asserting on a
   backward-pointing `retarget()`/`reanchor()` call would require this
   class to track state (commanded sign, live direction) it otherwise has
   no reason to hold, breaking its "knows nothing about goal kinds" boundary
   (this ticket's own Description). `Planner` performs the guard check
   BEFORE ever calling either entry point (architecture-update.md Decision
   10); this class's own doc comment states this explicitly so a future
   caller does not assume the wrapper defends against it.

**Files to create**: `source/motion/jerk_trajectory.h`, `source/motion/
jerk_trajectory.cpp`. **Files to modify**: none (`Planner` integration is
tickets 003-005).

**Testing plan**: a compile-and-run harness in `tests/sim/unit/` mirroring
`ruckig_smoke_harness.cpp`/`velocity_pid_harness.cpp`'s existing pattern —
hand-rolled assertions, not a gtest/pytest-native C++ framework. Cover: (a)
position-control solve-to-rest reproduces the smoke test's own no-reverse,
arrives-at-rest-at-target shape; (b) velocity-control solve-to-a-velocity's
sampled trajectory holds at the target velocity past its own duration
(past-duration extrapolation); (c) a stop-triggered-style second solve
(velocity-control, target=0) from a mid-cruise seeded state decelerates to
rest with no reverse; (d) `j_max == 0` produces the same shape (within
tolerance) as an explicit `max_jerk = infinity` solve, and `j_max > 0`
produces a measurably different (S-curve) profile (SUC-004's acceptance);
(e) the per-call `max_velocity` argument is respected independently of the
global config ceiling (a low per-call ceiling produces a lower-peak
trajectory than the global one alone would). **[Revision 2]** (f)
`retarget()`: from a mid-trajectory seeded state, a new solve to a smaller
or larger `newRemaining` produces a trajectory whose INITIAL sampled
velocity/acceleration equal the seed state exactly (no discontinuity) and
whose whole trace never reverses; (g) `reanchor()`: a solve seeded from an
explicit `(position, velocity)` argument (deliberately DIFFERENT from the
channel's own last remembered state, to exercise the discontinuity-accepted
path) produces a well-formed, never-reversing trajectory to the given
target — the test documents that a velocity discontinuity at the seam is
EXPECTED and correct for this entry point, unlike (f); (h) a
documentation-pinning test noting `retarget()`/`reanchor()` do NOT validate
that `newRemaining`/`position` is ahead of the seed in the commanded
direction — calling either with a backward-pointing target is defined
behavior (it solves backward) and is the CALLER's (Planner's) job to never
do, per Decision 10.

**Documentation updates**: a class-level doc comment on
`jerk_trajectory.h` following this codebase's existing convention
(`velocity_ramp.h`'s own doc comment as the template) — explain the two
solve modes, the per-call `max_velocity` argument, and the seeding
contract (never from measured state). **[Revision 2]** Also document
`retarget()`/`reanchor()`'s seeding contracts explicitly, and state plainly
that the never-solves-backward guard is the CALLER's responsibility, not
enforced here (architecture-update.md Decision 10).

## Acceptance Criteria

- [x] `Motion::JerkTrajectory` compiles under the firmware's exact flags
      (`gnu++20 -fno-exceptions -fno-rtti`) in both the ARM and host-sim
      builds (ticket 001's integration).
- [x] Position-control solve-to-rest: sampled velocity never goes negative
      across the whole trajectory for a positive target, arrives at rest
      exactly at the target position (mirrors `test_ruckig_smoke.py`'s own
      assertions, against this wrapper class specifically).
- [x] Velocity-control solve-to-a-velocity: sampled trajectory reaches and
      then HOLDS the target velocity past the ramp-up's own duration, with
      no additional bookkeeping in the wrapper beyond `at_time()`.
- [x] A second, stop-triggered-style solve (velocity-control, target=0)
      seeded from a mid-cruise state decelerates to rest with no reverse,
      regardless of when it is triggered relative to the first solve's own
      duration.
- [x] `j_max`/`yaw_jerk_max == 0` sentinel maps to `max_jerk = +infinity`
      (not literal 0); a positive value produces a measurably
      different (S-curve) profile. No config/wire change.
- [x] Per-call `max_velocity` argument is honored independently of the
      global `PlannerConfig` ceiling.
- [x] Linear-channel `max_acceleration`/`min_acceleration` are correctly
      direction-mirrored for a negative-direction solve (Open Question 2);
      rotational-channel limits are symmetric (`yaw_acc_max` both ways),
      with a test confirming no mirroring logic is needed/applied there.
- [x] The class never reads `leftObs`/`rightObs` — its current-state
      seeding is exclusively internal (its own last sample) or an explicit
      caller-provided seed, never a measured-observation argument.
- [x] **[Revision 2]** `retarget(newRemaining)` re-solves position-control-
      to-rest seeded from the channel's own last sampled velocity/
      acceleration (never the position — the call re-baselines to 0), with
      an externally supplied new remaining as target; unit test confirms
      velocity/acceleration continuity across the reseed and no reverse.
- [x] **[Revision 2]** `reanchor(position, velocity)` re-solves position-
      control-to-rest seeded from caller-supplied position/velocity with
      acceleration forced to 0; unit test confirms the resulting trajectory
      is well-formed and non-reversing even though it accepts a velocity
      discontinuity from the prior plan (intentional, Decision 10).
- [x] **[Revision 2]** The class doc comment states explicitly that the
      never-solves-backward guard is Planner-enforced, not wrapper-
      enforced — documented, not silently assumed.

## Testing

- **Existing tests to run**: `uv run pytest tests/sim/unit/test_ruckig_smoke.py`
  (regression check the underlying solver behavior this wrapper builds on
  is unchanged); full `uv run pytest`.
- **New tests to write**: a new `tests/sim/unit/test_jerk_trajectory.py` +
  `jerk_trajectory_harness.cpp` (compile-and-run pattern) covering the
  acceptance criteria above, **[Revision 2]** including `retarget()`/
  `reanchor()` coverage.
- **Verification command**: `uv run pytest tests/sim/unit/test_jerk_trajectory.py`
  then the full `uv run pytest`.
