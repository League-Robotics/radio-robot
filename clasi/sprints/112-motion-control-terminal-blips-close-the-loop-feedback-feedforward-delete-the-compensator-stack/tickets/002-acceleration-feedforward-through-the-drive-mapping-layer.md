---
id: '002'
title: Acceleration feedforward through the Drive mapping layer
status: open
use-cases:
- SUC-005
- SUC-006
- SUC-007
depends-on:
- '001'
github-issue: ''
issue: motion-control-terminal-blips-reconciled-fix-plan.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Acceleration feedforward through the Drive mapping layer

## Description

Issue step 4. `Motion::JerkTrajectory::sample()`/`peek()` already compute
acceleration (`State::acceleration`) but `Motion::Executor` discards it
before it reaches `App::Drive`. This ticket exposes the dominant channel's
sampled acceleration on `Executor::Twist` (`aRef` for kArc's linear channel,
`alphaRef` for kArc's heading-slaved rate and kPivot's rotational channel —
0 for kTimed, matching the existing `thetaRef`/`omegaDes` 0-for-kTimed
pattern), forwards it through `App::Pilot` to two new DEFAULTED parameters
on `App::Drive::setTwist()`, and has `Drive::tick()` combine a model
feedforward term (`actuation_lag * a`) into each wheel's velocity target via
the SAME `BodyKinematics::inverse()` map already used for velocity
(kinematics is linear, so reusing `inverse()` for acceleration is exact —
`aL = a_x - alpha*b/2`, `aR = a_x + alpha*b/2`). Adds a new `PlannerConfig`
field `actuation_lag` [s], defaulting to 0.130 (`Motion::kDeadTime`'s own
bench-derived value — see sprint Architecture Design Rationale Decision 4:
`Motion::kDeadTime` itself stays declared-but-unused; `Drive` gets its own
config-tunable field rather than a new `App::Drive -> Motion::` dependency),
and a `Drive::configure(const msg::PlannerConfig&)` method mirroring
`Executor::configure()`/`HeadingSource::configure()`'s own convention. This
is an ADDITIVE ticket — no deletion (deletions are tickets 001/004's scope)
— and claims no new harness `xfail` flip on its own; it is verified by
staying green/xfail-as-expected on top of ticket 001's flips.

## Acceptance Criteria

- [ ] `Motion::Executor::Twist` gains `float aRef` [mm/s^2] and
      `float alphaRef` [rad/s^2], populated each `tick()`:
      `aRef = linSample.acceleration` for kArc (0 for kPivot/kTimed);
      `alphaRef = headingRatioPerMm_ * linSample.acceleration` for a
      heading-bearing kArc, `rotSample.acceleration` for kPivot, else 0.
- [ ] `App::Drive::setTwist()` signature becomes `setTwist(float v_x, float
      omega, float a_x = 0.0f, float alpha = 0.0f)`; every EXISTING call
      site compiles and behaves unchanged — verify
      `RobotLoop::handleTwist()`'s raw teleop `TWIST` path still calls the
      2-arg form (or an equivalent that resolves `a_x`/`alpha` to 0).
- [ ] `App::Pilot::tick()` forwards `twist.aRef`/`twist.alphaRef` to
      `Drive::setTwist()`'s two new parameters.
- [ ] `App::Drive::tick()` computes `(aL, aR)` via
      `BodyKinematics::inverse(a_x_, alpha_, trackWidth_, aL, aR)` (the same
      function already used for velocity) and stages
      `left_.setVelocity(vL + actuationLag_ * aL)` /
      `right_.setVelocity(vR + actuationLag_ * aR)`.
- [ ] `App::Drive` gains `configure(const msg::PlannerConfig&)` reading
      `actuation_lag`; the boot wiring (`main.cpp`) calls it once, matching
      `Executor::configure()`/`HeadingSource::configure()`'s own call
      pattern.
- [ ] `msg::PlannerConfig`/`PlannerConfigPatch` gain `actuation_lag` (field
      number 38, the next free number after 37/`terminal_lead`) in
      `src/protos/planner.proto`; regenerated via `scripts/gen_messages.py`
      (never hand-edited); `gen_boot_config.py` bakes
      `ACTUATION_LAG_DEFAULT = 0.130` with a comment citing
      `Motion::kDeadTime`'s own derivation (sprint 100's bench-measured
      `motor_lag`, 120-140ms).
- [ ] No new harness `xfail` flip is claimed by this ticket — verify
      `test_straight_ramp_bounds`/`test_pivot_ramp_bounds` (flipped by
      ticket 001) stay passing, and every other currently-passing harness
      check stays passing. If the feedforward term introduces a NEW
      ramp- or terminal-region bound violation, this is a real finding:
      document it in this ticket's completion notes and adjust the
      implementation (e.g. a clamp) rather than silently re-marking a
      check `xfail`.
- [ ] **Guardrail (SUC-007)**: `App::Drive::tick()` adds no bus traffic —
      still bounded (two `inverse()` calls, two `setVelocity()` calls, no
      I2C, no sleeps), matching `drive.h`'s own existing "no I2C traffic
      and no internal sleeps" contract.
- [ ] **Guardrail (SUC-007)**: `git diff --stat` shows no changes to
      `src/firm/app/robot_loop.cpp`.
- [ ] **Guardrail (SUC-007)**: no `JerkTrajectory` solve is newly seeded
      from measured state by this ticket — `aRef`/`alphaRef` are read from
      the existing `sample()` result already computed for `out.v`/
      `omegaFf`, not a new solve.
- [ ] `app/DESIGN.md`'s `Drive` interface list and `motion/DESIGN.md`'s
      `Twist` field list are updated to document `aRef`/`alphaRef` and
      `Drive::configure()`.
- [ ] `uv run python -m pytest` is green end to end.

## Implementation Plan

- **Approach**: additive only — new `Twist` fields, new defaulted `Drive`
  parameters, new config field, new `Drive::configure()`. No deletion in
  this ticket.
- **Files to modify**: `src/firm/motion/executor.h`/`.cpp` (`Twist` fields
  + population), `src/firm/app/drive.h`/`.cpp` (`setTwist()` signature,
  `tick()` FF combination, `configure()`), `src/firm/app/pilot.cpp`
  (forward `aRef`/`alphaRef`), `src/protos/planner.proto` (new field),
  generated `src/firm/messages/planner.h` + siblings (regenerated),
  `src/scripts/gen_boot_config.py` (default + wiring), `src/firm/main.cpp`
  (`Drive::configure()` boot call), `src/firm/app/DESIGN.md`,
  `src/firm/motion/DESIGN.md`.
- **Documentation updates**: as listed above; note the new
  `App::Drive -> messages/planner.h` dependency edge in `app/DESIGN.md`'s
  own interface/dependency notes if that file enumerates them.

## Testing

- **Existing tests to run**: full `test_behavior_lock.py` harness, plus
  the full `uv run python -m pytest`.
- **New tests to write**: a targeted check (harness-based or a small unit
  test) that a raw `TWIST` (the 2-arg/defaulted `setTwist()` call site) is
  byte-for-byte unaffected by this ticket's changes.
- **Verification command**: `uv run python -m pytest`.
