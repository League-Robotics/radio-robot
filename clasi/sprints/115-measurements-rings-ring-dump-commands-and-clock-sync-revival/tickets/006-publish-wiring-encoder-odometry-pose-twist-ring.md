---
id: '006'
title: 'Publish wiring: encoder-odometry pose+twist ring'
status: open
use-cases:
- SUC-115-001
depends-on:
- '004'
github-issue: ''
issue: ''
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Publish wiring: encoder-odometry pose+twist ring

## Description

Depends on ticket 004. Wires `App::Odometry` (the third and last
producer this sprint) to publish a `PoseRecord` — pose AND twist — into
the `encoderPose` ring once per cycle. Independent of ticket 005 (both
depend only on 004); may be implemented in either order, but is
sequenced after 005 in this sprint's table for a cleaner cumulative diff
against `main.cpp`.

`Odometry::integrate()` currently computes `distance`/`headingDelta` (a
per-cycle delta pair) via `BodyKinematics::forward(deltaLeft, deltaRight,
trackWidth_, distance, headingDelta)`. `odometry.h`'s own file header
already documents the needed generalization: "This is valid without a
separate dt because forward()'s equations are linear/homogeneous in
vL/vR — feeding it position DELTAS directly yields (distance,
headingDelta) for exactly this cycle, the same way feeding it velocities
would yield (v, omega)." This ticket uses exactly that: call
`BodyKinematics::forward()` a SECOND time per cycle, over
`left_.velocity()`/`right_.velocity()` (NezhaMotor's own filtered
velocity, not position deltas), to get the twist `(v, omega)` for the
`PoseRecord`. Per project convention, `v_x` = the resulting forward
speed, `v_y` = 0 (tovez is a differential drivetrain).

## Implementation Plan

- **Approach**:
  - Add a `MeasurementRing<PoseRecord>&` constructor dependency to
    `App::Odometry` (the `encoderPose` member).
  - Add a `nowUs` parameter, matching `NezhaMotor::tick(nowUs)`/
    `Otos::tick(nowUs)`'s existing `[us]` convention — either as a new
    parameter on `integrate()` itself, or a separate call
    (implementer's choice; state which in completion notes).
  - After computing `distance`/`headingDelta` as today, compute
    `v_x, omega` via `BodyKinematics::forward(left_.velocity(),
    right_.velocity(), trackWidth_, v_x, omega)` — a second call, not a
    replacement of the existing delta-based integration (which must
    keep accumulating `x_`/`y_`/`theta_` exactly as it does today).
  - Publish `PoseRecord{stamp=nowUs, v_x, v_y=0.0f, omega, x=x_, y=y_,
    heading=theta_}` into `encoderPose` once per `integrate()` call.
  - Wire the new constructor dependency through `main.cpp` and the sim
    composition root.
- **Files to modify**: `src/firm/app/odometry.h`/`.cpp`,
  `src/firm/main.cpp`, sim composition root, and the call site in
  `App::RobotLoop::cycle()` that invokes `odom_.integrate()` (pass the
  new `nowUs` argument — the same `clock_.nowMicros()` reading
  `Otos::tick()`/`NezhaMotor::tick()` already receive that cycle).
- **Testing plan**: sim test commanding a known steady-state velocity,
  confirming the `encoderPose` ring's published `v_x`/`omega` match the
  commanded twist within tolerance, and that `x_`/`y_`/`theta_`'s
  existing accumulation (via `x()`/`y()`/`theta()`) is bit-for-bit
  unaffected by this change (regression check against pre-ticket
  behavior).
- **Documentation updates**: `odometry.h`'s file header already
  anticipates this generalization (quoted above) — update it to say the
  twist IS now computed this way (present tense), not "would".

## Acceptance Criteria

- [ ] `Odometry` gains a `MeasurementRing<PoseRecord>&` dependency (the
      `encoderPose` ring) and a `nowUs` [us] input, matching
      `NezhaMotor`/`Otos`'s existing `tick(nowUs)` convention.
- [ ] The published twist is computed via `BodyKinematics::forward()`
      over `left_.velocity()`/`right_.velocity()` (not position deltas).
- [ ] `v_y` is `0.0f` (tovez differential-drivetrain convention);
      `v_x`/`omega` are the forward speed/yaw rate `BodyKinematics::forward()`
      returns.
- [ ] `Odometry`'s existing `x()`/`y()`/`theta()`/`lastDistance()`/
      `lastHeadingDelta()` public surface and every existing caller
      (`HeadingSource`, `Pilot`, TLM staging) behave identically to
      pre-ticket — this is purely additive.
- [ ] `encoderPose` receives exactly one publish per `integrate()` call.
- [ ] Sim test: drive a known steady velocity, confirm the published
      twist matches the commanded velocity within tolerance.

## Testing

- **Existing tests to run**: any existing `Odometry`/`BodyKinematics` sim
  tests; full `uv run python -m pytest` sim suite; `just build-clean`.
- **New tests to write**: see Implementation Plan's Testing Plan bullet.
- **Verification command**: `uv run pytest`
