---
id: '002'
title: "App::StateEstimator core module — ZOH v1 wheel/body peer estimates"
status: open
use-cases:
- SUC-057
depends-on: []
github-issue: ''
issue: predict-to-now-odometry-estimator-ring-capture-dump-validation-trajectory-controller.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# App::StateEstimator core module — ZOH v1 wheel/body peer estimates

## Description

Build `App::StateEstimator` (`src/firm/app/state_estimator.{h,cpp}`): a new
passive, pure-computation `app/` module — no I2C bus access, no
`Devices::Clock&` collaborator, no sleeping — that holds per-wheel and
body state as PEER estimates (each independently `valid`/stale) and
answers "predict to now" queries via zero-order-hold (ZOH) extrapolation.
This is the estimator core only: `update()` ingests the SAME `Telemetry::
Frame` data `RobotLoop` already assembles each cycle (no new on-chip
storage), and the query API (`wheelAt`/`bodyAt`/`whereAmI`/`wheelNow`)
lets a caller (a future ticket's `RobotLoop` wiring, or a unit test with
hand-fed numbers) ask "where was/is X" at an arbitrary instant.

Per this sprint's Design Rationale (overlay `design/DESIGN.md`, "117"
paragraph): every public time-taking method takes an EXPLICIT `now`/`t`
argument rather than an owned clock reference — mirrors `Motion::
StopCondition`'s "hand-fed readings, no owned collaborator" precedent,
keeping this class constructible/testable with plain numbers, no fake
clock needed.

This ticket ships the v1 complementary-blend SCAFFOLD (a `setWeights()`
entry point and the blend math itself) but the weights are constructor-
injected plain values for now — the fail-closed config plumbing and the
live CONFIG-patch wire arm that actually FEED `setWeights()` are ticket
003's job. This ticket's own tests exercise the blend math directly with
hand-fed weights (including weight=0, proving pure-encoder passthrough,
and weight=1, proving pure-OTOS passthrough when fresh).

## Acceptance Criteria

- [ ] `WheelEstimate{distance, velocity, basisTime, valid}` (`// [mm]
      [mm/s] [ms]`) and `BodyEstimate{x, y, heading, v_x, v_y, omega,
      basisTime, valid}` (`// [mm] [mm] [rad] [mm/s] [mm/s] [rad/s]
      [ms]`) structs defined per `.claude/rules/coding-standards.md` (no
      units in field names; bracketed unit tags).
- [ ] `update(frame, now)` refreshes both wheel peers from `frame.
      encLeft`/`frame.encRight` (position, velocity, `time`) and the body
      peer from `frame.pose`/`frame.twist` (always) blended with `frame.
      otos`/`frame.otosPresent` (when fresh) via the v1 complementary
      weight.
- [ ] `wheelAt(wheel, t)` / `bodyAt(t)` reproduce the constant-velocity
      ZOH formula exactly: `distance = basis.position + basis.velocity ×
      (t − basis.basisTime)`; body pose extrapolates x/y along the
      basis-time world-frame velocity and heading via `heading =
      basis.heading + basis.omega × (t − basis.basisTime)` (the
      `HeadingSource::headingLead()` equation, generalized).
- [ ] `whereAmI(now)` is exactly `bodyAt(now)`. `wheelNow(wheel)` returns
      the wheel's raw basis reading with zero extrapolation.
- [ ] `reset(x, y, heading)` re-anchors ONLY the body peer's world pose
      (mirrors `Odometry::reset()`); wheel-peer state is untouched.
- [ ] `valid` is `false` for a peer before its first `update()` call
      contributes a reading, `true` after — verified independently for
      each wheel and for the body peer.
- [ ] `innovations()` returns the most recent OTOS-vs-predicted heading/
      omega residual, computed whenever a fresh OTOS reading is blended
      — even at weight 0 (diagnostic only at that weight, never fed back
      into the estimate).
- [ ] `setWeights(FusionWeights)` updates the live blend weights in
      memory; a unit test proves weight=0.0 yields pure-encoder-derived
      body output and weight=1.0 (with a fresh OTOS reading) yields
      pure-OTOS heading/omega output, with intermediate weights blending
      proportionally.
- [ ] No `#include` of any `messages/` or `config/` header — mirrors the
      `devices/` isolation invariant by analogy (this module is pure
      `app/`-internal computation, never touches `msg::*` types
      directly; wire-plane conversion, when it exists, stays at
      `RobotLoop`/`main.cpp`, per ticket 003).

## Implementation Plan

**Approach.** Follow `Motion::StopCondition`'s established shape for a
small, pure-computation module in this tree: a plain class, no owned
collaborators beyond what's passed into each call, fully unit-testable
with hand-fed numbers. `update()` is the one method that mutates state;
every query method (`wheelAt`/`bodyAt`/`whereAmI`/`wheelNow`/
`innovations`) is `const`. Age math: one integer/float subtract cast to
seconds, no 64-bit divides per query (mirrors `StopCondition`'s own
"convert once" precedent, adapted since this module's basis times are
`uint32_t` ms, matching `EncoderReading`/`OtosReading::time`'s existing
wire-frame units — NOT `uint64_t` us despite the source issue's original
sketch; see Design Rationale Decision 2 in the overlay for why explicit
`now`/`t` arguments were chosen, and note the granularity match to
`Frame`'s actual `[ms]` fields, not a claim of more precision than the
frame provides).

Body extrapolation: hold `v_x`/`v_y` (body-frame) and `omega` constant
from the basis; project into world-frame using the basis heading (a
first-order approximation valid for the typically-small ages this
sprint's basis-refresh-every-cycle behavior produces): `x = basis.x +
(v_x·cos(basisHeading) − v_y·sin(basisHeading)) × age`, `y = basis.y +
(v_x·sin(basisHeading) + v_y·cos(basisHeading)) × age`, `heading =
basis.heading + basis.omega × age`.

**Files to create:**
- `src/firm/app/state_estimator.h` / `state_estimator.cpp`.
- `src/firm/app/DESIGN.md` — already carries this module's design
  content via this sprint's overlay (`design/DESIGN.md`, applied at
  sprint close); no further edit needed from this ticket.

**Files to modify:** none — this ticket is additive only, not yet wired
into `RobotLoop`/`main.cpp` (ticket 004's job).

**Documentation updates:** none beyond the overlay (already written).

## Testing

- **Existing tests to run**: full `uv run python -m pytest` (confirm
  zero regression — this ticket adds files, touches nothing existing).
- **New tests to write**: `src/tests/sim/unit/app_state_estimator_harness.cpp`
  + `test_app_state_estimator.py` (new pair, mirroring `app_odometry_harness.cpp`'s
  existing shape) covering: ZOH distance/velocity extrapolation math,
  body pose extrapolation math, staleness/`valid` semantics (per-peer,
  independent), `reset()` behavior, `innovations()` residual computation,
  and the weight=0/weight=1/intermediate blend cases above.
- **Verification command**: `uv run python -m pytest src/tests/sim/unit/test_app_state_estimator.py`.
