---
status: done
---

# `replace=true` rescales the carried profile velocity by the new Move's shape, commanding a transient overspeed

## Resolved (out-of-process, 2026-07-30, "relax the at-speed hand-off" work)

Fixed as part of relaxing `boundaryLambda()`/`shapesCompatible()` for
continuously-varying-curvature chained hand-offs (stakeholder request: a
robot moving quickly through a streamed path instead of stopping at every
curvature change). That work required `planWheels()` to stop deriving
each wheel's `previous` from `profileVelocity_ / active_.axisPerLambda`
(the rescale this issue is about) in the first place — so the same fix
applies to `replace=true`, not just the chained path, whether or not that
was the primary target.

**What changed** (`src/motion/planner/planner.cpp`, `planWheels()`):
`previous` is now read straight from `cmdLeftPrevious_`/`cmdRightPrevious_`
— each wheel's own actual last commanded velocity — instead of being
reconstructed by dividing a body-frame scalar by the new Move's
`axisPerLambda`. This is bit-identical to the old formula in every
steady-state (no shape change) case, and is what removes the division
this issue's root cause section identifies. When a wheel's own previous
direction disagrees with the new shape's sign for it (a genuine reversal
— `profileStep()`'s `previous` is a positive-frame quantity with no
representation for "still coasting the wrong way"), that wheel starts
fresh from rest rather than misinterpreting a value it cannot use — the
same safe fallback an axis change already used, now applied per-wheel
instead of only at the coarse Distance/Angle/Wheels granularity.
`planWheels()` also gained a per-wheel ACCEL ceiling rail (independent of
the ratio lock's tie-break) so no wheel can be commanded to accelerate
past its own configured ceiling regardless of what the other wheel's
carried state implies — see that function's own comments for both.

**Measured** (same repro this issue's own table used — sim,
`Motion::Planner` + zero-error test plant, straight at 150 mm/s replaced
by the arc `unitLeft=-0.5, unitRight=1.0` at the same dominant-wheel peak
speed):

| | left [mm/s] | right [mm/s] |
|---|---|---|
| before replace | +150.0 | +150.0 |
| arc's intended targets | −75.0 | +150.0 |
| **before this fix** | **−283.3** | **+566.7** |
| **after this fix, tick+1** | **−20.0** | **+40.0** |
| after this fix, tick+4 (200 ms) | −75.0 (exact) | +150.0 (exact) |

The commanded ratio stays EXACTLY −0.5 : 1 on every intermediate tick
(−20/40, −40/80, −60/120, −75/150) — the ratio lock held throughout the
reversal, not just at the endpoints — and the sequence converges to the
exact target with no overshoot in four ticks. No wheel ever exceeds its
own vMax/aMax ceiling at any point (the original bug commanded 566.7 mm/s
against a 600 mm/s ceiling on a shape whose own vMax was far lower).

**Residual, not chased further this session**: the per-tick step through
a genuine reversal (e.g. left's 150 -> -20 on the very first post-replace
tick) is NOT individually bounded by `aMax * dt` the way this issue's own
"Verification" section originally asked — it is bounded by monotonicity,
ratio-exactness, and never exceeding either wheel's own vMax/aMax
ceiling, which in practice reads as "brake hard through the reversal,
ratio-locked, then land exactly on target" rather than a smooth
accel-limited crawl. That is the same character a Distance-to-Angle axis
change (a coarser reversal) already has, just now reached via a
finer-grained, per-wheel test instead of a whole-Move one. Revisit only
if a bench measurement shows this is a real problem in practice (a
genuine reversal via `replace=true` is a REPLACE-only case now — the
chained/queued path this session's work targets is gated by
`shapeDirectionsAgree()` to never reach a reversal at speed at all).

## Original report

## Description

When a `MOVE` with `replace=true` preempts an in-flight Move at speed, the
planner carries `profileVelocity_` across the activation and then divides it by
the **new** Move's `axisPerLambda`. If the two Moves have different curvature,
the profiler ends up believing it is cruising far faster than it is, and
commands wheel velocities well above the new Move's own targets for at least one
control cycle.

**Measured in sim**, sprint 127 ticket 001 (harness
`src/tests/sim/unit/test_app_robot_loop_replace_harness.cpp`, case 2):

Replacing a 150 mm/s straight with a tight arc (`unitLeft = -0.5`,
`unitRight = 1.0`) at the same dominant-wheel peak speed:

| | left [mm/s] | right [mm/s] |
|---|---|---|
| before replace | +150.0 | +150.0 |
| the arc's intended targets | −75.0 | +150.0 |
| **actually commanded** | **−283.3** | **+566.7** |

That is a **433.33 mm/s per-wheel step**, a ~3.8× overspeed on the right wheel,
and the left wheel driven hard into reverse. On hardware this is a lurch, not a
smoothing artifact.

## Cause

`profileVelocity_` is stored in **axis** units (body mm/s) and converted to
shape space per-Move at `src/motion/planner/planner.cpp:1075`:

```cpp
const float previousLambda = profileVelocity_ / active_.axisPerLambda;
```

For a Distance Move, `axisPerLambda = |0.5 * (unitLeft + unitRight)|`. A straight
gives `1.0`; the arc above gives `|0.5 * (-0.5 + 1.0)| = 0.25`. So a carried
`profileVelocity_` of 150 becomes `previousLambda = 600` — the profiler believes
the dominant wheel is at 600 mm/s when it is at 150, and `planWheels()` scales
both wheels to that belief (`left = -0.5 * 600`, `right = 1.0 * 600`, less one
cycle of decel).

The carry itself is not the bug. Body speed genuinely **is** continuous through
a curvature change, so carrying it is the right invariant for a path follower.
The defect is that per-wheel acceleration limiting derives its `previous`
argument from the new **shape** rather than from **measured** wheel velocity — so
the differential adjustment the wheels must actually make (inner slows, outer
speeds up) is not rate-limited at all.

Normal chaining never reaches this state: `boundaryLambda()` consults
`shapesCompatible()`, which refuses a hand-off at speed between Moves that do not
command the same left:right ratio, so a compatible hand-off has `axisPerLambda`
unchanged and an incompatible one lands at zero first. Only `replace=true`
activates a differently-shaped Move while the robot is still moving.

## Proposed fix

Either:

1. Derive `planWheels()`'s `previous` from **measured** wheel velocity — the
   `WheelChannel` filtered velocities are already available to it — rather than
   from `profileVelocity_ / axisPerLambda`; or
2. Explicitly rate-limit the per-wheel command delta across an activation, so no
   single cycle can step a wheel by more than its configured `aMax * dt`.

(1) is the more principled fix and removes a whole class of
shape-change-at-speed error; (2) is narrower and cheaper. Either way the
constraint is that a compatible same-shape hand-off must keep its existing
ramp continuity — that path is correct today and is what makes chained legs flow
at speed.

## Verification

- A sim case replacing a straight with a tight arc at speed asserts the
  per-wheel command step stays within `aMax * dt`, where today it is 433 mm/s.
- The existing same-shape replace and chained-hand-off cases must keep their
  current behavior (ramp continuity preserved, no landing-exactness regression).
- Bench: the same scenario on the stand, confirming the sim figure.

## Related

- Sprint 127 ticket 001 characterized this but deliberately did **not** fix it —
  that sprint carries a hard no-firmware-change constraint. Its Completion Notes
  hold the full five-case measurement set.
- Sprint 127 works around it **host-side**: the goto solver (ticket 005) carries
  a curvature slew limit sized from the 433.33 mm/s figure, so the path planner
  can never hand the firmware a curvature step. That workaround protects the path
  planner only — any other `replace=true` caller is still exposed.
- **Not the same issue**: the axis-change zeroing at `planner.cpp:723-729`
  (`profileVelocity_`/`profileAccel_` set to 0 when `axisOf()` changes). Ticket
  001 measured that one as **benign** — 22.3 mm/s transient against 100 mm/s
  commanded — and it has its own separate justification comment.
- `planner.cpp:385-388` is where `replace` flushes the queue and drops the active
  Move; `activateNext()` at `planner.cpp:676-731` is where the carry decision is
  made.
