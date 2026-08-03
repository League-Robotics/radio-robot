---
status: done
priority: high
sprint: '131'
tickets:
- 131-004
---

# Position rebaseline destroys the pose: `positionEpoch` has no consumer outside the wire

## Description

2026-08-02 post-130 review, **F1 (CRITICAL)**. Re-verified in the tree today.

`src/firm/app/robot_loop.cpp:366-374` rebaselines a wheel once its position
crosses ±`kPositionRebaselineMargin` (±30,000 mm) and bumps a counter:

```cpp
if (std::fabs(pos) >= kPositionRebaselineMargin) {
  motor.rebaseline();
  pos = motor.position();
  positionEpoch = static_cast<uint8_t>(positionEpoch + 1);
}
```

The same cycle feeds raw `motor.position()` into `Motion::Odometry::integrate()`
(`robot_loop.cpp:541` → `src/motion/odometry.cpp:17-21`), which takes a **bare
delta with no epoch check**. A ~−30,000 mm step therefore propagates as real
travel: heading jumps ~234 rad, x/y by ~15 m.

The planner's `PoseTracker`/`WheelChannel` (`src/motion/planner/estimation.cpp:8-48`)
have the identical raw-delta shape, so a Move in flight across the boundary is
corrupted too.

## `positionEpoch` was invented for exactly this and reads nowhere

Grep-verified consumers of `positionEpoch`, whole tree:

- `src/firm/app/telemetry.cpp:85,90` — copies it onto the wire.
- `src/tests/sim/unit/app_robot_loop_harness.cpp` — asserts the wire value.
  **That harness does not compile** — see
  [[B-app-robot-loop-harness-never-compiled]].
- `src/firm/types/robot_state.h:107` — the field declaration.

**No motion-side code reads it.** The signal exists, is published, is tested
only by a corpse, and changes nobody's behavior.

## Trigger is ordinary, not exotic

~30 m of net travel on one wheel ≈ **75 s at 400 mm/s on the stand**. A plain
soak run reaches it. `src/tests/bench/move_soak.py` exists.

Secondary defect: `rebaseline()` also zeroes `velocity_`, so the wheel reports
v = 0 for 1–2 cycles mid-motion — which Stage B will act on (see
[[A-commanded-zero-leaks-through-stage-b]]).

## What to do

Give the epoch consumers. Both candidates already have the primitive:

1. `Motion::Odometry` and the planner's `WheelChannel`/`PoseTracker` re-anchor
   on epoch change instead of differencing across it (both already have a
   re-anchor path for their own initialization), **or**
2. `publishWheel()` hands motion a **delta** rather than an absolute position,
   so the rebaseline is invisible downstream by construction.

Option 2 is the smaller surface and removes the class rather than patching the
instance — worth weighing first.

Whichever lands, `velocity_` must not read zero across the boundary.

## Verification

- A pose-sanity test across the rebaseline boundary: drive past the margin in
  sim, assert heading and x/y are continuous (no step) and that odometry total
  travel matches commanded.
- The same assertion for a Move in flight at the boundary — it completes at the
  right place.
- Wheel velocity does not report 0 during the rebaseline cycle.
- Soak: `move_soak.py` past 30 m with pose logged; no discontinuity.

## Related

- `docs/code_review/2026-08-02-post-130-wheels-solid-review.md` F1, F9.
- The only existing test of this policy lives in the non-compiling harness —
  fixing that is [[B-app-robot-loop-harness-never-compiled]], and this issue's
  test needs a live home.
