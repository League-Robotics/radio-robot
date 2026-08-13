# Heading hold vs streamed GO_TO — RESOLVED: it was cancelling commanded arcs

**Status:** resolved in `9f61cf62`; one hardware verification still owed
**Raised:** 2026-08-13, out of process, playfield session
**Caused by:** `64a55f2f` (heading hold re-enabled, `heading_hold_gain` 0.0 → 1.0)
**Fixed by:** `9f61cf62` (gate heading hold on the move commanding no rotation)

## What it actually was

Not a test race, and not an over-strict scenario — a real firmware defect that
the sim scenario caught correctly.

`applyHeadingHold()` ran on **every** `Move::Kind::Distance`. It pulls the robot
back toward `active_.baselineHeading`, the heading the move activated at. On a
move that *commands rotation*, that cancels exactly the curvature the caller
asked for. `Motion::Navigator` steers by issuing those arcs — so turning
heading hold on silently disabled GO_TO steering.

## The evidence

Instrumenting the failing scenario showed the robot at rest at **(1127, 0.0)**
with the final target at **(1050, 90)** — 119.8 mm away and *increasing*. Its
`y` was **exactly 0.0** at every sample: it drove straight east and never
steered toward any waypoint (targets were y = 0/30/60/90).

The decisive tell was in the velocities:

```
[rest] velL=26.4 velR=26.4    [rest] velL=15.4 velR=15.4
[rest] velL=22.0 velR=22.0    [rest] velL=11.0 velR=11.0
```

Both wheels **exactly equal** through the whole decay. Heading hold is
symmetric about the profiled mean (`cmdLeft = profiled − diff`,
`cmdRight = profiled + diff`), so equal wheels means the arc was *gone* — not
that a differential was fighting it. That ruled out "heading hold is
oscillating" and pointed straight at "heading hold erased the curvature".

| | gain 0.0 | gain 1.0 ungated | gain 1.0 gated |
|---|---|---|---|
| `WORLD_GOTO` arrival | 17.7 mm | 49.0 mm | **17.7 mm** |
| streamed `GO_TO` | passes | **fails** | **passes** |

## The fix

Gate on the move commanding no rotation — `omega == 0` for a Twist,
`vLeft == vRight` for a Wheels move:

```cpp
const bool commandsRotation =
    (m.velocityKind == Move::VelocityKind::Twist) ? (m.omega != 0.0f)
                                                  : (m.vLeft != m.vRight);
if (m.kind == Move::Kind::Distance && !commandsRotation) applyHeadingHold();
```

"Hold the heading" is only meaningful when the plan says the heading should not
change. When the plan says it *should*, holding it is wrong by definition.

Full suite: **1468 passed, 4 xfailed, 0 failed** (was 1467/4/1).

## What was tried and rejected first

Gating on the profiled speed being above `settleRestVelocity`, on the theory
that a differential at zero mean velocity is an in-place pivot. It did not fix
the scenario and was reverted. It was also added on a **misreading** of the
assertion: `checkTrue(!everRestedDuringStream, ...)` asserts the robot *never*
rests mid-stream (continuous flow through waypoints is the point), not that it
must rest.

## Still owed

**Hardware verification of the arc path.** Straight legs are unaffected by this
gate and were already hardware-verified under `64a55f2f` (veer 5.64° → 0.98°
mean; six camera-supervised square tours closing 8.0–40.1 mm), so the fix
cannot regress what was measured. But no arc has been driven on the real robot
with the gate in place.

The attempt was blocked, not skipped: on 2026-08-13 tovez was latching
`kFlagFaultI2CSafetyNet` (bit 6) + `kFlagFaultWedgeLatch` (bit 7) with
`vel=(0,0)` and frozen encoders — a commanded 300 mm straight leg produced
**11 mm** of camera travel. Pre-existing hardware/bus condition, needs a power
cycle. Arc measurements taken before that was noticed are void and are
deliberately not recorded.

To finish: power-cycle tovez, reflash (the fix is newer than the
v0.20260812.5 image on the robot), and drive
`move_twist(v_x, omega, stop_distance=D)` at a few omegas, checking the swept
angle against `omega * (D / v_x)` with camera truth.
