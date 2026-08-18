# RESOLVED: stall detector used a world-frame velocity COMPONENT as body speed

**Status:** fixed 2026-08-13. Stall detection is back ON
(`wheel_control.stall_window = 500.0`).

## What it was

`Control::DifferentialDrive` decided the body was still from `state.otos.v_x`:

```cpp
bodyStill = |state.otos.v_x| <= stallSpeed && ...
```

The OTOS reports linear velocity in the **WORLD frame**, so `v_x` is the
world-x COMPONENT of the robot's motion, not its forward speed. It collapses to
~0 whenever the robot drives north or south, however fast it is going. The
detector was therefore heading-dependent: it fired constantly along world-y and
never along world-x.

MEASURED on tovez at a commanded 140 mm/s, camera-confirmed motion:

| heading | \|v_x\| | \|v_y\| | speed |
|---|---|---|---|
| +150.3 deg | 117.6 | 71.4 | **137.2** |
| -119.4 deg | 74.8 | 111.2 | **134.0** |

Speed magnitude correct at both; each component swings 75 -> 118 purely with
heading. Against `stall_speed = 15` the `v_x` test matched 71% of moving
samples.

## Symptoms it produced (each chased as a separate bug first)

* a plain 400 mm straight leg travelling **75 mm**, `STALL_L`/`STALL_R` latched
* 300 mm arcs stopping at 44 / 71 / 95 mm
* world-frame `GO_TO` legs freezing mid-route -- pivot correct, drive 74 mm of
  383, then motionless for 15 s pointing straight at the target until timeout
* an "intermittent" ~20% failure rate that moved between targets, because it
  depended on which way the robot happened to be heading

## The fix

`bodyStill` now uses the speed magnitude `hypot(v_x, v_y)`, which is
heading-independent. `stall_window` restored to 500.0.

World-frame GO_TO on the robot's own sensors, camera scoring only, 5 legs
356-801 mm, stall protection ON: **14.7 / 10.4 / 13.3 / 19.5 / 7.5 mm**
(mean 13.1, max 19.5). Was mean 469 mm when this started.

## Corrections to earlier claims in this session

* "OTOS velocity reads ~12x low" was WRONG -- that measured `v_x` alone on a
  robot driving nearly along world-y. The decode fixed in `5bee6ce6`
  (velocity registers were being scaled with POSITION LSBs) was correct, and
  is independently confirmed by the speed column above.
* The stall test's own comment cites "|v_x| read ~150mm/s while rolling and
  4.6mm/s while jammed". Those readings were taken with the pre-5bee6ce6 2x
  scale AND with `v_x` as a proxy for speed, so both figures are unreliable;
  `stall_speed = 15.0` has never been derived against a correct body-speed
  measurement and is still owed one.

## Still worth doing

A bench assertion that fails when a commanded straight leg completes at less
than ~90% of its commanded distance. Every symptom above would have been caught
by it immediately; instead it was diagnosed as a planner bug, an arc bug, a
Navigator bug and a sensor-scale bug in turn.
