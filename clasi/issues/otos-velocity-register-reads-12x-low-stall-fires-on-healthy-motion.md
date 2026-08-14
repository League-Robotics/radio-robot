# OTOS velocity reads ~12x low; stall detector halts every move

**Status:** open. Stall detection is DISABLED on tovez as a stopgap
(`wheel_control.stall_window = 0.0`) — the robot currently has **no jam
protection**.
**Measured:** 2026-08-13, tovez, camera as ground truth

## Symptom

Every Distance move terminated early, at wildly varying points:

* plain 400 mm straight leg -> **75 mm** travelled, `STALL_L` + `STALL_R` latched
* 300 mm arcs -> 44 / 71 / 95 mm
* world-frame `GO_TO` legs froze mid-route: pivoted correctly, drove 74 mm of
  383 mm, then sat motionless for 15 s pointing straight at the target until
  the move timed out

It looked like a Navigator or planner bug for hours. It is neither.

## Cause

`Control::DifferentialDrive` (`differential_drive.cpp:339`) decides the body is
still from the OTOS:

```cpp
bodyStill = |state.otos.v_x| <= stallSpeed &&
            |state.otos.omega| * halfTrack <= stallSpeed;
```

Measured with the wheels genuinely turning (camera-confirmed motion, wheel
velocity median **141 mm/s**):

| quantity | value |
|---|---|
| OTOS `v_x` reported | **11.3 mm/s** median |
| `stall_speed` | 15.0 mm/s |
| moving samples at/below `stall_speed` | **30/42 (71%)** |

So `bodyStill` is TRUE while the robot drives at cruise, `demanding` is
trivially true, and the detector latches within its 500 ms window and halts.

The velocity register is reading roughly **12x low**. Note this is NOT simply
the earlier position-vs-velocity LSB confusion (`5bee6ce6`): immediately after
that fix the same measurement read **158 mm/s against a commanded 140** —
correct. Something between then and now dropped it to ~11 mm/s. That regression
is unexplained and is the actual defect to chase.

## Stopgap applied

`data/robots/tovez.json` -> `wheel_control.stall_window: 0.0`, which that
field's own note documents as the inert state ("the whole detector is inert on
any robot whose JSON leaves stall_window at 0"). With it off, a 5-leg
world-frame GO_TO tour lands every leg:

    before: mean 469 mm, outliers to 677 mm, legs freezing mid-route
    after : 15.6 / 16.1 / 16.3 / 21.4 / 22.2 mm  (mean 18.3, max 22.2)

## Why this matters beyond the numbers

The stall detector exists because "the robot repeatedly drove into the
playfield rails and ground there with nothing noticing" (its own note in
`tovez.json`). It is off now. Do not leave it off.

## What to fix, in order

1. Find why OTOS `v_x` reads ~11 mm/s at ~141 mm/s true. Start by re-running
   the empirical scale check that validated `5bee6ce6` (drive a known leg,
   compare reported `|v|` at cruise against camera-measured distance/time) and
   bisect against that commit.
2. Only then re-enable `stall_window` (500.0), and re-derive `stall_speed`
   against the CORRECTED reading rather than the historical inflated one — the
   original 15.0 was chosen when the register read 2x high, so it was
   effectively 7.5 mm/s of true body speed.
3. Add a bench check that fails if a commanded straight leg completes at less
   than ~90% of its commanded distance. Every symptom above would have been
   caught immediately by one such assertion; instead it was diagnosed as a
   planner bug, an arc bug, and a Navigator bug in turn.

## Related

`5bee6ce6` (velocity registers decoded with position LSBs — real, and its
consumer is exactly this stall test). The stall test's own comment cites
"|v_x| read ~150mm/s while rolling and 4.6mm/s while jammed"; those figures
predate both changes and should be re-measured before they are trusted again.
