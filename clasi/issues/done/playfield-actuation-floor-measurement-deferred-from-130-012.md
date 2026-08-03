---
status: done
priority: medium
---

# Playfield actuation-floor measurement, deferred from sprint 130 ticket 012

## Description

Sprint 130 ticket 012 was to measure the actuation floor (vMin / breakaway) on the
playfield and replace the provisional termination tolerance with a measured one.
It was never started: it requires the robot **translating on the playfield**, and
the robot spent the end of the sprint wedged (dead-I2C signature, four occurrences
in two days, each needing a physical power cycle).

Deferring rather than faking it. The provisional values stay in place and are
marked as provisional.

## Why the stand cannot substitute

This measurement needs the chassis to actually move. On the stand the wheels are
off the ground, so:

- there is no load, and the actuation floor is a **loaded** property — the duty at
  which a wheel starts turning against the robot's own weight is not the duty at
  which a free wheel starts spinning;
- the OTOS reports nothing useful, because it is an optical ground-tracking
  sensor with a static scene under it (this caused a false "frozen sensor" report
  during sprint 130 — see
  [[otos-frozen-at-a-constant-on-tovez]]);
- the two +500 acceptance criteria that need translation — net heading change
  <= 3 deg, and camera-measured travel 500 +/- 25 mm — are unmeasurable on the
  stand for the same reason, and were explicitly deferred by ticket 006.

## Current provisional values

From sprint 130 ticket 001's duty sweep, on the stand, **n=3 and flagged low
confidence** (the sweep was cut short when the robot wedged, so the right wheel
has only 3 near-breakaway points):

- `vMin` 99.7 mm/s
- `biasMax` +/- 23.8 mm/s
- breakaway band 0.04-0.09 duty (44-100 mm/s)

Ticket 001's parallel-lines test came back **slope-dominated (fanned)**, which is
the OPPOSITE of the intercept-only hypothesis Stage C's bias adaptation assumes —
also low confidence, n=3. If that result holds up with real data, Stage C needs
revisiting. **This measurement is what would settle it.**

Note also that `vMin` snapping is already causing trouble: it rounds small planner
differential corrections up to ~99.7 mm/s, which turns a 3 mm/s trim into a lurch
— see the speed-floor half of
[[sprint-130-regressions-speed-floor-snaps-differential-and-shaper-defaults]].
A measured, loaded floor is likely lower than 99.7 and would reduce that damage
independently.

## Verification

- Actuation floor measured with the robot translating under its own weight, per
  wheel, per direction.
- Termination tolerance derived from that measurement rather than assumed.
- The parallel-lines question answered with n well above 3.
- Playfield rules apply: camera-supervised, geofence checked inside the move at
  ~10 Hz, `estop()` (never `stop()`) on every halt path — see
  `.claude/rules/playfield-testing.md`.

## Related

- Deferred from sprint 130 ticket 012.
- Blocked in practice by
  [[tovez-hard-silent-i2c-wedge-blocks-completing-the-population-duty-sweep]].
