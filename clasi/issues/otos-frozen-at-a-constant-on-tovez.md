---
status: pending
priority: low
---

# OTOS reports a frozen constant on tovez

## CORRECTION 2026-08-02 — this was probably NOT a fault

**The robot was on the STAND for every observation in this issue.** Wheels off
the ground, so the chassis never translated. The OTOS is an optical
ground-tracking sensor: with a static scene under it, a constant reading is
CORRECT behaviour, not a frozen sensor.

What the original report treated as damning -- "identical to the millimetre
across two sessions, metres of travel and ~195 deg of rotation" -- was metres of
WHEEL travel measured by the encoders on free-spinning wheels. The robot itself
went nowhere, and the OTOS said so.

Re-checked 2026-08-02 with the sensor confirmed physically connected:
`STATUS ... otos=1`, telemetry `otos=(48, -4, -114)` -- still the same constant,
still on the stand, still consistent with "no chassis motion".

The claim was made by comparing two screenshots without accounting for the test
regime, which is exactly the mistake `.claude/rules/playfield-testing.md` and
`hardware-bench-testing.md` exist to keep separate: stand facts and playfield
facts are not interchangeable.

**What would actually settle it:** move the chassis -- by hand across a surface,
or a real playfield run -- and confirm the OTOS pose changes. Until someone does
that, there is no evidence of a sensor fault here. Do not spend time chasing
one; if it reads constant while the robot is genuinely translating, THAT is the
bug, and this issue can be reopened at high priority with that evidence.

Retained (still true, still worth doing): a live-but-unchanging sensor is
indistinguishable in the GUI from one that is not wired in. A staleness/liveness
check would make the difference visible either way -- see the Work section.

Across two TestGUI sessions ~45 minutes apart on 2026-07-31, separated by metres
of travel and ~195 deg of accumulated rotation, the OTOS reported:

```
otos  x +48  y -4  th -0.1     (16:53)
otos  x +48  y -3  th +0.2     (17:31)
```

`x` identical to the millimetre. This is not drift — the sensor is stuck at a
constant while the robot moves freely.

Because `weight_heading_otos`/`weight_omega_otos` are 0.0, this contributes
nothing to the estimate today and so caused no motion error. It did cost
debugging time: a frozen-but-reporting sensor is indistinguishable in the GUI
from a sensor that is simply not wired in, and the mislabelled "Fused" trace
made it look like a fusion disagreement (see
[[testgui-trace-sources-naming-and-visibility]]).

It becomes a real hazard the moment anyone sets a nonzero fusion weight.

## Work

1. Determine whether the chip is being read at all, is returning a stale cached
   register, or is genuinely not tracking (mounting height / surface / lens).
   `flags` bit 0 (`otos_present`) and the per-cycle freshness bit
   `input.otos.present` are the first things to read.
2. **Add a staleness/liveness fault flag.** `StateEstimator` gates blending on
   `present` and an age window only — a sensor that reports fresh readings that
   never *change* passes both checks. Same class of defect as the wheel-frozen
   flag: commanded-but-not-moving needs its own bit, and so does
   reporting-but-not-changing.
3. Surface it in telemetry and as a red banner in the GUI, like the wheel-frozen
   indicator — see [[wheel-frozen-fault-flag-in-telemetry]].

## Acceptance

- Lift and rotate the robot by hand; OTOS pose changes.
- With the sensor disconnected or frozen, telemetry raises an OTOS-stale fault
  within 1 s and the GUI shows it.
