---
status: pending
sprint: '070'
tickets:
- 070-004
---

# TestGUI Sim Errors: "From Calibration" button (inverse-calibration plant)

## Idea

Add a button next to **Apply** in the TestGUI Sim Errors panel that sets the
sim error knobs to the **inverse of the active robot's calibration**, so the
firmware's baked-in calibration exactly compensates the injected plant errors
and the sim robot behaves perfectly (modulo noise). The noise sigmas
(encoder noise mm, OTOS linear noise, OTOS yaw noise) are left untouched.

## Why

The sim firmware build bakes the active robot's calibration into
`source/robot/DefaultConfig.cpp` (auto-generated from `data/robots/tovez.json`:
`rotationalSlip=0.92`, `trackwidthMm=128`, `mmPerDegL/R`,
`otosLinearScale/AngularScale`). The plant, however, is ideal — so the
firmware's calibration *over-corrects* in sim (e.g. the known RT 9000
over-rotation to ~95° because `beginRotation` inflates the arc target by
÷0.92 while the plant has zero scrub). This button makes the plant match
what the calibration was fitted against, so calibrated firmware + errored
plant = perfect closed-loop behavior.

## Investigated mapping (inverse of calibration)

Key finding: `mmPerDegL/R` and the OTOS scale calibrations act only at the
**real-HAL sensor boundary** (`hal/real/Motor.cpp` deg→mm conversion,
`hal/real/OtosSensor.cpp` register programming) and are **inert in sim**
(the sim HAL reports plant mm / plant pose directly) — so their inverse is
the neutral value. The two calibrations live in the sim control path are
`rotational_slip` (used by `control/Odometry.cpp` predict and
`control/PlannerBegin.cpp` `beginRotation` arc inflation) and `trackwidth`.

Button sets:

| Knob | Value | Source |
|---|---|---|
| turn slip (`slip_turn_extra`) | 0.0 | scrub modeled truthfully via body rot scrub instead |
| body rot scrub | `calibration.rotational_slip` (0.92) | active robot config |
| body lin scrub | 1.0 | no linear-scrub calibration exists |
| motor offset L/R | 1.0 | no per-side motor calibration (kScale* all 1.0) |
| trackwidth (mm) | `geometry.trackwidth` (128.0) | matches firmware belief |
| enc scale err L/R | 0.0 | mmPerDeg calibration inert in sim |
| OTOS lin/ang scale err | 0.0 | OTOS register calibration inert in sim |
| OTOS lin/yaw drift | 0.0 | no drift calibration exists |
| encoder noise, OTOS lin/yaw noise | *untouched* | stakeholder-specified exception |

Calibration values come from
`robot_radio.config.robot_config.get_robot_config()` (the active robot).
After populating the spinboxes the button applies via the same path as
Apply (persist via `sim_prefs.save_sim_error_profile` + live-apply when
connected).

Caveat worth handling in design: `DefaultConfig.cpp` is generated at sim
*build* time; if the active robot JSON changed since the sim library was
built, the baked firmware values and the JSON can disagree. Optionally
cross-check live firmware values (`GET rotSlip`, `GET tw`) when connected
and warn on mismatch.

## Acceptance

With the button pressed in Sim mode (and noise at 0): `RT 9000` lands
~90.0° and closed-loop drives land on target — the known ideal-plant
over-rotation to ~95° disappears.
