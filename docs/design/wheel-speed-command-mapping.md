# Commanded → actual wheel speed: the feedforward mapping

Measured 2026-07-27 on tovez (bench stand, wheels free), open-loop firmware
— no wheel PID anywhere in the path.

**Both wheels ran an identical calibration constant during the measurement**:
`duty_per_speed_left = duty_per_speed_right = 0.00187325` (= 1/533.8, the
average of the two old per-wheel values), and `crawl_pulse = 0` so
`crawlDuty()` was a pure pass-through that shaped no sample. Because the two
wheels were driven through the same conversion, **every asymmetry reported
below is real hardware**, not an artifact of the constants. Absolute plant
gain is recovered as `slope × 533.8`.

## Why this measurement exists

The velocity PID needs a **first guess**: given a desired wheel speed, what
command lands closest to it immediately, so the loop only trims rather than
hunts. This study answers three questions about that map:

1. What shape does it need — scalar, affine, polynomial?
2. Does direction of approach matter (accelerating vs decelerating into a
   setpoint)?
3. How accurate can the first guess be — i.e. what must the PID still absorb?

## Method

50 speeds equally spaced over [0, 500] mm/s. One **pass** visits every speed
in some order, stepping speed-to-speed **without stopping**, holding each
command and recording both wheels' velocity 0.5 s after it lands (median of
the telemetry velocity registers over 0.42–0.58 s, robust to the occasional
wild sample). Both wheels are driven together at the same commanded speed —
the condition the robot operates in — so each trial yields a point per motor.

Three pass orderings — **ascending**, **descending**, **random** — repeated
for 12 passes / **600 trials** across two batches. Ordering is the point:
because the wheels never stop inside a pass, each trial approaches its
setpoint from whatever speed preceded it. Every trial records its own `prev`
speed and is classified `accel` (cmd > prev) or `decel` (cmd < prev)
**individually**, so the random passes contribute to both classes and the
classification never depends on the pass label.

Every command is ack-verified with retry; across 600 trials, zero were lost.

Analysis excludes commands above 450 mm/s (approaching the ~500 mm/s ceiling,
where saturation flattens the response) and the zero command, leaving **528
trials**, 264 per direction.

Tooling: `src/tests/bench/speed_map.py`. Data: `speed_map_sym{,2}.csv` (raw),
`speed_map_sym_combined.csv` (classified). Plot: `speed_map_sym.png`.

## Result 1 — the map is a straight line; do not fit a polynomial

Scalar, affine and quadratic fits are indistinguishable in quality (the
quadratic coefficient is ~10⁻⁵ and changes rms not at all). **A scalar per
wheel is the right model**, with an optional offset. Nothing about this
plant needs curvature.

## Result 2 — the four equations

`measured = a·cmd + b`, 264 trials each:

| wheel | direction | a | b [mm/s] | rms [mm/s] | true gain [mm/s per duty] |
|---|---|---|---|---|---|
| left | accel | 1.0993 | +9.23 | 6.8 | 587 |
| left | decel | 1.1036 | +11.73 | 5.6 | 589 |
| right | accel | 1.0823 | −3.66 | 9.0 | 578 |
| right | decel | 1.0681 | +5.18 | 7.2 | 570 |

Inverted — the feedforward the PID needs:

| wheel | direction | command for a desired speed |
|---|---|---|
| left | accel | `(desired − 9.23) / 1.0993` |
| left | decel | `(desired − 11.73) / 1.1036` |
| right | accel | `(desired + 3.66) / 1.0823` |
| right | decel | `(desired − 5.18) / 1.0681` |

## Result 3 — the wheels are nearly identical; the old asymmetry was fake

With one shared constant, the wheels' slopes differ by only:

| direction | left | right | right/left |
|---|---|---|---|
| accel | 1.0993 | 1.0823 | −1.5% |
| decel | 1.1036 | 1.0681 | −3.2% |

True gains: **left ≈ 588, right ≈ 574 mm/s per duty — about 2% apart.**

The previous study, run with the mismatched 1/560 and 1/510 constants,
reported a ~10% left/right difference and attributed it to the gearboxes.
That was **mostly the calibration, not the hardware**: the old constants
themselves differed by 10%, and the measurement inherited it. This is the
reason to calibrate both wheels identically before characterizing them.

## Result 4 — direction of approach: real, but small and unequal

Offset difference (decel − accel), the hysteresis term:

| wheel | accel offset | decel offset | difference |
|---|---|---|---|
| left | +9.23 | +11.73 | **+2.5 mm/s** |
| right | −3.66 | +5.18 | **+8.8 mm/s** |

Decelerating into a setpoint still lands *higher* than accelerating into it
— the wheel is coasting down through the target at the 0.5 s sample — but
the effect is **not the same size on both wheels** (2.5 vs 8.8 mm/s), and on
the left it is smaller than the trial-to-trial scatter. The slopes are
unaffected: the hysteresis is an offset, not a gain change.

Practical reading: the direction term is worth ~1–2% at cruise on the right
wheel and is lost in the noise on the left. It is a second-order correction,
not a headline effect.

## Result 5 — the wheels overshoot ~8–10%

All slopes exceed 1, so the wheels run faster than commanded under the shared
1/533.8 constant. Corrected per-wheel calibration, from the overall slope
(both directions pooled):

| wheel | overall slope | true gain | corrected `duty_per_speed` |
|---|---|---|---|
| left | 1.1353 | 606 | 1/606 = 0.00165005 |
| right | 1.0773 | 575 | 1/575 = 0.00173890 |

(The pooled slope runs slightly above the per-direction slopes because it
absorbs the positive offsets; either form is usable — the per-direction
equations in Result 2 are the more precise statement.)

Applying these should bring the measured slopes to ≈1.0 on re-measurement,
which is the acceptance test for a calibration, and should remove the square
tour's leg overrun (500 mm legs previously measuring ~553 mm).

## What the residual is — and is not

Residual rms is ~6–9 mm/s and does not shrink with a better functional form.
Two candidate explanations were tested and rejected on the earlier dataset:

- **Stiction / from-rest breakaway.** Only 12 of 264 accel trials start from
  rest; excluding them moved the fit by 0.04 mm/s.
- **Insufficient settling on large steps.** Correlation between residual and
  step size is ~0.2 for accel, ~0 for decel.

The residual is genuine trial-to-trial scatter of the plant. **This is the
floor on any feedforward**: no map gets a first guess closer than ~6–9 mm/s
(~2% at cruise). Closing that is the PID's job, and it is small enough that a
slow integral absorbs it quickly.

An earlier draft attributed the accel/decel separation to warm-up and battery
drift. **That was wrong and is withdrawn**: the repeat rounds run on an
already-warm machine with no appreciable battery change, and the separation
tracks each trial's own direction of approach, which the randomized passes
isolate from any time-ordered effect.

## The bottom of the range

Below ~30 mm/s commanded, a wheel arriving **from rest** does not move
(gearbox breakaway, 1–6% duty and state-dependent —
`src/tests/firmware/duty_min/RESULTS.md`). A wheel arriving at the same speed
**while already rolling** keeps rolling: breakaway applies to a stopped
wheel, not a moving one — the same accel/decel asymmetry appearing at the
bottom of the range.

No feedforward fixes this. Sustained motion below breakaway needs either a
brief kick above ~30 mm/s to start the wheel rolling before settling to the
target, pulsed drive (`crawl_pulse`, currently off), or an integral permitted
to wind into the dead zone. The kick approach is untested — the minimum kick
*duration* and the lowest sustainable rolling speed are both unmeasured.

## Validity and re-measurement

These constants describe **one robot** (tovez) on one day. The absolute gains
(≈588 left, ≈574 right mm/s per duty) are the per-robot hardware property;
slopes are only meaningful together with the calibration they were measured
against. Both belong in robot configuration, never in source — see
`clasi/issues/wheel-speed-feedforward-calibration.md`.

**Calibrate both wheels to the same constant before characterizing**,
otherwise the measurement inherits the very asymmetry it is meant to find
(Result 3). Then measure, apply the corrected per-wheel values, and
re-measure: slopes returning to ≈1.0 is the acceptance test.

Re-measure when the drivetrain changes (motors, gearboxes, wheels, tyres),
when commissioning a robot, or when tour leg length drifts by more than a few
percent. A full 600-trial characterization takes ~8 minutes.
