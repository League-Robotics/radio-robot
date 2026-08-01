---
status: pending
priority: high
---

# The robot runs at ~35% of commanded speed: three plant-gain constants that disagree

Pressing the unmanaged +500 button with a 150 mm/s request produces a measured
plateau of **~52 mm/s** (ratio 0.347), taking 10.5 s instead of ~4 s. Measured
on the bench 2026-07-31.

This path is **open loop by design** — `src/firm/app/drive.h:14`: *"There is no
controller here — duty is open loop from calibrated speed."* Closed-loop wheel
control lives in `Motion::Planner`'s duty stage (the *managed* button). So on
the unmanaged path, speed accuracy is entirely the calibration, and a uniform
speed deficit is a calibration defect, not a control defect.

## Three constants for one physical quantity

From `data/robots/tovez.json`:

| constant | value [duty/(mm/s)] | implies full duty = |
|---|---|---|
| `duty_per_speed_left`/`_right` | 0.00187325 | 534 mm/s |
| `vel_kff` | 0.0008 | 1250 mm/s |
| `_vel_kff_note`'s own derivation | 1/650 = 0.00154 | 650 mm/s |

`duty_per_speed` and `vel_kff` express the same thing and disagree by **2.34x**.
(`vel_kff` was deliberately detuned down from 0.00135 to 0.0008 in 106-002 to
kill a resonance — so it is knowingly not the plant truth. That is fine, but it
means nothing in the file records the plant truth.)

## The error decomposes into two factors that multiply

`App::Drive` inverts the `wheel_gain` correction *before* converting to duty:

| step | left | right |
|---|---|---|
| requested | 150.0 | 150.0 |
| after `(desired − intercept)/gain` | 98.4 | 109.4 |
| x `duty_per_speed` -> duty | 0.184 | 0.205 |
| observed | | ~52 mm/s |

- **1.52x** is thrown away by the `wheel_gain` inversion before duty is computed.
- **1.9x** more because duty 0.184 yields 52 mm/s — the real plant is ~282 mm/s
  per unit duty, not the 534 the config claims.

1.52 x 1.9 = **2.9x**, exactly the observed 0.347 ratio.

## The `wheel_gain` correction is pointing the wrong way

`wheel_gain_left_accel = 1.4703` encodes "command 100, measure 152" — the plant
*over*-delivers, so Drive commands 98 to get 150. The robot **under**-delivers
by ~1.9x. The correction is compounding the error rather than correcting it.

`_wheel_correction_note` in the same file says why, and warns about exactly
this: *"Measured against a SHARED duty_per_speed on both wheels — recalibrate
both wheels to one constant before re-characterizing, or the measurement
inherits the mismatch."* It was fitted to a plant that no longer matches and was
never refitted.

## MEASURED 2026-07-31 (duty sweep, firmware v0.20260731.13)

Two runs were needed. **The first was invalid** -- taken against firmware
flashed from an abandoned branch whose baked constants could not be read back
(there is no firmware->host config read-back path; `config.proto` has no
`ConfigSnapshot` arm). Its duty axis was computed from `tovez.json` constants
the robot was not necessarily using, and every conclusion drawn from it was
wrong. Recorded here because the failure mode is the reusable lesson:

> **Anchor on a measurement that needs no config constant before trusting one
> that does.** The saturation point (command a speed high enough that duty
> clamps to 1.0, then just read the speed) depends on nothing and takes 6
> seconds. It immediately showed the plant was fine, contradicting the sweep.

### Valid results, after reflashing from a known tree

Saturation (no constants involved): **L 760-795, R 696 mm/s at full duty** --
consistent with the historical 620-740 plateau measurements.

Linear fit, open loop via the `WHEELS` verb, 10 rungs 0.04-0.60, both directions:

| wheel | gain [mm/s per duty] | implied dps | breakaway [duty] |
|---|---|---|---|
| left  | 853.6 | 0.001172 | 0.102 |
| right | 837.8 | 0.001194 | 0.102 fwd / 0.164 rev |

**L/R spread: 1.9%.**

### What this overturns

- **`duty_per_speed` is too HIGH, not too low.** Config 0.00187325 claims 534
  mm/s per duty; the plant delivers ~845. Recommended default: **0.001182**.
  The robot should run ~1.6x FASTER than commanded, not slower -- so the
  observed 35%-of-commanded was never this constant.
- **There is no significant wheel mismatch.** 1.9%, not the 28% inferred from
  the invalid run. The "right wheel is mechanically weak" conclusion drawn
  across this whole session was an artifact of stale firmware.
- **Breakaway is uniform (~0.10) except right-reverse (0.164)** -- not the
  0.027-0.240 spread the invalid run reported. Still worth noting that all four
  are ~3-5x the configured `output_deadband` of 0.03.

### Still open

If `duty_per_speed` is 1.6x too generous, the measured 35%-of-commanded speed is
unexplained by calibration and needs its own root cause. The leading candidate
remains a live-config path overwriting `dutyPerSpeed` at connect time -- the
`pid.kff` routing documented as removed at `configurator.cpp:122-133`. Confirm
by re-running `duty_sweep.py` immediately after a connect-time calibration push
and comparing.

## Proposed work

1. Set `wheel_gain_* = 1.0`, `wheel_intercept_* = 0.0` (identity), as that note
   itself requires before any recalibration.
2. Open-loop duty sweep (0.10 -> 0.60), measure steady-state speed **per wheel**,
   fit `duty_per_speed` per wheel from the measured line. This also finally
   quantifies the left/right mismatch as a measurement rather than an inference
   — see [[testgui-unmanaged-drive-lease-expiry-and-terminal-pivot]], where a ~12 mm/s
   L/R difference under identical commands bent every straight leg.
3. Reconcile `vel_kff` to the measured gain, or record in the file why it is
   deliberately below it.
4. Re-run +500 against the acceptance spec below.

## Acceptance spec for the +500 button (agreed with stakeholder 2026-07-31)

Shape:
1. Both wheels rise to cruise in <= 0.3 s.
2. Flat plateau at **150 mm/s** held for the leg.
3. Ripple <= +/-10 mm/s (+/-7%) frame to frame.
4. |vL − vR| <= 10 mm/s through cruise — the two traces overlay.
5. Taper over the last 60 mm to the 90 mm/s floor, then stop. Neither wheel
   reaches 0 while the other still moves.
6. Elapsed ~3.3 s + ~0.5 s taper ~= 4 s.

Endpoint:
7. Encoders land 500 +/- 15 mm, wheels within 10 mm of each other.
8. Net heading change <= 3 deg, cross-track <= 30 mm.
9. Camera-measured travel 500 +/- 25 mm — the encoders are not allowed to be
   right on their own.

## Open question for the stakeholder

The stakeholder expected this path to be "locked in with a PID controller." It
is not, by design. Either the expectation moves to the managed button, or the
unmanaged path gains a controller — which is a deliberate architecture change to
a path documented as open loop, not a bug fix.

## Also

The "Wheel speed — commanded vs actual" chart plots only *actual* L/R; the
commanded series is never drawn. The one comparison that would settle this at a
glance is missing from the graph.
