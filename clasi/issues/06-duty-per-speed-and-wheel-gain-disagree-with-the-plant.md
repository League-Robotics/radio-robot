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

## MEASURED 2026-07-31 (duty sweep on the stand)

`src/tests/bench/duty_sweep.py`, open loop via the `WHEELS` verb, 12 rungs
0.04-0.60 duty, both wheels, both directions. Data in `duty_sweep.csv`, chart in
`duty_sweep.png`.

| wheel | dir | gain [mm/s per duty] | breakaway [duty] | implied dps |
|---|---|---|---|---|
| left  | fwd | 362.5 | 0.027 | 0.002759 |
| left  | rev | 444.5 | 0.087 | 0.002250 |
| right | fwd | 335.4 | 0.054 | 0.002982 |
| right | rev | 427.4 | **0.240** | 0.002339 |

**Config claims 534 mm/s per duty. Every direction of every wheel is weaker than
that** — mean ~392, so `duty_per_speed` should be ~**0.00255**, not 0.00187. The
config is ~1.4x too optimistic, which is most of the missing speed.

**The bigger finding is the deadband, not the gain.** `output_deadband` is
configured at **0.03**; measured breakaway ranges **0.027 to 0.240** — up to
**8x** the configured value. The right wheel in reverse does not turn at all
below 0.24 duty: at rungs 0.04 / 0.09 / 0.14 / 0.19 it measured exactly 0 mm/s.

This reframes the problem. The apparent 28% "L/R gain spread" in a combined fit
is mostly breakaway asymmetry leaking into the slope, not a gearbox difference —
which is why a host-side trim on encoder counts could never fix it, and why
forward and reverse legs behave like different robots. It is also the honest
explanation for the "right wheel is mechanically weak" impression: it is not
weak in gain (335 vs 362 fwd), it is nearly immovable from rest in reverse.

Follow-ups this opens:

- Fit `speed = m*(duty - breakaway)` per wheel per direction rather than one
  slope+intercept — the current fit conflates the two.
- `output_deadband` needs to be per wheel and per direction, or the boost at
  `nezha_motor.cpp:279-284` is meaningless for three of the four cases.
- A 0.24 breakaway on right-reverse warrants a mechanical inspection before any
  more calibration is fitted on top of it.

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
