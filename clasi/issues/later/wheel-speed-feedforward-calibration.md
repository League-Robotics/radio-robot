---
status: pending
---

# Wheel-speed feedforward: calibration values in robot config, applied in App::Drive

## Description

`App::Drive` converts a commanded wheel speed to duty with a single
per-wheel scalar (`control.duty_per_speed_left` / `_right`). Measured on the
bench, that conversion is **5–11% off**: both wheels run faster than
commanded, which shows up directly as the square tour's 500 mm legs
measuring ~553 mm.

A 600-trial characterization (`docs/design/wheel-speed-command-mapping.md`)
established what the correct mapping looks like:

- The relationship is **linear** — scalar or affine; a polynomial fits no
  better (quadratic coefficient ~10⁻⁵, rms unchanged).
- The two wheels differ by only **~2%** in true gain (588 vs 574 mm/s per
  duty) once both are measured through the same constant. An earlier run
  with mismatched constants reported ~10% and blamed the gearboxes — that
  was the calibration, not the hardware. Per-wheel values are still
  warranted, but the wire `kff` key (which sets both wheels at once) is a
  closer approximation than previously thought.
- **Direction of approach matters, unequally.** Decelerating into a setpoint
  lands higher than accelerating into it by **+2.5 mm/s (left)** and
  **+8.8 mm/s (right)** — a constant offset, not a slope change. On the left
  this is smaller than the trial-to-trial scatter; it is a second-order
  correction, not a headline effect.

Measured constants (tovez, 528 trials ≤450 mm/s, `measured = a·cmd + b`).
Both wheels ran an **identical** calibration constant during the run
(`duty_per_speed` = 1/533.8 both sides, crawl disabled), so the asymmetry
below is real hardware rather than inherited from the constants:

| wheel | direction | a | b [mm/s] | true gain [mm/s per duty] |
|---|---|---|---|---|
| left | accel | 1.0993 | +9.23 | 587 |
| left | decel | 1.1036 | +11.73 | 589 |
| right | accel | 1.0823 | −3.66 | 578 |
| right | decel | 1.0681 | +5.18 | 570 |

Corrected per-wheel calibration (pooled slope × 533.8):

| wheel | true gain | corrected `duty_per_speed` | current value |
|---|---|---|---|
| left | 606 | 0.00165005 | 0.00187325 |
| right | 575 | 0.00173890 | 0.00187325 |

The purpose is the **feedforward first guess** for the velocity PID: get as
close as possible the instant a setpoint is commanded, so the loop only
trims. The measurement also bounds how good that guess can be — residual
scatter is ~5–10 mm/s and is a property of the plant, not of the fit (both
stiction and settling-time explanations were tested and rejected), so the
PID must absorb roughly 2–3% at cruise no matter how good the map is.

## Cause

The current single scalar per wheel was derived from an earlier, coarser
sweep and carries no offset and no direction term. It was also the value
that got hard-coded in `drive.h` before being promoted to config (commit
`5c86c87a`), so the numbers now in the robot JSONs are the *old* ones
(L 1/560, R 1/510) — they have never been updated from this study.

## Proposed fix

### 1. Configuration keys

Extend the `control` block in `data/robots/*.json` (and the schema),
replacing `duty_per_speed_left` / `duty_per_speed_right` with the
calibration this study produced. Suggested shape — one slope per wheel plus
the two offsets, matching the measured structure rather than storing four
independent lines:

Absolute plant gain = slope × old divisor, giving the corrected values to
put in the config:

| wheel | true gain [mm/s per duty] | corrected `duty_per_speed` | old |
|---|---|---|---|
| left | 590 (593 accel / 587 decel) | 0.00169390 | 0.00178571 |
| right | 567 (568 accel / 566 decel) | 0.00176449 | 0.00196078 |

Note the wheels' true gains differ by only ~4%, not the ~10% the old
560/510 pair implied — most of that apparent asymmetry was an artifact of
the old constants.

```json
"control": {
  "wheel_gain_left":          1.054,   // measured/commanded, dimensionless
  "wheel_gain_right":         1.111,
  "wheel_offset_left_accel":    5.3,   // [mm/s]
  "wheel_offset_left_decel":   12.7,   // [mm/s]
  "wheel_offset_right_accel":  -3.3,   // [mm/s]
  "wheel_offset_right_decel":   3.2    // [mm/s]
}
```

All REQUIRED, following the existing fail-closed posture in
`src/scripts/gen_boot_config.py` (`_require()` aborts codegen with a named
key rather than shipping a boot image). Bake into `Config::DriveBootConfig`
alongside the existing `crawlPulse`; `App::Drive` keeps **no defaults** and
refuses to drive uncalibrated (already the case — `drive.cpp`'s
`calibrated_` guard).

Note the duty conversion still has to happen somewhere: this mapping is
speed→speed (what the wheels do vs what was asked), while Drive currently
holds speed→duty. Decide explicitly whether Drive composes the two
(`desired → corrected command → duty`) or whether the calibration is folded
into a single speed→duty constant per wheel per direction. The former keeps
the measured quantities separable and re-measurable; the latter is one
multiply.

### 2. Use in `App::Drive`

Drive already knows the previous commanded speed per wheel (it holds the
armed command's targets), so the direction of approach is available without
new state:

```
direction = (desired > previousTarget) ? accel : decel
command   = (desired - offset[wheel][direction]) / gain[wheel]
```

Apply per wheel independently — the two wheels can be moving in opposite
directions during a pivot, and each has its own gain and offsets.

Open question for whoever implements: the offset is a **settling-time
artifact** (at 0.5 s a decelerating wheel is still coasting through the
target). Once a velocity PID is closing the loop, the steady-state error the
offset corrects for may be absorbed by the integral instead, making the
direction term redundant or even harmful. Recommendation: implement the
per-wheel **gain** first (that is the ~8–10% error, unambiguous), and treat
the direction-dependent offset as a separate, measured-again-with-the-PID-in
decision rather than shipping both at once — especially since it is only
clearly above the noise on the right wheel.

### 3. Calibration procedure

Document in the bench catalogue and reference from
`.claude/rules/hardware-bench-testing.md`:

**Set both wheels to the same constant first.** Characterizing through
mismatched per-wheel constants makes the measurement inherit that mismatch
and report it as hardware asymmetry (this is how the ~10% figure arose).

```bash
# Robot on the stand, wheels free, freshly charged.
uv run python src/tests/bench/speed_map.py --port /dev/cu.usbmodem2121102 \
    --out src/tests/bench/speed_map.csv
uv run python src/tests/bench/speed_map.py --port /dev/cu.usbmodem2121102 \
    --out src/tests/bench/speed_map2.csv
```

Each invocation runs 3 orderings × 2 rounds = 300 ack-verified trials in
~4 minutes. Two invocations give the 600-trial set the constants above came
from; one is enough for a re-check.

Then fit: combine the batches, reconstruct each trial's `prev` (sequential
within a pass, each pass starting from rest), classify `accel`/`decel` per
trial, drop commands >450 mm/s and the zero command, and least-squares fit
`measured = a·cmd + b` per wheel per direction. The analysis is currently
ad-hoc; **fold it into a `--fit` mode of `speed_map.py`** that emits the
JSON block ready to paste into the robot config, so calibration is one
command and not a notebook exercise.

Acceptance for a calibration: the square tour's legs measure within a few
percent of 500 mm, and the fitted slopes land near 1.0 when the tour is
re-run with the new constants (a slope of 1.0 means commanded == actual).

Re-measure when the drivetrain changes (motors, gearboxes, wheels, tyres),
when commissioning a new robot, or when tour leg length drifts more than a
few percent.

## Verification

- Codegen fails closed on a missing key (remove one from the active robot
  JSON; `gen_boot_config.py` must abort naming it).
- With the new constants flashed, `speed_map.py` re-run shows fitted slopes
  ≈ 1.0 and offsets ≈ 0 — i.e. commanded speed is now achieved speed.
- `src/tests/bench/wheels_square_tour.py` legs measure ~500 mm rather than
  ~553 mm; total path error drops from ~+10% toward the encoder-truth noise
  floor.

## Related

- `docs/design/wheel-speed-command-mapping.md` — the study: method, the four
  equations, why a polynomial is not needed, and what the residual is.
- `src/tests/firmware/duty_min/RESULTS.md` — gearbox breakaway (1–6% duty,
  state-dependent). Below ~30 mm/s from rest no feedforward helps; that
  range needs `crawl_pulse` or integral wind-up, decided separately.
- Commit `5c86c87a` — the promotion of these constants out of `drive.h` and
  `main.cpp` into robot config, which this issue supplies real values for.
- `clasi/issues/command-ingestion-ring-buffered-comms-subsystem-routing-two-stops.md`
  §6 — the config-promotion requirement this builds on.
