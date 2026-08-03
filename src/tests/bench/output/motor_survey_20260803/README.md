# Motor survey — 4 robots, 8 motors, 2026-08-03

Every board flashed from **one firmware image** (`MICROBIT.hex`, built from
`active_robot.json` → `data/robots/tovez.json`), so identical code and
identical baked calibration ran everywhere. The only remaining variable is
hardware. All robots on stands, wheels off the ground.

Produced by [`motor_survey.py`](../../motor_survey.py); compared by
[`analyze_motor_survey.py`](../../analyze_motor_survey.py):

```bash
uv run python src/tests/bench/motor_survey.py --uids <uid>,<uid>,… \
    --out src/tests/bench/output/motor_survey_20260803
uv run python src/tests/bench/analyze_motor_survey.py \
    --survey src/tests/bench/output/motor_survey_20260803
```

**Chart: [motor_comparison.png](motor_comparison.png)** · machine-readable
[summary.csv](summary.csv) (48 rows: board × run × profile × wheel) · per-run
charts `<board>_<run>.png` and raw samples `<board>_<run>_samples.csv`.

## Board identity — YOU need to fill in the physical column

Unprogrammed boards have no name, and every board flashed from one image
announces the *same* baked name, so the announcement cannot identify a board
here. Labels are the UID tail.

| label | UID | port at survey time | which physical robot / wheels? |
|---|---|---|---|
| `tovez` | `…a8fdb5e413abb276…` | /dev/cu.usbmodem2121202 | the known robot |
| `board-312bde85` | `…312bde85515a72e6…` | /dev/cu.usbmodem212202 | **?** |
| `board-aba2e384` | `…aba2e384f40cfd6c…` | /dev/cu.usbmodem212302 | **?** |
| `board-b8e12372` | `…b8e12372c44f4f67…` | /dev/cu.usbmodem212402 | **?** |

Ports move on re-enumeration — the UID is the durable identity.

**Only three new boards enumerated, not four.** The fifth micro:bit on the hub
is `getez`, which is running RADIOBRIDGE relay firmware; it was deliberately
not flashed (`motor_survey.py` refuses a relay without `--allow-relay`,
because the untethered/field path depends on it).

## Metric

**Plateau speed reached** — the fraction of commanded speed a wheel actually
holds mid-profile — at each motor's *best* observed run. Measured away from
the profile's ends, so it is not contaminated by start/stop behaviour, and
best-of removes the cold-start transient.

Absolute millimetres are **not** comparable across boards: one baked
`mm_per_wheel_deg` is applied everywhere, so a board with different-sized
wheels reports different absolute travel for identical behaviour. The
*ratio* is what survives that, which is why the comparison is in percent.

## Capability

| board | left | right | L/R gap | weaker wheel |
|---|---:|---:|---:|---:|
| board-aba2e384 | 95.5% | **99.1%** | 3.6 | 95.5% |
| board-b8e12372 | 95.5% | 91.5% | 4.0 | 91.5% |
| board-312bde85 | 91.7% | 89.7% | 2.0 | 89.7% |
| tovez | 96.0% | **84.9%** | **11.1** | 84.9% |

**8 motors: mean 93.0%, sd 4.2, range 84.9–99.1% — a 14.2-point spread.**
Strongest `board-aba2e384 R` (99.1%); weakest `tovez R` (84.9%).

## On matching motors

Two different defects, and they need different fixes:

- **Both wheels weak** (`board-312bde85`, 91.7/89.7) — the robot drives
  straight but short. One scale factor corrects it.
- **Wheels disagree** (`tovez`, 96.0/84.9, an 11.1-point gap) — the robot
  pulls to one side. No scalar fixes this; it is a per-wheel correction or a
  mismatched pair.

Three of four robots are matched to within 2–4 points. `tovez` — our
reference robot, the one every calibration in `data/robots/tovez.json` was
measured on — is the outlier at 11.1. That is worth knowing on its own:
**the robot we calibrate against has the worst-matched pair in the set.**

Whether to match motors at build time depends on which spread dominates. The
within-robot gaps (2.0–11.1) are comparable to the between-motor spread
(14.2), so pairing motors by measured capability would meaningfully tighten
the fleet — but see the caveat below before acting on this ranking.

## Caveat — the ranking is not fully converged

Every board improves from cold to warm, and **some had not stopped improving
when the survey ended**, so best-of-3 understates them. Plateau tracking per
run (trapezoid/square):

| board | wheel | cold | warm1 | warm2 | converged? |
|---|---|---|---|---|---|
| board-312bde85 | L | 84.9/86.6 | 91.7/90.7 | 91.7/90.7 | yes |
| board-312bde85 | R | 78.6/84.6 | 86.4/88.7 | 89.1/89.7 | still rising |
| board-aba2e384 | L | 87.4/89.8 | 91.7/92.7 | 95.3/95.5 | **still rising** |
| board-aba2e384 | R | 86.4/91.5 | 92.0/97.2 | 94.1/99.1 | **still rising** |
| board-b8e12372 | L | 91.7/95.5 | 93.5/95.5 | 93.6/95.5 | yes |
| board-b8e12372 | R | 86.4/84.0 | 90.2/86.8 | 88.4/91.5 | still rising |
| tovez | L | 91.7/90.8 | 93.8/95.5 | 96.0/95.5 | yes |
| tovez | R | 80.7/81.1 | 84.7/83.5 | 84.9/84.0 | **converged, and weak** |

So capability and *rate of convergence* are partly confounded at 3 runs. A
longer soak (6–8 runs per board) would separate them. What is already solid:
`tovez R` is genuinely weak — it converged and stopped at 84.9% — while
`board-aba2e384` was still climbing and may be better than 99.1%.

## The cold-start deficit is firmware-wide, not a tovez defect

Mean plateau tracking, both wheels:

| board | cold | warm | gain |
|---|---:|---:|---:|
| board-312bde85 | 83.7% | 89.8% | +6.2 |
| board-aba2e384 | 88.8% | 94.7% | +5.9 |
| tovez | 86.1% | 89.7% | +3.7 |
| board-b8e12372 | 89.4% | 91.9% | +2.5 |

**All four boards drive short after a reset and converge with use.** This
confirms the earlier single-robot finding as firmware behaviour rather than
one bad robot — every robot's first run after power-on is its worst.
