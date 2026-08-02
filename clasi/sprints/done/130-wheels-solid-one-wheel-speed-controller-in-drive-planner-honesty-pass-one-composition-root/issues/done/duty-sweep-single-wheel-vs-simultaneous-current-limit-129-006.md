---
status: done
priority: high
sprint: '130'
tickets:
- 130-001
---

# duty_sweep.py found the plant saturates well below its historical ceiling, and two-wheel-simultaneous is worse than one-wheel-alone -- likely a power-delivery limit, not a calibration error

## Description

Bench finding from sprint 129 ticket 006 (`duty_sweep.py` plant-gain
measurement on `tovez`, 2026-08-01), surfaced while measuring the real
per-wheel open-loop duty-to-speed gain that ticket set out to fix.

### Finding 1: the response saturates and DECLINES well below the historical ceiling

The open-loop duty->speed response is only linear through duty~0.30
(left) / ~0.40 (right); above that it saturates and then DECLINES, even
at full duty=1.0 (measured only ~89-94 mm/s). This is far below the
historical ~500-560 mm/s ceiling recorded in `tovez.json`'s own
`control._drive_calibration_note` history (the 2026-07-27 `speed_sweep`
measurement this ticket's own note references).

Confirmed REPRODUCIBLE, not drift/heating/noise, via an interleaved
re-test: commanding duty 0.30/0.50/0.30/0.60/0.30/0.20/0.30 in that
sequence, duty=0.30 measured ~111-122 mm/s EVERY time regardless of its
position in the sequence, while duty=0.50/0.60 measured ~89-91 mm/s every
time they were tried -- ruling out simple cumulative motor heating over
the test's several-minute duration (which would show a monotonic
time-based decline, not a duty-keyed one).

### Finding 2 (more urgent): two wheels driven together underperform one wheel alone at the same command

Commanding BOTH wheels together at 150 mm/s (matching the original
2026-07-31 incident repro exactly, post wheel_gain-identity-fix from
this same ticket) measured only vL~104 / vR~68 mm/s. The SAME left wheel
driven ALONE at the identical 150 mm/s command measured ~129 mm/s --
notably higher. This points at a shared-power-budget/current-limit (or
battery-sag-under-combined-load) effect that `duty_sweep.py`'s
one-wheel-at-a-time methodology cannot characterize or correct, since it
only ever loads one wheel at a time by design.

### Why this matters

No static per-wheel `duty_per_speed` calibration alone can make the
"+500 button" (both wheels driven simultaneously) reliably hit a 150 mm/s
cruise under today's power conditions -- the plant may simply not have
enough current budget for both wheels to reach 150 mm/s together,
regardless of how `duty_per_speed` is tuned. Ticket 129-007's planned
adaptive `gainTrim` learner explicitly assumes it is compensating a
*calibration* residual around ticket 006's static baseline; if the real
residual is instead a power-delivery ceiling, gainTrim would just wind up
against a wall it cannot climb (it is deliberately slow/gentle and gated
off during transients -- not designed to fight a hard current limit).

### Reference data

- `src/tests/bench/duty_sweep.csv` / `duty_sweep.png` (129-006, captured
  2026-08-01) -- the full one-wheel-at-a-time sweep, both directions,
  both wheels, showing the saturation/decline past duty~0.30-0.40.
- `data/robots/tovez.json`'s `control._drive_calibration_note`, the
  2026-08-01 entry -- full measurement writeup and the restricted-range
  fit this ticket shipped (`|duty|<=0.30`).

## Recommendation

Re-run `duty_sweep.py` (its `--from-csv`/`--fit-duty-max` flags support
re-fitting without re-driving the robot, for the single-wheel half) against
a freshly-charged/verified battery, and add a **two-wheel-simultaneous**
sweep variant (drive both wheels together across a duty grid, not just one
at a time) to determine whether this is a session-specific battery-sag
artifact or a hard Nezha-board/battery current-limit ceiling -- before
129-007's adaptive learner is asked to compensate a residual that may
actually be a power-delivery limit rather than a calibration error.
