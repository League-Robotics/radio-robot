---
status: done
priority: medium
---

# +500 button transient criteria not met at shipped Stage B=0 gains; plant-gain/saturation drop vs ab303ee3 baseline unexplained

## Description

Context: sprint 130 ticket 006 (bench acceptance for App::Drive's unified
wheel-speed controller) re-verified the 06-issue's own +500-button
acceptance spec on real hardware (tovez, firmware v0.20260801.18,
2026-08-02). Full data/scripts:
`src/tests/bench/wheel_controller_ab_bench.{py,csv,png}`,
`src/tests/bench/bias_convergence_150.{csv,png}`.

### Finding 1 — +500 transient criteria fail at shipped gains; a real Stage B sweep may fix it

Stage C (bias trim) is CONFIRMED working correctly — a continuous 90s
`wheels()` hold at cmd 150mm/s (no intermediate `estop()`) showed bias
converging smoothly (biasRight 0 -> +14.0mm/s, biasLeft settling near -3
to -5mm/s), both well inside the ±23.8mm/s clamp, velocity converging to
within ~5mm/s of 150mm/s by t~15-20s. But the +500 button always starts
from a FRESH bias=0 (a real button press is a cold start), and Stage C's
own `tauAdapt=30s` is far too slow to help within one ~4s button press —
so the +500 spec's fast criteria (rise <=0.3s, ripple <=10mm/s, L-R split
<=10mm/s) depend on Stage B (the fast PID), which ships at all-zero gains
by design (Open Question 4, ticket 004).

Measured at shipped gains (Stage B=0): rise=0.59s (FAIL, bound 0.3s),
ripple L/R=24.3/22.5mm/s (FAIL, bound 10), max L-R split=20.9mm/s (FAIL,
bound 10). Endpoint criteria PASS (elapsed 3.85s, encoders 510/502mm vs
500±15mm target, split 8mm, taper reaches the 90mm/s floor without either
wheel hitting zero).

A single exploratory Stage B trial (kp=0.3, ki=0.02, iMax=20,
kaff=0.23=plant_tau, pidMax=30, pushed live via `config()`) cut rise to
0.18s (clears the bound!) but WORSENED ripple to 33.7mm/s and left the
L-R split at 19.7mm/s (still failing both). Reverted live (config back to
all-zero) rather than keep a mixed/net-unclear result — one trial point
is not a tuning sweep.

Recommended next step: a real Stage B gain SWEEP (not one point) on the
stand, using `src/tests/bench/wheel_controller_ab_bench.py` as the
harness (it already supports `--skip-stage-b` and a `config()` push
path) — vary kp/ki independently, watch for the 2-3Hz duty-domain limit
cycle this design is meant to avoid (`drive.h`'s own header warns about
it), and re-score against `score_plus500()`'s own rise/ripple/split
criteria until all three clear together, not just one at a time.

### Finding 2 — unexplained plant-gain/saturation drop vs the ab303ee3 baseline

This session's saturation reading (commanded 800mm/s, both wheels held
2.5s, steady-state read from the last 40%): L=571.7, R=532.5mm/s.
`ab303ee3` (2026-07-31, the commit that baked `Drive::kDutyPerSpeed`)
measured L 760-795, R 696mm/s at full duty on the SAME robot — a
saturation reading needs no config constant, so this is a real,
directly-measured ~24-25% drop, not a calibration artifact. The
residual-sweep percentages (cmd 100/150/250/400, fresh-start/short-hold)
are also uniformly lower than ab303ee3's own commanded-vs-measured table
(e.g. cmd150: this session L 83%/R 77% vs ab303ee3's L 97%/R 77% — LEFT
specifically got worse; RIGHT is flat).

NOT root-caused this session — no pack-voltage telemetry exists on this
robot to confirm or rule out a battery-state difference between the two
sessions (per project convention, do not narrate a power/battery cause
without a measurement backing it — this is flagged as an open,
unexplained finding, not attributed to any cause). Needs a fresh
characterization session (repeat `duty_sweep.py`'s saturation+linear-fit
protocol) to determine whether this is a one-session anomaly, genuine
mechanical/battery drift, or something else — and whether
`Drive::kDutyPerSpeed` (currently a baked firmware constant, ab303ee3)
still matches the real plant.

## Priority

Medium — Finding 1 blocks full +500-spec compliance but the controller is
otherwise safe/functional and Stage C works correctly given time; Finding
2 is a measurement anomaly that could affect calibration confidence
generally.
