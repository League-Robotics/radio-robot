---
status: pending
---

# Real-robot motion calibration — systematic ~6-12% undershoot on turns and straights

## Context (bench gate, 2026-07-11, firmware v0.20260711.4 on tovez)

The library-native motion redesign (Ruckig directional velocity bands,
ride-the-tail terminal stops, replan failure discipline — see
`segment-executor-stop-decel-drain-overshoot-reverses.md`'s resolution) was
flashed and verified on the stand:

- VER handshake: `0.20260711.4` ✓
- `cmd=` vs `vel=` are now DISTINCT on the wire (max |cmd−vel| ~125 mm/s
  during ramps) — the TLM cmd= mislabel fix is live; hardware tracking
  error is finally observable ✓
- **Zero commanded reversal** at every turn end (measured floor −0.4 °/s ≈
  noise) — the encoder-wedge reversal write-train trigger is structurally
  gone on hardware ✓

But absolute accuracy on the REAL plant undershoots systematically:

| move | landed | error |
|---|---|---|
| RT +90° | +78.8° | −11.2° (−12%) |
| RT −90° | −77.9° | +12.1° (symmetric) |
| RT +180° | +157.6° | −22.4° (−12%) |
| D 345 | 324.5 mm | −20.5 mm (−6%) |

Proportional (not fixed) error, symmetric in direction — a scale/tracking
deficit, not an endgame defect.

## Why (hypothesis, informed by the sim work)

The sim reached ±0.3°/±1mm only after its plant-specific quantities were
MEASURED as a set: exact feed-forward (kff = 1/plateau), honest velocity
filter, ceiling ≤ plant capability, and the effective dead time re-measured
for those gains (`kOutputHops`). The real robot has never had that pass:

- boot `kff = 0.001` vs a measured plateau of ~600-650 mm/s (1/650 ≈
  0.00154) — the feed-forward UNDERDRIVES ~35%, and ki is slow to make up
  the difference within a segment → the plant runs below its setpoint all
  segment → the encoder stops/exhaustion accept the shortfall
  ("residual accuracy is calibration work").
- `kOutputHops = 4` (80 ms) was measured for the OLD gain regime; the sim
  showed this constant is gain-dependent (its calibration moved 2.0 → 1.5
  after the gain fix).
- The divergence-replan extend-on-deficit path should be chasing exactly
  this shortfall — whether it fires on hardware (thresholds vs real noise
  floor) is unverified.

## Suggested method (transcribe the sim's, on the bench)

1. Measure the real per-wheel duty→speed plateau (steps at several duties,
   read `vel=` — cmd= is trustworthy now).
2. Set kff = 1/plateau in the robot JSON / boot config; re-pick kp/ki
   around it; decide the honest velocity-filter alpha.
3. Re-measure effective dead time (encoder-position vs command-integral
   cross-correlation — the notebook/discriminator scripts from 2026-07-11
   do this directly over serial).
4. Re-measure the replan divergence noise floor on hardware; verify the
   thresholds sit above it and below the deficit signal.
5. Acceptance: `wheel_motion_trace.ipynb` in bench mode — turns within a
   few degrees, D within a few mm, still zero reversal; then the recorded
   tour on the playfield.

HITL: wheels on the stand for everything except the final tour.

## Regression log (2026-07-11 bench session, wheels on stand)

Method as suggested above, executed as measure -> set -> flash -> score with
`wheel_motion_trace.ipynb` as the acceptance instrument. Scores are final
heading error per turn (deg):

| config | 90 | 180 | 360 |
|---|---|---|---|
| baseline (kff=0.001, hops=4) | -11.2 | -20.4 | -27.5 |
| iter 1: kff=1/650 | -16.0 | -13.4 | -24.6 |
| iter 2: + hw kOutputHops 4->6 (120ms, re-measured onset) | -10.0 | -3.1 | -5.5 |
| iter 3: + ki 0.0018->0.006 | -18.0 | -2.8 | -32.1 | REJECTED
| **landed = iter 2** (final clean capture) | **-10.6** | **-2.2** | **-9.0** |

Measurements behind the knobs:
- Duty-saturated plateau: readings varied ACROSS MEASUREMENT WINDOWS
  (620-740 mm/s per wheel typical; one window read ~408). Cause NOT
  established -- and per the stakeholder it is NOT a power issue (he
  monitors power directly). Leading candidates: measurement-window design
  (window opening before the hold is steady), velocity-estimation
  artifacts under frame loss, or genuine duty->speed nonlinearity. kff =
  1/650 was a mid-range pick; see iteration 5's conclusion below -- the
  loop must not depend on open-loop FF accuracy at all.
- Motion-onset dead time (command commit `now` vs first encoder movement
  `ts`): 112-136ms -> hw kOutputHops = 6 (120ms). GAIN-DEPENDENT: the old
  80ms fit the old sluggish (kff=0.001) plant's effective lag; after the
  FF fix the 80ms model made maybeReplanPivot() shrink-retarget every
  pivot ~15-25 deg short.
- Sub-plateau duty->speed nonlinearity: tracking sits ~7-8% below the
  plateau-fitted kff line at ~150-300 mm/s -- this is the remaining 90 deg
  residual (~-10 deg, pivots cruise ~250 mm/s); 180/360 cruise near the
  plateau where kff is exact. Raising ki to close it in-move (iter 3)
  destabilized the score instead.

## Open items

1. The ~-10 deg residual on SHORT pivots: needs either a second FF point /
   speed-dependent FF, a faster-but-stable integrator, or acceptance.
2. RUN-TO-RUN VARIANCE is a limiting factor for scoring: repeated trials
   per config are mandatory (single-run scores mislead). The variance's
   cause is unestablished; it is NOT power (stakeholder-verified). The
   robust fix is control-side: a feedback-dominant velocity loop makes the
   scores insensitive to whatever moves the open-loop duty->speed map.
3. Heavy telemetry frame loss during fast motion (known IRQ/serial-RX
   coupling) blinds host-side instruments: the cmd= integral undercounts on
   hardware, and completion flags can vanish -- any bench harness MUST
   treat encoder movement as the ground truth for "did it run" (a lossy
   run's busy flag never arriving led a retry to double-queue a segment:
   robot turned ~684 deg on a 360 ask). The notebook's capture guards
   encode these rules now.

## Iteration 4 (2026-07-11, later): reanchor velocity seed -- trajectory quality

The stakeholder's bench runs exposed that iter-2's decent ENDPOINTS hid
garbage TRAJECTORIES: mid-pivot the commanded velocity cliffed to zero,
the robot stalled ~0.25s, then re-accelerated a second full bell (heading
plateau clearly visible). Mechanism: gross-divergence `reanchor()` seeded
velocity = 0.0f ("no reliable measured angular rate") while the wheels ran
~300 mm/s -- Ruckig, told the robot was at rest, planned from rest. Fixed:
seed with the measured rate (vR - vL)/trackwidth (plan-sampled fallback).
The translate reanchor always did this; the pivot one was left lazy.

Scores after (same instrument, humps = trajectory-quality metric, sim = 1):

| turn | endpoint err | humps |
|---|---|---|
| 90  | **+3.9** (was -10.6) | 3 |
| 180 | **+4.3** (was -2.2)  | 4 |
| 360 | -19.8 (was -9.0)     | 4 |

90/180 now within ~4 deg of target with the bench heading riding ON the sim
curves. Remaining: 360 (ceiling-speed cruise) still ragged/short -- replans
still fire there (peak measured wheel 431 mm/s, above the sim plant model's
400); next measured knob is the hardware replan noise floor at ceiling
speeds. The endpoint-only scoring mistake is corrected: `humps` (count of
distinct acceleration bells; healthy = 1) is now part of the notebook
summary.

Bench-harness rule refined after burst losses sank two runs: a resend is
safe if-and-only-if nothing provably started (no busy flag AND no encoder
movement over a 3s probe); the notebook now retries up to 4 verified-idle
sends instead of skipping the source.

## Iteration 5 verdict (motion_control.ipynb, PID isolation)

The stakeholder-directed waveform experiment (sine/square direct velocity
setpoints through the escape-hatch drive arm -- planner fully out of the
loop) settles the planner-vs-PID question: **the velocity loop itself is
the defect.** Measured, sample-level, at 250 mm/s amplitude:

- Holding a constant setpoint the wheel runs 12-30% off it, barely
  corrected (the loop is feed-forward-dominated; kp=0.0022/ki=0.0018 are
  too weak to regulate the FF error out).
- Velocity steps overshoot ~40-50% (max +/-378 on a +/-250 ask), with
  ~50-70 ms reversal dead time; loop corner sits between 1 and 4 Hz.

Whatever makes the open-loop duty->speed map drift (NOT power --
stakeholder-verified; do not revisit), the control answer is the same:
retune feedback-dominant (kp/ki up until the loop regulates to setpoint,
FF deliberately conservative), acceptance = motion_control.ipynb (flat
tops at the setpoint, overshoot < ~10%), then re-score the turn notebook.

## Iterations 6-8 (2026-07-11 evening): the velocity loop, designed from the measured plant

Following the motion_control.ipynb verdict, the loop was redesigned from a
proper open-loop plant identification (pure-FF build: kp=ki=kaw=0,
kff=0.002, so commanded velocity IS scaled duty; steady holds, fresh-
sample tail means):

**The plant is linear and steady** -- L ~756 mm/s per unit duty (+8
offset), R ~721 (-12), sd 2-4 mm/s across every hold, symmetric in
reverse. (Earlier "varying plateau" readings were measurement-window
artifacts, nothing else.)

Designed gains (tovez.json, kaw newly plumbed through gen_boot_config):
kff = 1/740, kp = 0.0018 (loop gain ~1.3 vs ~120ms loop lag), ki = 0.008
(tau_I 0.22s), kaw = 15 (back-calculation anti-windup -- the +50% step
overshoots were the integrator winding up during the ~100ms reversal
dwell with kaw=0; a first try at kp=0.004 rang at ~4Hz and was backed
off). motion_control.ipynb acceptance: holds within +/-3% (sd 6), step
overshoot 1.3%/-0.6%, monotonic rise.

Final knob: kRotDivergenceThreshold 0.10 -> 0.22 rad -- at the 6 rad/s
yaw ceiling ONE loop pass of sampling jitter is ~0.15 rad of apparent
divergence, so long ceiling-speed cruises (360 pivots) shrink-retargeted
~25-30 deg short while 90/180 (which barely dwell at ceiling) were fine.
A stalled wheel accrues ~0.15 rad EVERY pass, so stall protection still
trips within ~2 passes.

**Final turn scores (bench vs target):**

| turn | session start | final |
|---|---|---|
| 90  | -11.2 | +6.2 |
| 180 | -20.4 | +6.7 |
| 360 | -27.5 | **-2.7** |

Remaining (small): a consistent +5..+7 deg on short/mid pivots (low-duty
map offset region; candidate trims: kff 0.00135 -> ~0.0013, or accept),
and hump counts of 2-3 on bench (vs sim 1) whose metric may partly count
sampling texture -- verify against the speed-panel shape before acting.

## Also discovered (needs its own issue/decision)

Runtime config application is UNWIRED on real firmware: binary SET
(config arm) acks and posts to bb.configIn, but main.cpp deliberately has
no Configurator draining it (093/094 removed runtime config authority;
096 built the binary arm expecting a future consumer). Consequence: the
TestGUI's connect-time calibration pushes ACK BUT DO NOTHING on hardware,
and live gain tuning is impossible (every iteration above required a
reflash). Wiring the Configurator into the real loop would make SET real
and cut bench-tuning cycles from ~5 min to seconds.
