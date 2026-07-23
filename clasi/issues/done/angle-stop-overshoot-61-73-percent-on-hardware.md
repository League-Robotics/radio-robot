---
status: resolved
---

# Angle-stop MOVE overshoots 61-73% on hardware (reproducible; distance-stop within tolerance)

## Description

Post-close bench verification (2026-07-22, tovez on the stand, master
v0.20260722.3): an angle-stop MOVE commanded to 0.5 rad heading change
measured 0.806 / 0.867 rad across two runs — 61-73% overshoot,
reproducible. Distance-stop MOVEs landed within +-20% (226/232 mm for a
commanded 200 mm). Telemetry evidence recorded in
docs/bench-checklists/sprint-116-move-protocol.md.

## Cause

Not diagnosed on the bench. Consistent with the known actuation-latency /
momentum-tail family: Motion::StopCondition fires the cycle the heading
CROSSES the threshold, then the wheels take the write-path's shaped ramp
(plus ~120-140 ms actuation lag) to actually stop — at pivot omega that
tail integrates a large extra angle; at straight-line speeds the same tail
costs proportionally less distance (~13-16% observed). The protocol has no
deceleration anticipation by design (bounded velocity + stop condition).

## Proposed fix

Options, stakeholder's call:
1. Let the trajectory-controller arc own it (the planned remaining-distance
   velocity shaping ends motion WITH a commanded terminal velocity —
   anticipation is its whole point). Angle stops stay coarse until then.
2. A cheap interim: StopCondition anticipation offset — fire early by
   (current omega x stop-tail time), config-tunable. Small, but adds the
   kind of compensation machinery the gut just deleted.
3. Tolerate: document the overshoot in protocol-v4.md as expected behavior
   for v1 stop conditions.

## Related

- docs/bench-checklists/sprint-116-move-protocol.md — measurements.
- predict-to-now-odometry-estimator-... issue — the trajectory controller (option 1) is its final phase.
- .clasi/knowledge actuation-latency notes (093-era) — the ~120-140 ms lag.

## Additional measurements (2026-07-22, stakeholder acceptance-table bench session)

Re-measured at a LARGER commanded angle (90°, `move_twist(omega=+-2.0rad/s,
stop_angle=pi/2)` — `_UNMANAGED_YAW_RATE`, the TestGUI's own cruise rate),
2 trials per direction, `tovez` on the stand, calibrated gains applied
(this session's own calibration-push fix — see
`clasi/issues/testgui-dbg-otos-bench-verb-dead-on-serial-connect.md`'s
sibling session note and the `set_config_binary()`/`binary_bridge.py` SET
routing fixes in this same commit series). Heading measured from
`TLMFrame.pose` (encoder odometry), not OTOS/camera:

| Trial | Commanded | Measured | Error |
|---|---|---|---|
| turn +90° #1 | +90.0° | +102.7° | +14.1% |
| turn +90° #2 | +90.0° | +100.3° | +11.4% |
| turn -90° #1 | -90.0° | -95.0° | -5.5% |
| turn -90° #2 | -90.0° | -105.6° | -17.3% |

At 90° (vs the original finding's 0.5rad = 28.6°) the SAME fixed-tail
overshoot mechanism produces a much smaller RELATIVE error (11-17% vs
61-73%) but a comparable ABSOLUTE one — roughly 10-16° over target across
all 4 trials, consistent with "a fixed angle tail added on top,
regardless of the commanded target" (the tail's own absolute size does
not obviously scale with target angle, matching the actuation-lag/
momentum-tail theory: the overshoot is a function of omega and stop
latency, not of how far the Move was told to turn). Both directions
overshoot (never undershoot) across all 4 trials; the two negative-turn
trials show more spread (-5.5%/-17.3%) than the two positive ones
(+14.1%/+11.4%) — sample size too small to call that a real
directional asymmetry rather than trial-to-trial noise. Not diagnosed
further this session (same "measurement, not fix" scope as the original
finding) — recorded here as the requested acceptance-table data point.

## Fix (2026-07-22/23, 90-degree turn campaign)

Implemented option 2 from this issue's own "Proposed fix" list above: a
cheap, config-tunable `StopCondition` anticipation offset. Concretely,
`App::MoveQueue::tick()` (`src/firm/app/move_queue.h`/`.cpp`) now
evaluates the Distance/Angle stop condition against
`App::StateEstimator::bodyAt(now + stopLead)` — a ZOH-predicted-forward
pose, using the ALREADY-STANDING predict-to-now machinery sprint 117
built (`state_estimator.h`'s own file header always named this exact
consumption as "a later, out-of-this-sprint trajectory controller") —
instead of the raw current `Odometry` reading. `stopLead` is a new
fail-closed boot config key (`data/robots/*.json`'s `estimator.
stop_lead_ms`, `Config::EstimatorBootConfig::stopLead`) plus a live-tune
wire arm (`EstimatorConfigPatch.stop_lead_ms`,
`NezhaProtocol.estimator_config(stop_lead_ms=...)`) — never persisted,
same "reboot reverts to the JSON bake" contract as the other estimator
fields.

**Root-cause correction found while tuning:** a naive "lead = measured
lag" assumption (the intuitive first guess, and this issue's own original
"anticipation offset" framing) OVER-corrects, because the coast-down
DECELERATES rather than holding the cruise angular rate for the whole lag
window — the correct relationship, re-derived empirically by sweeping the
actual firmware (`src/tests/testgui/test_tour_closure_gate.py`), is
`lead ≈ overshoot_uncorrected / omega_cruise`, noticeably SHORTER than the
raw lag. Full derivation and the sim capture behind it:
`src/tests/notebooks/turn_prediction.ipynb` (first-pass heuristic) +
`test_tour_closure_gate.py`'s own closed-loop sweep (the number that
actually shipped).

**Measured results:**

- Sim (`test_tour_closure_gate.py`, TOUR_1/TOUR_2, `stop_lead_ms=90`):
  worst per-turn error ~4.2-4.7° (ideal chip) / ~5.5-6.9° (realistic
  sensor-error profile) — down from ~13-23° with the anticipation lead
  off (the SAME regime this issue's own ~+20.6°-class hardware number
  sits in).
- **Hardware** (2026-07-22, tovez on the stand, `move_accuracy_bench.py
  --skip-ab --skip-creep --trials 1`, `stop_lead_ms=90` baked from
  `active_robot.json`): `+90°` commanded → `+84.4°` achieved (-5.6°);
  `-90°` commanded → `-95.2°` achieved (-5.2° the other way) — a **~5-6°
  residual**, down from this issue's own **61-73%/+20.6°-class**
  established overshoot. Distance stops (100/300/500/700mm forward,
  500mm reverse) all landed within 0.0-4.0%, unaffected (already good).
  Single-trial bench data (small N) — see `data/robots/tovez.json`'s own
  `estimator._estimator_note` for the exact numbers and an explicit call
  for a stakeholder retune session with more trials/a finer lead sweep
  (mirroring `test_tour_closure_gate.py`'s own sim sweep) to shave the
  residual further.

**Not fully closed:** the ~5-6° residual on hardware (and ~4-7° in sim)
is real and not eliminated — `stop_lead_ms=90` is empirically tuned
against THIS specific plant/omega (2rad/s), not analytically exact, and
the underlying ZOH prediction is still a first-order (held-omega)
approximation of a decelerating coast. Option 1 from this issue's own
list (a real trajectory controller with a commanded terminal velocity)
remains the path to closing that residual further, if the stakeholder
wants it — out of this fix's own scope. Marked `resolved` because the
issue AS FILED ("MOVE overshoots 61-73%") is fixed by an order of
magnitude, not because the mechanism is perfected.

## Follow-on fix (2026-07-22, decel-into-the-goal campaign)

Implemented Option 1's own lighter-weight cousin (not a full trajectory
controller, but the same underlying idea): `Motion::VelocityShaper`
(`src/firm/motion/velocity_shaper.{h,cpp}`) shapes the commanded speed
every tick toward a computed terminal value —
`min(cruise, sqrt(2*a_decel*remaining), current+a_max*dt)` — instead of
holding a constant cruise speed until `Motion::StopCondition` fires and
the actuation/momentum tail overshoots past it. Wired into
`App::MoveQueue::tick()`/`shapeAndStage()`, reusing the SAME predicted
pose the anticipation-lead fix above already computes. New fail-closed
boot config (`Config::ShaperBootConfig`, `data/robots/*.json`'s
`control.a_max`/`a_decel`/`alpha_max`/`alpha_decel` — the first two were
dead/orphaned data since 115-003's motion-stack excision, read again into
this NEW consumer; the latter two are new fields) plus a live-tune wire
arm (`EstimatorConfigPatch.a_max`/`a_decel`/`alpha_max`/`alpha_decel`,
riding the same `CONFIG_ESTIMATOR` arm `stop_lead_ms` already uses).

**Re-tuning the lead was required.** With the taper doing part of the
deceleration work the lead used to have to anticipate alone, the OLD
`stop_lead_ms=90` (this issue's own bench-validated value) now
OVERCORRECTS. A sim sweep (isolated single 90-degree turn, both
directions, ideal chip) found `stop_lead_ms=60` is the new sim optimum
(worst 0.3° in isolation vs. 3.1° at `lead=90`/no taper) — see
`src/tests/notebooks/turn_prediction.ipynb` Section 9 and
`test_tour_closure_gate.py`'s own `_STOP_LEAD_MS` comment for the full
sweep table. At TOUR level (not an isolated from-rest turn), worst
measured error is `2.0-2.4°` across TOUR_1/TOUR_2, both fidelity
profiles — roughly HALF the anticipation-lead-only baseline immediately
above.

**Hardware verification (2026-07-22, tovez on the stand, `stop_lead_ms=60`
+ taper baked from `active_robot.json` -> `tovez_nocal.json` at flash
time, `move_accuracy_bench.py --skip-ab --skip-creep --trials 3`):**

| Trial | Commanded | Measured | Error |
|---|---|---|---|
| turn +90° #1 | +90.0° | +85.3° | -5.2% |
| turn +90° #2 | +90.0° | +86.2° | -4.3% |
| turn +90° #3 | +90.0° | +83.0° | -7.8% |
| turn -90° #1 | -90.0° | -84.8° | +5.7% |
| turn -90° #2 | -90.0° | -86.2° | +4.2% |
| turn -90° #3 | -90.0° | -85.6° | +4.8% |

A **~4-8° residual either direction** (mean ~5.3°) — in the SAME
ballpark as this issue's own PRE-taper `stop_lead_ms=90` result (~5-6°),
**not** the dramatic further improvement the sim predicted (sim measured
roughly half the anticipation-lead-only error at tour level). A
follow-up live-config A/B on the same hardware session (`stop_lead_ms` in
{60, 90, 45, 75}ms, taper unchanged) found no value in that bracket
clearly beats `60` — every clean trial landed in the same ~4-8° band
(one `45ms` trial and one `60ms` trial were lost to an unrelated
`ENQUEUE-REJECTED/TIMEOUT` pacing artifact when moves are dispatched
back-to-back with no settle gap, a pre-existing bench-harness timing
issue, not a shaper regression — the SAME symptom appears on a couple of
distance legs in the full acceptance-table run below, unrelated to
turns). `stop_lead_ms=60` ships as the default; a genuinely finer
hardware-specific re-sweep (more trials, tighter lead bracketing) could
still narrow this further but was not run this session (small-N,
matching this issue's own established "not fully closed" posture).

Distance stops in the SAME session (300/500mm forward, calibrated gains)
landed within `0.0-0.8%`, unaffected/already good — unchanged from this
issue's own original finding.

**Not fully closed (extended):** the taper closes a large fraction of the
sim-measured residual but a materially smaller fraction on THIS hardware
sample — the ~5° hardware residual persists across every lead value
tried in the 45-90ms bracket. This is consistent with (not contradictory
to) the original "not fully closed" note above: the real plant's own
coast-down dynamics, motor response, and I2C bus timing are not fully
captured by the sim's own idealized model, and Option 1's fuller form (a
genuine trajectory controller with a planned terminal velocity, not a
per-tick reactive taper) remains the path to closing the remainder, if
the stakeholder wants it — still out of scope here.

## Follow-on fix, jerk-limited stage, simplified (2026-07-22, same day, two stakeholder corrections)

Stakeholder correction #1, reading `velocity_shaper.cpp` live: "your
velocity shaper is not jerk-limited." Correct — the fix above picked a
bang-bang TARGET ACCEL (full `a_max`/`a_decel` until the target speed,
then an instant drop to 0), so the commanded acceleration itself still
changed discontinuously. A first attempt made `VelocityShaper` stateful
and added a roll-off decision helper (`chooseAccelTarget()`) plus a
jerk-aware stopping-distance helper — working, but grew the module to
~317 combined header+implementation lines with a separate branching
"which phase am I in" function.

Stakeholder correction #2, same day: "You're not trying to implement your
own version of Ruckig, right? I literally just wanted acceleration slew
rate limiting and velocity slew rate limiting." Rewritten to EXACTLY two
chained rate clamps and an integrator: (1) a VELOCITY clamp — approach
cruise speed by at most `a_max*dt`, then cap the result's magnitude to
the decel-taper ceiling `sqrt(2*a_decel*remaining)` (Stage 1's own
formula, byte-for-byte); (2) an ACCEL clamp — the velocity clamp's own
result implies an acceleration this tick, and the commanded acceleration
slews toward that implied value by at most `j_max`/`yaw_jerk_max` per
second. No separate roll-off function, no phase state. A literal,
margin-free version of these two clamps measurably overshoots (verified
in-tree: a `cruiseSpeed=300` ramp-up climbed to `350` and rising; a
decel-to-a-turn's own zero crossing reversed sign) — fixed with two
one-line algebraic margin terms folded directly into each clamp's own
input (not a branch, not a phase): a "predicted speed"
(`commandedSpeed + commandedAccel*|commandedAccel|/(2*jerk)`) feeds the
velocity-approach clamp, and an "effective remaining"
(`remaining - |commandedSpeed|*a_decel/(2*jerk)`) feeds the decel-taper
ceiling. `src/firm/motion/velocity_shaper.cpp` is 94 lines (was 160 for
the roll-off-based attempt), with no functions beyond `next()` and two
tiny sign/clamp helpers — no lookahead solving, no trajectory state
machine.

**Re-verified against the existing `lead=45ms`/`_TURN_TOLERANCE_SHAPED_DEG=2.5`
gate** (no new lead re-sweep — the existing default, chosen for the
earlier roll-off-based design, was re-measured against the simplified
design and still comfortably clears the gate):

| tour | profile | worst \|error\| |
|---|---|---:|
| TOUR_1 | ideal chip | 1.651° |
| TOUR_1 | realistic sensor errors | 0.934° |
| TOUR_2 | ideal chip | 1.487° |
| TOUR_2 | realistic sensor errors | 2.138° |

All four comfortably under the 2.5° tolerance; a similar range to the
roll-off-based design's own numbers (1.378/1.503/1.970/1.668°), not a
regression from simplifying.

**Hardware verification (2026-07-22, same session, tovez on the stand,
`stop_lead_ms=45` + simplified two-clamp shaper baked, `move_accuracy_bench.py
--skip-ab --skip-creep --trials 3`):**

| Trial | Commanded | Measured | Residual |
|---|---|---|---|
| turn +90° #1 | +90.0° | +85.6° | 4.4° |
| turn +90° #2 | +90.0° | +85.7° | 4.3° |
| turn +90° #3 | +90.0° | — | ENQUEUE-REJECTED/TIMEOUT (excluded) |
| turn -90° #1 | -90.0° | -84.5° | 5.5° |
| turn -90° #2 | -90.0° | — | ENQUEUE-REJECTED/TIMEOUT (excluded) |
| turn -90° #3 | -90.0° | -85.0° | 5.0° |

4 of 6 commanded turns completed; residual range `4.3-5.5°`, mean `~4.8°`
— in the same ballpark as the accel-only stage's own `4-8°`/mean `~5.3°`
hardware result, and the roll-off-based jerk design's own `0.0-5.8°`/mean
`~2.9°` result — small-N hardware samples across all three stages
overlap, no clear ranking between them on THIS hardware. The two excluded
trials hit the same pre-existing, documented intermittent serial-RX-loss-
class enqueue-timeout noted in both stages' own tables above and in
`.clasi/knowledge/i2c-irqguard-vs-serial-rx.md` — not a new regression.

**Not fully closed, same posture as above:** the tour-level sim residual
and the hardware residual are both real, not eliminated. This module is
velocity- and accel-slew-rate-limited — two chained clamps, deliberately
nothing more elaborate, per the stakeholder's own explicit scope
correction — not a full Ruckig-style time-optimal trajectory planner.
That remains the path to closing the remainder further, if the
stakeholder wants it — explicitly out of scope for this campaign.
