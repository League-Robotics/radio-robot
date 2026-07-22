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
