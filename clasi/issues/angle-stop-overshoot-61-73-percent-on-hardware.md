---
status: pending
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
