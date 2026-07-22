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
