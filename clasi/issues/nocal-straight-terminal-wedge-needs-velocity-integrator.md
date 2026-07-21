---
status: pending
---

# nocal straight-leg terminal wedge is a pure-P velocity droop — needs a velocity integrator (vel_ki)

## Summary

The bench terminal wedge that stops TOUR_1 leg 1 (a 345 mm straight) on the
**tovez_nocal** profile is a **pure-P velocity-loop droop**, not a bus/brick
fault. nocal ships `vel_ki=0` (pure P, "droops by design" per its own config
note). At the leg's terminal ramp-down the P command falls below the wheel
deadband, the wheel stalls ~15 mm short, and with no integrator to push
through terminal stiction it stays stalled → the stalled encoder latches
(the "wedge") → Move times out → fault.

## Evidence (2026-07-20 overnight bench, tovez on the stand)

- **nocal** (`vel_ki=0`): leg 1 (345 mm) faults at ~10.4 s, reproducibly,
  independent of cycle order. Encoder freezes ~15 mm short; fault_bits bit 1
  (wedge) latches after the stall. Trace: `tour_tour_1_20260721T05*.csv`.
- **calibrated tovez.json** (`vel_ki=0.005`, `vel_kaw=20`): the SAME straight
  now **COMPLETES** (TOUR_1 leg 1 in ~3.2 s clean). This confirms the
  integrator is the fix. Built with `ROBOT_CONFIG=data/robots/tovez.json`.
- Sim does not reproduce (ideal plant has no terminal stiction).

## Caveat — not 100% yet

Even with the integrator the straight is **intermittent**: TOUR_1 leg 1
completed (3.2 s) but a later 345 mm straight faulted (~15 s, wedge-like). So
the integrator greatly reduces but does not fully eliminate the terminal
stall — the deadband/write-shaping (`output_deadband`, `reversal_dwell_ms`,
114-005's sub-deadband duty boost) may need co-tuning with the integrator.

## Options

1. Give nocal a small `vel_ki` (keeps it "uncalibrated" but non-droopy), OR
2. Bench with the calibrated **tovez.json** profile (already tuned), OR
3. Deadband-compensation work (sprint 114 territory) so even a pure-P loop
   lands the terminal.

Relates to the terminal-blip / motor-deadband work in sprints 111/112/114.
