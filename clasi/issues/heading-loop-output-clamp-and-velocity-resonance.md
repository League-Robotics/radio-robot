---
status: pending
---

> **SCOPE REDUCED (2026-07-14 stakeholder triage).** Part 1 (clamp the
> heading-loop output in Motion::SegmentExecutor) is OBSOLETE — the executor
> and the on-robot heading loop are deleted by the single-loop rebuild
> (`clasi/issues/single-loop-firmware-de-fiber-delete-the-elite-plumbing-telemetry-only-return-path.md`).
> Part 2 SURVIVES and matters MORE: the ~140 mm/s inner velocity-PID
> resonance lives in the kept Devices::MotorVelocityPid, and the rebuilt
> robot is a pure velocity follower — the inner loop becomes the only loop.
> Characterize and tame it (filter/feedforward/notch) against the new
> firmware with the on-stand step harness.

# Trajectory ringing: clamp the heading-loop output + tame the ~140 mm/s velocity resonance

## Context (found 2026-07-12 on the stand, after sprint 098)

The turn ENDPOINTS land on target (sprint 098), but the wheel-speed
TRAJECTORY rings during acceleration/cruise — the measured wheel speed
overshoots the command ~40% and oscillates before settling (stakeholder
noticed it in a `wheel_motion_trace` plot). Endpoints hid it (it damps out
before the stop). Diagnosed on the stand via live gain tuning
(`SET pid.kp`/`headingKp`, robot on the stand, wheels free):

**Two coupled causes:**

1. **The inner velocity PID has an overshoot RESONANCE at ~140 mm/s.** A clean
   velocity STEP (drive-arm, planner out) overshoots +24..+33% at 140 mm/s,
   but only +4% at 70 and +10% at 250 — a mid-speed peak, not monotonic. This
   is intrinsic to the loop at `vel_kp=0.0018` (the pre-existing tuning,
   validated only at 250 mm/s in `motion_control.ipynb`, which missed it).

2. **The outer heading loop drives the wheels straight into that band.** At
   `heading_kp=6` the heading-loop output (`omega_desired + kp*heading_error`)
   is NOT clamped, so during the acceleration lag it commands the wheels to
   ~1.4–1.7× the yaw ceiling (e.g. 240 on a 140 turn) — right into the
   resonance — then the plant overshoots and the loop pulls back → the ring.

## Interim (shipped 2026-07-12, stakeholder-accepted)

`vel_kp 0.0018→0.0014`, `vel_ki 0.008→0.005`, `vel_kaw 15→20` (in
`data/robots/tovez.json`): halves the step overshoot (+24%→+11%) and reduces
the heading over-drive (a softer inner loop lags the plan less abruptly, so
the heading error and its correction shrink: over-drive 1.58×→1.24×). Cost: a
small endpoint regression (full grid mean 0.40°, 95% within ±1°, worst cells
−30@384 −1.75° and 180@70 −1.05°, vs the pristine 0.0018 loop's 100%/0.59°).
The stakeholder chose this trade for the smoother trajectory over pursuing
the code fix immediately.

## The clean fix (both endpoint AND trajectory) — deferred

Pure gain-tuning can't give both (softening kp for damping loosens the
terminal correction). The structural fix, which keeps the pristine
`vel_kp=0.0018` endpoint (100%/0.59°) AND kills the ring:

1. **Clamp the heading-loop output to the yaw ceiling** in
   `Motion::SegmentExecutor` (the `omega = omega_desired + kp*(…) + kd*(…)`
   line for PRE_PIVOT/TERMINAL_PIVOT, sprint 098-002). Clamp `|omega|` to
   `rotationalCeiling_` (+ a small margin). Near the target the heading error
   is tiny so the clamp never engages → endpoint unaffected; during accel it
   caps the over-drive → wheel peak drops ~255→~174 on a 180@140 turn,
   removing the big spike. Low-risk, ~10 lines.
2. **Tame the ~140 mm/s velocity resonance itself** (the residual +24% after
   the clamp): candidates are the velocity filter (`vel_filt`, currently 0.3,
   NOT live-settable — a reflash knob; more smoothing damps but adds lag), an
   acceleration feedforward term, or a notch. Characterize with the on-stand
   velocity-step harness (`tests/bench`, drive-arm step at 70/140/250) —
   acceptance is <~10% step overshoot across the speed range with the rise
   time preserved.

Acceptance: `wheel_motion_trace`-style trajectory clean (no visible ring),
`turn_sweep.py --relay --both` endpoints back to ~100% within ±1°.

Related: `heading-loop-cascade-control-turns-terminate-on-target.md` (parent,
sprint 098), `real-robot-motion-calibration-undershoot.md` (the inner-loop
tuning history). Live tuning is available on the stand
(`SET pid.kp/ki/kff/iMax/kaw`, `SET headingKp/headingKd`).
