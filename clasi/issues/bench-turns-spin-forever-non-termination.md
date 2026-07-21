---
status: pending
sprint: '119'
---

# Bench: 90° turns spin forever (turn non-termination) — the hard blocker to a completing tour

## Summary

On the bench (real encoders, OTOS untrusted → encoder heading), a `RT 90`
(90° pivot) **never terminates**: the wheels hold full pivot speed
(velL≈−540, velR≈+590 mm/s) indefinitely, the encoder heading runs to
**~4600–4900 cdeg×10 = ~13 full rotations**, and the Move only ends when its
terminal ack times out (~9–13 s) → `outcome=fault`. This is **not** a wedge
(`fault_bits` stays 1 = benign; no wedge bit). The turn's velocity profile is
never ramped down — the executor holds constant omega instead of decelerating
to the target heading.

This is the **single hard blocker** to completing a full tour on the bench:
with the calibrated profile the straight legs complete, then leg 2 (the first
turn) spins forever.

## Evidence (2026-07-20 overnight bench session, tovez on the stand)

- Reproduced with BOTH cycle orders (committed "A" and the e7fb9be2 "C"
  restore) + calibrated profile → cycle order is NOT the cause.
- Trace `src/tests/bench/out/tour_tour_1_20260721T06*.csv`, leg_index==1:
  `sent_omega=0` (Move-based, firmware executes), `vel_l≈−540`/`vel_r≈+590`
  constant, `enc_r` climbs monotonically, `pose_h_cdeg` climbs linearly to
  ~460000–490000 cdeg, `done` flips True only at the timeout with `err0`.
- Sim does NOT reproduce this — sim turns complete (with a turn-accuracy
  error, see the cycle-order A/B issue). So it is specific to the **hardware
  encoder-heading turn-termination path**.

## Where to look

- `src/firm/motion/executor.cpp` / `jerk_trajectory.cpp` — the pivot's
  RAMP_TO_REST / dwell-completion gate: why does omega never ramp down?
- `src/firm/app/pilot.cpp` + `heading_source.cpp` — the encoder-heading
  feedback the turn completion depends on. Is the turn-complete condition
  (heading dwell within tolerance at low rate) ever evaluated against the
  encoder heading on the bench path? Is the encoder heading sign/scale right
  for a pivot (velL, velR opposite)?
- Compare the sim turn (completes) vs bench turn (spins) tick-by-tick.

## Note

Never seen a turn complete on the bench before this session (nocal wedged on
the first straight, before ever reaching a turn — see the integrator issue).
So this may be a long-standing latent bug only now exposed once the straights
started completing.
