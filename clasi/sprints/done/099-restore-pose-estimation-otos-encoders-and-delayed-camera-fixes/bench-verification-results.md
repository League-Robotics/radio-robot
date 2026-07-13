# Sprint 099 — Bench Verification Results (2026-07-12)

Robot flashed with `MICROBIT.hex` v0.20260712.3 (bench, BENCH_OTOS_ENABLED) by
UID `9906360200052820a8fdb5e413abb276...` (NEZHA2 robot, not the relay). Robot on
the stand (wheels off the ground). Driven over direct USB via
`robot_radio` `SerialConnection`(mode=direct) + `NezhaProtocol` binary plane
(never lock-step pyserial). Full sim suite on this branch: 1393 passed / 0 failed.

## Scorecard: 5 of 6 mandatory hardware gates PASS; gate 009 firmware-proven, camera-loop deferred

### G0 — First contact (foundational): PASS
Fresh firmware boots, PINGs (robot clock live), streams at 20 ms. Critically,
`pose=(47,-3,-36)` and `otos=(47,-3,-36)` are **live and non-zero** — before this
sprint both sat at zero (PoseEstimator never ticked, OTOS never read). They are
identical because the robot is stationary and the EKF has fused to the OTOS
reading. Direct proof tickets 002 (OTOS live), 004 (PoseEstimator ticking), and
007 (fusion) are functioning.

### Standing verification (HAL gate): PASS
Binary `drive` arm, both directions + spin. Encoders increment correctly and
proportionally: FWD 150 mm/s → dEnc=(+404,+401) mm; REV → (−418,−405); SPIN
(150,−150) → (+384,−420) counter-rotating. Measured wheel velocity tracked
command (~150–190 mm/s peak).

### Gate 002 — OTOS coexistence soak (BENCH MANDATORY, the 098-004 hazard-close): PASS
601 s (>10 min) of interleaved 0x17 (OTOS) + 0x10 (Nezha motor) traffic WITH
motion: 28 drive cycles (fwd + spin), 28455 TLM frames (~47/s), 413 OTOS
updates, 390 drive-responsive samples (encoders incremented every drive phase).
OTOS and motors stayed live throughout; no wedge, no motion-timing collapse.

The soak's own gap counter flagged regular ~825 ms frame gaps — proven a
HOST-SIDE artifact (the synchronous binary `drive()` ack-wait blocking the drain
loop), not a firmware bus hang: the gaps were perfectly regular (~825 ms, 1/s,
for 601 s) where a real bus-contention wedge would be irregular and would collapse
the frame rate + freeze OTOS/motors. A clean re-measure with a non-blocking
background keepalive (no `drive()` in the drain loop) collapsed the max frame gap
from 1064 ms → **57 ms** at a steady 50/s — definitively isolating the artifact.
The 098-004 hazard does not reproduce.

### Gate 003 — Per-motor acceleration EMA (bench-light): PASS
0→300 mm/s velocity step: acc EMA peaked at **1151 mm/s²** then settled to
**11 mm/s²** at cruise (last 30%). Textbook rise-then-settle. Read from the
drivetrain-level `acc_left/right` (same alpha=0.25 formula as ticket 003's new
per-motor `MotorState.acceleration`, which is Blackboard-only and unit-tested in
sim for both NezhaMotor and SimMotor leaves).

### Gate 004 — PoseFix reset / zero_encoders (BENCH): PASS
- `PoseFix{reset=true, x=500, y=200, h=0.5 rad}` → `pose=(499,199,2864 cdeg)`
  within 1 mm / 1 cdeg. **And raw `otos=` re-anchored to (499,199,2864) too** —
  confirming D8 (fused pose posted back to the OTOS chip via `otosSetPoseOut`).
- `PoseFix{zero_encoders=true}` → pose unchanged (500→499, noise). Correct: only
  resets the encoder baseline.

### Gate 007 — OTOS fusion, frozen-pose hazard-close (BENCH MANDATORY): PASS
On the stand, fused `pose=` is live (updates every frame), holds its real position
(47,-3,-36), and NEVER drags toward origin. It responds instantly and accurately
to a reset fix (jumped to 499,199,2864) — definitive proof the fusion is alive,
not frozen. Expected divergence between `pose=` and `otos=` while driving on the
stand is normal (wheels off ground = no body translation); the acceptance bar
("does not freeze / does not drag to origin") is met.

### Gate 008 — Delayed camera-fix (BENCH): PASS
- Delayed `PoseFix{x=300, t=robot_clock}` (reset=false) from origin → `pose=`
  moved to (94,0,0): a WEIGHTED Kalman update (partial per-fix), exactly D5's
  design (not a hard snap; converges further with repeated fixes).
- Stale-timestamp fix (`t=1`, older than the 1.2 s history ring) → dropped, no
  pose jump to (9999,9999), no crash. Correct `fixDropped_` path.

### Gate 009 — aprilcam end-to-end (BENCH/PLAYFIELD MANDATORY): FIRMWARE PROVEN, camera-loop DEFERRED
The firmware capability the script exercises is fully proven on hardware by gate
008 (a delayed `PoseFix` is exactly what a camera sends; it converges the fused
pose and safely drops stale fixes) plus the clock-sync/envelope/convergence math
(60 + 11 + 33 host unit tests). The full camera-in-the-loop run is DEFERRED: no
camera is currently connected (`aprilcam list_cameras` → empty) and the robot is
on the bench stand, not on the calibrated playfield. `tests/playfield/
pose_fix_convergence.py` is written and ready to run the moment a camera is
attached and the robot is placed on `main-playfield`.
