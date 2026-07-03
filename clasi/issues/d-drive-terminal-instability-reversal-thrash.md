---
status: pending
---

# D distance-drive terminal instability — lands short of the `>=` stop, stalls, reverses, thrashes, completes on a lunge

## Description

Root-caused 2026-07-02 from four TestGUI recordings (six D drives that day:
**5 failed, 1 clean** — stand and playfield identically). Full diagnostic
narrative and repro scripts referenced at the bottom.

### Failure signature (from TLM)

1. Clean trapezoid: ramp, cruise ~205 mm/s, decel — profile itself is fine.
2. Decel lands **1–3 mm short** of the distance target at near-zero speed
   (observed: 499.5/500, 499/500, 699/700, 698/700).
3. Robot sits stalled 0.3–0.5 s, then both wheels ramp **backward** smoothly
   (−5 → −150 mm/s over 1–3 s), retreating 40–110 mm.
4. Retreat degenerates into asymmetric thrash (wheels fight; robot yawed ~40°
   in the 20:10 playfield run), then a violent forward lunge.
5. The DISTANCE stop finally fires **mid-lunge** when mean-enc crosses the
   target at 300–600 mm/s; the post-stop coast adds 20–90 mm.
6. Net result for `D 200 200 500`: final wheel distances 493/590 mm, heading
   off ~40°. The move "completed" (EVT done D reason=dist) but is grossly
   wrong.

### Root cause chain

- **Asymptotic decel vs strict crossing stop.** The D decel hook
  (`Planner.cpp` `driveAdvance`, DISTANCE branch) caps commanded speed at
  `v_cap = sqrt(2·aDecel·d_remaining)` — speed reaches zero exactly AT the
  target — while `StopCondition::DISTANCE` requires `traveled >= target`
  (strict crossing). Real motors with stiction stop just short; the one
  clean run out of six was luck.
- **Down-only ratchet.** The hook only lowers the BVC target
  (`if (v_cap < targetV)`), so once the robot is at/near the target the
  setpoint is pinned at ~15–20 mm/s forever; the drive cannot re-approach
  after any retreat and mode D has no other terminal path except the TIME
  net.
- **Integrator freeze at exactly the wrong regime.** The pinned setpoint
  sits at/below `minWheelMms` (20 mm/s), where `VelocityController::update`
  freezes the integrator — so there is no guaranteed windup path to break
  stiction forward, and whatever value the integrator froze at persists in
  the PWM sum. Config bisection in the sim (real firmware code, forced-stall
  harness) shows the reversal requires `vel.kI > 0` AND `vel.kP > 0` AND the
  `minWheelMms` freeze; with `minWheelMms=0` there is **no reversal at all**.
  The cross-wheel sync coupling is **exonerated** (symmetric repro; `sync=0`
  unchanged).
- **Mean-of-wheels stop × encoder latch.** In recording
  `recording_20260702_163248.jsonl` drive #15, the right encoder wedged solid
  at 651 (known Nezha D-decel/stop latch) and the mean-based DISTANCE stop
  then drove the healthy left wheel to **785 for a 700 mm target** before the
  mean crossed. Reproduced in sim (one-wheel-latch scenario).

### Recommended fixes (roughly in order)

1. Make the DISTANCE stop **signed/direction-aware** — tracked separately as
   a safety issue (`distance-stop-fabsf-accepts-backward-completion.md`).
2. Terminal completion guarantee: floor the terminal `v_cap` at
   ~`minWheelMms` so the profile punches through the threshold, and/or an
   arrive tolerance ("|remaining| ≤ tol and stalled → done").
3. Let the ratchet re-approach after retreat, or treat "stalled short" as a
   terminal condition, so mode D cannot hang in the degenerate regime.
4. Testability: the sim plant (`PhysicsWorld`) is zero-lag / zero-stiction
   (`vel = pwm/100 × 400` algebraically), so it **cannot land short** and
   this failure class is structurally untestable today. Adding motor
   lag + stiction/breakaway knobs is exactly sprint 069's "complete hardware
   fit" charter — this issue is the concrete field failure justifying it.

### Evidence / repro

- Recordings: `recordings/recording_20260702_163248.jsonl` (4 drives, incl.
  the clean one and the encoder-latch case), `recording_20260702_194235.jsonl`,
  `recording_20260702_194340.jsonl`, `recordings/latest.jsonl` (playfield).
- Forced-stall sim harness (caps reported encoders just short of target each
  tick, before `sim_tick`): reproduces windup → reversal → full-reverse
  runaway against the real firmware code. Note the harness cap leaks
  `v·dt` per tick — pin BEFORE the tick and mind the artifact when reading
  per-tick traces.
- Analysis notebook: `tests/bench/wheel_velocity_from_recording.ipynb`
  (velocity-vs-time / velocity-vs-distance per wheel from a recording).
