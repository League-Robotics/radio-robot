---
date: 2026-07-02
tags: [d-drive, distance-stop, decel-profile, stop-condition, fabsf, minWheelMms,
       integrator-freeze, sync-coupling, stiction, sim-plant, forced-stall,
       otos, ekf-rejection, telemetry, recordings]
related-tickets: []
---

# D-drive terminal instability: lands short, reverses, thrashes, completes on a lunge

**Status:** design flaws confirmed in code and reproduced against the real
firmware (sim build) with a forced-stall harness; exact hardware-side
term-by-term arithmetic of the sign flip not pinned (needs on-robot
instrumentation or a stiction-capable sim plant). Issues filed:
`clasi/issues/d-drive-terminal-instability-reversal-thrash.md`,
`clasi/issues/distance-stop-fabsf-accepts-backward-completion.md`,
`clasi/issues/otos-not-used-frozen-pose-ekf-rejects-everything.md`.

---

## Problem

`D 200 200 500` (and 420/700-mm variants) end their moves violently: the
robot decelerates cleanly toward the target, stalls just short, drives
*backwards*, oscillates with growing amplitude, spins tens of degrees, then
lunges forward and reports `EVT done D reason=dist` — with final per-wheel
distances grossly wrong (e.g. 493/590 mm for a 500 mm straight drive).
Six D drives recorded on 2026-07-02: **five failed this way, one completed
cleanly.** Identical on the stand and on the playfield, so it is a pure
firmware control defect, load-independent.

## Symptoms (TLM signature to recognize it)

- Clean trapezoid (ramp, cruise ~205 mm/s, decel matching
  `v = sqrt(2·aDecel·d_rem)` with aDecel=250 exactly).
- Mean encoder lands 1–3 mm **short** of the target at ~15 mm/s, mode stays
  `D`.
- 0.3–0.5 s stall, then smooth symmetric negative wheel velocity ramp
  (−5 → −150 mm/s over 1–3 s); encoders count DOWN 40–110 mm.
- Asymmetric thrash follows (encpose heading swings tens of degrees), then a
  forward lunge at 300–600 mm/s; the DISTANCE stop fires mid-lunge when the
  *mean* encoder crosses the target; coast adds 20–90 mm.
- TLM `vel=0,0` in mode `I` while encoders still advance is a display
  artifact (MotorController zeroes `velMms[]` when targets are zero), not
  evidence the wheels stopped instantly.

## What Was Tried (and what each attempt taught)

1. **"Robot on a stand / OTOS frozen" theory — wrong.** The distance loop is
   encoder-only; the stakeholder corrected this and the failure then
   reproduced on the playfield. Lesson: check which sensors actually feed
   the loop in question before blaming a frozen one.
2. **Hand-derived integrator story (frozen-negative I) — wrong in detail.**
   A Python replica of the full control stack (BVC trapezoid + decel hook +
   sync coupling + VelocityController with back-calculation) showed the
   integrator ends *positive* at the end of decel with stock gains.
3. **Ideal sim repro — structurally impossible.** The `PhysicsWorld` plant is
   zero-lag/zero-stiction (`vel = pwm/100 × 400` algebraically), so it can
   never land short; stock and imperfect-knob (`SIMSET motorOffset*/
   encNoise*`) runs complete cleanly at ~44 mm/s crossing speed.
4. **Forced-stall harness — reproduced.** Pinning the *reported* encoders
   just short of the target each tick (emulating stiction) drove the real
   firmware code into: forward windup (+16 PWM, correct) → sign flip →
   committed **−100 PWM full reverse** → drove >1 m backwards → completed
   `reason=dist` at |−500| ≥ 500.
5. **Config bisection over the forced stall** (8 variants): reversal requires
   `vel.kI>0` AND `vel.kP>0` AND the `minWheelMms` integrator-freeze
   deadband; `minWheelMms=0` → **no reversal at all**; `sync=0` → unchanged
   (cross-wheel coupling **exonerated** — the repro is perfectly symmetric).

## What Worked (root causes, confirmed in code)

1. **Asymptotic decel vs strict-crossing stop.** `Planner::driveAdvance`'s
   DISTANCE hook caps speed at `sqrt(2·aDecel·d_remaining)` — zero speed AT
   the target — while `StopCondition::DISTANCE` needs `traveled >= target`.
   Real motors stop 1–3 mm short. Marginal by design; 1-in-6 success was
   luck.
2. **Down-only ratchet.** The hook only lowers the BVC target, so once
   near-target the setpoint pins at ~15–20 mm/s forever and the drive can
   never re-approach after a retreat; mode D has no terminal path but the
   TIME net.
3. **`minWheelMms` integrator freeze in exactly that regime.** The pinned
   setpoint sits at/below the 20 mm/s deadband where
   `VelocityController::update` freezes I — no guaranteed windup to break
   stiction forward, and the frozen value persists in the PWM sum.
4. **`fabsf(traveled)` in the DISTANCE stop** (`StopCondition.cpp:101-104`)
   accepts a *backward* runaway as completion — the unbounded end-state; a
   playfield robot could back off the table at full speed and report
   success. The decel hook has the same `fabsf` on `d_traveled`.
5. **Mean-of-wheels stop × encoder latch.** Recording 163248 drive #15: right
   encoder wedged at 651 (known Nezha D-decel/stop latch), so the mean-based
   stop drove the healthy left wheel to 785 for a 700 mm target.

## Why It Works (the failure, end to end)

Stall short → tiny ratcheted setpoint below the integrator-freeze deadband →
no robust forward completion; measurement noise / gearbox rollback /encoder
latch perturbs the loop → PID (kP+kI, with the frozen-I bias) crosses into
reverse → retreat *increases* `d_remaining` but the ratchet can't re-raise
the target, so nothing restores forward progress → oscillation grows until a
forward lunge happens to carry mean-enc across the threshold → `fabsf` stop
accepts the crossing (and would equally have accepted −500 mm).

## Debugging techniques worth reusing

- **Recordings are ground truth.** `recordings/*.jsonl` console captures
  parse with `robot_radio.robot.protocol.parse_tlm` after stripping the
  TestGUI `[HH:MM:SS] < ` prefix (regex `^\[[^\]]*\]\s*[<>]\s*` — it is NOT
  a relay prefix, `_strip_relay` won't remove it).
- **Forced-stall sim technique**: cap `sim_set_reported_enc_l/r` **before
  each `sim_tick`** at a value below the stop threshold. Caveat: the plant
  integrates `v·dt` on top of the pin within the tick, so the firmware sees
  `cap + v·dt` — a leak that pollutes per-tick micro-traces (it cost this
  session a false "filter dynamics" lead). Good enough for black-box
  behavior; not for term-by-term PID forensics.
- **White-box getters** exist in `sim_api.cpp`: `sim_get_pwm_l/r`,
  `sim_get_vel_l/r` (firmware's measured vel), `sim_get_enc_l/r` (the
  filtered `_hw.encMm` the controller actually sees), wedge flags, reset
  counters. Set `argtypes=[c_void_p]` on any getter not pre-declared in
  `firmware.py` or ctypes segfaults.
- **Config bisection over a forced fault** (SET one knob per fresh Sim) is
  fast and decisive for "which term is load-bearing".
- Physical plausibility checks on encoder deltas cut both ways: loaded max
  is ~370 mm/s but free-spinning (stand) wheels exceed it — don't call
  >400 mm/s "impossible" without knowing the load state.

## Side finding: OTOS not used

All day, TLM `otos=` was frozen within each session (identical pose across
every frame, including 500 mm of real playfield travel) while `ekf_rej`
climbed every EKF tick — the EKF rejected everything and ran encoder-only.
Also, bench mode was not simulating OTOS as designed. Separate issue filed;
not the cause of the thrash, but it silently removes the sensor that would
have flagged the resulting pose error.

## Future Guidance

- Any profiled-approach-to-threshold design needs either a **terminal speed
  floor** that punches through the threshold, an **arrive tolerance**, or a
  position servo for the last millimeters — never an asymptotic profile
  against a strict-crossing stop.
- Distance/rotation stops should be **direction-aware**; blanket `fabsf`
  turns runaway into "success".
- The sim plant needs **lag + stiction/breakaway** before terminal-behavior
  bugs of this class are testable (sprint 069 charter; this incident is the
  concrete justification).
- When a drive "completes" with wheels at speed, suspect the stop fired
  mid-transient, not at a controlled stop — check per-wheel distances and
  heading against the command before trusting the move.
