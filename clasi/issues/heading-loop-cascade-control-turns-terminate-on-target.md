---
status: pending
sprint: 098
---

# Heading-loop cascade control — turns terminate exactly on target

## Goal (stakeholder acceptance, 2026-07-11)

Command an in-place turn of any angle at any speed and have the robot
**terminate exactly on the requested heading** — no speed-dependent overshoot,
no run-to-run scatter. Acceptance instrument already exists:
`tests/bench/turn_sweep.py --relay --both` over the playfield; a passing run
lands every cell within a small tolerance of target (goal ≈ ±1°), with the
90°-ridge and the fast-turn overshoot gone.

## What the dataset proved (see `turn_sweep_analysis.ipynb`, 58 playfield turns)

The overshoot is **not** momentum coast and **not** geometry:

- **Coast after command-zero is ~1°** — the wheels stop when told to. The
  ω²/2α braking-distance story does not fit this plant (equal entry rate does
  NOT give equal overshoot: full-ceiling 90° overshoots +8..+10° while 180°/360°
  at the same-or-higher entry rate overshoot half as much).
- The error is built **while moving**: a ~−22° commanded/measured *tracking
  deficit* during acceleration versus a ~+24..+30° *surplus* during
  deceleration. The landed error is the small difference of two ~25° open-loop
  transients — which is also why the SAME turn scatters σ≈2° run-to-run.
- On some short fast turns the **commanded integral itself exceeds the ask**
  (94° commanded on a 90° ask): the executor's threshold-triggered divergence
  replan re-anchors to the lagging measured position and re-commands the accel
  deficit, *forgiving* the deficit that would otherwise cancel the decel
  surplus → the 90° ridge.
- After the encoder stop fires, the executor **rides the Ruckig plan's
  remaining tail open-loop**, delivering whatever the plant is ahead by at fire
  time on top of the target.

**Root cause, one sentence: nothing in the firmware regulates heading.** The
wheel PID loops regulate wheel *speed* (and do it well — see the velocity-loop
redesign, `real-robot-motion-calibration-undershoot.md`); the Ruckig heading
profile is then played essentially open-loop, with two crude patches where a
heading feedback loop should be (the bang-bang divergence replans and the
ride-the-tail terminal). A fixed empirical aim-short table cannot fix this: the
best 2-parameter fit (`overshoot ≈ −2.0 + 1.5·ω`) still leaves the 90° ridge at
+4..+6° and cannot touch the σ≈2° variance.

## The fix — cascade control (stakeholder-provided architecture)

Add the missing **outer heading loop**. Keep the excellent inner wheel-velocity
PID loops exactly as they are.

```
Turn command
      ↓
Ruckig plans the HEADING trajectory   (θ_desired, ω_desired, α_desired each tick)
      ↓
Heading controller (NEW, outer loop):
   ω_cmd = ω_desired + Kp·(θ_desired − θ_measured) + Kd·(ω_desired − ω_measured)
      ↓
Convert to wheel targets:  vL = −ω_cmd·track/2,  vR = +ω_cmd·track/2
      ↓
Wheel velocity PID loops   (UNCHANGED — they already track well)
      ↓
Motors
```

Design points:

1. **Ruckig already plans heading.** The rotational channel solves θ in radians
   to rest on target; `SegmentExecutor::tick()` already samples desired
   position/velocity/acceleration every pass. The pieces are in hand — the
   change is to *use* the sampled desired-heading and close a loop on it instead
   of emitting the plan velocity open-loop.
2. **Continuously correct against measured heading** rather than assume the plan
   is followed. This nulls the tracking asymmetry AND the run-to-run variance at
   the source (a servoed endpoint has no open-transient-difference to scatter).
3. **Retire the divergence-replan machinery to stall-protection only.** The
   outer loop is the continuous corrector now; keep only the gross-divergence
   path as a stalled-wheel safety (a stalled wheel still trips it within ~2
   passes). Remove the ride-the-tail open-loop terminal accounting.
4. **Completion = tolerance, not plan-exhaustion.** Declare the turn done only
   when `|heading_error| < tol AND |angular_rate| < rate_tol`, held for a short
   dwell (≈100–200 ms). Suggested start: 0.5° and ~1°/s. This lets the loop
   remove residual error instead of accepting whatever the plan exhausted at.
   NOTE: a heading PD nulling residual error implies tiny terminal reversals —
   exactly what the `Hal::Motor` reversal-dwell/deadband armor exists to make
   safe, but given the encoder-wedge history this MUST be verified on the stand,
   not assumed. (See `.clasi/knowledge/` wedge notes; `[[motor-armor-policy-lives-in-base]]`.)
5. **Empirical compensation demoted to calibration.** Any small residual bias
   after the loop is a labeled tunable in `data/robots/tovez.json`
   (`control.turn_*` — clearly marked "calibration, expected ≈0, per-robot"),
   NOT primary control. Plumb through `gen_boot_config.py` like the vel_* gains.
6. **New PD gains (Kp, Kd) are labeled per-robot tunables** in
   `tovez.json`/`PlannerConfig`, documented like the vel gains. Loop-separation
   is comfortable: inner loops corner ~1–4 Hz, so an outer Kp on the order of a
   few /s sits a decade below.

## Heading source — encoder first, OTOS as staged upgrade

**Current state (verified 2026-07-11):** the live `main.cpp` loop does NOT tick
OTOS or run any PoseEstimator — `NezhaHardware::tick()` only pumps the motor/
encoder flip-flop, and `bb.otos` is never committed. 093/094 stripped the loop
to a bare wheel-driving executive. The `Hal::OtosOdometer` leaf still exists and
`begin()`s, but nothing reads it live.

Therefore, stage the heading source:

- **Stage 1 — encoder heading (primary, certain):** heading =
  `(encR − encL)/trackwidth`, already every tick, already the dataset's ground
  truth. The whole cascade can close on this with ZERO new sensor plumbing. The
  dataset shows the residual is tracking asymmetry, not gross slip, so an
  encoder-heading loop is *sufficient* to make turns terminate on target. This
  stage alone satisfies the acceptance criterion and must land first,
  independently valuable.
- **Stage 2 — OTOS heading (slip-immunity upgrade, stakeholder-blessed):**
  revive OTOS ticking in the live loop, commit heading to the blackboard, and
  let the SegmentExecutor consume OTOS heading when valid with **encoder
  fallback**. Heading (unlike OTOS *position*, which has the off-center lever-arm
  problem — `[[otos-offset-register-unwritable]]`) is mount-offset-independent,
  so it is the safe OTOS quantity to trust. CAUTION: OTOS is a separate I2C
  device NOT in the flip-flop sequencer; ticking it every pass adds an I2C
  transaction per loop — weigh against the actuation-latency/​flip-flop coupling
  issue (`[[motor-actuation-latency-flipflop-coupling]]`) and the
  radio-needs-yield rule. Stage 2 must not regress Stage 1's turn accuracy or
  the loop-yield/​radio timing.

The stop conditions today evaluate with an empty `PoseEstimate` (encoder-only) —
there is already a plumbing seam to upgrade the heading authority.

## Live tuning — wire the Configurator (de-risks the tuning loop)

Runtime config application is currently UNWIRED on real firmware: binary SET
acks into `bb.configIn` but nothing drains it, so gain changes require a ~5-min
reflash each. Wiring a minimal Configurator that drains `bb.configIn` and
applies the new heading/vel gains live would cut hardware tuning from minutes to
seconds. This is optional-but-valuable; if included it must respect 093/094's
"boot config applied once at construction" model — a *runtime* apply is
additive, not a return to the old full config authority. (See
`real-robot-motion-calibration-undershoot.md` "Also discovered".)

## Sim vs hardware

The sim plant has essentially no tracking asymmetry (it reached ±0.3° already),
so in sim the heading loop must be a **no-op-to-improvement**: the sim
regression suite (currently 615 green) must stay green, and sim turn accuracy
must not regress. Tune Kp/Kd in sim first (instant rebuild), flash once,
validate on the playfield over the radio, re-tune live if the Configurator is
wired.

## Acceptance

1. Sim regression suite green (no regression from 615).
2. New unit coverage: heading-loop math, tolerance-based completion, replan
   retired-to-stall-protection, OTOS-with-encoder-fallback source selection.
3. `turn_sweep.py --relay --both` on the playfield: every cell lands within
   goal tolerance (≈±1°), 90° ridge gone, run-to-run scatter collapsed.
4. Still ZERO commanded terminal reversal at the wheel level beyond what the
   motor armor absorbs (verify on the stand — no wedge).

## Related

- `real-robot-motion-calibration-undershoot.md` (the velocity-loop redesign that
  got the inner loops right — this issue adds the outer loop on top).
- `restore-pose-estimation-otos-encoders-delayed-camera-fixes.md` (sprint 099;
  full pose fusion. This issue only needs OTOS *heading*, not full fusion —
  keep the two scoped apart; Stage 2 here is a minimal heading tap, not the
  099 fusion restoration).
- `motor-actuation-latency-flipflop-coupling.md` (the I2C/actuation-lag
  constraint any OTOS-per-pass read must respect).
