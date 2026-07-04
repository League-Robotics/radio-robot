---
status: pending
---

# Encoder wedge freezes a wheel mid-leg — now THE blocker for bench tour closure

## Problem

With the bench-OTOS stack fixed (measured-motion feed, OZ/SI re-anchor,
slip-unified heading law — merged 2026-07-03), tour geometry on the bench
is now correct **between wedge strikes**: RT 9000 lands at +92.4° ± 0.2
believed (coast only) with fused == encpose, and D legs measure
99.9–100.6% of commanded.  What still wrecks tours is the pre-existing
**encoder wedge**: a wheel's cumulative encoder freezes (mostly wheel=L
in today's runs), which

- **starves the DISTANCE stop** — the leg runs to the TIME backstop and
  ends short/long (`EVT done D reason=time`), e.g. `D 700` ending at
  478 mm right where `EVT enc_wedged wheel=L enc=476` fired;
- **injects phantom rotation** — the frozen wheel reads 0 while the
  other counts, so every pose frame integrates a turn that never
  happened: a "straight" D 700 measured **+130.8° of heading change in
  the RAW ENCODER frame** with the OTOS entirely out of the EKF
  (`SET lag.otos=0`), proving the fault is below the estimator.

One strike translates/rotates every subsequent leg — the "path drives
off the table" picture, even though per-leg geometry is otherwise clean.

## Quantified evidence (2026-07-03 bench sessions, tovez on stand)

- Strike rate: wedge events in essentially every multi-leg run today
  (23 `enc_wedged` in the E1–E3 set; 8 in the two clincher runs; 2+ in
  the user's 17:45 GUI run).  The long D 700 legs are hit most.
- E1 (nocal, OTOS fused): D 700 leg showed **−110.4°** fused heading
  change on a straight leg → closure 686 mm.
- E3 (nocal, `lag.otos=0`, encoder-only): same leg **+130.8°** in the
  encoder frame → 622 mm.  OTOS/EKF exonerated.
- User GUI run (calibrated, fixed fw 0.20260703.25): `enc_wedged
  wheel=L enc=476` mid-D 700 → leg ended at 478/700 mm (68%); 10 TLM
  frames with `wedge=1,0`.
- Clean runs (no mid-leg strike) close at 53 mm / ~30° (coast-limited).

Prior knowledge: encoder-wedge-boundary-latch (2026-07-01) described the
latch at D-decel/stop as self-healing at the next D's atomic reset.
Today's data shows it also strikes MID-LEG and corrupts the leg itself.

## Escalation (2026-07-03 evening)

Latch rate CLIMBED over the evening's continuous bench running: 2-4
episodes/tour (afternoon) -> 4 -> 6 -> 9 episodes/tour (18:28-18:30
runs), both wheels (L-dominant), closures degrading 53 mm -> 578/434/
2401 mm despite estimator-side defenses.  The robot had been driving
tours near-continuously for ~4 h — thermal state of the Nezha
controller is the prime suspect for the rate escalation.  At 9
latches/tour the encoder stream is unusable at the source; KB guidance
for the persistent flavor is a Nezha POWER-CYCLE (physical switch).

Estimator-side defenses landed on master tonight (phantom-rotation
holds + release-jump clamps + capped hold window; commits 53da040,
1c6afe6-era).  Two recovery designs were tried and REVERTED with
evidence: mid-leg atomic-read re-prime (it is itself a documented latch
trigger — runs got worse) and full healthy-wheel substitution into
_hw.encPos (correct long-term design, but it changes encoder-stream
semantics for 15 tests' worth of consumers — belongs in this sprint,
with command-sign-aware substitution: frozen wheel mirrors
healthy * sign(tgtFrozen*tgtHealthy) so both straights AND spins
survive; wire it into stop conditions via the existing enc0/encDiff0
per-wheel recovery).

## Reliable reproducer (2026-07-03 evening)

`tests/bench/wedge_latch_repro.py` — back-to-back short D legs (250 mm/s,
150 mm; maximum decel boundaries per minute), firmware `EVT enc_wedged` as
ground truth.  Measured: **38 episodes in 60 legs (0.63/leg)**, one every
~6 s of runtime.  Signature: latch value = the leg's decel landing point
(enc=144/145 of a 150 mm leg) in nearly every episode → boundary latch at
D-deceleration/stop, exactly the KB hypothesis (write-throttle bypass at
stop/reversal interleaved with 0x46 reads).  Dominantly wheel=L on this
unit; occasional wheel=R and mid-leg values.  A/B protocol for any
candidate fix: run 60 legs before/after, compare episodes/leg.

`tests/bench/wedge_latch_matrix.py` — DBG WEDGE stimulus-matrix runner
(time-to-latch per config); WedgeTest's instant-verdict quirk needs review
before its numbers are trusted (see script docstring).

## PIVOTAL: motor-swap experiment (2026-07-03 20:05)

Eric physically replaced both motors and re-ran the 60-leg reproducer:
**0.63 episodes/leg -> 0.05/leg (12x reduction; 1 EVT latch + 2 blind
boundary classifications).**  The latch susceptibility is dominantly in
the MOTOR UNIT itself — the removed left motor (was port M2, 34 latches
in 60 legs) is a pathologically latch-prone article and explains the
evening's rate escalation and most tour wreckage.

Consequences:
- KEEP the bad motor, labeled — a 0.63/leg reproducer unit is the
  perfect test article for the fix sprint.
- The reproducer doubles as an incoming-inspection tool: 60 legs per
  motor, episodes/leg as the accept metric.
- The residual 0.05/leg on fresh motors has the same decel-boundary
  signature (enc=146) — the firmware-side trigger work remains valid
  but drops in priority; the bench-mode substitution defense covers the
  residual rate.

## Direction

The wedge detector already fires (`EVT enc_wedged`, TLM `wedge=` flags,
EKF omega gating).  Candidates for the fix sprint:

1. Recovery in place: on wedge detection, re-prime the encoder read
   (rebaselineSoft / atomic re-read) DURING the leg, not only at the
   next drive start.
2. Stop-condition resilience: while a wheel is wedged, drive the
   DISTANCE stop from the healthy wheel (single-wheel distance ×2 for
   straights) instead of the starved sum.
3. Pose resilience: gate the heading integration on the wedge flag
   (hold dTheta) so a frozen wheel cannot inject phantom rotation —
   the EKF omega gate already does this for the velocity channel.
4. Root cause below all of that: why does the Nezha 0x46 read latch
   (I2C traffic pattern at decel? register read-settle?) — see
   docs/knowledge/2026-07-01-encoder-wedge-boundary-latch-flavor.md.

## Impact

Dominant remaining bench failure.  Until fixed, expect 1–3 corrupted
legs per tour and closures degrading from ~50 mm to 200–700 mm at
random.

## WEDGELAB campaign (2026-07-04, overnight): ROOT CAUSE ISOLATED, FIX PROVEN ON BENCH

Standalone lab (`wedgelab/`, self-contained CODAL project) with 4 motors:
M1/M2 = old latch-prone pair, M3/M4 = fresh. Dual driver: raw from-scratch
wire functions vs VERBATIM production Motor/I2CBus copies.

1. **Reproducer**: `run reset` — cruise, production `Motor::resetEncoder()`
   mid-motion, immediate reversal (a D-preemption mimic). Old motors latch
   on essentially EVERY hot +->- flip; fresh motors never (immune to the
   whole trigger battery all night).
2. **Chip-confirmed**: at each latch, two raw-path 0x46 reads 60 ms apart
   (independent of the production driver) return identical values while
   the wheel spins, all transactions ACK, zero bus errors ever (XCHECK
   CHIP, exp10). Not driver error-masking. The register truly latches.
3. **The trigger is the REVERSAL WRITE TRAIN, not the read burst**:
   burst-only arm clean (0/20); reversal-only arm (rebaselineSoft = zero
   burst I2C) latches 5/5 of +->- flips when hot. Direction-asymmetric
   (only +->- on these motors; rhymes with the L-dominant field data).
   Production `setSpeed`'s reversal exemption writes the flip immediately,
   slew-stepped +-25 through zero across consecutive 10 ms ticks
   interleaved with encoder reads — that sequence is the latch inducer.
   Production decel sign-dither at every stop boundary = a train of
   micro-reversals through this same path (the boundary-latch signature).
4. **Fixes proven (exp12-14)**, hot controls bracketing every run:
   - **Zero-dwell reversal**: command 0 and hold >=50 ms before the new
     direction. 0 latches / ~75 hot susceptible flips (soak n=150).
     20 ms is NOT enough (12/12 latched); threshold in (20, 50] ms.
   - **Gentle reversal ramp** (<=5 PWM-pct per 10 ms through zero):
     0/25 hot cycles.
   - Stop-verified-then-reset (mode 2): clean; good D-boundary discipline.
5. **Production fix recommendation**: in `Motor::setSpeed`, replace the
   immediate reversal exemption with a two-phase reversal — write 0, hold
   >=50 ms (100 ms conservative), then apply the new direction (or ramp
   at <=5/tick). Apply to ANY sign change; keep stop (pct==0) immediate.
   Also keep 064-003 rebaselineSoft (hard resets only at verified rest).

Data: `wedgelab/out/*exp09..14*`. Lab usage: `wedgelab/README.md`.
