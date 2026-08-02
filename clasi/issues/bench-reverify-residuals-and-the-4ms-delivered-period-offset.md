---
status: pending
priority: high
---

# Bench re-verification residuals from sprint 130, and the +4 ms delivered-period offset

## Description

Sprint 130 ticket 011 re-verified the honesty pass on hardware and found two real
defects before the robot wedged. The defects were fixed or characterized; two of
the ticket's four criteria could not be re-measured because the robot needed a
physical power cycle nobody was present to perform. Closing the ticket with those
residuals here rather than leaving it half-open.

### What ticket 011 DID establish

1. **The heading-hold loop goes unstable at the longer control period.** A full
   bench square tour produced heading **411.8 deg vs 360 commanded (+51.8)** and
   80.6 mm closure, where sim showed 41.7/43.0 mm and clean turns.
   `Planner::applyHeadingHold()` is a P-loop at gain 2.0 rad/s per rad closing on
   **pure-encoder heading** (the OTOS blend is fail-closed everywhere). Isolated
   single-leg A/B on the same build: gain 2.0 gave sustained ~1 Hz wheel reversal
   (11.35 s for a leg that should take ~4 s); gain 0.0 was clean. `wheels()` (the
   raw `App::Drive` path) was unaffected either way, isolating it to the planner's
   profile path, not sprint 130's new controller.
   **Mitigated** by setting `heading_hold_gain: 2.0 -> 0.0` in
   `data/robots/tovez.json`. That is a disable, not a fix.

2. **The `PlannerLimits` 34->18 ABI reshape is clean on hardware.**
   `planner_harness.py`'s `plannerStructSizes` / `plannerLimitsOffsets` asserts
   and all four scenario checks passed against a freshly built library. Silent
   field misalignment was the main risk of ticket 009; it did not happen.

### The +4 ms delivered-period offset is NOT new — correct the record

Ticket 011 measured **54.000 ms +/- 0.006 idle**, 54.09 +/- 0.38 under load, with
`kCycle = 50`, and flagged it as a finding. It is real but it is **long-standing**:
with `kCycle = 40` the delivered period was **44 ms** (visible in TestGUI
telemetry readouts as `loop 21.3ms / 44.0ms`). Same +4 ms, both times.

So sprint 130 ticket 007 did not introduce it — 007 moved the nominal from 40 to
50 and inherited the offset. `cycleBusy` is only 21-23 ms, so this is not
busy-time overrun; it is a deterministic fixed cost, plausibly `markTime()`'s
ms-truncation compounding across the loop's four `runAndWait()` pacing blocks.

**The actionable part is the lie, not the 4 ms.** The robot JSON's timing note
currently claims "the delivered period IS the nominal." It never has been. Either
make the loop deliver the nominal, or make the constant mean what is delivered —
but the note must stop asserting something measurably false, because every gain
tuned against "50 ms" is really tuned against 54.

## Unverified criteria (need a working robot)

- **Bench square-tour closure after the heading-hold fix.** The fix is flashed
  (`v0.20260802.1`); its confirming tour never completed. Re-run
  `src/tests/bench/planner_square_tour.py`.
- **+500 / saturation / bias re-measurement on the 50 ms firmware.** Ticket 006's
  numbers were taken on the 40 ms build:
  endpoint 510/502 mm split 8 mm PASS; transients FAIL (rise 0.59 s vs <=0.3,
  ripple L 24.3 / R 22.5 vs <=10, |vL-vR| 20.9 vs <=10); Stage C bias converged to
  +14.0 mm/s right over a 90 s hold; saturation L 571.7 / R 532.5 (about 25% below
  the historical 760-795, still unexplained). Re-run
  `src/tests/bench/wheel_controller_ab_bench.py`. **The longer period plausibly
  moves rise time and ripple in either direction — that is the interesting
  variable.**
- **Live `pid.kp` tuning round-trip** — code-verified only this session
  (`Configurator::applyMotorConfigPatch` still routes into
  `Drive::setControlGains()`), never re-exercised on the wire.

## Proposed follow-on work

- Re-enable heading hold with a gain that is stable at the delivered period, or
  establish that the loop needs restructuring (it closes on encoder-only heading,
  which is the deeper problem — it cannot distinguish a real heading error from
  encoder drift).
- Reconcile nominal vs delivered period, and fix the note either way.
- The four robot wedges in two days (dead-I2C signature, each needing a physical
  power cycle) are their own reliability problem and gate all bench work — see
  [[tovez-hard-silent-i2c-wedge-blocks-completing-the-population-duty-sweep]].

## Related

- Residual of sprint 130 ticket 011.
- [[plus500-transient-criteria-and-plant-gain-drift-followup]] — the transient
  criteria and the unexplained saturation drop.
