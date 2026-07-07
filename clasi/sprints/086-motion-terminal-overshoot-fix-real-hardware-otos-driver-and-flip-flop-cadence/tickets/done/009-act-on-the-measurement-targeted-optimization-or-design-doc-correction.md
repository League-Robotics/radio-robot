---
id: 009
title: 'Act on the measurement: targeted optimization or design-doc correction'
status: done
use-cases:
- SUC-009
depends-on:
- 008
github-issue: ''
issue: flip-flop-cadence-below-design-target.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Act on the measurement: targeted optimization or design-doc correction

## Description

Final ticket in the flip-flop cadence set. Depends on ticket 008's
measurement. Closes the parent issue
(`flip-flop-cadence-below-design-target.md`).

**Two possible outcomes, decided FROM ticket 008's data — both are
acceptable closes for this ticket and this issue:**

1. **Targeted optimization**: if ticket 008's measurement shows genuine
   double-counted or otherwise reclaimable clearance time, apply the
   narrowest fix that reclaims it (e.g., de-double-counting the
   `preClear`/`postClear` pair identified in ticket 008), then re-measure to
   confirm improved Hz.
2. **Doc correction**: if no safe win exists, update the design document's
   cadence estimate (the 079 tick-model sketch this issue references) to
   state the measured ~44-52 Hz as the real budget, with the reasoning from
   ticket 008. **This is an ACCEPTED, successful outcome — not a sign this
   ticket under-delivered.**

**NON-GOAL (hard constraint, applies to outcome 1 only, but stated here
regardless of which outcome is chosen): do NOT regress the 079-006
TWIM-stall fix or the reversal-latch armor to chase Hz.** If any candidate
optimization risks either, do not take it — fall back to outcome 2 instead.
This is not a tradeoff to negotiate; a doc-only correction is strictly
preferable to a Hz gain that reopens the 079-006 stall risk or weakens the
armor.

## Acceptance Criteria

- [x] The chosen outcome (optimize vs. document) is explicitly recorded in
      this ticket's completion notes, with ticket 008's supporting
      measurement cited. **(Outcome 2: DOC CORRECTION.)**
- [ ] **If optimizing**: the fix is narrowly scoped to what the measurement
      justified (e.g., only the identified double-counted clearance, not a
      broader rewrite); the change is re-measured (repeat ticket 008's
      method) to confirm an actual Hz improvement; the full 079-006
      TWIM-stall fix's own tests/soak pass unmodified; the motor-armor
      tests (ticket 002's Invariant A/B tests plus the full
      `test_motor_policy.py` suite) still pass unmodified. **(N/A — not the
      chosen outcome; 008 refuted the double-counting hypothesis, so there is
      no reclaimable time and any clearance reduction would reopen 079-006.)**
- [x] **If documenting**: the design doc's cadence estimate section
      (the 079 tick-model sketch) is updated to state the measured ~44-52 Hz
      as the real budget, cross-referenced to this issue and ticket 008's
      measurement, with no firmware code changed.
- [x] Either way, no change to `I2CBus`'s public per-device clearance
      semantics that any OTHER caller (the OTOS leaf from ticket 006, any
      future device) relies on, without confirming those callers still work
      correctly. **(Zero code changed — doc-only.)**

## Completion Notes (team-lead, 2026-07-06)

**Chosen outcome: 2 — DOC CORRECTION** (an accepted, successful close per this
ticket's own framing). 008's stand measurement REFUTED the double-counted-
clearance hypothesis with data: the `writeMotorRun`(`0x60`, `postClear=4000`)
→ next `requestEncoder`(`0x46`, `preClear=4000`) gap measured **~4.97 ms
across five duty-write cycles (4958–4980 µs), not ~8 ms** — `I2CBus`'s
`max(readyAt, lastEnd+preClear)` collapses the paired clearances on the same
device to a single 4 ms window. There is therefore NO reclaimable/double-
counted time. The ~52 Hz cadence is two genuinely-separate, non-overlapping
~4 ms clearance windows per port-visit (request-settle + inter-transaction
gap), each of which is the irreducible 079-006 TWIM-stall safety margin.
Reducing either is ticket 009's explicit hard NON-GOAL (it reopens the
multi-second CODAL `waitForStop()` stalls). Optimization outcome 1 is thus
correctly rejected on the data.

**Doc change (no firmware code touched):** annotated the 079 tick-model
design sketch's cadence estimate —
`clasi/sprints/done/079-.../issues/done/tick-model-command-flow-and-the-command-board-design-sketch.md`
— in two places:
1. The **Cadence table** (2-port row `~11–13 ms (~80–90 Hz)`): added a
   CORRECTION block stating the real measured budget is **~19–21 ms
   (~48–52 Hz)**, explaining that the estimate predated the 079-006 clearance
   pair (each port-visit pays two ~4 ms windows, ≈ 9.5–10.5 ms, not the
   assumed 5.3–6.5 ms), that the two clearances are NOT double-counted, and
   that the embedded PID closes the loop cleanly at ~52 Hz (accepted budget,
   not a defect). Cross-referenced to this issue and 086-008.
2. The **Case 2 walkthrough** (`per-motor period ≈ 12 ms (~83 Hz)`): added a
   pointer correction showing `reqP1` actually recurs at ~t=19 ms (~52 Hz),
   not t=12, because the collect/duty→next-request transition also pays a
   full ~4 ms clearance.

**No `I2CBus` public clearance semantics changed** — doc-only, so the OTOS
leaf (006) and every existing caller are unaffected. No code tests apply;
the doc change accurately reflects 008's measured data.

## Implementation Plan

**Approach**: Read ticket 008's recorded measurement and its
double-counting-hypothesis verdict first. If confirmed and a safe, narrow
fix is apparent (e.g., an ordering/accounting fix in how `NezhaMotor::
requestEncoder()`'s `preClear` and `writeMotorRun()`'s `postClear` combine
for a single in-use port), implement it and re-run ticket 008's measurement
method to confirm the improvement. If not confirmed, or if the only
available fixes carry stall/armor risk, write the doc correction instead —
this is equally valid completion of this ticket.

**Files to create/modify** (outcome-dependent):
- Optimization outcome: `source/com/i2c_bus.{h,cpp}` and/or
  `source/hal/nezha/nezha_motor.cpp` (`requestEncoder()`/`writeMotorRun()`'s
  clearance arguments) — narrowly scoped to the measured issue.
- Doc-only outcome: the 079 tick-model design document (locate via
  `clasi/sprints/done/079-.../` per the issue's own citation) — update the
  cadence-estimate section.

**Testing plan**:
- If optimizing: re-run ticket 008's measurement method to confirm improved
  Hz; re-run the full 079-006 TWIM-stall regression suite/soak; re-run the
  full motor-armor test suite (ticket 002's additions plus the pre-existing
  `test_motor_policy.py` cases) unmodified.
- If documenting: no code tests apply; confirm the doc change accurately
  reflects ticket 008's data.

**Documentation updates**: The 079 cadence-estimate section is updated
either way — either to state the achieved improved Hz (optimization
outcome) or the corrected real-budget estimate (doc-only outcome). Close
the parent issue file.
