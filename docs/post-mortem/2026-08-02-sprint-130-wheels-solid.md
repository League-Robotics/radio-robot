# Post-mortem: sprint 130, "wheels-solid"

**Closed** 2026-08-02, `v0.20260802.2`. Twelve tickets, all done, five residuals
filed. Roughly 30 hours wall-clock across two days.

## Outcome

| | before | after |
|---|---|---|
| 150 mm/s commanded | 52 mm/s actual (35%) | lands 500 mm ±10, wheels 8 mm apart |
| sim circle closure | 58.2 mm | 9.6 mm |
| sim square tour | FAIL, ~361 mm | PASS, 41.7 / 43.0 mm (80 mm gate) |
| worst per-turn error | −20.8° | 2.72° |
| wheel controllers in tree | 3 generations coexisting | 1, in `App::Drive`, serving every writer |
| `PlannerLimits` | 34 fields | 18, grouped, ABI verified on hardware |
| sim suite | 458 passed / 2 failed | 460 passed / 0 failed |

The sprint achieved its goal. This document is about how it went, because how it
went is more informative than what it shipped.

---

## The central finding

**Twelve tickets. Twelve latent defects found.** Not one ticket was a clean
execution of its written scope. Every single one uncovered something already
broken:

| ticket | stated work | what it actually found |
|---|---|---|
| 001 | population duty sweep | the sweep tool inverted its duty axis off a config field the firmware had stopped using |
| 002 | unify composition roots | the sim booted ~10% different geometry from the robot it simulated |
| 003 | extend Drive interface | (clean — the one exception, and it was pure plumbing) |
| 004 | controller algorithm | a commanded-zero wheel could creep from an adapted bias |
| 005 | move controller into Drive | two controllers on one signal; two regressions from tickets 002/004 |
| 006 | bench acceptance | `estop()` resets bias by design, which reads as "adaptation is dead" |
| 007 | delete parked duty stage | two bench scripts had been reading a constant zero since sprint 128 |
| 008 | tick() state machine | a dead `Settling` sub-state and a defer path gated on an always-false flag |
| 009 | PlannerLimits 34→23 | five fields the sprint's own plan wanted to KEEP had zero readers (landed at 18) |
| 010 | fix turn undershoot | not a shaping constant at all — a completion predicate that only worked in isolation |
| 011 | bench re-verify | a heading loop that cannot survive a control-period change |
| close | run the gate | two tests that had never compiled |

A 1-in-12 clean rate is not bad luck. It is a measurement of the codebase.

---

## Root causes

### 1. Checking a proxy instead of the thing itself

This is the single recurring mechanism, and nearly every defect above is an
instance of it.

- A Move was "done" when a **short lookahead** said so, rather than when the plant
  had actually stopped. Fine in isolation (the wheel coasts to rest, nobody
  notices); catastrophic when chained, because the next Move overrides the
  still-turning wheels. That is the entire ~10°/turn undershoot.
- The control period "was" 50 ms because a **constant said so**, not because it
  was measured. It is 54, and was 44 when the constant said 40.
- Two tests were "known failing" because a **summary line said so**, not because
  anyone read the error. They had never compiled — a missing `-I src`.
- `wedgeSuspect()` reported a wheel fault from the **ungated latch**, which fires
  on healthy moves too.

The fix is cultural, not mechanical: **make the assertion about the observable.**
Where that is expensive, say so at the assertion site.

### 2. Circular calibration

Two constants were each fitted against the other's error:

`duty_per_speed` (0.00187, claiming 534 mm/s per unit duty) and `wheel_gain`
(≈1.47, encoding "the plant over-delivers") were measured against each other. The
plant actually delivers ~845 and **under**-delivers, so the correction pointed the
wrong way and *compounded* the error. Combined: a 150 mm/s request became 52.

Worse, `tovez_nocal.json` carried rotation constants (1.006 / +12.1°) that had
been **fitted to a sim artifact** and were injecting ~12.5° of under-rotation into
every real turn.

Both are the same failure: a correction layer added on top of an uncharacterized
plant, then a second layer added on top of the first. Each layer made the next
measurement invalid.

**Rule going in: never fit a correction against a constant that is itself
unverified. Anchor on a measurement that depends on no config value.** For this
plant that is the saturation reading — command a speed high enough that duty
clamps, read the result. It needs nothing and takes six seconds. It would have
caught the bad duty sweep instantly, and did, once used.

### 3. No firmware→host config read-back

`config.proto` has no `ConfigSnapshot` arm. **There is no way to ask the robot
what constants it is actually running.**

This turned a routine measurement into a wrong one: a duty sweep computed its
x-axis from `tovez.json` while the firmware had switched to a baked constant, a
~1.6× error, undetectable from the host. Both my own first sweep and ticket 001's
hit this independently.

Every calibration workflow in this project is exposed to it. This is the single
highest-leverage missing capability.

### 4. Accumulated generations

Three wheel-control implementations were live or parked simultaneously
(`WheelVelocityPid`, `WheelPid`/`stageDuty()`, `WheelTrim`), plus the open-loop map
in `Drive`. Only one reached the wheels. Sprint 128 deleted one and parked another;
this sprint deleted the remaining two and consolidated into `App::Drive`.

The cost was not just clutter. `WheelTrim` was summed onto `cmdVelocity` in the
planner while `Drive` closed its own loop — **two controllers on one signal**,
which is why the teleop path and the planner path behaved differently and why
"the unmanaged button has a path planner" was a real complaint.

Parking code is a decision to pay later. This sprint paid.

### 5. Sim and bench diverged silently

The composition roots had drifted: the sim booted its own literal planner limits,
its own track width (raw 128 vs effective 140.4), and wheel-trim gains that
silently defaulted to their fail-closed zero while hardware booted them live.
Unifying them moved tour closure from 45.4 mm to 2.2 mm in an earlier attempt —
the divergence was not cosmetic.

Even after unification, **the heading-hold instability appeared only on hardware**
and no sim test catches it. A gain tuned at 40 ms went unstable at 54 ms. The sim
is now much closer to the robot, but "passes in sim" still does not mean "works."

---

## Errors of analysis

These were mine (team-lead), and they cost more time than any code defect. Each is
listed with its mechanism, because the mechanism generalizes.

**Reported a duty sweep taken against stale firmware.** Produced a confident,
detailed, wrong result: a claimed 28% L/R gain mismatch and a 0.24 breakaway,
leading to a recommendation to inspect the right wheel mechanically. The truth was
1.9% and ~0.10. *Mechanism:* trusted a computed axis over a constant-free one.
*Correction:* anchor on saturation first, always.

**Told the stakeholder the ESTOP fix was missing, twice, and advised cutting
power.** It had been merged as `964b6c90` (sprint 129) with a better
implementation than the one I had written and abandoned. *Mechanism:* carried a
stale belief about my own abandoned work without checking master. *Correction:*
verify against the tree, not against memory of what I did.

**Filed "OTOS frozen at a constant" at high priority.** The robot was on the stand
for every observation; it is an optical ground-tracking sensor, the chassis never
moved, and a constant reading was correct. The "metres of travel" was free-spinning
wheels. *Mechanism:* compared two screenshots without accounting for test regime.
*Correction:* stand facts and playfield facts are different kinds of fact.

**Claimed "the encoders cannot see the curve."** They could. A 1 mm encoder split
was the *symptom* of a terminal pivot, not evidence of a straight line.
*Mechanism:* read a clean-looking number as proof of absence.

**Left a debugger halted and then diagnosed the robot as unresponsive.** `monitor
halt` plus a live gdbserver; a halted core services nothing. I then spent effort
on a UICR boot-loop theory. *Mechanism:* forgot my own intervention was part of
the system under test.

**Sized a plot's canonical x-domain from one tour.** Set 0–15 s from the circle
(12 s); the square runs 33 s, so more than half of every time series was silently
cropped — and since the acceptance band is only drawn where the trace is, the gate
stopped covering the run at 15 s. *Mechanism:* generalized from a single sample.
Fixed with an explicit out-of-domain check, because silent truncation was the real
defect.

**Marked tickets done without moving them.** `update_ticket_status` rewrites
frontmatter; `move_ticket_to_done` moves the file. The state machine requires the
latter, so the sprint could not have closed. Caught by the stakeholder.

The through-line: **six of seven were confident conclusions drawn from
insufficient evidence, then reported as findings.** The code defects were mostly
honest complexity. The analysis errors were mostly premature certainty.

---

## Process observations

**Agent reliability varied more than expected.** Ticket 007's agent completed its
work and stalled without committing — 434k tokens, 224 tool calls, returning a
sentence about waiting on a monitor. The work was found loose in the tree and every
verification step re-run by hand. Ticket 010's ran 106 minutes and 275 tool calls
and reported well. **Verify the tree at each ticket boundary; do not trust a
completion report.**

**Agents are heads-down by construction.** They fixed what was in front of them
and, to their credit, repeatedly refused to paper over things — ticket 005 found
regressions from earlier tickets and declined to patch them blind; ticket 011
refused to mark itself done with 2 of 4 criteria unverified; ticket 006 reverted a
Stage B trial that improved one metric and worsened another. That judgment was
good. What none of them could do is notice that *twelve for twelve* is a pattern.
That is the orchestrator's job and it took the stakeholder pointing at it.

**Bench availability gated everything.** The robot wedged four times in two days
(dead-I2C signature, core spinning in `codal::system_timer_wait_cycles` inside
`Preamble::probeSlot`), each needing a physical power cycle. Two tickets were
partly or wholly deferred because of it.

**Two robots on one hub is a live hazard.** `vizev` belongs to a concurrent
session. Every convenient default — `mbdeploy deploy` with no target, `pyocd` with
no `-u`, a remembered port number — selects the wrong robot when ours is
unplugged, and port numbers move between sessions
(`/dev/cu.usbmodem2121102` was tovez one morning and a different robot by
afternoon). Now documented in `.claude/rules/hardware-bench-testing.md`.

---

## Recommendations

Ordered by leverage.

1. **Add firmware→host config read-back.** A `ConfigSnapshot` arm in
   `config.proto`. Without it, every calibration is measured against an assumption,
   and this sprint burned real hours proving that twice.

2. **Reconcile nominal and delivered control period, and fix the note.** The
   config asserts they are equal; they differ by 4 ms and always have. Either make
   the loop deliver the nominal or make the constant mean the delivered value —
   but stop asserting something measurably false, because every gain is tuned
   against it.

3. **Give `Preamble::probeSlot` a timeout.** The most common bench condition
   (motor power off) currently produces the least informative failure the firmware
   can produce: no banner, no error, nothing on the wire, and a debugger needed to
   learn "slot 1 did not answer." A bounded probe reporting that would have saved
   hours across four wedges.

4. **Fix heading hold rather than leaving it disabled.** It is currently
   `heading_hold_gain: 0.0` on tovez. It closes on encoder-only heading, so it
   cannot distinguish a heading error from encoder drift — the gain is the
   symptom, the input is the problem.

5. **Settle the slope-vs-intercept question with real data.** Stage C's bias
   adaptation assumes intercept-only variation. Ticket 001's parallel-lines test
   came back slope-dominated, at n=3 and low confidence. If it holds, Stage C needs
   redesign. The playfield actuation-floor measurement is what would settle it.

6. **Do a code review before the next sprint.** Twelve-for-twelve says the next
   sprint will also spend most of its effort on things nobody planned for. Better
   to find them deliberately.

---

## Residuals

Five issues, all filed rather than carried:

- `bench-reverify-residuals-and-the-4ms-delivered-period-offset.md`
- `sprint-130-regressions-speed-floor-snaps-differential-and-shaper-defaults.md`
- `tour2-146-degree-turn-still-undershoots-after-130-010.md`
- `plus500-transient-criteria-and-plant-gain-drift-followup.md`
- `playfield-actuation-floor-measurement-deferred-from-130-012.md`

The agent-facing companion to this document — what a future agent should know
before touching this code — is
`docs/knowledge/2026-08-02-sprint-130-residuals-and-what-went-wrong.md`.
