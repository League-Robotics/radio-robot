# Sprint 130 — the residuals, and what they tell you

Written at sprint close, 2026-08-02, for whoever picks this up next.

Sprint 130 shipped: one wheel-speed controller in `App::Drive` serving every
command path, the planner honesty pass, one composition root. A 150 mm/s command
used to produce 52 mm/s; it now lands 500 mm within 10 mm with the wheels 8 mm
apart. Sim square went from failing at ~361 mm to passing at 41.7/43.0 mm.

That is the good news and it is real. This document is about the rest.

## The pattern worth noticing

**Every one of the twelve tickets surfaced a latent defect rather than merely
doing its stated work.** That is not twelve unlucky tickets, it is a code-health
signal, and it is the single most useful thing to carry forward:

- ticket 003 → a completion predicate that only worked when Moves ran in isolation
- ticket 005 → two controllers on one signal, plus two regressions from tickets
  002/004 that it found by bisecting isolated builds
- ticket 009 → five `PlannerLimits` fields the sprint's own plan wanted to keep,
  with zero readers
- ticket 010 → the turn undershoot was a *completion* bug, not a shaping constant
- ticket 011 → a heading loop that cannot survive a control-period change
- the close → two tests that had never compiled

The recurring shape: **something is checked against a proxy rather than against
the thing itself.** A Move is "done" when a lookahead says so rather than when the
plant has stopped. A period "is" 50 ms because a constant says so rather than
because it was measured. A test is "known failing" because a summary line says so
rather than because someone read the error.

## The residuals

Five issues were filed rather than papered over. Roughly in priority order:

### 1. `bench-reverify-residuals-and-the-4ms-delivered-period-offset.md`

**Two things live here, and the second is the important one.**

`Planner::applyHeadingHold()` — a P-loop at gain 2.0 rad/s per rad closing on
**pure-encoder heading** — goes unstable at the longer control period. Bench
square tour: heading 411.8° vs 360 commanded, sustained ~1 Hz wheel reversal,
11.35 s for a leg that should take 4 s. Sim showed none of it. It is currently
**disabled** on tovez (`heading_hold_gain: 0.0`), which is a mitigation, not a
fix. The deeper problem is that it closes on encoder-only heading and so cannot
distinguish a real heading error from encoder drift.

The delivered control period is **+4 ms above nominal, and always has been**:
44 ms when `kCycle` was 40, 54 ms now that it is 50. Sprint 130 did not introduce
this; ticket 007 moved the nominal and inherited it. `cycleBusy` is 21-23 ms, so
it is not budget overrun — it is a deterministic fixed cost, plausibly
`markTime()`'s ms truncation compounding across the loop's four `runAndWait()`
pacing blocks.

**The actionable defect is the lie, not the 4 ms.** The robot JSON's timing note
asserts "the delivered period IS the nominal." It never has been. Every gain ever
tuned against "40 ms" was tuned against 44, and every gain tuned against "50 ms"
is tuned against 54 — including the heading-hold gain that just went unstable.

### 2. `sprint-130-regressions-speed-floor-snaps-differential-and-shaper-defaults.md`

Two regressions this sprint introduced. `applySpeedFloor()` rounds any sub-`vMin`
command up to ~99.7 mm/s. Correct for a teleop command; **wrong for the planner's
small differential corrections**, which are legitimately tiny — a 3 mm/s trim
becomes a 99.7 mm/s lurch. Four testgui tour tests FAULT on it.

Separately, a test asserting "shaping is off unless pushed" now fails because the
unified boot path bakes real shaper defaults. That one is probably the *test*
encoding a pre-unification assumption — confirm before editing either side.

### 3. `tour2-146-degree-turn-still-undershoots-after-130-010.md`

Ticket 010 fixed the general undershoot (worst per-turn error −20.8° → 2.72°) but
TOUR_2's 146° turn still misses by −10.12°. **The residual does not scale with
angle**, which suggests a second additive mechanism rather than more of the one
that was fixed. Three alternative fixes were already tried and rejected — read
those before proposing a fourth.

### 4. `plus500-transient-criteria-and-plant-gain-drift-followup.md`

The +500 button's *endpoint* criteria pass; its *transients* do not — rise 0.59 s
(want ≤0.3), ripple 24 (want ≤10). Stage B (the fast PID) ships at **zero gains
on every robot**, because the robot was unavailable to tune it. A trial at
kp=0.3/ki=0.02 cut rise to 0.18 s but worsened ripple to 33.7, and was reverted
rather than kept as a mixed result.

Also unexplained: **saturation dropped ~25%**, from the historical 760-795 mm/s to
571.7/532.5. No battery telemetry exists to test the obvious hypothesis.

### 5. `playfield-actuation-floor-measurement-deferred-from-130-012.md`

Never started — it needs the robot translating on the playfield, and the stand
cannot substitute (no load, and the OTOS sees a static scene). The provisional
`vMin` 99.7 / `biasMax` ±23.8 / breakaway 0.04-0.09 stand, marked provisional and
flagged **n=3, low confidence**.

Buried in there is a real threat to the design: ticket 001's parallel-lines test
came back **slope-dominated**, the *opposite* of the intercept-only hypothesis
Stage C's bias adaptation is built on. Also n=3. If that holds up with real data,
Stage C needs revisiting — and this measurement is what would settle it.

## What future agents should know

**Anchor on a measurement that needs no config constant.** A duty sweep produced
badly wrong numbers (a claimed 28% L/R mismatch, a 0.24 breakaway) because it
computed its x-axis from a config value the firmware no longer used — and there is
**no firmware→host config read-back path** to catch that. A saturation reading
(command a speed high enough that duty clamps, read the result) depends on nothing
and takes six seconds. Do it first, always.

**Stand facts and playfield facts are not interchangeable.** A "frozen OTOS" was
reported at high priority on the strength of two screenshots; the robot was on the
stand the whole time, so the chassis never moved and the optical sensor correctly
said so. The encoder "metres of travel" was free-spinning wheels.

**"Known pre-existing failure" is a claim, not a fact.** Two tests were carried
through this entire sprint as a known-failing baseline, quoted in every ticket
brief. They were missing an `-I src` flag and had never compiled. Nobody read the
error. Read the error.

**The sim only looked nondeterministic.** ~30 mm of apparent run-to-run spread in
tour closure came from comparing a background-tick-thread harness
(`SimLoop.connect(start_tick_thread=True)`) against the bit-reproducible
single-step harness. Two harnesses, treated as one. Measure turn accuracy on the
single-step harness, and prefer **per-turn yaw error** to end-to-end closure —
closure conflates turn error with leg-length error, and both were present.

**Adding a fudge constant is how this got hard.** `tovez_nocal.json` carried
rotation constants (1.006 / +12.1°) fitted to a *sim artifact*, injecting ~12.5°
of under-rotation into every real turn. `duty_per_speed` and `wheel_gain` were
circularly calibrated against each other's errors, with the correction pointing
the wrong way. Both were removed this sprint. Do not add a third.

**The bench is unreliable and that is its own problem.** tovez wedged **four times
in two days** — same dead-I2C signature, core spinning in
`codal::system_timer_wait_cycles` inside `Preamble::probeSlot`'s motor probe, each
needing a physical power cycle. It gates all bench work. Note also that the probe
has no timeout, so the most common bench condition (motor power off) produces the
least informative failure the firmware is capable of: no banner, no error, nothing
on the wire.

**Two robots share the USB hub.** Ours is `tovez`; `vizev` is someone else's and is
driven by a concurrent session. Every convenient default selects the wrong one when
tovez is absent, and port numbers move between sessions
(`/dev/cu.usbmodem2121102` was tovez one morning and a different robot by
afternoon). Resolve by UID from `mbdeploy list` — see
`.claude/rules/hardware-bench-testing.md`.

## One process note

Ticket 007's agent completed its work and stalled without committing; the work was
found loose in the tree and every verification step re-run by hand before it could
be trusted. Ticket 010's ran 106 minutes and 275 tool calls. If you are
orchestrating this: **verify the tree yourself at each ticket boundary** rather
than trusting a completion report, and check that `move_ticket_to_done` was
called and not just `update_ticket_status` — otherwise the sprint cannot close.
