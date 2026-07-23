---
id: '002'
title: Stop decision consumes this-cycle odometry (relocate MoveQueue::tick into the
  pace block)
status: done
use-cases:
- SUC-063
depends-on:
- '001'
github-issue: ''
issue: stop-decision-must-see-this-cycles-odometry.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Stop decision consumes this-cycle odometry (relocate MoveQueue::tick into the pace block)

## Description

`MoveQueue::tick()` — the MOVE stop decision — currently runs (and, after
ticket 001, will continue to run) in the R-settle block, BEFORE
`odom_.integrate()`/`stateEstimator_.update()` run in the trailing pace
block. Every stop decision therefore reads odometry integrated at the END
of the PREVIOUS cycle: a full cycle of heading/distance staleness (5.7°
at 2 rad/s on the old 50ms sim cycle; proportionally less at 40ms, but
still one whole cycle). The 45ms `stop_lead_ms` anticipation constant in
`MoveQueue` exists in large part to cancel this exact ordering choice.

Move `moveQueue_.tick()` (+ its completion ack/fault staging currently in
the R-settle block per `robot_loop.cpp`) OUT of the R-settle block and
INTO the trailing pace block, positioned AFTER `applyOtosSample()` →
`odom_.integrate()` → `stateEstimator_.update()`. All three relocated
pieces (`MoveQueue::tick()`, `Odometry::integrate()`,
`StateEstimator::update()`) are pure compute with no bus traffic —
verified against `move_queue.cpp` (reads cached odometry),
`odometry.cpp:17-18` (reads cached motor positions), `state_estimator.cpp`
(arithmetic only) — so this is legal to relocate freely within the pace
block without touching bus-discipline invariants.

**Constraint — do not touch `applyOtosSample()`'s position.** It performs
the rate-limited OTOS I2C burst read and must stay fenced in the pace
block outside any motor request/collect window, exactly where it already
is; this ticket only moves `moveQueue_.tick()` relative to it and to
`odom_.integrate()`/`stateEstimator_.update()`.

**Reconciliation with ticket 001 is mandatory, not optional**: ticket 001
restores the schedule with `moveQueue_.tick()` still in R-settle (matching
its own issue's target skeleton, which predates this relocation). This
ticket lands on top of that restored schedule and must not silently
revert ticket 001's other changes (constants, `drive_.tick()` placement,
per-port interleave). After this ticket, the R-settle block holds only
`processMessage(cmd)` and `drive_.tick()`; a MOVE that arrives and
activates in cycle N gets its first shaped/staged twist at N's own pace
block via the normal `enqueue()`/`activate()` path (unaffected by this
change — only the STOP DECISION moves, not activation).

## Acceptance Criteria

- [x] `moveQueue_.tick(now, odom_)` call (+ `kFlagFaultMoveTimeout`
      staging + completion `tlm_.ack()`) moved from the R-settle block to
      the trailing pace block, positioned AFTER `applyOtosSample()` →
      `odom_.integrate()` → `stateEstimator_.update()` and BEFORE
      `updateLineColor()`. Implemented in `src/firm/app/robot_loop.cpp`'s
      `cycle()` — `nowUs` (the pace block's own already-captured "now",
      the same value `applyOtosSample()`/`stateEstimator_.update()` used
      immediately above) is reused for the `moveQueue_.tick()` call
      instead of a second `clock_.nowMicros()` read, matching
      `move_queue.h`'s own "never re-read a current value mid-tick"
      convention now that tick() lives in the same block as the other
      readings.
- [x] R-settle block after this change holds only `processMessage(cmd)`
      and `drive_.tick()` (both pure compute). Verified by reading the
      landed `cycle()` body.
- [x] `applyOtosSample()`'s position and bus-discipline fencing unchanged
      — not touched by this ticket's diff.
- [x] Interleave schedule invariants from ticket 001 preserved: per-port
      select→settle→collect, no bus traffic in any settle window.
      `grep 'runAndWait\|sleepUntil' src/firm/app/robot_loop.cpp` is
      still exactly the same 4 `runAndWait` call sites ticket 001 left
      (plus the 2 defining occurrences in `runAndWait`/`sleepUntil`
      themselves) — no wait added or removed, only a pure-compute call
      moved between two existing blocks. `app_robot_loop_harness`'s
      ScriptedI2CHook scenarios (exact bus-transaction-budget checks,
      `bus.errCount(...) == 0`) all still pass, including the NEW
      ordering scenario below, confirming no bus-discipline regression.
      I2C clearance safety-net fault bit (flags bit 6) staying clear
      during normal driving is a bench-only observable (real hardware
      timing) — deferred to phase-B per this ticket's own last bullet,
      same as ticket 001.
- [x] `app_robot_loop_harness`'s ordering test asserts `moveQueue_.tick()`
      reads odometry/estimator state updated in the SAME cycle (not the
      previous one). Added
      `scenarioMoveDistanceStopReadsThisCyclesOdometryNotLastCycles()`
      (SUC-063) to `src/tests/sim/unit/app_robot_loop_harness.cpp`: a
      ScriptedI2CHook-based scenario that scripts an exact, deterministic
      10mm/cycle straight-line encoder ramp so `odom_.pathLength()`'s
      growth per cycle is known exactly, places a 30mm DISTANCE stop
      threshold to land exactly on cycle 3's own `odom_.integrate()`
      call, and asserts the Move has ended by the END of cycle 3 (not
      cycle 4). **A/B-verified**: this exact assertion FAILS against the
      pre-relocation code (git-stashed `robot_loop.cpp` only, scenario
      added) with `expected true, got false`, and PASSES against the
      landed relocation — confirming the test genuinely exercises the
      ordering fix, not a tautology. Also confirms the completion ack
      rides the NEXT cycle's frame (staged in cycle 3's own pace block,
      which runs after that cycle's own `tlm_.emit()` in the kClear
      block — visible starting cycle 4), unchanged from before this
      ticket.
- [ ] Sim tour-closure gate passes at current-or-better per-leg bands
      (accuracy should improve or hold, not regress — the whole point of
      removing a cycle of staleness). **NOT MET at the current
      `stop_lead_ms=45.0` default — investigated honestly, not silently
      retuned.** Full before/after data and the sweep this bullet's own
      instructions authorized:

      **The relocation itself works exactly as designed** — the closure
      gate's failure mode genuinely *shifts*, not just persists unchanged,
      confirming the staleness fix is real and load-bearing:
      - Before (pre-002, staleness present, `stop_lead_ms=45`, same 40ms
        cycle): `TOUR_2/ideal` fails (turns 6/12/14 miss by
        +4.84/+3.77/+4.51deg); `TOUR_1/ideal`, `TOUR_1/realistic`,
        `TOUR_2/realistic` — worst 1.61/1.63/4.66deg (3 of 4 combos pass
        or are close).
      - After (this ticket, staleness fixed, `stop_lead_ms=45`
        unchanged): `TOUR_2/ideal` now PASSES cleanly (worst 2.42deg,
        down from 4.84) — direct confirmation the fix helps exactly the
        turns it targets. But `TOUR_1/ideal` now FAILS instead (turns
        8/12 miss by +4.20/+4.39deg, was 1.61deg passing before) and
        `TOUR_1/realistic` moves from 1.63deg (passing) to 2.63deg
        (barely failing). Net: still 1 test failure
        (`test_tour_1_and_tour_2_ninety_degree_turns_land_within_the_shaped_band`),
        different combos/turns, overall worst |error| about the same
        (4.84→4.43deg) — the accuracy-vs-staleness trade is real but the
        NET effect at the unchanged lead is a wash against the 2.5deg
        shaped-band bar, not the improvement the ticket hoped a clean
        staleness removal alone would deliver.
      - **Wider blast radius than the closure gate alone**: the SAME
        root cause (a 45ms lead now under-anticipating against fresh
        rather than stale odometry) also newly regresses
        `test_gui_button_acceptance.py::test_managed_angle_preset[±90]`
        and `::test_managed_seg_0_cdeg_turn[±90]` — isolated single
        90deg turns that passed CLEANLY before this ticket (measured
        88.1-89.3deg, slight undershoot, well inside their own tightened
        ±3.0deg 90deg-case band) now measure +93.7 to +93.8deg (~+3.7 to
        +3.8deg overshoot), just outside that band. A/B-confirmed the
        same way as the closure gate (git-stashed `robot_loop.cpp`
        only): all 4 pass before, all 4 fail after, at the identical
        `stop_lead_ms=45`. `test_tour_2_runs_to_completion` (own 5deg
        tolerance) also fails in both the before and after runs (leg 6
        misses by +6.06deg before, leg 2 by +5.81deg after) — NOT
        attributable to this ticket; it was already past its own noise
        floor beforehand (ticket 001's own report already flagged it as
        "borderline-flaky" at 5.001deg once).
      - **Sweep performed** (0-120ms in 5-10ms steps, then 0.5-1ms steps
        near the best region found; `_make_loop(stop_lead_ms=...)`
        against TOUR_1+TOUR_2 × ideal+realistic — the closure gate's own
        exact path; ad hoc analysis script, not committed test
        infrastructure): overall worst |error| is 9.31deg at lead=0,
        falls through 45ms (4.43deg, failing) to a narrow window at
        62.0-62.5ms (worst=2.375deg — the ONLY sampled values under the
        2.5deg bar), then rises again past 63ms (3.48deg) and keeps
        climbing through 70-120ms (4.3 to 8.98deg, turn-error signs flip
        from overshoot- to undershoot-dominated around 65-70ms). The
        immediate neighbors of the one passing window are NOT close:
        60.5/61.0/61.5ms measure 3.0-3.1deg, 63.0/63.5ms measure 3.48deg
        — the passing region is about 1ms wide with ~0.13deg (5%) of
        headroom, nothing like the "broad, flat plateau (spans
        lead=30-54ms)" this codebase's own `test_tour_closure_gate.py`
        comment documents as the standard this project holds a
        `stop_lead_ms` pick to. This sweep never even checked the
        isolated-90deg-turn regression above (a second, independent
        constraint) — the closure-gate-only search already had no safe
        margin.
      - **Decision: `stop_lead_ms` left UNCHANGED at 45.0** (source
        default and all three `data/robots/*.json` — no edits made). Per
        this ticket's own instructions ("do NOT retune stop_lead
        silently... permitted ONLY if the closure gate demands it, must
        be data-derived (sweep)... reported") and `sprint.md`'s Out of
        Scope: a re-baseline is data-justified only when it produces a
        genuinely safer operating point. 62ms does not — it is a
        coincidental near-zero crossing of two independently-drifting
        error curves (TOUR_1 turn 4 undershooting more as lead rises past
        60, TOUR_2 turn 4/realistic peaking around the same region), not
        a validated plateau, and would be a documented-fragile 4th
        same-issue retune (see
        `clasi/issues/land-at-zero-completion-delete-stop-lead.md`,
        which already records three prior retunes in one day: 90→60→45).
        Shipping a value with ~5% margin on a metric this ticket's own
        sweep shows swings by 2-9deg across a 120ms range would very
        likely reintroduce exactly the flakiness that issue already
        diagnoses `stop_lead_ms` itself as causing structurally, and
        ticket 003 (sim/firmware cadence-parity fixes) still has to land
        on top of this before the schedule is fully settled — retuning
        now would very plausibly need a 5th retune once ticket 003 lands.
      - **Escalated, not silently resolved.** Full sweep table and
        before/after transcripts recorded as an addendum to
        `clasi/issues/land-at-zero-completion-delete-stop-lead.md`
        (sprint 119's own planned `stop_lead_ms` deletion + land-at-zero
        completion gate) — this data is direct, load-bearing input to
        that work, not a dead end: it reinforces that issue's own "no
        single tuned value survives an unrelated pipeline-stage change"
        finding with a fourth data point measured against the NOW-CORRECT
        (fresh, not stale) odometry basis. Per this ticket's own
        instructions this is reported to the team-lead rather than
        resolved unilaterally, matching ticket 001's own precedent for
        its own (related) closure-gate finding.
- [ ] Full `uv run python -m pytest` suite green. **NOT MET** — same root
      cause as the bullet above (all 6 failures are turn-accuracy
      assertions downstream of the `stop_lead_ms`/fresh-odometry
      interaction, not a bus-discipline, compile, or logic defect in the
      relocation itself): `test_tour_closure_gate.py::test_tour_1_and_tour_2_ninety_degree_turns_land_within_the_shaped_band`,
      `test_gui_button_acceptance.py::test_managed_angle_preset[90]`,
      `::test_managed_angle_preset[-90]`, `::test_managed_seg_0_cdeg_turn[90]`,
      `::test_managed_seg_0_cdeg_turn[-90]`, `::test_tour_2_runs_to_completion`.
      Full run: **1366 passed, 2 skipped, 9 xfailed, 2 xpassed, 6 failed**
      (497.77s). Every non-turn-accuracy test in the suite — including
      every `app/`, `devices/`, `motion/`, sim-plant, config, and protocol
      test, and the `app_robot_loop_harness` C++ harness (11 scenarios,
      including the new SUC-063 ordering scenario) — passes cleanly.
- [x] `docs/design/design.md` / `src/firm/app/DESIGN.md` overlay
      (already edited ahead of implementation in this sprint's `design/`
      directory) verified to still describe the landed call order
      accurately; reconcile any drift found during implementation.
      Verified: `design/DESIGN.md` (the `src/firm/app/DESIGN.md` overlay
      copy) already stated, ahead of implementation, that
      `moveQueue_.tick()` moves "from the R-settle block into the
      trailing pace block, evaluated AFTER
      `applyOtosSample()`/`odom_.integrate()`/`stateEstimator_.update()`"
      — matches the landed code exactly, no drift, no edit needed.
      `design/design.md` (system doc overlay) makes no per-block claim
      about `moveQueue_`'s position, so nothing there could drift either.
- [x] Bench verification is DEFERRED to the phase-B bench session — not
      required to close this ticket.

## Testing

- **Existing tests to run**: `uv run python -m pytest` (full suite);
  `app_robot_loop_harness` (ordering-sensitive); sim tour-closure gate;
  button-acceptance suite.
- **New tests to write**: an explicit ordering assertion in
  `app_robot_loop_harness` (or equivalent) proving `moveQueue_.tick()`
  observes THIS cycle's `odom_`/`stateEstimator_` state rather than the
  prior cycle's — e.g. drive a scenario where a stop condition is crossed
  exactly at the integration boundary and assert completion fires on the
  cycle the crossing occurs, not one cycle later.
- **Verification command**: `uv run python -m pytest` plus the sim
  tour-closure gate and button-acceptance suite runs.
