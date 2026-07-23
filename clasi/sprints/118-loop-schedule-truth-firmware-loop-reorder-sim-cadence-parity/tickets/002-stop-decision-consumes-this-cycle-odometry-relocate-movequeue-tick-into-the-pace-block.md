---
id: '002'
title: Stop decision consumes this-cycle odometry (relocate MoveQueue::tick into the
  pace block)
status: open
use-cases: [SUC-063]
depends-on: ['001']
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

- [ ] `moveQueue_.tick(now, odom_)` call (+ `kFlagFaultMoveTimeout`
      staging + completion `tlm_.ack()`) moved from the R-settle block to
      the trailing pace block, positioned AFTER `applyOtosSample()` →
      `odom_.integrate()` → `stateEstimator_.update()` and BEFORE
      `updateLineColor()`.
- [ ] R-settle block after this change holds only `processMessage(cmd)`
      and `drive_.tick()` (both pure compute).
- [ ] `applyOtosSample()`'s position and bus-discipline fencing unchanged.
- [ ] Interleave schedule invariants from ticket 001 preserved: per-port
      select→settle→collect, no bus traffic in any settle window, I2C
      clearance safety-net fault bit (flags bit 6) clear during normal
      driving.
- [ ] `app_robot_loop_harness`'s ordering test asserts `moveQueue_.tick()`
      reads odometry/estimator state updated in the SAME cycle (not the
      previous one) — add this assertion if the harness doesn't already
      have an ordering-sensitive check (coordinate with ticket 001's own
      "New tests to write" note).
- [ ] Sim tour-closure gate passes at current-or-better per-leg bands
      (accuracy should improve or hold, not regress — the whole point of
      removing a cycle of staleness).
- [ ] Full `uv run python -m pytest` suite green.
- [ ] `docs/design/design.md` / `src/firm/app/DESIGN.md` overlay
      (already edited ahead of implementation in this sprint's `design/`
      directory) verified to still describe the landed call order
      accurately; reconcile any drift found during implementation.
- [ ] Bench verification is DEFERRED to the phase-B bench session — not
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
