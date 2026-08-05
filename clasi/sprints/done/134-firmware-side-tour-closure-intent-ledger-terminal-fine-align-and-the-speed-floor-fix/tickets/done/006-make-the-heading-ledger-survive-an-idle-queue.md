---
id: '006'
title: Make the heading ledger survive an idle queue
status: done
use-cases:
- SUC-002
depends-on:
- '001'
- '003'
github-issue: ''
issue: A-turn-baseline-ledger-ignores-the-preceding-legs-heading-drift.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Make the heading ledger survive an idle queue

**Sequencing: this ticket lands BEFORE ticket 004 is re-run.** 004 is the
proof; without this fix its sequential arm re-measures the same regression.

## Description

**Source of truth: `docs/bench-reports/sprint-134-004-bench-acceptance-2026-08-05.md`
(commit `dec47af4`). Read it before starting. Do not re-derive its numbers — cite
them.**

### The measured problem

Ticket 004's bench A/B found this sprint **improves the pipelined tour** (2.2 /
8.1 / 13.2 mm closure vs. control 5.4 / 24.2 / 27.5 — better 3/3) and
**regresses the sequential tour** (worse 6/6, mean Δ +19.7 mm).

Root cause is isolated to one **pre-existing** line, `Motion::Planner::
activateNext()`, `src/motion/planner/planner.cpp:867`:

```cpp
carryValid_ = carryValid_ && pendingCount_ > 0;  // carry consumed below or dropped
```

Present since planner v1 (`4aea58c1`); **this sprint never touched it.** A caller
that waits for each Move's completion ack before enqueuing the next — which
`planner_square_tour.py --sequential` does, and which is the natural way to drive
from a GUI or a script — always has an empty queue at the boundary. So the
cumulative-intent ledger that ticket 001 restored is **dropped at every corner**,
and 001 is inert in that arm.

Measured on the same leg+turn, differing only in whether the next Move was queued
before completion:

| carry | mean corner residual | corners in tolerance |
|---|---|---|
| LIVE | −0.38° | 4/4 |
| DROPPED | +1.22° | 2/4 |

With the ledger dead, **ticket 003's align phase is actively harmful**: it drives
each turn onto its own activation heading + 90°, which deletes the pre-sprint
firmware's ~+0.7°/corner open-loop over-rotation that had been partially
cancelling the −1.35°/leg curl. Two errors were cancelling; the sprint removed
one of them. That is the whole regression — not a defect in 003.

### The defect, precisely

The guard's *intent* is sound. An empty queue may mean **someone else moved the
robot** — teleop, an estop, a manual shove — and a stale heading carry would then
be wrong. The defect is that it uses **queue occupancy as a proxy for "nothing
disturbed the robot"**, and that proxy is false whenever a caller simply waits.

### The fix: replace the proxy with the real condition

Check the actual condition, from state `Motion::Planner` already has:

- On the transition to idle (the queue-empty exits in `tick()` around
  `planner.cpp:846-852` and the `pendingCount_ == 0` early return in
  `activateNext()`), **latch `pose_.heading()`** into new planner state
  (e.g. `idleHeading_` / `idleLatched_`).
- When the next Move activates with an empty queue, the carry **stays valid only
  if the measured heading has not moved beyond a small tolerance since that
  latch**, and no estop / `replace=true` / cancellation intervened.
- If it moved, **drop the carry exactly as today.** Fail closed.

Use a wrapped angular difference (the planner already has heading-difference
helpers — reuse one, do not write a second).

## Constraints (write these into the implementation, do not negotiate them)

1. **`src/motion/DESIGN.md` §3 forbids `Config::*` / `App::*` / `Devices::*` in
   `src/motion`.** The check must use `pose_` and existing planner state only.
2. **Prefer reusing an existing tolerance** — `limits_.landing.settleEpsilonAngular`
   (0.035 rad on `tovez`, per `src/firm/config/boot_config.cpp:180`) is the
   intended one. **Do not add a `PlannerLimits` field**: ticket 003 traced that
   path at ~16 files, and this sprint is explicitly minimal. If a new field is
   *genuinely* required, **say so and stop** (throw a ticket exception) rather
   than adding it.
   - Caveat when testing: `planner_types.h:141`'s in-struct default is `0.005f`
     and `src/tests/bench/planner_harness.py:370` sets `0.005`. A ctest must set
     the tolerance it intends explicitly rather than inheriting either default.
3. **Must fail closed.** Estop (`Planner::estop()`, `planner.cpp:300`),
   `replace=true` preemption, cancellation, and a genuine external disturbance
   all still drop the carry. When in doubt, drop it.
4. **Must not regress the pipelined arm**, which currently works (better 3/3).
   The `pendingCount_ > 0` path must behave exactly as it does today.
5. Do not touch tickets 001/002/003's landed behavior beyond this one guard.

## Acceptance Criteria

- [x] `carryValid_` is no longer gated on `pendingCount_ > 0` alone; the
      queue-occupancy proxy is replaced by a heading-since-idle check using
      `pose_` and existing planner state only.
- [x] Idle-transition heading latch is set on every path that leaves the planner
      with an empty queue, and cleared/invalidated on estop, `replace=true`, and
      cancellation.
- [x] No new `PlannerLimits` / `Config::*` field is added; the check reuses
      `limits_.landing.settleEpsilonAngular`.
- [x] No `Config::*` / `App::*` / `Devices::*` include or symbol is added under
      `src/motion` (DESIGN.md §3 holds).
- [x] **New ctest: carry SURVIVES an idle gap** — complete an Angle Move, leave
      the queue empty for a gap with the robot undisturbed, enqueue the next
      Move; the adopted `baselineHeading` is the carried ledger value, not
      `pose_.heading()` at activation.
- [x] **New ctest: carry DROPS when the heading moved during the gap** — same
      sequence, but the pose heading is displaced beyond
      `settleEpsilonAngular` during the idle gap; the next Move re-anchors to its
      activation heading.
- [x] Existing planner ctests still pass — **8/8 as of `10b7e13e`**.
- [x] Sim slice matches baseline by identity (see Testing).

## Testing

- **Existing tests to run**:
  - Planner ctests (`planner_tests`): **8/8 must pass**, as of `10b7e13e`.
  - `motion_tests` standalone build.
  - Sim slice baseline is **4 failed / 1406 passed** — the four are
    `test_gen_boot_config_planner` ×2 and `test_gen_boot_config_robot_groups` ×2.
    **Match by identity, not by count.**
  - TestGUI baseline is **7 failed / 591 passed**. Match by identity.
- **New tests to write**: the two ctests in the acceptance criteria (carry
  survives an undisturbed idle gap; carry drops on a disturbed one), in the
  planner ctest suite alongside the existing 8.
- **What the sim CANNOT prove**: corner accuracy. Sim corners sign-flip versus
  hardware — do not read a sim tour closure as evidence this worked. **This
  ticket's real proof is ticket 004's re-run on `tovez`.**
- **Verification command**: the planner/motion ctest builds, then
  `uv run python -m pytest` for the sim and testgui slices.
