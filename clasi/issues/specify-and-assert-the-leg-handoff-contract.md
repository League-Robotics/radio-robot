---
status: pending
filed: 2026-07-22
filed_by: team-lead (turn-execution review R5/D5, claims verified against code)
related:
- land-at-zero-completion-delete-stop-lead.md
- simple-velocity-control-acceleration-limited-shaper.md
---

# Specify and assert the chain-advance leg hand-off contract

## Description

Chain-advance deliberately carries shaper state across legs (SUC-051):
`activate()` stages the carried-over `commandedSpeed()` per axis rather than
raw cruise (`move_queue.cpp:45-47,65-66,70-71`), and chain-advance
(`:261-265`) never resets the shapers (reset only on empty-queue,
`:275-278`). A D→RT boundary additionally crosses `NezhaMotor`'s per-wheel
100 ms reversal dwell (`reversal_dwell_ms: 100` in all three JSONs, armed at
`nezha_motor.cpp:576-582`) on the reversing wheel only — asymmetric by
construction. This is the measured isolated-vs-tour turn gap (~0.3° vs
~1.4-1.7°).

None of this is specified anywhere: `motion/DESIGN.md:311-317` lists it under
Open Questions as a tuned-around limitation. Nobody has written what the
carried state SHOULD do to heading at a boundary, so each campaign re-tunes
around it.

## Proposed fix

1. **One contract paragraph in `src/firm/motion/DESIGN.md`** (move out of
   Open Questions), deciding at minimum:
   - Carried axis the next Move commands: ramp from carried speed (SUC-051,
     keep).
   - Carried axis the next Move does NOT command: decay behavior (current:
     shaped decay from carry-over) — state it and its heading cost bound.
   - Sign reversal on an axis at a boundary: carried speed does not survive
     a reversal (clamp toward zero through the dwell); state the expected
     per-wheel dwell asymmetry on D→RT and its accepted heading budget, OR
     specify symmetric dwell (both wheels wait) if the budget is rejected.
     The existing `simple-velocity-control-acceleration-limited-shaper.md`
     issue's vExit design (exit velocity = next move's cruise on the axis; 0
     on reversal or empty queue) is the reference semantics — adopt or
     explicitly reject it in the paragraph.
2. **Assert it in the boundary test**: the tour-level boundary test
   (`test_tour_closure_gate.py`, the two-compatible-distance-legs test)
   asserts the specified carried-velocity behavior and the specified D→RT
   heading budget, instead of xfail-ing around it. Re-point its xfail reason
   (which cites a deleted issue file,
   `cycle-order-reorder-experiment-ab-before-hardware.md`) at this issue or
   at `restore-the-interleaved-request-settle-tick-loop-schedule.md`, and
   un-xfail once the loop-schedule work lands.

Behavior changes here should be minimal-to-none; this is
specify-then-assert. Any behavior change discovered to be needed (e.g.
reversal clamp) rides the land-at-zero ticket's acceptance bands.

## Acceptance

- DESIGN.md contract paragraph exists; Open Questions entry removed.
- Boundary test asserts the contract (carried velocity through compatible
  boundary; specified reversal/dwell behavior) and is no longer xfail, or its
  xfail cites a live issue with a concrete unblocking condition.
- Tour vs isolated turn gap measured and within the budget the contract
  states.
