---
status: pending
---

# Motion terminal precision: decel/coast anticipation for the new motion verbs

## Context

Sprint 084 restored the closed-loop motion verbs (`D/T/R/TURN/RT/G`) on the new
`source/` tree, ported from `source_old` but deliberately WITHOUT the
sprint-072-era D-mode `SAFETY_MARGIN`/`ARRIVE` refinements and without RT/TURN
coast/slip anticipation (084 architecture Open Question 1; ticket 001 scope). As a
result, on both sim and the real robot the verbs complete and emit their
`EVT done ... reason=` at the target, but terminal precision is loose:

- `D` overshoots (~538mm on a 500mm command in sim) — the ramp decelerates only
  after the stop condition fires, no look-ahead deceleration.
- `TURN`/`RT`/`G` show a few mm / few degrees of terminal settle-back, traced to
  the sprint-081 velocity-PID **zero-crossing dwell + reset-guard armor** kicking in
  as the wheels ramp through zero at start/stop of an in-place reversal (measured on
  the bench, sprint 084 ticket 003/004/009).

For the TestGUI-revival demos (tours, GOTO) this is functional but visibly
imprecise — a tour accumulates a few mm/degrees of error per leg.

## Scope (a future refinement sprint, not blocking the revival)

Port/add deceleration-anticipation so motion decelerates to arrive at the target
rather than overshooting-then-settling, and reconcile the terminal-approach control
with the motor zero-crossing dwell/reset-guard armor (so a controlled turn-in-place
isn't penalized by the safety armor). Reference: `source_old`'s D-mode
`SAFETY_MARGIN`/`ARRIVE` + RT coast-arc anticipation, and the sprint-073 sim turn
accuracy work. Verify against tighter sim geometry tolerances + a bench re-run.

## Dependencies

Depends on sprint 084 (the motion verbs it refines). Independent of the TestGUI
host work (083/085) — the verbs are usable as-is; this only tightens precision.
