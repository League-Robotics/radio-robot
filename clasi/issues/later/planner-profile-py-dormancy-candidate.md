---
status: pending
---

# `planner/profile.py` — dormancy candidate, no consumer outside its own package

**Source:** 128-009 (doc-rot-and-minor-sweep), team-lead-accumulated sweep
item #4: after 128-007 deleted `planner/executor.py` (`StreamingExecutor`,
zero production callers), `planner/profile.py`'s trapezoidal setpoint
generator lost its own last real consumer along with it.

## What is confirmed

`planner/profile.py`'s `ProfileLimits`/`ProfileSetpoint`/
`profile_for_distance`/`profile_for_turn` have exactly two referrers left
in the whole tree:

1. Its own unit test.
2. `planner/__init__.py`'s re-export (a lazy/plain import forwarding the
   names out of the package).

Nothing outside `robot_radio.planner` itself imports or calls any of them —
`planner/tour.py` (the live tour runner) does not use it; the module it
used to feed, `planner/executor.py`'s `StreamingExecutor`, is gone (128-007).
`src/host/robot_radio/DESIGN.md`'s `planner/` row already records this:
"`profile.py`'s trapezoidal setpoint generator remains dormant -- its only
remaining consumers are its own unit test and `planner/__init__.py`'s
re-export ... which nothing outside the package itself imports."

## Why this is NOT deleted now

128-009 is explicitly instructed not to delete this file — it is a
dormancy candidate for a FUTURE sprint's decision, not a call this
mechanical-sweep ticket makes unilaterally. Per sprint 115's Design
Rationale (Decision 6), dormancy in this package tree is a recorded,
deliberate stakeholder decision, not an invitation for opportunistic
cleanup — `profile.py` deserves the same explicit-decision treatment
`planner/executor.py` got before ITS deletion (128-007), not a drive-by
removal bundled into an unrelated doc-rot ticket.

## What to do (future sprint)

- Decide, with an explicit stakeholder/architecture call: does
  `profile.py`'s trapezoidal setpoint math get resurrected by a future
  motion-planning revival (the same "sprint 116's MOVE protocol is the
  expected path back to life for most of `planner/`/`path/`/`nav/`" bet
  `DESIGN.md` §6 already places on the rest of this tree), or is it truly
  orphaned math with no future consumer?
- If deleted: remove `profile.py`, its unit test, and the
  `planner/__init__.py` re-export; update `DESIGN.md`'s `planner/` row.
- If kept: no action needed beyond periodically re-confirming it's still
  zero-consumer (this issue can be closed once a real decision is made
  either way).

## Acceptance

- A stakeholder/architecture decision recorded (delete or keep,
  explicitly, not by default) — this issue closes once that decision is
  made and any resulting change lands.
