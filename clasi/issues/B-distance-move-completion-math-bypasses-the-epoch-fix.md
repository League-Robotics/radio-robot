---
status: pending
priority: medium
---

# `Motion::Planner`'s Distance-Move completion math is a third un-epoch-aware reader of raw wheel position

## Description

Found by sprint 131 ticket 004's implementer while closing the position-rebaseline
defect (post-130 review F1). Correctly flagged rather than fixed — it sits outside
that ticket's acceptance criteria, which name only the two `integrate()` methods.

Ticket 004 (`0edefd06`) gave `positionEpoch` real motion-side consumers:
`Motion::Odometry::integrate()` and `Motion::PoseTracker::integrate()` both now
take per-wheel epochs and re-anchor on a change instead of differencing across
the ~30,000 mm rebaseline jump.

**A third consumer was missed by that framing.** `Motion::Planner`'s Distance-kind
Move completion math (`active_.baselinePath` / `meanPosition`, `planner.cpp`) reads
`WheelChannel::basisPosition()` **directly**, bypassing `PoseTracker` entirely. It
is therefore still differencing raw, rebaseline-visible absolute positions.

## The surviving failure

A Distance-kind Move whose activation and rebaseline crossing straddle the
±30,000 mm margin has its residual distance corrupted — the same class of defect
F1 described, on a narrower path. The Move completes in the wrong place.

Trigger is the same as F1's: ~30 m of net travel on a wheel, about 75 s at
400 mm/s. `src/tests/bench/move_soak.py` reaches it.

## Why F1's own text pointed here and the ticket still missed it

`A-position-rebaseline-destroys-the-pose.md` says *"the planner's
`PoseTracker`/`WheelChannel` (`src/motion/planner/estimation.cpp:8-48`) have the
identical raw-delta shape."* Ticket 004's implementer verified by inspection that
`PoseTracker` — not `WheelChannel` — holds the raw-delta shape (`WheelChannel`
only anchors an absolute position/velocity/time triple). That was correct **for
`integrate()`**. What neither the issue nor the ticket caught is that a *different*
call path reads `WheelChannel::basisPosition()` for Move completion, and that path
has the same exposure by another route.

Worth recording as a pattern, not just a bug: the fix was scoped to the two
functions that *resembled* the defect, when the real invariant is **"any reader of
an absolute wheel position must know about epochs."** Enumerate the readers, not
the functions that look like the original report.

## What to do

1. Enumerate every consumer of raw absolute wheel position across `src/motion` and
   `src/firm` — grep `basisPosition`, `position()`, `state.wheel*.position` — and
   classify each as epoch-aware or not. Record the enumeration so the next person
   does not re-derive it.
2. For the Distance-Move completion path: either route it through `PoseTracker`
   (now epoch-aware) or give it the same per-wheel epoch re-anchor.
3. **Consider the structural fix instead.** Having `publishWheel()` hand motion a
   *delta* rather than an absolute position closes the whole class by
   construction, so no future reader can be exposed. That option was weighed
   during sprint 131's planning and the re-anchor approach was chosen; this second
   miss is evidence for revisiting the decision rather than patching a third
   reader.

## Verification

- A Distance-kind Move that activates before and completes after a rebaseline
  crossing lands at the commanded distance, verified against a no-rebaseline
  control run — the same shape as ticket 004's
  `src/tests/sim/system/rebaseline_pose_harness.cpp` Move-in-flight scenario,
  which covered only the Angle-kind case.
- If option 3 is taken instead, the harness proves no consumer sees an absolute
  position at all.

## Related

- Sprint 131 ticket 004 (`0edefd06`) and its completion notes — the fix this
  extends, and the harness to extend.
- [[A-position-rebaseline-destroys-the-pose]] — the originating issue. It is
  marked complete by ticket 004; this path remained open, which is why this is
  filed separately rather than reopening it.
- `docs/code_review/2026-08-02-post-130-wheels-solid-review.md` F1.
