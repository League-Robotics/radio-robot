---
title: "Cycle-order experiment \u2014 keep reordered loop through motion work, A/B\
  \ revert-and-compare just before hardware"
filed: 2026-07-19
filed_by: team-lead (Eric's live cycle-order experiment on pid-debugging)
status: pending
related:
- motion-control-terminal-blips-reconciled-fix-plan.md
- later/turn-lead-compensation-gain-cotuning.md
sprint: '115'
tickets:
- 115-003
---

# Cycle-order experiment — A/B revert-and-compare before hardware

## What's going on

On `pid-debugging` (as of 2026-07-19), `RobotLoop::cycle()` in
`src/firm/app/robot_loop.cpp` has `drive_.tick()` **hoisted to the top** of the
cycle, running BEFORE `pilot_.tick()` (which sits late in the kSettle block).
This contradicts the `pilot_.tick()` comment claiming it is placed "BEFORE
`drive_.tick()`."

This is **not a defect** — it is a deliberate experiment: Eric reordered the
main loop to test whether stage ordering contributes to the motion errors we're
chasing (the terminal blips; see the reconciled fix plan). The 2026-07-18
architecture review snapshotted this WIP and read it as its finding "F1."

## Observation so far

**The reorder changed the observed errors.** Ordering matters — this is signal,
not noise. That result is itself evidence for the F1 latency thesis (a fresh
reference / sensor pose lagging the loop by a cycle costs accuracy) and is worth
preserving rather than losing at revert time.

## Decision (Eric, 2026-07-19)

**Keep the reorder in place for the remainder of the motion-blips work.** The
current sim tests depend on it, and it appears to help. Do **not** revert it now.

The A/B comparison is **deferred to just before returning to hardware**:

1. Finish everything else in the reconciled fix plan
   (`motion-control-terminal-blips-reconciled-fix-plan.md`) with the loop
   in its current reordered state.
2. Just before the hardware bench pass, **revert to the intended order**
   (`pilot_.tick()` before `drive_.tick()`, matching the comment/DESIGN).
3. Re-run the same sim acceptance traces (D700 straight, 360° pivot) both ways
   and A/B compare — capture the numbers, not just an impression.
4. **Keep whichever order wins.** If the reordered version wins, make it the
   *real* order: update the `pilot_.tick()` comment and `src/firm/DESIGN.md` /
   sprint cycle-placement table so code and comments agree, and retire the
   "experiment" framing. If the intended order wins, revert and record why the
   reorder had looked better (likely a latency artifact the rest of the fix
   plan removed).

## Scope note — don't conflate with the odometry-staleness half of F1

The drive/pilot swap is one thing; the ~1-cycle **odometry/OTOS staleness**
(`applyOtosSample()` integrates at kPace, end of cycle, and is read by the
*next* cycle's `pilot_.tick()`) is a separate, more durable latency that
survives any drive/pilot ordering and is corroborated by
`later/turn-lead-compensation-gain-cotuning.md`. Evaluate that one on its own
merits — this experiment does not settle it.

## Supersedes

The "restore the baseline first before judging F1" sequencing in the reconciled
fix plan's execution order — that plan has been updated to defer the revert to
this issue instead.
