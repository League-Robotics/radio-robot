---
status: done
filed: 2026-07-23
filed_by: team-lead (stakeholder bug report, /issue during sprint 120 execution)
related:
- chain-advance-completion-margin-narrow-pocket.md
- land-at-zero-at-orthogonal-chain-boundaries.md
tickets:
- 121-002
sprint: '121'
---

# Tour 1 never completes its final leg — it hangs until you press STOP

## Description (stakeholder-observed, 2026-07-23)

Running **Tour 1** (mid-sprint-120 build), the robot executes every leg
**except the last one** and then **just sits there** — the tour does not
finish. Pressing **STOP** causes the last step to then complete, and only
then does the tour report done.

In other words: the final `Move`'s completion is not being recognized on its
own. Something about issuing `STOP` is what unblocks / retires the last leg.

## Why this is suspicious right now

Sprint 120's first ticket (120-001, commit `2f1a2e9f`) **replaced Telemetry's
single ack slot with a bounded ack ring**, and the completion-ack contract is
exactly the mechanism a tour relies on to know a leg finished: a later frame's
ack slot carries `ack_corr == Move.id` (the completion ack). If the host tour
orchestration is waiting on that completion ack for the FINAL move and never
sees it — while `STOP` produces an ack the host *does* consume (or `STOP`
drains/flushes the ring) — that would produce exactly this symptom: all legs
but the last retire normally, the last one hangs, and `STOP` releases it.

This is a **lead, not a diagnosis.** Do not assume it before reproducing.
Candidate mechanisms to check, in order:

1. **Host side** — how the tour runner consumes completion acks from the new
   ack ring. Is it still reading a single ack slot per frame (missing the
   final completion ack when it shares/collides with the enqueue ack in the
   ring), or is it draining the ring correctly? Does the final leg have a
   different completion path than intermediate legs?
2. **Firmware side** — does the final `Move` actually emit its completion ack
   when the queue empties, or is the completion ack only pushed as a side
   effect of the next enqueue / of `STOP`? (An empty-queue drain has no
   deadman; confirm the final completion ack is pushed to the ring on the
   move's own stop-condition, not deferred.)
3. **Is it a hang or a heading-hold?** Confirm whether the robot is truly idle
   (wheels stopped, `kFlagActive` cleared) or still actively holding at the
   final target — the two point at different layers. Note this is NOT the
   `land-at-zero` / chain-margin heading-accuracy family (related issues);
   those are about *how far* it rotates, not about the tour failing to retire
   the final leg.

## Reproduce

Run Tour 1 on the current sprint-120 branch
(`sprint/120-bench-tour-bring-up-with-fake-otos`) — via TestGUI (Sim → Tour 1)
and/or on the stand. Watch the telemetry ack slots across the final leg: does
a frame ever carry `ack_corr == <final Move.id>` with `kFlagActive` dropping,
WITHOUT a `STOP` being sent? Capture the frames around the last leg's expected
completion and around the `STOP` that unblocks it.

## Notes

- Filed as a bug report only (quick capture). Not yet triaged into a ticket;
  sprint 120 ticket 003 is currently in-progress on an unrelated arc.
- If this reproduces in **Sim** (deterministic), that isolates it to the
  host/firmware completion-ack path independent of hardware timing — the
  fastest place to bisect against `2f1a2e9f` (pre- vs post- ack-ring).
