---
status: done
---

# Host-side ack-drop fault injection for deterministic retry-path testing

## Description

**Low priority. Explicitly not blocking anything.** This is a testing-gap
follow-up, not a defect report.

Sprint 127 ticket 002 verified `App::RobotLoop::handleMove()`'s `Move.id`
dedup (`alreadyAccepted()`/`recordAccepted()`/`acceptedMoveIds_`) against
all four rules from `clasi/sprints/127-host-side-path-planner-goto-and-
path-following/issues/duplicate-move-enqueue-on-ack-loss-retry.md`, in
sim, driving the real firmware code path via `TestSim::SimHarness`. Rule
3 (window outlives completion) was also confirmed incidentally on real
hardware: a radio-relay run's reused move IDs collided with a prior run's
already-accepted `acceptedMoveIds_` ring, and the firmware correctly
suppressed them.

The ticket's original hardware acceptance criterion wanted at least one
run whose log contained a real, naturally-occurring `(retry N for move
…)` line — proof that a host retry (triggered by a genuinely lost enqueue
ack) landed on an already-accepted id and was suppressed, not double-
executed. 160 enqueue commands across two transports (direct USB and the
RADIOBRIDGE relay) produced zero natural ack loss, so that specific
scenario was never exercised end-to-end over the real wire.

Traced (ticket 002's completion notes) to commit `cc04ac84` (2026-07-28),
which turned off the I2C IRQ guard (`bus.setIrqGuard(false)`,
`src/firm/main.cpp`) to fix ~7-8% inbound command loss on direct USB. The
duplicate-execution bug the dedup fixes was itself provoked by that same
loss (a lost *command* consumed a retry, and a subsequent retry landing
on a lost *ack* is what the original issue actually observed before the
firmware had any dedup at all). With the guard off, inbound loss is
effectively zero on both transports measured — which also killed the
natural-retry path this ticket's hardware criterion wanted. The ticket
was closed on amended criteria (sim verification + the incidental
hardware confirmation of Rule 3) rather than reproducing a condition
whose root cause has already been fixed. See `clasi/issues/
make-irq-guard-off-permanent-and-reconcile-the-docs.md` for the guard's
own removal proposal.

## Proposal

Build a small, explicit host-side fault-injection capability that can
deterministically drop one specific enqueue ack by `corr_id`, so a host
retry fires on demand instead of depending on a link that is now mostly
loss-free. Candidate shapes:

- A `SerialConnection`/`NezhaProtocol` wrapper (or a decorator around the
  read path) that filters out frames carrying a targeted ack ring entry
  before the caller sees them.
- A helper living in a NEW bench script — not `planner_square_tour.py`
  itself, to keep that file's own reproducer honest and unmodified by
  synthetic behavior.

This would give a deterministic regression test for the dedup's
retry-suppression path specifically (as opposed to the sim's own coverage,
which is real-firmware-code-path but not real-wire), closing the one gap
ticket 002 left open. Not urgent: the dedup is already verified via sim
plus the incidental hardware evidence, and the path this would exercise
is now unreachable in ordinary operation — which is the intended,
positive outcome of the IRQ-guard fix, not a regression to chase.

## Related

- `clasi/sprints/127-host-side-path-planner-goto-and-path-following/tickets/done/002-move-id-dedup-verification-under-ack-loss.md`
  — the ticket this follow-up was proposed from; see its Completion
  Notes for the full USB + relay run data and the `cc04ac84` analysis.
- `clasi/issues/make-irq-guard-off-permanent-and-reconcile-the-docs.md`
  — the IRQ guard's own removal proposal (why the guard-off state, and
  therefore the near-zero loss rate, should be made permanent).
