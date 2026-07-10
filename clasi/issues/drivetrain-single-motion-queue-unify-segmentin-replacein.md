---
status: pending
---

# Drivetrain gets ONE motion queue — unify segmentIn/replaceIn, retire DrivetrainCommand

## Stakeholder direction (2026-07-09)

> I don't like the split between segmentIn and replaceIn. We can't have a
> different one; we got to just make that one. The whole drivetrain should
> only have one queue. I'm thinking that DrivetrainCommand doesn't really
> have a purpose anymore.
>
> I can see why you need to have a work queue and a mailbox, but you should
> get something from the mailbox, and if there's something in the mailbox,
> you clear out the work queue.

## Current shape (what's wrong)

`Subsystems::Drivetrain::tick()` (source/subsystems/drivetrain.{h,cpp})
takes THREE inbound motion channels from the blackboard
(source/runtime/blackboard.h) and drains them in a fixed priority order:

1. `driveIn` — `Rt::WorkQueue<msg::DrivetrainCommand, 8>`, the S/STOP
   escape hatch (WHEELS/TWIST/NEUTRAL). Drained first, one per tick;
   preempts everything.
2. `replaceIn` — `Rt::Mailbox<Motion::Segment>` (MOVER, OOP 2026-07-09).
   Latest-wins replace: supersedes the ring and replans from current
   velocity via `SegmentExecutor::replaceStream`.
3. `segmentIn` — `Rt::WorkQueue<Motion::Segment, 8>` (MOVE). Drained in
   full into the Drivetrain's internal 8-slot `ring_`; append semantics.

Three channels with three different drain semantics is the complaint:
append (`segmentIn`) vs replace (`replaceIn`) should not be two separate
inbound types, and `msg::DrivetrainCommand`/`driveIn` may have no remaining
purpose now that the Drivetrain is the segment executor.

## Desired shape

**One queue, replace as a segment attribute.** (Stakeholder refinement,
2026-07-09, second pass.) There is exactly one inbound channel:
`segmentIn`. A MOVER segment is not a different channel — it is a segment
carrying a "replace" marker (a type/flag on `Motion::Segment`). The drain
rule lives entirely inside the Drivetrain:

- On drain, the Drivetrain first inspects the **newest** (last-posted)
  item in the queue.
- If that item is a MOVER/replace-type segment, the Drivetrain **deletes
  everything in front of it** in the queue (and the in-flight plan is
  superseded — replan from current velocity, today's `replaceStream`
  behavior), then executes from that segment.
- Otherwise, drain as a plain FIFO append into the ring, as today.

The point of anchoring the rule this way: **nobody else deals with MOVER
semantics.** Wire handlers just parse and post segments to the one queue;
only the Drivetrain is allowed to manipulate its queue. No mailbox, no
second drain rule at the blackboard level, no producer-side coordination.

(An earlier sketch from the same conversation kept a replace Mailbox whose
occupancy flushes the queue — superseded by the flag-on-the-segment design
above, which needs no second channel at all.)

**Retire `msg::DrivetrainCommand` / `driveIn`.** The stakeholder's view is
it no longer has a purpose. S/STOP/NEUTRAL should be expressible through
the same single-queue design (e.g. as segments, or as the mailbox-replace
path) rather than a third, higher-priority command type. If any residual
use survives design review (e.g. an emergency-stop latency argument), that
must be argued explicitly back to the stakeholder, not silently kept.

## Touchpoints

- `source/subsystems/drivetrain.{h,cpp}` — `tick()` signature and the
  driveIn → replaceIn → segmentIn drain order; `dispatchEscapeHatch()`;
  `ring_` interaction.
- `source/runtime/blackboard.h` — `driveIn`, `segmentIn`, `replaceIn`
  members and their doc comments.
- `source/commands/motion_commands.*` — MOVE/MOVER wire handlers post to
  the unified channel(s); S/STOP handlers lose their `msg::DrivetrainCommand`
  fan-out.
- `source/motion/segment.h` — `Motion::Segment` gains the replace
  marker (type/flag).
- `source/motion/segment_executor.h` — `replaceStream` entry point.
- `main.cpp` wiring of the tick parameters; sim/bench tests that post to
  `bb.segmentIn`/`bb.driveIn` directly.

## Acceptance sketch

- Drivetrain::tick() takes exactly ONE inbound segment queue; `replaceIn`
  (the Mailbox) is gone from the blackboard and the tick signature.
- `Motion::Segment` carries a replace marker; MOVER's wire handler sets it
  and posts to the same queue MOVE does — handlers contain zero
  replace-semantics logic.
- On drain, a replace-marked newest entry causes the Drivetrain to discard
  everything queued ahead of it and replan from current velocity.
- `msg::DrivetrainCommand` and `bb.driveIn` are removed (or their retention
  is an explicit, stakeholder-approved decision recorded in the sprint
  architecture update).
- MOVE (append), MOVER (replace), S, and STOP all still work over the wire,
  verified on the bench stand per the standing hardware gate.
