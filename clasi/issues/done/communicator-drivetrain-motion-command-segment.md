---
status: done
sprint: 094
tickets:
- 094-006
- 094-007
---

# Communicator-issued drivetrain motion command (drive the motion planner directly)

## Context

Once the Drivetrain owns motion planning
([`drivetrain-becomes-the-motion-planner-segment-executing-subsystem.md`](drivetrain-becomes-the-motion-planner-segment-executing-subsystem.md))
and the main loop is gutted
([`simplify-the-main-loop-strip-it-to-bare-wheel-driving.md`](simplify-the-main-loop-strip-it-to-bare-wheel-driving.md),
[`get-wire-output-events-telemetry-out-of-the-main-loop.md`](get-wire-output-events-telemetry-out-of-the-main-loop.md)),
the drivetrain needs **one very simple command coming straight off the
communicator** so we can drive it around doing **motion planning, not path
planning**, with no hiccups.

Stakeholder framing (2026-07-08): "the drivetrain's got a very simple command. I
want [a command] for that coming out of the communicator so that we can drive the
drivetrain around doing motion planning, not path planning, and we can do that
with no hiccups. Everything about this is getting rid of a lot of the lags and
crap that's made this hard."

**Terminology:** the stakeholder said "statement"; per
`.claude/rules/naming-and-style.md` §4 the wire-inbound thing is a **command**
(the "statement" category was removed sprint-wide 2026-07-07). This issue names it
a command.

**Motion planning, not path planning:** the command specifies *one motion*
(how far / which way / final heading) and the drivetrain's motion planner
generates the trajectory and executes it. It does **not** carry waypoints, world
poses, or pursuit goals — path/route selection stays with a higher-level component
(parked with the Planner/GOTO). This is the direct, low-latency drive path for
bench + testing.

## What this issue covers

The **command surface** from the Communicator to the Drivetrain's motion planner —
the companion to the Drivetrain-restructure issue (which owns the executor/queue
internals). Specifically:

1. **One motion command verb** parsed by the communicator/command layer and
   enqueued as a unified **segment** onto the Drivetrain's queue (the segment
   shape is defined by the Drivetrain issue: `distance` `// [mm]`, `direction`
   `// [rad]`, `finalHeading` `// [rad]`, + motion limits). No blackboard
   round-trip through a Planner — the handler builds a segment and hands it to the
   Drivetrain directly (or via the one drive mailbox), matching the gutted-loop
   data path.
2. **Degenerate cases fall out of the one shape** (per the Drivetrain issue):
   straight drive (`finalHeading` = travel direction), in-place turn
   (`distance = 0`), translate-then-pivot-to-heading.
3. **A direct wheel-drive escape hatch stays** for the barest testing: `S <l> <r>`
   (signed wheel velocities) → direct wheel targets, no planning — as the gutted
   loop already provides. `STOP` triggers the graceful decel-to-zero.
4. **No hiccups / no lags** is the acceptance spirit: the command must take effect
   the tick after it arrives, with no multi-hop mailbox latency and no dropped
   commands (respect the 093 yield-once-per-slack RX fix and the actuation
   dead-time work).

## Open questions for sprint planning

- **Exact verb + arg grammar.** Reuse an existing verb (`D`/`TURN`/`RT` re-parsed
  into segments, per the Drivetrain issue's "Integration seam") or introduce one
  new compact segment verb (e.g. `MOVE <distance> <direction> <finalHeading>
  [limits…]`)? Recommend deciding alongside the Drivetrain restructure so the wire
  grammar and the segment struct land together.
- **Velocity/timed paths** (`S` stream, `T` timed) are velocity/time-bounded, not
  distance-bounded — keep `S` as the direct-twist/wheel escape hatch; defer `T`.
- **Correlation id / completion event** — a completed segment should still surface
  a `done`-style event, drained through the wire-layer seam from the
  get-wire-output issue (not a synchronous emit in any loop).

## Depends on

- `drivetrain-becomes-the-motion-planner-segment-executing-subsystem.md` (segment
  shape, executor, queue, graceful stop) — **plan/land together**.
- `simplify-the-main-loop-strip-it-to-bare-wheel-driving.md` +
  `get-wire-output-events-telemetry-out-of-the-main-loop.md` (the gutted loop and
  the wire-output drain seam this command's completion event uses).

## Closed 2026-07-09 — delivered by sprint 094 (+ teleop OOP follow-on)

`MOVE` (parse → `Motion::Segment` → `bb.segmentIn`, ticket 094-006) and
`MOVER` (REPLACE-semantics deadman-velocity segment, stakeholder-designed
OOP follow-on, commit `8306edf6`) are live, alongside re-parsed `D`/`T`/`RT`
and the preserved `S`/`STOP` escape hatch. Bench-verified on the stand via
extended stakeholder teleop sessions (see ticket 094-007 completion notes);
sprint 094 closed at v0.20260709.17. Completion events remain pull-based
(`TLM`), per the sprint's deferred-decision record.

## Verification (bench, on the stand)

- Send the motion command and watch a straight drive, an in-place turn, and a
  translate-then-pivot each execute and stop **gracefully** (no terminal
  reverse-creep — regression vs 093).
- Confirm the command takes effect the next tick (no visible lag) and none are
  dropped over the real link (serial at bench; relay for the radio path).
- `S`/`STOP` direct path still works for bare testing.
