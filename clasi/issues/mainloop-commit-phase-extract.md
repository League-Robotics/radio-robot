---
status: pending
sprint: 090
---

# Extract the COMMIT block into a private MainLoop::commit(bb, now)

Extracted from `sprint-089-omnibus.md` entry 5 (stakeholder design discussion 2026-07-07).

## Motivation

`MainLoop::tick` should read as a sequence of named phases, not a wall of
state-copy plumbing. The COMMIT block (`main_loop.cpp:262-276`) is verbose and
buries `tick()`'s phase structure.

## Direction — a private MainLoop phase method, NOT a Blackboard API

Extract to a private **`MainLoop::commit(bb, now)`**, exactly matching the
precedent just landed on this branch: `serviceWatchdogs()` (commit `0b2929c5`,
"extract MainLoop's watchdog block"). `tick()` then reads as phases —
`serviceWatchdogs → control → plan → commit → routeOutputs`.

`MainLoop` (the composition root) keeps owning the wiring and the commit
ordering; the block is one function-open away for anyone debugging
clock-edge/x[k] timing.

## Rejected: `Blackboard::update(drivetrain, poseEstimator, planner, odometer, hardware)`

The omnibus originally proposed a `Blackboard::update(...)` taking references to
every subsystem. Rejected:

- **Dependency inversion.** `Blackboard` is a dumb DTO — the data flowing
  between planes; today it depends on nobody. `update(all subsystems)` makes it
  depend on the entire subsystem graph (drivetrain, planner, pose-estimator,
  hardware, odometer), on a struct whose job is to *not* know them. Subsystems
  already read `bb`; this makes `bb` read subsystems — a coupling knot and cycle
  risk.
- **Execution inside a data struct.** The block is not pure copying —
  `odometer->tick(now)` (line 271) is a side-effecting tick. `bb.update()` would
  drive subsystem execution from inside the blackboard.
- **Hides the clock edge.** COMMIT is the x[k]→x[k+1] transition that makes the
  two-plane ordered-tick model work — the block you most want visible/owned by
  the composition root, not buried in a DTO method.

## The block shrinks for free once neighbors land

Independent of this extraction, COMMIT gets shorter once the sibling issues land:
- [[drivetrain-owns-motor-observation-resolution]] moves the per-port index loop
  into the `bb.motors` resolution.
- [[null-odometer-object]] + [[odometer-owns-reset-and-fusability]] turn the
  `odometer != nullptr` commit branch into `bb.otosValid =
  odometer->fusableThisPass()`.

So `commit()` ends up a tidy handful of lines, not the current block.

## Scope

- `source/runtime/main_loop.{h,cpp}` (new private `commit(bb, now)`; call it from
  `tick()` where the inline block is now)
