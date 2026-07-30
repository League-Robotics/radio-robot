---
id: '006'
title: gotoWorld/gotoRobot planner loop
status: open
use-cases:
- SUC-006
depends-on:
- '003'
- '004'
- '005'
github-issue: ''
issue: sprint-127-host-side-path-planner-goto-path-following.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# gotoWorld/gotoRobot planner loop

## Description

Design issue T5. The outer position loop: `gotoWorld`/`gotoRobot` as one
implementation with two entry points (`gotoRobot` composes through
`gotoWorld` via `T_world_from_odom`, not a second implementation). Each
cycle: read telemetry → update `WorldPose` (ticket 004) → solve (ticket
005's `solveArcToPoint`) → issue `move_twist(...,
stop_distance=arcLength, replace=True)` when the solution has moved
materially (throttled — cuts link traffic and command churn, does not
re-send every cycle regardless of whether anything changed).

**Explicit termination, not an infinite null-the-error loop**: a
tolerance and a give-up rule. Below the drivetrain deadband, commands get
zeroed and the robot stalls (the failure already fixed once via the
`copysign(deadband)` boost — do not reproduce it here by commanding an
unreachably tight tolerance).

**Decision 4's provisional-constant contract (sprint.md Architecture,
amended)**: the minimum reliable move distance/turn angle that should set
this loop's termination tolerance is **unmeasured at the time this ticket
is implemented** — ticket 007 measures it, after this loop exists to be
measured against (007 depends on 006, not the reverse; the measurement
can only be taken once a real `gotoWorld` loop exists to attempt short
moves under its own throttling/termination logic). Ship a **provisional**
termination-tolerance/give-up constant now:

- Isolate it as **one named value, at one clearly identifiable
  definition site** (e.g. a single module-level constant in
  `pathplan/planner.py`, not scattered across multiple call sites).
- Mark it explicitly in a comment at that site:
  `# PROVISIONAL -- pending ticket 007's measured minimum reliable move
  distance/turn angle; do not treat as final.`
- Pick a conservative starting value from the actuation-delay analysis
  already in the design issue (carrot distance ≥100 mm at the ~150 ms
  actuation delay) rather than an arbitrary guess.
- Ticket 007 updates this exact constant with its measured value as part
  of its own acceptance — this ticket does not need to "finish" the
  tuning, only make it obviously findable and updatable.

**Geofence and halts**: geofence checked **inside** the ~10 Hz
time-advance primitive (the existing `drainFrames(proto, seconds,
geofence)` idiom `square_tour.py`/other bench scripts already use), never
between segments. Every halt path calls `estop()`, never `stop()`.
Backend split copied from `square_tour.py`'s `_Backend` so the same loop
runs against `SimLoop` and hardware unchanged.

**Move-id monotonicity** (Architecture Open Question 2): the planner must
issue a fresh, strictly monotonic `Move.id` on every replacement within a
session — `handleMove()` short-circuits on a duplicate id **before**
honoring `replace` (confirmed, `robot_loop.cpp:216-225` per the design
issue), so a planner that reuses or fails to advance ids would have its
own replacements silently vanish while still acking OK. This is this
ticket's responsibility to implement correctly, not an incidental detail.

This ticket's own acceptance is **unit + sim only** — full hardware/
playfield convergence proof is ticket 007's scope. Do not run a bench or
playfield session as part of this ticket; a sim-level convergence smoke
(one `gotoWorld` call to a nearby target converges against `SimLoop`) is
sufficient here.

**Files**:
- New: `src/host/robot_radio/pathplan/planner.py` — `gotoWorld`,
  `gotoRobot`, the throttling/termination logic, the geofence-inside-
  advance wiring, the move-id counter.
- Depends on ticket 003 (`field.Geofence`), ticket 004 (`WorldPose`),
  ticket 005 (`solveArcToPoint`) — compose, do not reimplement any of
  their logic.

**Coding standards**: no units in identifiers (a give-up timeout is
`giveUpTimeout` with a `# [s]` tag, not `giveUpTimeoutS`).
lowerCamelCase functions/variables (`gotoWorld`, `gotoRobot`,
`nextMoveId`), UpperCamelCase types.

## Acceptance Criteria

- [ ] `gotoRobot` is implemented as a thin composition through `gotoWorld`
      via `T_world_from_odom` — confirmed by test or code inspection that
      there is exactly one control-loop implementation, not two.
- [ ] Replacement is measurably throttled: a unit/sim test shows the
      command rate drops (fewer `move_twist` calls) when the solution is
      not moving materially between cycles, compared to sending on every
      cycle unconditionally.
- [ ] The termination-tolerance/give-up constant is isolated at one named,
      commented definition site, explicitly marked
      `# PROVISIONAL -- pending ticket 007`.
- [ ] A give-up case (target unreachable within N attempts/time) is
      reachable in a test and reports explicitly why it gave up — not a
      silent infinite retry.
- [ ] Move-id allocation is strictly monotonic per session; a unit test
      sends several replacements and confirms no two share an id and none
      regress.
- [ ] Geofence check is wired inside the ~10 Hz time-advance primitive
      (confirmed by test or inspection — not a between-segment check).
- [ ] Every halt path in this module calls `estop()`; `stop()` does not
      appear anywhere in a halt path.
- [ ] A sim-level convergence smoke test passes: `gotoWorld` to a nearby
      target converges against `SimLoop.get_true_pose()` within a stated
      tolerance.

## Testing

- **Existing tests to run**: `uv run python -m pytest src/tests/sim -q`
  (confirm no regression); tickets 003/004/005's own tests still pass.
- **New tests to write**: unit tests for throttling logic, termination/
  give-up logic, and move-id monotonicity (all pure, no I/O); one sim-tier
  convergence smoke test against `SimLoop`.
- **Verification command**:
  `uv run python -m pytest src/tests/unit -k planner -q` and
  `just build-sim && uv run python -m pytest src/tests/sim -k goto -q`
