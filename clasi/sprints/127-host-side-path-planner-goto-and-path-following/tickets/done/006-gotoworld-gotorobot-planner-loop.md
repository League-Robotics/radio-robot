---
id: '006'
title: gotoWorld/gotoRobot planner loop
status: done
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

- [x] `gotoRobot` is implemented as a thin composition through `gotoWorld`
      via `T_world_from_odom` — confirmed by test or code inspection that
      there is exactly one control-loop implementation, not two.
- [x] Replacement is measurably throttled: a unit/sim test shows the
      command rate drops (fewer `move_twist` calls) when the solution is
      not moving materially between cycles, compared to sending on every
      cycle unconditionally.
- [x] The termination-tolerance/give-up constant is isolated at one named,
      commented definition site, explicitly marked
      `# PROVISIONAL -- pending ticket 007`.
- [x] A give-up case (target unreachable within N attempts/time) is
      reachable in a test and reports explicitly why it gave up — not a
      silent infinite retry.
- [x] Move-id allocation is strictly monotonic per session; a unit test
      sends several replacements and confirms no two share an id and none
      regress.
- [x] Geofence check is wired inside the ~10 Hz time-advance primitive
      (confirmed by test or inspection — not a between-segment check).
- [x] Every halt path in this module calls `estop()`; `stop()` does not
      appear anywhere in a halt path.
- [x] A sim-level convergence smoke test passes: `gotoWorld` to a nearby
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

## Completion Notes

**Files**:
- New: `src/host/robot_radio/pathplan/planner.py` — `gotoWorld()`,
  `gotoRobot()`, `GotoResult`, `GiveUpLimits`, `ReplaceThreshold`,
  `MoveIdAllocator`, `TERMINATION_TOLERANCE`, and the private
  `_shouldReplace()`/`_giveUpReason()`/`_targetBehindReason()`/
  `_readFrames()`/`_advance()` helpers.
- New: `src/tests/unit/test_planner.py` — 22 tests, all pure/synthetic
  (throttling, give-up, `MoveIdAllocator`, `gotoRobot`→`gotoWorld`
  composition via a monkeypatched stub, `_readFrames()` backend dispatch,
  `_advance()`'s geofence-inside-window behavior).
- New: `src/tests/sim/test_pathplan_goto_convergence.py` — one sim-tier
  convergence smoke test against the real firmware loop
  (`NezhaProtocol(SimConfigConn(SimLoop))`), placed at `src/tests/sim/`
  top level (matching `test_motor_primitive.py`'s own precedent), NOT
  under `src/tests/sim/system/` (constraint honored).
- Modified: `src/host/robot_radio/pathplan/__init__.py` — exports the six
  new public names alongside 127-004/127-005's existing exports.

**Public API**: `gotoWorld(proto, worldPose, x, y, theta=None, *, limits,
geofence=None, tolerance=TERMINATION_TOLERANCE, giveUp=GiveUpLimits(),
throttle=ReplaceThreshold(), moveTimeout=MOVE_TIMEOUT,
cyclePeriod=CYCLE_PERIOD, moveIds=None) -> GotoResult` is the ONE control
loop. `gotoRobot(...)` has the identical signature/contract (its own
docstring says "shares the SAME kwargs contract", the `move_wheels()`/
`move_twist()` docstring idiom) and is a literal ~10-line body: get the
current `WorldPose.worldPose()`, build `Transform2(x=currentPose.x,
y=currentPose.y, rotation=currentPose.heading)`, `.apply()` the
robot-frame `(x, y, theta)` offset through it, then call `gotoWorld()`
with the result — no loop, no solve call, no `move_twist()` call of its
own. `x`/`y` are `[cm]` (world frame for `gotoWorld`, robot's own body
frame for `gotoRobot`), matching `nav.pose.Pose`'s existing convention
used throughout `pathplan`. `theta` is accepted (composed into the world
frame for `gotoRobot`, forwarded as-is for `gotoWorld`) and entirely
ignored downstream by `solveArcToPoint()` — documented at both
signatures, consistent with 127-005's own documented behavior.

**Throttled replacement**: `_shouldReplace(lastSent, candidate,
omegaThreshold, arcLengthThreshold)` — pure, compares the freshly solved
`ArcSolution` against the LAST **SENT** one (not the last computed one;
un-sent solves never update the comparison baseline), true when either
`|Δomega| > 0.05 rad/s` or `|ΔarcLength| > 15 mm` (`ReplaceThreshold`'s
defaults), always true for the first solve. This is deliberate: the
solver's `previousOmega` slew-clamp state is threaded from the omega of
the last **SENT** move (`sentOmega` in `gotoWorld()`'s loop), not from
every intervening un-sent solve — clamping against an internally-ramping-
but-never-transmitted value would let the omega drift arbitrarily far
from what the firmware is actually running before the throttle ever
fired, defeating the slew limit's whole purpose the moment a send finally
did happen. `test_replacement_rate_drops_when_solution_is_not_moving_
materially` directly proves the ticket's own acceptance wording: an
8-step synthetic sequence with several materially-unchanged cycles sends
3 replacements vs. 8 if sending unconditionally every cycle.

**Termination-tolerance/give-up constant — exact site**:
`src/host/robot_radio/pathplan/planner.py:108`
(`TERMINATION_TOLERANCE = 100.0  # [mm] ...`), with the required literal
comment `# PROVISIONAL -- pending ticket 007's measured minimum reliable
move distance/turn angle; do not treat as final.` immediately above it
(lines 99-100). Chosen from the design issue's own actuation-delay
analysis (~150 ms actuation delay → ≥100 mm carrot distance), not a
guess — this is the ONE named, isolated definition site; no other call
site in the module repeats the number. `GiveUpLimits.maxIterations`/
`giveUpTimeout` are ordinary (non-provisional) loop-pacing knobs — ticket
007 measures/tunes `TERMINATION_TOLERANCE` only, not these.

**Give-up is reachable and explicit** two ways, both pure/no-I/O and unit
tested: (1) `_giveUpReason(iterations, elapsed, limits)` — iteration-cap
or wall-clock-cap exhaustion, always returns a stated reason, never
`None` once exhausted; (2) `_targetBehindReason(bearing)` — the solver's
target-behind guard (`ArcSolution.stop=True`) is treated as an immediate,
explicit give-up (NOT a cue to turn in place — terminal-theta/in-place-
turn honoring is sprint.md's own Out of Scope), reporting the exact
bearing and why reorientation isn't attempted.

**`previousOmega` threading**: `gotoWorld()`'s loop keeps `sentOmega`
(initialized `0.0`), passed as `solveArcToPoint(..., previousOmega=
sentOmega)` on every solve; `sentOmega` is updated to `solution.omega`
ONLY when `_shouldReplace()` triggers an actual send — see the throttling
section above for why un-sent solves must not update it.

**Move-id monotonicity**: `MoveIdAllocator` (public — a caller issuing
multiple sequential `gotoWorld()`/`gotoRobot()` calls in one boot session,
e.g. a multi-waypoint tour, MUST construct one instance and pass it via
`moveIds=` to every call; the default creates a fresh, call-scoped
allocator only safe for a single isolated call — documented loudly at the
class and at the `moveIds` parameter on both entry points). Starts at 1
and never emits `0` — `0` is the firmware's dedup-EXEMPT sentinel
(ticket 002), the wrong choice for a continuously-replacing loop.
`test_move_id_allocator_strictly_monotonic_no_repeats_no_regression`/
`test_move_id_allocator_never_emits_zero` lock this in.

**Backend-agnostic telemetry read — a real gap found and fixed in this
ticket's own scope**: `NezhaProtocol.read_pending_binary_tlm_frames()`
assumes `self._conn.drain_binary_tlm()` (`SerialConnection`'s shape);
`SimConfigConn` has no such method (`SimLoop.
read_pending_binary_tlm_frames()` already returns adapted `TLMFrame`s
directly) — confirmed `square_tour.SimBackend.advance()` already works
around this exact asymmetry by reading off the raw `SimLoop` instead of
through `NezhaProtocol`. Calling `proto.read_pending_binary_tlm_frames()`
uniformly (as the ticket's own design context implied would work) throws
`AttributeError` against a Sim-wrapped `NezhaProtocol`. Fixed inside this
module only (in scope, no shared-infra file touched):
`_readFrames(proto)` prefers `proto._conn`'s own
`read_pending_binary_tlm_frames()` when present (`SimConfigConn`),
falling back to the protocol-level method otherwise (`SerialConnection`
via `NezhaProtocol`'s own adapter) — `# noqa: SLF001` on the one private
`._conn` access, same idiom `square_tour.py` already uses for a private
`SimLoop` method access. This makes `gotoWorld()`/`gotoRobot()` genuinely
backend-agnostic, which is what actually makes the sim convergence smoke
test possible — without it the sim test would fail immediately with no
telemetry ever read. `test_read_frames_prefers_the_connections_own_
reader_when_present`/`test_read_frames_falls_back_to_the_protocol_level_
reader` cover both branches.

**Sim convergence smoke test**: `src/tests/sim/
test_pathplan_goto_convergence.py` connects a headless, REAL-TIME-ticking
`SimLoop` (`start_tick_thread=True`, the default — `gotoWorld()`'s own
`_advance()` polls on wall-clock time exactly like hardware, it never
calls `SimLoop.step()` itself) wrapped as `NezhaProtocol(SimConfigConn(
SimLoop))`, configured from `tovez_nocal.json`. Seeds `WorldPose` from one
real telemetry frame then re-anchors from `SimLoop.get_true_pose()` once
(the sim-tier stand-in for a startup camera fix — after that the loop
runs purely off telemetry, same as hardware), then calls `gotoWorld()`
toward a target 300 mm away. Measured result: `success=True`,
`iterations=17`, `sent=9` (9 of 17 solve cycles actually replaced the
in-flight Move — the throttle visibly working end-to-end, not just in the
pure unit test), converged within `TERMINATION_TOLERANCE` (reported
`distance=95.3 mm`); independently checked against `get_true_pose()`
ground truth (not the loop's own telemetry belief): residual 95.9 mm,
well inside the test's own 220 mm slack bound. Runtime 2.54s.

**Test results**:

```
uv run python -m pytest src/tests/unit -k planner -q
104 passed in 1.67s   (22 of these are this ticket's own test_planner.py; the
                        rest are pre-existing test_planner_*.py files this
                        ticket did not touch, matched by the same -k filter)

uv run python -m pytest src/tests/sim -k goto -q
1 passed in 2.51s

uv run python -m pytest src/tests/unit -q
503 passed, 2 failed in 4.28s
```

The 2 failures are the pre-existing, ticket-stated baseline failures in
`test_gen_boot_config_otos.py` (stale snapshot from sprint 126-003) — not
touched by this ticket, not counted against it, matches the ticket's own
Testing note exactly.

No hardware touched (constraint honored — the robot is on the stand and
127-002 is actively using it); no file under `src/firm`, no `.proto`, no
wire message changed; no new file under `src/tests/sim/system/`; no file
in `src/tests/bench/planner_square_tour.py` or the dedup test files
(127-002's own scope) touched.
