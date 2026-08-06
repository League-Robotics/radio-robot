---
id: '003'
title: Navigator state machine (Motion::Navigator)
status: open
use-cases:
- SUC-001
- SUC-003
- SUC-004
- SUC-005
depends-on:
- '002'
- '008'
github-issue: ''
issue: replaceable-go-to-moves-in-the-motion-library.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Navigator state machine (Motion::Navigator)

## Description

Build `Motion::Navigator` (`src/motion/navigator/navigator.{h,cpp}`): the
state machine that drives one goto target to completion by repeatedly
calling `Motion::ArcSolver` (ticket 002) and issuing internal
`Planner::move(next, replace=true)` calls into a real `Motion::Planner`.
This is the navigation-policy module — it owns the current goal, decides
each cycle whether to re-solve/replace/hold/pivot/stop, and owns the
one-completion-ack-per-goto contract. It does NOT parse the wire (ticket
004) and does NOT solve arc geometry itself (delegates to `ArcSolver`,
ticket 002).

**Dependency boundary**: `Navigator` depends on `Motion::Planner` (its
public surface — `move()`, `plannedStop()`, `tick()`'s `TickResult`) and
`Types::RobotState` (for OTOS pose) — no other `src/firm` dependency,
matching `planner/`'s own narrower dependency rule (`src/motion/DESIGN.md`
§3: exactly one `src/firm` header, `types/robot_state.h`). Do not reach
into `App::*`, `Devices::*`, or any bus/timing collaborator — the whole
point of this tree is hand-fed-numbers testability with no hardware.

**The internal Move this issues is an ORDINARY Move, not a new kind** (see
ticket 002's Description for the full field mapping — repeated here since
you'll be constructing these directly):

```cpp
// src/motion/planner/planner_types.h -- Motion::Move, already exists:
Move m;
m.kind = Move::Kind::Distance;              // NOT a new "Arc" kind
m.velocityKind = Move::VelocityKind::Twist;
m.threshold = solution.arcLength;           // [mm]
m.v_x = solution.v_x;                       // [mm/s] signed
m.omega = solution.omega;                   // [rad/s] signed
m.timeout = /* per-segment safety backstop */;  // [ms]
m.id = 0;                                   // internal segment
planner.move(m, /*replace=*/true);
```

For the stop-then-pivot sequence (SUC-004), the pivot itself is also an
ordinary Move (`Kind::Angle`, `VelocityKind::Twist`, `omega` set,
`v_x == 0`) queued behind a `plannedStop()` entry, not a new Move shape —
`Planner::plannedStop()` already exists (`planner.cpp`) and sequences
brake→rest→next-entry cleanly; use it rather than hand-rolling a stop.

## Sign convention for scripted RobotState OTOS fixtures

This ticket's ctests script `Types::RobotState` OTOS values by hand
(construct a pose sequence, feed it to `Navigator`/`Planner`, assert on
the result) — depends on ticket 008, which is the ONE place this sprint
settles which heading sign is correct. **Script your `RobotState.otos.
heading` fixtures using the HARDWARE-mounted sign convention** (the sign
`planner.cpp:513`'s existing negation expects, i.e. optical heading as
the real OTOS chip reports it, opposite the encoder-derived sign on a
positive rotation) — NOT the sim's pre-008 encoder-derived sign, which
was the wrong convention ticket 008 fixes. Ticket 008's own Completion
Notes are the authoritative citation for this convention once it lands;
if this ticket starts before 008 completes, coordinate rather than
guessing — a Navigator ctest scripted against the wrong sign will pass
for the wrong reason (mirroring exactly the bug ticket 008 fixes) and is
worse than no test.

## State machine

Idle → (Pivot | Cruise) → FineApproach → Done/Aborted, per sprint.md's
Architecture. Concretely, each 50 ms cycle while a target is set:

1. Read `state.otos.{x,y,heading,connected}`. If stale (no fresh sample
   this window but still `connected`), skip re-solving this cycle and let
   whatever Move is already in flight continue — do not replace blind.
   If `connected` is false, switch to bounded `state.pose`-delta
   propagation (SUC-005) and start a disconnect timer.
2. Otherwise call `ArcSolver::solve(pose, target, limits, previousOmega)`.
3. If the solution's bearing exceeds the pivot-first threshold, or the
   solver reports `stop` (target behind): if currently moving, issue
   `plannedStop()` then a queued pivot Move; once at rest, issue the pivot;
   once the pivot lands, fall through to step 4 on the next cycle
   (SUC-004).
4. Otherwise: if the new solution differs materially from the last one
   issued (material-change throttle — port the spirit of `solver.py`'s
   `ReplaceThreshold`: curvature/arc-length deltas past a threshold, PLUS
   a mandatory refresh once half the in-flight arc is consumed), issue
   `planner_.move(m, replace=true)`. Otherwise do nothing this cycle — let
   the already-issued Move continue; replacing every tick would disarm
   the Planner's own stall backstop (`activateNext()` resets
   `stallTicks`, `planner.cpp:980-993`) and fight its arrival detection.
5. Consume `tick()`'s `TickResult` each cycle. When the in-flight Move's
   own arrival/rest lands within the target's configured tolerance (or
   when this WAS the terminal segment and it completed), declare the goto
   Done and emit exactly one completion ack keyed on the goto's id.
6. On disconnect timeout (step 1) or on the goto's own overall `timeout`
   elapsing, declare Aborted: zero the Move, emit a fault-flagged
   completion ack, release Drive ownership.

## Acceptance Criteria

- [ ] `Motion::Navigator` constructed with `NavigatorLimits` (ticket 002)
      and holding a reference/pointer to a `Motion::Planner` it drives.
- [ ] A goto sent from rest, ticked against a real `Planner` with scripted
      `RobotState` pose updates, converges to within the configured
      arrival tolerance and comes to rest (SUC-001), verified in a ctest
      scenario.
- [ ] **Velocity continuity, asserted numerically, not just plausibly**:
      across N re-solves against a stable target, no tick's commanded
      wheel velocity (read via `Planner::commandedLeft()`/
      `commandedRight()` or equivalent) steps by more than the configured
      per-cycle bound — assert this as a hard numeric check across every
      transition in the scenario, mirroring the discontinuity measurement
      `solver.py`'s own module docstring describes
      (`CASE2_EDGE_B_DISCONTINUITY_MM_S = 433.3333` was the FAILURE mode
      being guarded against — this test proves the guard holds).
- [ ] Material-change throttle: a scenario with a near-stationary target
      shows the Navigator does NOT replace every single tick (replacing
      every tick would disarm `stallTicks`, `planner.cpp:980-993`) —
      assert the replace count is materially less than the tick count.
- [ ] Mandatory half-arc refresh: a scenario where the target doesn't move
      but the in-flight arc is long shows a replace still fires once half
      the arc is consumed, even with no material solution change.
- [ ] Target-behind / large-bearing (SUC-004): a scenario with a target
      initially behind the robot exercises stop-then-pivot-then-arc via
      `plannedStop()` + a queued `Kind::Angle` pivot Move, and completes
      without ever handing the Planner an infeasible arc.
- [ ] Small-bearing scenario (bearing just under the pivot-first
      threshold): confirms no pivot fires and no oscillation between
      pivot and curvature correction occurs across repeated re-solves
      (the limit-cycle `goto_otos.py`'s own TURN_FIRST constant was
      introduced to avoid).
- [ ] OTOS staleness (SUC-005): a single stale-sample cycle (connected
      stays true) shows the in-flight Move continues unchanged that
      cycle — no replace issued.
- [ ] OTOS disconnect (SUC-005): a sustained `connected == false` run
      shows the goto aborts within the configured bounded window, with a
      fault-flagged completion ack and Drive ownership released (however
      "Drive ownership" is exposed at this ticket's layer — coordinate
      the exact signal shape with ticket 004, which wires the real
      `drive_.takeover()` call).
- [ ] Exactly one completion ack is observed per goto id across every
      scenario above — never zero (silently stuck), never more than one
      (double-ack).
- [ ] `NavigatorLimits`' fields from ticket 002 are all genuinely read by
      at least one code path here (no orphaned config fields — the
      project's own recurring failure mode per `src/motion/DESIGN.md`'s
      "Wheel control generations" history of dead config fields).

## Build-list touch points

Same as ticket 002: this ticket adds `src/motion/navigator/navigator.cpp`
(and `.h`) to whichever standalone CMake target ticket 002 chose. Root
`CMakeLists.txt` needs no edit (glob-based, see ticket 002). `src/sim/
CMakeLists.txt`'s `MOTION_SOURCES` and the `src/tests/sim/*` pytest
`_APP_SOURCES` lists still don't need this file yet — they're needed once
`RobotLoop` actually links `Navigator` (ticket 004). Don't add it early;
match what ticket 004 actually needs when it lands.

## Testing

- **Existing tests to run**:
  ```
  cmake -S src/motion/planner -B src/motion/planner/build
  cmake --build src/motion/planner/build --target planner_tests
  ```
  (confirms this ticket's new caller of `Planner::move()` hasn't disturbed
  the Planner's own test suite — it shouldn't, since `Navigator` only
  calls existing public methods).
- **New tests to write**: the Navigator+Planner scenario ctests described
  in Acceptance Criteria above — construct a real `Motion::Planner` and a
  hand-scripted `Types::RobotState` sequence (pose updates simulating an
  idealized or lightly-noisy plant), tick both every 50 ms, assert on the
  resulting `TickResult`s and commanded velocities.
- **Verification command**: whichever `ctest` target ticket 002 set up,
  e.g.
  ```
  cmake --build src/motion/navigator/build --target navigator_tests
  ctest --test-dir src/motion/navigator/build --output-on-failure
  ```
