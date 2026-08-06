---
id: '003'
title: Navigator state machine (Motion::Navigator)
status: done
use-cases:
- SUC-001
- SUC-003
- SUC-004
- SUC-005
depends-on:
- '002'
- 008
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

- [x] `Motion::Navigator` constructed with `NavigatorLimits` (ticket 002)
      and holding a reference/pointer to a `Motion::Planner` it drives.
- [x] A goto sent from rest, ticked against a real `Planner` with scripted
      `RobotState` pose updates, converges to within the configured
      arrival tolerance and comes to rest (SUC-001), verified in a ctest
      scenario.
- [x] **Velocity continuity, asserted numerically, not just plausibly**:
      across N re-solves against a stable target, no tick's commanded
      wheel velocity (read via `Planner::commandedLeft()`/
      `commandedRight()` or equivalent) steps by more than the configured
      per-cycle bound — assert this as a hard numeric check across every
      transition in the scenario, mirroring the discontinuity measurement
      `solver.py`'s own module docstring describes
      (`CASE2_EDGE_B_DISCONTINUITY_MM_S = 433.3333` was the FAILURE mode
      being guarded against — this test proves the guard holds).
- [x] Material-change throttle: a scenario with a near-stationary target
      shows the Navigator does NOT replace every single tick (replacing
      every tick would disarm `stallTicks`, `planner.cpp:980-993`) —
      assert the replace count is materially less than the tick count.
- [x] Mandatory half-arc refresh: a scenario where the target doesn't move
      but the in-flight arc is long shows a replace still fires once half
      the arc is consumed, even with no material solution change.
- [x] Target-behind / large-bearing (SUC-004): a scenario with a target
      initially behind the robot exercises stop-then-pivot-then-arc via
      `plannedStop()` + a queued `Kind::Angle` pivot Move, and completes
      without ever handing the Planner an infeasible arc.
- [x] Small-bearing scenario (bearing just under the pivot-first
      threshold): confirms no pivot fires and no oscillation between
      pivot and curvature correction occurs across repeated re-solves
      (the limit-cycle `goto_otos.py`'s own TURN_FIRST constant was
      introduced to avoid).
- [x] OTOS staleness (SUC-005): a single stale-sample cycle (connected
      stays true) shows the in-flight Move continues unchanged that
      cycle — no replace issued.
- [x] OTOS disconnect (SUC-005): a sustained `connected == false` run
      shows the goto aborts within the configured bounded window, with a
      fault-flagged completion ack and Drive ownership released (however
      "Drive ownership" is exposed at this ticket's layer — coordinate
      the exact signal shape with ticket 004, which wires the real
      `drive_.takeover()` call).
- [x] Exactly one completion ack is observed per goto id across every
      scenario above — never zero (silently stuck), never more than one
      (double-ack).
- [x] `NavigatorLimits`' fields from ticket 002 are all genuinely read by
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

## Completion Notes

### Files

- `src/motion/navigator/navigator.h` / `navigator.cpp` — new: `Motion::
  Navigator`, `Motion::GotoTarget`, `Motion::NavResult`, and the
  material-change/half-arc/disconnect/default-tolerance constants
  (`kNavOmegaReplaceThreshold`, `kNavArcLengthReplaceThreshold`,
  `kNavRefreshFraction`, `kNavDefaultArrivalTolerance`,
  `kNavOtosDisconnectAbortWindow`, `kNavSegmentTimeoutMultiplier/Floor/
  Cap`).
- `src/motion/navigator/CMakeLists.txt` — added `navigator.cpp` to the
  `navigator` library, added a new `motion_planner_lib` static library
  (built directly from `planner/profile.cpp`/`shape.cpp`/`estimation.cpp`/
  `planner.cpp`, PUBLIC-linked into `navigator`), and added `navigator_test`
  to `NAVIGATOR_TESTS`.
- `src/motion/navigator/tests/navigator_test.cpp` — new: 9 ctest scenarios,
  one per acceptance criterion (see below).
- `src/motion/navigator/tests/test_support.h` — new: `TestNav::IdealPlant`
  (perfect-velocity wheel tracking + arc-exact world-pose integration,
  publishing both `state.pose` and `state.otos`) and a `cycle()` helper,
  self-contained (no dependency on `planner/tests/test_support.h`).
- `clasi/sprints/135-.../tickets/003-....md` — this file (frontmatter +
  acceptance criteria + these notes).

### Build-target/dependency-linkage approach

Ticket 002 left the choice open between `add_subdirectory(../planner)` and
linking planner's sources directly (mirroring `planner/CMakeLists.txt`'s
own `pose_ownership_test` pattern). Chose the latter: a new
`motion_planner_lib` static library built from planner's four `.cpp`
files directly, with `${CMAKE_CURRENT_SOURCE_DIR}/../planner` and
`../../firm` as its public include dirs, PUBLIC-linked into the
`navigator` library. `add_subdirectory` was rejected because it would
nest a second `project()`/`enable_testing()`/CTest registration inside
this directory's own for no benefit this ticket needed.

### Ownership of `Planner::tick()`/`update()`

`Navigator::tick()` calls `planner_.tick(state)` and `planner_.update
(state)` itself, internally, once per call — it is NOT a "decide only,
let the caller drive Planner separately" module. This is the "one owner"
rule (sprint.md Architecture Overview #3) taken literally: whichever
subsystem owns Drive this cycle is the one thing calling
`planner_.tick()`. Ticket 004's `App::RobotLoop::cycle()` is expected to
call either `navigator_.tick(state)` (a goto owns Drive) or its own direct
`planner_.tick()/update()` pair (a plain MOVE/WHEELS owns Drive) — never
both in the same cycle. Per-cycle causality inside `tick()` is "execute
what was staged last cycle, then decide what to stage for next cycle" —
`moveResult` (this cycle's `TickResult`) reflects the Move `move()`/
`plannedStop()` staged on a *prior* call, since a staged entry only
activates on the Planner's own *next* `tick()`.

### OTOS sign convention

`state.otos.heading` is the wire/hardware-mounted sign (ticket 008); this
file negates it exactly once, at the single point it builds its own
`Pose` from `state.otos` (`pose.heading = -state.otos.heading`), mirroring
`planner.cpp:513`'s reconciliation. This is not a *second*, competing sign
flip on top of ticket 008's fix — it is the same reconciliation applied
independently, because `Motion::Planner` exposes no pose accessor
Navigator could read instead (its public surface has none). `state.otos.x`/
`y` and `state.pose.*` are never sign-flipped (only heading is, per ticket
008's own scope).

### Material-change throttle values (and why)

Ported verbatim from `pathplan.planner.ReplaceThreshold` (`solver.py`'s
host-side loop), NOT re-derived against the 50 ms firmware cadence:
`kNavOmegaReplaceThreshold = 0.05 rad/s`, `kNavArcLengthReplaceThreshold =
15.0 mm`, `kNavRefreshFraction = 0.5`. Unlike `arc_solver.h`'s
`maxWheelStep` (a physical per-solve-period budget that genuinely needed
re-deriving for a different loop rate), these are ordinary,
cadence-independent loop-pacing knobs — `refreshFraction` is dimensionless
by construction, and the other two absorb ordinary per-cycle pose noise/
convergence rather than encoding a rate. Kept as `Navigator`-owned
constants in `navigator.h` (not new `NavigatorLimits` fields), to avoid
touching ticket 002's already-closed, ctypes-mirrored struct shape for
values that are policy, not per-robot-physical config. The half-arc
refresh itself is measured via Euclidean pose displacement since the last
issue (`covered = hypot(pose - issuedAtPose_)`), directly porting
`planner.py:1239-1242`'s own `covered = hypot(currentPose - sentPose)`
mechanism, not a time×speed estimate.

### Pivot omega / per-segment timeout derivations

No new `NavigatorLimits` field for a pivot-specific angular speed:
`pivotOmega() = 2*limits_.speed/limits_.trackWidth` (both wheels driven at
the configured cruise linear speed, differentially) — Planner's own
profiler clamps this to `PlannerLimits::Ceilings::omegaMax` regardless, as
it does for any Move's requested cruise. Per-segment/pivot Move `timeout`
is ported from `pathplan.planner._moveTimeoutFor()`'s derivation (ideal
duration × 3, +3000 ms floor, capped at 20000 ms) — a travel-time budget,
not a per-cycle one, so reused verbatim rather than re-derived for the
different loop rate (matching how `arc_solver.h` reused `behindAngle`/
`turnFirstAngle` verbatim while re-deriving `maxWheelStep`).

### Arrival ("Done") criterion — a correction made during testing

The ticket's own wording ("arrival/rest... declare Done") is satisfied by:
freeze further re-solving once `distance <= target.tolerance` (`frozen_`),
then declare Done once EITHER `TickResult.completed && TickResult.settled`
is observed, OR `Planner::lifecycle() == Idle`. The `&& settled` guard was
added after `testConvergesFromRestAndSettles` first failed: Planner's own
`completed` flag also fires on an OVERSHOOT (`plannedRemaining <= epsilon`
on a signed residual — planner.cpp's own comment), which can land
`completed == true` while the wheels are still commanded well above rest
(measured: ~14 mm/s). Falling back to `lifecycle() == Idle` catches that
case a cycle or two later, once the drain Navigator still services every
cycle (`planner_.tick()/update()` run unconditionally, even while
`frozen_`) reaches zero.

### Drive-ownership-released signal for ticket 004

Exposed as `Navigator::active()` — `bool`, `true` from `start()` until the
exact `tick()` call whose `NavResult::completed` is `true` (or until
`cancel()`), at which point it flips to `false` in the SAME call
(`doneResult()`/`abortResult()` set `phase_ = Phase::Idle` before
returning). Ticket 004 can poll this directly rather than a separate flag;
once false, Navigator has issued its last command to the Planner for this
goto and ownership may be handed elsewhere. `NavResult::fault` distinguishes
Done (`false`) from Aborted (`true`, OTOS-disconnect-timeout or the goto's
own overall timeout) for whatever fault-flagged-ack shape ticket 004's wire
layer needs.

### A test-harness bug found and fixed along the way (not a Navigator bug)

`testTargetBehindStopThenPivotThenArc` initially hung (never converged
within the tick budget). Root cause was in THIS ticket's own test-only
`TestNav::IdealPlant::step()` (`tests/test_support.h`), not in
`Navigator`: its arc-exact world-pose integration used
`arc_solver.cpp`'s own `kMinBearing` threshold (`1e-9`) to decide between
a straight-line step and the `sin(heading+dtheta)-sin(heading))/dtheta`
arc formula. After a 180° pivot lands heading near π, a numerically-zero
but not-exactly-zero omega (~1.3e-7 rad/s, ordinary float noise) gave a
`dtheta` just above that threshold, so the plant took the arc branch —
subtracting two `std::sin()` evaluations of a large argument (~π) that
differ by less than the sine function's own ~1e-7 relative error at that
magnitude, then dividing by the tiny `dtheta` (amplifying the cancellation
noise by ~1e9). Measured effect: the integrator's position update froze
at (numerically) zero every tick despite a genuine 150 mm/s commanded
speed. Fixed by widening the straight-line threshold to `1e-5` rad (still
far below any real commanded turn's per-tick `dtheta`, comfortably above
the float-noise floor) — confirmed via direct instrumentation
(`PLANTDBG`/`NAVDBG` prints, removed before commit) that `Navigator`
itself was issuing correct, sensible Moves throughout; only the test
plant's own integration was wrong. Left a full derivation in the fixed
code's own comment so a future reader doesn't reintroduce the tight
threshold.

### Test results

```
cmake --build src/motion/planner/build --target planner_tests
ctest --test-dir src/motion/planner/build --output-on-failure
  -> 100% tests passed, 8/8 (unaffected by this ticket, confirmed by running)

cmake --build src/motion/navigator/build --target arc_solver_test
  -> builds clean (ticket 002's own test still passes after the
     CMakeLists.txt edit)

cmake --build src/motion/navigator/build --target navigator_tests
ctest --test-dir src/motion/navigator/build --output-on-failure
  -> 100% tests passed, 2/2 (arc_solver_test, navigator_test)
  -> navigator_test: PASS navigator_test (9 checks)
```

The 9 `navigator_test` scenarios, one per acceptance criterion: SUC-001
convergence-and-rest; velocity continuity (numeric per-tick bound);
material-change throttle (replace count << tick count); mandatory
half-arc refresh (an orbiting-target construction where the arc solution
is byte-identical every solve, so only the half-arc mechanism can explain
any replace after the first); SUC-004 target-behind stop-then-pivot-then-
arc (asserts `MoveLifecycle::Stopping` was observed, i.e. `plannedStop()`
genuinely engaged, and eventual fault-free arrival); small-bearing
no-pivot/no-oscillation (asserts `Stopping` is never observed across a
sustained near-threshold bearing); OTOS single-cycle staleness (no
replace); OTOS sustained disconnect (fault-flagged abort within the
bounded window, `disconnectedTicks` in `[20, 25]`); exactly-one-
completion-ack (run 600 ticks past arrival, assert `completions == 1`).
