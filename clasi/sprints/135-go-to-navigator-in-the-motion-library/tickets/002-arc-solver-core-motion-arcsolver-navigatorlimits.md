---
id: '002'
title: Arc-solver core (Motion::ArcSolver, NavigatorLimits)
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-004
depends-on: []
github-issue: ''
issue: replaceable-go-to-moves-in-the-motion-library.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Arc-solver core (Motion::ArcSolver, NavigatorLimits)

## Description

Port the host's `solveArcToPoint` (`src/host/robot_radio/pathplan/
solver.py`) into pure C++: `Motion::ArcSolver`
(`src/motion/navigator/arc_solver.{h,cpp}`). This is geometry only — given
a current pose and a target point, compute the one tangent arc through the
target (curvature, arc length, cruise velocity), with a curvature slew
clamp and a target-behind guard — no navigation policy, no Planner call,
no wire awareness. `Motion::Navigator` (ticket 003) is the only caller.

**This ticket does NOT touch `Motion::Move` or `PlannerLimits`, and does
NOT add a new `Move::Kind`.** That was an explicitly rejected design
(sprint.md Architecture > Design Rationale, Decision 1) — the Planner's
discrete-exact-landing proof assumes a fixed left:right ratio captured
once at Move activation, and a per-tick re-solved curvature inside the
Planner would make the ratio-handoff hazard the steady state. The arc this
solver produces becomes, in ticket 003, an ORDINARY Move using fields that
already exist in `src/motion/planner/planner_types.h`:

```cpp
Move m;
m.kind = Move::Kind::Distance;          // existing enumerator
m.velocityKind = Move::VelocityKind::Twist;  // existing enumerator
m.threshold = arcLength;                // [mm] existing field
m.v_x = cruiseVelocity;                 // [mm/s] existing field, signed
m.omega = cruiseOmega;                  // [rad/s] existing field, signed;
                                         // curvature = omega / v_x
m.timeout = safetyBackstop;             // [ms] existing field
m.id = 0;                               // internal segment -- ticket 004
                                         // must keep this from reaching
                                         // the ordinary completion-ack path
```

Verify at completion that `git diff` touches nothing in
`planner_types.h` — if it does, the design has drifted from what
architecture review approved and that is a stop-and-flag, not a
judgment call to push through.

## NavigatorLimits

Define a plain, motion-owned `NavigatorLimits` struct (same convention as
`PlannerLimits`: no wire types, trivially copyable, constructible with
hand-fed numbers) covering at minimum what `solver.py`'s own
`SolverLimits` class covers — read that class before designing this one,
it is the spec:

- the behind-guard angle (~90°, no tangent arc exists beyond it)
- the pivot-first threshold (TURN_FIRST, ~50° in `goto_otos.py` — carry
  this as a configurable limit, not a hardcoded constant, per
  configuration-discipline)
- the omega slew-rate limit (the curvature-clamp derivation in
  `solver.py`'s own module docstring has the full reasoning — read it,
  the bound exists specifically to keep a `replace=true` transition from
  landing as a step discontinuity)
- approach-speed taper parameters (near-target slowdown — tracking
  accuracy, not feasibility; `shapeLimits()` already handles feasibility)

This ticket defines the struct's shape; ticket 004 is what wires it to the
robot JSON / bake path. Don't invent config plumbing here — just the
struct `ArcSolver` is constructed with.

## Build-list touch points for this ticket's new files

Two new files this ticket adds:
`src/motion/navigator/arc_solver.h`, `src/motion/navigator/arc_solver.cpp`.

**Verified directly against the current build files at planning time —
don't assume, re-verify at ticket time since the tree moves:**

1. **The ARM firmware build (root `CMakeLists.txt`) needs NO edit.** It
   uses `RECURSIVE_FIND_FILE(MOTION_SOURCE_FILES ... "*.cpp")` — a glob
   over all of `src/motion`, excluding only `/planner/(tests|bench|py|
   build)/` and `/planner/capi.cpp` (CMakeLists.txt:333-360). A new file
   under `src/motion/navigator/` is picked up automatically. If you find
   yourself editing this file for this ticket, that's a signal something
   is misplaced, not a step you're expected to take.
2. **This ticket's own standalone ctest build.** Decide (this ticket's own
   call — sprint.md Architecture's Open Question 2): either a new sibling
   `src/motion/navigator/CMakeLists.txt` (mirroring `planner/`'s own
   standalone-project pattern — its header comment at the top of
   `src/motion/planner/CMakeLists.txt` is the template to follow), or add
   `arc_solver.cpp` to the existing `src/motion/CMakeLists.txt`
   `motion_tests` target. Either satisfies the "fast, Python-free
   iteration loop" contract this tree exists for — record which you chose
   and why in this ticket's Completion Notes, since ticket 003 needs to
   know where to add `navigator.cpp` next to it.
3. **`src/sim/CMakeLists.txt`'s `MOTION_SOURCES` list** (currently
   `body_kinematics.cpp`, `odometry.cpp`, and the four `planner/*.cpp`
   files, ~line 135-142) — only needs `arc_solver.cpp` added once
   something in the sim/ARM build actually calls it, which is ticket 004
   (`RobotLoop` wiring), not this ticket. Note it here so it isn't
   forgotten when that ticket lands; this ticket's own ctest doesn't
   require it.
4. **The `src/tests/sim/` pytest `_APP_SOURCES` lists** — same timing as
   #3: only needed once `RobotLoop` actually links `Navigator`/
   `ArcSolver` (ticket 004). Re-derive the current file list fresh at that
   point with `grep -rln "planner.cpp" src/tests/sim/` — do not trust a
   number written down during sprint planning; the tree moves. (22 files
   matched as of this sprint's planning pass — treat that as a floor to
   verify against, not a ceiling to stop at.)

## Acceptance Criteria

- [x] `Motion::ArcSolver` (`src/motion/navigator/arc_solver.{h,cpp}`)
      exposes a `solve(pose, target, limits, previousOmega)`-shaped
      function returning an `ArcSolution` (curvature/arc length/cruise
      v_x/omega/a `stop` flag) — the same conceptual surface as
      `solver.py`'s `solveArcToPoint`/`ArcSolution`.
- [x] The curvature slew clamp (`solver.py`'s `_clampOmegaStep`) is
      ported: `omega` never steps between successive calls, only ramps,
      bounded by `NavigatorLimits`.
- [x] The target-behind guard is ported: a target whose body-frame bearing
      magnitude approaches ~180° returns `ArcSolution.stop = true` rather
      than an extreme, physically-unrealizable arc.
- [x] `NavigatorLimits` is defined as a plain, motion-owned struct (no
      wire types), covering behind-angle, TURN_FIRST-equivalent pivot
      threshold, omega slew rate, and approach-taper parameters.
- [x] A standalone ctest exists and is wired into a real `ctest` target
      (per the build-list decision above) mirroring `solver.py`'s own test
      surface closely enough to be a parity check, not just a smoke test —
      same representative poses/targets, comparable tolerances.
- [x] `git diff` at completion shows zero changes to
      `src/motion/planner/planner_types.h` or any other `planner/` file —
      `ArcSolver` is purely additive and standalone.
- [x] Root `CMakeLists.txt` is NOT edited (per touch-point #1 above); if a
      change there seemed necessary, that's flagged and explained in
      Completion Notes rather than silently made.

## Testing

- **Existing tests to run**: `planner_tests` should still pass unchanged
  (this ticket adds no dependency on or change to `planner/`):
  ```
  cmake -S src/motion/planner -B src/motion/planner/build
  cmake --build src/motion/planner/build --target planner_tests
  ```
- **New tests to write**: the arc-solver parity ctest described above.
- **Verification command** (adjust target name to whichever build-list
  choice was made):
  ```
  # if a sibling src/motion/navigator/CMakeLists.txt project:
  cmake -S src/motion/navigator -B src/motion/navigator/build
  cmake --build src/motion/navigator/build --target navigator_tests
  ctest --test-dir src/motion/navigator/build --output-on-failure

  # if folded into the existing motion_tests target instead:
  cmake -S src/motion -B src/motion/build --target motion_tests
  cmake --build src/motion/build --target motion_tests
  ctest --test-dir src/motion/build --output-on-failure
  ```

## Completion Notes

**Files added** (all new, nothing else touched):
- `src/motion/navigator/arc_solver.h` — `Motion::Pose`, `Motion::
  NavigatorLimits`, `Motion::ArcSolution`, `Motion::ArcSolver::solve()`.
- `src/motion/navigator/arc_solver.cpp` — the implementation.
- `src/motion/navigator/CMakeLists.txt` — the standalone build (see below).
- `src/motion/navigator/tests/arc_solver_test.cpp` — the parity ctest.

**Build-list decision (Open Question 2): a new sibling
`src/motion/navigator/CMakeLists.txt`**, mirroring `planner/CMakeLists.txt`'s
own standalone-project pattern (`project(motion_navigator CXX)`, a static
`navigator` library, a `NAVIGATOR_TESTS` foreach loop, a `navigator_tests`
custom target running `ctest`). Chosen over folding `arc_solver.cpp` into
`src/motion/CMakeLists.txt`'s existing `motion_tests` target because
**ticket 003's `Motion::Navigator`**, the very next file to land in this same
directory, depends on `Motion::Planner::move()`/`TickResult` — a dependency
`motion_tests` deliberately does NOT carry (`src/motion/DESIGN.md` §4:
"Separate from, and does not depend on, `planner/`'s own CMake project").
Putting `navigator/` in its own standalone project now means ticket 003 can
add that `Motion::Planner` dependency (e.g. `add_subdirectory` on
`planner/`, or linking its sources directly the way `planner/`'s own
`pose_ownership_test` target already does for `odometry.cpp`/
`body_kinematics.cpp`) without disturbing `motion_tests`' own narrower,
already-documented scope. THIS ticket's own `arc_solver.cpp` has zero
dependency on `Motion::Planner` or `Types::RobotState` — no `src/firm`
include root was needed, unlike `planner/CMakeLists.txt`'s own (which adds
`src/firm` for the one sanctioned `types/robot_state.h` include).
**Ticket 003: add `navigator.{h,cpp}` next to `arc_solver.cpp` in this same
`src/motion/navigator/CMakeLists.txt` project`, adding whatever `Motion::
Planner` linkage it needs.**

**`maxWheelStep` default re-derived, not copied.** `solver.py`'s own
`MAX_WHEEL_STEP = 250.0 mm/s` is derived against the HOST planner loop's
100 ms re-solve cadence (`_SLEW_ACCEL * _SOLVE_PERIOD = 2500.0 * 0.1`). This
port's caller (`Motion::Navigator`, ticket 003) re-solves on the
**firmware's own 50 ms cycle** instead (sprint 135 `sprint.md` Solution:
"re-solving a tangent arc every 50 ms cycle"; confirmed against
`App::RobotLoop::kCycle = 50` in `src/firm/app/robot_loop.h`). Re-deriving
against the loop that actually spends the budget — exactly the warning
`solver.py`'s own docstring gives for its `_SOLVE_PERIOD` constant — gives
`kArcSolverMaxWheelStepDefault = 2500.0 * 0.05 = 125.0 mm/s` per solve, not
`250.0`. Both the constants and the full derivation prose live in
`arc_solver.h` (not `arc_solver.cpp`) so they are publicly visible to the
parity ctest, matching how `solver.py`'s own `MAX_WHEEL_STEP` is a public,
importable module constant its own `test_max_wheel_step_derivation` locks
against.

**`NavigatorLimits` fields beyond what `solve()` itself reads.** Per the
ticket body, `NavigatorLimits` also carries `turnFirstAngle` (goto_otos.py's
`TURN_FIRST`, ~50°) and `approachRadius`/`approachSpeed` (near-target speed
taper) even though `ArcSolver::solve()` does not consult `turnFirstAngle` at
all (that is `Motion::Navigator`'s own pivot-policy decision, ticket 003) —
both live on this one struct because `NavigatorLimits` is a single
configuration group (one robot-JSON block, one bake path, ticket 004) even
though only part of it feeds this ticket's own pure-geometry `solve()`. The
approach-speed taper IS applied inside `solve()` itself (it only scales the
commanded `v_x`/`omega` proportionally along the same tangent-circle
geometry, so it stays "geometry-adjacent" pure math, not navigation policy)
— `approachRadius <= 0` disables it (fail-open, matching `PlannerLimits`'
own `<= 0` convention), so every caller that never sets it gets the
untapered constant `speed` exactly as `solver.py`'s own behavior.

**Ctest coverage (17 checks, `arc_solver_test`, all green):** on-heading
target (zero curvature), the 90°-offset hand-computed tangent-circle
half-turn, a symmetric 30°-left/right pair, the target-behind guard at
directly-behind/120°/80° (stop vs. no-stop on both sides of the 90°
cutoff), the zero-distance degenerate case, the curvature slew clamp (a
single clamped step, a multi-step ramp converging in the exact number of
steps the budget predicts, and an abrupt-reversal clamp), purity (same
inputs → same outputs), target-final-heading-ignored, the `maxWheelStep`
derivation lock, and the default `behindAngle`/`turnFirstAngle` values.
`pursuitTarget()`'s own test surface was NOT ported — it stays host-side
unchanged, out of this ticket's scope per sprint 135's Out of Scope
section. The "`ArcSolution` cannot express an Angle stop" criterion is
enforced as a **compile-time** structured-binding decomposition (`const
auto& [v_x, omega, arcLength, stop, bearing] = solution;`) rather than
Python's runtime `dataclasses.fields()` introspection — C++ has no direct
equivalent, and the structured binding is arguably a stronger guard since a
future 6th field breaks the *build*, not just a test run.

**Verification run:**
```
cmake -S src/motion/navigator -B src/motion/navigator/build
cmake --build src/motion/navigator/build --target navigator_tests
# 100% tests passed, 1/1 (arc_solver_test, 17 checks)

cmake -S src/motion/planner -B src/motion/planner/build
cmake --build src/motion/planner/build --target planner_tests
# 100% tests passed, 8/8 -- unchanged, confirming zero disturbance to planner/
```
`git diff --stat -- src/motion/planner/` and `git diff --stat -- CMakeLists.txt`
both report no changes. `git status --short -- src/motion/navigator/` shows
exactly the four files listed above (the `build/` directories under both
`src/motion/navigator/` and `src/motion/planner/` are covered by the
repo-wide `build` gitignore rule and are not staged).
