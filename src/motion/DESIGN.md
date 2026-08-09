# Motion (`src/motion`) — Motion-Control Library

**Owner:** Eric Busboom · **Last reviewed:** 2026-07-31 · **Status:** stable

---

## 1. Purpose

`src/motion/` is the motion-control half of sprint 122's two-layer split
(base vs. motion library): a SIBLING tree to `src/firm` (not a child of
it — sprint 122 Design Rationale Decision 3), holding the twist
decomposition, shaping, estimation, and odometry logic that is still
under active development (goal-exact tours, same-axis carry, heading
hold) — separated from the hardware-facing base (`src/firm`), which the
stakeholder intends to eventually freeze and move to its own repository
via `git subtree split`. The split lets each layer evolve at its own
rate: the base is a frozen-candidate surface (buses, devices, the wire,
the loop schedule); this tree is where the motion-control accuracy work
actually happens, with a fast, Python-free, sim-free unit-test loop
(`motion_tests`, and `planner/`'s own separate `planner_tests`) to
iterate against instead of the full `libfirmware_host` sim harness.

**Not in `.clasi/config.yaml`'s validated `sources:` list** (which stays
exactly `[src/firm, src/host]` — stakeholder-locked, sprint 122 Design
Rationale Decision 3's own consequence). This directory is real, current
documentation, unvalidated by the mechanical design-doc checker — the
same treatment `src/firm/platform/host`/`src/protos`/`src/scripts`/`src/tests`/
`src/utils` already get (see `docs/design/design.md` §2's "Other source
trees" table and §4's rationale for why the validated root set is
narrower than a bare `[src]`). Do not add `src/motion` to `sources:` —
this is a deliberate scope decision, not an oversight.

**128 (complexity reduction): the dead generation half is gone.**
Through sprint 127, this tree held two halves: a small set of already-live
leaves (`body_kinematics`, `odometry`, `state_estimator` — the last of
these deleted outright by ticket 016, see "Estimator roster" below, plus a
zero-instantiation closed-loop wheel-velocity-PID class deleted outright by
ticket 015 — see "Wheel control generations" below) and a larger
`MoveQueue`/`WheelSink`/`StopCondition`/`VelocityShaper` stack that was the
ORIGINAL plan for how
Motion would drive wheel actuation. That stack was superseded in practice
by `Motion::Planner` (`planner/`, added 125–127) without ever being
deleted — `Motion::Planner::update()` wrote `Types::RobotState::Wheel::
cmdVelocity` directly from the start, `App::RobotLoop::cycle()` consumed
it directly, and `MoveQueue` was never wired into either. Sprint 128
ticket 014 confirmed zero callers and deleted `wheel_sink.h`,
`move_queue.{h,cpp}`, `stop_condition.{h,cpp}`, `velocity_shaper.{h,cpp}`
outright (~1,500 lines) along with their `motion_tests` ctest targets.
**`planner/` is now the larger, live half of this tree** — see §2/§4
below. The land-at-zero completion predicate's own empirical derivation
(118/119/121) is preserved as dated history, not lost:
[`docs/design/history/land-at-zero-margin-derivation.md`](../../docs/design/history/land-at-zero-margin-derivation.md).

**Wheel control generations (128 ticket 015; superseded 130-005; generation
2 deleted outright by 130-007).** Four generations of the wheel
velocity-control law have existed in this codebase; this section tracks
which of them an engineer debugging misbehaving wheels should suspect:
1. The closed-loop wheel-velocity PID relocated here from
   `src/firm/devices/velocity_pid.{h,cpp}` (125-003) — a full,
   never-instantiated class (`App::Drive` never actually held an instance
   by the time 122-002/125-002 finished reshaping `Drive` into a bare duty
   sink). **DELETED outright** — `wheel_velocity_pid.{h,cpp}`, zero callers.
2. `Motion::WheelPid`/`Planner::stageDuty()` (formerly
   `planner/wheel_pid.{h,cpp}`, `Planner::stageDuty()` in
   `planner/planner.cpp`) — a full per-wheel PID that ran every cycle (~21
   evaluations/s) whose output `main.cpp` used to note was "computed every
   tick and DISCARDED." Ticket 015 first **PARKED** it (removed
   `stageDuty()`'s live-tick call site, keeping the class and its own ctest
   tiers warm for a possible future duty-sink cutover). **DELETED outright
   (130-007)**, reversing that PARK: generation 4 below is now the proven,
   shipped law, so the parked stage has no future — `wheel_pid.{h,cpp}`,
   `stageDuty()`, `dutyLeft_`/`dutyRight_` and their accessors,
   `applyVelGains()`, and the stage's own ctest tiers (`wheel_pid_test.cpp`,
   `planner_duty_scenarios_test.cpp`, `tests/duty_plant.h`) are all gone;
   git preserves the history for any future duty-sink revisit. The `pid.*`
   CONFIG wire keys these used to (silently) target were already repointed
   onto generation 4 by ticket 005 — this deletion just removes the dead
   destination they no longer reach.
3. `Motion::WheelTrim` (`planner/wheel_trim.{h,cpp}`) — the velocity-domain
   trim loop `Planner::update()` used to run every cycle, summed into
   `Types::RobotState::Wheel::cmdVelocity` on top of the profiled command.
   **DELETED outright (130-005, wheel-speed-controller-moves-into-
   drive.md Phase 3)** — no redirect stub, no future caller; git preserves
   the history. `Motion::Planner` sheds ALL wheel-actuation code as of this
   ticket: `Planner::update()` now publishes `cmdVelocity` bit-for-bit as
   the profiled command, always. This is a responsibility RELOCATION, not
   a base-depends-on-motion violation — the code moved to generation 4,
   below, which is `App::`-owned; `src/firm` still imports nothing from
   `src/motion` for this feature (sprint 130 Architecture Design Rationale
   Decision 2).
4. `App::Drive`'s unified wheel-speed controller (`src/firm/app/drive.{h,cpp}`,
   130-004/130-005) — the CURRENT live law, and the only one actuating any
   wheel today. Three-timescale: a calibrated conversion map (offline,
   per-wheel/per-direction affine, unchanged from Drive's own prior
   open-loop feedforward), a bounded slow-timescale bias trim (Stage C,
   generation 3's velocity-domain closed-loop idea, relocated and
   redesigned as bounded intercept adaptation rather than an unbounded
   PI integral), and a small-authority fast PID (Stage B). Reached by
   EVERY `cmdVelocity` writer alike — a WHEELS teleop command and a
   planner Move both go through the exact same `Drive::tick()`, no
   privileged or degraded path (drive.h's own header has the full
   per-stage algorithm and parameter table).

**Estimator roster (128 ticket 016,
robot-state-pose-needs-exactly-one-writer.md).** Three names an engineer
debugging "which pose is the robot's pose" might reach for; only two
still exist:
- **`Motion::Odometry`** (`odometry.{h,cpp}`) — the telemetry trip
  odometer: encoder-only dead reckoning, and `Types::RobotState::pose`'s
  ONE writer (`App::RobotLoop::publishPose()`, robot_loop.cpp). This is
  the pose a host reading telemetry sees.
- **`Motion::Planner`'s internal `PoseTracker`** (`planner/planner.cpp`,
  the `pose_` member) — the planner's OWN working estimate, separately
  dead-reckoned from Odometry's and optionally OTOS-heading-blended
  (`limits_.headingOtosWeight`, fail-closed default 0.0) — the estimate
  that actually drives the robot's own motion decisions. Never wired to
  `RobotState::pose` (it used to write there too, ordering-dependent on
  which of Odometry/Planner ran last a given cycle — that second write
  was the exact bug ticket 016 closed); it does feed
  `RobotState::estimate.body`/`estimate.wheelLeft`/`estimate.wheelRight`
  every cycle (the ZOH-basis section below).
- **`Motion::StateEstimator`** — DELETED (ticket 016), no replacement in
  this tree. It lived at `state_estimator.{h,cpp}`: predict-to-now
  wheel/body PEER estimates via ZOH extrapolation plus its own separate
  v1 complementary OTOS blend, constructor-injected weights, wired into
  `App::RobotLoop::cycle()`'s trailing kPace block every cycle
  (`stateEstimator_.update(state_, now)`) since sprint 117 — but its
  `update()` took `RobotState` by CONST reference and held its own
  private peer basis, so it never actually wrote back into
  `RobotState::estimate` (Planner did, and still does — see above); its
  `wheelAt()`/`bodyAt()`/`whereAmI()` query surface had zero production
  callers the whole time it existed (only its own test harnesses called
  them). A per-cycle computation with no consumer, closed as dead work.
  A future estimator rebuild is separate, tracked work — see
  `clasi/issues/later/estimator-v2-otos-fusion-sim-first.md` — and starts
  fresh from `PoseTracker`'s own math rather than reviving this class.

**Historical/derivation record for the surviving leaves.** This file
documents the CURRENT orientation of the whole tree (what's here, the
boundary, the build). The original multi-sprint design derivation for
`BodyKinematics` (moved from `src/firm/kinematics/`, ticket 122-001) is
NOT re-derived here; it is kept, unchanged and still authoritative, in
[`src/firm/kinematics/DESIGN.md`](../firm/kinematics/DESIGN.md), which
redirects here for current orientation but retains the original
math/rationale/tuning-history prose. The now-deleted `StopCondition`/
`VelocityShaper`/`MoveQueue` stack's own pre-128 design history remains
in [`src/firm/motion/DESIGN.md`](../firm/motion/DESIGN.md) (marked
RETIRED there since 122, kept solely as a historical derivation record —
math at a location the code no longer occupies) and
[`src/firm/app/DESIGN.md`](../firm/app/DESIGN.md) (`MoveQueue`/pre-122
history sections) — neither file is rewritten to remove that history;
see each file's own header for the "why this stays" rationale.

## 2. Orientation

| File / directory | Role |
|---|---|
| `body_kinematics.{h,cpp}` | `BodyKinematics` — stateless differential-drive twist/wheel-speed maps (`inverse`/`forward`) and curvature-preserving `saturate`. Moved verbatim from `src/firm/kinematics/` (122-001; that directory is retired, folded flat here rather than nested as `src/motion/kinematics/`). The one permitted `src/firm` dependency: `#include "messages/common.h"` for the array-form overloads' `msg::BodyTwist3` parameter. Full derivation: [`src/firm/kinematics/DESIGN.md`](../firm/kinematics/DESIGN.md) and [`docs/kinematics-model.md`](../../docs/kinematics-model.md). |
| `odometry.{h,cpp}` | `Motion::Odometry` — encoder-only dead-reckoning: integrates both wheels' position deltas through `BodyKinematics::forward()` into a world pose (`x`/`y`/`theta`) plus cumulative `pathLength()`. `integrate()` takes the caller's current wheel positions as plain float parameters — no `Devices::Motor&` held. `Types::RobotState::pose`'s ONE writer (128 ticket 016 — see "Estimator roster" in §1). OTOS sampling (`applyOtosSample()`) stays base-side (`src/firm/app/otos_sample.{h,cpp}`) since it needs `Devices::Otos&`/telemetry types, neither of which this tree may depend on. |
| `state_estimator.{h,cpp}` | **Deleted (128 ticket 016).** Was: `Motion::StateEstimator`, predict-to-now wheel/body PEER state estimates via ZOH extrapolation plus a v1 complementary OTOS blend — a per-cycle computation with no consumer. See "Estimator roster" in §1 for the full accounting and what remains (`Odometry`, `Planner`'s own internal `PoseTracker`). |
| `planner/` | **`Motion::Planner`** — the on-robot motion decider (125–128, superseding the deleted `Motion::MoveQueue`): its own STANDALONE CMake project (`planner/CMakeLists.txt`), not built by this directory's own `CMakeLists.txt`. Profile generation (`profile.{h,cpp}`), jerk-limited shaping (`shape.{h,cpp}`), wheel-command estimation (`estimation.{h,cpp}`), and the top-level `Planner` (`planner.{h,cpp}`) that ties them together and writes `Types::RobotState::Wheel::cmdVelocity` directly every cycle — no boundary interface, the blackboard field IS the boundary (128). The duty-stage closed loop (`wheel_pid.{h,cpp}`, PARKED from the live tick as of 128-015) is **DELETED outright (130-007)** — see "Wheel control generations" in §1. `wheel_trim.{h,cpp}` (the velocity-domain trim loop that used to be summed into `cmdVelocity` here) is **DELETED outright (130-005)** — `Motion::Planner` publishes the bare profiled command now; the wheel-speed controller lives entirely in `App::Drive` (`src/firm/app/drive.{h,cpp}`) — see "Wheel control generations" in §1. `capi.cpp` builds the `motionplanner` shared library, the ctypes surface `src/tests/bench/planner_harness.py` drives. Its own `tests/` directory holds seven ctest executables (`profile_test`, `shape_test`, `estimation_test`, `planner_scenarios_test`, `planner_noise_test`, `ratio_lock_test`, `pose_ownership_test` — 128-016's own regression guard for `Odometry` being `state.pose`'s sole writer; 130-007 removed `wheel_pid_test`/`planner_duty_scenarios_test` with the deleted duty stage), run via the `planner_tests` custom target. See §4 below. |
| `CMakeLists.txt` | The standalone `motion_tests` build (Design Rationale Decision 4) for THIS directory's own remaining leaves (`body_kinematics`/`odometry`) — plain CMake, no `libfirmware_host`, no ctypes, no Python. Builds a static `motion` library from those two `.cpp` files (128-015 deleted a third, `wheel_velocity_pid.cpp` — zero instantiations; 128-016 deleted a fourth, `state_estimator.cpp` — see "Wheel control generations"/"Estimator roster" in §1). As of 128-014 there is no standalone (non-pytest) ctest coverage for either of them attached to this target yet — see the file's own comment and §4 below. Separate from, and does not depend on, `planner/`'s own CMake project. |

## 3. Constraints and Invariants

- **`src/motion` imports NOTHING from `src/firm` except `messages/`
  headers and `firm/types/`.** Grep-verifiable: `grep -rn '#include "'
  src/motion | grep -v 'messages/' | grep -v 'firm/types/'` shows no
  other `src/firm` path (excluding `planner/`, which has its own,
  narrower rule below). `body_kinematics.h`'s `#include
  "messages/common.h"` is the one explicit, permitted exception in this
  directory's own two remaining leaves (the array-form `BodyKinematics`
  overloads take `msg::BodyTwist3`); `odometry.{h,cpp}` imports neither
  `messages/` nor `firm/types/` at all (its API is plain floats).
  `state_estimator.h`'s own former `#include "firm/types/robot_state.h"`
  (sprint 124 architecture) died with the file (128 ticket 016).
- **`planner/` has its OWN, narrower dependency rule**: exactly one
  `src/firm` header, `types/robot_state.h` (`planner/CMakeLists.txt`'s
  own include-root comment) — the planner's own former `types/` mirror
  was deleted at the 2026-07-26 joint checkpoint in favor of depending on
  the real `firm/types/robot_state.h` directly, the same dependency-free
  floor `state_estimator.h` used to use (128 ticket 016: `planner/` is
  now this file's only consumer).
- **`Types::RobotState::Wheel::cmdVelocity` is THE base<->motion
  actuation boundary** (128, superseding the deleted `Motion::WheelSink`
  velocity-sink interface). No abstract interface, no concrete
  implementation to hold — `Motion::Planner::update()` (or `App::Drive::
  update()` for WHEELS teleop) writes the field directly;
  `RobotLoop::cycle()` reads it directly. See `robot_state.h`'s own
  `cmdVelocity` field comment for the full writer/consumer contract.
- **No `Devices::*`, `App::*`, or bus/timing collaborator anywhere in
  this tree.** Every module takes plain data (floats, explicit `now`
  timestamps, plain structs) as parameters — never a held
  `Devices::Clock&`, `Devices::Motor&`, or telemetry-frame reference.
  This is what makes every module here constructible and testable with
  hand-fed numbers alone, identically under `HOST_BUILD`, on ARM, and in
  the standalone `motion_tests`/`planner_tests` builds.
- **Qualified `#include "motion/...h"` paths resolve unchanged** in every
  `src/firm` caller (e.g. `src/firm/app/robot_loop.h`'s `#include
  "motion/odometry.h"`/`#include "motion/planner/planner.h"`). Both the
  ARM build (root
  `CMakeLists.txt`) and the sim build (`src/firm/platform/host/CMakeLists.txt`) add
  `src/` itself (the parent of both `src/firm` and `src/motion`) as an
  include root, so the directory literally being named `motion` at its
  top-level location keeps every qualified include text valid with zero
  call-site churn.

## 4. Design

**Module relationship inside this tree.** `Odometry` calls
`BodyKinematics::forward()` to fold encoder deltas into a world pose, and
is `Types::RobotState::pose`'s ONE writer (`App::RobotLoop::
publishPose()` reads `x()`/`y()`/`theta()` off it every cycle — 128
ticket 016, "Estimator roster" in §1). `Motion::StateEstimator`, formerly
a peer of `Odometry` here, is deleted (same ticket) — see §1 for the full
accounting.
`planner/` is entirely self-contained: it does not link against or
depend on any of this directory's own three leaves (it has its own
kinematics/shaping/estimation code, see `planner/CMakeLists.txt`) — the
only thing tying `planner/` to the rest of this tree is that both halves
write into the SAME `Types::RobotState` blackboard, from the base's own
composition root -- `App::composeRobot()`/`App::RobotGraph`
(`src/firm/app/boot_wiring.h`), the ONE function both `main.cpp` and
`src/firm/platform/host/sim_harness.h` call (130-002, unify-sim-and-robot-composition-
roots.md) -- never from one motion-library module calling into another
across the `planner/` boundary.

**The actuation boundary (`Types::RobotState::Wheel::cmdVelocity`).**
Whichever subsystem currently owns motion — `Motion::Planner` for a Move,
`App::Drive` for WHEELS teleop — writes this cycle's commanded wheel
speed directly onto the shared blackboard; `RobotLoop::cycle()` reads it
back and hands it to `App::Drive::tick()` for actuation. Exclusivity is
enforced by ORDERING (exactly one of the two `update()` calls actually
writes, per cycle — see `robot_loop.h`'s own doc comment), not by a
locked/shared resource or an abstract interface. This SUPERSEDES the
122-era `Motion::WheelSink` boundary interface (`setWheels()`/`stop()`),
which was deleted in sprint 128 ticket 014 after being confirmed to have
zero callers — `Motion::Planner::update()` wrote `cmdVelocity` directly
from the moment it was integrated (125–127), never through `WheelSink`,
and `RobotLoop::cycle()` never routed through it either.

**The `motion_tests` build (Design Rationale Decision 4), THIS
directory's three leaves only.** A real CMake project at
`src/motion/CMakeLists.txt` — configure/build it standalone with `cmake
-S src/motion -B src/motion/build && cmake --build src/motion/build
--target motion_tests`, or `ctest --output-on-failure` from an
already-configured build directory. No Python process anywhere in that
sequence — this is the whole point of the split (SUC-001): a fast,
hardware- and Python-free iteration loop for motion-control logic.
Sprint 128 ticket 014 deleted this target's three ctest executables
(`motion_stop_condition_tests`/`motion_velocity_shaper_tests`/
`motion_move_queue_chained_tests`) along with the `MoveQueue`-generation
code they exercised; ticket 015 deleted a fourth leaf outright
(`wheel_velocity_pid.{h,cpp}`, zero instantiations — see "Wheel control
generations" in §1); ticket 016 deleted a fifth leaf outright
(`state_estimator.{h,cpp}`, a per-cycle computation with no consumer —
see "Estimator roster" in §1). `body_kinematics`/`odometry` are
exercised today through the pytest ctypes harness under
`src/tests/sim/unit/` (`app_odometry_harness.cpp`), not through this
CMake project; `Odometry`'s pose-single-writer contract additionally has
a dedicated regression guard, `pose_ownership_test`, in `planner/`'s own
`planner_tests` suite (§2 above), since exercising it needs both
`Odometry` and `Motion::Planner` in the same process.
`motion_tests` currently runs `ctest` over zero registered tests (a
valid, if trivial, green result) — SUC-001's own fast-iteration-loop
contract for this tree remains live even though nothing in ticket 014's
scope attaches new coverage to it; a future ticket adding standalone
coverage for either remaining leaf has this target ready to attach to.

**The `planner/` build (its own standalone project).** `cmake -S
src/motion/planner -B src/motion/planner/build && cmake --build
src/motion/planner/build --target planner_tests` builds every planner
test executable and runs them via `ctest`, exiting nonzero on any
failure. `motionplanner` (a `SHARED` library wrapping `capi.cpp`) is the
ctypes surface `src/tests/bench/planner_harness.py` drives for
bench-side planner exercising outside the full sim harness. See
`planner/CMakeLists.txt`'s own header comment for the exact commands.

## 5. Interfaces

### Exposes

- **`BodyKinematics::inverse()`/`forward()`/`saturate()`** (scalar and
  `msg::BodyTwist3`/`wheels[2]` array-form overloads) — see
  [`src/firm/kinematics/DESIGN.md`](../firm/kinematics/DESIGN.md) §5.
- **`Motion::Odometry(trackWidth, initialLeftPosition,
  initialRightPosition)` / `integrate(leftPosition, rightPosition)` /
  `pathLength()` / `reset(x, y, heading)`** — see `odometry.h`'s own doc
  comment for the current, exact contract. `Types::RobotState::pose`'s
  ONE writer (128 ticket 016 — see "Estimator roster" in §1); `Motion::
  StateEstimator`'s former `update()`/`wheelAt()`/`bodyAt()`/`whereAmI()`/
  `wheelNow()`/`innovations()`/`setWeights()` surface is deleted, same
  ticket, no replacement in this tree.
- **`Motion::Planner::move(move, replace)` / `plannedStop(id)` /
  `estop()` / `tick(state)` / `update(state)` / `active()` /
  `shaperConfigured()` / `applyShaperLimits(...)`** — the live motion
  decider; see `planner/planner.h`'s own doc comment for the current,
  exact contract (this file does not re-derive it — `planner/` owns its
  own interface documentation inline, no separate `planner/DESIGN.md`
  exists as of this writing).
- **`Types::RobotState::Wheel::cmdVelocity`** — the actuation boundary
  itself; see §4 above and `robot_state.h`'s own field comment.

### Consumes

- **`messages/common.h`** (`msg::BodyTwist3`) — used only by
  `body_kinematics.h`'s array-form overloads.
- **`firm/types/robot_state.h`** — used by every `planner/` module that
  touches `Types::RobotState` (`planner.cpp`'s own `update()`); no longer
  used by anything directly in this directory (`state_estimator.h`, its
  former consumer here, is deleted — 128 ticket 016).
- **`src/tests/sim/plant/wheel_plant.{h,cpp}`** — reused, deterministic
  physics scaffolding; not linked into this tree's own `motion` library,
  only into pytest/ctypes harnesses that exercise it externally.
- Nothing else. No `Devices::*`, `App::*`, `Config::*`, or wire-codec
  dependency anywhere in this tree — see §3's invariants above.

## 6. Open Questions / Known Limitations

- **`motion_tests` currently has zero attached ctest coverage** (see §4)
  — a real gap for a future ticket, not a silent regression: the three
  leaves it could cover are exercised via pytest/ctypes instead today.
- **RESOLVED (130-004/130-005, stakeholder 2026-08-01): the wheels' one
  true control law is `App::Drive`'s unified three-timescale controller**
  (generation 4, "Wheel control generations" in §1) — not the duty stage,
  not `WheelTrim`. `WheelTrim` is deleted outright (130-005); the duty
  stage (`Motion::WheelPid`/`Planner::stageDuty()`, PARKED since 128-015)
  is now ALSO deleted outright (130-007, sprint 130's own planner-honesty
  pass), reversing the PARK now that generation 4 is the proven, shipped
  law (sprint 130 Design Rationale).
- **No `planner/DESIGN.md` exists yet.** `planner/` is documented inline
  here (§2/§4/§5) rather than in its own co-located file; a future pass
  consolidating this tree's documentation may choose to split it out,
  matching the co-located-`DESIGN.md` convention every other directory in
  this repo follows — not attempted by ticket 128-014, which is scoped to
  reconciling this file with the current tree, not restructuring the doc
  set further.
- **Historical derivation for the retired generation stack lives in two
  separate pre-128 documents**, not consolidated here (deliberate — see
  §1's own note): [`src/firm/motion/DESIGN.md`](../firm/motion/DESIGN.md)
  (StopCondition/VelocityShaper, RETIRED since 122) and
  [`src/firm/app/DESIGN.md`](../firm/app/DESIGN.md) (MoveQueue, marked
  there as pre-128 history), plus
  [`docs/design/history/land-at-zero-margin-derivation.md`](../../docs/design/history/land-at-zero-margin-derivation.md)
  for the land-at-zero completion predicate specifically. A future
  `consolidate-architecture` pass may choose to fold all of it into this
  file; not attempted by ticket 128-014, which is scoped to
  reconciliation, not consolidation.
