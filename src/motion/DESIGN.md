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
same treatment `src/sim`/`src/protos`/`src/scripts`/`src/tests`/
`src/utils` already get (see `docs/design/design.md` §2's "Other source
trees" table and §4's rationale for why the validated root set is
narrower than a bare `[src]`). Do not add `src/motion` to `sources:` —
this is a deliberate scope decision, not an oversight.

**128 (complexity reduction): the dead generation half is gone.**
Through sprint 127, this tree held two halves: a small set of already-live
leaves (`body_kinematics`, `odometry`, `state_estimator`, plus a
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

**Wheel control generations (128 ticket 015).** Three generations of the
wheel velocity-control law existed at once in this codebase; this ticket
resolved which of the three an engineer debugging misbehaving wheels should
suspect:
1. The closed-loop wheel-velocity PID relocated here from
   `src/firm/devices/velocity_pid.{h,cpp}` (125-003) — a full,
   never-instantiated class (`App::Drive` never actually held an instance
   by the time 122-002/125-002 finished reshaping `Drive` into a bare duty
   sink). **DELETED outright** — `wheel_velocity_pid.{h,cpp}`, zero callers.
2. `Motion::WheelPid`/`Planner::stageDuty()` (`planner/wheel_pid.{h,cpp}`,
   `Planner::stageDuty()` in `planner/planner.cpp`) — a full per-wheel PID
   that ran every cycle (~21 evaluations/s) whose output `main.cpp` used to
   note was "computed every tick and DISCARDED." **PARKED** — ticket 015
   removed `stageDuty()`'s live-tick call site (the decision is visible IN
   CODE, not left implicit); the `WheelPid` class and its own ctest tiers
   stay warm for a future duty-sink cutover. Owner: whoever picks up that
   cutover — see this file's own §6 for the open-question entry.
3. `Motion::WheelTrim` (`planner/wheel_trim.{h,cpp}`) — the live,
   velocity-domain trim loop `Planner::update()` actually runs every cycle;
   its output is what reaches `Types::RobotState::Wheel::cmdVelocity` and,
   through it, the wheels.

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
| `odometry.{h,cpp}` | `Motion::Odometry` — encoder-only dead-reckoning: integrates both wheels' position deltas through `BodyKinematics::forward()` into a world pose (`x`/`y`/`theta`) plus cumulative `pathLength()`. `integrate()` takes the caller's current wheel positions as plain float parameters — no `Devices::Motor&` held. OTOS sampling (`applyOtosSample()`) stays base-side (`src/firm/app/otos_sample.{h,cpp}`) since it needs `Devices::Otos&`/telemetry types, neither of which this tree may depend on. |
| `state_estimator.{h,cpp}` | `Motion::StateEstimator` — predict-to-now wheel/body PEER state estimates: zero-order-hold extrapolation from the latest basis reading, plus a v1 complementary blend against OTOS heading/omega (fail-closed weights, defaulting to 0.0, supplied by the caller — never read from `config/` directly). `update()` takes `Motion::StateEstimator::Input`, a type alias onto `Types::RobotState` (`src/firm/types/robot_state.h`, the dependency-free shared floor both trees stand on — see §3). |
| `planner/` | **`Motion::Planner`** — the on-robot motion decider (125–128, superseding the deleted `Motion::MoveQueue`): its own STANDALONE CMake project (`planner/CMakeLists.txt`), not built by this directory's own `CMakeLists.txt`. Profile generation (`profile.{h,cpp}`), jerk-limited shaping (`shape.{h,cpp}`), wheel-command estimation (`estimation.{h,cpp}`), a duty-stage closed loop (`wheel_pid.{h,cpp}`, PARKED from the live tick as of 128-015 — see "Wheel control generations" in §1) plus trim (`wheel_trim.{h,cpp}`, the LIVE loop that reaches the wheels), and the top-level `Planner` (`planner.{h,cpp}`) that ties them together and writes `Types::RobotState::Wheel::cmdVelocity` directly every cycle — no boundary interface, the blackboard field IS the boundary (128). `capi.cpp` builds the `motionplanner` shared library, the ctypes surface `src/tests/bench/planner_harness.py` drives. Its own `tests/` directory holds nine ctest executables (`profile_test`, `shape_test`, `estimation_test`, `planner_scenarios_test`, `planner_noise_test`, `ratio_lock_test`, `wheel_trim_test`, `wheel_pid_test`, `planner_duty_scenarios_test`), run via the `planner_tests` custom target. See §4 below. |
| `CMakeLists.txt` | The standalone `motion_tests` build (Design Rationale Decision 4) for THIS directory's own three leaves (`body_kinematics`/`odometry`/`state_estimator`) — plain CMake, no `libfirmware_host`, no ctypes, no Python. Builds a static `motion` library from those three `.cpp` files (128-015 deleted the fourth, `wheel_velocity_pid.cpp` — zero instantiations; see "Wheel control generations" in §1). As of 128-014 there is no standalone (non-pytest) ctest coverage for any of them attached to this target yet — see the file's own comment and §4 below. Separate from, and does not depend on, `planner/`'s own CMake project. |

## 3. Constraints and Invariants

- **`src/motion` imports NOTHING from `src/firm` except `messages/`
  headers and `firm/types/`.** Grep-verifiable: `grep -rn '#include "'
  src/motion | grep -v 'messages/' | grep -v 'firm/types/'` shows no
  other `src/firm` path (excluding `planner/`, which has its own,
  narrower rule below). `body_kinematics.h`'s `#include
  "messages/common.h"` is one explicit, permitted exception (the
  array-form `BodyKinematics` overloads take `msg::BodyTwist3`);
  `state_estimator.h`'s `#include "firm/types/robot_state.h"` is the
  second (sprint 124 architecture) — NOT a same-shape exception, since
  `firm/types` is itself a dependency-free leaf (cstdint-level includes
  only, no `msg::`/`messages/` types, no `app/` types — see
  `src/firm/types/DESIGN.md`) that both `src/firm` and `src/motion` stand
  on, rather than a base-owned type this tree reaches into.
- **`planner/` has its OWN, narrower dependency rule**: exactly one
  `src/firm` header, `types/robot_state.h` (`planner/CMakeLists.txt`'s
  own include-root comment) — the planner's own former `types/` mirror
  was deleted at the 2026-07-26 joint checkpoint in favor of depending on
  the real `firm/types/robot_state.h` directly, the same dependency-free
  floor `state_estimator.h` uses.
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
  "motion/odometry.h"`/`#include "motion/state_estimator.h"`/`#include
  "motion/planner/planner.h"`). Both the ARM build (root
  `CMakeLists.txt`) and the sim build (`src/sim/CMakeLists.txt`) add
  `src/` itself (the parent of both `src/firm` and `src/motion`) as an
  include root, so the directory literally being named `motion` at its
  top-level location keeps every qualified include text valid with zero
  call-site churn.

## 4. Design

**Module relationship inside this tree.** `Odometry` calls
`BodyKinematics::forward()` to fold encoder deltas into a world pose.
`StateEstimator` is a peer, not a collaborator of either — it reads the
same per-cycle wheel/pose data `RobotLoop` already assembled into
`Types::RobotState` (handed to it via its own `Input` alias onto that
type) and refreshes its own ZOH/complementary-blend estimates; nothing in
this tree yet consumes its `whereAmI()`/`wheelAt()` output (a future
trajectory controller does — out of current scope).
`planner/` is entirely self-contained: it does not link against or
depend on any of this directory's own three leaves (it has its own
kinematics/shaping/estimation code, see `planner/CMakeLists.txt`) — the
only thing tying `planner/` to the rest of this tree is that both halves
write into the SAME `Types::RobotState` blackboard, from the base's own
composition root (`main.cpp`/`src/sim/sim_harness.h`), never from one
motion-library module calling into another across the `planner/`
boundary.

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
generations" in §1); `body_kinematics`/`odometry`/`state_estimator` are
exercised today through the pytest ctypes harnesses under
`src/tests/sim/unit/` (`app_odometry_harness.cpp`/
`app_state_estimator_harness.cpp`), not through this CMake project.
`motion_tests` currently runs `ctest` over zero registered tests (a
valid, if trivial, green result) — SUC-001's own fast-iteration-loop
contract for this tree remains live even though nothing in ticket 014's
scope attaches new coverage to it; a future ticket adding standalone
coverage for any of the three remaining leaves has this target ready to
attach to.

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
  comment for the current, exact contract.
- **`Motion::StateEstimator::update(input, now)` / `wheelAt(wheel, t)` /
  `bodyAt(t)` / `whereAmI(now)` / `wheelNow(wheel)` / `reset(x, y,
  heading)` / `innovations()` / `setWeights(weights)`** — see
  `state_estimator.h`'s own doc comment; `update()` takes a
  `Motion::StateEstimator::Input` type alias onto `Types::RobotState`.
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
- **`firm/types/robot_state.h`** — used by `state_estimator.h` (this
  directory) and by every `planner/` module that touches
  `Types::RobotState` (`planner.cpp`'s own `update()`).
- **`src/tests/sim/plant/wheel_plant.{h,cpp}`** — reused, deterministic
  physics scaffolding; not linked into this tree's own `motion` library,
  only into pytest/ctypes harnesses that exercise it externally.
- Nothing else. No `Devices::*`, `App::*`, `Config::*`, or wire-codec
  dependency anywhere in this tree — see §3's invariants above.

## 6. Open Questions / Known Limitations

- **`motion_tests` currently has zero attached ctest coverage** (see §4)
  — a real gap for a future ticket, not a silent regression: the three
  leaves it could cover are exercised via pytest/ctypes instead today.
- **`Planner::stageDuty()`'s duty stage is PARKED, not adopted or
  deleted** (128-015 — see "Wheel control generations" in §1). It is no
  longer called from the live tick, but `Motion::WheelPid`'s class and
  ctest tiers are kept warm for a future duty-sink cutover — whoever picks
  that ticket up owns deciding whether the duty stage or `WheelTrim`
  should be the wheels' one true control law, and re-wiring `Planner`'s
  own live loop if the answer is the duty stage. Left open deliberately;
  not this ticket's decision to make (sprint 128 Decision 2, settled by
  the stakeholder as PARK).
- **`StateEstimator` is not yet wired to drive motion.** Its
  `whereAmI()`/`wheelAt()` output has no consumer in this tree or the
  base — the future trajectory controller that consumes it remains out
  of scope.
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
