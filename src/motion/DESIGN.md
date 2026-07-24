# Motion (`src/motion`) — Motion-Control Library

**Owner:** Eric Busboom · **Last reviewed:** 2026-07-24 · **Status:** stable

---

## 1. Purpose

`src/motion/` is the motion-control half of sprint 122's two-layer split
(base vs. motion library): a SIBLING tree to `src/firm` (not a child of
it — sprint 122 Design Rationale Decision 3), holding the twist
decomposition, queueing, shaping, estimation, and odometry logic that is
still under active development (goal-exact tours, same-axis carry,
heading hold) — separated from the hardware-facing base (`src/firm`),
which the stakeholder intends to eventually freeze and move to its own
repository via `git subtree split`. The split lets each layer evolve at
its own rate: the base is a frozen-candidate surface (buses, devices, the
velocity PID, the wire, the loop schedule); this tree is where the
motion-control accuracy work actually happens, with a fast, Python-free,
sim-free unit-test loop (`motion_tests`) to iterate against instead of
the full `libfirmware_host` sim harness.

**Not in `.clasi/config.yaml`'s validated `sources:` list** (which stays
exactly `[src/firm, src/host]` — stakeholder-locked, sprint 122 Design
Rationale Decision 3's own consequence). This directory is real, current
documentation, unvalidated by the mechanical design-doc checker — the
same treatment `src/sim`/`src/protos`/`src/scripts`/`src/tests`/
`src/utils` already get (see `docs/design/design.md` §2's "Other source
trees" table and §4's rationale for why the validated root set is
narrower than a bare `[src]`). Do not add `src/motion` to `sources:` —
this is a deliberate scope decision, not an oversight.

**Historical/derivation record.** This file documents the CURRENT
orientation of the whole tree (what's here, the boundary, the build). The
original multi-sprint design derivation for the three oldest modules —
`StopCondition`/`VelocityShaper` (moved from `src/firm/motion/` in ticket
122-001) and `BodyKinematics` (moved from `src/firm/kinematics/`, same
ticket) — is NOT re-derived here; it is kept, unchanged and still
authoritative, in
[`src/firm/motion/DESIGN.md`](../firm/motion/DESIGN.md) and
[`src/firm/kinematics/DESIGN.md`](../firm/kinematics/DESIGN.md), which
now redirect here for current orientation but retain the original
math/rationale/tuning-history prose. `MoveQueue`/`StateEstimator`/
`Odometry`'s own pre-122 design history (dozens of dated sprint notes:
land-at-zero derivation, chain-advance margin sweeps, the estimator's
ZOH/complementary-blend design) is likewise kept, unchanged, in
[`src/firm/app/DESIGN.md`](../firm/app/DESIGN.md) (marked there as
pre-122 history) rather than reproduced here.

## 2. Orientation

| File | Role |
|---|---|
| `stop_condition.{h,cpp}` | `Motion::StopCondition` — per-`Move` stop-condition comparison (Time/Distance/Angle + timeout backstop): captures activation baselines once, at construction, and answers "has this motion ended?" every `tick()`. Stateless, pure, zero dependency on `MoveQueue`/`Drive`/any `msg::*` wire type. Moved verbatim from `src/firm/motion/` (122-001) — zero behavior change. Full derivation: [`src/firm/motion/DESIGN.md`](../firm/motion/DESIGN.md). |
| `velocity_shaper.{h,cpp}` | `Motion::VelocityShaper` — the decel-into-the-goal, jerk-limited speed shaper: two chained rate clamps (velocity, then accel) plus an integrator, one instance per axis (`v_x`/`omega`, or per-wheel for a `Wheels` Move). Moved verbatim from `src/firm/motion/` (122-001) — zero behavior change. Full derivation, margin-term math, and the chain-advance hand-off contract: [`src/firm/motion/DESIGN.md`](../firm/motion/DESIGN.md). |
| `body_kinematics.{h,cpp}` | `BodyKinematics` — stateless differential-drive twist/wheel-speed maps (`inverse`/`forward`) and curvature-preserving `saturate`. Moved verbatim from `src/firm/kinematics/` (122-001; that directory is retired, folded flat here rather than nested as `src/motion/kinematics/`). The one permitted `src/firm` dependency: `#include "messages/common.h"` for the array-form overloads' `msg::BodyTwist3` parameter. Full derivation: [`src/firm/kinematics/DESIGN.md`](../firm/kinematics/DESIGN.md) and [`docs/kinematics-model.md`](../../docs/kinematics-model.md). |
| `move_queue.{h,cpp}` | `Motion::MoveQueue` — owns the lifecycle of the robot's queued and active bounded motions: the 5-slot array (1 active + 4 pending), replace/flush/enqueue/`ERR_FULL` bookkeeping, chain-advance on stop/timeout, one `Motion::StopCondition` per active `Move`, the land-at-zero completion predicate, and (122-002) the twist-decomposition call (`BodyKinematics::inverse()`) that used to live in `App::Drive::setTwist()` — `MoveQueue` computes `v_left`/`v_right` itself and hands them down through the `WheelSink` boundary via `setWheels()`. Moved from `src/firm/app/move_queue.*` (122-002); no longer holds a concrete `App::Drive&` (holds a `Motion::WheelSink&` instead) or a `Devices::Clock&` (`tick()`/`enqueue()` take `now` as an explicit parameter). Pre-122 design history (land-at-zero derivation, margin-factor sweeps, chain-advance contract): [`src/firm/app/DESIGN.md`](../firm/app/DESIGN.md). |
| `odometry.{h,cpp}` | `Motion::Odometry` — encoder-only dead-reckoning: integrates both wheels' position deltas through `BodyKinematics::forward()` into a world pose (`x`/`y`/`theta`) plus cumulative `pathLength()`. Moved from `src/firm/app/odometry.*` (122-002); no longer holds a `Devices::Motor&` — `integrate()` takes the caller's current wheel positions as plain float parameters. OTOS sampling (`applyOtosSample()`) split out to stay base-side (`src/firm/app/otos_sample.{h,cpp}`) since it needs `Devices::Otos&`/`Telemetry::Frame&`, neither of which this tree may depend on. |
| `state_estimator.{h,cpp}` | `Motion::StateEstimator` — predict-to-now wheel/body PEER state estimates: zero-order-hold extrapolation from the latest basis reading, plus a v1 complementary blend against OTOS heading/omega (fail-closed weights, defaulting to 0.0, supplied by the caller — never read from `config/` directly). Moved from `src/firm/app/state_estimator.*` (122-002); `update()` takes a plain `Motion::StateEstimator::Input` struct instead of `App::Telemetry::Frame` (this tree may not depend on `app/`). Does not yet drive motion — greenfield/quarantined, same posture as pre-122. |
| `wheel_sink.h` | `Motion::WheelSink` — the ONE boundary header (122-001): an abstract wheel-velocity command sink (`setWheels(v_left, v_right)`/`stop()`), the per-wheel state struct (`WheelState`) Motion reads, and the plain config struct (`WheelSinkConfig`) Motion is constructed with. See §3/§5 below for the full contract. No concrete implementation lives here — that's the base's job (`App::Drive`, `src/firm/app`). |
| `CMakeLists.txt` | The standalone `motion_tests` build (Design Rationale Decision 4): plain CMake, no `libfirmware_host`, no ctypes, no Python anywhere in the build/run path. Builds a static `motion` library from all six `.cpp` files above and three `ctest`-registered executables (StopCondition, VelocityShaper, and the end-to-end chained-`Wheels`-`Move` scenario against the model plant), plus a `motion_tests` custom target that builds and runs all three via `ctest --output-on-failure`. See §4 below and the file's own header comment for exact build/run commands. |

## 3. Constraints and Invariants

- **`src/motion` imports NOTHING from `src/firm` except `messages/`
  headers.** Grep-verifiable: `grep -rn '#include "' src/motion | grep
  -v 'messages/'` shows no other `src/firm` path.
  `body_kinematics.h`'s `#include "messages/common.h"` is the one
  explicit, permitted exception (the array-form `BodyKinematics`
  overloads take `msg::BodyTwist3`); every other module in this tree has
  zero `src/firm` dependency of any kind.
- **The boundary is a velocity sink, never a duty sink** (sprint 122
  Design Rationale Decision 1, stakeholder-locked 2026-07-24) —
  `Motion::WheelSink::setWheels(v_left, v_right)`/`stop()`, not a duty
  target. The duty-sink rewrite (folding sprint 2's PID-placement
  decision in — `velocity_pid.*` and `Devices::NezhaMotor`'s PID
  ownership stay in the base, unchanged, this sprint) is explicitly
  deferred to sprint 2's base-hardening work
  (`clasi/issues/firmware-base-hardening-bounded-wheel-moves-and-wheel-observer.md`),
  not a discretionary call this tree reopened.
- **No concrete `WheelSink` implementation lives in this tree.**
  `App::Drive` (`src/firm/app/drive.{h,cpp}`) is the base's
  implementation; `src/motion` only ever holds the boundary's abstract
  interface and consumers of it (`Motion::MoveQueue`).
- **No `Devices::*`, `App::*`, or bus/timing collaborator anywhere in
  this tree.** Every module takes plain data (floats, explicit `now`
  timestamps, plain structs) as parameters — never a held
  `Devices::Clock&`, `Devices::Motor&`, or `App::Telemetry::Frame&`. This
  is what makes every module here constructible and testable with
  hand-fed numbers alone, identically under `HOST_BUILD`, on ARM, and in
  the standalone `motion_tests` build.
- **Zero behavior change from the pre-extraction tree.** Sprint 122's
  entire extraction (tickets 001-002) is a pure mechanical move — every
  existing test passes unchanged; see sprint 122's Test Strategy for the
  recorded pre-/post-extraction baseline diff. This tree's own
  correctness bar going forward is ordinary code review, same as any
  other subsystem — the zero-behavior-change bar applied only to the
  move itself, not to future motion-control work done here.
- **Qualified `#include "motion/...h"` paths resolve unchanged** in every
  `src/firm` caller (e.g. `src/firm/app/drive.h`'s `#include
  "motion/wheel_sink.h"`, `src/firm/app/robot_loop.h`'s `#include
  "motion/move_queue.h"`). Both the ARM build (root `CMakeLists.txt`) and
  the sim build (`src/sim/CMakeLists.txt`) add `src/` itself (the parent
  of both `src/firm` and `src/motion`) as an include root, so the
  directory literally being named `motion` at its new top-level location
  keeps every qualified include text valid with zero call-site churn. The
  one path that DID change: `#include "kinematics/body_kinematics.h"` →
  `#include "motion/body_kinematics.h"` (`kinematics/` is retired, folded
  flat into `src/motion` rather than nested).

## 4. Design

**Module dependency graph inside this tree.** `MoveQueue` is the one
consumer that ties everything else together: it holds a `WheelSink&`
(the boundary out), a `Motion::StopCondition` per active `Move`, one or
more `Motion::VelocityShaper` instances (one per axis), and calls
`BodyKinematics::inverse()` directly for the twist-decomposition path.
`Odometry` calls `BodyKinematics::forward()` to fold encoder deltas into
a world pose. `StateEstimator` is currently a peer, not a `MoveQueue`
collaborator — it reads the same per-cycle wheel/pose data `MoveQueue`
and `Odometry` produce (handed to it via its own `Input` struct by the
caller, `App::RobotLoop`), but nothing in this tree yet consumes its
`whereAmI()`/`wheelAt()` output (the future trajectory controller does,
out of this sprint's scope).

**The boundary interface's In/Out shape (`wheel_sink.h`).** In (Motion
reads): `WheelState` — one wheel's `position`/`velocity`/`sampleTime` per
cycle, mirroring what `Devices::Motor` already exposes without depending
on that header. Out (Motion commands): `WheelSink::setWheels(v_left,
v_right)`/`stop()`. Config (Motion is constructed with, once, never
mutated by Motion itself): `WheelSinkConfig` — track width plus the
`VelocityShaper` accel/decel/jerk ceilings for both axes. The concrete
`WheelState`/config plumbing that actually flows through `MoveQueue` in
practice is its own `ShaperLimits` struct (`move_queue.h`) mirroring
`Config::ShaperBootConfig` — `main.cpp` converts between the two at the
composition root, since `config/` may not depend on `motion/` either.

**The `motion_tests` build (Design Rationale Decision 4).** A real CMake
project at `src/motion/CMakeLists.txt` — configure/build it standalone
with `cmake -S src/motion -B src/motion/build && cmake --build
src/motion/build --target motion_tests`, or `ctest
--output-on-failure` from an already-configured build directory. No
Python process anywhere in that sequence — this is the whole point of
the split (SUC-001): a fast, hardware- and Python-free iteration loop for
motion-control logic. Three `ctest`-registered scenarios:

- `motion_stop_condition_tests` / `motion_velocity_shaper_tests` — reuse
  the existing `src/tests/sim/unit/motion_{stop_condition,velocity_shaper}_harness.cpp`
  files as-is (not copied/moved — module boundary is normative, harness
  file layout is not); the same pytest-subprocess wrappers
  (`test_motion_stop_condition.py`/`test_motion_velocity_shaper.py`)
  still work unchanged and can still be run under `uv run pytest`, they
  just no longer need to be for a developer iterating on motion logic
  alone.
- `motion_move_queue_chained_tests` — the new (122-002) end-to-end
  scenario: two chained `Wheels` `Move`s through a REAL
  `Motion::MoveQueue`/`Motion::Odometry` pair against the REAL model
  plant (`TestSim::WheelPlant`, `src/tests/sim/plant/wheel_plant.cpp` —
  reused verbatim, test-only physics scaffolding, never folded into the
  `motion` library itself), asserting the completion/chain-advance
  sequence with no `SimHarness`, no `ctypes`, no Python.

`motion_tests` (the one named target SUC-001 asks a developer to build
and run) depends on all three test binaries and runs them via `ctest` as
its own recipe — `cmake --build . --target motion_tests` alone builds
AND runs every scenario, exiting nonzero on any failure.

## 5. Interfaces

### Exposes

- **`Motion::StopCondition(kind, threshold, timeout, now, pathLength,
  theta)` / `tick(now, pathLength, theta)`** — see
  [`src/firm/motion/DESIGN.md`](../firm/motion/DESIGN.md) §5 for the full
  contract.
- **`Motion::VelocityShaper::next(cruiseSpeed, remaining, dt, aMax,
  aDecel, jMax)` / `reset()` / `syncTo(speed)` / `commandedSpeed()` /
  `commandedAccel()`** — see
  [`src/firm/motion/DESIGN.md`](../firm/motion/DESIGN.md) §5.
- **`BodyKinematics::inverse()`/`forward()`/`saturate()`** (scalar and
  `msg::BodyTwist3`/`wheels[2]` array-form overloads) — see
  [`src/firm/kinematics/DESIGN.md`](../firm/kinematics/DESIGN.md) §5.
- **`Motion::MoveQueue::enqueue(move, now)` / `tick(now, odom)` /
  `flush()` / `active()`** — see [`src/firm/app/DESIGN.md`](../firm/app/DESIGN.md)
  §5 (pre-122 history; the signatures there predate the explicit-`now`/
  `WheelSink&` change 122-002 made — see this tree's own `move_queue.h`
  doc comment for the current, exact parameter list).
- **`Motion::Odometry(trackWidth, initialLeftPosition,
  initialRightPosition)` / `integrate(leftPosition, rightPosition)` /
  `pathLength()` / `reset(x, y, heading)`** — see `odometry.h`'s own doc
  comment for the current, exact contract (122-002 dropped the
  constructor/`integrate()`'s `Devices::Motor&` in favor of plain float
  parameters — see §2 above).
- **`Motion::StateEstimator::update(input, now)` / `wheelAt(wheel, t)` /
  `bodyAt(t)` / `whereAmI(now)` / `wheelNow(wheel)` / `reset(x, y,
  heading)` / `innovations()` / `setWeights(weights)`** — see
  [`src/firm/app/DESIGN.md`](../firm/app/DESIGN.md) §5 (pre-122 history;
  `update()` now takes a plain `Motion::StateEstimator::Input` instead of
  `App::Telemetry::Frame` — see this tree's own `state_estimator.h` doc
  comment).
- **`Motion::WheelSink::setWheels(v_left, v_right)` / `stop()`** — the
  boundary itself; see §4 above.

### Consumes

- **`messages/common.h`** (`msg::BodyTwist3`) — the ONE `src/firm`
  dependency, used only by `body_kinematics.h`'s array-form overloads.
- **`src/tests/sim/plant/wheel_plant.{h,cpp}`** (`motion_tests` only, not
  the `motion` library itself) — the model plant the end-to-end chained-
  Move scenario drives against.
- Nothing else. No `Devices::*`, `App::*`, `Config::*`, or wire-codec
  dependency anywhere in this tree — see §3's invariants above.

## 6. Open Questions / Known Limitations

- **Naming/namespace layout is a suggestion, not yet re-litigated.**
  Sprint 122 sprint.md's Open Questions 1-3 (exact boundary-interface/
  class names, whether `App::Drive` keeps its name, whether the pytest
  wrappers around the `motion_*_harness.cpp` files are eventually retired
  in favor of this tree's own `ctest` target) remain genuinely open —
  this ticket (122-004) is documentation-only and does not resolve them.
- **The velocity-sink boundary will need a non-trivial second revision in
  sprint 2** (Design Rationale Decision 1's own consequence): the
  duty-sink rewrite moves `velocity_pid.*`/PID ownership into this tree,
  changes `WheelSinkConfig`'s shape, and adds an `appliedDuty` feedback
  path. Expected and acceptable — not a defect in this sprint's boundary,
  which was deliberately scoped not to make that later revision harder
  (e.g. no "velocity" baked into the interface name in a way that fights
  a later rename).
- **`StateEstimator` is not yet wired to drive motion.** Its
  `whereAmI()`/`wheelAt()` output has no consumer in this tree or the
  base — the future trajectory controller that consumes it is explicitly
  out of this sprint's scope (see sprint 122's Out of Scope section).
- **Full historical derivation for the six moved modules lives in three
  separate pre-122 documents**, not consolidated here (deliberate — see
  §1's own note): [`src/firm/motion/DESIGN.md`](../firm/motion/DESIGN.md)
  (StopCondition/VelocityShaper), [`src/firm/kinematics/DESIGN.md`](../firm/kinematics/DESIGN.md)
  (BodyKinematics), and [`src/firm/app/DESIGN.md`](../firm/app/DESIGN.md)
  (MoveQueue/StateEstimator/Odometry, marked there as pre-122 history). A
  future `consolidate-architecture` pass may choose to fold all of it
  into this file; not attempted by this ticket, which is scoped to
  reconciliation, not consolidation.
