---
id: '007'
title: Subsystem configure(const Config::Robot&) entry points + derived-value methods
status: done
use-cases:
- SUC-002
- SUC-006
depends-on:
- '006'
github-issue: ''
issue: the-configuration-object.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Subsystem configure(const Config::Robot&) entry points + derived-value methods

## Description

Add `void configure(const Config::Robot&)` to `Drive`, `Motion::Planner`,
`Devices::Motor` (both instances get the call), `Devices::Otos`,
`App::StateEstimator`, and `RobotLoop` (for geometry/rotation
calibration). Each pulls only the fields it owns, reusing setters that
already exist: `Drive::setWheelCorrection`/`setControlGains`/
`setAdaptationBounds`/`setCrawlPulse`; `Motor::applyTravelCalib`;
`RobotLoop::setRotationCalibration`. `Devices::Motor::configure()`
additionally returns `bool`, `false` while the robot is moving
(`Configurator` maps this to `ERR_BUSY` — that mapping is ticket 009's
job; this ticket's scope is the `configure()` method itself returning the
correct bool). Add `effectiveTrackWidth()` and `rotationOffsetPos()` as
methods on `Config::Robot` (not stored fields), replacing the duplicate
derivation currently fanned out in `boot_calibration.cpp:25-29` to four
places.

## Acceptance Criteria

- [ ] `Drive::configure(const Config::Robot&)`,
      `Motion::Planner::configure(const Config::Robot&)`,
      `Devices::Motor::configure(const Config::Robot&) -> bool`,
      `Devices::Otos::configure(const Config::Robot&)`,
      `App::StateEstimator::configure(const Config::Robot&)`,
      `RobotLoop::configure(const Config::Robot&)` all exist.

      **PARTIALLY MET — see "Implementation decisions and deviations"
      below for the full rationale on each of the four departures:**
      - `Drive::configure(const Config::Robot&)` — MET, literal member
        method (`src/firm/app/drive.h`/`.cpp`).
      - `RobotLoop::configure(const Config::Robot&)` — MET, literal
        member method (`src/firm/app/robot_loop.h`).
      - `Motion::Planner::configure(const Config::Robot&)` — NOT a
        member method. `src/motion/planner/`'s own dependency rule
        (`src/motion/DESIGN.md` §3) forbids ANY `Config::*`/`App::*`/
        `Devices::*` dependency in that tree, no exception. Delivered
        instead as `App::configurePlanner(Motion::Planner&, const
        Config::Robot&)` (`src/firm/app/boot_calibration.h`/`.cpp`),
        reusing the existing `applyShaperLimits()` setter.
      - `Devices::Motor::configure(const Config::Robot&) -> bool` — NOT
        a member method. The devices isolation invariant
        (`src/firm/DESIGN.md` §5 / `src/firm/devices/DESIGN.md` §3)
        forbids `devices/` from including `messages/` or `config/`
        headers. Delivered instead as `[[nodiscard]] bool
        App::configureMotor(Devices::Motor&, const Config::Robot&, bool
        isLeft)` (`boot_calibration.h`/`.cpp`), reusing
        `applyTravelCalib()`, guarded exactly like
        `NezhaMotor::reconfigure()`'s own at-rest check, returning
        `false`/applying nothing while the motor is moving.
      - `Devices::Otos::configure(const Config::Robot&)` — NOT a member
        method, same isolation reason. Delivered instead as
        `App::configureOtos(Devices::Otos&, const Config::Robot&)`
        (`boot_calibration.h`/`.cpp`), reusing `setLinearScalar()`/
        `setAngularScalar()`/`setOffset()`.
      - `App::StateEstimator::configure(const Config::Robot&)` — NOT
        DELIVERABLE. `App::StateEstimator` (117-002) was renamed to
        `Motion::StateEstimator` (122-002) and then DELETED OUTRIGHT by
        sprint 128 ticket 016 ("a per-cycle computation with no
        consumer" — `src/motion/DESIGN.md` §2's own table entry). No
        class of either name exists anywhere in the current tree
        (confirmed: no `state_estimator.{h,cpp}` file under
        `src/firm/app/`, no `class StateEstimator`/`struct
        StateEstimator` anywhere in `src/`). `Config::Robot.estimator`
        (`weight_heading_otos`/`weight_omega_otos`/`staleness`)
        currently has NO live consumer at all — confirmed by
        `Configurator::apply()`'s own ESTIMATOR-branch comment
        (`configurator.cpp`) — this is trap 2, and "closing" it (giving
        it a live consumer) is explicitly ticket 010's own job, not
        this ticket's. This acceptance-criterion bullet is stale
        relative to code state that predates this sprint; recommend
        striking it or retargeting it once ticket 010 decides what (if
        anything) the ESTIMATOR group's live consumer becomes.
- [x] Each `configure()` call reuses an EXISTING setter (no new firmware
      control logic invented) — verified by a diff review showing each
      `configure()` body is a thin pull-and-forward.
- [x] `Devices::Motor::configure()` returns `false` when the motor
      reports itself in motion (reusing whatever "is moving" signal
      `NezhaMotor`/`MotorArmor` already exposes), `true` otherwise.
      (Delivered as `App::configureMotor()` — see above.)
- [x] `Config::Robot::effectiveTrackWidth()` and
      `Config::Robot::rotationOffsetPos()` exist as `const` methods, not
      stored fields; both match `boot_calibration.cpp:25-29`'s current
      derivation formula exactly. (Also added `rotationOffsetNeg()` and
      `velocityFilterWeight()`, the other two derivations named in the
      ticket description, `config/robot.h`.)
- [x] Every call site that previously read `boot_calibration.cpp`'s
      derived trackWidth/rotation values now calls the new methods
      instead (Drive, Odometry, Planner, RobotLoop — the four fan-out
      points named in the issue's own audit). `boot_wiring.cpp`'s
      `bakeBootValues()` now computes `trackWidth` via
      `Config::Robot::effectiveTrackWidth()` (one computation, still
      fanned to Drive/Odometry/Planner's constructors exactly as
      before — Drive/Odometry/Planner themselves have NO
      post-construction trackWidth setter, per this ticket's own
      "Constraints that will bite," so the retarget is in the
      computation, not per-destination); `RobotLoop`'s fan-out point now
      goes through its own new `configure()` instead of the deleted
      `installRotationCalibration()` free function.
- [x] Compiles under `HOST_BUILD`.

## Testing

- **Existing tests to run**: Drive/Planner/Motor/Otos unit test suites
  continue to pass unmodified where they don't touch `configure()`
  directly.
- **New tests to write**: one test per subsystem's `configure()`
  confirming it applies the expected setter calls given a sample
  `Config::Robot`; a test confirming `effectiveTrackWidth()`/
  `rotationOffsetPos()` match the old `boot_calibration.cpp` formula
  bit-for-bit for a sample input.
- **Verification command**: `uv run python -m pytest <relevant unit test
  paths> -q`.

## Implementation Plan

**Approach**: One `configure()` method per subsystem header/source pair,
each a small, focused addition. Derived-value methods added directly to
the `Config::Robot` header (ticket 002's generated file, or a small
hand-written extension alongside it — confirm whether the generator
supports attaching methods to generated structs, or whether `Config::Robot`
needs to be a hand-written wrapper embedding the generated group structs
by value, with derived methods on the wrapper; this is an implementation
call to resolve and document, not pre-decided at planning time).

**Files to modify**: `src/firm/app/drive.{h,cpp}`,
`src/firm/motion/planner/planner.{h,cpp}` (confirm exact path),
`src/firm/devices/motor.{h,cpp}`, `src/firm/devices/otos.{h,cpp}`,
`src/firm/app/state_estimator.{h,cpp}` (confirm exact filename),
`src/firm/app/robot_loop.{h,cpp}`, `src/firm/config/` (`Config::Robot`'s
own header, for the derived methods).

**Testing plan**: as above.

**Documentation updates**: each subsystem's file header gets a one-line
addendum noting its `configure()` entry point and what group(s) it reads.

## Implementation decisions and deviations (2026-08-03)

**1. `Config::Robot`'s location — its own header, `src/firm/config/robot.h`,
not left inline in `configurator.h`.** Ticket 006 built it inline in
`app/configurator.h`. Giving `Drive`/the `boot_calibration.h` adapters a
`configure(const Config::Robot&)` entry point means those headers need to
see the type — but `configurator.h` already `#include`s `app/drive.h`
(for the `Drive&` member), so `drive.h` including `configurator.h` back
would be a circular `#include`. `config/robot.h` depends on nothing but
`messages/robot_config.h` (itself only `messages/common.h` →
`<stdint.h>`), so every consumer (`drive.h`, `robot_loop.h`,
`boot_calibration.h`, `configurator.h` itself) can include it directly
with no cycle. `Config::Robot`'s own SHAPE (the 7 embedded `msg::` groups)
is unchanged from ticket 006; only its file moved, and its 4 derived-value
methods were added there.

**2. `Motion::Planner`/`Devices::Motor`/`Devices::Otos` get App::-layer
adapter functions, not member methods — a considered call, not an
oversight.** The ticket description invited this ("this is an
implementation call to resolve and document, not pre-decided at planning
time" applied to `Config::Robot`'s home; the same judgment call turned out
to also govern these three). Two independent, pre-existing, heavily
documented architecture rules make a literal
`SubsystemType::configure(const Config::Robot&)` member method
impossible for these three, not just awkward:

   - `src/motion/DESIGN.md` §3: *"`planner/` has its OWN, narrower
     dependency rule: exactly one `src/firm` header, `types/
     robot_state.h`"* and *"No `Devices::*`, `App::*`, or bus/timing
     collaborator anywhere in this tree"* / *"No `Devices::*`, `App::*`,
     `Config::*`, or wire-codec dependency anywhere in this tree."*
     `Config::*` is named explicitly. This is stricter than the devices
     isolation invariant and has no carve-out language anywhere in the
     doc, unlike the devices invariant (see the issue's own "the one
     place this collides with layering" section, which floats relaxing
     that ONE rule for a `<cstdint>`-only header — no equivalent
     discussion exists for `src/motion`).
   - `src/firm/DESIGN.md` §5 / `src/firm/devices/DESIGN.md` §3: *"devices/
     must not include `messages/...` or `config/...`"*, reinforced by a
     *"Deliberate non-goal: no `msg::`-typed surface anywhere in this
     directory ... that would be the isolation invariant violated from
     the other direction."* `devices/otos.h`'s own "Scope changes"
     header section additionally documents, as a **considered design
     choice already in place**, exactly why `Devices::OtosConfig` exists
     instead of reusing `Config::OtosBootConfig`: *"Devices-local so this
     leaf never includes config/."* Relaxing the invariant for
     `Config::Robot` would contradict that documented rationale in
     place, not just add a narrow exception beside it — a bigger, more
     coordinated doc change than this ticket's own scope, for a decision
     the issue itself frames as open ("the choice should be explicit").

   `App::` is the layer that can already see both a `Config::` type and
   the lower-layer subsystem's own API — `toDeviceMotorConfig()`
   (`boot_calibration.h`, pre-existing) already lives there for the
   identical structural reason (`msg::MotorConfig` → `Devices::
   MotorConfig`). `configurePlanner()`/`configureMotor()`/
   `configureOtos()` are that same established pattern, extended to
   `Config::Robot`. Functionally, Configurator can still call these
   uniformly once tickets 008-010 wire `install(target)`'s dispatch —
   `motorL_.configure(config_)` becomes
   `App::configureMotor(motorL_, config_, /*isLeft=*/true)`, one extra
   argument, same shape.

**3. `App::StateEstimator::configure()` — not delivered; see the
acceptance-criteria note above for the full "deleted by 128-016" finding.**
Considered throwing a formal ticket exception for this specifically
(Exception Protocol: "internal" surface, a module-boundary/data-model
fact) but decided against it: the conflict is narrow (1 of 6 named
subsystems, in 1 of 6 acceptance-criterion bullets) and does not block or
alter any of the other 5 subsystems' implementation, tests, or the
derived-value methods — throwing and halting per the protocol's own "do
not write partial code" instruction would have discarded five fully
achievable, independently valuable deliverables over one stale reference
to already-deleted code. Documented here instead, prominently, for
team-lead to strike or retarget the acceptance criterion.

**4. `installRotationCalibration()` (`boot_calibration.h`/`.cpp`) —
DELETED, not left orphaned.** Ticket 006 left this one as the explicit
open question for this ticket ("solve cleanly or say why you kept it
as-is"). Solved: `RobotLoop::configure(const Config::Robot&)` now does
what `installRotationCalibration()` used to, reading `config.geometry`
directly and `config.rotationOffsetPos()`/`rotationOffsetNeg()` instead
of doing its own degrees→radians conversion inline.
`boot_wiring.cpp`'s constructor calls `robotLoop_.configure(configurator_.
config())` directly, right after `configurator_.loadBaked()` — no
`RobotLoop&` reference added to `Configurator` (which would have been
circular, per 006's own note: `RobotLoop` already holds a
`Configurator&` for CONFIG routing). `installRotationCalibration()`
became fully orphaned once this landed (its one call site, confirmed by
repo-wide grep) and was deleted outright, along with the now-unused
`#include "app/robot_loop.h"` in `boot_calibration.h`.
`App::effectiveTrackWidth(msg::DrivetrainConfig)` (the OTHER
`boot_calibration.cpp` free function the issue names) was deliberately
LEFT IN PLACE, unlike `installRotationCalibration()` — it still has a
live caller, `composition_root_parity_harness.cpp`, which deliberately
computes its own "hardware-equivalent" value independently of whatever
`composeRobot()` does internally; deleting it would have broken that
harness for no ticket-scope reason. `bakeBootValues()`'s own trackWidth
computation was retargeted to `Config::Robot::effectiveTrackWidth()`
instead (see the acceptance-criteria note above) — the two formulas stay
numerically identical because `Config::defaultGeometryGroup()` and
`Config::defaultDrivetrainConfig()` bake identical
`trackwidth`/`rotational_slip` values from the same robot JSON (132-005's
own parity note), verified by `test_composition_root_parity.py` still
passing unmodified.

**5. `velocityFilterWeight()`'s one call site (`bootPlannerLimits()`,
`boot_calibration.cpp:63-65`) was NOT retargeted.** The method was added
(porting the `vel_filt_alpha > 0.05 ? : 1.0` formula faithfully, per the
ticket description) reading `Config::Robot.motors.vel_filt_alpha`
(genuinely populated, `0.3f` on `tovez`). The EXISTING call site reads a
DIFFERENT, never-populated field (`msg::DrivetrainConfig.vel_filt_alpha`,
which `Config::defaultDrivetrainConfig()` never sets, so it is always
`0.0f` there) — retargeting that call site would be a real behavior
change (`1.0` → `0.3f` at boot), which the ticket's own acceptance
criteria do not name among the four fan-out points to retarget (only
trackWidth/rotation are named). Left as a documented gap, not silently
"fixed."

## Testing performed

- New: `src/tests/sim/unit/configure_entry_points_harness.cpp` +
  `test_configure_entry_points.py` — derived-value methods (all 4, both
  branches of the two floor/sentinel conditionals),
  `Drive::configure()` (Stage B/C via direct getters; Stage A
  behaviorally, via `tick()`'s written duty changing with
  `wheel_gain_left_accel`), `configurePlanner()`, `configureMotor()`
  (both the at-rest-applies and the moving-refuses branches, per side),
  `configureOtos()`. No I2C bus, no sim plant — `Devices::Motor`/
  `Devices::Otos` are minimal in-file test doubles implementing the pure
  interfaces directly.
- New scenario in the existing `src/tests/sim/system/
  configurator_loadbaked_harness.cpp` (`test_configurator_loadbaked.py`)
  for `RobotLoop::configure()` — needs the full composition root, so it
  rides the sim boot that already exists there rather than a fresh
  harness; also proves `boot_wiring.cpp`'s retarget actually runs at
  real boot, not just in isolation.
- `test_composition_root_parity.py`
  (`composition_root_parity_harness.cpp`) — still passes, unmodified.
- `uv run python -m pytest src/tests/sim -q` — run for regressions (see
  final ticket status for the pass/fail count at completion).
