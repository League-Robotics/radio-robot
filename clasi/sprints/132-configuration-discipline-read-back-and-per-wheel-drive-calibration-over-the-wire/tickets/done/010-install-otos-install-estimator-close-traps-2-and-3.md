---
id: '010'
title: "install(OTOS)/install(ESTIMATOR) \u2014 close traps 2 and 3"
status: done
use-cases:
- SUC-003
depends-on:
- 008
github-issue: ''
issue: the-configuration-object.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# install(OTOS)/install(ESTIMATOR) — close traps 2 and 3

## Description

Complete `Configurator::install(ConfigTarget)` for `OTOS` and
`ESTIMATOR`, closing two confirmed-live bugs as part of building this
honestly (Design Rationale Decision 4):

- **Trap 2**: `EstimatorConfigPatch`'s `weight_heading_otos`/
  `weight_omega_otos`/`staleness_ms` currently ack `0` and land nowhere
  (`configurator.cpp`'s `ESTIMATOR` branch has a comment admitting this).
  Wire `ESTIMATOR`'s push to an actual `App::StateEstimator::setWeights()`
  call (or equivalent) — today's `Configurator` has no reference to a
  `StateEstimator` at all; add one.
- **Trap 3**: `RealOtos::begin()` converts the baked scale MULTIPLIER
  through `scaleToRegister()` before calling `setLinearScalar()`/
  `setAngularScalar()` (`otos.cpp:45-46`), which take the chip's raw
  int8 REGISTER value directly; the current live wire path
  (`configurator.cpp:159-160`) passes the pushed value straight through
  with no conversion, installing a 1-LSB scalar instead of the intended
  multiplier. Fix `install(OTOS)` to apply `scaleToRegister()` the same
  way `begin()` does before calling the chip-level setters.

**Implementation-time finding on trap 2 (recorded here, not just in code
comments):** `App::StateEstimator` does not exist in this tree — it was
renamed to `Motion::StateEstimator` (122-002) and then deleted outright as
dead code (sprint 128 ticket 016, `robot-state-pose-needs-exactly-one-
writer.md`; see `src/motion/DESIGN.md`'s "Estimator roster"). Its one
candidate successor, `Motion::PoseTracker::blendHeading()`
(`src/motion/planner/estimation.h`), is itself dead: 130-009 deleted its
only call site (`Planner::tick()`) AND its own config fields
(`PlannerLimits::headingOtosWeight`/`otosStaleness`) outright, in favor of
a from-scratch fusion redesign tracked at `clasi/issues/later/
estimator-v2-otos-fusion-sim-first.md` — confirmed by `planner_types.h`'s
own removal note and `planner.cpp:383-392`'s own comment at the deleted
call site. There is therefore no live consumer to wire to today, by two
independent, deliberate prior architecture decisions, not by oversight.
Building one would mean either resurrecting logic 130-009 explicitly
retired, or building the estimator-v2 fusion redesign itself — the latter
explicitly out of THIS ticket's scope (it is its own tracked, unplanned
issue) and contrary to this sprint's own module boundary (sprint.md Step
3: "this sprint gives them a wire path, it does not invent new firmware
logic"). Trap 2 is therefore closed here by making the dead end EXPLICIT
and PERMANENT — `install(ESTIMATOR)` keeps ticket 008's `ERR_UNIMPLEMENTED`
(now documented as a terminal state, not a to-be-filled gap) — rather than
by adding a `StateEstimator&` reference that has nothing real behind it.
`Configurator`'s constructor is UNCHANGED (no new parameter): see the
Acceptance Criteria below, revised to match.

## Acceptance Criteria

- [x] ~~`Configurator` holds a reference to `App::StateEstimator`... so
      `install(ESTIMATOR)` has something real to call.~~ REVISED
      (implementation-time finding, see Description): `App::StateEstimator`
      does not exist, and its one candidate successor's call site was
      itself deliberately deleted (130-009) pending a from-scratch
      estimator-v2 redesign. `Configurator` holds NO estimator-shaped
      reference — its constructor is unchanged. Confirmed instead:
      `Configurator::install(ConfigGroupTarget::ESTIMATOR)` permanently
      returns `ERR_UNIMPLEMENTED` (unchanged from ticket 008), now
      documented (`configurator.h`'s re-appliability table,
      `configurator.cpp`'s own case comment, `robot_config.proto`'s
      `Estimator` message comment) as a deliberate, permanent terminal
      state — not a to-be-filled gap — citing sprint 128 ticket 016, 130-009,
      and `clasi/issues/later/estimator-v2-otos-fusion-sim-first.md`.
- [x] ~~An `ESTIMATOR` group push... changes the live `StateEstimator`'s
      fusion weights, verified by a new test reading them back after the
      push.~~ REVISED: a new test (`configurator_applygroup_harness.cpp`)
      confirms an `ESTIMATOR` group push still DECODES into `config_`
      (`config().estimator.weight_heading_otos` reflects the push) — the
      read-back promise stays honest — while `install()`'s return stays
      `ERR_UNIMPLEMENTED`, not `ERR_NONE`: config that acks OK and lands
      nowhere is worse than config that is rejected; `ESTIMATOR` does
      neither.
- [x] `install(OTOS)`'s scale application calls `scaleToRegister()` (or
      an equivalently-named conversion) on a pushed `linearScale`/
      `angularScale` value before reaching `setLinearScalar()`/
      `setAngularScalar()`, matching `begin()`'s existing conversion.
      `scaleToRegister()` moved out of `RealOtos`'s private section to a
      `Devices::`-namespace-scope free function (`otos.h`/`otos.cpp`) so
      `App::configureOtos()` (`app/boot_calibration.cpp`) can call the
      SAME conversion `RealOtos::begin()` calls, rather than duplicating
      the formula.
- [x] A new regression test: push `linearScale = 1.0` live to `OTOS` and
      confirm the resulting chip-level register write equals what
      `begin()` would install for the SAME `1.0` baked value (i.e., the
      live and boot paths agree on what "1.0" means) — trap 3's own
      regression guard. (`configurator_applygroup_harness.cpp`'s "OTOS
      scale regression guard" scenario; a direct `scaleToRegister()` unit
      scenario also added to `devices_otos_harness.cpp`.)
- [x] The `OTOS` lever-arm (`offset_x`/`offset_y`/`offset_yaw`) push path
      is unchanged (it already has no domain mismatch — `setOffset()`
      takes the value directly).
- [x] Compiles under `HOST_BUILD`.

## Testing

- **Existing tests to run**: OTOS/Configurator existing unit coverage
  continues to pass (`test_devices_otos.py`,
  `test_configurator_applygroup.py`, `test_configurator_loadbaked.py`,
  `test_sim_fidelity.py`).
- **New tests written**: the OTOS scale-domain regression test (push
  `linear_scale`/`angular_scale = 1.0`, confirm register 0 lands, matching
  `begin()`); a direct `Devices::scaleToRegister()` unit scenario
  (`devices_otos_harness.cpp`); the ESTIMATOR read-back-stays-honest-
  despite-`ERR_UNIMPLEMENTED` test (REVISED from the originally-planned
  "weights actually land" test — see Acceptance Criteria).
- **Verification command**: `uv run python -m pytest
  src/tests/sim/unit/test_devices_otos.py
  src/tests/sim/unit/test_configurator_applygroup.py
  src/tests/sim/system/test_configurator_loadbaked.py
  src/tests/sim/system/faults/test_sim_fidelity.py -q`.

## Implementation Plan

**Approach (REVISED from the original plan during implementation — see
Description's "Implementation-time finding on trap 2")**: `install(ConfigTarget::
OTOS)` now routes the scale fields through the same conversion `begin()`
already applies — `scaleToRegister()` moved out of `RealOtos`'s private
section to a `Devices::`-namespace-scope free function
(`otos.h`/`otos.cpp`) both `RealOtos::begin()` and `App::configureOtos()`
call. `install(ConfigTarget::ESTIMATOR)` is NOT given a `StateEstimator&`
constructor parameter — investigation established no live consumer exists
or can be wired without either resurrecting logic 130-009 deliberately
retired, or building the estimator-v2 redesign itself (out of scope, its
own tracked issue). `Configurator`'s constructor is unchanged;
`install(ESTIMATOR)` keeps ticket 008's `ERR_UNIMPLEMENTED`, now documented
throughout as permanent.

**Files modified**: `src/firm/devices/otos.{h,cpp}` (`scaleToRegister()`
exposed as a free function), `src/firm/app/boot_calibration.{h,cpp}`
(`configureOtos()` applies the conversion), `src/firm/app/configurator.
{h,cpp}` (documentation only — the re-appliability table and the
`install(target)` case comments), `src/protos/robot_config.proto`
(`Otos`/`Estimator` message doc comments), `src/scripts/gen_boot_config.py`
+ regenerated `src/firm/config/boot_config.cpp` (doc comment only, no value
change), test harnesses (`configurator_applygroup_harness.cpp`,
`devices_otos_harness.cpp`, `configure_entry_points_harness.cpp` (132-007's
own `configureOtos()` scenario expected the old pass-through — updated to
expect the register-domain conversion), `sim_fidelity_harness.cpp` deduped
onto the now-exposed free function). `src/firm/app/boot_wiring.{h,cpp}` is
UNCHANGED — no new constructor parameter, per the revised approach above.

**Testing plan**: as above.

**Documentation updates**: `configurator.h`'s file header gets a note
that traps 2/3 (referenced by name, matching this sprint's `sprint.md`)
are closed here, so a future reader doesn't need to re-discover them.
