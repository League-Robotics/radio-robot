---
id: '010'
title: "install(OTOS)/install(ESTIMATOR) — close traps 2 and 3"
status: open
use-cases:
- SUC-003
depends-on:
- '008'
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

## Acceptance Criteria

- [ ] `Configurator` holds a reference to `App::StateEstimator` (confirm
      exact type/name during implementation) so `install(ESTIMATOR)` has
      something real to call.
- [ ] An `ESTIMATOR` group push with `weight_heading_otos`/
      `weight_omega_otos`/`staleness_ms` changes the live
      `StateEstimator`'s fusion weights, verified by a new test reading
      them back after the push (not just confirming a setter was called
      with no consumer, mirroring the old bug).
- [ ] `install(OTOS)`'s scale application calls `scaleToRegister()` (or
      an equivalently-named conversion) on a pushed `linearScale`/
      `angularScale` value before reaching `setLinearScalar()`/
      `setAngularScalar()`, matching `begin()`'s existing conversion.
- [ ] A new regression test: push `linearScale = 1.0` live to `OTOS` and
      confirm the resulting chip-level register write equals what
      `begin()` would install for the SAME `1.0` baked value (i.e., the
      live and boot paths agree on what "1.0" means) — trap 3's own
      regression guard.
- [ ] The `OTOS` lever-arm (`offset_x`/`offset_y`/`offset_yaw`) push path
      is unchanged (it already has no domain mismatch — `setOffset()`
      takes the value directly).
- [ ] Compiles under `HOST_BUILD`.

## Testing

- **Existing tests to run**: OTOS/StateEstimator existing unit coverage
  continues to pass.
- **New tests to write**: the ESTIMATOR-weights-actually-land test; the
  OTOS scale-domain regression test above.
- **Verification command**: `uv run python -m pytest
  <otos/estimator/configurator test paths> -q`.

## Implementation Plan

**Approach**: Add a `StateEstimator&` (or correct type) constructor
parameter to `Configurator`, threaded through from the composition root
(`boot_wiring.cpp`). Fill in `install(ConfigTarget::ESTIMATOR)` to call
its real setter. Fill in `install(ConfigTarget::OTOS)` to route the scale
fields through the same conversion `begin()` already applies (factor
`scaleToRegister()` out of `otos.cpp` into something both `begin()` and
`Configurator` can call, if it isn't already reachable — confirm during
implementation whether it needs to move out of `RealOtos`'s private
section).

**Files to modify**: `src/firm/app/configurator.{h,cpp}`,
`src/firm/app/boot_wiring.{h,cpp}` (new constructor parameter),
`src/firm/devices/otos.{h,cpp}` (expose `scaleToRegister()` if needed).

**Testing plan**: as above.

**Documentation updates**: `configurator.h`'s file header gets a note
that traps 2/3 (referenced by name, matching this sprint's `sprint.md`)
are closed here, so a future reader doesn't need to re-discover them.
