---
id: '009'
title: "install(DRIVE)/install(WHEEL_CONTROL) — per-wheel Stage-A correction\
  \ live over the wire"
status: open
use-cases:
- SUC-003
- SUC-006
depends-on:
- '008'
github-issue: ''
issue: the-configuration-object.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# install(DRIVE)/install(WHEEL_CONTROL) — per-wheel Stage-A correction live over the wire

## Description

Complete `Configurator::install(ConfigTarget)` for `DRIVE` and
`WHEEL_CONTROL`: `DRIVE` calls `Drive::configure()` with the 8 per-wheel
Stage-A gain/intercept values (`wheelGainLeft`/`RightAccel`/`Decel` + 4
intercepts) via the existing `setWheelCorrection()`, plus `crawlPulse`
via `setCrawlPulse()`; `WHEEL_CONTROL` calls `Drive::configure()` with
Stage B/C gains via the existing `setControlGains()`/
`setAdaptationBounds()`. This is the sprint's original headline feature
landing as a wire arm on top of firmware logic that already exists
(`setWheelCorrection` et al. — confirmed present in `drive.h`). Also wire
the `MOTORS` target's guarded case: a push while a motor is in motion
returns `ERR_BUSY` (`Devices::Motor::configure()`'s bool return, from
ticket 007, mapped here).

## Acceptance Criteria

- [ ] A `DRIVE` group push containing per-wheel Stage-A gain/intercept
      values reaches `Drive::setWheelCorrection()` with the pushed
      values, verified in sim by a test that pushes a group and then
      asserts `Drive`'s reported wheel correction matches.
- [ ] A `WHEEL_CONTROL` group push reaches `Drive::setControlGains()`/
      `setAdaptationBounds()`, verified similarly.
- [ ] A `MOTORS` (`travel_calib`) push while a motor is reported in
      motion returns `ERR_BUSY`, not `OK`.
- [ ] A `MOTORS` push while at rest applies via `Motor::applyTravelCalib()`.
- [ ] A sim-level scenario test shows commanded wheel behavior changing
      after a live `DRIVE` push (an observable behavioral effect, not
      just that a setter was called).
- [ ] Compiles under `HOST_BUILD`.

## Testing

- **Existing tests to run**: Drive's existing Stage A/B/C harness tests
  continue to pass.
- **New tests to write**: the group-push-changes-live-behavior scenario
  above; the `MOTORS` busy-guard test.
- **Verification command**: `uv run python -m pytest
  <drive/configurator test paths> -q`.

## Implementation Plan

**Approach**: Fill in `install(ConfigTarget::DRIVE)`/
`install(ConfigTarget::WHEEL_CONTROL)`/`install(ConfigTarget::MOTORS)`
branches in `Configurator::install()`, each calling the relevant
subsystem's `configure()` (ticket 007) or, for `MOTORS` specifically,
checking the returned bool and mapping to `ERR_BUSY`.

**Files to modify**: `src/firm/app/configurator.cpp`.

**Testing plan**: as above.

**Documentation updates**: none beyond inline comments at each branch
explaining which setter it reaches.
