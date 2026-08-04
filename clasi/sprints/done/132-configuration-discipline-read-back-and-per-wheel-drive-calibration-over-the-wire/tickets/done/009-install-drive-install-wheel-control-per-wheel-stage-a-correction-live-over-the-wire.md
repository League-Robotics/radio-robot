---
id: 009
title: "install(DRIVE)/install(WHEEL_CONTROL) \u2014 per-wheel Stage-A correction\
  \ live over the wire"
status: done
use-cases:
- SUC-003
- SUC-006
depends-on:
- 008
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

- [x] A `DRIVE` group push containing per-wheel Stage-A gain/intercept
      values reaches `Drive::setWheelCorrection()` with the pushed
      values, verified in sim by a test that pushes a group and then
      asserts `Drive`'s reported wheel correction matches.
- [x] A `WHEEL_CONTROL` group push reaches `Drive::setControlGains()`/
      `setAdaptationBounds()`, verified similarly.
- [x] A `MOTORS` (`travel_calib`) push while a motor is reported in
      motion returns `ERR_BUSY`, not `OK`.
- [x] A `MOTORS` push while at rest applies via `Motor::applyTravelCalib()`.
- [x] A sim-level scenario test shows commanded wheel behavior changing
      after a live `DRIVE` push (an observable behavioral effect, not
      just that a setter was called).
- [x] Compiles under `HOST_BUILD`.

## Completion Note (132-009)

**What was already in place when this ticket started.** Ticket 008's own
`install(ConfigGroupTarget)` (`configurator.cpp`) had already implemented
the DRIVE/WHEEL_CONTROL/MOTORS branches this ticket's acceptance criteria
describe — `DRIVE`/`WHEEL_CONTROL` call `drive_.configure(config_)`
(132-007's entry point, which reaches `setWheelCorrection()`/
`setCrawlPulse()`/`setControlGains()`/`setAdaptationBounds()`), and
`MOTORS` calls `App::configureMotor()` per side with the busy guard
surfaced as `ERR_BUSY` — plus `configurator_applygroup_harness.cpp`
already carried a behavioral test (pushing `wheel_gain_left_accel=2.0`
over the wire and asserting `tick()`'s written duty changes accordingly)
and a per-side `MOTORS` busy-guard test. This is flagged explicitly rather
than silently claimed as this ticket's own new work — all six acceptance
criteria above were independently re-verified (test run recorded below),
not merely inherited unchecked.

**What this ticket actually built.** `Configurator::install()` — the
no-arg, BOOT-TIME fan-out (`RobotGraph`'s constructor: `loadBaked()` then
`install()`) — was, until this ticket, NOT retargeted onto the same
`Drive::configure(config_)` call `install(target)` uses; it still
re-derived Stage A/B/C inline a second time (configurator.h's own
"tickets 009/010's job" note). This ticket retargets it, and resolves the
one loose end explicitly left for it: the `kDutyPerSpeed` decision (see
below). A new regression scenario was added to
`configurator_applygroup_harness.cpp` covering `loadBaked()+install()`
specifically (the boot path, distinct from every wire-push scenario
above), since no existing sim test could observe it — every
`TestSim::SimHarness`-based test unconditionally overrides `dutyPerSpeed`
immediately after construction (`sim_harness.h`), for reasons specific to
the sim plant, independent of what the boot path installs.

## Design Decision: the `kDutyPerSpeed` reversal

**Decision: REVERSED.** `Configurator::install()` now reads
`config_.drive.duty_per_speed_left/right` (the active robot JSON, via
`Config::defaultDriveGroup()`) instead of the hardcoded
`Drive::kDutyPerSpeed` C++ literal it read from 2026-07-31 until this
ticket.

**Why.** The 07-31 "MEASURED, NOT CONFIGURED" decision solved a real
problem — `duty_per_speed` and `wheel_gain` had been circularly fitted
against each other (`tovez.json`'s own `_wheel_correction_note`) — but at
the cost of two standing violations, both confirmed on disk:

1. The 2026-08-03 configuration-discipline rule
   (`.claude/rules/configuration-discipline.md`): "every value the robot
   uses comes from the file," no production-boot exception. A hardcoded
   C++ literal `Configurator::install()` read instead of `config_.drive`
   is a direct, named violation.
2. This project's own pre-existing "no defaults, always configured" boot
   posture (`drive.h`'s `dutyPerSpeedLeft_`/`Right_` doc comment: "Baking
   a value here is what made one robot's gearboxes every robot's"). Before
   this ticket, `Drive::kDutyPerSpeed` was read identically for
   `tovez`/`togov`/`tovez_nocal` regardless of which JSON was baked — the
   exact anti-pattern that doc comment's own policy exists to prevent, and
   a bug independent of the configuration-discipline rule.

The design's own resolution (`the-configuration-object.md`) is to keep
`dutyPerSpeed` a ONE-population-scale value with ALL per-wheel deviation
expressed in `wheel_gain`/`wheel_intercept` instead — i.e., prevent the
circular-fit risk by CONVENTION (the file's two fields are measured
together and kept numerically equal, never re-fit against `wheel_gain`
residuals — `duty_sweep.py`'s own "constant-free saturation reading"
cross-check already enforces this in practice) rather than by keeping the
value outside the file entirely. Sourcing it from `config_.drive` is
therefore **behavior-preserving, not a new calibration decision**:
`data/robots/tovez.json`/`tovez_nocal.json`'s `duty_per_speed_left/right`
were corrected from a stale `0.00187325` (the ~1.6x error
`the-configuration-object.md`'s Cause section cites) to `0.001182` — the
value that was ALREADY running on every robot via the hardcoded constant
this replaces, verified by regenerating `boot_config.cpp` and confirming
the diff touches only those two literals. No robot's actual boot behavior
changes the moment this lands. `togov.json`'s own `duty_per_speed` (its
own independent 2026-07-27 measurement) was deliberately left untouched —
it now finally takes effect if `togov` is ever built as the active robot,
which was silently impossible before this ticket.

`Drive::kDutyPerSpeed` itself is **kept**, unread by production boot, as
the documented measurement the file's value traces to and as
`duty_sweep.py`'s own cross-check anchor. `Drive::configure()` (the LIVE
wire-push path) still does **not** touch `dutyPerSpeed` — that stays
boot-only by design; the wire's `DRIVE` group never carried it and still
does not (unrelated to this reversal, which is purely about where the
BOOT value's SOURCE OF TRUTH lives).

**What was deliberately left alone.**

- `boot_calibration.cpp`'s `installDriveCalibration()`/
  `installWheelController()`/`installShaperLimits()` are dead code (no
  call site — `Configurator::install()` already did this inline before
  this ticket, for the latter two; deletion is a cleanup-ticket's call,
  not this one's scope). Their doc comments were annotated to flag them as
  dead and to stop the pre-reversal `installDriveCalibration()` body's
  "MEASURED, NOT CONFIGURED" framing from silently contradicting the live
  path next to it — the body itself was deliberately NOT updated to track
  the reversal, since it has no caller.
- Stage B/C re-clamping on a lowered live bound (`setControlGains()`/
  `setAdaptationBounds()` do not re-clamp already-running state) remains
  open — already documented on `install(target)`'s own DRIVE/
  WHEEL_CONTROL case (ticket 008) as a known gap, unaddressed by this
  ticket per the-configuration-object.md's own caveat on the "safe"
  setters.
- OTOS's `scaleToRegister()` domain mismatch and ESTIMATOR's missing
  consumer (traps 2/3) are ticket 010's job, untouched here.

**Verification.** `uv run python -m pytest
src/tests/sim/unit/test_configurator_applygroup.py
src/tests/sim/unit/test_configure_entry_points.py
src/tests/sim/system/test_configurator_loadbaked.py
src/tests/sim/system/test_composition_root_parity.py
src/tests/unit/test_gen_boot_config_robot_groups.py -v` — 13 passed. Full
`src/tests/sim`/`src/tests/unit` collection also run per sprint policy
(not gating, informational): no new failures attributable to this
ticket's changes beyond the sprint's already-accepted mid-sprint breakage
(ticket-020 pydantic reshape collection error, pre-existing).

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
