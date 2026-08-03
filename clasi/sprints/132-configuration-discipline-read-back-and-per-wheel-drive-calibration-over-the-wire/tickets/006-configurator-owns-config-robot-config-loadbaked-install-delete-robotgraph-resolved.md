---
id: '006'
title: "Configurator owns Config::Robot — config(), loadBaked(), install(); delete\
  \ RobotGraph::Resolved"
status: open
use-cases:
- SUC-002
depends-on:
- '005'
github-issue: ''
issue: the-configuration-object.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Configurator owns Config::Robot — config(), loadBaked(), install(); delete RobotGraph::Resolved

## Description

Rebuild `App::Configurator` (`src/firm/app/configurator.{h,cpp}`) around
ownership of one `Config::Robot` instance: `loadBaked()` populates it via
ticket 005's baking output; `config()` returns a `const` reference for
read-back; `install()` (no argument) pushes every group to its consumer
once, at boot. Delete `RobotGraph::Resolved` (`boot_wiring.h:185-196`,
`boot_wiring.cpp`'s `resolve()`) — confirmed by direct read this
round that it is a private struct never read again after the constructor
body finishes; its job becomes `Configurator::loadBaked()`.
`boot_wiring.cpp`'s three `install*Calibration()` calls after the
constructor's member-init list
(`installShaperLimits`/`installDriveCalibration`/`installWheelController`/
`installRotationCalibration`) are replaced by `Configurator::install()`'s
fan-out.

**Note**: `applyGroup()`/`applyField()` (the WIRE-facing decode entry
points) are NOT this ticket's scope — tickets 008/012. This ticket's
`install()` is the boot-time fan-out only. Full per-group correctness for
`DRIVE`/`WHEEL_CONTROL`/`OTOS`/`ESTIMATOR` lands in tickets 009/010; this
ticket may port the existing `install*Calibration()` call bodies directly
as a starting point.

## Acceptance Criteria

- [ ] `Config::Robot config_` is a member of `Configurator`; `config()`
      returns `const Config::Robot&`.
- [ ] `loadBaked()` populates `config_` using ticket 005's baking output.
- [ ] `install()` (no-arg) is callable and produces the same net baked
      behavior main.cpp's pre-this-ticket boot sequence produced
      (`installShaperLimits`/`installDriveCalibration`/
      `installWheelController`/`installRotationCalibration`'s combined
      effect) — a behavioral parity check against the pre-ticket boot
      sequence, not byte-identical structs (byte-identical isn't
      required this sprint — see sprint.md Migration Concerns).
- [ ] `RobotGraph::Resolved` no longer exists in `boot_wiring.h`;
      `boot_wiring.cpp`'s `resolve()` is deleted or reduced to nothing.
- [ ] `RobotGraph`'s constructor calls `configurator_.loadBaked()` +
      `configurator_.install()` in place of the old `resolve()` +
      `install*Calibration()` sequence.
- [ ] Compiles under `HOST_BUILD`.

## Testing

- **Existing tests to run**: `composition_root_parity_harness.cpp` is
  NOT required to pass yet at this ticket (byte-identical boot is a
  ticket-018 concern) — but a lighter smoke test (does the sim
  composition root construct without crashing, does a basic Move still
  execute) should pass.
- **New tests to write**: a unit test constructing a `RobotGraph` (or the
  sim composition root) and asserting `Configurator::config()` reflects
  `tovez.json`'s values after boot.
- **Verification command**: `uv run python -m pytest <sim
  composition-root smoke test> -q`.

## Implementation Plan

**Approach**: Read `boot_wiring.h/.cpp` and `configurator.h/.cpp` in full
before starting. Move `resolve()`'s logic into
`Configurator::loadBaked()`; move the four `install*Calibration()` call
bodies into `Configurator::install()`'s per-target dispatch (a
straightforward relocation for this ticket — the traps-2/3 fixes inside
OTOS/ESTIMATOR's dispatch are ticket 010's job).

**Files to modify**: `src/firm/app/configurator.{h,cpp}`,
`src/firm/app/boot_wiring.{h,cpp}`.

**Testing plan**: as above.

**Documentation updates**: `configurator.h`'s file header updated to
describe the new ownership model; `boot_wiring.h`'s `RobotGraph` doc
comment updated to remove references to `Resolved`.
