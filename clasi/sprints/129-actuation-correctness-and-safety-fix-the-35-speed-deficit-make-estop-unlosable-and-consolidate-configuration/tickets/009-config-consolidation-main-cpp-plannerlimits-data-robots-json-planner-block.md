---
id: 009
title: "Config consolidation: main.cpp PlannerLimits \u2192 data/robots/*.json planner\
  \ block"
status: open
use-cases: [SUC-008]
depends-on: []
github-issue: ''
issue:
- 03-main-cpp-constants-move-to-robot-config.md
- 02-move-hard-coded-values-to-configuration.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Config consolidation: main.cpp PlannerLimits → data/robots/*.json planner block

## Description

First of the two "mechanical sweep, run last" tickets. Sequenced after
007 (the calibration work) as the start of that grouping — there is no
longer a direct file collision forcing this order (duty_per_speed is
never removed from config in this sprint), but this remains last of the
firmware-touching work because it is the widest-touching, most mechanical
change and would otherwise collide with any ticket still actively editing
the same files.

`src/firm/main.cpp:316-433` assembles the entire `Motion::PlannerLimits`
struct — including the plant-gain constant (`kPlantGain = 1370.0f`) —
as C++ literals, while surrounding lines source everything else from real
per-robot JSON. This contradicts the sprint-114 convention (config is
fail-closed truth from `data/robots/*.json`, no behavioral defaults baked
into source) and recreates the "one robot's numbers become every robot's"
failure already paid for once (`Config::DriveBootConfig`'s own history).

1. Add a `planner` block to `data/robots/*.json` carrying every value
   currently in `main.cpp`'s literal block (profile ceilings, loop
   timing, settle/rest, plant model, duty-stage PID, trim loop),
   `[unit]`-documented in the same style as sibling blocks. Derived values
   (`velKff = 1/plant_gain`, `velKaff = plant_tau/plant_gain`, `trimKaff =
   plant_tau/2`) computed in the loader from the measured primitives — the
   derivation stays in one reviewed place.
2. Add `Config::defaultPlannerLimits()` mirroring the sibling loaders
   (`defaultDriveConfig()` etc.); replace the literal block in `main()`.
3. **Fail closed**: a robot JSON without the `planner` block does not
   boot into a driveable state — same treatment as sprint 114's
   unconfigured-device rule. `tovez.json`, `tovez_nocal.json`, `togov.json`
   all carry the block; `togov`'s values initially copied and marked
   `"_uncalibrated": true` until its own plant ID runs.
4. Preserve provenance — the measurement history in the deleted comment
   block (plant ID dates, sweep results, limit-cycle warning) moves into
   the JSON `_domain_note`s and/or a short doc, not lost.
5. Wire the block through the existing `config()` push path where live
   tuning already exists (`applyVelGains`/`applyShaperLimits`).

## Acceptance Criteria

- [ ] `main.cpp` contains no numeric planner/tuning literal — `grep` for
      `plannerLimits.` assignments shows only `trackWidth`/
      `velocityFilterWeight` plumbing sourced from other config.
- [ ] Booting with a `planner`-less JSON raises the configured boot fault.
- [ ] Booting `tovez.json` yields byte-identical `PlannerLimits` to
      today's literals — a host-side test comparing loader output to the
      recorded values.
- [ ] Full clean build + `motion_tests` + planner `ctest` suite +
      firmware pytest tiers pass; one square-tour sim run matches
      pre-change closure numbers (no behavior change from the move
      itself).

## Testing

- **Existing tests to run**: `just build-clean` (this ticket touches a
  shared-adjacent config header — clean build required), `motion_tests`,
  planner `ctest`, firmware pytest tiers, `uv run python -m pytest`.
- **New tests to write**: host-side test comparing
  `Config::defaultPlannerLimits()`'s loader output against the recorded
  pre-change literal values (byte-identical assertion); boot-fault test
  for a `planner`-less JSON.
- **Bench verification**: standing bench smoke unchanged (per the
  sprint's end-of-sprint gate); this ticket's own square-tour sim
  comparison is the primary regression check.
- **Verification command**: `uv run pytest`.

## Implementation Plan

- **Approach**: mirror the existing sibling-loader pattern
  (`defaultDriveConfig` etc.) exactly — this is a well-trodden path in
  this codebase, not a new config-loading mechanism.
- **Files to modify**: `data/robots/{tovez,togov,tovez_nocal}.json`,
  `robot_config.schema.json`, `config/robot_config.py`,
  `src/scripts/gen_boot_config.py`, `src/firm/config/boot_config.{h,cpp}`
  (regenerated, never hand-edited), `src/firm/main.cpp`.
- **Documentation updates**: the `_domain_note` provenance text in the
  new JSON block (plant-ID dates, sweep results, limit-cycle warning —
  carried forward verbatim from the deleted `main.cpp` comment block).
