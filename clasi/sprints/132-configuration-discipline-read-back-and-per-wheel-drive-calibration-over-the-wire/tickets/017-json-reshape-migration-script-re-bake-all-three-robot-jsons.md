---
id: '017'
title: "JSON reshape — migration script + re-bake all three robot JSONs"
status: open
use-cases:
- SUC-008
depends-on:
- '015'
- '016'
github-issue: ''
issue: the-configuration-object.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# JSON reshape — migration script + re-bake all three robot JSONs

## Description

The scheduled-last ticket, exactly as directed by the stakeholder. Write
a one-time migration script (`src/scripts/`, not part of the ongoing
build pipeline afterward) that reshapes `tovez.json`/`togov.json`/
`tovez_nocal.json` from the old 13-section shape into `Config::Robot`'s
grouped shape (`Geometry`/`Motors`/`Drive`/`WheelControl`/`Planner`/
`Otos`/`Estimator`, plus the untouched host-only `Identity`/`Connection`/
`Vision` and the non-wire-addressable sections that stay in the file
under the one-file rule but never cross the wire — wheels/encoders/
gripper/peripherals). Drop the 17 dead `control` keys identified in the
issue's audit in the same pass (re-verify the list is still accurate
against the CURRENT `control` section before dropping — do not blindly
trust the issue's original count without a fresh check). Retarget
`gen_boot_config.py`'s (ticket 005's retargeted baking) field paths from
`control.*` to the new grouped section names — this is the point where
the OLD-JSON-path bridge every prior ticket relied on gets removed and
replaced with paths matching the NEW shape.

**This is the moment host-side JSON loading — broken since ticket 002's
pydantic model regenerated in the new shape — starts working again.**

## Acceptance Criteria

- [ ] A migration script exists (one-time use, documented as such) that
      rewrites a robot JSON from the old shape to the new grouped shape.
- [ ] `tovez.json`, `togov.json`, `tovez_nocal.json` are all migrated and
      committed in the new shape.
- [ ] The 17 dead `control` keys (re-verified against the current file,
      not assumed) are absent from all three reshaped files.
- [ ] All three reshaped files validate against the generated pydantic
      model (ticket 002/016's `extra='forbid'` model) with ZERO errors.
- [ ] `gen_boot_config.py`'s (ticket 005's) field paths are retargeted
      from `control.*`/old section names to the new grouped section
      names — a full re-check against every `_require()` call site, not
      a partial pass.
- [ ] A fresh ARM build from a reshaped JSON succeeds and boots (a
      sim-level composition-root construction at minimum; full ARM
      toolchain build if feasible in this ticket's environment).
- [ ] `active_robot.json`/whatever currently points at `tovez.json` still
      resolves correctly post-reshape (no dangling pointer to an old key
      path).

## Testing

- **Existing tests to run**: previously-xfail/skip-marked tests from
  tickets 005/016 (expected to fail against the old-shape JSON) should
  now PASS — go back and un-mark them.
- **New tests to write**: a full round-trip test (load reshaped JSON ->
  pydantic validates -> bake -> `Config::Robot` -> compare against
  pre-reshape baked values for behavioral equivalence, not byte-identical
  struct layout).
- **Verification command**: `uv run python -m pytest <robot_config +
  gen_boot_config test paths> -q`, plus an ARM build invocation.

## Implementation Plan

**Approach**: Write the migration script against `tovez.json` first (the
primary bench robot), verify its output validates, then apply to
`togov.json`/`tovez_nocal.json`. Retarget `gen_boot_config.py` last, once
all three files are confirmed in the new shape.

**Files to create**: `src/scripts/` migration script (one-time).

**Files to modify**: `data/robots/tovez.json`, `data/robots/togov.json`,
`data/robots/tovez_nocal.json`, `src/scripts/gen_boot_config.py` (path
retargeting).

**Testing plan**: as above.

**Documentation updates**: a short note in the migration script's own
docstring recording that it was run once, on this date, against these
three files — so a future reader doesn't try to re-run it against
already-migrated files.
