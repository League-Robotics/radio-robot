---
id: '001'
title: 'model_tau_lin/model_tau_ang config plumbing: planner.proto, gen_boot_config.py,
  Pilot::configureHeading()'
status: open
use-cases: [SUC-004]
depends-on: []
github-issue: ''
issue: config-as-truth-sim-configure-on-open.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# model_tau_lin/model_tau_ang config plumbing: planner.proto, gen_boot_config.py, Pilot::configureHeading()

## Description

`App::Pilot`'s reference-model time constants (`modelTauLin_`/`modelTauAng_`,
`src/firm/app/pilot.h:342-343`) are currently plain hardcoded member
initializers (`= 0.10f` / `= 0.08f`) with **no config path of any kind** —
`msg::PlannerConfig` has no field for them, so nothing (not the robot JSON,
not the sim, not a wire command) can influence them today. Meanwhile
`data/robots/tovez_nocal.json`'s `control` section already carries
`model_tau_lin: 0.1` / `model_tau_ang: 0.08` (added this session, per the
JSON's own `_neutral_note`: "the SIM-VALIDATED motion values (model-reference
feedback)... there are to be NO tunable defaults in the source") — two keys
sitting unused in the file.

This ticket gives these two values a config path, mirroring the shape
`actuation_lag`/`distance_kp`/`distance_tol` already established
(`planner.proto` field + `gen_boot_config.py` mapping function +
`Pilot::configureHeading()` reads it): it is the **foundation** every other
ticket in this sprint builds on — the sim can't push `model_tau_lin`/
`model_tau_ang` (tickets 002/004/005) until the field exists on the wire
struct, and this is also what makes real-firmware boot pick these values up
from JSON for the first time (a deliberate, low-risk side effect — see
Design Rationale Decision 4 and Open Question 2 in `sprint.md`).

**Do not** add these two fields to the live `PlannerConfigPatch`
(`config.proto`) — see `sprint.md`'s Design Rationale Decision 4: they follow
the `heading_lead_bias`/`plan_lead`/`terminal_lead`/`distance_tol` precedent
of boot-only, no live wire-tuning arm.

## Acceptance Criteria

- [ ] `src/protos/planner.proto`'s `PlannerConfig` message gains
      `float model_tau_lin = 41;` and `float model_tau_ang = 42;` (next
      available field numbers after `distance_tol = 40`), each with a doc
      comment naming the field's purpose (mirrors `App::Pilot`'s reference-
      model lag) and unit tag `[s]`.
- [ ] Generated message code (`src/firm/messages/`, per the project's
      codegen — see `scripts/gen_messages.py` if regeneration is required)
      is regenerated/updated so `msg::PlannerConfig` exposes
      `model_tau_lin`/`model_tau_ang` with `setModelTauLin()`/
      `setModelTauAng()` accessors matching the existing field style (no
      hand-edits to generated headers — fix the generator input, i.e. the
      `.proto` file, and regenerate).
- [ ] `src/scripts/gen_boot_config.py` gains a `model_tau_for_config(cfg)`
      function (mirrors `actuation_lag_for_config()`'s exact shape: read
      `control.model_tau_lin`/`control.model_tau_ang` from the JSON dict,
      falling back to new `MODEL_TAU_LIN_DEFAULT = 0.10` /
      `MODEL_TAU_ANG_DEFAULT = 0.08` module constants — matching `pilot.h`'s
      current hardcoded values exactly, so a robot JSON without these keys
      produces byte-identical boot behavior to today) and wires it into
      `generate()`'s `Config::defaultPlannerConfig()` output
      (`cfg.setModelTauLin(...)` / `cfg.setModelTauAng(...)`).
- [ ] `src/firm/app/pilot.h`'s `configureHeading(const msg::PlannerConfig&
      config)` copies `config.model_tau_lin`/`config.model_tau_ang` into
      `modelTauLin_`/`modelTauAng_` (same style as the existing
      `headingKp_`/`headingKd_`/`distanceKp_` assignments in that method).
      The in-class initializers (`= 0.10f` / `= 0.08f`) stay as-is — they
      remain the harmless pre-`configureHeading()`-call default.
- [ ] `src/sim/sim_harness.h`'s private `makeExecutorConfig()` explicitly
      sets `cfg.model_tau_lin = 0.10f;` / `cfg.model_tau_ang = 0.08f;`
      alongside its other hardcoded literals, so every existing C++ test
      harness that default-constructs `SimHarness` observes byte-for-byte
      identical `Pilot` behavior to before this ticket (SUC-005 — do not
      rely on the new proto fields' zero-value default; a `msg::
      PlannerConfig{}` with these fields unset would silently change
      `modelTauLin_`/`modelTauAng_` to 0.0, which changes `Pilot::tick()`'s
      `alphaLin`/`alphaAng` computation in `pilot.cpp` from "first-order lag
      toward the reference" to "instant, unfiltered reference" — a real
      behavior change for every existing sim scenario/characterization test,
      not a no-op).
- [ ] `Pilot`'s model-lag arithmetic itself (`pilot.cpp` lines ~59-60) is
      untouched — this ticket only threads the two values through config,
      it does not change how they're used.

## Testing

- **Existing tests to run**: `src/tests/sim/unit/test_gen_boot_config_planner.py`
  (must still pass unmodified for every field it already pins); the full
  `uv run python -m pytest` suite (~6 min) — must show zero behavior change
  in any existing `src/tests/sim/` C++ or Python test, since `sim_harness.h`'s
  default construction path is explicitly preserved.
- **New tests to write**: extend (or add a sibling to)
  `test_gen_boot_config_planner.py` pinning `model_tau_for_config()`'s two
  cases — JSON present (`tovez_nocal.json`'s `0.1`/`0.08`, and a synthetic
  cfg with different values to prove the JSON path is actually read, not
  just coincidentally matching the default) and JSON absent (falls back to
  `MODEL_TAU_LIN_DEFAULT`/`MODEL_TAU_ANG_DEFAULT`) — mirroring that file's
  existing present/absent coverage style for `actuation_lag_for_config()`/
  `distance_gains_for_config()`.
- **Verification command**: `uv run python -m pytest src/tests/sim/unit/test_gen_boot_config_planner.py -v`
  for the fast pin, then the full suite: `uv run python -m pytest`.

## Files to touch

- `src/protos/planner.proto` (new fields)
- Generated message code under `src/firm/messages/` (regenerate, do not
  hand-edit — see `scripts/gen_messages.py`)
- `src/scripts/gen_boot_config.py` (`model_tau_for_config()` + wiring into
  `generate()`, two new `*_DEFAULT` constants)
- `src/firm/app/pilot.h` (`configureHeading()`)
- `src/sim/sim_harness.h` (`makeExecutorConfig()` — two new explicit literal
  assignments)
- `src/tests/sim/unit/test_gen_boot_config_planner.py` (new/extended
  regression coverage)
