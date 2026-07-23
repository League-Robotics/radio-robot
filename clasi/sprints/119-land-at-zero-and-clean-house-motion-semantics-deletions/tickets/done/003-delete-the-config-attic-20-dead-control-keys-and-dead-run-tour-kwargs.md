---
id: '003'
title: Delete the config attic (20 dead control.* keys) and dead run_tour kwargs
status: done
use-cases: []
depends-on: []
github-issue: ''
issue: delete-the-config-attic-and-dead-tour-kwargs.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Delete the config attic (20 dead control.* keys) and dead run_tour kwargs

## Description

"Config as source of truth" has become config-as-attic: 20 `control.*`
keys survive in `data/robots/*.json` and the pydantic schema with no
living consumer (115-003 deleted every planner/executor/pilot consumer).
Each dead key is a future agent's invitation to "wire it back up."

**Verified against the tree (2026-07-23, post-118, via direct JSON
parse)** — all 20 keys are still present today in the `control` object
of `data/robots/tovez.json` and `data/robots/tovez_nocal.json`; 19 of the
20 in `data/robots/togov.json` (`turn_gate` absent there, as the issue
itself already notes):

```
heading_kp, heading_kd, heading_source, heading_dwell_tol_deg,
heading_dwell_rate_dps, heading_lead_bias, plan_lead, terminal_lead,
distance_kp, distance_tol, actuation_lag, model_tau_lin, model_tau_ang,
turn_gate, arrive_dwell, arrive_tol_mm, sync, min_speed, yaw_rate_max,
max_rot_accel_dps2
```

`stop_lead_ms` is confirmed ABSENT from every JSON and from
`config_sync_allowlist.json` (118 ticket 004 already deleted it cleanly)
— the original issue's own "coordinate if sequenced with the land-at-zero
deletion" scope guard is moot; there is nothing left to coordinate.

Evidence anchors (per the issue, unchanged): firmware reads = 0
(`config.proto:134-142` documents the wholesale `PlannerConfigPatch`
deletion); `gen_boot_config.py` mapping functions for these keys deleted
(only vel_gains/output_deadband/reversal_dwell/trackwidth/estimator/
shaper mappers remain); `calibration_kwargs()` emits none of them;
`_ALL_SET_KEYS` contains none; `planner_boot_config_for()` deleted.

## Traps confirmed for the implementer (per the issue)

- Host `PlannerParams.heading_kp/kd` (`planner/model.py`, default 0.4,
  read by `planner/heading.py`) is a SEPARATE namespace with no bridge
  from `ControlConfig` — do not confuse; it stays.
- `arrive_tol_mm` is JSON-only (already absent from the pydantic model);
  `turn_gate` is absent from `togov.json`.
- Any `robot_config.py` block comments claiming `gen_boot_config`
  "already reads every one of these" are stale (predate 115-003) —
  delete with the fields.
- The stray `min_speed` hits in `sensors/motion_monitor.py`/`io/cli.py`
  are unrelated locals/CLI args; leave them.

## Also delete (same sweep)

- `run_tour()`'s four documented-UNUSED kwargs `a_max`, `alpha_max`,
  `cadence`, `inter_leg_settle` (`tour.py`) — full call-site inventory
  must confirm (re-verify at execution time; do not trust the pre-118
  inventory blindly) no caller passes any of them. (`omega_max` is LIVE
  — keep.)
- `DEFAULT_INTER_LEG_SETTLE` (`tour.py`) — the one do-nothing constant,
  co-deleted with its kwarg. The other `DEFAULT_*` constants in that
  block are live; keep them.

## Scope guard

Wire/serialized keys already excluded by the project's naming convention
stay untouched. Schema deletion = pydantic model + `robot_config.schema.json`
+ all three robot JSONs + `config_sync_allowlist.json` entries, in one
commit, with the full suite green — same discipline 118 ticket 004 used
for its own one-field schema deletion.

## Acceptance Criteria

- [x] All 20 keys gone from all three `data/robots/*.json` files, the
      pydantic model, `robot_config.schema.json`, and
      `config_sync_allowlist.json`.
- [x] The four `run_tour()` kwargs + `DEFAULT_INTER_LEG_SETTLE` gone from
      `tour.py`; every `run_tour()` caller re-verified green (do not
      assume the pre-118 call-site inventory still holds).
- [x] Grep gate: no surviving reference to any deleted key outside
      `src/archive/` and closed-sprint records.
- [x] Full `uv run python -m pytest` suite green.
- [x] Sim tour-closure gate and button-acceptance suite green.
- [x] Bench verification is DEFERRED to the phase-B bench session — not
      required to close this ticket.

## Testing

- **Existing tests to run**: `uv run python -m pytest` (full suite);
  every `run_tour()` caller (`test_tour_closure_gate.py`,
  `turn_prediction_capture.py`, TestGUI managed-motion paths); config
  schema validation tests if any exist.
- **New tests to write**: none expected — this is pure deletion with an
  existing full-suite regression net; add a grep-gate check if the
  project's test tooling supports one, otherwise document the grep
  command used for manual verification.
- **Verification command**: `uv run python -m pytest`, plus
  `grep -rn "heading_kp\|heading_kd\|heading_source\|heading_dwell_tol_deg\|heading_dwell_rate_dps\|heading_lead_bias\|plan_lead\|terminal_lead\|distance_kp\|distance_tol\|actuation_lag\|model_tau_lin\|model_tau_ang\|turn_gate\|arrive_dwell\|arrive_tol_mm\|\bsync\b\|min_speed\|yaw_rate_max\|max_rot_accel_dps2" data/ src/host/ --include=*.json --include=*.py`
  (adjust as needed) to confirm no surviving reference outside
  `src/archive/`.
