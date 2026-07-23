---
status: pending
filed: 2026-07-22
filed_by: "team-lead (turn-execution review \xA76-3/\xA76-4/\xA76-7, consumer audit\
  \ verified 2026-07-22)"
related:
- land-at-zero-completion-delete-stop-lead.md
- turn-error-characterization-postcompensation-tests-need-rewrite-after-lead-deletion.md
sprint: '119'
---

# Delete the config attic (20 dead control.* keys) and the dead run_tour kwargs

## Description

"Config as source of truth" has become config-as-attic: 20 `control.*` keys
survive in `data/robots/*.json` and the pydantic schema whose every consumer
was deleted by 115-003 (planner/executor/pilot excision). Each dead key is a
future agent's invitation to "wire it back up." Full consumer audit
(2026-07-22) confirmed **NO living consumer** for any of:

`heading_kp, heading_kd, heading_source, heading_dwell_tol_deg,
heading_dwell_rate_dps, heading_lead_bias, plan_lead, terminal_lead,
distance_kp, distance_tol, actuation_lag, model_tau_lin, model_tau_ang,
turn_gate, arrive_dwell, arrive_tol_mm, sync, min_speed, yaw_rate_max,
max_rot_accel_dps2`

Evidence anchors: firmware reads = 0 (`config.proto:134-142` documents the
wholesale PlannerConfigPatch deletion); `gen_boot_config.py` mapping fns for
these keys deleted (only vel_gains/output_deadband/reversal_dwell/trackwidth/
estimator/shaper mappers remain, `:347-471`); `calibration_kwargs()` emits
none of them (`push.py:76-181`); `_ALL_SET_KEYS` contains none
(`protocol.py:627-671`); `planner_boot_config_for()` deleted
(`sim_boot_config.py:91-100`).

**Traps confirmed for the implementer:**
- Host `PlannerParams.heading_kp/kd` (`planner/model.py`, default 0.4, read
  by `planner/heading.py:76,116`) is a SEPARATE namespace with no bridge from
  `ControlConfig` — do not confuse; it stays.
- `arrive_tol_mm` is JSON-only (already absent from the pydantic model);
  `turn_gate` is absent from togov.json.
- The `robot_config.py:214-284` block comments claiming gen_boot_config
  "already reads every one of these" are stale (predate 115-003) — delete
  with the fields.
- The stray `min_speed` hits in `sensors/motion_monitor.py` / `io/cli.py`
  are unrelated locals/CLI args; leave them.

## Also delete (same sweep)

- `run_tour()`'s four documented-UNUSED kwargs `a_max`, `alpha_max`,
  `cadence`, `inter_leg_settle` (`tour.py:611-617`): full call-site inventory
  shows NO caller passes any of them, and the retention comment's named
  caller `tests/bench/tour_bench_run.py` does not exist. (`omega_max` is
  LIVE — keep.)
- `DEFAULT_INTER_LEG_SETTLE` (`tour.py:247-255`) — the one do-nothing
  constant, co-deleted with its kwarg. The other `DEFAULT_*` in that block
  are live; keep.

## Scope guard

Wire/serialized keys already excluded by the naming convention stay
untouched. `stop_lead_ms` and the `_estimator_note`/`_shaper_note` blocks
are handled by `land-at-zero-completion-delete-stop-lead.md` — do not
double-delete; coordinate if sequenced in the same sprint. Schema deletion =
pydantic model + robot_config.schema.json + all three robot JSONs +
config_sync_allowlist.json entries, in one commit, with the full suite green.

## Acceptance

- All 20 keys gone from JSONs, pydantic model, JSON schema, allowlist.
- The four kwargs + constant gone from tour.py; all run_tour callers still
  green.
- Grep gate: no surviving reference to any deleted key outside src/archive/
  and closed-sprint records.
- Full pytest suite green; button-acceptance suite green.
