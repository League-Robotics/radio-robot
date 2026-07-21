---
id: '002'
title: 'Eliminate gen_boot_config.py behavioral fallback defaults: extend robot JSON
  schema/profiles, hard-fail build on missing required key'
status: done
use-cases:
- SUC-002
depends-on: []
github-issue: ''
issue: config-as-truth-completion-no-defaults-fail-closed-version-erase.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Eliminate gen_boot_config.py behavioral fallback defaults: extend robot JSON schema/profiles, hard-fail build on missing required key

## Description

Delete every behavioral `*_DEFAULT` fallback constant from
`src/scripts/gen_boot_config.py` and make the generator fail the build
loudly when the active robot JSON is missing a key this ticket makes
required. Migrate the fallback values into the JSON schema and all three
shipped robot profiles first (value-preserving — same numbers), so no
robot's compiled behavior changes on its next reflash unless its own JSON
already diverges.

## Context

`gen_boot_config.py` documents its own current design choice in its module
docstring and in `src/firm/config/DESIGN.md` §3: "Missing/bad robot JSON
degrades to bench defaults, not a build failure... deliberate." This ticket
reverses that choice for every *behavioral* field (see sprint.md's
Architecture Boundary list) while leaving every *structural* constant
(`K_MOTOR_COUNT`, `LEFT_PORT`/`RIGHT_PORT`, `polled_for_ports()`, and the
`TRAVEL_CALIB_PLACEHOLDER`/`FWD_SIGN` placeholders for the two non-drive
ports) exactly as-is — those are not behavioral tunables and are out of
scope.

`data/robots/robot_config.schema.json`'s `control` object
(`additionalProperties: false`) is already stale relative to reality:
`tovez_nocal.json` already carries `heading_kp`/`distance_kp`/
`model_tau_lin`/`model_tau_ang`/`actuation_lag`/`min_speed` etc., none of
which the schema declares. This is non-blocking today only because
`robot_config.py` validates via its own `pydantic` model, not this schema
file — but this ticket should repair the schema properly while adding the
new required keys, not leave a still-incomplete schema behind.

## Approach

1. **Enumerate every field to migrate** (current `gen_boot_config.py` state,
   as of sprint 113): `VEL_KP`, `VEL_KI`, `VEL_KFF`, `VEL_IMAX`, `VEL_KAW`,
   `VEL_FILT_ALPHA`, `TRACKWIDTH_DEFAULT`, `OTOS_OFFSET_X_DEFAULT`,
   `OTOS_OFFSET_Y_DEFAULT`, `OTOS_OFFSET_YAW_DEFAULT`,
   `OTOS_LINEAR_SCALE_DEFAULT`, `OTOS_ANGULAR_SCALE_DEFAULT`,
   `A_MAX_DEFAULT`, `A_DECEL_DEFAULT`, `V_BODY_MAX_DEFAULT`,
   `YAW_RATE_MAX_DEFAULT`, `YAW_ACC_MAX_DEFAULT`, `J_MAX_DEFAULT`,
   `YAW_JERK_MAX_DEFAULT`, `HEADING_KP_DEFAULT`, `HEADING_KD_DEFAULT`,
   `HEADING_SOURCE_DEFAULT`, `HEADING_DWELL_TOL_DEG_DEFAULT`,
   `HEADING_DWELL_RATE_DPS_DEFAULT`, `HEADING_LEAD_BIAS_DEFAULT`,
   `PLAN_LEAD_DEFAULT`, `TERMINAL_LEAD_DEFAULT`, `ACTUATION_LAG_DEFAULT`,
   `DISTANCE_KP_DEFAULT`, `DISTANCE_TOL_DEFAULT`, `MODEL_TAU_LIN_DEFAULT`,
   `MODEL_TAU_ANG_DEFAULT`, `ARRIVE_DWELL_DEFAULT`, `MIN_SPEED_DEFAULT`.
   (Explicitly **not** migrated: `K_MOTOR_COUNT`, `LEFT_PORT`, `RIGHT_PORT`,
   `TRAVEL_CALIB_PLACEHOLDER`, `FWD_SIGN` — structural/documented-exception,
   stay as Python constants.)

2. **For each field**, check whether `data/robots/tovez_nocal.json` already
   carries the corresponding key (many already do — `heading_kp`,
   `distance_kp`, `distance_tol`, `actuation_lag`, `model_tau_lin`,
   `model_tau_ang`, `min_speed`, `arrive_dwell`, `yaw_rate_max`,
   `max_rot_accel_dps2`, `vel_*`, `geometry.trackwidth`,
   `geometry.odometry_offset_mm` are already present). For each **missing**
   key (at minimum: `heading_source`, `heading_dwell_tol_deg`,
   `heading_dwell_rate_dps`, `heading_lead_bias`, `plan_lead`,
   `terminal_lead`, `a_max`, `a_decel`, `v_body_max`, `j_max`,
   `yaw_jerk_max`, `calibration.otos_linear_scale`,
   `calibration.otos_angular_scale` — `control.output_deadband`/
   `control.reversal_dwell_ms` are ticket 003's to add), add it to
   `tovez_nocal.json`, `tovez.json`, and `togov.json`, each seeded with
   **exactly** the current `*_DEFAULT` Python value (value-preserving —
   verify by diffing `boot_config.cpp` before/after this ticket for the
   active profile: it must be byte-identical).

3. **Update `data/robots/robot_config.schema.json`**: add every migrated key
   to the `control` object's `properties` (it currently only documents the
   legacy text-protocol vocabulary — add proper `description`/`type`
   entries for the binary-tree fields, matching the existing style but with
   accurate current semantics). Add `otos_linear_scale`/`otos_angular_scale`
   to the `calibration` object if not already declared.

4. **Rewrite each `*_for_config()` helper** in `gen_boot_config.py`: replace
   `_get(ctrl, "key", default=X_DEFAULT)` with a call that fails loudly
   (raise, or a clear `sys.exit(1)` naming the key and the resolved JSON
   path — match whatever error-handling convention `load_robot_config()`
   already uses) when the key is absent. Delete each `X_DEFAULT`
   module-level constant once nothing references it.

5. **Re-run codegen** against all three profiles and confirm `boot_config.cpp`
   is byte-identical to pre-ticket output for each.

## Files to Touch

- `src/scripts/gen_boot_config.py`
- `data/robots/robot_config.schema.json`
- `data/robots/tovez_nocal.json`, `data/robots/tovez.json`,
  `data/robots/togov.json`
- (Regenerated, not hand-migrated:) `src/firm/config/boot_config.cpp`

## Acceptance Criteria

- [x] Every field listed in Approach step 1 has no remaining `*_DEFAULT`
      Python constant in `gen_boot_config.py`.
- [x] Each of the three shipped robot JSONs is independently sufficient to
      generate `boot_config.cpp` (no `ROBOT_CONFIG` fallback needed).
- [x] Deleting any one required key from any one of the three JSONs (tested
      independently, one at a time) causes `gen_boot_config.py` to exit
      non-zero with a message naming the missing key and the JSON path —
      not a silently-generated placeholder file.
- [x] `boot_config.cpp` generated from each of the three JSONs post-ticket is
      byte-identical to the pre-ticket generated output for that same JSON
      (value-preserving migration, verified by diff, not assumed).
- [x] `data/robots/robot_config.schema.json`'s `control` object declares
      every field this ticket adds, plus the pre-existing-but-previously-
      undeclared binary-tree fields found stale during sprint 114 planning
      (`heading_kp`, `heading_kd`, `distance_kp`, `distance_tol`,
      `actuation_lag`, `model_tau_lin`, `model_tau_ang`, `min_speed`,
      `arrive_dwell`).
- [x] `K_MOTOR_COUNT`/`LEFT_PORT`/`RIGHT_PORT`/`TRAVEL_CALIB_PLACEHOLDER`/
      `FWD_SIGN` are untouched (structural, out of scope).

## Completion Notes (2026-07-20)

- 33 `*_DEFAULT` module constants deleted (5 stayed: `K_MOTOR_COUNT`,
  `LEFT_PORT`, `RIGHT_PORT`, `TRAVEL_CALIB_PLACEHOLDER`, `FWD_SIGN` --
  structural, byte-identical values confirmed unchanged).
- New `MissingRobotConfigKeyError`/`_require()` in `gen_boot_config.py`;
  every `*_for_config()` helper rewritten to hard-fail; two brand-new
  mapping functions added (`motion_limits_for_config()` for
  a_max/a_decel/v_body_max/j_max/yaw_jerk_max -- previously not wired to
  JSON AT ALL; `trackwidth_for_config()`). `heading_dwell_for_config()`
  rewired to actually read `control.heading_dwell_tol_deg`/
  `heading_dwell_rate_dps` for the first time (previously hardcoded,
  ignored `cfg` entirely).
- Value-preserving migration verified two ways: (1) a byte-identical-output
  regression pin (`test_gen_boot_config_required_keys.py`) diffing fresh
  `generate()` output for all three profiles against golden snapshots
  captured from the pre-ticket generator/JSON, committed as
  `src/tests/sim/unit/fixtures/boot_config_golden_*.cpp`; (2) a manual
  before/after `diff` during implementation, also byte-identical.
- Downstream blast radius beyond the ticket's own Files-to-Touch, required
  to avoid breaking existing behavior/tests (not scope creep -- mechanical
  consequences of deleting module-level constants other code imported):
  `src/host/robot_radio/calibration/sim_boot_config.py` (now calls
  `motion_limits_for_config()` instead of reading the deleted constants
  directly); `src/host/robot_radio/config/robot_config.py`'s
  `ControlConfig` gained 7 new `Optional[float]` fields
  (`heading_dwell_tol_deg`/`heading_dwell_rate_dps`/`a_max`/`a_decel`/
  `v_body_max`/`j_max`/`yaw_jerk_max`) -- without these the pydantic model
  would silently drop the newly-required JSON keys at parse time, hard-
  failing `configure_from_robot()` (the real TestGUI sim-configure path)
  for every robot; `src/scripts/config_sync_allowlist.json` gained 7
  matching `pydantic-field-no-patch` entries (Tier-2 boot-only, no live SET
  key, mirroring the existing `control.actuation_lag`-style entries).
  Several existing tests asserting the OLD fallback-to-default behavior
  were rewritten to assert the NEW hard-fail behavior instead (their
  premise was the exact thing this ticket reverses):
  `test_gen_boot_config_planner.py`, `test_gen_boot_config_otos.py`,
  `test_gen_boot_config_fwd_sign.py`, `test_sim_boot_config.py`,
  `test_sim_boot_config_parity.py`. `test_calibration_kwargs.py`'s
  tovez.json snapshot gained two new live `SET minSpeed=16`/
  `SET distanceKp=8` entries (tovez.json previously omitted these two
  Tier-2 fields entirely; both now equal the exact value the firmware
  already boots with, so this is a no-op push, not a behavior change).
  `test_turn_error_characterization.py`'s import of the three deleted
  `HEADING_LEAD_BIAS_DEFAULT`/`PLAN_LEAD_DEFAULT`/`TERMINAL_LEAD_DEFAULT`
  constants was replaced with a call to `lead_compensation_for_config()`
  against `tovez_nocal.json` (the currently-active profile) -- same
  numeric values, now sourced from JSON instead of an importable constant.
  `src/firm/config/boot_config.cpp` regenerated against the active robot
  (`tovez_nocal.json`); this also happened to correct a PRE-EXISTING
  staleness unrelated to this ticket (the committed file predated several
  `tovez_nocal.json` control-value edits from earlier sprint 114 work --
  confirmed via `git diff`, not introduced by this ticket's own migration).
- 3 pre-existing failures in `test_turn_error_characterization.py`
  (`test_postcompensation_realistic_holds_ticket_009_bar[30.0/170.0]`,
  `test_at_rest_residual_is_not_rate_dependent`) confirmed via `git stash`
  to fail identically before and after this ticket's changes -- unrelated
  sim/firmware state on this `pid-debugging` branch, not caused by or
  fixable within this ticket's scope.

## Testing

- **Existing tests to run**: `test_gen_boot_config_planner.py` (or
  equivalent existing regression pin) and
  `src/tests/sim/system/test_sim_boot_config_parity.py` (reads both
  `tovez_nocal.json` and `tovez.json` — must still pass, confirming both
  remain complete and mutually parity-correct with the sim's own Tier-2
  path).
- **New tests to write**: one parametrized test case per migrated field —
  delete/omit that field from a temporary copy of a complete JSON, assert
  the generator exits non-zero with an informative message; a
  byte-identical-output regression pin comparing pre/post-ticket
  `boot_config.cpp` for all three profiles.
- **Verification command**:
  `uv run python -m pytest src/tests/sim/system/test_sim_boot_config_parity.py -v -s`,
  then full suite `uv run python -m pytest`.
