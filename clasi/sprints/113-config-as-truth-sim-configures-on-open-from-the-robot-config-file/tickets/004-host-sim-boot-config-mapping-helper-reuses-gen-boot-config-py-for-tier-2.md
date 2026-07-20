---
id: '004'
title: 'Host: sim_boot_config mapping helper (reuses gen_boot_config.py for Tier 2)'
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-004
depends-on:
- '001'
github-issue: ''
issue: config-as-truth-sim-configure-on-open.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Host: sim_boot_config mapping helper (reuses gen_boot_config.py for Tier 2)

## Description

Ticket 002 gives the sim a way to *receive* the Tier-2 (boot-only)
`PlannerConfig`/motor-config field set. This ticket computes *what values to
send* from a `RobotConfig` — by **reusing `gen_boot_config.py`'s own pure
mapping functions**, not re-deriving the JSON→field mapping a second time
(the exact bug class this sprint exists to close, one level up from where the
issue found it — see `sprint.md` Design Rationale Decision 2).

`src/scripts/gen_boot_config.py` already has everything needed as
plain, already-tested `cfg: dict -> value(s)` functions: `vel_gains_for_config()`
(for `vel_filt`), `fwd_sign_for_ports()` (for per-port `fwd_sign`),
`heading_gains_for_config()`, `heading_source_for_config()`,
`heading_dwell_for_config()`, `lead_compensation_for_config()`,
`min_speed_for_config()` (already covered by ticket 003's Tier-1 push, but
harmless to also compute here for a complete Tier-2 snapshot), `profile_rot_limits_for_config()`,
`arrive_dwell_for_config()`, `actuation_lag_for_config()`,
`distance_gains_for_config()`, and (after ticket 001)
`model_tau_for_config()`. `A_MAX_DEFAULT`/`A_DECEL_DEFAULT`/`V_BODY_MAX_DEFAULT`/
`J_MAX_DEFAULT`/`YAW_JERK_MAX_DEFAULT` have no per-robot JSON override
mapping function at all today (`generate()` uses the module constants
directly) — this helper does the same: reads the module constants directly
for those five.

`src/tests/sim/unit/test_gen_boot_config_planner.py` already establishes the
precedent for importing `gen_boot_config.py` from Python test/host code
outside `src/scripts/`: `sys.path.insert(0, str(_SCRIPTS_DIR)); import
gen_boot_config as gbc`. Use the identical pattern.

## Acceptance Criteria

- [x] A new module, e.g. `src/host/robot_radio/calibration/sim_boot_config.py`,
      imports `gen_boot_config` via the established `sys.path` pattern
      (comment at the import site: this module takes a runtime dependency on
      `src/scripts/gen_boot_config.py` staying import-safe — pure functions,
      no argv/stdout side effects at import time — matching what's already
      true today).
- [x] A function `planner_boot_config_for(config) -> dict[str, float | int]`
      takes a `RobotConfig` (or the raw JSON dict `gen_boot_config.py`'s own
      functions expect — resolve the exact shape by checking how
      `RobotConfig` differs from the raw dict `cfg` these functions take;
      convert as needed) and returns every Tier-2 `PlannerConfig` scalar this
      sprint covers: `a_max`, `a_decel`, `v_body_max`, `yaw_rate_max`,
      `yaw_acc_max`, `j_max`, `yaw_jerk_max`, `min_speed`, `heading_kp`,
      `heading_kd`, `arrive_dwell`, `heading_source` (as the wire int enum
      value, not the string — resolve via `gen_boot_config.py`'s
      `_HEADING_SOURCE_WIRE_NAMES`-equivalent mapping, or add a small
      string->int helper alongside), `heading_dwell_tol`, `heading_dwell_rate`,
      `heading_lead_bias`, `plan_lead`, `terminal_lead`, `actuation_lag`,
      `distance_kp`, `distance_tol`, `model_tau_lin`, `model_tau_ang` — by
      calling `gen_boot_config`'s existing functions, not reimplementing any
      of them.
- [x] A function `motor_boot_config_for(config, port) -> dict[str, float |
      int]` returns `{"vel_filt_alpha": ..., "fwd_sign": ...}` for the given
      port (1=left, 2=right), via `vel_gains_for_config()` (take just the
      `filt` element) and `fwd_sign_for_ports()` (index by port).
- [x] Every value returned matches, field-for-field, what `gen_boot_config.py`
      would bake into `Config::defaultPlannerConfig()`/`defaultMotorConfigs()`
      for the identical input JSON — this is the crux of the sprint's parity
      goal and is directly asserted by ticket 007's tests.
- [x] No JSON-to-value mapping logic is duplicated in the new module — every
      mapping decision (including fallback-to-default behavior) lives in
      `gen_boot_config.py` and is only *called*, never *re-expressed*, here.

## Testing

- **Existing tests to run**: `src/tests/sim/unit/test_gen_boot_config_planner.py`
  and `test_gen_boot_config_fwd_sign.py` — confirm this ticket's import of
  `gen_boot_config` doesn't perturb its own standalone-script behavior
  (e.g. no accidental top-level side effect introduced by how it's imported
  from a new location).
- **New tests to write**: unit tests for `planner_boot_config_for()`/
  `motor_boot_config_for()` against both `tovez.json` and `tovez_nocal.json`
  fixtures, asserting each returned value equals what
  `gen_boot_config.py`'s own functions independently compute for the same
  input (a direct call-through comparison, not a hardcoded expected-value
  table, so the test can't silently drift from the generator it's supposed
  to mirror) — plus one fallback case (a minimal `cfg` missing the `control`
  section entirely) proving every field still resolves to its documented
  default, matching `gen_boot_config.py`'s own no-JSON fallback path.
- **Verification command**: `uv run python -m pytest src/host/... -k sim_boot_config -v`
  (adjust path to wherever the new test file lands), then the full suite.

## Files to touch

- New: `src/host/robot_radio/calibration/sim_boot_config.py`
- New: a unit test file for it (e.g.
  `src/tests/unit/test_sim_boot_config.py` or co-located under
  `src/tests/sim/unit/` — match whichever convention the ticket 003 test
  file ends up using, for consistency).

## Depends On

- Ticket 001 (needs `gen_boot_config.py`'s `model_tau_for_config()` to
  exist).
