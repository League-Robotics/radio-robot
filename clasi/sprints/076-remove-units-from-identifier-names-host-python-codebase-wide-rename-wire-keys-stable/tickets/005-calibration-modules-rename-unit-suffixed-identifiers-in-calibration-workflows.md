---
id: '005'
title: 'Calibration modules: rename unit-suffixed identifiers in calibration workflows'
status: open
use-cases:
- SUC-004
depends-on:
- '002'
github-issue: ''
issue: remove-units-from-identifier-names-host-python.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Calibration modules: rename unit-suffixed identifiers in calibration workflows

## Description

`host/robot_radio/calibration/helpers.py`, `linear.py`, `angular.py`,
`push.py`, and `fit_sim_error_model.py` push calibration values to firmware
(`SET`) and fit the sim error model (`SIMSET`). This subsystem depends only
on `robot/protocol.py` (`parse_tlm`, `TLMFrame` — ticket 002, already
renamed) and owns the one host-side file (`fit_sim_error_model.py`) with
direct `SIMSET`-key dict literals, plus the `sim_prefs.py`-adjacent
two-layer key pattern that ticket 008 will also need to understand.

Renames (per Step 5): `calibration/angular.py` (`target_deg`/`otos_deg`/
`gt_deg`/`cam_deg`/`achieved_deg` → bare names with `# [deg]`);
`calibration/linear.py` (`actual_mm`/`otos_x_mm` → bare names with
`# [mm]`); `calibration/fit_sim_error_model.py` (`total_ms`/
`sample_period_ms` → bare names with `# [ms]`; `SIMSET_BOUNDS`/
`DEFAULT_CANDIDATE_KEYS` dict-key *strings* untouched, see Exclusion Table
below); `calibration/push.py` (`left_mm_per_deg`/`right_mm_per_deg` →
**recommended** `wheel_travel_calib_left`/`wheel_travel_calib_right` with
`# [mm/deg]`, mirroring 071's own `wheelTravelCalibL/R` derived-unit-name
choice on the firmware side — a recommendation, not a requirement; per
Open Question 5, choose a different descriptive name if this collides with
anything in this file).

`calibration/helpers.py` is already clean (zero unit-suffix hits, Step 1)
but is in this ticket's review scope to confirm.

Total scope: 175 rename-eligible occurrences (Step 3).

## Wire-Compatibility Exclusions Relevant to This Ticket

(Restated from `architecture-update.md`'s Wire-Compatibility Exclusion
Table — do **not** rename any of the following in this ticket's files.)

- Every `SET`/`GET` wire-key string 071 already confirmed unit-free (`ml`,
  `mr`, `tw`, `rotSlip`, `odomOffX/Y`, `odomYaw`, `sTimeout`, etc.) — these
  appear as literal strings in `push.py`'s command builders. Only the
  surrounding Python variable/parameter names that hold the *value* are
  renamed.
- `set_config(**kwargs)` call sites' keyword-argument names (e.g.
  `set_config(ml=..., mr=...)`) — the kwarg name **is** the wire key at
  that call site. Re-verify before editing any `set_config(...)` call in
  this ticket's files that no target identifier collides with a live kwarg
  name (this pass found zero such collisions).
- `SIMSET_BOUNDS`, `DEFAULT_CANDIDATE_KEYS`, and step-size dict keys (e.g.
  `"trackwidthMm"`) in `fit_sim_error_model.py` — direct wire-key-as-dict-key,
  single layer, matching `SimCommands.cpp`'s `kSimRegistry[]` pattern.
  **Exclude wholesale.**

## Hard Contract (applies to this and every sprint 076 ticket)

- **Pure rename — no behavioral change.** Calibration fit/replay
  round-trips must produce numerically identical results.
- **Every renamed declaration carries a `# [unit]` comment.**
- **Wire keys are STABLE** per the exclusions above — every `SET`/`SIMSET`
  key string literal in `calibration/` stays byte-identical (diff-verify).
- **Full suite green throughout**: `uv run python -m pytest -q` remains
  **2682 passed, 0 failed**.
- **Cross-cutting kwargs**: any call into `robot/protocol.py` using a
  ticket-002-renamed keyword argument (e.g. `read_timeout=`) must already
  use the converged name; fix any stale one found here.
- **Ignore environmental `data/robots` drift.**

## Acceptance Criteria

- [ ] `calibration/angular.py`: `target_deg`/`otos_deg`/`gt_deg`/`cam_deg`/
      `achieved_deg` → bare names with `# [deg]`.
- [ ] `calibration/linear.py`: `actual_mm`/`otos_x_mm` → bare names with
      `# [mm]`.
- [ ] `calibration/fit_sim_error_model.py`: `total_ms`/`sample_period_ms` →
      bare names with `# [ms]`; `SIMSET_BOUNDS`/`DEFAULT_CANDIDATE_KEYS`
      dict-key strings are byte-identical to pre-076 (diff-confirm).
- [ ] `calibration/push.py`: `left_mm_per_deg`/`right_mm_per_deg` renamed
      to a descriptive quantity name with `# [mm/deg]` (recommended:
      `wheel_travel_calib_left`/`wheel_travel_calib_right`, per Open
      Question 5 an implementation-time judgment call, not a hard
      requirement).
- [ ] `calibration/helpers.py` is confirmed to remain clean (zero
      unit-suffixed identifiers) — no edit expected.
- [ ] Every `SET`/`SIMSET` key string literal in `calibration/` is
      unchanged (diffed against pre-076): `ml`, `mr`, `tw`, `rotSlip`,
      `odomOffX/Y`, `odomYaw`, and every `SIMSET_BOUNDS`/
      `DEFAULT_CANDIDATE_KEYS`/step-size dict key (`trackwidthMm`, etc.).
- [ ] `NezhaProtocol.set_config(**kwargs)` call sites in this ticket's
      files that pass a wire key as a keyword argument are **not** touched.
- [ ] `tests/simulation/unit/test_calibration_push.py`,
      `test_calibrate_linear.py`, `test_calibration_helpers.py` (per
      `usecases.md` SUC-004) pass unchanged.
- [ ] Hard Contract above holds.

## Testing

- **Existing tests to run**: `tests/simulation/unit/test_calibration_push.py`,
  `tests/simulation/unit/test_calibrate_linear.py`,
  `tests/simulation/unit/test_calibration_helpers.py`.
- **New tests to write**: none required — pure rename.
- **Verification command**: `uv run python -m pytest -q` (confirm 2682
  passed, 0 failed).

## Implementation Plan

**Approach**: Rename file-by-file, treating `fit_sim_error_model.py`'s
dict-key literals as a hard exclusion boundary throughout.

1. `calibration/angular.py`, `calibration/linear.py` — rename
   unit-suffixed identifiers per the mapping above.
2. `calibration/fit_sim_error_model.py` — rename local
   variables/parameters only; leave every `SIMSET_BOUNDS`/
   `DEFAULT_CANDIDATE_KEYS` dict key exactly as-is.
3. `calibration/push.py` — rename `left_mm_per_deg`/`right_mm_per_deg`
   (recommended target name above); leave every `SET` command's wire-key
   string literal exactly as-is.
4. Confirm `calibration/helpers.py` needs no edit.
5. Grep this file set for every `SET`/`SIMSET` key string to confirm none
   were altered, and for every renamed identifier's old name to confirm no
   internal call site was missed.
6. Run the three named calibration unit tests, then the full suite.

**Files to create/modify**:
- `host/robot_radio/calibration/linear.py`
- `host/robot_radio/calibration/angular.py`
- `host/robot_radio/calibration/push.py`
- `host/robot_radio/calibration/fit_sim_error_model.py`
- `host/robot_radio/calibration/helpers.py` — reviewed only, no edit
  expected.

**Testing plan**: Run
`tests/simulation/unit/test_calibration_push.py`,
`test_calibrate_linear.py`, `test_calibration_helpers.py` individually,
then `uv run python -m pytest -q` and confirm the 2682 baseline holds.

**Documentation updates**: None in this ticket.
