---
id: '006'
title: Sim-Errors panel verification
status: done
use-cases:
- SUC-007
depends-on: []
github-issue: ''
issue: host-testgui-full-revival.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sim-Errors panel verification

## Description

The Sim Errors panel (`host/robot_radio/testgui/__main__.py`'s
`sim_errors_group`, `_on_sim_errors_apply`, `_on_sim_errors_from_cal`) and
`sim_prefs.py`'s profile<->setter map (083's territory, unchanged this
sprint) already implement Apply (persist + live-apply the panel's
error-injection profile via `SimTransport.apply_error_profile`) and "From
Calibration" (populate the panel with the inverse of the active robot's
calibration, so the firmware's baked-in correction and the ideal sim
plant's lack of scrub cancel, then run Apply's same save/apply path once).
This is sim-only fidelity tooling — a distinct consumer (the ctypes
error-injection ABI) from calibration push (ticket 005, which talks to
firmware `SET`/OTOS) even though both read the same robot calibration data.

This code predates the greenfield rebuild. This ticket ports the three
un-ported test files covering it and verifies the panel against the sim,
fixing anything a real run surfaces.

## Acceptance Criteria

- [x] `tests_old/testgui/test_sim_errors_panel.py`,
      `test_sim_errors_from_cal_button.py`, and
      `test_sim_errors_from_calibration.py` are ported to
      `tests/testgui/`, updated for any API drift, and pass under
      `QT_QPA_PLATFORM=offscreen`.
- [x] "From Calibration" with an uncalibrated robot ("tovez nocal") yields
      the all-neutral (zero-error) panel.
- [x] "From Calibration" with a calibrated robot yields the documented
      inverse mapping (`rotational_slip` -> body rot scrub,
      `geometry.trackwidth` -> trackwidth, every other knob neutral).
- [x] "From Calibration" is confirmed to reuse `_on_sim_errors_apply`'s
      save/apply path exactly once each (`sim_prefs.save_sim_error_profile`
      and, when connected, `SimTransport.apply_error_profile`) — not a
      second, independently-written apply path.
- [x] A missing or partial robot config falls back to neutral per-field
      with a logged `[WARN]`, never raising.
- [x] The three no-ctypes-backing noise fields (`sim_err_encoder_mm`,
      `sim_err_otos_linear`, `sim_err_otos_yaw` — 083's documented
      no-effect fields) are left untouched by "From Calibration."
- [x] Note during implementation whether
      `test_sim_errors_from_cal_button.py` and
      `test_sim_errors_from_calibration.py` fully overlap (planning-time
      Open Question 3) — do not delete either without stakeholder input;
      just record the finding in this ticket.
- [x] Any genuine bug surfaced by a real run is fixed here and documented.

## Implementation notes (2026-07-06)

Ported all three files to `tests/testgui/` (19 tests total). **Zero
production code changes** — `_on_sim_errors_apply`/`_on_sim_errors_from_cal`/
`sim_prefs.py` already worked exactly as documented; confirmed by direct
run against the current tree, not just by reading `resolve_calibration_
defaults()`'s own comment (which explicitly states it was written in
073-003 to keep `test_sim_errors_from_cal_button.py`'s exact monkeypatch
point working unchanged).

**Test-only bug found and fixed (API drift, not a production bug):** the
pre-rebuild `test_sim_errors_panel.py` set `sim_err_otos_yaw_drift` to
`-1.5`/`-2.0` in two tests, assuming the old SIMSET wire protocol's
per-SECOND drift-rate units. Ticket 083-001 reconciled this knob's ctypes
ABI to a per-TICK additive term with a much smaller spinbox range
(-0.05..0.05 rad/tick) — `QDoubleSpinBox.setValue()` silently clamps
out-of-range input, so both tests were asserting against a value the
widget could never actually hold. Fixed by using in-range values (-0.03,
-0.02) with an inline comment explaining the unit-convention history; the
sibling `sim_err_otos_lin_drift` knob's range (-5.0..5.0 mm/tick) was
already wide enough for the old test's values and needed no change.

**Open Question 3 (file overlap) — recorded, not resolved:**
`test_sim_errors_from_cal_button.py` and `test_sim_errors_from_calibration.py`
test overlapping ground (both assert the rotational_slip/trackwidth ->
body_rot_scrub/trackwidth mapping for a nocal and calibrated robot) from two
different historical eras, confirmed by porting both. Neither fully
supersedes the other: `test_sim_errors_from_cal_button.py` is the strictly
broader suite (missing-config-entirely fallback, missing-single-field
fallback per field, the same-path-as-Apply invariant, a fake connected
SimTransport's `apply_error_profile` call) via a MOCKED
`get_robot_config()`; `test_sim_errors_from_calibration.py` is the only one
exercising the REAL config loader (`ROBOT_CONFIG`/`load_robot_config`) end
to end against the REAL `data/robots/tovez.json` values. Both kept, per
this ticket's instruction — flagged as a candidate future consolidation,
not resolved here.

Full `tests/testgui` suite: 205 passed (up from 186 pre-ticket).

## Testing

- **Existing tests to run**: full `tests/testgui` suite (regression).
- **New tests to write**: port the three files above.
- **Verification command**: `QT_QPA_PLATFORM=offscreen uv run pytest
  tests/testgui -q`
