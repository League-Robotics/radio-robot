---
id: "006"
title: "Sim-Errors panel verification"
status: open
use-cases: [SUC-007]
depends-on: []
github-issue: ""
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

- [ ] `tests_old/testgui/test_sim_errors_panel.py`,
      `test_sim_errors_from_cal_button.py`, and
      `test_sim_errors_from_calibration.py` are ported to
      `tests/testgui/`, updated for any API drift, and pass under
      `QT_QPA_PLATFORM=offscreen`.
- [ ] "From Calibration" with an uncalibrated robot ("tovez nocal") yields
      the all-neutral (zero-error) panel.
- [ ] "From Calibration" with a calibrated robot yields the documented
      inverse mapping (`rotational_slip` -> body rot scrub,
      `geometry.trackwidth` -> trackwidth, every other knob neutral).
- [ ] "From Calibration" is confirmed to reuse `_on_sim_errors_apply`'s
      save/apply path exactly once each (`sim_prefs.save_sim_error_profile`
      and, when connected, `SimTransport.apply_error_profile`) — not a
      second, independently-written apply path.
- [ ] A missing or partial robot config falls back to neutral per-field
      with a logged `[WARN]`, never raising.
- [ ] The three no-ctypes-backing noise fields (`sim_err_encoder_mm`,
      `sim_err_otos_linear`, `sim_err_otos_yaw` — 083's documented
      no-effect fields) are left untouched by "From Calibration."
- [ ] Note during implementation whether
      `test_sim_errors_from_cal_button.py` and
      `test_sim_errors_from_calibration.py` fully overlap (planning-time
      Open Question 3) — do not delete either without stakeholder input;
      just record the finding in this ticket.
- [ ] Any genuine bug surfaced by a real run is fixed here and documented.

## Testing

- **Existing tests to run**: full `tests/testgui` suite (regression).
- **New tests to write**: port the three files above.
- **Verification command**: `QT_QPA_PLATFORM=offscreen uv run pytest
  tests/testgui -q`
