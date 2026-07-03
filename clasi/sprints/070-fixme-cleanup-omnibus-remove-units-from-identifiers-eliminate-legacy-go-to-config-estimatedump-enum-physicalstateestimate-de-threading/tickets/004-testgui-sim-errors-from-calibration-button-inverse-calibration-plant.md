---
id: '004'
title: 'TestGUI Sim Errors: From Calibration button (inverse-calibration plant)'
status: done
use-cases: []
depends-on: []
github-issue: ''
issue: testgui-sim-errors-from-calibration-button.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# TestGUI Sim Errors: From Calibration button (inverse-calibration plant)

## Description

Add a **From Calibration** button next to the existing **Apply** button in
the TestGUI "Sim Errors" panel (`sim_errors_group`,
`host/robot_radio/testgui/__main__.py` lines ~656-819). Pressing it populates
the panel's spin boxes with the **inverse of the active robot's calibration**
so the sim firmware's baked-in calibration exactly compensates the injected
plant errors — the sim robot then behaves perfectly, modulo noise. Full
rationale and investigation are in issue
`testgui-sim-errors-from-calibration-button.md`.

Why this matters: the sim firmware bakes the active robot's calibration into
`source/robot/DefaultConfig.cpp` (`rotationalSlip=0.92f`,
`trackwidthMm=128.0f` — generated from `data/robots/tovez.json`), but the sim
plant is ideal (zero scrub). `PlannerBegin.cpp::beginRotation()` inflates the
RT arc target by `1/rotationalSlip` to compensate for *real* chassis scrub,
so against the ideal plant it over-rotates (RT 9000 → ~95° instead of 90°,
per ticket 069-002/`test_069_rt_90deg_body_scrub.py`). This button injects a
matching plant-side scrub so the firmware's correction and the plant's
scrub cancel out.

**Value mapping** (from the issue's investigation — `mmPerDegL/R` and the
OTOS scale calibrations act only at the real-HAL sensor boundary and are
inert in sim, so their inverse is the neutral value; only `rotational_slip`
and `trackwidth` are live in the sim control path):

| Knob | objectName | Value | Source |
|---|---|---|---|
| turn slip | `sim_err_slip_turn` | `0.0` | scrub modeled via body rot scrub instead |
| body rot scrub | `sim_err_body_rot_scrub` | `calibration.rotational_slip` | active robot config |
| body lin scrub | `sim_err_body_lin_scrub` | `1.0` | no linear-scrub calibration exists |
| motor offset L | `sim_err_motor_offset_l` | `1.0` | no per-side motor calibration |
| motor offset R | `sim_err_motor_offset_r` | `1.0` | no per-side motor calibration |
| trackwidth (mm) | `sim_err_trackwidth` | `geometry.trackwidth` | matches firmware belief |
| enc scale err L/R | `sim_err_enc_scale_l`/`_r` | `0.0` | mmPerDeg calibration inert in sim |
| OTOS lin/ang scale err | `sim_err_otos_lin_scale`/`sim_err_otos_ang_scale` | `0.0` | OTOS register calibration inert in sim |
| OTOS lin/yaw drift | `sim_err_otos_lin_drift`/`sim_err_otos_yaw_drift` | `0.0` | no drift calibration exists |
| encoder noise, OTOS lin/yaw noise | `sim_err_encoder_mm`, `sim_err_otos_linear`, `sim_err_otos_yaw` | **untouched** | stakeholder-specified exception |

`calibration.rotational_slip` and `geometry.trackwidth` come from
`robot_radio.config.robot_config.get_robot_config()` (already imported at
`__main__.py` line ~394; both fields are `Optional[float]` on
`RobotConfig`/`CalibrationConfig`/`GeometryConfig` and `get_robot_config()`
itself can return `None`). When the config or either field is unavailable,
fall back to the neutral value for that knob (`1.0` for body rot scrub,
`sim_prefs.DEFAULT_PROFILE["trackwidth_mm"]` (128.0) for trackwidth) and log
a warning via `_append_log` — never raise or crash the panel.

After populating the spin boxes, the button persists and live-applies
through the **same code path as Apply** (`sim_prefs.save_sim_error_profile` +
`SimTransport.apply_error_profile` when connected) — no duplicated
save/apply logic. The cleanest way to guarantee this is for the new
handler to programmatically invoke the existing `_on_sim_errors_apply()`
closure after setting the spin values, rather than reimplementing the
profile-dict-and-save steps.

## Acceptance Criteria

- [x] A new `QPushButton("From Calibration")`, `objectName
      "sim_errors_from_cal_btn"`, appears next to `sim_errors_apply_btn` in
      the Sim Errors panel (both buttons share a row, e.g. via a small
      `QHBoxLayout` row widget replacing the single `addWidget(
      sim_errors_apply_btn)` call).
- [x] Pressing it sets exactly the mapped values in the table above on:
      `sim_err_slip_turn`, `sim_err_body_rot_scrub`, `sim_err_body_lin_scrub`,
      `sim_err_motor_offset_l`, `sim_err_motor_offset_r`, `sim_err_trackwidth`,
      `sim_err_enc_scale_l`, `sim_err_enc_scale_r`, `sim_err_otos_lin_scale`,
      `sim_err_otos_ang_scale`, `sim_err_otos_lin_drift`,
      `sim_err_otos_yaw_drift` — read live from `get_robot_config()`, not
      hardcoded.
- [x] It does **not** touch `sim_err_encoder_mm`, `sim_err_otos_linear`, or
      `sim_err_otos_yaw` — a unit test that pre-sets one of these three to a
      nonzero value, clicks the button, and asserts the value is unchanged.
- [x] The resulting profile is persisted via
      `sim_prefs.save_sim_error_profile` and, when a connected Sim transport
      is present, live-applied via `SimTransport.apply_error_profile` —
      exactly the same call sequence as the existing Apply button (verified
      by a test that the button's handler invokes the existing
      `_on_sim_errors_apply`-equivalent path rather than a second,
      independently-written save/apply call).
- [x] Missing/partial config is handled gracefully: `get_robot_config()`
      returning `None`, or returning a config whose
      `calibration.rotational_slip` or `geometry.trackwidth` is `None`, falls
      back to the neutral value for that knob (`1.0` / `128.0` respectively)
      and logs a warning — does not raise or crash the panel.
- [x] New unit tests in `tests/testgui/` cover: the mapping (button click →
      exact spin-box values from a fake/monkeypatched `get_robot_config()`),
      the noise-field exception (three noise spins untouched), and the
      missing-config fallback (both `None` config and a config with `None`
      calibration fields).
- [x] A new sim system test (`tests/simulation/system/`) shows the effect
      end-to-end: with the inverse-calibration mapping applied (`bodyRotScrub
      = rotational_slip` from the active robot config, `trackwidthMm =
      geometry.trackwidth`, every other knob at its neutral/mapped value,
      noise at 0), `RT 9000` lands within the same tolerance
      `test_069_rt_90deg_body_scrub.py` uses (`_NEAR_90_TOL_DEG = 5.0`) of
      90° true heading — the ~95° ideal-plant over-rotation disappears.
- [x] Full suite green: `uv run python -m pytest`.

## Testing

- **Existing tests to run**: `tests/testgui/test_sim_errors_panel.py`,
  `tests/testgui/test_sim_prefs.py`,
  `tests/simulation/system/test_069_rt_90deg_body_scrub.py` (as a reference
  baseline, not modified by this ticket), full default suite.
- **New tests to write**:
  - `tests/testgui/test_sim_errors_panel.py` (or a new sibling file): button
    existence (`sim_errors_from_cal_btn`), the mapping (monkeypatch
    `get_robot_config` to return a fake config with known
    `calibration.rotational_slip`/`geometry.trackwidth`, click the button,
    assert every mapped spin box's value), the three-noise-fields-untouched
    invariant (pre-set a noise spin to a nonzero value first), the same-path
    invariant (monkeypatch `sim_prefs.save_sim_error_profile` /
    `SimTransport.apply_error_profile` the same way
    `TestSimErrorsApplyButton` does, assert both are called once with the
    mapped profile), and the missing-config fallback (`get_robot_config`
    returns `None`, and returns a config with `calibration.rotational_slip is
    None`/`geometry.trackwidth is None`).
  - A new `tests/simulation/system/test_070_00x_sim_errors_from_cal.py` (or
    similarly named) sim system test mirroring
    `test_069_rt_90deg_body_scrub.py`'s `sim` fixture usage: apply the
    mapping's `SIMSET bodyRotScrub=<rotational_slip>
    trackwidthMm=<trackwidth>` (plus the other neutral knobs) and run `RT
    9000`, asserting `sim.get_true_pose()`'s heading lands within
    `_NEAR_90_TOL_DEG` (5.0°) of 90°. Note `DefaultConfig.cpp` already bakes
    `rotationalSlip=0.92f`/`trackwidthMm=128.0f` to match
    `data/robots/tovez.json`, so no `SET rotSlip=` is needed to reproduce the
    scenario — only the `SIMSET` side (the plant) needs configuring.
- **Verification command**: `uv run python -m pytest`

## Implementation Plan

**Approach**:
1. In `host/robot_radio/testgui/__main__.py`, wrap `sim_errors_apply_btn` and
   a new `sim_errors_from_cal_btn` (`QPushButton("From Calibration")`) in a
   shared `QHBoxLayout` row, replacing the current
   `sim_errors_layout.addWidget(sim_errors_apply_btn)` call.
2. Add a handler `_on_sim_errors_from_cal()`, defined after the existing
   `_on_sim_errors_apply()` closure (so it can call it directly) and before
   the button-click connections at the end of the panel block. It:
   - Calls `get_robot_config()` (already imported in this function's scope
     at line ~394-398 via `from robot_radio.config.robot_config import
     (get_robot_config, list_robots, set_active_robot)`).
   - Computes `rot_slip = cfg.calibration.rotational_slip if cfg is not None
     and cfg.calibration.rotational_slip is not None else 1.0` (logging a
     `[WARN]` via `_append_log` when falling back).
   - Computes `tw = cfg.geometry.trackwidth if cfg is not None and
     cfg.geometry.trackwidth is not None else sim_prefs.DEFAULT_PROFILE[
     "trackwidth_mm"]` (same warning-on-fallback treatment).
   - Sets the 12 mapped spin boxes' values (`.setValue(...)`) per the table
     above; does not touch the 3 noise spin boxes.
   - Logs an `[INFO]` line summarizing what was applied (mirroring
     `_on_sim_errors_apply`'s logging style).
   - Calls `_on_sim_errors_apply()` to perform the save + live-apply — this
     is what guarantees "no duplicated apply logic" (AC 3).
3. Connect `sim_errors_from_cal_btn.clicked.connect(_on_sim_errors_from_cal)`
   alongside the existing `sim_errors_apply_btn.clicked.connect(
   _on_sim_errors_apply)`.
4. No changes needed to `sim_prefs.py` or `transport.py` — this ticket only
   adds a new way to populate the same profile dict that `_on_sim_errors_apply`
   already saves/applies; the SIMSET wire mapping (`PROFILE_TO_SIMSET_KEY`)
   and live-apply path are unchanged.

**Files to modify**:
- `host/robot_radio/testgui/__main__.py` (new button, new handler, row
  layout change)

**Files to create**:
- `tests/testgui/test_sim_errors_from_cal_button.py` (or added to the
  existing `test_sim_errors_panel.py` — programmer's judgment call; a
  separate file keeps `test_sim_errors_panel.py` from growing unbounded)
- `tests/simulation/system/test_070_00x_sim_errors_from_cal.py` (exact
  numbering per the programmer's ticket ID once known, e.g.
  `test_070_004_sim_errors_from_cal.py`)

**Testing plan**: unit tests first (fast, no sim build dependency), then the
new sim system test (requires the built sim lib — `tests/_infra/sim/build/`,
`build_lib` fixture per `tests/CLAUDE.md`), then the full suite.

**Documentation updates**: none required — this is a TestGUI-only operator
convenience feature; no `docs/` file describes the Sim Errors panel's button
set at a level of detail that needs updating (the module docstrings in
`__main__.py`/`sim_prefs.py` are the living documentation and are updated as
part of the code change itself, not a separate doc).
