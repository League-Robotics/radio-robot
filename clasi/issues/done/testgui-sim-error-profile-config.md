---
status: done
tickets:
- NONE
---

# TestGUI: configurable Sim encoder/OTOS error profile (GUI panel + persisted file)

## Request

In Sim mode, let the operator configure the error/noise the simulator injects
for the encoders and the OTOS, instead of the two hardcoded constants.

## Current state

`SimTransport._apply_field_profile` (host/robot_radio/testgui/transport.py)
applies only two hardcoded module constants:
- `_SIM_SLIP_TURN_EXTRA = 0.26` (turn-slip encoder over-report fraction)
- `_SIM_OTOS_LINEAR_NOISE = 0.05` (OTOS linear noise σ fraction)

The sim exposes more knobs that are currently unused:
- `sim_set_encoder_noise(side, sigma_mm)` — per-side encoder noise (mm)
- `sim_set_otos_yaw_noise(sigma_fraction)` — OTOS yaw noise

All four C symbols are present in the prebuilt lib
(`tests/_infra/sim/build/libfirmware_host.dylib`). `firmware.py` already binds
`sim_set_otos_yaw_noise` but exposes no wrapper; it does not bind
`sim_set_encoder_noise` at all.

## Stakeholder decisions (binding)

- **GUI panel + persisted JSON file.** A "Sim Errors" group (visible only in
  Sim mode) with editable fields for all four knobs, backed by
  `data/testgui/sim_error_profile.json` (mirror the `camera_prefs.json`
  convention). Edit live; re-applies on connect and via an Apply button.
- Out of process.

## Scope

- New Qt-free `host/robot_radio/testgui/sim_prefs.py`: load/save the profile
  (keys: `encoder_noise_mm`, `slip_turn_extra`, `otos_linear_noise`,
  `otos_yaw_noise`; defaults 0.0 / 0.26 / 0.05 / 0.0), never raising.
- `firmware.py` (tests/_infra/sim): add `set_encoder_noise(side, sigma_mm)`
  and `set_otos_yaw_noise(sigma_fraction)` wrappers; guard bindings with
  `hasattr(lib, …)` so a stale lib degrades gracefully.
- `transport.py`: `SimTransport._apply_field_profile` loads the profile and
  applies all four knobs (each guarded); add `apply_error_profile(profile)`
  for live re-apply; `turn_scrub_factor` reflects the applied `slip_turn_extra`.
- `__main__.py`: the "Sim Errors" panel (shown only when Sim is selected),
  populated from the file, with an Apply button that saves + live-applies to a
  connected SimTransport.
- Tests in `tests/testgui/` (Qt-free prefs round-trip; panel visibility/apply
  with fakes). `tests/simulation` must stay green.
