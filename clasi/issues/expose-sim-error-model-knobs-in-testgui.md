---
status: pending
---

# Expose the simulator's existing error-model knobs (scale error, drift) in the TestGUI Sim Errors panel

## Background

The TestGUI "Sim Errors" panel currently exposes only four knobs:
encoder noise (mm), turn slip, OTOS linear noise, and OTOS yaw noise.
The stakeholder asked whether the panel should also model a constant
error factor — i.e. `y = (A + e1)x + b + e2` — and yaw drift.

Investigation (2026-07-02) found the simulator plant **already
implements almost exactly that model**; the gap is exposure, not
modeling:

- **OTOS** (`source/hal/sim/SimOdometer.cpp`, tick()):
  `delta_out = (1 + scaleErr) · (1 + N(0,σ)) · delta_true + drift_per_tick`
  — the existing panel "noise" fields are the fractional `e1·x` gain
  noise; `setLinearScaleError` / `setAngularScaleError` (the `A`,
  ticket 057-005) and `setDriftPerTickMm` / `setDriftPerTickRad`
  (the `b`, including **yaw drift**) exist but are not exposed.
- **Encoders** (`source/hal/sim/PhysicsWorld.h`):
  `reported = (1 + scaleErr_perWheel) · (1 − slip) · true + N(0, σ_mm)`
  — per-wheel `setEncoderScaleError` (ticket 058-001) exists but is
  not exposed. Per-wheel scale asymmetry is the physically important
  case (mismatched wheel diameters → heading drift while driving
  straight). `setOffsetFactor` (per-wheel actuation asymmetry) and
  `setTrackwidth` (geometry error) also exist unexposed.

These setters are only exported through `drive_api.cpp` (the
pytest/bench harness surface: `drive_api_configure_otos_model`,
`drive_api_set_encoder_scale_error`). The TestGUI path —
`sim_api.cpp` → `host/robot_radio/io/sim_conn.py` →
`host/robot_radio/testgui/sim_prefs.py` → the panel — only carries
the four existing knobs.

## Proposed work

Expose six more fields in the Sim Errors panel (all plumbing, no new
plant modeling):

1. Encoder scale error L (%) and R (%)
2. OTOS linear scale error (%) and angular scale error (%)
3. OTOS linear drift (mm/s) and yaw drift (deg/s) — GUI presents
   per-second; wrapper converts to per-tick using the sim tick rate

Optionally: motor offset factor L/R (actuation asymmetry) and
trackwidth error (%).

Plumbing per knob:

- Export the setter in `tests/_infra/sim/sim_api.cpp` (mirroring the
  existing `drive_api.cpp` exports)
- Add a ctypes wrapper in `host/robot_radio/io/sim_conn.py` (guard
  for stale prebuilt libs missing the symbol, as done for
  `sim_set_encoder_noise`)
- Extend the profile keys + defaults in
  `host/robot_radio/testgui/sim_prefs.py` (defaults 0.0 = no change
  in behavior until an operator opts in)
- Add the spinboxes to the Sim Errors panel in
  `host/robot_radio/testgui/__main__.py` and apply via
  `transport.py`

## Explicitly out of scope (noted as possible future extensions)

- Encoder additive bias `b` — physically wrong for incremental
  encoders (would count while stationary); scale error is the
  meaningful "constant error factor" there
- Random-walk (Brownian) yaw drift — current constant-rate drift is a
  reasonable first-order gyro-bias model; a bias random walk would be
  a genuine model extension to stress the EKF harder
- Fractional (gain) noise on encoders — slip's turn-rate term already
  modulates gain dynamically; second-order effect
