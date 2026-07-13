---
id: '014'
title: 'Fix pivot rotational velocity: bake control.yaw_rate_max/max_rot_accel through
  gen_boot_config (was hardcoded 6.0 rad/s, ~500mm/s wheels, caused unstable overshooting
  turns on hardware)'
status: in-progress
use-cases: []
depends-on: []
github-issue: ''
issue: motion-stack-v2-a-self-contained-stateless-motion-control-subsystem.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Fix pivot rotational velocity: bake control.yaw_rate_max/max_rot_accel through gen_boot_config (was hardcoded 6.0 rad/s, ~500mm/s wheels, caused unstable overshooting turns on hardware)

## Description

Field validation (011/012) on the playfield surfaced that the motion-v2
**closed-loop pivot is violently unstable on hardware** — a commanded +90°
pivot thrashes ±100° for ~3s and lands at a wrong/random heading, and even a
"straight" DISTANCE picks up a large spurious rotation. Open-loop drive
(straight AND spin) is clean and stable, and OTOS heading agrees with the
camera on every sample, so the plant, wiring, and feedback sign are all fine.

**Root cause (config-plumbing bug, not control law):** the pivot's rotational
velocity ceiling is `PlannerConfig.yaw_rate_max`, baked by
`scripts/gen_boot_config.py`. That generator's `drive_limits_for_config()`
reads `control.*` for PlannerConfig fields 15-31, but the profile limits
(fields 4-5, `yaw_rate_max`/`yaw_acc_max`) still emit the **hardcoded
`YAW_RATE_MAX_DEFAULT = 6.0 rad/s` / `YAW_ACC_MAX_DEFAULT = 20 rad/s²`
constants**, silently ignoring `data/robots/tovez.json`'s intended
`control.yaw_rate_max = 70` (deg/s = 1.22 rad/s) and
`control.max_rot_accel_dps2 = 600` (deg/s² = 10.5 rad/s²). So firmware planned
pivots at **6 rad/s (~344°/s, ~380-500 mm/s at the wheels)** instead of the
intended ~78 mm/s. Confirmed on the stand: a +90 pivot commands sustained
`vel=(-494,+460)` mm/s. With the robot's ~120-140ms actuation latency, that
overshoots ~45-57° per latency period → limit-cycle oscillation. Sim doesn't
exercise the same latency, so it's stable there.

`yaw_rate_max` is NOT live-SET-able (absent from the wire `PlannerConfigPatch`)
and NOT overridable per-segment (`drive_bridge.h:driveGoal()` ignores every
per-segment field), so the only source is the baked boot config — i.e. this
generator.

Stakeholder (2026-07-13): pivot wheel velocity should be **under 200 mm/s**.

## Fix

Wire `control.yaw_rate_max` (deg/s → rad/s) and `control.max_rot_accel_dps2`
(deg/s² → rad/s²) through `gen_boot_config.py`, falling back to the existing
rad-valued `*_DEFAULT` constants when the keys are absent (same fall-back
discipline as every other `control.*` mapping in that file). tovez.json's 70
deg/s → ~78 mm/s wheels (under 200). No firmware control-law change.

## Acceptance Criteria

- [ ] `gen_boot_config.py` reads `control.yaw_rate_max`/`control.max_rot_accel_dps2`
      (deg→rad), defaults preserved when absent; regenerated `boot_config.cpp`
      shows `setYawRateMax(1.2217…)` / `setYawAccMax(10.47…)` for tovez.
- [ ] Full `uv run python -m pytest` stays green (regen + any golden boot_config fixture updated).
- [ ] HARDWARE stand: a commanded pivot now drives wheels < 200 mm/s (was ~494).
- [ ] HARDWARE floor (camera): pivots are STABLE (no oscillation) and land near
      target across a ±45/90/135 sweep, both directions.

## Testing

- **Existing tests to run**: `uv run python -m pytest` (esp. any gen_boot_config / boot_config golden test)
- **New tests to write**: generator unit check that a control block with yaw_rate_max/max_rot_accel_dps2 is honored (deg→rad) and that absence falls back to defaults.
- **Verification command**: `uv run python -m pytest`
- **HITL**: stand pivot wheel-velocity check + floor camera turn-accuracy sweep (pivot_sweep.py).
