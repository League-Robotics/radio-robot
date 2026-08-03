---
id: '001'
title: "robot_config.proto — the one schema, end-state grouped shape + wire messages"
status: open
use-cases:
- SUC-001
- SUC-005
- SUC-006
- SUC-008
depends-on: []
github-issue: ''
issue: the-configuration-object.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# robot_config.proto — the one schema, end-state grouped shape + wire messages

## Description

Author `src/protos/robot_config.proto` declaring `Config::Robot`'s complete
END-STATE field set: 3 host-only groups (`Identity`, `Connection`,
`Vision`) and 7 robot-config groups (`Geometry`, `Motors`, `Drive`,
`WheelControl`, `Planner`, `Otos`, `Estimator`), each robot-config group
carrying a `ConfigTarget` enum value. Also declare the wire envelope
messages this schema drives: the `ConfigTarget` enum itself, plus
`SetConfigGroup`/`GetConfig`/`ConfigSnapshot`/`SetConfigField`. Add a
`(host_only)` proto option to `options.proto` (extending its existing
`(min)`/`(max)`/`(abs_max)` extension mechanism) marking
`Identity`/`Connection`/`Vision` so the generator (ticket 002) knows to
skip them for the C++ and wire targets.

This schema is authored in the **end-state grouped shape directly** — not
mirroring today's 13-section JSON — per sprint.md's Design Rationale
Decision 2. It replaces `config.proto`'s curated `*ConfigPatch` messages
(deleted later, ticket 013). Every field it declares must trace to an
existing `gen_boot_config.py` `_require()` call site (`src/scripts/gen_boot_config.py`):
`otos_boot_config_values`/`vel_gains_for_config`/`output_deadband_for_config`/
`reversal_dwell_for_config`/`trackwidth_for_config`/`rotational_slip_for_config`/
`rotation_calibration_for_config`/`estimator_config_for_config`/
`shaper_config_for_config`/`wheel_correction_for_config`/
`drive_config_for_config`/`wheel_controller_config_for_config`/
`planner_config_for_config` — mapped onto the target grouping (e.g.
`wheel_gain_*`/`wheel_intercept_*` → `Drive`; `wheel_pid_*`/`wheel_v_min`/
`wheel_bias_max`/etc. → `WheelControl`; `a_max`/`a_decel`/`alpha_max`/
`alpha_decel`/`j_max`/`yaw_jerk_max`/`v_max`/`omega_max`/`control_period`/
`actuation_delay` → `Planner`; OTOS scale/offset fields → `Otos`;
`weight_heading_otos`/`weight_omega_otos`/`staleness_ms` → `Estimator`;
`trackwidth`/`rotational_slip` → `Geometry`; `travel_calib`/`fwd_sign` →
`Motors`).

This ticket does **not** touch the robot JSON files (ticket 017) or
generate anything (ticket 002) — schema authorship only.

## Acceptance Criteria

- [ ] `robot_config.proto` exists in `src/protos/`, declares
      `Identity`/`Connection`/`Vision` (no `ConfigTarget` value, marked
      `(host_only)`) and `Geometry`/`Motors`/`Drive`/`WheelControl`/
      `Planner`/`Otos`/`Estimator` (each with a `ConfigTarget` enum value).
- [ ] Every field `gen_boot_config.py`'s `_require()` calls currently read
      (control/geometry/calibration/estimator/planner sections) has a
      corresponding field in the schema, grouped per the target layout
      above — verified by a field-by-field checklist against
      `gen_boot_config.py`'s actual `_require()` call sites, not just a
      field count.
- [ ] `ConfigTarget`, `SetConfigGroup`, `GetConfig`, `ConfigSnapshot`,
      `SetConfigField` are declared (per-group payload bodies for
      `SetConfigGroup` may be minimal/placeholder — the generated group
      struct from ticket 002 IS the per-group message; this ticket
      declares the envelope shape).
- [ ] A `(host_only)` proto option is added to `options.proto` and applied
      to `Identity`/`Connection`/`Vision`.
- [ ] The proto parses cleanly under the same parser `gen_messages.py`
      already uses (a bare parse/compile check — full codegen is ticket
      002's job).
- [ ] `ColorConfig`/`LineConfig` are **not** declared as groups (Out of
      Scope per sprint.md).

## Testing

- **Existing tests to run**: none apply yet — nothing consumes this file
  until ticket 002.
- **New tests to write**: a minimal parse-only check confirming the file
  compiles with no errors under the existing proto toolchain.
- **Verification command**: run the project's existing `.proto` parser
  (whatever `gen_messages.py` already invokes) directly against this new
  file.

## Implementation Plan

**Approach**: Read `src/protos/config.proto`, `drivetrain.proto`,
`motor.proto`, `options.proto`, and `data/robots/tovez.json`'s
`control`/`geometry`/`calibration`/`estimator`/`planner` sections side by
side; transcribe field-for-field into the new grouped shape. Cross-check
every `gen_boot_config.py` `_require()` call site listed above to confirm
every field it reads has a home in the new schema.

**Files to create**: `src/protos/robot_config.proto`.

**Files to modify**: `src/protos/options.proto` (add `(host_only)` option).

**Testing plan**: parse-only smoke check as above.

**Documentation updates**: none required this ticket — documentation
describing how the schema is consumed is a later ticket's concern once
the generator actually reads it.
