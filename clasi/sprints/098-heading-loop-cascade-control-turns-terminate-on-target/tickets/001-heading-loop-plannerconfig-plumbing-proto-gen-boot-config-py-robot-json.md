---
id: '001'
title: Heading-loop PlannerConfig plumbing (proto, gen_boot_config.py, robot JSON)
status: open
use-cases: [SUC-001, SUC-003]
depends-on: []
github-issue: ''
issue: heading-loop-cascade-control-turns-terminate-on-target.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Heading-loop PlannerConfig plumbing (proto, gen_boot_config.py, robot JSON)

## Description

Today `main.cpp`'s hand-written `defaultMotionConfig()` hardcodes every
`msg::PlannerConfig` field OUTSIDE the `gen_boot_config.py`-generated boot
path every other per-robot tunable (the velocity PID gains, trackwidth,
fwd_sign, ...) already goes through. This ticket closes that gap AND adds
the two new fields the outer heading loop needs: `heading_kp`/`heading_kd`.
No control-law behavior changes in this ticket — the gains exist and are
readable, but nothing consumes them yet (ticket 002 does). Sim behavior
must therefore be **bit-identical** before/after.

Reference: `architecture-update.md` M1/M2, Decision 2 (starting-gain
policy), Decision 3 (why tolerance/dwell are NOT here — they stay file-local
constants, added in ticket 002).

## Acceptance Criteria

- [ ] `protos/planner.proto`: `PlannerConfig` gains `heading_kp = 13` and
      `heading_kd = 14` (the first free field numbers after the existing
      1-9/12 and `reserved 10, 11`). Field comments state the unit tag
      (`// [1/s] outer heading-loop proportional gain, per-robot tunable`
      for `heading_kp`; `// dimensionless outer heading-loop derivative
      gain, per-robot tunable` for `heading_kd`), per this project's
      unit-in-comment-not-identifier convention.
- [ ] `source/messages/planner.h` regenerated (`scripts/gen_messages.py`,
      part of the normal build) — new `setHeadingKp()`/`setHeadingKd()`
      chainable setters appear on `msg::PlannerConfig`. The header is NOT
      hand-edited (generated files are never hand-edited — fixes go in the
      generator).
- [ ] `scripts/gen_boot_config.py`: new `heading_gains_for_config(cfg)`
      function, mirroring `vel_gains_for_config()`'s exact shape — reads
      `control.heading_kp`/`control.heading_kd` from the robot JSON,
      falling back to firmware defaults `HEADING_KP_DEFAULT = 3.0` and
      `HEADING_KD_DEFAULT = 0.0` when either key is absent (an unmigrated
      robot JSON simply inherits the conservative firmware default, same
      fallback discipline as every other mapping in this file).
- [ ] `scripts/gen_boot_config.py`'s `generate()` emits a new
      `Config::defaultPlannerConfig()` C++ function in `boot_config.cpp`
      that sets ALL nine currently-meaningful `PlannerConfig` fields: the
      seven motion-limit fields `main.cpp`'s `defaultMotionConfig()`
      currently hardcodes — `a_max=800.0f`, `a_decel=800.0f`,
      `v_body_max=1000.0f`, `yaw_rate_max=6.0f`, `yaw_acc_max=20.0f`,
      `j_max=5000.0f`, `yaw_jerk_max=100.0f` (moved verbatim, same numeric
      values, same units — NOT renumbered or retuned by this ticket) PLUS
      `heading_kp`/`heading_kd` from `heading_gains_for_config()`.
      `arrive_tol`/`turn_in_place_gate`/`min_speed` stay unset (`0.0f`
      default) — unchanged behavior, `main.cpp`'s old function never set
      them either.
- [ ] `source/config/boot_config.h`'s `Config` namespace declares
      `defaultPlannerConfig()` (mirroring `defaultDrivetrainConfig()`'s
      existing declaration).
- [ ] `source/main.cpp`: the local hand-written `defaultMotionConfig()`
      function is DELETED; `hardware_main()` calls
      `drivetrain.configureMotion(Config::defaultPlannerConfig())` instead.
- [ ] `data/robots/tovez.json`'s `control` block gains `"heading_kp": 3.0`
      and `"heading_kd": 0.0`, each a **per-robot tunable**, plus a
      `"_heading_gains_note"` string (mirroring the file's existing
      `_vel_gains_note` pattern) stating: these are conservative STARTING
      values, NOT yet bench-tuned; `heading_kp` ~ a few `/s` sits roughly a
      decade below the inner wheel-velocity loop's ~1-4 Hz corner
      (`motion_control.ipynb`); `heading_kd` starts at `0.0` (pure P,
      derivative term off) — iterate both against
      `tests/bench/turn_sweep.py --relay --both` in ticket 003.
- [ ] `just build-sim` succeeds; `just build-clean` succeeds (hex + sim).
- [ ] Full `uv run python -m pytest` stays green with **IDENTICAL**
      pass/fail counts to the pre-ticket baseline — this ticket changes
      zero runtime behavior, it only relocates where `PlannerConfig`'s
      values come from.
- [ ] A new/extended sim assertion confirms `Config::defaultPlannerConfig()`
      produces the SAME seven motion-limit values the old
      `main.cpp::defaultMotionConfig()` produced (a regression pin against
      a silent value change during the move) and that `heading_kp`/
      `heading_kd` read back as `3.0`/`0.0` for the active (`tovez`) robot
      config.

## Testing

- **Existing tests to run**: full `uv run python -m pytest` (collects
  `tests/sim`, `tests/unit`, `tests/testgui` per `pyproject.toml`
  `testpaths`); confirm the pass/fail COUNT is identical before and after.
- **New tests to write**: a boot-config regression pin (host-side —
  extend an existing harness or add a small new one) asserting
  `Config::defaultPlannerConfig()`'s 7 motion-limit fields match the
  pre-ticket hardcoded values exactly, and that `heading_kp`/`heading_kd`
  resolve to `3.0`/`0.0` for the active robot config.
- **Verification command**: `uv run python -m pytest` (project convention —
  always through `uv run`, never bare `pytest`).

## Implementation Plan

**Approach**: Mirror `vel_gains_for_config()`'s exact shape for the new
`heading_gains_for_config()`. Fold `main.cpp`'s 7 hardcoded motion-limit
constants into `gen_boot_config.py` as new module-level constants (e.g.
`A_MAX_DEFAULT = 800.0`), matching the file's existing "Bench-tuned
firmware defaults" section style and comments.

**Files to modify**: `protos/planner.proto`, `source/messages/planner.h`
(regenerated, not hand-edited), `scripts/gen_boot_config.py`,
`source/config/boot_config.h`, `source/main.cpp`, `data/robots/tovez.json`.

**Files to create**: none.

**Testing plan**: as above — sim/host-only, no firmware/hardware
verification needed for this purely-additive plumbing ticket (that begins
at ticket 003).

**Documentation updates**: none required beyond code comments — no
wire-protocol doc lists `PlannerConfig` fields individually today
(confirmed by grep during planning); `docs/design/message-inventory.md`'s
`PlannerConfig` rows are a historical RobotConfig-migration crosswalk, not
a live field reference, and are not extended for new fields with no old
`RobotConfig` equivalent.
