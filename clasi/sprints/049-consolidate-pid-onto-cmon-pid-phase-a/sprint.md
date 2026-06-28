---
id: 049
title: Consolidate PID onto cmon-pid (Phase A)
status: planning-docs
branch: sprint/049-consolidate-pid-onto-cmon-pid-phase-a
use-cases: []
issues:
- consolidate-control-code-onto-vendored-libraries-cmon-pid-tinyekf.md
---

# Sprint 049: Consolidate PID onto cmon-pid (Phase A)

## Goals

Replace the hand-rolled PI/PID core in `VelocityController` with the vetted
header-only `cmon-pid` library (BSD-2-Clause), vendored as a `float` variant
under `libraries/cmon-pid/`. Delete the dead `RatioPidController` and its
associated `pid.*` config keys. Behaviour of the velocity control loop must
be unchanged — all affected sim unit tests pass green.

## Issues addressed

- `consolidate-control-code-onto-vendored-libraries-cmon-pid-tinyekf.md` —
  Phase A (PID consolidation only; Phase B EKF work is Sprint 050)

## Rationale / grouping

Phase A is the low-risk half of the vendoring work. cmon-pid is header-only,
fits the no-heap/no-STL/no-RTTI constraint, and the only adaptation needed is
a mechanical `double`→`float` conversion in the vendored header. Doing PID
first establishes the `libraries/` vendoring infra and dual-build include
wiring that Phase B (Sprint 050) will reuse.

## Scope sketch

- Vendor `cmon-pid` into `libraries/cmon-pid/` (float-adapted header + LICENSE)
- Wire include dir into root `CMakeLists.txt` and `tests/_infra/sim/CMakeLists.txt`
- Refactor `source/control/VelocityController.{h,cpp}` to use cmon-pid core;
  keep feedforward, sign handling, deadband, and PWM clamp in the thin wrapper
- Delete `source/control/RatioPidController.{h,cpp}` and remove from sim build
- Delete `pid.*` config keys from `source/types/Config.h` and
  `source/robot/ConfigRegistry.cpp`; remove `tests/simulation/unit/test_ratio_pid.py`
- Validate: `test_velocity_controller.py`, `test_motor_controller.py`,
  `test_body_velocity_controller.py`, `test_vendor_confinement.py`
- Detail-planning will produce tickets

## Dependencies

- Sprint 048 (eliminate-ifdef) should land first; no hard file conflict but
  clean baseline is preferred before vendoring changes
- No dependency on 050; 049 runs independently and unblocks 050

## Success gate

Clean firmware + host-sim build (`python build.py --clean`); full sim unit
suite green with no `RatioPidController` references remaining in source.

## Test strategy

IMPORTANT: The canonical test command for this sprint is:

```
uv run --with pytest python -m pytest tests/simulation -q
```

Do NOT use bare `uv run pytest` — it uses an ephemeral interpreter missing
project deps and falsely reports mass failures.

Known pre-existing baseline: exactly 2 failures unrelated to this sprint:
- `tests/simulation/unit/test_default_config_pin.py::test_default_robot_config_unchanged`
- `tests/simulation/unit/test_robot_config.py::TestSchemaValidation::test_tovez_validates_against_schema`

A ticket is acceptable if and only if it introduces NO new failures beyond these 2.
Deleting `test_ratio_pid.py` will reduce the passed count by one — this is expected.

## Tickets

Tickets execute serially in dependency order.

| # | Title | Depends on |
|---|---|---|
| 001 | Vendor cmon-pid as float-adapted header into libraries/cmon-pid/ | — |
| 002 | Wire libraries/cmon-pid/ include path into firmware and host-sim builds | 001 |
| 003 | Refactor VelocityController to compose cmon-pid backcalculation core | 002 |
| 004 | Delete RatioPidController and remove pid.* config keys and N13 test references | 003 |
| 005 | Validate Phase A: full sim suite green, confinement gate passes | 004 |
