---
id: '002'
title: 'source/drive/ core: types, arc_math, master_profile + isolation grep test'
status: open
use-cases: [SUC-002, SUC-008]
depends-on: ['001']
github-issue: ''
issue: motion-stack-v2-a-self-contained-stateless-motion-control-subsystem.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# source/drive/ core: types, arc_math, master_profile + isolation grep test

## Description

Build the self-contained foundation of `source/drive/` (namespace
`Drive`): `types.h` (`Pose`/`Twist`/`WheelState`/`BodyState`/
`WheelVelocities`/`Limits` — plain values, no `msg::`), `arc_math.
{h,cpp}` (`composeArc`/`poseAlongArc`/exact-circle-projection/
`wrapAngle`, adapted from `source/kinematics/body_kinematics.{h,cpp}`),
`master_profile.{h,cpp}` (the Ruckig wrapper, adapted from `source/
motion/jerk_trajectory.h`'s `solveToRest`/`solveToVelocity` pattern and
generalized to `solveToExit` with a nonzero target velocity). Also lands
the grep isolation test that structurally enforces `source/drive/`'s
self-contained boundary for the rest of this sprint and forever after.

This is a **copy**, not a move or an include — `body_kinematics.{h,cpp}`
and `jerk_trajectory.{h,cpp}` stay untouched (per the issue's explicit
"code that exists elsewhere is copied in" direction). Read both files in
full before starting; their doc comments explain the exact contracts
(seeding discipline, direction-mirrored acceleration bounds, the jerk
sentinel) to preserve in the copy.

## Acceptance Criteria

- [ ] `source/drive/types.h` defines `Pose{x,y,h}` / `Twist{v_x,v_y,
      omega}` / `WheelState` / `BodyState` / `WheelVelocities` / `Limits`
      as plain value types — zero `msg::`, zero `Hal::`, zero CODAL
      includes. Naming follows `.claude/rules/naming-and-style.md`: no
      units in identifiers, units in `// [unit]` tags, `UpperCamelCase`
      types, `lowerCamelCase` members/functions.
- [ ] `source/drive/arc_math.{h,cpp}`: `composeArc`/`poseAlongArc`/exact
      circle projection/`wrapAngle`, hand-ported from
      `body_kinematics.{h,cpp}` (that file itself is untouched by this
      ticket — verify with `git diff --stat source/kinematics/`).
- [ ] `source/drive/master_profile.{h,cpp}`: the Ruckig wrapper,
      hand-ported from `jerk_trajectory.{h,cpp}` (untouched by this
      ticket). `solveToExit(targetPosition, exitVelocity, maxVelocity)`
      supports a NONZERO `exitVelocity` via Ruckig's own
      `InputParameter::target_velocity` (valid iff `|v_target| <=
      max_velocity`); the directional no-reversal band generalizes
      `jerk_trajectory`'s same-sign band to the nonzero-exit-speed case.
      Preserve the seeding contract (seed from the channel's OWN
      remembered last sample, never a measured observation) and the
      jerk-sentinel (`0.0f` -> Ruckig's own `+infinity` default)
      verbatim.
- [ ] Grep isolation test exists and fails loudly (naming the offending
      file/line) if any file under `source/drive/` references `msg::`,
      `Hal::`, `Subsystems::`, `MicroBit`, `kOutputHops`, or `kDeadTime`,
      or `#include`s anything outside `source/drive/`, libc/libm, or
      `libraries/ruckig`.
- [ ] C++ unit harnesses (`tests/sim/unit` compile-and-run pattern,
      mirroring `jerk_trajectory_harness.cpp`/`test_jerk_trajectory.py`
      exactly — `gnu++20 -fno-exceptions -fno-rtti`, matching
      `CMakeLists.txt`'s actual firmware flags): arc-math round-trips
      (compose then project recovers the original arc within float
      tolerance); `solveToExit` boundary tuples (zero exit speed matches
      `solveToVelocity(0)`'s existing shape; nonzero exit speed within
      `maxVelocity` solves; `|exitVelocity| > maxVelocity` fails cleanly,
      never UB).
- [ ] `just build` (or the project's equivalent firmware build recipe)
      succeeds with `source/drive/*.cpp` present but uncalled from
      `main.cpp`. `arm-none-eabi-size` before/after comparison shows a
      near-zero flash delta (architecture-update.md Decision 3 — this is
      an empirical claim to VERIFY, not assume). Record the actual byte
      delta in completion notes.
- [ ] `uv run python -m pytest` passes.

## Testing

- **Existing tests to run**: `uv run python -m pytest`; `just build`
  (firmware) + `arm-none-eabi-size` comparison.
- **New tests to write**: the grep isolation test; C++ unit harnesses for
  arc-math round-trips and `solveToExit` boundary behavior.
- **Verification command**: `uv run pytest`

## Implementation Plan

**Approach**: copy-and-adapt, not reference. Read `source/motion/
jerk_trajectory.h` and `source/kinematics/body_kinematics.h` in full
first. Do not `#include` either file from `source/drive/` — hand-port
the logic, preserving every documented contract (the seeding discipline
in particular — do not let a measured observation leak into
`master_profile`'s seed path, the exact failure mode `jerk_trajectory.h`'s
own doc comment warns against, ticket 087-009's history).

**Files to create**:
- `source/drive/types.h`
- `source/drive/arc_math.h`, `source/drive/arc_math.cpp`
- `source/drive/master_profile.h`, `source/drive/master_profile.cpp`
- A grep-isolation test (either a small Python script under `tests/sim/
  unit/` or a shell-based pytest wrapper — programmer's choice; document
  which in completion notes)
- `tests/sim/unit/drive_arc_math_harness.cpp` + `test_drive_arc_math.py`
- `tests/sim/unit/drive_master_profile_harness.cpp` +
  `test_drive_master_profile.py`

**Testing plan**: new C++ unit harnesses per the
`jerk_trajectory_harness.cpp`/`test_jerk_trajectory.py` compile-and-run
pattern; the grep isolation test; a firmware build + size check.

**Documentation updates**: none required this ticket — `source/drive/`'s
own doc comments (per the issue's two header sketches, which this
ticket's files should match in doc-comment density and style) carry the
design intent.
