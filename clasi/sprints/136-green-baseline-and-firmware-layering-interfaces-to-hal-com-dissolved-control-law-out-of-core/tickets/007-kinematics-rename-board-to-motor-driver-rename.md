---
id: '007'
title: Kinematics rename + board-to-motor-driver rename
status: open
use-cases: ["SUC-002"]
depends-on: ["006"]
github-issue: ''
issue: firmware-layering-cleanup-interfaces-to-hal-impls-to-platform-microbit.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Kinematics rename + board-to-motor-driver rename

## Description

Phase 5 (kinematics rename) plus Phase 5b (board → motor-driver rename)
of the layering-cleanup issue — batched together because both are
name-only mechanical renames touching non-overlapping files.

**Kinematics rename**: `kinematics/differential_kinematics.{h,cpp}` →
`kinematics/differential.{h,cpp}`, `Kinematics::DifferentialKinematics` →
`Kinematics::Differential`; `kinematics/mecanum_kinematics.*` →
`kinematics/mecanum.*`, `Kinematics::MecanumKinematics` →
`Kinematics::Mecanum`. Removes the stutter against the enclosing
namespace. No behavior change.

**Board → motor-driver rename** (stakeholder directive, 2026-08-11:
*"The HiWonder board and board motor should be called motor drivers, not
boards. Everything's a board."*):

| Today | Rename to |
|---|---|
| `Hal::MotorBoard` (`hal/motor_board.h`) | `Hal::MotorDriver` (`hal/motor_driver.h`) |
| `Hardware::HiwonderBoard` (`hardware/hiwonder/hiwonder_board.*`) | `Hardware::HiwonderDriver` (`hiwonder_driver.*`) |
| `Hardware::BoardMotor` (`hardware/generic/board_motor.*`) | `Hardware::MotorDriverChannel` (`motor_driver_channel.*`) |

The third name (`MotorDriverChannel`) is this sprint's own naming call
(the source issue left it open — "name it at sprint time; `DriverMotor` is
the mechanical answer but reads poorly"). See sprint.md's Design Rationale
for the alternatives considered (`DriverMotor`, `GenericMotor`,
`MotorDriverLeaf`) and why `MotorDriverChannel` was chosen: it states
exactly what the object is — one channel of a `Hal::MotorDriver`-family
board, presented as a `Hal::Motor`. This is the one name in this sprint
nobody but the sprint-planner picked (flagged in sprint.md's Open
Questions) — if the stakeholder has a different preference once it's
visible in the tree, that's a cheap follow-up rename, not a redesign.

Both `HiwonderDriver` and `MotorDriverChannel` keep their "NOT WIRED IN
YET" header comment verbatim — they had zero callers before this ticket
and this ticket does not wire them in.

**Do not assume the glob-only claim — verify it.** The source issue
states `hardware/generic/board_motor.cpp` and
`hardware/hiwonder/hiwonder_board.cpp` reach the ARM image **glob-only**
(appear in no explicit non-ARM build/test source list anywhere). This was
not independently re-confirmed during sprint planning beyond trusting the
issue's own text — this ticket must verify it directly, both before
renaming (to know what's actually at stake) and after (to prove nothing
broke silently, since a missed *explicit* reference to either old
filename would show up as nothing at all unless the ARM build is actually
run).

## Acceptance Criteria

- [ ] `kinematics/differential.{h,cpp}` (renamed from
      `differential_kinematics.*`), class `Kinematics::Differential`.
- [ ] `kinematics/mecanum.{h,cpp}` (renamed from `mecanum_kinematics.*`),
      class `Kinematics::Mecanum`.
- [ ] `hal/motor_driver.h` (renamed from `motor_board.h`), interface
      `Hal::MotorDriver`.
- [ ] `hardware/hiwonder/hiwonder_driver.{h,cpp}` (renamed from
      `hiwonder_board.*`), class `Hardware::HiwonderDriver`.
- [ ] `hardware/generic/motor_driver_channel.{h,cpp}` (renamed from
      `board_motor.*`), class `Hardware::MotorDriverChannel`.
- [ ] All four renames are name-only — zero behavior change;
      `HiwonderDriver`/`MotorDriverChannel` keep their "NOT WIRED IN YET"
      header verbatim.
- [ ] Every call site, `#include`, and prose reference
      (`hardware/DESIGN.md`, `hal/DESIGN.md`) updated repo-wide, excluding
      `.claude/worktrees/rogo-revival/`.
- [ ] A fresh grep of the full ~49-file explicit build/test source-list
      set for `board_motor`/`hiwonder_board` (old names) run **before**
      renaming, and again for `motor_driver_channel`/`hiwonder_driver`
      (new names) run **after** — both results cited in Completion Notes.
      If either old file turns out to be named explicitly somewhere after
      all (contradicting the inherited "glob-only" claim), that reference
      is updated too, not left broken.
- [ ] `just build-sim` and ARM build both clean.
- [ ] `test_layer_isolation.py` still passes (within-layer renames only,
      no boundary change).

## Testing

- **Existing tests to run**: `just build-sim`, `uv run python -m pytest
  src/tests/sim/unit/test_layer_isolation.py`, ARM build, plus the
  before/after explicit-list grep described above.
- **New tests to write**: none — pure rename.
- **Verification command**: `just build-sim && uv run python -m pytest
  src/tests/sim/unit/test_layer_isolation.py`
