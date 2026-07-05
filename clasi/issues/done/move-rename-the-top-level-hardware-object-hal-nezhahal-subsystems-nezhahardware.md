---
status: done
---

# Move + rename the top-level hardware object: `Hal::NezhaHal` → `Subsystems::NezhaHardware`

## Context

While reviewing `source/hal/nezha/nezha_hal.h`, the concern was raised that the
Nezha HAL had "partly been turned into a subsystem" — it wears the faceplate
contract (`apply()`/`tick()`/`motor()`) but sits in `namespace Hal` /
`source/hal/`, not in `namespace Subsystems` / `source/subsystems/`.

Investigation confirmed the code was **not** half-migrated: "faceplate" is a
contract applied deliberately at two tiers (`Hal::Motor` and
`Subsystems::Drivetrain` are both "faceplates"), and the HAL was intentionally a
tier below Subsystems. But the stakeholder decided the **top-level hardware
aggregator genuinely *is* a subsystem** — it owns the bus + four motors, runs the
brick flip-flop schedule, and distributes addressed commands (a peer of
`Drivetrain`, not a per-device faceplate). Decisions:

- **Move + rename** the aggregator: `Hal::NezhaHal` → `Subsystems::NezhaHardware`,
  files → `source/subsystems/nezha_hardware.{h,cpp}`. (It drops "Hal" from its
  name because it's leaving `namespace Hal`.)
- **Rename its command-edge types** to track the consumer (naming rule 4):
  `CommandProcessorToHalCommand` → `CommandProcessorToHardwareCommand`,
  `DrivetrainToHalCommand` → `DrivetrainToHardwareCommand`.
- **Keep the HAL *layer* vocabulary**: `namespace Hal`, `source/hal/`,
  `hal/capability/hal_command.h` (filename), `Hal::Motor`, `Hal::NezhaMotor`,
  `Hal::AddressedMotorCommand`, and `Hal::kNezhaDeviceAddr` all stay — that's
  where the individual hardware elements live.

Done **out of process** (OOP) — direct, targeted change, no sprint/tickets. The
source-edit gate is satisfied: [.claude/rules/source-code.md](.claude/rules/source-code.md)
accepts "stakeholder said out of process" in lieu of an in-progress ticket.

**Why it's safe / preserves the layering:** nothing that stays in `Hal`
references the aggregator as a *type* (only comments) — so the move creates **zero
new `Hal → Subsystems` dependencies**. `Subsystems::NezhaHardware` depending on
`Hal::NezhaMotor`/`Hal::Motor`/`Hal::…Command` is the blessed `Subsystems → Hal`
direction. Behavior is byte-for-byte unchanged — this is namespace + rename +
relocation only.

## ⚠️ This is a *targeted token* rename, NOT `s/Hal/Hardware/g`

Most occurrences of "Hal" must **stay**. Only these exact tokens change:

| Rename (word-boundary) | Keep unchanged |
|---|---|
| `NezhaHal` → `NezhaHardware` (class) | `namespace Hal`, `Hal::` qualifier on retained types |
| `nezha_hal.h/.cpp` → `nezha_hardware.h/.cpp` (files) | `source/hal/`, `hal/capability/`, `hal/nezha/` dirs & include paths |
| `CommandProcessorToHalCommand` → `…ToHardwareCommand` | `hal_command.h` (filename) |
| `DrivetrainToHalCommand` → `DrivetrainToHardwareCommand` | `Hal::Motor`, `Hal::NezhaMotor`, `Hal::AddressedMotorCommand` |
| `hasHalCommand`/`halCommand` → `hasHardwareCommand`/`hardwareCommand` | `Hal::kNezhaDeviceAddr` |
| the `hal` instance/pointer handle → `hardware` | `HALT` verb, unrelated words |

Note the two-part edit on the class qualifier: `Hal::NezhaHal` →
`Subsystems::NezhaHardware` (namespace changes **and** the class renames).

## The change

### 1. Relocate + rename the files (preserve history)

- `git mv source/hal/nezha/nezha_hal.h  → source/subsystems/nezha_hardware.h`
- `git mv source/hal/nezha/nezha_hal.cpp → source/subsystems/nezha_hardware.cpp`

No CMake edit — [CMakeLists.txt:256](CMakeLists.txt#L256) recursively globs
`source/**/*.cpp`. `nezha_motor.{h,cpp}` and `motor_slew.h` stay in `source/hal/nezha/`.

### 2. Rewrite the moved files

In `nezha_hardware.{h,cpp}`:
- `namespace Hal {` → `namespace Subsystems {` (+ closing comment).
- Class + ctor `NezhaHal` → `NezhaHardware`.
- Qualify names that stay in `Hal` (Google style — no `using namespace`; the
  `.cpp` may use a few targeted `using Hal::…;` declarations): `Motor&` →
  `Hal::Motor&`; `NezhaMotor` members/`motorAt()` → `Hal::NezhaMotor`;
  `kNezhaDeviceAddr` → `Hal::kNezhaDeviceAddr`.
- `apply()` params → `Hal::CommandProcessorToHardwareCommand` /
  `Hal::DrivetrainToHardwareCommand`.
- `nezha_hardware.cpp` self-include → `"subsystems/nezha_hardware.h"`.
- Rewrite each file's header comment so it describes a Subsystems-tier
  distribution subsystem, not a HAL-tier object.

### 3. Rename the edge-type structs

[source/hal/capability/hal_command.h](source/hal/capability/hal_command.h)
(stays `namespace Hal`, keeps filename): struct `CommandProcessorToHalCommand`
(`:49`) → `CommandProcessorToHardwareCommand`; struct `DrivetrainToHalCommand`
(`:60`) → `DrivetrainToHardwareCommand`; `AddressedMotorCommand` unchanged.
Update the Decision-1 rationale comment: the struct *placement* is still correct,
but the stated *reason* shifts from "avoid a `Hal → Subsystems` include" to
"avoid a `Drivetrain ↔ NezhaHardware` mutual include" — keep it truthful.

### 4. Update producers, wiring, and the outbox

- [source/subsystems/drivetrain.{h,cpp}](source/subsystems/drivetrain.h) —
  `Hal::DrivetrainToHalCommand` → `Hal::DrivetrainToHardwareCommand` (`takeCommand()`
  return `h:123`, `heldCommand_` `h:201`, `cpp:194`) + comment mentions.
- [source/commands/dev_commands.h](source/commands/dev_commands.h) — include
  (`:118`) → `subsystems/nezha_hardware.h`; pointer `Hal::NezhaHal* hal` (`:199`)
  → `Subsystems::NezhaHardware* hardware`; `Hal::NezhaHal::kPortCount` (`:211`) →
  `Subsystems::NezhaHardware::kPortCount`; outbox fields `hasHalCommand`/
  `halCommand` (`:206-207`) → `hasHardwareCommand`/`hardwareCommand`;
  `buildBroadcastNeutral()` return type (`:234`).
- [source/commands/dev_commands.cpp](source/commands/dev_commands.cpp) — every
  `state.halCommand`/`state.hasHalCommand` (`:358-362, 841-847, 883-884`) →
  `hardwareCommand`/`hasHardwareCommand`; `state.hal->motor(...)` (`:418, 453,
  765-766, 869`) → `state.hardware->motor(...)`; `Hal::CommandProcessorToHalCommand`
  (`:909-910`) → `…ToHardwareCommand`; `Hal::NezhaHal::kPortCount` (`:868`).
- [source/main.cpp](source/main.cpp) — include (`:71`) → `subsystems/nezha_hardware.h`;
  instance `hal` → `hardware` (`:170-171, 199, 219-220, 237, 259, 272, 274, 282,
  292` and the header comment block `:8, 15-41`); `Hal::NezhaHal` →
  `Subsystems::NezhaHardware` (`:128, 141, 170, 205`); `devState.hasHalCommand`/
  `devState.halCommand` (`:258-260`) → `…Hardware…`; `devState.hal = &hal` →
  `devState.hardware = &hardware`.

### 5. Update the tests

- [tests/sim/unit/nezha_flipflop_harness.cpp](tests/sim/unit/nezha_flipflop_harness.cpp)
  — include (`:70`) → `subsystems/nezha_hardware.h`; `Hal::NezhaHal` →
  `Subsystems::NezhaHardware` (all sites); `Hal::CommandProcessorToHalCommand` /
  `Hal::DrivetrainToHalCommand` → `…ToHardwareCommand`; scenario fn
  `scenarioDrivetrainToHalCommandForwarding` → `…HardwareCommand…`.
- [tests/sim/unit/dev_command_outbox_harness.cpp](tests/sim/unit/dev_command_outbox_harness.cpp)
  — include (`:39`); `Hal::NezhaHal` → `Subsystems::NezhaHardware`;
  `f.state.halCommand`/`hasHalCommand` → `…Hardware…`; `Hal::CommandProcessorToHalCommand`
  (`:347`) → `…ToHardwareCommand`.
- [tests/sim/unit/drivetrain_harness.cpp](tests/sim/unit/drivetrain_harness.cpp)
  — `Hal::DrivetrainToHalCommand` (`:221, 238, 262, 291, 307`) → `…ToHardwareCommand`.
- **Test-runner source-path constants (must move with the file):**
  [test_nezha_flipflop.py:31](tests/sim/unit/test_nezha_flipflop.py#L31) and
  [test_dev_command_outbox.py:41](tests/sim/unit/test_dev_command_outbox.py#L41)
  — `_NEZHA_HAL_SRC = _SOURCE_DIR / "hal" / "nezha" / "nezha_hal.cpp"` →
  `_SOURCE_DIR / "subsystems" / "nezha_hardware.cpp"` (+ docstring path mentions).

### 6. Comment sweep

`nezha_motor.{h,cpp}` and `hal_command.h` comments reference "NezhaHal" — update
to "NezhaHardware" for accuracy (these are the retained-layer files; only the
prose changes, no code). Test assertion strings like "stages a HAL command" →
"stages a hardware command" where they describe the renamed field/type.

## Verification

1. **Firmware compiles** — `just build` (or `mbdeploy deploy --build` when
   flashing). Confirms the recursive glob found the moved `.cpp` and every
   caller re-qualified + renamed cleanly.
2. **Sim unit tests green** — `uv run python -m pytest tests/sim` (per
   [source-code.md](.claude/rules/source-code.md)). The flip-flop, outbox, and
   drivetrain harnesses all reference the renamed types and prove the sweep holds.
3. **Grep gate** — after editing, `rg -n "NezhaHal|ToHalCommand|hasHalCommand|halCommand"`
   over `source/` + `tests/` (excluding `**/done/**`) must return **only**
   intended survivors (none, ideally) — catches any missed call site (see memory
   `rename-sprint-latent-call-site-breakage`).
4. **Bench smoke on the stand (recommended)** — touches the HAL, so
   [hardware-bench-testing.md](.claude/rules/hardware-bench-testing.md)'s gate
   applies; behavior is unchanged, so this confirms no regression: flash, verify
   encoders read and both wheels spin with encoders climbing (`docs/protocol-v2.md`
   §13 / DEV family). Safe — robot is on the stand.
5. **Finalize** — commit (branch first if preferred over `master`), then
   `dotconfig version bump` + `chore: bump version` per
   [git-commits.md](.claude/rules/git-commits.md).
