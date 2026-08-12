---
status: done
sprint: '136'
tickets:
- 136-003
- 136-004
- 136-005
- 136-006
- 136-007
- 136-008
- 136-009
---

# Firmware layering cleanup — interfaces to `hal/`, impls to `platform/microbit/`

## Description

The August 2026 platform/hardware/hal reorganization created the layer *directories*
but left several files in the wrong one, and left one directory (`com/`) outside the
layering entirely. The tree is meant to read `platform/` → `hardware/` → `hal/` →
`kinematics/` → `motion/` → `core/` with dependencies running strictly downward; today
it does not.

Specific mismatches:

- **`platform/clock.h` and `platform/i2c_bus.h` are pure interfaces** (abstract bases,
  no chip knowledge, `namespace Platform`) sitting in the directory whose job is
  per-target *implementations*. `hal/` is the directory for hardware abstractions and is
  uniformly `Hal::`.
- **`com/` holds concrete CODAL byte pipes in the global namespace.** `SerialPort`,
  `Radio` and `formatBanner()` are the only firmware symbols with no namespace at all.
  The abstraction they want — `Core::Transport` — is buried in `core/comms.h` behind
  `#ifndef HOST_BUILD`, together with two trivial pass-through adapters
  (`Core::SerialTransport`, `Core::RadioTransport`).
- **`com/radio_channel.h` is dead.** Zero includers repo-wide; no `radio_channel.cpp`
  exists, so `radiochan::load()`/`save()` have no definition anywhere. The radio channel
  has come from `Config::kRadioChannel` (baked from the robot JSON's
  `connection.radio_channel`) since that cutover — `main.cpp:141`.
- **`core/fake_otos.{h,cpp}` is a build variant nothing enables.** `-DFAKE_OTOS` is OFF
  by default, no robot JSON references it, and the host/sim build never compiles it (it
  uses `TestSim::OtosPlant`). It is commonly mistaken for simulation code; it is not —
  it is an ARM-only fallback for a real robot with no OTOS chip.
- **`core/differential_drive.{h,cpp}` is the wheel-speed control law**, not
  orchestration and not kinematics — `fastPid()` plus the Stage A/B/C correction,
  bias adaptation, and the stall/deficit latches. It holds `trackWidth` as a scalar and
  never calls into `Kinematics::`.
- **`Kinematics::DifferentialKinematics` / `MecanumKinematics` stutter** against their
  own namespace.
- **`src/motion/` and `src/sim/` are stale build residue.** `git ls-files` returns zero
  tracked files for both; neither holds a `.h` or `.cpp` outside `build/` and
  `__pycache__`. Their CMake targets (`motion_stop_condition_tests`,
  `motion_velocity_shaper_tests`, `motion_move_queue_chained_tests`) build files sprint
  128 ticket 014 deleted.

## Cause

Sprint 130's reorganization was scoped as a directory move plus include fixup. Several
placement questions were explicitly deferred rather than answered — `hal/DESIGN.md` §4
records two of them (the `Hal::Wheel` migration, and `Core::FakeOtos`'s home) — and
`com/` predates the layering entirely, so it was never brought under it. The stale
`src/motion/` and `src/sim/` trees are gitignored build output that survived their
sources' move into `src/firm/`.

## Proposed fix

Six phases, ordered so each is independently verifiable.

**1 — Dead code.** `rm -rf src/motion src/sim`. Delete `com/radio_channel.h`. Delete
`Core::FakeOtos` and the `FAKE_OTOS` flag entirely: `core/fake_otos.{h,cpp}`, the
`#ifdef FAKE_OTOS` member and ctor-init blocks in `core/boot_wiring.{h,cpp}` (`otos_`
binds `realOtos_` unconditionally; keep the `Hal::Otos&` seam), `option(FAKE_OTOS ...)`
in the root `CMakeLists.txt`, `--fake-otos` in `build.py`,
`src/tests/sim/unit/app_fake_otos_harness.cpp` + `test_app_fake_otos.py`, and the
comment references in `hardware/DESIGN.md`, `core/DESIGN.md`, `core/debug.h`,
`src/protos/commands.proto`, `devices_otos_harness.cpp`.

**2 — Interfaces to `hal/`.** Move `platform/clock.h` and `platform/i2c_bus.h` into
`hal/`, renaming `namespace Platform` → `namespace Hal` (`Hal::Clock`, `Hal::Sleeper`,
`Hal::I2CBus`). `Platform::MicroBitClock`/`MicroBitSleeper`/`MicroBitI2CBus` keep
`Platform::`; `TestSim::SimClock`/`SimSleeper`/`SimPlant` re-base onto `Hal::*`.
Rewrite the 14 firm includes plus 2 test harnesses.

This makes `platform/` an implementor of HAL interfaces, so
`src/tests/sim/unit/test_layer_isolation.py`'s rule table must change — `platform/`
gains `hal/` as an allowed include prefix:

```python
_LAYERS = (
    ("hal",      ("hal/",)),
    ("platform", ("platform/", "hal/")),
    ("hardware", ("hardware/", "hal/", "platform/")),
)
```

**3 — Dissolve `com/`.** Extract `Hal::Transport` into a new `hal/transport.h` (moved
verbatim from `core/comms.h`). Move the two byte pipes into `platform/microbit/`,
implementing `Hal::Transport` directly and absorbing the adapters:

- `com/serial_port.{h,cpp}` → `platform/microbit/microbit_serial_port.{h,cpp}`,
  `Platform::MicroBitSerialPort : public Hal::Transport`. `readLine`/`send`/
  `sendReliable` become `override` — the signatures already match exactly.
- `com/radio.{h,cpp}` → `platform/microbit/microbit_radio_link.{h,cpp}`,
  `Platform::MicroBitRadioLink : public Hal::Transport`. **Not** `MicroBitRadio` — that
  is CODAL's own global type, held here by reference. `poll()` becomes `readLine()`
  (same signature); `sendReliable()` forwards to `send()`, as `Core::RadioTransport`
  already does.
- `com/banner.{h,cpp}` → `platform/microbit/microbit_banner.{h,cpp}`,
  `Platform::formatBanner()`.

Then delete `com/` including its `DESIGN.md`, drop the `#ifndef HOST_BUILD` block from
`core/comms.{h,cpp}`, rename `Core::Transport` → `Hal::Transport` repo-wide, re-base
`TestSupport::FakeTransport`, and simplify `main.cpp` (the two adapter objects
disappear).

**4 — Control law out of `core/`.** New `src/firm/control/` holding
`Control::DifferentialDrive`. Pure relocation — the control law is untouched. Layer
position `hal/` → `control/` → `core/`; it gets no `test_layer_isolation.py` entry
(same as `core/`, `kinematics/`, `motion/`, which also carry cross-cutting `config/` and
`firm/types/` deps). Needs a co-located `control/DESIGN.md`.

The `Hal::Wheel` migration described in `hal/DESIGN.md` §4 stays deferred — that is a
behavioral change to the code that drives the robot and belongs in its own sprint.

**5 — Kinematics rename.** `differential_kinematics.{h,cpp}` → `differential.{h,cpp}`,
`mecanum_kinematics.*` → `mecanum.*`; `Kinematics::DifferentialKinematics` →
`Kinematics::Differential`, `MecanumKinematics` → `Mecanum`.

**5b — "Board" → "motor driver" rename.** Stakeholder directive (2026-08-11):
*"The HiWonder board and board motor should be called motor drivers, not boards.
Everything's a board."* These stay — HiWonder is a planned robot class, not dead
code — they are just misnamed:

| Today | Rename to |
|---|---|
| `Hal::MotorBoard` (`hal/motor_board.h`) | `Hal::MotorDriver` (`hal/motor_driver.h`) |
| `Hardware::HiwonderBoard` (`hardware/hiwonder/hiwonder_board.*`) | `Hardware::HiwonderDriver` (`hiwonder_driver.*`) |
| `Hardware::BoardMotor` (`hardware/generic/board_motor.*`) | one channel of a `MotorDriver` presented as a `Hal::Motor` — name it at sprint time; `DriverMotor` is the mechanical answer but reads poorly |

**Nearly free**: both concrete classes carry a "NOT WIRED IN YET" header and have
**zero callers** outside their own files. The only other references are prose in
`hardware/DESIGN.md` and `hal/DESIGN.md`. Do not confuse this with
`docs/hiwonder/hiwonder-motor-board.md` — that is a hardware wiring guide for a
physical product that really is called a board, and its filename should not change.

**6 — Documentation.** `CLAUDE.md`'s architecture block (including its stale claim that
`Motion::WheelVelocityPid` owns the control law — that class was deleted in sprint 128),
`docs/design/design.md` §2, `src/firm/DESIGN.md` (stale wholesale), and the co-located
`hal/`, `platform/`, `hardware/`, `core/`, `kinematics/` docs. Also
`.claude/rules/hardware-bench-testing.md`, which claims the firmware applies
`radiochan::kDefault` — already false. And the stale `src/motion` doc paths in
`src/protos/robot_config.proto` and the note strings in `data/robots/*.json`.

## Verification

Per phase, smoke only:

```bash
just build-sim                     # ~8s; catches host-link and include breakage
uv run python -m pytest src/tests/sim/unit/test_layer_isolation.py
```

Once, at the end:

```bash
uv run python3 build.py --clean    # ARM MICROBIT.hex + host sim library
uv run python -m pytest            # src/tests/{sim,unit,testgui}
```

Then the standing hardware gate (this touches the HAL and the command transport, so
tests alone do not close it) — on `gopiv`, addressed by UID
`9906360200052820049d38a46da36a83000000006e052820`: flash, sleep ~5 s, then
`src/tests/bench/twist_drive.py` and `src/tests/bench/move_protocol_bench.py`. `gopiv`
has no OTOS, line, or colour sensor (flags 216) — do not gate on those. Restore
`data/robots/active_robot.json` to `tovez.json` and rebuild afterward so the tracked
`src/firm/config/boot_config.cpp` returns to tovez values.

What each phase proves: phase 1 by the clean build (nothing referenced the deleted
code); phase 2 by `test_layer_isolation.py`, whose rule change is the point; phase 3 by
the ARM build (`com/` gone) *and* `just build-sim` (host still links without it);
phases 4-5 by the full pytest run, where the 24 hardcoded `_APP_SOURCES`-family lists
fail loudly if one is missed.

## Related

- `clasi/issues/proposal-platform-hardware-hal-core-reorganization.md` — the parent
  proposal; its "Open questions" section names `FakeOtos`'s placement and whether
  "platform" widens to cover peripheral-board families.
- `clasi/issues/later/fakeotos-belongs-in-devices-not-app-invariant-tension.md` and
  `clasi/issues/later/otos-fake-seam-should-be-one-interface-two-implementations.md` —
  both resolved by deleting `FakeOtos`.
- `src/firm/hal/DESIGN.md` §4 — the deferred `Hal::Wheel` migration, and why
  `Core::FakeOtos` was left in the composition layer.

**Traps for whoever executes this:**

1. `.claude/worktrees/rogo-revival/` is a second checkout holding a full
   pre-reorganization copy (`src/firm/app/`, `src/motion/`, `src/firm/devices/`). Every
   repo-wide grep or sed hits it twice — exclude it from every sweep.
2. The ARM image **globs** `src/firm/**/*.cpp` (root `CMakeLists.txt` lines 301-317), so
   moves inside `src/firm` are free for that target. Everything else is an explicit
   list: `src/firm/platform/host/CMakeLists.txt`, three `motion/**/CMakeLists.txt`, **24
   pytest files** with `_APP_SOURCES`-family lists, and ~20 more naming individual
   `_*_SRC` paths.
3. `hardware/generic/board_motor.cpp` and `hardware/hiwonder/hiwonder_board.cpp` reach
   the image **glob-only** — they appear in no explicit list anywhere.
4. `build.py` regenerates `src/firm/config/boot_config.cpp` from
   `data/robots/active_robot.json` and runs `dotconfig version bump`, so a build for a
   non-active robot rewrites tracked files.
