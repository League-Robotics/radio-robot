---
status: approved
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 081 Use Cases

`docs/usecases.md`'s UC-001..UC-019 describe production motion/sensor/discovery
use cases reached through the full Planner/Superstructure stack that lives in
`source_old/` (parked by sprint 077's greenfield rebuild). This sprint has no
parent there either — it stands up a **developer-facing test capability**
(a host-side simulator for the new `source/` tree) with no analogue in the
production use-case catalog. Each SUC below maps 1:1 onto one of this
sprint's six tickets.

## SUC-001: Extract the shared velocity PID without changing real-robot behavior
Parent: None (infrastructure prerequisite for SUC-003)

- **Actor**: Firmware developer verifying the motor control loop
- **Preconditions**: `NezhaMotor::runVelocityPid()` (`source/hal/nezha/nezha_motor.cpp:291`,
  declared `nezha_motor.h:146`) is embedded directly in the Nezha leaf; no
  host-clean, reusable PID class exists yet.
- **Main Flow**:
  1. Developer extracts the control law into `Hal::MotorVelocityPid`
     (`source/hal/velocity_pid.{h,cpp}`) — same gains, same anti-windup
     back-calculation, same `dt<=0 -> kNominalDt` fallback, same output
     domain (`[-1,1]` duty fraction).
  2. `NezhaMotor` is refactored to own a `MotorVelocityPid` member and call
     `compute(target, measured, dt)` where it used to call
     `runVelocityPid(...)` directly — no other line in `tick()`'s 5-step
     sequence changes.
  3. Developer deploys to the robot (`mbdeploy deploy --build`) and runs a
     bench step-response comparison (command a velocity step, capture rise
     time / overshoot / settle) before and after the extraction.
- **Postconditions**: `Hal::MotorVelocityPid` is a pure, host-clean class
  with no I2C/CODAL dependency, usable by a future `Hal::SimMotor`
  (SUC-003) without re-deriving the control law.
- **Acceptance Criteria**:
  - [ ] Bench step-response (rise time, overshoot, settle) matches
        pre-extraction behavior within measurement noise — this is a
        hardware-bench-testing gate item (`.claude/rules/hardware-bench-testing.md`),
        not just a host-side test.
  - [ ] `Hal::MotorVelocityPid` compiles with no `#include "MicroBit.h"` and
        no I2C dependency.
  - [ ] Existing `tests/sim/unit/*` host-compiled harnesses (and any new one
        added for the extracted class) pass.

## SUC-002: Run the identical dev-loop body from both the firmware and the simulator
Parent: None (infrastructure prerequisite for SUC-003/SUC-004)

- **Actor**: Firmware developer; the simulator's C ABI caller (Python, via SUC-004)
- **Preconditions**: `source/main.cpp` inlines the whole dev-loop body
  (two-slice hardware tick, statement dispatch, outbox drain, Drivetrain
  governance, watchdog check) with no shared, host-clean extraction —
  `source/dev_loop.{h,cpp}`, `source/hal/capability/motor_hal.h`-equivalent
  seam, and `source/types/clock.h` are all absent from the tree (verified
  2026-07-05; sprint 079 rewrote the loop inline but did not extract it).
  `Subsystems::NezhaHardware` (renamed from `Hal::NezhaHal` the same day) is
  the only concrete implementation of the hardware-owner role.
- **Main Flow**:
  1. Developer introduces `Subsystems::Hardware` (`source/subsystems/hardware.h`),
     an abstract owner base exposing exactly the surface `devLoopTick` and
     the DEV command layer need: `motor(port)`, `tick(now)`, the two
     `apply()` overloads, `kPortCount`.
  2. `Subsystems::NezhaHardware` is retrofitted to `: public Subsystems::Hardware`
     (no behavior change — same methods, `override` added).
  3. Developer extracts `devLoopTick(DevLoop&, uint32_t now, const DevLoopStatement*)`
     into `source/dev_loop.{h,cpp}`, matching `main.cpp`'s real, current loop
     body exactly (not the design write-up's schematic pseudocode — see the
     architecture update's reconciliation section).
  4. `DevLoopState::hardware` retypes from `Subsystems::NezhaHardware*` to
     `Subsystems::Hardware*`; `commands/dev_commands.h`/`.cpp` no longer
     name the concrete `NezhaHardware` type.
  5. `source/types/clock.h`'s `systemClockNow()` replaces
     `system_commands.cpp`'s direct `system_timer_current_time()` call;
     `handleId`'s CODAL identity calls get a fixed host stand-in under
     `HOST_BUILD`.
  6. Developer deploys to the robot and runs the bench smoke sequence.
- **Postconditions**: `main.cpp` calls `devLoopTick()` instead of inlining
  the loop; the identical function will be the sim's `sim_tick`/`sim_command`
  entry point once SUC-004 exists.
- **Acceptance Criteria**:
  - [ ] ARM build behavior is byte-identical at the wire (bench smoke per
        `.claude/rules/hardware-bench-testing.md`): PING/DEV M/DEV DT family,
        watchdog `EVT dev_watchdog`, all round-trip exactly as before.
  - [ ] `source/dev_loop.{h,cpp}` compiles with no `#include "MicroBit.h"`
        and no `Subsystems::Communicator` dependency.
  - [ ] `commands/dev_commands.h`/`.cpp` include `subsystems/hardware.h`,
        not `subsystems/nezha_hardware.h`.

## SUC-003: Simulate the motors and OTOS behind the same hardware-owner seam
Parent: None (the core simulated-plant capability)

- **Actor**: Firmware developer writing or running a sim-backed test
- **Preconditions**: SUC-001 (`Hal::MotorVelocityPid`) and SUC-002
  (`Subsystems::Hardware`, `devLoopTick`) exist. `source/hal/sim/` is absent;
  `Hal::Odometer` (`source/hal/capability/odometer.h`) exists but is declared
  only, with no concrete leaf yet.
- **Main Flow**:
  1. Developer ports `Hal::PhysicsWorld` (motor/pose/encoder plant only —
     aux line/color/port truth channels dropped, per the design's resolved
     decision 2) to `source/hal/sim/physics_world.{h,cpp}`.
  2. Developer implements `Hal::SimMotor : public Hal::Motor` (VELOCITY mode
     calls the SAME `Hal::MotorVelocityPid` NezhaMotor calls) and
     `Hal::SimOdometer : public Hal::Odometer` under `source/hal/sim/`.
  3. Developer implements `Subsystems::SimHardware : public Subsystems::Hardware`
     (`source/subsystems/sim_hardware.{h,cpp}`) — a Subsystems-tier peer of
     `NezhaHardware`, owning the one `PhysicsWorld` + 4 `SimMotor`s + the
     `SimOdometer`, with its own same-`now` re-entry guard (see the
     architecture update's dt=0 rationale).
  4. Developer adds `Hal::` free setter functions (one per error knob:
     motor scale/slip/noise, stiction/lag, OTOS noise/scale/drift, trackwidth,
     body scrub, plant port binding) in `source/hal/sim/sim_setters.h`.
- **Postconditions**: A `Subsystems::SimHardware` instance is a drop-in
  substitute for `Subsystems::NezhaHardware` behind `devLoopTick`, with no
  wire-visible difference (no `SIMSET`/`SIMGET`, no sim-only `TLM` fields).
- **Acceptance Criteria**:
  - [ ] A standalone-compiled harness (matching the existing
        `tests/sim/unit/*_harness.cpp` ad hoc-compile convention — no CMake
        needed yet) proves: a repeated `SimHardware::tick(now)` call at an
        **unchanged** `now` does not re-invoke any `SimMotor`'s
        `MotorVelocityPid::compute()` a second time (the dt=0
        double-integration hazard — see architecture update).
  - [ ] All error knobs at zero -> true encoder == reported encoder == OTOS
        accumulator, bit-for-bit.
  - [ ] `SimMotor::capabilities()` reports `position=false`; `DEV M n POS`
        style commands answer `ERR unsupported` against it.
  - [ ] No `SIMSET`/`SIMGET` wire command and no sim-specific `TLM` field is
        introduced anywhere in `source/commands/` or `docs/protocol-v2.md`.

## SUC-004: Build and expose the simulator as a host-loadable shared library
Parent: None (host-tooling prerequisite for SUC-005)

- **Actor**: Firmware/host developer running `just build-sim`
- **Preconditions**: SUC-002 (`dev_loop.*`) and SUC-003
  (`Subsystems::SimHardware`) exist. `tests/_infra/sim/` does not exist in
  the working tree (only in the parked `tests_old/_infra/sim/`); `build.py`'s
  `build_host_sim()` and `just build-sim` already point at
  `tests/_infra/sim/` and self-heal the moment it reappears.
- **Main Flow**:
  1. Developer writes `tests/_infra/sim/CMakeLists.txt` with an **explicit**
     source list (`kinematics/*.cpp`, `subsystems/drivetrain.cpp`,
     `commands/{arg_parse,command_processor,dev_commands,system_commands}.cpp`,
     `dev_loop.cpp`, `hal/sim/*.cpp`, `hal/velocity_pid.cpp`,
     `types/clock_host.cpp`, `com/i2c_bus_host.cpp`, `sim_api.cpp`) —
     absent: `com/i2c_bus.cpp`, `communicator.*`, `hal/nezha/*.cpp`,
     `types/clock.cpp`, `main.cpp`.
  2. Developer writes `tests/_infra/sim/sim_api.cpp`: a `SimHandle` owning
     `Subsystems::SimHardware` + `Subsystems::Drivetrain` + `CommandProcessor`
     + `DevLoop`, a reply store, `sim_tick`/`sim_command` (the dt=0
     synchronous-command trick), and the ctypes-only knob/telemetry surface.
  3. Developer runs `just build-sim`.
- **Postconditions**: `tests/_infra/sim/build/libfirmware_host.{dylib,so}`
  exists and exposes the ~40-function C ABI `sim_conn.py` already expects.
- **Acceptance Criteria**:
  - [ ] `just build-sim` succeeds and produces `libfirmware_host.dylib` (or
        `.so`).
  - [ ] No `SIMSET`/`SIMGET` wire family and no sim-only `TLM` field exists;
        every error knob and every ground-truth read is reachable **only**
        via a `sim_*` ctypes entry point.
  - [ ] `sim_command(handle, line, ...)`'s dt=0 re-run does not double-count
        watchdog feeds, PID integration, or plant advancement (builds on
        SUC-003's guard).

## SUC-005: Drive the simulator from pytest and get real, passing sim tests
Parent: None (the sprint's headline deliverable)

- **Actor**: Any developer running `uv run python -m pytest tests/sim`
- **Preconditions**: SUC-004's shared library and C ABI exist.
  `tests/sim/conftest.py` is currently a documented placeholder (no
  `build_lib`/`sim` fixtures); `tests/sim/unit/test_placeholder.py` is the
  only real test collected under `tests/sim/`.
- **Main Flow**:
  1. Developer writes `tests/_infra/sim/firmware.py`'s `Sim` class (ctypes
     wrapper, context manager, `tick_for(total, step=24)`).
  2. Developer replaces `tests/sim/conftest.py`'s placeholder with a
     session-scoped `build_lib` fixture (runs `just build-sim` once) and a
     function-scoped `sim` fixture (fresh `Sim()` instance per test, issuing
     `DEV WD 3600000` immediately after create so the 1 s
     `SerialSilenceWatchdog` does not neutralize motors mid-`tick_for`).
  3. Developer fixes up `host/robot_radio/io/sim_conn.py` against the new
     ABI.
  4. Developer writes the first real tests: plant correctness (drive/turn
     geometry vs. truth), errored-observation split, velocity-PID response
     (step -> rise/overshoot/settle within the bench's own envelope, since
     sim and hardware now share `MotorVelocityPid`), protocol round-trips
     (PING/DEV family/`ERR unsupported`/watchdog `EVT`), and the
     determinism gate (same script twice -> bit-identical logs).
- **Postconditions**: `uv run python -m pytest tests/sim` collects and
  passes a real suite, not a placeholder.
- **Acceptance Criteria**:
  - [ ] `uv run python -m pytest tests/sim` is green and collects more than
        the placeholder test.
  - [ ] The zero-error determinism gate (SUC-003's acceptance) is exercised
        end-to-end through the Python `Sim` wrapper, not only at the C++
        harness level.
  - [ ] A dedicated test lowers the watchdog window and confirms the
        `EVT dev_watchdog` path fires.

## SUC-006: Regression-guard the encoder/OTOS/stiction error models with ported legacy tests
Parent: None (closes out the sim harness's test-value delivery)

- **Actor**: Any developer running the sim test suite
- **Preconditions**: SUC-005's harness and fixtures exist.
  `tests_old/simulation/` holds the pre-rebuild encoder-error, OTOS-error,
  and stiction/lag suites, unreachable from the new tree.
- **Main Flow**:
  1. Developer selects the highest-value suites from
     `tests_old/simulation/` (encoder scale/slip/noise, OTOS noise/scale/
     drift, stiction+lag response envelopes).
  2. Developer ports each to the new `Sim`/`sim_conn` API and the new
     `source/`-tree ctypes surface, adapting call sites where the ABI or
     naming has changed (never reintroducing a unit-suffixed or pre-rename
     `Hal` identifier along the way).
  3. Developer places them under `tests/sim/unit/` or `tests/sim/system/`
     per `tests/CLAUDE.md`'s domain split.
- **Postconditions**: The new tree has deterministic, off-hardware
  regression coverage for the error models that the encoder-wedge saga
  showed were previously exercisable only unreliably on the bench.
- **Acceptance Criteria**:
  - [ ] Ported encoder-error, OTOS-error, and stiction/lag suites pass under
        `uv run python -m pytest tests/sim`.
  - [ ] No ported test references a pre-rename `Hal::NezhaHal`/`...ToHalCommand`
        name or a unit-suffixed identifier.
  - [ ] EKF/fusion-dependent tests remain explicitly excluded (no firmware
        consumer of OTOS exists yet — see the architecture update's "OTOS gap"
        note) rather than silently skipped or mis-asserted.
