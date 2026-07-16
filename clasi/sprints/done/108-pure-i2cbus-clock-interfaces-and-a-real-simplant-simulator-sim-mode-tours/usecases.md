---
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 108 Use Cases

## SUC-038: Firmware developer swaps the I2C bus implementation without touching device leaves
Parent: UC-DEVICES (device-subsystem seam)

- **Actor**: Firmware developer
- **Preconditions**: `source/devices/i2c_bus.h` still declares a concrete
  class with two `#ifdef HOST_BUILD` forks; a host build and an ARM build
  cannot both link the same `.h` without excluding one `.cpp` by CMake
  filter.
- **Main Flow**:
  1. Developer reduces `I2CBus` to a pure abstract interface (`write`,
     `read`, `clearanceSafetyNetCount`, virtual dtor).
  2. Developer adds `MicroBitI2CBus : I2CBus` holding the real CODAL
     machinery, in `source/devices/microbit_i2c_bus.{h,cpp}`.
  3. `main.cpp:99`'s injection slot constructs a `MicroBitI2CBus` in the
     `I2CBus&` reference every device leaf (`NezhaMotor`, `Otos`,
     `LineSensorLeaf`, `ColorSensorLeaf`) already holds by reference.
  4. The ARM CMake glob never sees a host `.cpp`; the `i2c_bus_host.cpp`
     FILTER-EXCLUDE line is deleted because there is no such file anymore.
- **Postconditions**: `source/devices/i2c_bus.h` has zero `#ifdef
  HOST_BUILD`, zero private data, and no scripted-fake surface. A new bus
  implementation (real or simulated) can be substituted purely by what is
  constructed and injected, with no `.h` or CMake change.
- **Acceptance Criteria**:
  - [ ] `grep -n "#ifdef HOST_BUILD\|#ifndef HOST_BUILD" source/devices/i2c_bus.h` returns nothing.
  - [ ] `python build.py --fw-only` builds the ARM firmware unchanged.
  - [ ] `CMakeLists.txt`'s `i2c_bus_host.cpp` FILTER-EXCLUDE line is gone (the file no longer exists).

## SUC-039: Firmware developer swaps the Clock/Sleeper implementation without an `#ifdef` fork
Parent: UC-DEVICES

- **Actor**: Firmware developer
- **Preconditions**: `source/devices/clock.h` declares `Clock`/`Sleeper` with
  an `#ifdef HOST_BUILD` fork, the last device-level `#ifdef HOST_BUILD` in
  `source/devices/` once SUC-038 lands.
- **Main Flow**:
  1. Developer reduces `Clock`/`Sleeper` to pure interfaces.
  2. Developer adds `MicroBitClock`/`MicroBitSleeper` (real,
     `source/devices/microbit_clock.{h,cpp}`) wrapping
     `system_timer_current_time_us()`/`fiber_sleep()`/`schedule()`.
  3. Developer adds `SimClock`/`SimSleeper` under the new
     `tests/_infra/sim/` sim-harness tree (steppable counter + sleep/yield
     counters, mirroring the deleted `clock_host.cpp`'s inspection surface).
  4. `main.cpp` and the sim harness are updated to the new concrete types;
     `clock_real.cpp`/`clock_host.cpp` and the matching CMake exclude line
     are deleted.
- **Postconditions**: `grep -rn "HOST_BUILD" source/devices/` returns
  nothing at all — the device subsystem is `#ifdef`-free end to end.
- **Acceptance Criteria**:
  - [ ] `grep -rn "HOST_BUILD" source/devices/` is empty.
  - [ ] `python build.py --fw-only` builds unchanged.
  - [ ] Full pytest gate (`uv run python -m pytest tests/sim`) is green.

## SUC-040: Test engineer scripts I2C register behavior in Python via a SimPlant hook
Parent: UC-SIM (host simulator)

- **Actor**: Test engineer / CI
- **Preconditions**: The 13 register-level scenarios (wrong OTOS product
  ID, motor NAK, sensor-probe absence, boot-detection sequence, …) used to
  live as C++ harnesses driving a scripted FIFO fake bus
  (`i2c_bus_host.cpp`'s `scriptWrite`/`scriptRead`) that has been deleted.
- **Main Flow**:
  1. Test engineer boots the ctypes sim (`sim_ctypes`/`sim_loop.py`) against
     the real compiled firmware graph over `SimPlant`.
  2. Test registers a read or write hook via `sim_set_read_hook`/
     `sim_set_write_hook`, injecting the specific register-level scenario
     (e.g., an OTOS product-ID mismatch, a motor-address NAK).
  3. The hook decides per-call whether to pass through to
     `sim_default_read`/`sim_default_write` (SimPlant's live physics-backed
     response) or fully override the bytes/status.
  4. Test steps the sim and asserts on resulting telemetry or device state
     (e.g. `present()==false`).
- **Postconditions**: All scripting lives in Python, against one honest
  simulator bus; no C++ scripted-FIFO fake exists anywhere in the tree.
- **Acceptance Criteria**:
  - [ ] `tests/sim/support/sim_api.{h,cpp}` and every former
        `tests/sim/unit/*_harness.cpp` register-scripting file are deleted.
  - [ ] Each of the 13 former scenarios has an equivalent Python test using
        a registered hook, and `uv run python -m pytest tests/sim` is green.

## SUC-041: Test engineer runs a whole-robot sim scenario against SimPlant's live physics
Parent: UC-SIM

- **Actor**: Test engineer / CI
- **Preconditions**: The prior sim ("SimApi" + `DutyPredictor`) *predicted*
  firmware I2C behavior from a duty-write count instead of *responding* to
  the actual wire bytes, and desynced under an arbitrary twist stream (left
  encoder freezes, right runs away — the divergence bug).
- **Main Flow**:
  1. `sim_harness` constructs the real `App::RobotLoop` graph with a
     `SimPlant` in the `I2CBus&` slot (no scripted fake, no predictor).
  2. `SimPlant::defaultWrite()` parses the actual Nezha `0x60`/`0x46` frames
     and OTOS registers off the wire; `defaultRead()` returns the live,
     physics-integrated encoder/pose bytes.
  3. `SimPlant::tick(dt)` steps both `WheelPlant`s and the `OtosPlant` from
     the duty that was actually parsed off the wire — never from a
     predicted/back-channel value.
  4. Test drives a straight twist through the harness and asserts heading
     stays near zero (no divergence).
- **Postconditions**: The whole-robot scenario tests
  (`sim_api`/`profiled_motion`/`scripted_twist_demo`/`fault_knobs`) run
  against `SimPlant`, and a standalone straight-twist check demonstrates
  the divergence bug is gone.
- **Acceptance Criteria**:
  - [ ] `tests/_infra/sim/live_sim.h` and the `Responder` seam are deleted.
  - [ ] The 4 migrated system tests pass against `SimPlant`/`sim_harness`.
  - [ ] A standalone driver shows a straight twist keeps heading ~0 over the
        full run.

## SUC-042: TestGUI operator runs Tour 1 in Sim mode and watches the trace draw
Parent: UC-TESTGUI (sprint 107's tour-execution surface)

- **Actor**: TestGUI operator
- **Preconditions**: Sprint 107 scoped tour execution to real-hardware
  transports only; `SimTransport` tour support was explicitly deferred
  (`clasi/issues/sim-api-ctypes-abi-for-sim-mode-tours.md`) because no
  ctypes ABI existed over any SimPlant-shaped harness, and the Tour
  buttons were gated off for Sim in `__main__.py`.
- **Main Flow**:
  1. Operator launches `just testgui`, selects the Sim backend, and presses
     Connect.
  2. `SimTransport` constructs a `sim_loop.TwistTransport`-shaped object
     over the new `sim_ctypes` ABI (create/step/inject_twist/inject_stop/
     drain_tlm/true-pose) instead of the dead `sim_conn.SimConnection`.
  3. Operator presses **Tour 1** (now un-gated for Sim). `planner/tour.py`'s
     `run_tour()` drives the same twist-based executor sprint 107 built for
     hardware, now against the sim loop.
  4. The canvas draws the trace as legs execute; tour closure is asserted
     finite/small in a headless equivalent test.
- **Postconditions**: Sim-mode tours are a first-class, headless-runnable,
  physics-backed CI path, not merely a hardware-only capability.
- **Acceptance Criteria**:
  - [ ] `host/robot_radio/io/sim_conn.py` is deleted; `sim_loop.py` exists
        and satisfies the `TwistTransport`-shaped protocol
        (`twist()`/`stop()`/`read_pending_binary_tlm_frames()`).
  - [ ] Headless: a Tour 1 run through `sim_loop` completes every leg with
        finite/small closure.
  - [ ] Manual/bench: `just testgui` → Connect (Sim) → **Tour 1** → the
        trace draws on the canvas.

## SUC-043: TestGUI operator gets a clear result from Sim-mode single commands, never a dead-verb crash
Parent: UC-TESTGUI

- **Actor**: TestGUI operator
- **Preconditions**: `testgui/binary_bridge.py` translates `R`/`TURN`/`G`
  into a `segment`/`replace` `CommandEnvelope` arm that no longer exists in
  `protos/envelope.proto` (the oneof carries only `twist`/`config`/`stop`);
  a prior launch-unblock (107-003) guarded the module-level import so the
  GUI process itself no longer crashes, but `translate_command()` still
  returns a uniform "unavailable" reply for every verb it used to
  translate.
- **Main Flow**:
  1. Operator launches the TestGUI in Sim mode; `transport.py` imports
     `binary_bridge` successfully (no `ImportError`).
  2. `SimTransport`'s command path (unaffected by the historical
     `legacy_render`/`legacy_verbs` deletion, since it never routed through
     `binary_bridge.translate_command()`'s segment/replace builders) is
     verified not to depend on that dead translation as Stage 3's rewrite
     lands.
  3. If a manual command row would otherwise construct a `segment`/
     `replace` envelope in Sim mode, it surfaces an explicit, user-visible
     message rather than silently failing or crashing the process.
- **Postconditions**: The GUI launches and connects in Sim mode reliably;
  no code path reachable from `SimTransport` constructs a dead wire arm.
- **Acceptance Criteria**:
  - [ ] `python -m robot_radio.testgui` (Sim backend) launches without
        `ImportError`.
  - [ ] Nothing in `SimTransport`'s call graph invokes
        `binary_bridge.translate_command()`'s `segment`/`replace` builders.

## SUC-044: Firmware developer trusts the color sensor's absent/present latch
Parent: UC-DEVICES

- **Actor**: Firmware developer / bench operator
- **Preconditions**: `Devices::ColorSensorLeaf::beginStep`
  (`source/devices/color_sensor.cpp:57-70`) probes via a status-ignoring
  `readReg8()`; a NAK'd read leaves `out=0`, and `en==0x00` is exactly the
  "detected" condition, so a robot with NO color sensor latches
  `present()==true` and issues failing APDS transactions forever.
- **Main Flow**:
  1. `beginStep()`'s APDS probe is changed to call the status-returning
     `readReg8Status()` (already present on the class, used elsewhere) and
     require the transaction to report OK before concluding `en==0x00`
     means "detected."
  2. A Python hook test (SUC-040's ABI) NAKs the APDS probe address and
     asserts `present()` latches `false`.
  3. On the bench, an image booted with the color sensor unplugged shows
     `present()==false` and the perception slot skips it — no recurring
     bus errors.
- **Postconditions**: A NAK'd probe can never be mistaken for "device
  present."
- **Acceptance Criteria**:
  - [ ] `beginStep()`'s APDS probe path checks transaction status before
        concluding presence.
  - [ ] A Python hook test: APDS probe NAK'd -> `present()==false`.
  - [ ] Bench: color sensor unplugged -> `present()==false`, no recurring
        bus errors in I2C diagnostics.
