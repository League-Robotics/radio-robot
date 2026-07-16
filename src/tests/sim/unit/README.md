# src/tests/sim/unit/

Per-module `App::`/`Devices::` host-build harnesses -- one `*_harness.cpp` +
`test_*.py` pair per module (e.g. `test_devices_motor.py`,
`test_app_robot_loop.py`). Each `test_*.py` compiles its own throwaway
binary via `subprocess` (no shared build step, no shared pytest fixture --
see `src/tests/sim/conftest.py`'s own file header) and asserts it exits 0.

## Register-level scripting: SimPlant + a hook, not a second scripted bus

Sprint 108 deleted the old scripted-FIFO `Devices::I2CBus` fake
(`src/firm/devices/i2c_bus_host.cpp`, `queueWrite()`/`queueRead()`/
`txnCount()`/`errCount()`) -- `Devices::I2CBus` is now a pure interface
(`src/firm/devices/i2c_bus.h`), with exactly two concrete implementations:
`Devices::MicroBitI2CBus` (the real ARM chip) and `TestSim::SimPlant`
(`tests/_infra/sim/sim_plant.h`) -- a REAL, physics-backed simulator that
parses the actual Nezha/OTOS wire protocol and integrates real wheel/OTOS
physics (`src/tests/sim/plant/`).

A harness that needs a scenario SimPlant's own live physics can express
directly (a twist ramping to a target velocity, a pivot's resulting heading,
whole-loop boot+cycle+telemetry behavior) should compose a `TestSim::
SimPlant` (or, for a whole `App::RobotLoop` graph, `TestSim::SimHarness`,
`tests/_infra/sim/sim_harness.h`) and drive it live -- see
`src/tests/sim/plant/plant_harness.cpp` and `src/tests/sim/system/` for that
pattern, and `src/tests/testgui/test_sim_loop.py` for the equivalent pure-Python
`robot_radio.io.sim_loop.SimLoop` ctypes wrapper over the same ABI.

A harness that needs EXACT, deterministic per-call register-level control
that SimPlant's live responses cannot give directly (a specific NAK, a
specific encoder count, a wrong product-ID byte, an exact transaction-count
budget across two motors + OTOS on one shared bus) registers a
`TestSim::ScriptedI2CHook` (`scripted_i2c_hook.h`, this directory) on a
`TestSim::SimPlant` instance instead. `ScriptedI2CHook` reproduces the
deleted fake's exact FIFO-scripting semantics (`queueWrite()`/
`queueRead()`, `txnCount()`/`errCount()`/`lastErr()`, the "an unscripted
call returns a distinct mismatch status rather than crashing" convention)
but implements it AS a `SimPlant` read/write hook -- see that header's own
file comment for the full API and usage pattern. Every C++ harness in this
directory that scripts register-level behavior (`devices_motor_harness.cpp`,
`devices_otos_harness.cpp`, `devices_sensors_harness.cpp`,
`app_preamble_harness.cpp`, `app_odometry_harness.cpp`,
`app_drive_harness.cpp`, `app_robot_loop_harness.cpp`) uses this pattern;
`devices_color_sensor_apds_probe_harness.cpp` (ticket 108-008) is the
smaller, single-scenario precedent this pattern generalizes.

**When adding a new register-level scenario**: construct a
`TestSim::SimPlant plant;` and a `TestSim::ScriptedI2CHook bus(plant);`,
script the exchange with `bus.queueWrite()`/`bus.queueRead()`, and pass
`plant` (not `bus`) to whatever `Devices::` leaf constructor takes an
`I2CBus&` -- `SimPlant::write()`/`read()` dispatch through the hook
automatically. Do NOT write a new standalone scripted-fake `Devices::I2CBus`
subclass -- that mechanism was deliberately deleted (sprint 108 ticket 001);
`ScriptedI2CHook` is the one sanctioned register-level scripting seam.

## `devices_i2c_bus_harness.cpp` -- deleted, not migrated

The old `devices_i2c_bus_harness.cpp` (ticket DB-003) tested the deleted
scripted fake's OWN bookkeeping mechanics -- FIFO ordering, per-device
`txnCount()`/`errCount()`/`lastErr()` counters, the lazy `preClear`/
`postClear` clearance-timer bookkeeping (`clear()`, `clearanceSafetyNet
Count()`), and the IRQ-guard default-on flag -- i.e. it was a test of the
fake's own internals, not of any device leaf's behavior. That class and its
file are gone; the equivalent bookkeeping now lives entirely on
`Devices::MicroBitI2CBus` (`src/firm/devices/microbit_i2c_bus.{h,cpp}`), which
is CODAL/ARM-only and not host-buildable, so there is nothing left on the
host side to script or assert those mechanics against. `TestSim::SimPlant`
deliberately does NOT reimplement that bookkeeping (`clearanceSafetyNet
Count()` always returns 0 -- see `sim_plant.h`'s own comment: "SimPlant has
no clearance timers... a hook has nothing useful to do with them"), so
fabricating a host test against it would test nothing real. Sprint 108
ticket 009 deleted this harness outright rather than porting it, per that
ticket's own "read the original file's test cases before deciding, then
document exactly what coverage was dropped and why it's not fabricable"
guidance.
