# DBX — Complete cutover: DeviceBus becomes the live device layer

Stakeholder directive (2026-07-13, overnight): after encoders were validated
working reliably on hardware via the DeviceBus bring-up image, do a COMPLETE
cutover — the real firmware's device layer (`Subsystems::NezhaHardware` + its
I2C flip-flop + `Hal::NezhaMotor` + `Hal::OtosOdometer`) is replaced by
`Devices::DeviceBus` (the fiber-owned subsystem). Validate on hardware, then
continue the rest of sprint 100.

## Approach (lowest-risk: adapter, motion stack unchanged)

`Subsystems::Hardware` is the interface the whole motion stack talks to
(`motor(i)→Hal::Motor&`, `motorState(i)→msg::MotorState`, `odometer()→
Hal::Odometer*`, `tick(now)`, `motorStates()`, `apply(...)`, `motorConfig(i)`,
`begin()`). `Hal::Motor`/`Hal::Odometer` are pure-virtual interfaces.

Create **`Subsystems::DeviceBusHardware : public Subsystems::Hardware`**,
backed by one owned `Devices::DeviceBus`, plus thin forwarding leaves:

- **`Subsystems::DeviceBusMotor : public Hal::Motor`** — a PURE PASSTHROUGH to
  a `Devices::Motor` handle. `setVelocity(v)`→`handle.setVelocity(v)`;
  `setDutyCycle(d)`→`handle.setDuty(d)`; `setNeutral(m)`→`handle.setNeutral`;
  `position()/velocity()/appliedDuty()/connected()`→handle readings;
  `tick(now)`→**NO-OP** (the DeviceBus fiber already runs the real
  collect+PID+armored-write cycle — do NOT run a second PID/armor here);
  `writeRawDuty/hardReset/softRebaseline`→forward to the handle or no-op;
  `capabilities()`→a fixed differential capability. This leaf holds NO control
  state of its own.
- **`Subsystems::DeviceBusOdometer : public Hal::Odometer`** — passthrough to
  the `Devices::Odometer` handle: `pose()`→convert `Devices::PoseReading`→
  `msg::PoseEstimate` (with a valid stamp when the reading is fresh);
  `connected()/present()`→handle; `tick(now)`→NO-OP (fiber owns it);
  `fusableThisPass()`→a one-shot read-and-clear derived from the handle's
  freshness (mirror `Hal::OtosOdometer::fusableThisPass()` semantics);
  `setPose/init/resetTracking/setLinearScalar/setAngularScalar`→handle.
- **`DeviceBusHardware`**: owns the `Devices::DeviceBus` (constructed against
  `uBit.i2c` / the real bus), `start()`s its fiber in `begin()`, holds the
  4 motor leaves + the odometer leaf. `motor(i)`→leaf; `motorState(i)`→leaf
  `.state()` (or convert the handle reading); `odometer()`→the odometer leaf;
  `tick(now)`→NO-OP or light bookkeeping (the fiber does the I/O async — the
  old `hardware_.tick()` flip-flop is GONE). `apply(...)` legacy routes:
  minimal/no-op as the current NezhaHardware does. Boot config: map the same
  `msg::MotorConfig`/OTOS values `boot_config.cpp` bakes into
  `Devices::MotorConfig`/`OtosConfig` (the bring-up's `buildMotorConfig()` in
  bringup_main.cpp already shows the exact numbers — reuse them).

## main.cpp cutover
Swap `static Subsystems::NezhaHardware hardware(...)` → `static
Subsystems::DeviceBusHardware hardware(bus/uBit.i2c, ...)`. NOTHING else in
main.cpp's loop body or the motion stack changes — `Drivetrain drivetrain(
hardware)`, `MainLoop loop(hardware, drivetrain, poseEstimator)`, and the loop
body stay identical. NezhaHardware/Hal::NezhaMotor/Hal::OtosOdometer are LEFT
ON DISK (parked, not deleted) — a later cleanup ticket removes them.

## Isolation note
`source/subsystems/device_bus_hardware.{h,cpp}` (and the two leaves) are the
BRIDGE — they legitimately include BOTH `devices/*.h` (the DeviceBus side) and
`msg::`/`hal/`/`subsystems/` (the motion-stack side). They live under
`source/subsystems/`, NOT `source/devices/`, so `test_devices_isolation.py`
(which guards source/devices/) is not violated — source/devices/ stays pure.

## async fiber vs the synchronous tick() contract
The DeviceBus fiber runs continuously on its own CODAL fiber. The main loop
reads the latest handle snapshot each 20ms pass (as fresh as the fiber's last
~16ms cycle) and stages setpoints (the fiber picks them up next cycle). The
old "flush-staged-then-collect, one-pass latency" model loosens — document it;
behavior is fine (fiber is faster than the loop). The measurement rings make
the handle reads snapshot-safe across the fiber boundary.

## Verify
- [x] Host: build (`just build`) — the real firmware now links DeviceBusHardware;
  sim lib unaffected (sim still uses SimHardware). Host-test the conversion
  helpers (Devices::MotorReading→msg::MotorState, PoseReading→PoseEstimate,
  MotorCommand→handle) with a small harness.
  - DONE. `source/subsystems/device_bus_hardware.{h,cpp}` implements
    `Subsystems::DeviceBusHardware`/`DeviceBusMotor`/`DeviceBusOdometer`;
    `source/main.cpp` swapped `Subsystems::NezhaHardware` →
    `Subsystems::DeviceBusHardware` (constructed directly against `uBit.i2c`).
    Real ARM firmware build: FLASH 340540 B / 364 KB = **91.36%**, RAM
    120768 B / 122816 B = **98.33%** (normal per project convention — see
    `.clasi/knowledge/codal-ram-always-near-full.md`). Host sim lib
    (`libfirmware_host`) built unaffected — it does not reference the new
    bridge file at all (its CMakeLists.txt uses an explicit source list, not
    a glob). `tests/sim/unit/device_bus_hardware_harness.cpp` +
    `test_device_bus_hardware.py` host-test the conversion helpers
    (`deviceBusMotorConfigToMsg`/`msgToDeviceBusMotorConfig`/
    `otosBootConfigToDeviceBus`/`deviceBusPoseToEstimate`/
    `msgNeutralToDeviceBus`) AND a real, host-constructed `DeviceBusHardware`
    (capabilities/apply()-gating/active()-toggling/motorConfig()/odometer()
    wiring) — 12 scenarios, all passing.
- [x] Full `uv run python -m pytest` stays green (sim path unchanged).
  - DONE. **1458 passed, 3 skipped, 4 xfailed, 1 xpassed** (was 1457/3/4/1
    before this ticket's one added test file) — zero failures, zero
    regressions. `test_devices_isolation.py` re-verified passing (source/
    devices/ gained no messages::/hal:: include from this ticket).
- [ ] HARDWARE (team-lead runs): flash the real firmware, confirm the standing
  bench gate — sensors alive (encoders, OTOS, color, line), wheels drive both
  directions + encoders increment, a MOVE/segment motion command executes,
  pose updates. This is the real cutover validation.
  - NOT YET RUN (out of this agent's scope per the dispatch — "HITL deferred
    to team-lead"). See the programmer's final report for the exact HITL
    steps and the known limitations (wedged()/wedgeSuspect()/
    hardResetCount()/acceleration() telemetry gap; live `DEV M <n> CFG`/OI/
    OR/OL/OA no-ops; color/line sensors out of this ticket's bridge scope)
    the team-lead should be aware of while validating on the stand.
