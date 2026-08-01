---
status: pending
---

# Unify sim and robot composition roots to eliminate all sim/hardware firmware differences

## Goal (stakeholder directive, 2026-07-31)

The firmware that runs in the simulator must be virtually identical to the
firmware that runs on the hardware. The target end state: **the only
difference between the two builds is that one composition root constructs a
real I2C bus (`Devices::MicroBitI2CBus`) and the other constructs a simulated
one (`TestSim::SimPlant`)**, with sensors/encoders faked at the I2C wire
level — encoders driven by the duty cycle the firmware actually wrote, OTOS
answered through the real register-pointer protocol.

## Where we already are

The device layer already has this shape (sprint 108): the sim runs the REAL
`NezhaMotor`, `MotorArmor`, `RealOtos`, `ColorSensorLeaf`, and
`LineSensorLeaf` drivers against `SimPlant` (`src/sim/sim_plant.h`), which
implements the pure `Devices::I2CBus` interface and answers at the wire-byte
level (Nezha 0x60/0x46 frames, OTOS register-pointer protocol, NAK on unknown
addresses). Encoders are integrated from the wire-parsed duty in `WheelPlant`.
There is no back-channel. Both roots link the same App::/Motion:: TUs, and
`RobotLoop` takes only pure interfaces (`I2CBus&`, `Clock&`, `Sleeper&`,
`Transport&`).

The gap is the **composition root and configuration path**, not the device
fakes. `app/boot_wiring.{h,cpp}` (2026-07-31, currently uncommitted) started
the fix: main.cpp now boots through shared `App::effectiveTrackWidth()` /
`installDriveCalibration()` / `bootPlannerLimits()` / `installShaperLimits()`
/ `installRotationCalibration()`. The sim side has not been converted.

## Assessed differences to eliminate (ranked)

1. **`TestSim::SimHarness` still hand-wires its own boot literals**
   (`simPlannerLimits()`, `src/sim/sim_harness.h`), materially different from
   `bootPlannerLimits()`: shaping effectively off (aMax 1e6 vs 300), control
   period 40 ms vs measured 47, headingHoldGain 0 vs 2, dutyFloor 0.03 vs
   0.18, different velocity gains, and — worst — wheel trim gains left at
   their fail-closed all-zero default, i.e. the closed loop that actually
   reaches the wheels was OFF in every sim session while ON in every robot
   session. The sim also never installs rotation calibration.
2. **Configuration source**: hardware bakes the robot JSON via
   `config/boot_config.cpp`; the sim build deliberately excludes it and boots
   unconfigured (all-zero motor configs, default estimator weights, drive
   calibrated to the synthetic plant's own gain), relying on a later Python
   config push. `boot_config.cpp` has zero CODAL includes — nothing prevents
   linking it host-side.
3. **Timing model**: the sim steps virtual time at `RobotLoop::kCycle`
   = 40 ms while `bootPlannerLimits()` tells the planner the measured 47 ms
   delivered period. Once the sim adopts the shared limits, the step period
   and the planner's `controlPeriod` must derive from ONE constant or the
   planner's discrete math is wrong in sim.
4. **Boot sequence shape**: `SimHarness::boot()` hand-steps the preamble and
   gates `markConfigured()` on `configureMotor()` calls; main.cpp configures
   everything then marks immediately. Sim tests exercise a boot path main()
   never takes.
5. **(Noted, confined, acceptable)** `FAKE_OTOS` is a hardware-side app-level
   fake selected by one `#ifdef` at the main.cpp root for benches without the
   OTOS mounted — the inverse of the I2C-level principle, but cleanly
   confined to the composition root.

## Work items

1. **Finish boot_wiring adoption in the sim** — point `SimHarness` at
   `bootPlannerLimits()` / `installShaperLimits()` /
   `installRotationCalibration()` / `installDriveCalibration()`. Any value
   that genuinely must differ in sim is overridden AT the sim call site, next
   to a comment saying why — a visible deliberate exception, never silence.
   Build note: the ARM build globs `src/firm` recursively so
   `boot_wiring.cpp` is already picked up there, but it must be added to
   `src/sim/CMakeLists.txt` AND the ~10 pytest `_APP_SOURCES` lists under
   `src/tests/sim/` (the known four-source-lists trap) or host builds break
   at link.
2. **Link `config/boot_config.cpp` into the host lib** so both roots boot
   from the same robot-JSON bake. Sim scenarios needing a synthetic plant
   config still override via the existing config-push path.
3. **Extract the rest of main.cpp's graph construction into one shared
   `composeRobot(bus, clock, sleeper, serial, radio, tuningStore*)`** (in
   `app/`), leaving main.cpp and SimHarness as ~20-line shells parameterized
   only by the leaf implementations. At that point "one constructs
   `MicroBitI2CBus`, the other constructs `SimPlant`" is literally true in
   the code and drift becomes structurally impossible.
4. **Unify the control period**: derive the sim's step dt and the planner's
   `controlPeriod`/`actuationDelay` from the same value (the measured 47 ms,
   or model the ~7 ms vendor bus-clearance overhead explicitly).
5. **Triage the sim-behavior test fallout**: adopting real shaping, trim,
   heading hold, and dutyFloor changes sim dynamics; scenario tests that
   asserted instant-commit/unshaped semantics will need either the documented
   per-test override (via `planner().applyShaperLimits()` etc.) or updated
   expectations. (12 failures already observed from the first unification
   step — being triaged under the active OOP session.)

## Separate axis (out of scope here, note for a follow-on issue)

Code parity vs **plant fidelity**: even with identical composition,
`WheelPlant` is a synthetic first-order plant. The real gearbox measured
gain ~1370 mm/s-per-duty, tau ~230 ms, breakaway ~0.18 duty. Loading the
plant model's parameters from the same robot JSON's measured constants would
make sim predictions match the bench — a fidelity improvement, distinct from
the parity work above.
