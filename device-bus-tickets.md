# DeviceBus — sequenced ticket breakdown

Source of truth: `clasi/issues/device-bus-fiber-owned-self-contained-device-subsystem.md`.
This file sequences that design into implementable tickets. It does NOT
redesign it. Scope is the GREENFIELD subsystem only: build + host-test
`source/devices/` in isolation, with a dedicated bring-up main as the only
consumer. NO cutover, NO edits to existing `source/`, NO `main.cpp` change.

Directory / namespace (from the issue's "Shape"): **`source/devices/`,
`namespace Devices`.**

## Standing isolation invariant (enforced from DB-001 onward)

`source/devices/` may `#include` ONLY: its own headers, the C/C++ standard
library (libc/libm — `<cstdint>`, `<math.h>`, `<cstdio>`, `<cstring>`, …),
and CODAL/micro:bit (`MicroBit.h` and friends). NO include may reach
`messages/`, `hal/`, `com/`, `subsystems/`, `config/`, or any other project
path. This mirrors sprint 100's `source/drive/` discipline and is why the
subsystem carries its OWN value/config types (see DB-001, the load-bearing
consequence). A pure-Python grep test (`test_devices_isolation.py`, landed in
DB-001) enforces it against every subsequent ticket.

## Coding-standard notes applied to all ported code

- Ported code is re-cased to the project rule when it lands in `source/devices/`:
  lowerCamelCase functions/methods, UpperCamelCase types/namespaces,
  snake_case filenames, `member_` trailing underscore (the old `_member`
  leading-underscore style in `i2c_bus.*` is converted). Acronym types stay
  all-caps: `I2CBus`.
- No units in identifiers; unit lives in a leading `// [unit]` comment tag.
- Wire-key strings and vendor register names are exempt (unchanged).

## Resolved open questions (needed to sequence; see "Open questions" in the issue)

- **OQ1 leftover (Neutral vocabulary):** the isolation invariant forbids
  `#include "messages/common.h"`, so `msg::Neutral` cannot be used. Resolved:
  a **`Devices`-local `Neutral` enum** (`Coast`/`Brake`). Same reasoning
  forces `Devices`-local equivalents of every other currently-`msg::*` type
  the ported code touches (see DB-001).
- **OQ2 leftover (PID flag scope):** resolved **per-motor**, matching the
  handle-level `Motor::setPidEnabled(bool)` in the public surface.
- **OQ3 leftover (stamp width):** resolved **`uint64_t` [us]** stored in
  `Sample` so `bracket()`/lerp are wrap-free (RAM cost trivial per the issue).
  `updatedAt()` returns `uint64_t` — a deliberate deviation from the sketch's
  `uint32_t`, which the issue itself flags.
- **Sim/host-test strategy (issue "Sim / host-test story" leaves it open):**
  for this greenfield sprint, host tests use the **scripted-`I2CBus`-fake +
  `Clock`/`Sleeper` seam** (real leaves, faked bus/clock), NOT a
  `PhysicsWorld`-backed `SimDeviceBus`. A physics-backed sim front is a
  consumer-side concern deferred to cutover.
- **Cycle testability (mechanism the issue's cycle sketch implies):** the
  `for(;;)` fiber body is factored into a plain `runCycleOnce()` method the
  host harness steps deterministically; the real fiber is just
  `while(!stopRequested_) runCycleOnce();`. Not a redesign — it is the seam
  the "cycle body parameterized on a sleeper/clock interface" line requires.

---

## DB-001 — Scaffold `source/devices/`, Devices value/config types, isolation test
- **Depends-on:** none
- **Host/HITL:** Host
- **Description:** Create the greenfield directory + `namespace Devices` and
  the self-contained value/config types that sever the `msg::*` dependency the
  isolation invariant forbids (issue "Shape", "The public surface"). Types:
  `MotorReading{position,velocity,appliedDuty}`, `ColorReading{r,g,b,c}`,
  `LineReading{raw[4],normalized[4]}`, `PoseReading{x,y,heading,v_x,v_y,omega}`,
  `Neutral{Coast,Brake}`, `Gains`, `MotorConfig`, `OtosConfig`,
  `ColorConfig`, `LineConfig`. All plain, trivially-copyable (the concurrency
  contract's "plain struct stores/copies" requirement — issue "Concurrency
  contract" rule 2). Establish the isolation grep test that guards every later
  ticket.
- **Acceptance criteria:**
  - `devices_types_harness.cpp` compiles under `-DHOST_BUILD` and
    `static_assert`s every reading/config type is `trivially_copyable` and
    `standard_layout`; run by `test_devices_types.py` (exit 0).
  - `test_devices_isolation.py` greps every `source/devices/*.{h,cpp}`
    `#include` and fails if any path contains `/` and does not start with
    `devices/`, excluding a whitelist (`MicroBit`, CODAL, `<...>` stdlib).
    Passes on the DB-001 tree.
- **Files:** create `source/devices/device_types.h`,
  `source/devices/device_config.h`; `tests/sim/unit/devices_types_harness.cpp`,
  `tests/sim/unit/test_devices_types.py`, `tests/sim/unit/test_devices_isolation.py`.

## DB-002 — `MeasurementRing<T>` + `Sample<T>` + interpolation
- **Depends-on:** DB-001
- **Host/HITL:** Host
- **Description:** The 6-slot gap-write ring (issue "Measurement rings"):
  6 physical slots, 5 published, write into the gap then advance head with a
  single aligned store; tail implicit (head−4); published slots never mutated.
  `publish()/latest()/sample(age)/bracket(t,older,newer)`. Plus the
  interpolation helpers each reading type uses for `sampleAt()`: linear lerp
  AND the **wrap-aware angular lerp** for heading the issue explicitly flags
  as a trap. `Sample.stamp` is `uint64_t` [us] (OQ3). Pure host-clean C++ —
  the issue calls this "plain host-testable."
- **Acceptance criteria:** `measurement_ring_harness.cpp` (run by
  `test_measurement_ring.py`, exit 0) proves: fill/wrap past 6 writes keeps 5
  correct published samples in order; a reader copy taken before a `publish()`
  is unchanged after it (immutability); `bracket()` returns the correct
  straddling pair and false outside the window; linear lerp midpoint; angular
  lerp across the ±180° seam interpolates the short way (e.g. 170°→−170°
  midpoint ≈ 180°, not 0°).
- **Files:** create `source/devices/measurement_ring.h`,
  `source/devices/interpolation.h`; `tests/sim/unit/measurement_ring_harness.cpp`,
  `tests/sim/unit/test_measurement_ring.py`.

## DB-003 — Port I2C bus (scripted fake) + `Clock`/`Sleeper` time seam
- **Depends-on:** DB-001
- **Host/HITL:** Host
- **Description:** Move the bus plane in (issue "Shape": `com/i2c_bus.*`).
  Port `i2c_bus.{h,cpp}` + `i2c_bus_host.cpp` into `Devices::I2CBus`
  (re-cased members), keeping IRQ guard default-ON (TWIM errata,
  non-negotiable — issue "Rejected alternative" & "Armor stays intact"),
  the lazy preClear/postClear clearance timers, and the HOST_BUILD scripted
  FIFO + steppable microsecond clock. Add the `Clock` (`nowMicros()` [us]) and
  `Sleeper` (`sleepMillis`/`yield`) interfaces the cycle is parameterized on
  (issue "Sim / host-test story"): real impls wrap
  `system_timer_current_time_us()` + `fiber_sleep`; host impls are the
  steppable fakes the harness advances. This is the ONLY bus toucher in the
  subsystem (issue "Sole-ownership rule").
- **Acceptance criteria:** `devices_i2c_bus_harness.cpp` (run by
  `test_devices_i2c_bus.py`, exit 0), adapted from
  `i2c_bus_clearance_harness.cpp`, proves scripted write/read FIFO ordering,
  per-device txn/err counters, `clear()` peek vs the entry-spin against the
  fake clock, and IRQ-guard default-on. `devices_clock_harness.cpp` (run by
  `test_devices_clock.py`) proves the host `Clock` advances only when stepped
  and the host `Sleeper` records requested sleeps without wall-clock blocking.
- **Files:** create `source/devices/i2c_bus.{h,cpp}`,
  `source/devices/i2c_bus_host.cpp`, `source/devices/clock.h`,
  `source/devices/clock_real.cpp`, `source/devices/clock_host.cpp`;
  `tests/sim/unit/devices_i2c_bus_harness.cpp`, `tests/sim/unit/test_devices_i2c_bus.py`,
  `tests/sim/unit/devices_clock_harness.cpp`, `tests/sim/unit/test_devices_clock.py`.

## DB-004 — Port motor leaf + armor policy + velocity PID
- **Depends-on:** DB-001, DB-003
- **Host/HITL:** Host
- **Description:** Port `hal/velocity_pid.*`, the `hal/capability/motor.h`
  armor/write-gate machinery, and the `hal/nezha/*` leaf into `Devices`
  (issue "Shape"; "Armor stays intact"). The armor (reversal dwell, output
  deadband, standstill-guarded reset, wedge detector) is ported VERBATIM in
  behavior; the split-phase `requestSample()`/`collectEncoder()` primitives
  and the embedded per-motor PID come along. PID gets the per-motor on/off
  flag (OQ2): PID-off routes staged raw duty straight through the ARMORED
  write path (issue "The public surface" — `setPidEnabled`/`setDuty`; armor
  applies in both modes). The internal leaf is `Devices::NezhaMotor`; it
  consumes `Devices::MotorConfig`/`Gains` (DB-001), not `msg::*`. NOT ported:
  the flip-flop cross-pass sequencer (retired — the fiber owns pairing).
- **Acceptance criteria:** `devices_motor_harness.cpp` (run by
  `test_devices_motor.py`, exit 0), modeled on `motor_policy_harness.cpp` +
  `velocity_pid_harness.cpp` against the scripted `Devices::I2CBus`, proves:
  request→collect encoder pairing produces expected `position()/velocity()`;
  reversal dwell writes 0 then holds through the deadline; sub-deadband duty
  is immediate/unclamped; standstill-guarded reset gates on rest ticks; wedge
  latch + wedge-suspect derive as before; PID-on chases a velocity target;
  PID-off feeds raw duty through the armor unchanged.
- **Files:** create `source/devices/velocity_pid.{h,cpp}`,
  `source/devices/motor_armor.h`, `source/devices/nezha_motor.{h,cpp}`;
  `tests/sim/unit/devices_motor_harness.cpp`, `tests/sim/unit/test_devices_motor.py`.

## DB-005 — Port OTOS driver
- **Depends-on:** DB-001, DB-003
- **Host/HITL:** Host
- **Description:** Port `hal/otos/*` into `Devices::Otos` (issue "Shape").
  Carries the fire-and-forget IMU-calibration kickoff (no block-poll), the
  combined 12-byte position+velocity burst, the per-transaction clearance, the
  same-instant-heading lever-arm compensation, and the wrap-aware heading
  handling. Emits `Devices::PoseReading`/`OtosConfig` (DB-001), not `msg::*`.
  The staged `setPose()` re-anchor request (issue "The public surface":
  `Odometer` staged re-anchor replacing `MainLoop::applySetPose()`) is defined
  here as a plain staged cell the fiber drains (wired in DB-007).
- **Acceptance criteria:** `devices_otos_harness.cpp` (run by
  `test_devices_otos.py`, exit 0), modeled on `otos_odometer_harness.cpp`
  against the scripted bus, proves: PRODUCT_ID detect gates all traffic;
  `readDue()` rate-limits real reads; a burst decodes to expected pose;
  lever-arm compensation cancels on a scripted pure spin (no phantom
  translation); a burst failure holds prior pose and marks the sample stale.
- **Files:** create `source/devices/otos.{h,cpp}`;
  `tests/sim/unit/devices_otos_harness.cpp`, `tests/sim/unit/test_devices_otos.py`.

## DB-006 — Port color + line sensor drivers (from `source_old`)
- **Depends-on:** DB-001, DB-003
- **Host/HITL:** Host
- **Description:** Port the PlanetX/APDS color driver and the line driver from
  `source_old/hal/real/{ColorSensor,LineSensor}.*` into
  `Devices::{ColorSensor,LineSensor}` internal leaves (issue "Shape";
  "Line and color sensing don't exist in the new tree yet"). Preserve the
  **re-wake-each-retry** color detection sequence (0x81=0xCA / 0x80=0x17 per
  retry, 0x43 primary / 0x39 APDS fallback) called out in the issue and
  `docs/knowledge/encoders-read-zero-i2c-bus-hang.md`. Non-blocking reads
  only (no 250 ms poll loops — the fiber schedules cadence). Emits
  `Devices::ColorReading`/`LineReading` (DB-001).
- **Acceptance criteria:** `devices_sensors_harness.cpp` (run by
  `test_devices_sensors.py`, exit 0) against the scripted bus proves: color
  detection succeeds via the re-wake retry path and falls back to APDS; a
  scripted color frame decodes to expected r/g/b/c; line raw→normalized
  produces expected 4-channel values; absent-device detection marks the leaf
  not-connected without hanging.
- **Files:** create `source/devices/color_sensor.{h,cpp}`,
  `source/devices/line_sensor.{h,cpp}`; `tests/sim/unit/devices_sensors_harness.cpp`,
  `tests/sim/unit/test_devices_sensors.py`.

## DB-007 — `DeviceBus` root + handle classes + straight-line cycle + rings
- **Depends-on:** DB-002, DB-003, DB-004, DB-005, DB-006
- **Host/HITL:** Host
- **Description:** The subsystem's core (issue "The public surface", "The
  fiber and its cycle", "Concurrency contract"). Build the root `DeviceBus`
  owning the bus + all leaves + one `MeasurementRing` per stream, and the
  handle classes `Motor`/`ColorSensor`/`LineSensor`/`Odometer` (private ctors,
  `friend class DeviceBus`, non-copyable, returned by reference). Handle
  getters serve latest published sample and never touch the bus
  (`latest()/sample(age)/sampleAt(t)/updatedAt()/connected()`); setters stage
  a request cell the fiber drains at cycle top. Implement `runCycleOnce()` as
  the straight-line schedule: `drainStagedInputs()` → `requestEncoder` →
  settle-sleep (via `Sleeper`) → `collect` + PID + armored duty write →
  round-robin `perceptionSlot` (line|color|OTOS) → `publishSamples(now)` →
  pace-sleep. Wire the stale-target/RX-watchdog neutralize gate (issue: fiber
  writes neutral itself if targets go stale). Enforce the concurrency contract
  (single-writer rings; no yield inside publish/stage/sample-copy).
- **Acceptance criteria:** `device_bus_cycle_harness.cpp` (run by
  `test_device_bus_cycle.py`, exit 0) steps `runCycleOnce()` with the host
  `Clock`/`Sleeper` + scripted bus and proves: bus transactions occur in the
  documented schedule order with the settle sleep between request and collect;
  no duty write is ever emitted between a pending encoder request and its
  collect (the 093 hazard is structurally absent — a scripted mid-pair duty
  attempt cannot be injected); a staged `setVelocity()` reaches an armored
  duty write within one cycle; `publishSamples()` populates each ring with a
  monotonic [us] stamp; `sampleAt(otosStamp)` on the motor handle returns a
  bracket-interpolated reading; PID-off `setDuty()` drives the armored write;
  stale staged targets cause the cycle to write neutral.
- **Files:** create `source/devices/device_bus.{h,cpp}`,
  `source/devices/handles.h`; `tests/sim/unit/device_bus_cycle_harness.cpp`,
  `tests/sim/unit/test_device_bus_cycle.py`.

## DB-008 — Fiber lifecycle: `start()`/`stop()`/`running()`, preamble, epilogue
- **Depends-on:** DB-007
- **Host/HITL:** Host (lifecycle state machine) — real `create_fiber` proven in DB-009 (HITL)
- **Description:** Wrap `runCycleOnce()` in the CODAL fiber lifecycle (issue
  "The fiber and its cycle"): `start()` spawns one fiber (`create_fiber`)
  that runs the **detection preamble** (power-settle wait, per-device
  `begin()` with retries, absent devices marked/skipped —
  `present()`-not-`connected()` lesson) then `while(!stopRequested_)
  runCycleOnce()`; `stop()` requests exit, joins, and neutralizes all motors
  before exit (wheels never left driven); `running()` reports state. A
  `FiberRunner` seam lets host tests inject a synchronous runner that calls
  `runCycleOnce()` N times in place of `create_fiber`, so the lifecycle
  state machine and neutralize-on-exit ordering are host-verified; the real
  `create_fiber` path is exercised on hardware in DB-009.
- **Acceptance criteria:** `device_bus_lifecycle_harness.cpp` (run by
  `test_device_bus_lifecycle.py`, exit 0) proves via the injected runner:
  preamble marks a scripted-absent device not-present and skips its slot;
  `running()` is false before `start()` / true after / false after `stop()`;
  `stop()` emits a neutral duty write for every motor as its last bus action;
  a preamble device-begin retry that eventually succeeds does not block the
  loop from starting.
- **Files:** modify `source/devices/device_bus.{h,cpp}`; create
  `source/devices/fiber_runner.h`; `tests/sim/unit/device_bus_lifecycle_harness.cpp`,
  `tests/sim/unit/test_device_bus_lifecycle.py`.

## DB-009 — HITL bring-up firmware main + DEV driver + bench gates
- **Depends-on:** DB-007, DB-008
- **Host/HITL:** HITL
- **Description:** The dedicated bring-up firmware where `DeviceBus` is the
  ONLY thing running, driven directly by DEV commands (issue OQ5 resolution:
  greenfield, own dedicated main, no coexistence with the legacy stack). Add a
  `bringup_main.cpp` in `source/devices/` (its own `create_fiber` DeviceBus +
  a minimal serial DEV command parser to stage targets / toggle PID / read
  handles / dump rings) and a dedicated CODAL build config that selects it as
  the image entry point — the single sanctioned build-config addition; no
  existing `source/` file and no `main.cpp` is modified. Run the issue's
  "Bench gates":
  1. dual-per-motorId encoder-request pipelining probe (gates pipelined vs
     alternating-port cycle);
  2. `fiber_sleep(4)` actual-latency distribution measurement;
  3. reversal-stress armor re-verification under the new cadence;
  4. serial/radio health (binary-vs-text same-boot discriminator);
  5. flash/RAM delta check (flash overflow is the real limit).
  (098/099 motion non-regression gates are cutover-time, out of scope here.)
- **Acceptance criteria:** `tests/bench/device_bus_bringup.py` drives the
  bring-up image on the bench and records: a wheel actuates within ~one cycle
  of a staged `setVelocity` (latency win vs the ~80–160 ms flip-flop); handles
  report fresh timestamped readings for each connected device; reversal-stress
  produces no runaway/wedge escape; serial/radio health no worse than the
  per-transaction IRQ-guard baseline; measured `fiber_sleep(4)` distribution
  and flash/RAM deltas are logged. Results captured per
  `.claude/rules/hardware-bench-testing.md`.
- **Files:** create `source/devices/bringup_main.cpp`, `codal.devicebus.json`
  (dedicated bring-up build config); `tests/bench/device_bus_bringup.py`.

---

## Dependency graph

```
DB-001 ─┬─ DB-002 ─────────────────────────┐
        ├─ DB-003 ─┬─ DB-004 ─┐            │
        │          ├─ DB-005 ─┤            │
        │          └─ DB-006 ─┤            │
        └───────────────────  ├─► DB-007 ─► DB-008 ─► DB-009 (HITL)
                              (DB-002,003,004,005,006)
```

DB-004 / DB-005 / DB-006 are parallelizable once DB-003 lands.
DB-001…DB-008 are all host-testable; DB-009 is the only HITL/bench ticket and is last.
