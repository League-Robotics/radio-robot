---
root: ../../../docs/design/design.md
---

# Hardware

**Owner:** Eric Busboom · **Last reviewed:** 2026-08-09 · **Status:** in-flux

---

## 1. Purpose

`hardware/` owns every concrete device driver on the robot — one class per
physical part, each holding that part's register map, timing quirks, and
hardware workarounds and **nothing else**. It sits between `platform/`
(bus and clock primitives, below) and `hal/` (the interfaces these drivers
implement, above): a driver here reaches DOWN to a `Platform::I2CBus&` and
UP only as far as the `Hal::` interface it satisfies. Nothing here depends
on the wire schema, on generated boot configuration, or on the loop that
drives it.

The seam is the hardware boundary itself, so the rest of the firmware —
and the whole host-side test and simulation story — drives the same driver
code against either real silicon or `Platform::host`'s `SimPlant`.

## 2. Orientation — the three-way filing test

A driver's directory says **who else could reuse it**, not what it does.
`Platform::I2CBus&`-only is *not* sufficient to call something generic: a
bespoke board speaking its own wire protocol over a standard bus is still
nobody else's device.

- **`generic/`** — genuinely standard, publicly documented parts, reusable
  by anyone on any I2C/SPI/UART-capable platform without project-specific
  knowledge, plus the driver-agnostic policy that composes over them.
  - `real_otos.*` — the SparkFun OTOS, a commercial documented part.
  - `motor_armor.h` — `MotorArmor`, a decorator over *any* `Hal::Motor`
    (reversal dwell, deadband, standstill-guarded resets, wedge
    detection). Generic by the same test: it names no chip.
  - `board_motor.*` — `BoardMotor`, one channel of any `Hal::MotorBoard`
    presented as a `Hal::Motor`. Board-agnostic by construction.
- **Named hardware families** — a specific peripheral-board family that is
  not itself a compute target. One directory per family:
  - `nezha/` — `NezhaMotor`, this project's own motor-controller board.
  - `hiwonder/` — `HiwonderBoard`, the HiWonder 4-channel encoder driver
    behind `Hal::MotorBoard`. Not wired in yet.
  - `planetx/` — `ColorSensorLeaf`, `LineSensorLeaf` (the PlanetX
    ecosystem parts at 0x43/0x1A). `ColorSensorLeaf` also detects an
    APDS9960 at 0x39 as a fallback, which *is* a generic part — splitting
    that path out into `generic/` would be a behavior change, not a move,
    so it stays here with the chip it shares a class with.
- **Compute-platform-intrinsic hardware** — hardware physically part of
  one compute board and unable to exist anywhere else (the micro:bit's
  onboard compass). That lives at `platform/<target>/hardware/`, **not
  here**. Nothing occupies that slot today.

Named-family directories are siblings under `hardware/`, deliberately not
`platform/<family>/hardware/`. The reorganization proposal left that open;
`hardware/` wins because "platform" then stays strictly compute-target-only
(micro:bit, host, a future Pi). The alternative reading forces
`platform/hiwonder/` and `platform/planetx/` too, and a colour sensor is
not a compute target under any reading.

All three categories implement the same `Hal::` interfaces where one
applies — the split is about where a driver lives and who can reuse it,
never about whether it participates in HAL.

Every leaf follows the same non-blocking shape: construction takes an
`I2CBus&` and a config struct; a `begin()`/`beginStep(nowUs)` detects the
chip (some leaves — color, line — need several paced retries, so their
detection is itself a single-step, call-every-cycle state machine, never a
blocking retry loop); `present()` latches permanently once detection
succeeds; `connected()` is the live, per-tick bus-health result; `tick
(nowUs)` does the one real, rate-limited or split-phase, unit of bus work
per call. No leaf sleeps, blocks, or retries internally — the caller (the
loop, `app/robot_loop.cpp`) supplies "now" and decides the cadence.

For the full system-level flow (comms in → dispatch → motor service →
state out → pace) and the schedule these leaves are ticked from, see root
`DESIGN.md` §2 and its `runAndWait` timing primitive (§4).

## 3. Constraints and Invariants

- **Hardware isolation invariant:** nothing under `hardware/` may
  `#include "messages/..."` or `#include "config/..."`. Every value a
  driver accepts or publishes is a plain HAL-local aggregate
  (`hal/device_config.h`/`hal/device_types.h`), never a `msg::*` or
  `Config::*` type. Breaking this couples a driver to the wire schema or
  to generated boot config and kills its reuse under `-DHOST_BUILD`/sim —
  the composition root (`app/boot_wiring.*`, and `main.cpp` above it) is
  the one place both a wire type and its HAL-local counterpart are
  reachable, and conversion happens only there.
- **Dependencies point one way.** A driver may reach DOWN to `platform/`
  and name the `hal/` interface it implements. It may not reach UP into
  `kinematics/`, `motion/`, or `app/`, and it may not include a driver
  from another family directory. If two drivers need to share something,
  that something belongs in `generic/` or in `hal/`.
- **No `#ifdef HOST_BUILD` forks inside a shared header:** the platform
  seams (`Platform::Clock`, `Platform::Sleeper`, `Platform::I2CBus`) are
  plain virtual bases with zero preprocessor conditionals. The real ARM
  implementation lives in its own `platform/microbit/microbit_*` file
  (which includes `MicroBit.h` and is therefore ARM-only); the host
  implementation lives in `platform/host/`. Reintroducing a same-header
  fork was the pre-108 shape this subsystem replaced — don't regress to it.
- **No leaf sleeps or blocks.** Every `tick()`/`beginStep()` takes its
  "now" as a `uint64_t` [us] parameter and returns having done at most one
  bounded unit of bus work. A leaf that calls `fiber_sleep()` or spins on
  a chip's own ready bit reintroduces exactly the loop-timing collapse the
  single-loop rebuild (`docs/design/design.md` §5) exists to prevent — the
  color/line sensors' non-blocking `beginStep()` retry state machines are
  the direct fix for detection loops that used to do this.
- **`present()` and `connected()` are different questions — never
  conflate them.** `present()` latches once, permanently, the first time
  detection succeeds; `connected()` is re-evaluated every `tick()` and can
  go false and true again. A caller deciding whether to give a leaf a bus
  slot at all must gate on `present()`; gating on `connected()` instead
  lets one transient I2C glitch permanently stop scheduling an otherwise
  healthy chip (`otos.h`, `color_sensor.h`, `line_sensor.h` all repeat
  this warning at their own declarations — it is a real, repeated trap,
  not boilerplate).
- **Per-transaction bus clearance is mandatory on shared-bus writes.**
  Every `I2CBus::write()`/`read()` call that touches a device sharing the
  bus with tight timing (the Nezha motor's 0x46 encoder register, OTOS's
  register reads) must carry a nonzero `preClear`/`postClear` gap. Omitting
  it reproduces the nRF52 TWIM `NRF52I2C::waitForStop()` errata stall this
  clearance discipline exists to prevent (a real, hardware-confirmed
  multi-second bus hang, not a theoretical risk).
- **`MotorArmor`'s leaf contract is fixed call order.** A derived leaf
  supplies `writeRawDuty()`/`hardReset()`/`softRebaseline()` (the *how*)
  and must call the four protected armor steps
  (`processResetIfPending()` → sample/collect its own encoder →
  `updateWedgeDetector()` → mode dispatch through `armoredWrite()` →
  `updateRestTracking()`) in exactly that order from its own `tick()`.
  Reordering breaks the documented data dependencies: wedge detection
  reads this tick's fresh `position()`/last tick's `appliedDuty()`; rest
  tracking reads whatever `armoredWrite()` just decided this same tick.
  `NezhaMotor::tick()` is the reference implementation of the order.
- **A reversal always passes through zero-dwell, never a direct sign
  flip.** `MotorArmor::armoredWrite()` forces a commanded sign change to
  write 0, hold for `reversalDwell_`, then release the new direction — the
  one exception is `reversalDwell_ == 0` (explicit legacy configuration),
  which skips the dwell transition deliberately. Bypassing this in a leaf
  reintroduces the encoder-wedge/latch failure mode
  (`.clasi/knowledge/`, `docs/knowledge/2026-07-04-encoder-wedge.md`).
- **The wedge latch has no target-gating or arming-grace.**
  `updateWedgeDetector()`'s raw stuck-encoder counter fires on N
  consecutive identical `position()` reads regardless of what was
  commanded. Do not add a "only count it if we were trying to move"
  qualifier to the raw latch — `wedgeSuspect()` already exists as the
  motion-qualified derivation for exactly that use case, kept as a
  *second*, independent counter rather than a modification of the first.
- **OTOS's `sensorToCentre()` lever-arm transform requires the
  same-instant heading.** The heading passed in must come from the same
  I2C burst as the position it is transforming — never a heading left
  over from a previous tick. A prior regression (commit db11b7c) produced
  ~433mm of phantom translation on a pure spin because the transform used
  a heading lagging the live spin by a constant `omega*dt`; the residual
  is a lever-arm circle proportional to spin rate, invisible at rest and
  severe during a fast turn. Any new call site of this transform must
  honor the same contract.
- **`MeasurementRing<T>::publish()` never mutates a published slot.**
  It writes the new sample into the currently-unpublished write-gap slot,
  then advances `head_` with a single store — that store is what makes the
  sample visible to readers. A reader that has copied a `Sample<T>` out
  holds a value that cannot change underneath it. Do not "optimize" this
  into an in-place update of the newest slot; that reintroduces a
  torn-read race with any reader between fiber yield points.
- **Deliberate non-goal: no `msg::`-typed surface anywhere in this
  directory.** No leaf exposes or accepts `msg::MotorCommand`,
  `msg::MotorState`, `msg::PoseEstimate`, or any other wire type — that
  would be the isolation invariant violated from the other direction. The
  loop (`app/`) is the translation boundary, not a handle class inside
  `devices/`.
- **Deliberate non-goal: no onboard POSITION mode.** The Nezha chip's
  0x5D absolute-angle move is not wired into `NezhaMotor` — the leaf's
  public surface only covers raw-duty mode. Adding it back requires a
  fresh design pass (a staged target, a completion signal, interaction
  with the armor's reset/wedge state), not a quick register-write bolt-on.
- **(Historical, pre-125-003) Deliberate non-goal: no additive velocity
  feedforward term beyond the control law's own feedforward gain.** The
  velocity control law's output used to be exactly the PI (+ feedforward)
  law with no separate feedforward path layered on top of it. 125-003
  relocated the whole control law off this leaf to the motion library
  (`src/motion/wheel_velocity_pid.h`) — this section describes the
  pre-move design; see that header for the current one.

## 4. Design

**Why leaves, not a shared device base class beyond `MotorArmor`.** Only
the four motors share enough policy (reversal dwell, deadband, reset,
wedge) to justify a common base; OTOS, the color sensor, and the line
sensor each have a hardware register map and timing quirk profile
specific enough that a shared base would either grow a pile of unused
virtuals or force a lowest-common-denominator interface. Each leaf instead
follows the same *shape* (constructor, `begin`/`beginStep`, `present`/
`connected`, `tick`) by convention, not by inheritance.

**Split-phase encoder sequencing (`NezhaMotor`).** The Nezha V2 controller
answers a `0x46` (read-angle) request with a value that is not ready
until a settle window has elapsed. (The former "register refreshes only
every ~80ms" claim here was measured FALSE 2026-07-26 — the register is
live at ≤16 ms; historical staleness was interposed-0x10-traffic sample
invalidation under the pre-118 schedule. See
`docs/design/encoder-refresh-characterization.md`.)
`requestSample()`/`requestEncoder()`
(phase 1, a write) and `collectEncoder()` (phase 2, a read) are therefore
split across two loop slices — the loop requests a sample one slice,
collects it a later slice — rather than blocking in place. `tick()`'s
step 2 additionally gates velocity/glitch computation on a *freshness*
check (the collected raw count differs from the last fresh raw count):
running that computation on every tick, including the ~4 out of 5 ticks
that re-collect the same stale raw count, was a hardware-confirmed bug
(permanently starved `filteredVelocity_`, false glitch rejection) fixed by
keying the computation off the raw wire count rather than the derived
position.

**`fwdSign` convention.** `MotorConfig::fwdSign` is `+1` or `-1`, applied
identically to both the write path (`writeRawDuty()`'s `effective =
fwdSign * written`) and the read path (`tick()`'s `pos = ... *
config_.wheelTravelCalib * fwdSign`) so a mirror-mounted wheel's encoder
sign and commanded-duty sign stay consistent with each other without
either the caller or the armor policy needing to know about the physical
mounting.

**Bus clearance and the TWIM stall.** `I2CBus::write()`/`read()` take
`preClear`/`postClear` [us] parameters: lazy per-device deadlines a caller
attaches to a transaction so the *next* transaction to that device (from
any caller) waits out a real settle window before proceeding, without
spinning — the wait, when one is due, happens via a cooperative sleep
(`waitForClearance()`), never a busy-loop. `NezhaMotor`'s duty writes and
encoder requests, and every OTOS register access, attach a clearance
window for the same underlying reason: the nRF52's TWIM peripheral has a
silicon errata that stalls under back-to-back transactions with
insufficient real-world settle time, confirmed on hardware and
documented in `docs/knowledge/2026-07-04-encoder-wedge.md`. The
`MicroBitI2CBus::clearanceSafetyNetCount()` counter is a diagnostic
signal only — a caller entering a transaction before its own clearance
deadline elapsed (the loop schedule was supposed to own that gap) is
counted and cooperatively waited out rather than allowed to proceed
early, but a nonzero count means the loop's own schedule, not this class,
has a timing defect.

**(Removed, 128-013) The full-transaction IRQ guard.** A separate, now-
deleted mechanism used to mask IRQs for the duration of an entire
`write()`/`read()` transaction (not just the `inUse_` flag
check-and-set) as a second line of defense against the same TWIM
errata — `MicroBitI2CBus::setIrqGuard()`/`irqGuard()`. It cost ~7-8%
inbound serial command loss (DMA RX drops bytes inside the masked
window) to reduce the odds of a fault that is now bounded a different
way: the mandatory `MOVE` timeout backstop and explicit wedge detection
(neither existed when the guard was made "non-negotiable"). Guard-off
measured ~0% inbound command loss (stakeholder decision 2026-07-30).
The per-device clearance wait described above and the `inUse_`
re-entrancy flag are unrelated mechanisms and were not touched by this
removal.

**`present()`/`connected()` and detection as a state machine.** Detection
for the color and line sensors is not a fire-once `begin()` the way
`NezhaMotor`'s and `Otos`'s is — the color sensor in particular needs its
wake registers re-asserted on every retry (a chip still powering up on
the first attempt would otherwise never be found), which the pre-rebuild
driver did via a blocking `for` loop with `fiber_sleep(50)` between
attempts. `beginStep(nowUs)` is that same retry logic restructured as a
single non-blocking step, paced by the caller's own clock rather than a
real sleep, called once per loop cycle until `detectDone()` is true. This
is the general pattern for any future leaf whose detection needs more
than one attempt: never a blocking retry loop, always a step function the
loop drives.

**`Otos` is an interface with two implementations — the `FAKE_OTOS` bench
seam (otos-fake-seam refactor; supersedes 120-002).** `Otos` is the one
leaf in this directory that is NOT a single concrete class: it is a pure
abstract interface (the pose/freshness accessors the loop reads via
`applyOtosSample()`, plus the calibration/offset setters a CONFIG patch
pushes). Two classes implement it:

- **`Devices::RealOtos`** — the SparkFun OTOS I2C leaf; all the behavior
  documented above (fire-and-forget IMU calibration, the combined burst
  read, lever-arm compensation, wrap-aware heading, staged `setPose()`)
  lives here. This is the only implementation with any real-chip or bus
  dependency.
- **`App::FakeOtos`** ([`../app/fake_otos.h`](../app/fake_otos.h), in
  `app/` not here) — a bench synthesizer whose `tick()` reports the
  dead-reckoned `App::Odometry` pose + the `BodyKinematics`-fused wheel
  twist as if it were the chip, so a bench tour on a free-spinning stand
  tracks its commanded path through the SAME `applyOtosSample()` code path
  the table build uses. It lives in `app/` precisely because it needs
  `App::Odometry` — a dependency the devices/ isolation invariant (§3)
  forbids this layer from taking. That is exactly why the SEAM is the
  interface (here) and the app-coupled implementation sits above it: the
  invariant is preserved (`RealOtos` and this header stay app-free) while
  the fake gets the context it needs.

Why an interface rather than 120-002's original `feedSyntheticSample()`
method + per-cycle `#ifdef FAKE_OTOS`: the old shape put a test-only
synthetic-sample method on the production chip driver and branched the hot
loop on the build. The interface confines the build choice to a single
construction-time selection (`main.cpp`, `#ifdef FAKE_OTOS`), gives the
loop one uniform `Devices::Otos&` call site, and keeps synthesis logic
inside the fake where it belongs. The abstract base carries an out-of-line
virtual destructor (defined in `otos.cpp`) as its key function — the
anchor that pins the single vtable/typeinfo to one TU; without it the
embedded arm link fails on every virtual call site. Coverage:
`devices_otos_harness.cpp` exercises `RealOtos`; `app_fake_otos_harness.cpp`
(`test_app_fake_otos.py`) exercises `FakeOtos`.

Build selection is the CMake option `FAKE_OTOS` (root `CMakeLists.txt`,
OFF by default), `add_definitions(-DFAKE_OTOS)` for the WHOLE firmware
target when ON — compile-time only, never a runtime/wire toggle. Select it
via `cmake .. -DFAKE_OTOS=ON` or `build.py --fake-otos` (which always
passes an explicit `-DFAKE_OTOS=ON`/`OFF`, so a stale `CMakeCache.txt`
can never leave the flag stuck). In `main.cpp` a `RealOtos` is constructed
and, under `#ifdef FAKE_OTOS`, an `App::FakeOtos` too; a `Devices::Otos&`
is bound to whichever the build selected and handed to `Preamble` and
`RobotLoop` — the ONE place the variant diverges.

**`MotorArmor`'s reset dispatch (hard vs. soft).** A staged
`resetPosition()` request is resolved at the top of the leaf's next
`tick()`: if the motor has been at rest (`|velocity| < kRestVelocity` and
zero commanded duty) for `kRestTicksRequired` consecutive ticks, the leaf
performs a `hardReset()` (an atomic hardware re-prime burst, safe only at
verified standstill); otherwise it performs an immediate `softRebaseline()`
(a software-only offset adjustment, no bus transaction, always safe). The
standstill guard exists because the hardware re-prime sequence is not
safe to run while the wheel is actually moving.

**(Historical, pre-125-003) The velocity control law — a reduced PI with
back-calculation anti-windup.** This subsection describes the control law
that used to be embedded here, before 125-003 relocated it wholesale to a
standalone class in the motion library — itself later deleted outright
(128-015, zero instantiations; see `src/motion/DESIGN.md`'s "wheel control
generations" note) — kept for history, not current architecture.
The control law is a discrete PI (+ feedforward, `Gains::kff`) with
back-calculation anti-windup against `Gains::iMax`, plus one integrator
behavior worth calling out: the integrator is *frozen* (left unchanged)
while `|target| <= velDeadband`, and explicitly *reset to zero* on the
tick the deadband is first entered (edge-triggered, not level-held) —
this clears whatever bias the integrator built up sustaining a prior,
unrelated motion (e.g. a fast turn) before it can leak into a
newly-commanded stop as an oversized, wrong-signed correction. A
continuing low/zero target after that first tick keeps freezing exactly
as before; only the *entry* transition resets.

**Exact-zero target AND near-rest measured velocity bypasses the P-term
too, not just the integrator (2026-07-22 bench fix, refined same day).**
The deadband freeze above only ever silenced the *integral* term —
`compute()` still computed and returned `kp * err` for an in-deadband
target, including the literal `target == 0.0f` case `Drive::stop()`/an
emptied Move queue produce at rest. Since `err = target - measured`, an
exact-zero target's own "error" is just whatever residual/noisy
`measured` velocity the plant happens to report that tick, and
`writeShapedDuty()` (`nezha_motor.cpp`) boosts that noise-driven nonzero
P output up to the full `outputDeadband_` magnitude, in whatever sign the
noise landed on, every tick it flips ("clicking" at rest). Confirmed on
the bench: 20s of idle telemetry showed one wheel's encoder position
drifting ~12mm while its reported velocity alternated sign at roughly the
deadband-boost magnitude the entire time.

The first cut made `compute()` return a hard `0.0f` whenever `target ==
0.0f` alone, before ever computing `err`/`kp * err`. A same-day
stakeholder live report caught this cut's own regression: the P4 `Move`
model is bang-bang (full commanded velocity until the stop condition
fires, then `target` snaps directly to `0.0f` — no deceleration ramp of
its own), so gating on `target == 0.0f` alone ALSO killed the P-term's
active braking the instant a Move ended while the wheel was still
genuinely moving fast — confirmed by two sim regressions (STOP
convergence from ~500mm/s measurably slower to cross a 5mm/s tolerance;
`SUC-050`'s own angle-stop tolerance missed by 0.4%). Fixed by ALSO
requiring `fabsf(measured)` to already be within a rest-noise floor
(`kZeroTargetRestNoiseFloor = 15mm/s`, `velocity_pid.cpp`, matching the
bench's own observed at-rest noise envelope and the pre-existing —
though not yet wired to a live `velDeadband` boot value —
`tovez_nocal.json` `drive.motor_deadband=15.0`; `velDeadband` itself
still wins if a future boot-config fix ever makes it live and larger)
before the exemption fires: real deceleration from speed keeps its full
active braking all the way down to the noise floor, and only the last,
noise-dominated tail below it gets hard-zeroed instead of dithered. Both
gates are narrower than the deadband-freeze branch above (which still
applies to a small-but-*nonzero* target, e.g. sprint 114 ticket 005's own
sub-deadband-boost terminal-trim scenario, `scenario
DeadbandBoostSettlesNotHuntsAcrossResidualSweep`) and only ever silence
the literal "settled at a complete stop" state.

**Measurement rings: 6 physical slots, 5 published.** `MeasurementRing<T>`
keeps one slot permanently as an unpublished write gap so `publish()` can
write a full `Sample<T>` into it and then make it visible with a single
aligned store to `head_` — the two steps are ordered so no reader ever
observes a partially-written sample. `bracket(t, ...)` scans newest to
oldest and stops at the first invalid (never-published) slot, since
`publish()` only ever extends history forward in time. `Sample<T>::stamp`
is a `uint64_t` [us] (not the sketch's originally-proposed `uint32_t`)
specifically so `bracket()`/lerp math never has to reason about a stamp
wraparound.

**Angular interpolation is a separate helper, not folded into `lerp()`.**
`lerpAngle()` takes the shortest signed angular delta (`wrapAngle(newer -
older)`) and steps `frac` of that delta from `older`, rather than lerping
the raw angle values — a naive lerp across the ±180° seam interpolates
the *long* way around and by roughly 9x the true angular distance for a
value near the wrap point. `PoseReading::heading` is the one field that
needs this; every other interpolated field uses plain `lerp()`.

## 5. Interfaces

### Exposes

- **`Devices::Clock`/`Devices::Sleeper`** (`clock.h`): `nowMicros()` [us],
  `sleepMillis(duration)` [ms], `yield()`. One ARM instance of each
  (`MicroBitClock`/`MicroBitSleeper`), owned by `main()`, passed by
  reference to the loop and any module needing "now" (`App::Deadman`,
  `App::Preamble`). This is the fiber-cycle time seam — a *different* seam
  from `I2CBus`'s own internal clearance-timer clock, which is scoped
  purely to that class's own preClear/postClear bookkeeping.
- **`Devices::I2CBus`** (`i2c_bus.h`): `write()`/`read()` (mirroring
  `MicroBitI2C`'s signature, with the `preClear`/`postClear` [us] pair —
  see §3/§4) and `clearanceSafetyNetCount()` (a diagnostic counter read by
  `app/robot_loop.cpp` each cycle to populate a telemetry fault bit). One
  ARM instance (`MicroBitI2CBus`), owned by `main()`, held by every device
  leaf as an `I2CBus&`.
- **Device leaves** (`NezhaMotor`, `Otos`, `ColorSensorLeaf`,
  `LineSensorLeaf`): each leaf's own `begin()`/`beginStep(nowUs)`,
  `present()`/`connected()`, `tick(nowUs)`, and reading accessor
  (`position()`/`velocity()`/`appliedDuty()`; `pose()`/`poseFresh()`;
  `reading()`/`readingFresh()`) are this subsystem's primary public
  surface, called directly by the loop (`app/robot_loop.cpp`) — see each
  header's own declaration comments for the per-call contract (rate
  limits, no-op-until-detected behavior, staleness semantics).
  `Motor`/`Otos` additionally expose `sampleTime()` [us] (124-002): the
  `nowUs` of the tick that produced the CURRENTLY-cached reading — the
  last ACCEPTED fresh sample's own timestamp (`NezhaMotor::lastFreshUs_`/
  `RealOtos::lastReadUs_`), never "now" at call time. `MotorArmor` forwards
  it unmodified (no armor-side state of its own); every fake/stub in
  `tests/sim/unit/` and `App::FakeOtos` implement it too, since it is part
  of the abstract interface, not an ad hoc addition to the two real leaves.
  Added so a caller (telemetry's per-sample `age` fields) can compute a
  REAL per-sample age instead of stamping every reading with the same
  cycle's `now` — see
  `clasi/sprints/124-protocol-v5-robotstate-blackboard-and-radio-bench-gate/issues/protocol-v5-one-line-packets-command-prefix-and-newline-cobs.md`
  §B2.
  `Otos` is an abstract interface with two implementations —
  `Devices::RealOtos` (the chip) and `App::FakeOtos` (the `FAKE_OTOS`
  bench synthesizer) — see §4's own paragraph for the split and build seam.
- **`Devices::MeasurementRing<T>`/`Devices::Sample<T>`**
  (`measurement_ring.h`): `publish(value, stamp)`, `latest()`,
  `sample(age)`, `bracket(t, older, newer)` — see §3/§4 for the gap-write
  publish contract and the immutability guarantee readers may rely on.
- **`Devices::MotorArmor`** (`motor_armor.h`): the leaf contract described
  in §3/§4 — a leaf inherits this base, supplies three primitives, and
  gets `resetPosition()`/`wedged()`/`wedgeSuspect()`/
  `hardResetCount()`/`softResetCount()` for free.

### Consumes

- **CODAL / codal-microbit-v2 vendor SDK** (`microbit_clock.*`,
  `microbit_i2c_bus.*` only): `system_timer_current_time_us()`,
  `fiber_sleep()`, `schedule()`, `MicroBitI2C`, `target_disable_irq()`/
  `target_enable_irq()` — confined to the `microbit_*` ARM-only files; no
  other file in this directory includes `MicroBit.h`. See root
  `DESIGN.md`'s "Consumes" for the vendor-name naming exemption.
- **`app/`** (by construction, not by include): `main.cpp` constructs
  every leaf and the two seam singletons and wires them together; the
  loop (`app/robot_loop.cpp`) is the sole caller of every leaf's
  `tick()`/`beginStep()` and the sole reader of `clearanceSafetyNetCount()`.
  See `docs/design/design.md` §5 for the loop's own schedule and the
  `runAndWait` primitive that provides the timing gaps this subsystem's
  clearance windows and rate limits assume.

## 6. Open Questions / Known Limitations

- **Velocity-estimator and duty-averaging bench knobs
  (`NezhaMotor::setVelEstimator`/`setDutyAvg`) are live-tunable but not
  yet settled on a shipped default beyond EMA/no-averaging.** They exist
  for on-stand A/B comparison; a future pass may promote one combination
  to the sole implementation and delete the other.
- **OTOS velocity registers reuse the position/offset LSB scale
  constants** (`kPosMmPerLsb`/`kHdgRadPerLsb`) despite the chip
  documenting a different native velocity LSB scale — a known,
  out-of-scope-for-this-doc discrepancy carried forward unchanged from
  the pre-rebuild driver. A live twist-scaling correction needs its own
  bench-verifiable change, not a doc-only note.
- **(Resolved 115-005) Steady-state line/color sampling.** This used to
  read "not yet wired into the loop's cycle" — that gap is closed:
  `RobotLoop::updateLineColor()` now ticks one of the two leaves per
  cycle, alternating, from the trailing pace block (see
  [`../app/DESIGN.md`](../app/DESIGN.md) §2), and `Telemetry`'s packed
  `line`/`color` words carry the result. Left as a dated note rather
  than deleted outright, so a reader who only remembers the old gap can
  see it was closed and when.
