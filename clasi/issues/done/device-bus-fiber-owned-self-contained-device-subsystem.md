---
status: pending
---

# DeviceBus — a self-contained, fiber-owned device subsystem (all I2C, all device code, one handle API, ring-buffered measurements)

## Summary

Pull the entire I2C device plane out of the main loop into a new,
self-contained subsystem in its own directory. A dedicated CODAL fiber owns
the bus and runs a **straight-line, explicitly-timed schedule** — request
encoders, sleep the settle window, collect, run PID, write duty, read the
perception sensors — so the whole bus timeline is readable in one function.
The rest of the firmware talks to it only through per-device **handle
classes** obtained from one root object, and every measurement stream is a
**6-slot ring buffer** (5 published samples + 1 write gap) so published
values are immutable, history is iterable, and readings can be
time-bracket-interpolated.

This is the same move as motion-stack-v2 (sprint 100) for the drive layer:
gut the coupled version, rebuild the subsystem self-contained, cut over,
retire the old one — with bring-up first (stakeholder, 2026-07-12, resolved
open question 5): greenfield directory, tested via its own dedicated main
where DeviceBus is the only thing running, driven by DEV commands; the
complete cutover (never a both-systems-live coexistence) comes later as its
own work. It is the "real fix" that
[[motor-actuation-latency-flipflop-coupling]] has been waiting for, and this
issue absorbs that issue's "viable directions" section.

## Why (the recorded pain)

- **Actuation is coupled to sensing cadence.** The flip-flop does one bus
  phase per main-loop pass and duty writes happen only in COLLECT, so a wheel
  gets a duty write every ~80 ms, worst case ~160 ms after `setVelocity()`
  ([[motor-actuation-latency-flipflop-coupling]]).
- **The 093 decouple attempt hung the bus** because a duty write landed
  between a pending `0x46` encoder REQUEST and its COLLECT. The protection is
  sequencing, and today that sequencing is a cross-pass state machine other
  actors can interleave into.
- **The bus schedule is implicit.** "One action per pass" makes bus timing an
  emergent property of everything else in the loop — which is why 098-004's
  naive per-pass OTOS tick wrecked motion timing (-90° became -192°) and had
  to be reverted, and why 099-002 needed a page of hazard analysis to add one
  scheduled slot.
- **Settle windows are enforced by spinning.** `I2CBus`'s entry-side
  clearance busy-spins the whole loop until the deadline — CPU stolen from
  serial drain and telemetry.
- **Line and color sensing don't exist in the new tree yet** (capability
  headers only). They have to land somewhere; this subsystem is their home.
- **Values change under consumers.** Today a consumer reads whatever the last
  tick left in place; there is no history, no immutability guarantee, and no
  way to match two sensors' readings at a common past instant.

## Shape

- **New directory, new namespace, built greenfield**: `source/devices/`,
  `namespace Devices`. Proven policy code is *ported, not re-derived* (armor
  policy, velocity PID, OTOS driver, Nezha primitives); the flip-flop
  sequencer is not ported — it is retired at cutover.
- **Completely self-contained.** Everything that touches the bus moves in:
  - `source/com/i2c_bus.*` (the diagnostic wrapper, IRQ guard, clearance
    timers, HOST_BUILD scripted fake),
  - the Nezha motor leaf + armor base policy (`hal/nezha/*`, the
    `hal/capability/motor.h` armor/write-gate machinery),
  - the velocity PID (`hal/velocity_pid.*`),
  - the OTOS driver (`hal/otos/*`),
  - line + color sensor drivers (ported from `source_old` — including the
    PlanetX color re-wake-each-retry detection sequence, per
    `docs/knowledge/encoders-read-zero-i2c-bus-hang.md`).
- **Sole-ownership rule:** after cutover, no code outside `source/devices/`
  performs an I2C transaction, ever. Known violations to migrate: the
  `MainLoop::tick()` OTOS re-anchor (`odometer()->applySetPose()` — becomes a
  staged request the fiber drains), device `begin()`/detection (moves into
  the fiber's startup preamble), and any DBG/DEV command that pokes the bus
  (routes through staged requests or a fiber-internal diagnostic slot).
- `Subsystems::NezhaHardware` (the flip-flop) is deleted at cutover;
  `Subsystems::Drivetrain`, `Subsystems::PoseEstimator`, telemetry, and
  `Rt::MainLoop`'s commit all become handle consumers.

## The public surface

One root class; handles are member objects obtainable **only** from it
(private constructors, `friend class DeviceBus`, returned by reference,
non-copyable).

```cpp
namespace Devices {

class DeviceBus {
 public:
  void start();          // spawn the fiber: detection preamble, then the cycle
  void stop();           // request fiber exit; motors neutralized before exit
  bool running() const;

  Motor& motor(uint8_t port);   // 1-based port (wire/config convention)
  ColorSensor& color();
  LineSensor& line();
  Odometer& odometer();         // the OTOS
};

}  // namespace Devices
```

Every handle follows the same contract: **getters serve the most recent
published sample and never touch the bus; the stamp of that sample is always
available; setters stage a request the fiber picks up at its next cycle
top.** All handle methods are non-blocking and callable only from fiber
context (main loop), never from an ISR.

```cpp
namespace Devices {

struct MotorReading {
  float position;      // [mm]
  float velocity;      // [mm/s] signed
  float appliedDuty;   // [-1, 1]
};

class Motor {
 public:
  // Setters stage a request the fiber applies at its next cycle top; the
  // duty shaping, armor, and PID behind them are DeviceBus-internal. Multiple
  // setters are fine — velocity is the primary control target; neutral is a
  // real command a motor must always accept (coast vs brake); reset zeroes
  // the encoder (staged, standstill-guarded by the armor policy).
  void setVelocity(float velocity);          // [mm/s] signed — PID target
  void setNeutral(msg::Neutral mode);        // coast / brake
  void resetPosition();                      // zero encoder (staged, at-rest-guarded)

  // Bench/DEV surface (resolved open question 2): a flag turns the PID
  // controller on/off. PID off routes staged raw duty straight to the
  // ARMORED write path (armor applies in both modes).
  void setPidEnabled(bool on);               // default true
  void setDuty(float duty);                  // [-1, 1] applied only while PID is off

  Sample<MotorReading> latest() const;
  Sample<MotorReading> sample(uint8_t age) const;   // age 0 = newest … 4 = oldest
  bool sampleAt(uint32_t t, MotorReading& out) const;   // [us] bracketed lerp
  uint32_t updatedAt() const;                // [us] == latest().stamp (width: open Q3 note)
  bool connected() const;
  // diagnostics pass-through: wedged()/wedgeSuspect()/encGlitchCount()/…
};

class ColorSensor {   // r/g/b/c reading, same latest()/sample()/sampleAt()/updatedAt() surface
};
class LineSensor {    // 4-channel raw+normalized reading, same surface
};
class Odometer {      // pose (+ OTOS velocities), same surface, plus a staged
                      // setPose() re-anchor request (drained by the fiber at a
                      // safe slot — replaces MainLoop's inline applySetPose())
};

}  // namespace Devices
```

The velocity PID moves inside: it already lives in the motor leaf (the
`setVelocity()` target is "the target the embedded PID chases in tick()"),
but its *cadence* is currently the flip-flop's COLLECT phase. Here it runs at
the fiber's cycle rate with a duty write every cycle — actuation latency
drops from ~80–160 ms to roughly one cycle (~10–20 ms). Consumers never see
duty except as telemetry readback.

## Measurement rings (the 6-slot gap-write buffer)

Every measurement stream (each motor's encoder reading, OTOS pose, line,
color) is published through a fixed-depth single-writer ring:

- **6 physical slots, 5 published.** The slot just past the head is the
  *write gap*. The fiber writes the new sample into the gap, then advances
  the head with a single aligned store; the tail is implicit (head − 4). No
  published slot is ever mutated — readers can hold or copy any of the 5
  published samples without racing the writer.
- **Published samples are immutable.** This is the "nobody's values change
  underneath them" guarantee, and it holds independent of the cooperative-
  scheduler argument (belt and suspenders — it would stay correct even under
  a preemptive writer, though we are deliberately not building one).

```cpp
namespace Devices {

template <typename T>
struct Sample {
  T value{};
  uint32_t stamp = 0;   // [us] system_timer_current_time_us() at the fiber's
                        // read (resolved open question 3; width/wrap: Q3 note)
  bool valid = false;   // false until the stream's first publish
};

// Single-writer (the fiber), multi-reader. kSlots = 6, kDepth = 5: the
// unpublished 6th slot is the write gap.
template <typename T>
class MeasurementRing {
 public:
  void publish(const T& value, uint32_t stamp);      // [us] fiber-only
  Sample<T> latest() const;
  Sample<T> sample(uint8_t age) const;               // age 0 = newest … kDepth-1
  // Bracketed lookup for interpolation: the two published samples with
  // older.stamp <= t <= newer.stamp. False if t is outside the window.
  bool bracket(uint32_t t, Sample<T>& older, Sample<T>& newer) const;   // [us]
};

}  // namespace Devices
```

What the history enables (the point of the exercise):

1. **Derived quantities from raw history** — e.g. acceleration as a
   difference quotient over the velocity ring, computed consumer-side at
   whatever filter the consumer wants. (The base-class `trackAcceleration()`
   EMA can stay during transition; the ring may eventually obsolete it —
   consumer choice, not mandated here.)
2. **Cross-sensor time alignment** — `sampleAt(t)` brackets a past instant
   and linearly interpolates, so a consumer can ask "what was the encoder
   position at the OTOS sample's stamp" (or at a camera frame's stamp)
   instead of pairing whatever happened to be latest. Each reading type
   supplies its own lerp; **OTOS heading needs wrap-aware angular lerp** —
   naive linear interpolation across ±180° is a known trap.
3. **PoseEstimator integration at true sample times** — the stamps ride with
   the values (the `sampleTime()` aliasing-staircase fix, generalized).

RAM cost is trivial (6 × ~16–32 B per stream × ~7 streams ≲ 1.5 KB) but gets
checked at build like everything else — flash overflow is the real
constraint on this target, not the by-design-near-full RAM.

## The fiber and its cycle

`start()` spawns one CODAL fiber (`create_fiber`). Its body:

1. **Detection preamble** — power-settle wait, then per-device `begin()` with
   retries (color re-wakes its registers each retry). Because this runs in
   the fiber, retries no longer freeze the control loop or block boot — the
   main loop is already serving serial/radio while detection proceeds.
   Absent devices are marked and their slots skipped (the 099-002
   `present()`-not-`connected()` lesson carries over).
2. **The cycle** — straight-line code, explicit sleeps, one readable
   timeline (illustrative; exact order/budget is bench-tuned):

```cpp
for (;;) {
  if (stopRequested_) break;
  drainStagedInputs();            // targets, config deltas, OTOS setPose — no bus
  requestEncoder(port1);          // 0x46 write   (+ port2 if the brick pipelines)
  fiber_sleep(4);                 // [ms] vendor settle — YIELDS, never spins
  collectEncoder(port1);          // paired read; PID; armored duty write
  collectEncoder(port2);          //   (or alternate ports per cycle if not pipelined)
  perceptionSlot();               // round-robin one of: line | color | OTOS
  publishSamples(now);            // ring publishes — plain stores, no yield
  fiber_sleep(cycleRemainder);    // [ms] pace the cycle; main loop runs here
}
neutralizeAllMotors();            // stop() epilogue — wheels never left driven
```

Budget sketch from measured transaction costs (encoder pair ≈ 8 ms
settle-bound, duty ≈ 0.5 ms, line 2.4 ms, color 4.0 ms, OTOS ≈ 2 ms): a
~16 ms cycle gives **encoders + PID + duty at ~60 Hz** and each perception
sensor at ~20 Hz — versus today's ~12 Hz effective duty cadence per wheel and
no line/color at all.

Also inside the subsystem:

- **The REQUEST→COLLECT hazard dies structurally.** The pairing is
  straight-line code in one function and the fiber is the sole bus owner —
  there is no longer any way for another actor to inject a `0x60` write
  between a pending request and its collect (the exact 093 hang).
- **Watchdog/neutralize policy moves in.** The fiber is the thing writing
  duty, so the stale-target motor-neutralize gate must live here: if staged
  targets (or the host RX watchdog feed) go stale past the deadline, the
  fiber writes neutral itself. The main loop can crash and the wheels still
  stop.
- **Armor stays intact.** Reversal dwell, output deadband, standstill-guarded
  resets, wedge detector — ported verbatim, and `I2CBus::_irqGuard` stays ON
  (TWIM errata). Non-negotiable per the prior issue's constraints.
- **Config plane:** `Rt::Configurator` keeps its one-authority role; deltas
  reaching motor/PID/odometer scope are staged onto DeviceBus and applied by
  the fiber at cycle top (gains stay live-tunable over the radio, per the
  098-005 bench workflow).

## Concurrency contract

CODAL fibers are cooperative — context switches happen only at yield points.
The rules, stated once and enforced in review:

1. The fiber is the only writer of rings and the only bus toucher; consumers
   are the only writers of staged-input cells.
2. **No yield inside a publish, a staged-input store, or a consumer-side
   sample copy.** All are plain struct stores/copies; none call anything that
   can sleep.
3. Handles are main-loop-fiber API. Nothing here is ISR-safe, by design.
4. The main loop keeps its own `uBit.sleep(1)` yield per pass (radio delivery
   depends on it); the device fiber's sleeps are what hand time back the
   other way.

## Rejected alternative — timer-interrupt ping-pong (recorded so it isn't re-proposed)

Two timer ISRs arming each other, doing the bus work in interrupt context,
was evaluated and rejected:

- Its premise fails: the IRQ guard exists for the nRF52 TWIM errata ("under
  higher levels of background interrupt load"), not for mutual exclusion.
  Higher-priority IRQs (radio, UARTE) still preempt a timer ISR, so the
  masking would still be needed — now nested inside interrupt context.
- "Pause the right amount of time" inside an ISR is a busy-wait (no sleeping
  in interrupt context): a request→4 ms settle→collect→color batch is a
  6–8 ms ISR freezing every fiber and same-or-lower-priority IRQ. Any IRQ
  masking already costs serial RX bytes; this makes the windows longer and
  contiguous. Timer-chained micro-ISRs only reach parity with today's
  per-transaction guard, at far higher complexity.
- It reintroduces real preemption: every shared cell then needs true critical
  sections (IRQ masking in the main loop — the exact mechanism that drops
  serial bytes), and the CODAL I2C driver + Nezha/armor/OTOS code paths were
  never written for ISR context (the OTOS init even calls `fiber_sleep`).
- Its only genuine advantage — cadence immune to main-loop load — stops
  mattering once the bus work leaves the main loop: the remaining main-loop
  pass is sub-millisecond math and parsing, so the fiber's sleeps are honored
  within ~1–2 ms, and nothing on this bus needs better.

## Sim / host-test story

- The cycle body is parameterized on a sleeper/clock interface: `fiber_sleep`
  + `system_timer` on hardware; the steppable fake clock in host tests. The
  existing HOST_BUILD scripted-`I2CBus` flip-flop-harness pattern carries
  over to drive the cycle deterministically (order, settle deadlines, hazard
  cases).
- The handle API is the one consumer surface for sim too: a `SimDeviceBus`
  (or DeviceBus fronting sim leaves — decide in the architecture update)
  publishes into the same rings from `PhysicsWorld`, so Drivetrain/
  PoseEstimator/telemetry code is identical on both targets.
- `MeasurementRing` itself is plain host-testable C++ (publish/wrap/bracket/
  lerp edge cases, including the angular-lerp wrap case).

## Bench gates (standing hardware gate applies, plus these specifics)

1. **Does the Nezha brick hold two per-motorId encoder requests pending
   simultaneously?** Gates the pipelined dual-request cycle; fallback is
   alternating ports with duty writes every cycle.
2. **Measure `fiber_sleep(4)` actual latency distribution** on the bench
   before committing the cycle budget (late is safe — settle windows are
   minimums — but the budget should be honest).
3. **Reversal-stress re-verification**: the wedge/runaway armor must be
   re-proven under the new cadence (`wedge_latch_matrix.py`, DBG WEDGE).
4. **Serial/radio health**: binary-vs-text same-boot discriminator before and
   after cutover — the per-transaction IRQ-guard pattern must not get worse.
5. **Motion non-regression**: 098's heading-loop turn accuracy (100% within
   ±1°) re-run after cutover; plus the 099 pose-estimation acceptance.
6. Flash/RAM delta check (flash overflow is the real limit).

## Open questions (decide in the sprint's architecture update, not silently)

1. **~~Neutral/coast and encoder-zero vs the one-setter contract.~~
   RESOLVED (stakeholder, 2026-07-12): no one-setter mandate — multiple
   setters are fine.** The motor handle carries `setVelocity()` (PID target),
   `setNeutral()` (coast vs brake — distinct from `setVelocity(0)`, which
   means "PID actively chases zero"), and `resetPosition()` (staged,
   at-rest-guarded encoder zero, which also serves the SI/ZERO arm). All three
   stage a request the fiber applies at cycle top; none touch the bus from the
   caller. Still to settle in the architecture update: whether `msg::Neutral`
   is the handle's coast/brake vocabulary or a `Devices`-local enum.
2. **~~DEV/debug duty access.~~ RESOLVED (stakeholder, 2026-07-12): a flag
   that turns the PID controller on and off.** PID on (default):
   `setVelocity()` targets feed the PID as specced. PID off: staged raw-duty
   commands drive the armored duty write directly — this is the bench
   tooling's raw-duty surface (wedge matrix, step tests). The armor policy
   stays in the write path in both modes (non-negotiable, see Related).
   Still to settle in the architecture update: per-motor vs bus-wide flag,
   and its DEV-command spelling.
3. **~~Stamp resolution.~~ RESOLVED (stakeholder, 2026-07-12):
   microseconds.** Ring stamps come from `system_timer_current_time_us()`;
   the sketches above are specced [us]. One detail for the architecture
   update: stamp width — a uint32_t of [us] wraps at ~71.6 minutes, so
   either carry uint64_t (RAM cost trivial) or make `bracket()`/lerp
   wrap-safe. Blackboard/TLM [ms] stamps stay as they are; convert at the
   consumer boundary.
4. **~~Ports (GPIO digital/analog) and servo.~~ RESOLVED (stakeholder,
   2026-07-12): out of scope.** This subsystem owns the I2C bus, not all
   peripherals; ports/servo stay where they are.
5. **~~Cutover sequencing vs sprint 100.~~ RESOLVED (stakeholder,
   2026-07-12): cutover is explicitly deferred — just create the subsystem
   competently, greenfield.** Initial delivery is the self-contained
   subsystem tested with its OWN dedicated main — a bring-up firmware where
   DeviceBus is the only thing running, driven directly via DEV commands. No
   consumer cutover in the same sprint, and **no runtime coexistence with
   the legacy stack, ever**: the two systems never both run in one binary —
   if both lines of code happen to be compiled together, one of them is
   unwired. Cutover, when it comes, is complete (Drivetrain/PoseEstimator/
   telemetry become handle consumers, flip-flop deleted) — a single swap,
   not an incremental both-alive migration. This removes any bus-ownership
   arbitration question by construction.

## Related

- [[motor-actuation-latency-flipflop-coupling]] — absorbed; this is its
  "real fix."
- [[i2c-irqguard-vs-serial-rx]], [[encoder-wedge-boundary-latch]] — the
  constraints any bus redesign must honor (guard stays on; armor stays).
- `docs/knowledge/loop-timing-and-control-frequency.md` — measured
  transaction costs behind the cycle budget.
- `docs/knowledge/encoders-read-zero-i2c-bus-hang.md` — detection-placement
  lessons the fiber preamble encodes.
- Sprint 100 (`motion-stack-v2`) — the consumer on the other side of the
  motor handles, and the precedent for the gut-and-rebuild process.
