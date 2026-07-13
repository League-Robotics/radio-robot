// device_bus.h — Devices::DeviceBus: the subsystem root. Owns the I2C bus,
// every device leaf, one MeasurementRing per stream, and the Clock/Sleeper
// time seam; hands out the handle classes (handles.h) that are the ONLY way
// the rest of the firmware ever touches a device.
//
// Ticket DB-007 (device-bus-tickets.md). Implements clasi/issues/
// device-bus-fiber-owned-self-contained-device-subsystem.md's "The public
// surface", "The fiber and its cycle", and "Concurrency contract" sections.
//
// --- Scope: DB-007 vs DB-008 ---
// This ticket delivers the ROOT OBJECT and the straight-line CYCLE BODY
// (runCycleOnce()) — the "host-steppable seam" device-bus-tickets.md's
// header resolves the issue's own open cycle-testability question into:
// "the for(;;) fiber body is factored into a plain runCycleOnce() method the
// host harness steps deterministically; the real fiber is just
// while(!stopRequested_) runCycleOnce();". start()/stop()/running() and the
// detection preamble (issue "The fiber and its cycle", step 1: power-settle
// wait, per-device begin()/beginStep() retries, absent-device slot-skipping)
// are explicitly DB-008's ticket ("Fiber lifecycle... preamble, epilogue") —
// this file does NOT spawn a fiber and does NOT run any retry-driven
// preamble loop. A host test (or DB-009's bring-up main, later) that needs a
// leaf actually detected before stepping runCycleOnce() drives begin()/
// beginStep() directly through the HOST_BUILD test-seam accessors below
// (motorLeaf()/otosLeaf()/colorLeaf()/lineLeaf()) — DB-008 replaces that
// ad hoc priming with the real async, fiber-hosted preamble for production.
//
// --- The exact runCycleOnce() schedule (device-bus-tickets.md's DB-007
// section, itself a firmer commitment than the issue's own illustrative,
// bench-gated-pending sketch — see the "pipelined vs alternating ports"
// note below) ---
//   drainStagedInputs()          -- targets, watchdog gate; no bus
//   requestEncoders()            -- 0x46 write, motor1 THEN motor2
//   sleeper_.sleepMillis(4)      -- [ms] vendor settle -- YIELDS, never spins
//   collectAndDrive(now)         -- collect+PID+armored write, motor1 THEN motor2
//   perceptionSlotStep(now)      -- round-robin ONE of: line | color | OTOS
//   publishSamples(now)          -- ring publishes -- plain stores, no yield
//   sleeper_.sleepMillis(pace)   -- [ms] pace the cycle
//
// --- Why the 093 REQUEST->COLLECT hazard is STRUCTURALLY absent ---
// Every motor's requestSample() (encoder-select 0x46 write) happens in
// requestEncoders(), and every motor's collect+possible duty write (0x60)
// happens in collectAndDrive() — and requestEncoders() always runs to
// completion, for BOTH motors, strictly before collectAndDrive() begins.
// There is therefore no code path by which ANY duty write (which only ever
// originates inside NezhaMotor::tick(), itself only ever called from
// collectAndDrive()) can land between motor N's own request and motor N's
// own collect: the only bus traffic between them is motor (3-N)'s OWN
// encoder request (also a 0x46 select write, not a 0x60 duty write) — never
// a duty write. This is a structural property of runCycleOnce()'s fixed
// call order, not a runtime check — matching the issue's own claim ("there
// is no longer any way for another actor to inject a 0x60 write between a
// pending request and its collect").
//
// --- Pipelined-vs-alternating dual-request note ---
// The issue's own cycle sketch ("The fiber and its cycle") hedges this exact
// point pending DB-009's bench gate #1 ("Does the Nezha brick hold two
// per-motorId encoder requests pending simultaneously?"). device-bus-
// tickets.md's DB-007 section resolves the hedge for THIS ticket by
// specifying "requestEncoder (all motors) -> settle-sleep -> collect... (all
// motors)" explicitly — i.e. commits to the pipelined dual-request form.
// DB-009 (HITL, later) re-verifies this against real hardware; if the brick
// cannot hold two pending requests, that ticket's own bench gate is where
// the fallback (alternating ports, one request+collect pair per cycle) gets
// wired in — a schedule change local to requestEncoders()/collectAndDrive(),
// not a redesign of this file's public surface.
//
// --- Stale-target / RX-watchdog neutralize gate ---
// drainStagedInputs() (device_bus.cpp) re-asserts Neutral::Coast on any
// motor whose most recent Motor::setVelocity() call is older than
// kVelocityStaleUs — the fiber, not the (possibly crashed) main loop, is
// what actually holds the wheels safe (issue: "The main loop can crash and
// the wheels still stop"). See handles.h's Motor class design note for why
// this needs exactly one extra timestamp field on the handle and nothing
// more elaborate.
//
// --- Concurrency contract (issue) ---
// 1. This object (via runCycleOnce(), eventually fiber-hosted by DB-008) is
//    the ONLY writer of every ring and the ONLY bus toucher; handle setters
//    (handles.h) are the only writers of staged-input state, and every one
//    of them is a plain, yield-free store (see handles.h's own design
//    notes for Motor/Odometer).
// 2. No yield inside publishSamples()/drainStagedInputs()/any handle
//    sample-copy — all plain struct stores/copies (measurement_ring.h's own
//    publish()/latest()/sample()/bracket() are already yield-free by
//    construction).
// 3. Handles are main-loop-fiber API only — nothing here is ISR-safe.
#pragma once
#ifndef HOST_BUILD
#include "MicroBit.h"
#endif
#include <cstdint>

#include "devices/clock.h"
#include "devices/color_sensor.h"
#include "devices/device_config.h"
#include "devices/device_types.h"
#include "devices/handles.h"
#include "devices/i2c_bus.h"
#include "devices/line_sensor.h"
#include "devices/measurement_ring.h"
#include "devices/nezha_motor.h"
#include "devices/otos.h"

namespace Devices {

class DeviceBus {
 public:
  // Two NezhaMotor channels (the current Tovez differential-drive target —
  // see device-bus-tickets.md's DB-007 "2+ NezhaMotor" wording; a 4-channel
  // mecanum bring-up is a follow-on ticket's scope change, not this one's),
  // one Otos, one ColorSensorLeaf, one LineSensorLeaf, wired to the ONE
  // I2CBus this object owns (i2c_bus.h's own "Usage" note: "one I2CBus
  // instance, owned by DeviceBus, constructed before any device leaf and
  // passed to each by reference").
#ifndef HOST_BUILD
  DeviceBus(MicroBitI2C& i2c, const MotorConfig& motor1Config,
            const MotorConfig& motor2Config, const OtosConfig& otosConfig,
            const ColorConfig& colorConfig, const LineConfig& lineConfig);
#else
  DeviceBus(const MotorConfig& motor1Config, const MotorConfig& motor2Config,
            const OtosConfig& otosConfig, const ColorConfig& colorConfig,
            const LineConfig& lineConfig);
#endif

  DeviceBus(const DeviceBus&) = delete;
  DeviceBus& operator=(const DeviceBus&) = delete;

  // --- The public surface (issue) -- start()/stop()/running() are DB-008's
  // addition (this file's own header comment, "Scope: DB-007 vs DB-008"). ---

  // 1-based port (wire/config convention, matches MotorConfig::port). Any
  // port other than 2 resolves to the first channel -- config validation is
  // an upstream concern (not this ticket's), so this getter is deliberately
  // permissive rather than fallible.
  Motor& motor(uint8_t port);
  ColorSensor& color() { return colorHandle_; }
  LineSensor& line() { return lineHandle_; }
  Odometer& odometer() { return odometerHandle_; }

  // runCycleOnce() -- the host-steppable cycle core; see this file's own
  // header comment for the exact schedule. DB-008 wraps this in
  // `while (!stopRequested_) runCycleOnce();` inside the real fiber; a host
  // harness (this ticket's own acceptance tests) steps it directly.
  void runCycleOnce();

#ifdef HOST_BUILD
  // -------------------------------------------------------------------
  // HOST_BUILD-only test seam -- mirrors I2CBus's own HOST_BUILD-only
  // scriptWrite()/scriptRead()/setClock() surface (i2c_bus.h). Gives a host
  // harness direct access to: the owned I2CBus (to script transactions --
  // no other way to reach it, since every leaf/handle keeps its own
  // reference private) and each leaf (to run begin()/beginStep() priming
  // ahead of runCycleOnce(), standing in for DB-008's not-yet-built
  // preamble -- see this file's own "Scope: DB-007 vs DB-008" note) and the
  // Clock/Sleeper seam (to step/inspect them directly). NEVER compiled into
  // real firmware.
  // -------------------------------------------------------------------
  I2CBus& bus() { return bus_; }
  NezhaMotor& motorLeaf(uint8_t port);
  Otos& otosLeaf() { return otos_; }
  ColorSensorLeaf& colorLeaf() { return color_; }
  LineSensorLeaf& lineLeaf() { return line_; }
  Clock& clock() { return clock_; }
  Sleeper& sleeper() { return sleeper_; }
#endif

 private:
  // [ms] vendor settle window between requestEncoders() and collectAndDrive()
  // -- matches nezha_motor.cpp's own requestEncoder()/writeMotorRun()
  // clearance windows and the issue's own cycle sketch's `fiber_sleep(4)`.
  static constexpr uint32_t kEncoderSettleMs = 4;

  // [ms] pace-sleep budget for the remainder of the issue's own "~16ms
  // cycle" sketch (encoders+PID+duty at ~60Hz, each perception sensor at
  // ~20Hz via the 3-way round robin) after the 4ms settle sleep already
  // spent above -- bench-tunable (DB-009), not a hard real-time guarantee
  // this ticket enforces.
  static constexpr uint32_t kCyclePaceMs = 12;

  // [us] RX-watchdog neutralize deadline -- see this file's own "Stale-
  // target / RX-watchdog neutralize gate" header note. A starting,
  // bench-tunable value (matches the project's existing ~200ms DEV-serial
  // watchdog-feed cadence with headroom -- see
  // .clasi/knowledge or dev-serial-passive-pump-sampling precedent) more
  // than DB-009's job to tighten against real telemetry loss statistics.
  static constexpr uint64_t kVelocityStaleUs = 300000;

  // Round-robin perception schedule -- device-bus-tickets.md's DB-007
  // "round-robin perceptionSlot (line | color | OTOS, one per cycle)".
  enum class PerceptionSlot : uint8_t { Line, Color, Otos, kCount };

  void drainStagedInputs();
  void applyStaleGate(Motor& handle, NezhaMotor& leaf, uint64_t nowUs);  // [us]
  void requestEncoders();
  void collectAndDrive(uint64_t nowUs);    // [us]
  void perceptionSlotStep(uint64_t nowUs);  // [us]
  void publishSamples(uint64_t nowUs);      // [us]

  // ---- Declaration order IS construction order -- see device_bus.cpp's
  // constructor for the dependency chain this order satisfies (bus_ before
  // every leaf; leaves+rings+clock_ before every handle). ----
  I2CBus bus_;
  Clock clock_;
  Sleeper sleeper_;

  NezhaMotor motor1_;
  NezhaMotor motor2_;
  Otos otos_;
  ColorSensorLeaf color_;
  LineSensorLeaf line_;

  MeasurementRing<MotorReading> motor1Ring_;
  MeasurementRing<MotorReading> motor2Ring_;
  MeasurementRing<PoseReading> otosRing_;
  MeasurementRing<ColorReading> colorRing_;
  MeasurementRing<LineReading> lineRing_;

  Motor motor1Handle_;
  Motor motor2Handle_;
  ColorSensor colorHandle_;
  LineSensor lineHandle_;
  Odometer odometerHandle_;

  // Round-robin cursor: which perception leaf THIS cycle serviced (set by
  // perceptionSlotStep(), read by publishSamples() -- see that function's
  // own comment for why a fresh-flag check alone is not enough).
  PerceptionSlot perceptionSlot_ = PerceptionSlot::Line;
  PerceptionSlot lastPerceptionSlot_ = PerceptionSlot::Line;
};

}  // namespace Devices
