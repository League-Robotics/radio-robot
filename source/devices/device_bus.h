// device_bus.h — Devices::DeviceBus: the subsystem root. Owns the I2C bus,
// every device leaf, one MeasurementRing per stream, and the Clock/Sleeper
// time seam; hands out the handle classes (handles.h) that are the ONLY way
// the rest of the firmware ever touches a device.
//
// Tickets DB-007 and DB-008 (device-bus-tickets.md). Implements clasi/issues/
// device-bus-fiber-owned-self-contained-device-subsystem.md's "The public
// surface", "The fiber and its cycle", and "Concurrency contract" sections.
//
// --- Scope: DB-007 vs DB-008 ---
// DB-007 delivered the ROOT OBJECT and the straight-line CYCLE BODY
// (runCycleOnce()) — the "host-steppable seam" device-bus-tickets.md's
// header resolves the issue's own open cycle-testability question into:
// "the for(;;) fiber body is factored into a plain runCycleOnce() method the
// host harness steps deterministically; the real fiber is just
// while(!stopRequested_) runCycleOnce();". DB-008 (this revision) adds
// start()/stop()/running() and the detection preamble (issue "The fiber and
// its cycle", step 1: power-settle wait, per-device begin()/beginStep()
// retries, absent-device slot-skipping) — see runPreamble()/start()/stop()'s
// own declaration comments, below, and fiber_runner.h for the FiberRunner
// seam that lets a host test exercise start()/stop() without CODAL. A host
// test that wants a leaf primed WITHOUT exercising start()'s own preamble
// (e.g. DB-007's own device_bus_cycle_harness.cpp, predating this ticket)
// may still drive begin()/beginStep() directly through the HOST_BUILD
// test-seam accessors below (motorLeaf()/otosLeaf()/colorLeaf()/lineLeaf())
// — that ad hoc path and the real preamble both remain valid, independent
// ways to get a leaf detected in a host test.
//
// --- The exact runCycleOnce() schedule ---
//   drainStagedInputs()          -- targets, watchdog gate; no bus
//   serviceMotor(motor1)         -- 0x46 request -> settle-sleep -> collect+PID+armored write
//   serviceMotor(motor2)         -- same, SEPARATELY (see the alternating note below)
//   perceptionSlotStep(now)      -- round-robin ONE of: line | color | OTOS
//   publishSamples(now)          -- ring publishes -- plain stores, no yield
//   sleeper_.sleepMillis(pace)   -- [ms] pace the cycle
//
// --- Alternating dual-request: resolved by DB-009 HITL ---
// The issue's cycle sketch and device-bus-tickets.md's DB-007 section
// originally committed to a PIPELINED form -- requestEncoder(BOTH motors) ->
// ONE settle-sleep -> collect(BOTH) -- and hedged it pending DB-009's bench
// gate #1 ("Does the Nezha brick hold two per-motorId encoder requests
// pending simultaneously?"). DB-009's HITL bench answered it: NO. The brick
// holds only ONE pending 0x46 request, so the pipelined form dropped motor
// 2's request every cycle (motor 1's request, issued first, stayed pending)
// and motor 2's encoder never refreshed -- frozen position, vel=0, false
// wedge latch -- even as its wheel physically spun. The pre-specified
// fallback is now wired in: serviceMotor() gives EACH motor its own
// request -> settle -> collect pair, serviced in alternation. It is local to
// the motor phase, not a redesign of this file's public surface, exactly as
// the hedge anticipated.
//
// --- Why the 093 REQUEST->COLLECT hazard is STRUCTURALLY absent ---
// A duty write (0x60) only ever originates inside NezhaMotor::tick(), itself
// called only from serviceMotor(), AFTER that same motor's own collect.
// Nothing at all touches the bus between a motor's own 0x46 request and its
// own collect -- the alternating form makes this even tighter than the
// pipelined one did (which had the OTHER motor's request sitting in that
// window). So there is no code path by which any actor can inject a 0x60
// write between a pending request and its collect -- a structural property
// of runCycleOnce()'s fixed call order, not a runtime check.
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
#include "devices/fiber_runner.h"
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

  // --- Fiber lifecycle (DB-008; issue "The fiber and its cycle" / "The
  // public surface") ---
  //
  // start() -- hands the fiber body (detection preamble, then
  // `while (!stopRequested_) runCycleOnce();`) to the currently-injected
  // FiberRunner (fiber_runner.h) and marks running() true. Real (CODAL)
  // builds: the FiberRunner spawns an async fiber and start() returns to its
  // caller IMMEDIATELY -- detection retries never block boot (issue: "the
  // main loop is already serving serial/radio while detection proceeds").
  // Host builds: the default (or injected) FiberRunner runs the preamble
  // plus a BOUNDED number of cycles synchronously, in place -- see
  // fiber_runner.h's own header comment.
  void start();

  // stop() -- request exit, join, THEN neutralize (device-bus-tickets.md's
  // DB-008 section's own three-step ordering, not the issue's illustrative
  // sketch, which puts the neutralize call at the tail of the fiber body
  // itself -- see fiber_runner.h's header comment and this file's own
  // neutralizeAllMotors() comment for why stop() -- not the fiber body --
  // owns the neutralize call here): (1) stopRequested_ = true so the cycle
  // loop exits at its next while-condition check; (2) join -- block
  // (cooperative yield-poll; a no-op in host builds, where the loop already
  // finished synchronously inside start()) until nothing is still touching
  // the bus; (3) neutralizeAllMotors() -- a real, synchronous bus write per
  // motor, run HERE, so it is unconditionally the LAST bus action any motor
  // sees before stop() returns. running() is false only once all three
  // steps are complete.
  void stop();

  bool running() const { return running_; }

  // Lets a host test inject a different FiberRunner than the one this
  // object constructs for itself by default (defaultFiberRunner_, below) --
  // device-bus-tickets.md's own DB-008 wording: "an interface letting host
  // tests inject a synchronous runner." Must be called before start();
  // `runner` must outlive every start()/stop() call made against this
  // object. Not gated behind HOST_BUILD -- any FiberRunner-conforming
  // object may be injected on real hardware too, though CodalFiberRunner is
  // the only sensible choice there.
  void setFiberRunner(FiberRunner& runner) { fiberRunner_ = &runner; }

  // runCycleOnce() -- the host-steppable cycle core; see this file's own
  // header comment for the exact schedule. The real fiber body (run by
  // whichever FiberRunner start() is holding, above) is
  // `while (!stopRequested_) runCycleOnce();`; a host harness (this
  // ticket's and DB-007's own acceptance tests) may also step it directly.
  void runCycleOnce();

#ifdef HOST_BUILD
  // -------------------------------------------------------------------
  // HOST_BUILD-only test seam -- mirrors I2CBus's own HOST_BUILD-only
  // scriptWrite()/scriptRead()/setClock() surface (i2c_bus.h). Gives a host
  // harness direct access to: the owned I2CBus (to script transactions --
  // no other way to reach it, since every leaf/handle keeps its own
  // reference private) and each leaf (for pre-DB-008-style ad hoc priming,
  // still useful when a test wants a leaf primed WITHOUT exercising
  // start()'s own preamble -- e.g. DB-007's device_bus_cycle_harness.cpp)
  // and the Clock/Sleeper seam (to step/inspect them directly). NEVER
  // compiled into real firmware.
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
  // FiberRunner implementations (fiber_runner.h) are separate types, not
  // members of Devices::DeviceBus, but need to call runPreamble()/
  // stopRequested()/markLoopExited() below (all private) as part of running
  // this object's fiber body -- friended rather than made public, matching
  // handles.h's own friend-based access pattern for the same reason (a
  // minimal public surface with a named, auditable exception list). Only
  // the ONE concrete FiberRunner this build actually compiles is friended.
#ifndef HOST_BUILD
  friend class CodalFiberRunner;
#else
  friend class HostFiberRunner;
#endif

  // --- Fiber-body primitives -- called ONLY by the FiberRunner in play
  // (via the friend declarations above) as part of running this object's
  // fiber body; never called directly by ordinary consumer code. ---

  // The detection preamble (issue "The fiber and its cycle" step 1):
  // power-settle wait, then each device's begin()-style detection. Motor/
  // OTOS detection is a single begin() call each -- NezhaMotor::begin()
  // (hardReset()) and Otos::begin() (product-ID probe) are both already
  // fully self-contained, bounded operations (DB-004/DB-005); this method
  // does not loop either of them. Color/line detection instead uses each
  // leaf's own non-blocking beginStep(nowUs) state machine (DB-006), so
  // THIS method drives a bounded, LOCAL retry-pacing loop -- see
  // device_bus.cpp's own comment on why nowUs is advanced by hand rather
  // than re-read from clock_ between attempts. Absent devices are marked
  // (present() false) and structurally skipped from then on: every leaf's
  // own tick()/beginStep() already no-ops when !initialized_ (DB-004
  // through DB-006's own precedent), so this method adds no separate
  // "skip" bookkeeping of its own.
  void runPreamble();

  bool stopRequested() const { return stopRequested_; }
  void markLoopExited() { loopExited_ = true; }

  // stop()'s own epilogue (see stop()'s own comment above for why this is
  // NOT called from inside the fiber body/FiberRunner): stages Neutral on
  // both motors then serviceMotor()s each once more (the SAME per-motor
  // request->settle->collect step runCycleOnce() itself calls, in the SAME
  // alternating order) so each motor's neutral duty write
  // lands through a properly-paired encoder request/collect, never a bare
  // unpaired bus read -- and, because NezhaMotor::tick()'s own 5-step order
  // always collects (reads) BEFORE it dispatches this tick's mode (writes),
  // the neutral WRITE is unconditionally the last bus action either motor
  // sees from this call.
  void neutralizeAllMotors();

  // [ms] boot power-settle wait, the preamble's first step (issue "The
  // fiber and its cycle" step 1) -- a starting, bench-tunable value
  // (DB-009's job to tighten against real chip power-up timing).
  static constexpr uint32_t kPowerSettleMs = 50;

  // [ms] preamble retry pacing between color_/line_ beginStep() attempts --
  // matches color_sensor.h's kAltRetryPeriod / line_sensor.h's kRetryPeriod
  // (both 50000us == 50ms) exactly, so every preamble tick lands a due
  // attempt on WHICHEVER of the two leaves is still detecting.
  static constexpr uint32_t kPreambleRetryPacingMs = 50;

  // Safety bound on the preamble's color_/line_ retry loop -- color_'s own
  // worst case is kMaxAltAttempts+1 (AltProbe exhaustion, then one
  // ApdsProbe attempt) == 21 ticks at this pacing; line_'s is kMaxAttempts
  // == 20. This cap is never reached in normal operation; it exists only to
  // guard against a future leaf regression turning this into a real
  // infinite loop (the same defensive-bound spirit as NezhaMotor::
  // hardReset()'s own kMaxRetries).
  static constexpr int kMaxPreambleTicks = 64;

  // OTOS product-ID probe retry (101-001): the SparkFun OTOS needs ~1s after
  // power-on before its ID register reads 0x5F. Otos::begin() is a single probe
  // with no retry, so runPreamble() retries it up to kOtosBeginAttempts times
  // paced kOtosBeginRetryPacingMs apart (~2s worst case) so a slow OTOS boot no
  // longer marks the sensor absent forever (the connected=False root cause).
  static constexpr int kOtosBeginAttempts = 20;
  static constexpr uint32_t kOtosBeginRetryPacingMs = 100;  // [ms]

  // [ms] vendor settle window between each motor's own request and collect
  // inside serviceMotor()
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
  void serviceMotor(NezhaMotor& motor);     // one request->settle->collect pair
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

  // ---- Fiber lifecycle state (DB-008) ----
  bool running_ = false;         // running()'s backing field -- see stop()/start()
  bool stopRequested_ = false;   // the cycle loop's own exit condition
  bool loopExited_ = true;       // true whenever no fiber body is currently
                                  // mid-loop -- starts true (nothing has
                                  // ever been spawned yet); start() clears
                                  // it, the FiberRunner in play sets it back
                                  // via markLoopExited() once the loop
                                  // itself has stopped calling
                                  // runCycleOnce() (see fiber_runner.h);
                                  // stop()'s join polls this before its own
                                  // neutralizeAllMotors() call.

  // The FiberRunner this object uses -- defaults to an internally-owned
  // instance (real: CodalFiberRunner; host: a HostFiberRunner with a 0-cycle
  // budget, i.e. "run the preamble only unless a test injects a different
  // one" -- see setFiberRunner()/fiber_runner.h's own header comment).
  // defaultFiberRunner_ MUST be declared before fiberRunner_ (declaration
  // order is construction order -- the same rule this file's own
  // "Declaration order IS construction order" comment states for the
  // bus_/leaf/ring/handle chain above) so fiberRunner_'s default member
  // initializer can safely take its address.
#ifndef HOST_BUILD
  CodalFiberRunner defaultFiberRunner_;
#else
  HostFiberRunner defaultFiberRunner_{0};
#endif
  FiberRunner* fiberRunner_ = &defaultFiberRunner_;
};

}  // namespace Devices
