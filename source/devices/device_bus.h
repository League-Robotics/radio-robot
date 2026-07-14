// device_bus.h — Devices::DeviceBus: the subsystem root. Owns the I2C bus,
// every device leaf, one MeasurementRing per stream, and the Clock/Sleeper
// time seam; hands out the handle classes (handles.h) that are the ONLY way
// the rest of the firmware ever touches a device.
//
// Tickets DB-007 and DB-008 (device-bus-tickets.md), narrowed by sprint 102
// ticket 005 (single-loop firmware rebuild): the fiber-owned lifecycle
// (start()/stop()/running(), the pluggable FiberRunner injection seam) is
// REMOVED — sprint 102 deletes the background-fiber concurrency model
// wholesale (fiber_runner.h, devices/bringup_main.cpp) in favor of ONE
// single foreground loop that calls runPreamble() once at boot and
// runCycleOnce() every pass directly, with no fiber, no CODAL create_fiber()
// call anywhere in this subsystem. runPreamble()/neutralizeAllMotors() are
// therefore public now (previously private, reached only through the
// now-deleted FiberRunner friend seam) — a single-loop caller invokes them
// directly, the same way it already calls runCycleOnce().
//
// --- Scope: DB-007 vs DB-008 ---
// DB-007 delivered the ROOT OBJECT and the straight-line CYCLE BODY
// (runCycleOnce()) — the "host-steppable seam" device-bus-tickets.md's
// header resolves the issue's own open cycle-testability question into:
// "the for(;;) fiber body is factored into a plain runCycleOnce() method the
// host harness steps deterministically". DB-008 added the detection preamble
// (issue "The fiber and its cycle", step 1: power-settle wait, per-device
// begin()/beginStep() retries, absent-device slot-skipping) — see
// runPreamble()'s own declaration comment below. A host test that wants a
// leaf primed WITHOUT exercising runPreamble() (e.g. DB-007's own
// device_bus_cycle_harness.cpp) may still drive begin()/beginStep() directly
// through the HOST_BUILD test-seam accessors below (motorLeaf()/otosLeaf()/
// colorLeaf()/lineLeaf()) — that ad hoc path and the real preamble both
// remain valid, independent ways to get a leaf detected in a host test.
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
// 1. This object (via runCycleOnce(), called every pass of the single
//    foreground loop — see this file's own header comment) is the ONLY
//    writer of every ring and the ONLY bus toucher; handle setters
//    (handles.h) are the only writers of staged-input state, and every one
//    of them is a plain, yield-free store (see handles.h's own design
//    notes for Motor/Odometer).
// 2. No yield inside publishSamples()/drainStagedInputs()/any handle
//    sample-copy — all plain struct stores/copies (measurement_ring.h's own
//    publish()/latest()/sample()/bracket() are already yield-free by
//    construction).
// 3. Handles are single-foreground-loop API only — nothing here is ISR-safe.
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

  // OTOS probe diagnostics (101-001): a read-only snapshot of the OTOS leaf's
  // detect state plus this bus's transaction stats for the OTOS address, for
  // bench triage of the DeviceBus connected=False condition. Reads counters
  // only -- issues NO I2C traffic, so it is safe to call any time from the
  // single foreground loop.
  struct OtosProbeDiag {
    bool connected;
    bool present;
    uint32_t txnCount;   // I2C transactions to 0x17 (0 => probe never ran)
    uint32_t errCount;   // of those, how many errored (NAK/bus)
    int lastErr;         // last error code for 0x17
    uint8_t lastProbeId; // last product-ID byte begin() read (0x5F = correct)
  };
  OtosProbeDiag otosProbeDiag() const;

  // --- Single-loop lifecycle (narrowed by sprint 102 ticket 005 -- see this
  // file's own header comment) ---
  //
  // runPreamble() -- the detection preamble (power-settle wait, per-device
  // begin()/beginStep() retries, absent-device slot-skipping). Call ONCE at
  // boot, directly from the single foreground loop, before the first
  // runCycleOnce() call. Previously private + reached only through the
  // now-deleted FiberRunner friend seam; public now that there is no fiber
  // to hide it behind. See this method's own definition comment
  // (device_bus.cpp) for the full contract.
  void runPreamble();

  // runCycleOnce() -- the host-steppable cycle core; see this file's own
  // header comment for the exact schedule. Call every pass of the single
  // foreground loop, directly -- no fiber, no FiberRunner indirection.
  void runCycleOnce();

  // neutralizeAllMotors() -- a real, synchronous bus write per motor,
  // unconditionally each motor's LAST bus action from this call. Call
  // directly on shutdown/e-stop; previously stop()'s own private epilogue,
  // public now for the same reason runPreamble() is (no fiber, no owning
  // stop() method left to call it from). See this method's own definition
  // comment (device_bus.cpp) for the full contract.
  void neutralizeAllMotors();

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
};

}  // namespace Devices
