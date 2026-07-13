// device_bus_lifecycle_harness.cpp — off-hardware acceptance harness for
// ticket DB-008 (device-bus-tickets.md): exercises the REAL
// Devices::DeviceBus fiber lifecycle (source/devices/device_bus.cpp's
// start()/stop()/running(), runPreamble(), neutralizeAllMotors()) against
// DB-003's HOST_BUILD scripted Devices::I2CBus fake, DB-003's steppable
// Devices::Clock/Sleeper, and the injected Devices::HostFiberRunner seam
// (source/devices/fiber_runner.h) in place of CODAL's create_fiber() — no
// MicroBitI2C, no CODAL, no wall clock, no real sleeps. Real create_fiber()
// is exercised on hardware in DB-009 (not this ticket).
//
// Modeled on device_bus_cycle_harness.cpp (DB-007's own acceptance harness)
// — same hand-rolled assertion plumbing, same scripted-I2CBus fixture
// helpers where they carry over unchanged (baseMotorConfig(), pushEncoderRead(),
// scriptMotorCycle(), testVelGains()). Run by test_device_bus_lifecycle.py,
// which compiles and runs this binary via subprocess. Includes ONLY devices/
// headers plus plain C/C++ stdlib (isolation invariant) — no messages/*.h,
// no hal/*, no source_old/*.
//
// --- The ONE shared-FIFO scripting hazard every scenario below must respect
// ---
// i2c_bus_host.cpp's scripted fake is TWO GLOBAL FIFOs (one for writes, one
// for reads) shared across EVERY device address in the process — NOT one
// FIFO per device. A call whose address doesn't match the FRONT of the
// relevant FIFO still POPS that entry (a "mismatch", counted as an error,
// never a crash) — so pushing scriptWrite()/scriptRead() calls for ONLY the
// device(s) a scenario cares about, while leaving OTHER devices completely
// unscripted, is safe ONLY when those other devices' calls happen while the
// shared queue is genuinely EMPTY (nothing pushed for them yet). Because
// DeviceBus::start() runs the ENTIRE fiber body (preamble for every leaf,
// then the cycle loop) in ONE synchronous call in host builds, a scenario
// that pushes ANY content at all must push it in the EXACT chronological
// order production code will actually call write()/read() — across EVERY
// device runPreamble() touches (motor1_, motor2_, otos_, then color_/line_'s
// retry loop), not just the one leaf the scenario is really about. The
// scenarios below that push scripted content past the very first bus call
// therefore ALWAYS account for motor1_.begin()/motor2_.begin()/otos_.begin()
// first (scriptMotorBeginConverge()/scriptOtosBeginAbsent(), below), even
// when a scenario does not care about those two leaves' own outcome — the
// only exception is scenarioPreambleMarksAbsentDeviceAndSkipsSlot(), which
// deliberately scripts NOTHING at all (see that scenario's own comment).

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include "devices/device_bus.h"

namespace {

// --- Hand-rolled assertion plumbing (mirrors device_bus_cycle_harness.cpp) ---

int g_failureCount = 0;
std::string g_scenarioName;

void beginScenario(const std::string& name) {
  g_scenarioName = name;
  std::printf("--- %s\n", name.c_str());
}

void fail(const std::string& what) {
  ++g_failureCount;
  std::printf("  FAIL [%s]: %s\n", g_scenarioName.c_str(), what.c_str());
}

void checkTrue(bool condition, const std::string& what) {
  if (!condition) fail(what + " -- expected true, got false");
}

void checkFalse(bool condition, const std::string& what) {
  if (condition) fail(what + " -- expected false, got true");
}

void checkFloatEq(float actual, float expected, const std::string& what,
                   float tol = 1e-3f) {
  if (std::fabs(actual - expected) > tol) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %g, got %g", what.c_str(),
                  static_cast<double>(expected), static_cast<double>(actual));
    fail(buf);
  }
}

void checkUintEq(uint32_t actual, uint32_t expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %u, got %u", what.c_str(),
                  static_cast<unsigned>(expected), static_cast<unsigned>(actual));
    fail(buf);
  }
}

// --- Fixture helpers (baseMotorConfig/pushEncoderRead/scriptMotorCycle/
// testVelGains mirror device_bus_cycle_harness.cpp's own identically-named
// helpers) --------------------------------------------------------------

constexpr uint16_t kNezhaWireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);
constexpr uint16_t kOtosWireAddr = static_cast<uint16_t>(Devices::kOtosDeviceAddr << 1);
constexpr uint16_t kAltWireAddr = static_cast<uint16_t>(Devices::kColorDeviceAddrAlt << 1);
constexpr uint16_t kLineWireAddr = static_cast<uint16_t>(Devices::kLineDeviceAddr << 1);

Devices::MotorConfig baseMotorConfig(uint32_t port) {
  Devices::MotorConfig cfg;
  cfg.port = port;
  cfg.fwdSign = 1;
  cfg.wheelTravelCalib = 1.0f;
  cfg.velFiltAlpha = 1.0f;  // no smoothing
  return cfg;
}

// Packs positionMm into the little-endian int32 tenths-of-degree raw
// encoder reading NezhaMotor::collectEncoder()/readEncoderAtomicRaw()
// decodes.
void pushEncoderRead(Devices::I2CBus& bus, float positionMm) {
  int32_t raw = static_cast<int32_t>(std::lround(positionMm * 10.0f));
  uint8_t data[4] = {
      static_cast<uint8_t>(raw & 0xFF),
      static_cast<uint8_t>((raw >> 8) & 0xFF),
      static_cast<uint8_t>((raw >> 16) & 0xFF),
      static_cast<uint8_t>((raw >> 24) & 0xFF),
  };
  bus.scriptRead(kNezhaWireAddr, data, 4, /*status=*/0);
}

// Scripts ONE runCycleOnce()'s worth of MOTOR-ONLY bus traffic. The cycle
// now services each motor in alternation (request1, collect1, request2,
// collect2 -- device_bus.h), but the fake bus is count-only (separate
// write/read FIFOs, content/interleaving unchecked), so this pushes the
// per-kind counts: two requestSample() writes, up to `extraDutySlack`
// possible same-cycle duty writes, then motor1's and motor2's
// collectEncoder() reads carrying the given positions. Does NOT
// account for whatever perceptionSlotStep() does this same cycle (line/
// color/OTOS) -- each scenario below scripts that separately, in call
// order, when its round-robin turn is expected to land on a present leaf.
void scriptMotorCycle(Devices::I2CBus& bus, float position1Mm, float position2Mm,
                       int extraDutySlack) {
  bus.scriptWrite(kNezhaWireAddr, 0);  // motor1 requestSample() 0x46
  bus.scriptWrite(kNezhaWireAddr, 0);  // motor2 requestSample() 0x46
  for (int i = 0; i < extraDutySlack; ++i) {
    bus.scriptWrite(kNezhaWireAddr, 0);  // possible duty write(s) this cycle
  }
  pushEncoderRead(bus, position1Mm);  // motor1 collectEncoder()
  pushEncoderRead(bus, position2Mm);  // motor2 collectEncoder()
}

// scriptMotorBeginConverge() -- scripts a clean, DETERMINISTIC hardReset()
// convergence for ONE NezhaMotor::begin() call: 4 atomic reads (median-of-3
// snapshot + 1 readback), each decoding raw=0 tenths-of-degrees with a REAL
// (status=0) transaction -- 0 is always within hardReset()'s
// kReadbackThreshold of itself, so this deterministically converges on the
// FIRST snapshot (nezha_motor.cpp's own hardReset(), see that function's
// comment) rather than relying on an unscripted mismatch's accidental
// zero-decode. Exactly 4 writes + 4 reads -- see this file's own header
// comment on why every scenario that scripts ANYTHING past the very first
// bus call must account for this pair (motor1_.begin(), then motor2_.begin())
// precisely, even when a scenario does not care about either motor's own
// preamble outcome.
void scriptMotorBeginConverge(Devices::I2CBus& bus) {
  for (int i = 0; i < 4; ++i) {
    bus.scriptWrite(kNezhaWireAddr, 0);
    pushEncoderRead(bus, 0.0f);
  }
}

// scriptOtosBeginAbsent() -- scripts a deterministic ABSENT product-ID probe
// for Otos::begin() (readReg8(kRegProductId): 1 write + 1 read, per
// otos.cpp's own readReg8()), decoding a byte != Otos::kExpectedProductId
// (0x5F) so initialized_ reliably ends up false. Used purely to keep this
// harness's shared script FIFO aligned (see this file's own header
// comment) in scenarios that do not care about OTOS's own preamble outcome.
void scriptOtosBeginAbsent(Devices::I2CBus& bus) {
  bus.scriptWrite(kOtosWireAddr, 0);  // readReg8(kRegProductId)'s reg-select write
  uint8_t wrongId[1] = {0x00};        // != kExpectedProductId -- deterministic absence
  bus.scriptRead(kOtosWireAddr, wrongId, 1, 0);
}

// scriptColorAltDetectSuccess() -- scripts ONE successful ColorSensorLeaf
// AltProbe attempt (beginStep()'s Phase 1): writeReg8(0x81,0xCA) +
// writeReg8(0x80,0x17) + readReg16Alt(0xA4) (2 writes + 2 reads) = 4 writes
// + 2 reads total, decoding a non-zero probe -- found on the FIRST attempt.
// Matches device_bus_cycle_harness.cpp's own primeColorAlt() transaction
// shape (minus that helper's own leaf.beginStep() call -- THIS harness lets
// DeviceBus's own runPreamble() drive beginStep(), not the test).
void scriptColorAltDetectSuccess(Devices::I2CBus& bus) {
  bus.scriptWrite(kAltWireAddr, 0);  // writeReg8(0x81, 0xCA)
  bus.scriptWrite(kAltWireAddr, 0);  // writeReg8(0x80, 0x17)
  bus.scriptWrite(kAltWireAddr, 0);  // readReg16Alt(0xA4) lo write
  bus.scriptWrite(kAltWireAddr, 0);  // readReg16Alt(0xA4) hi write
  uint8_t lo[1] = {0x34};
  uint8_t hi[1] = {0x12};  // probe = 0x1234, non-zero -- found
  bus.scriptRead(kAltWireAddr, lo, 1, 0);
  bus.scriptRead(kAltWireAddr, hi, 1, 0);
}

// scriptLineDetectAttemptFail() -- scripts ONE failed LineSensorLeaf
// beginStep() attempt: readRaw() bails after its FIRST channel write
// returns a non-OK status (line_sensor.cpp's own readRaw(): "if
// (writeStatus != kOk) return false;") -- exactly 1 scripted write, no read.
void scriptLineDetectAttemptFail(Devices::I2CBus& bus) {
  bus.scriptWrite(kLineWireAddr, /*status=*/1);
}

// scriptLineDetectAttemptSuccess() -- scripts ONE successful 4-channel
// LineSensorLeaf readRaw() (used by BOTH beginStep()'s detection probe and
// tick()'s steady-state read -- the same primitive, line_sensor.cpp) --
// exactly 4 writes (channel select) + 4 reads (channel value).
void scriptLineDetectAttemptSuccess(Devices::I2CBus& bus, const uint16_t raw[4]) {
  for (int ch = 0; ch < 4; ++ch) {
    bus.scriptWrite(kLineWireAddr, 0);
    uint8_t val[1] = {static_cast<uint8_t>(raw[ch])};
    bus.scriptRead(kLineWireAddr, val, 1, 0);
  }
}

Devices::Gains testVelGains() {
  // Matches devices_motor_harness.cpp's / device_bus_cycle_harness.cpp's own
  // gain set -- large enough that a fresh error immediately produces an
  // above-deadband duty on the very first tick.
  return Devices::Gains{/*kp=*/0.01f, /*ki=*/0.05f, /*kff=*/0.002f,
                         /*iMax=*/1.0f, /*kaw=*/2.0f};
}

// --- Scenarios ---------------------------------------------------------

// 1. Preamble marks a scripted-absent device not-present and skips its
//    slot. Deliberately scripts NOTHING AT ALL: an entirely empty I2CBus
//    decodes every write/read as a "mismatch" status (never kOk), which is
//    exactly the STATUS-based absence check LineSensorLeaf::beginStep()'s
//    readRaw() uses (line_sensor.cpp: "if (writeStatus != kOk) return
//    false;") -- so line_ reliably exhausts its kMaxAttempts retries and
//    ends up NOT present, with zero scripting and zero shared-FIFO
//    alignment risk (this file's own header comment). color_'s own
//    ApdsProbe fallback happens to use a VALUE-based check instead (en ==
//    0x00), which an unscripted call's zero-initialized default
//    accidentally satisfies -- so this scenario asserts nothing about
//    color_'s outcome, only line_'s.
void scenarioPreambleMarksAbsentDeviceAndSkipsSlot() {
  beginScenario("preamble marks a scripted-absent device not-present and skips its slot");

  Devices::DeviceBus deviceBus(baseMotorConfig(1), baseMotorConfig(2),
                                Devices::OtosConfig{}, Devices::ColorConfig{},
                                Devices::LineConfig{});

  deviceBus.start();  // default FiberRunner (host: preamble only, 0 cycles)

  checkTrue(deviceBus.lineLeaf().detectDone(),
            "line's preamble retry state machine reached a terminal state");
  checkFalse(deviceBus.lineLeaf().present(),
             "line marked NOT present after the preamble exhausts every retry");
  checkFalse(deviceBus.line().connected(),
             "the line HANDLE also reports not connected");

  uint32_t lineTxnBefore = deviceBus.bus().txnCount(Devices::kLineDeviceAddr);

  // Step several cycles DIRECTLY (runCycleOnce() is DB-007's own public,
  // host-steppable seam) -- an absent leaf's tick() is a structural no-op
  // (DB-004 through DB-006's own present()/initialized_ gate), so line's
  // address should see ZERO further traffic across every round-robin turn.
  for (int i = 0; i < 6; ++i) {
    deviceBus.runCycleOnce();
  }

  checkUintEq(deviceBus.bus().txnCount(Devices::kLineDeviceAddr), lineTxnBefore,
              "line's bus address sees ZERO further traffic on its round-robin turn -- its slot is skipped");

  deviceBus.stop();
}

// 2. running() is false before start() / true after start() / false after
//    stop(). Nothing scripted -- pure lifecycle-flag mechanics, unrelated
//    to bus content (every call harmlessly mismatches against the empty
//    queue).
void scenarioRunningLifecycleTransitions() {
  beginScenario("running() is false before start() / true after start() / false after stop()");

  Devices::DeviceBus deviceBus(baseMotorConfig(1), baseMotorConfig(2),
                                Devices::OtosConfig{}, Devices::ColorConfig{},
                                Devices::LineConfig{});

  checkFalse(deviceBus.running(), "running() false before start()");

  deviceBus.start();
  checkTrue(deviceBus.running(), "running() true immediately after start() returns");

  deviceBus.stop();
  checkFalse(deviceBus.running(), "running() false after stop()");
}

// 3. stop() emits a neutral duty write for every motor as its last bus
//    action. motor1 is driven to a genuine nonzero duty (PID chasing a
//    staged setVelocity(), matching device_bus_cycle_harness.cpp's own
//    scenarioStagedVelocityReachesArmoredWriteWithinOneCycle() precedent:
//    one cycle is enough) via an injected HostFiberRunner(1) that runs
//    exactly one cycle immediately after the preamble; motor2 never
//    receives any target at all (mode_ stays None -- exercises the "first
//    write ever" -128 sentinel path for the neutral write, not just the
//    "was already driven" path).
void scenarioStopNeutralizesEveryMotorAsLastBusAction() {
  beginScenario("stop() emits a neutral duty write for every motor as its last bus action");

  Devices::MotorConfig cfg1 = baseMotorConfig(1);
  cfg1.velGains = testVelGains();
  cfg1.velDeadband = 5.0f;
  Devices::DeviceBus deviceBus(cfg1, baseMotorConfig(2), Devices::OtosConfig{},
                                Devices::ColorConfig{}, Devices::LineConfig{});

  // Nonzero clock baseline -- see device_bus_cycle_harness.cpp's own
  // scenarioStagedVelocityReachesArmoredWriteWithinOneCycle() comment: a
  // first write at EXACTLY nowUs==0 hits writeRawDuty()'s write-rate-limit
  // throttle "0us since the (never-happened) last write" coincidence, and
  // Motor::setVelocity() stamps velocityStagedUs_ from the clock at call
  // time, so staging before advancing the clock would make the very first
  // drainStagedInputs() see an already-stale target.
  deviceBus.clock().setMicros(1000000);
  deviceBus.motor(1).setVelocity(300.0f);  // staged BEFORE start() -- touches no bus

  Devices::HostFiberRunner runner(/*maxCycles=*/1);
  deviceBus.setFiberRunner(runner);

  // Preamble: motor1_/motor2_/otos_ accounted for deterministically (this
  // scenario does not care about their outcome); color_/line_ both
  // detected present on their FIRST attempt (cheap, deterministic) so
  // neither drags this scenario into a multi-attempt retry script.
  scriptMotorBeginConverge(deviceBus.bus());   // motor1_.begin()
  scriptMotorBeginConverge(deviceBus.bus());   // motor2_.begin()
  scriptOtosBeginAbsent(deviceBus.bus());       // otos_.begin()
  scriptColorAltDetectSuccess(deviceBus.bus()); // color_.beginStep() tick 1
  uint16_t lineDetectRaw[4] = {10, 20, 30, 40};
  scriptLineDetectAttemptSuccess(deviceBus.bus(), lineDetectRaw);  // line_.beginStep() tick 1

  // The ONE injected cycle: perceptionSlot_ starts at Line (device_bus.h),
  // so THIS cycle's round-robin turn lands on line_, now present -- its
  // tick() issues one more real (4W+4R) read (readDue() is true on line_'s
  // very first tick() call, independent of when beginStep() last ran).
  scriptMotorCycle(deviceBus.bus(), 0.0f, 0.0f, /*extraDutySlack=*/1);
  uint16_t lineTickRaw[4] = {11, 21, 31, 41};
  scriptLineDetectAttemptSuccess(deviceBus.bus(), lineTickRaw);

  checkFalse(deviceBus.running(), "precondition: not running before start()");
  deviceBus.start();
  checkTrue(deviceBus.running(), "running() true after start()");

  checkUintEq(deviceBus.bus().errCount(Devices::kNezhaDeviceAddr), 0,
              "the driving cycle ran cleanly (no script under-run)");
  checkTrue(std::fabs(deviceBus.motorLeaf(1).appliedDuty()) > 0.03f,
            "precondition: motor1 actively driving (nonzero duty) before stop()");
  checkFloatEq(deviceBus.motorLeaf(2).appliedDuty(), 0.0f,
               "precondition: motor2 was never given a target -- still at its -128 'no write yet' sentinel");

  uint32_t nezhaTxnBefore = deviceBus.bus().txnCount(Devices::kNezhaDeviceAddr);
  scriptMotorCycle(deviceBus.bus(), 6.0f, 0.0f, /*extraDutySlack=*/2);  // stop()'s own neutralize pass

  deviceBus.stop();

  checkFalse(deviceBus.running(), "running() false after stop()");
  checkUintEq(deviceBus.bus().errCount(Devices::kNezhaDeviceAddr), 0,
              "no script under-run across stop()'s neutralize pass");
  checkUintEq(deviceBus.bus().txnCount(Devices::kNezhaDeviceAddr) - nezhaTxnBefore, 6u,
              "stop()'s neutralize pass performs exactly 2 requests + 2 collects + 2 neutral writes (one per motor)");
  checkFloatEq(deviceBus.motorLeaf(1).appliedDuty(), 0.0f,
               "motor1's LAST bus action is a neutral (0 duty) write -- replacing its earlier nonzero duty");
  checkFloatEq(deviceBus.motorLeaf(2).appliedDuty(), 0.0f,
               "motor2's LAST bus action is a neutral (0 duty) write -- its very FIRST write ever");
}

// 4. A preamble device-begin retry that eventually succeeds does not block
//    the loop from starting. line_ fails its first two beginStep()
//    attempts, succeeds on the third -- the preamble's bounded retry-pacing
//    loop (device_bus.cpp's runPreamble()) carries it through all three
//    without ever blocking, and an injected HostFiberRunner(1) proves the
//    cycle loop then runs cleanly immediately afterward, in the SAME
//    start() call.
void scenarioPreambleRetryEventuallySucceedsDoesNotBlockLoopStarting() {
  beginScenario(
      "a preamble device-begin retry that eventually succeeds does not "
      "block the loop from starting");

  Devices::DeviceBus deviceBus(baseMotorConfig(1), baseMotorConfig(2),
                                Devices::OtosConfig{}, Devices::ColorConfig{},
                                Devices::LineConfig{});

  scriptMotorBeginConverge(deviceBus.bus());   // motor1_.begin()
  scriptMotorBeginConverge(deviceBus.bus());   // motor2_.begin()
  scriptOtosBeginAbsent(deviceBus.bus());       // otos_.begin()
  scriptColorAltDetectSuccess(deviceBus.bus()); // color_ present on its FIRST preamble tick
  scriptLineDetectAttemptFail(deviceBus.bus()); // line_ attempt 1 -- fails
  scriptLineDetectAttemptFail(deviceBus.bus()); // line_ attempt 2 -- fails
  uint16_t lineDetectRaw[4] = {10, 20, 30, 40};
  scriptLineDetectAttemptSuccess(deviceBus.bus(), lineDetectRaw);  // line_ attempt 3 -- succeeds

  // ONE cycle immediately after the preamble, in the SAME start() call
  // (HostFiberRunner(1), injected below) -- proves the loop is not blocked
  // by line_'s multi-attempt retry. perceptionSlot_ starts at Line, so
  // this cycle's round-robin turn lands on line_, now present.
  scriptMotorCycle(deviceBus.bus(), 0.0f, 0.0f, /*extraDutySlack=*/0);
  uint16_t lineTickRaw[4] = {11, 21, 31, 41};
  scriptLineDetectAttemptSuccess(deviceBus.bus(), lineTickRaw);

  Devices::HostFiberRunner runner(/*maxCycles=*/1);
  deviceBus.setFiberRunner(runner);

  checkFalse(deviceBus.running(), "precondition: not running before start()");
  checkUintEq(deviceBus.bus().txnCount(Devices::kNezhaDeviceAddr), 0u,
              "precondition: no bus traffic at all before start() runs (scripting only queues, never executes)");
  deviceBus.start();  // runs the ENTIRE preamble AND the one injected cycle,
                       // synchronously, in this one call (HostFiberRunner) --
                       // there is no way to observe "after preamble, before
                       // cycle" separately from outside this call.

  checkTrue(deviceBus.running(), "running() true immediately after start() returns");
  checkTrue(deviceBus.colorLeaf().present(), "color detected on its first attempt");
  checkTrue(deviceBus.lineLeaf().detectDone(),
            "line's preamble retry state machine reached a terminal state");
  checkTrue(deviceBus.lineLeaf().present(),
            "line eventually detected present after two failed retries -- "
            "the retry that eventually succeeds");
  checkUintEq(deviceBus.bus().errCount(Devices::kLineDeviceAddr), 2,
              "exactly the two SCRIPTED failed attempts counted as errors on line's address -- no extras");

  checkUintEq(deviceBus.bus().errCount(Devices::kNezhaDeviceAddr), 0,
              "the injected cycle ran cleanly -- line's multi-attempt preamble "
              "retry did not desync or block it");
  // 16 (motor1_.begin() + motor2_.begin(), scriptMotorBeginConverge() x2,
  // 4W+4R each) + 4 (the one injected cycle's 2 requests + 2 collects) == 20
  // -- the loop DID start and run its full, clean schedule immediately
  // after the preamble finished, not just "started and stalled."
  checkUintEq(deviceBus.bus().txnCount(Devices::kNezhaDeviceAddr), 20u,
              "the preamble's own motor traffic (16) PLUS one full cycle's "
              "worth (4) both landed -- the loop DID start, unblocked by "
              "line's multi-attempt retry");

  deviceBus.stop();
  checkFalse(deviceBus.running(), "running() false after stop()");
}

}  // namespace

int main() {
  scenarioPreambleMarksAbsentDeviceAndSkipsSlot();
  scenarioRunningLifecycleTransitions();
  scenarioStopNeutralizesEveryMotorAsLastBusAction();
  scenarioPreambleRetryEventuallySucceedsDoesNotBlockLoopStarting();

  if (g_failureCount == 0) {
    std::printf("OK: all device_bus lifecycle scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the device_bus lifecycle scenarios\n",
              g_failureCount);
  return 1;
}
