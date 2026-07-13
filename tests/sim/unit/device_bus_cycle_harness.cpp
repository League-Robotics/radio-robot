// device_bus_cycle_harness.cpp — off-hardware acceptance harness for ticket
// DB-007 (device-bus-tickets.md): exercises the REAL Devices::DeviceBus root
// (source/devices/device_bus.cpp) — its runCycleOnce() schedule, the handle
// classes (source/devices/handles.h), and the measurement rings — against
// DB-003's HOST_BUILD scripted Devices::I2CBus fake and DB-003's steppable
// Devices::Clock/Sleeper, all real leaves (NezhaMotor, Otos, ColorSensorLeaf,
// LineSensorLeaf). No MicroBitI2C, no CODAL, no wall clock, no real sleeps.
//
// Modeled on devices_motor_harness.cpp (scripted request/collect pairing
// technique) and devices_otos_harness.cpp / devices_sensors_harness.cpp
// (begin()/beginStep() priming technique) — this harness composes all three
// rather than introducing a new scripting style. Hand-rolled assertions,
// PASS/FAIL per scenario, nonzero exit on any failure. Run by
// test_device_bus_cycle.py, which compiles and runs this binary via
// subprocess. Includes ONLY devices/ headers plus plain C/C++ stdlib
// (isolation invariant) — no messages/*.h, no hal/*, no source_old/*.
//
// --- Scripting model recap (i2c_bus_host.cpp) ---
// scriptWrite()/scriptRead() are TWO SEPARATE, address-agnostic-within-a-
// device FIFOs: content is never checked (only address, in call order), so
// which "logical" write consumes which queued entry does not matter, only
// the COUNT does (devices_motor_harness.cpp's own scriptEncoderRequestCollect()
// comment establishes this precedent; this file's scriptMotorCycle() below
// follows it exactly). Both of motor1_/motor2_ share ONE wire address
// (0x10 << 1) — the motorId lives in the payload, which this fake never
// inspects — so their writes/reads share one FIFO each; because the fake
// checks only the per-kind COUNT (never content or write-vs-read
// interleaving), the push order here need not mirror the consumption order.
// runCycleOnce()'s actual order is now ALTERNATING (device_bus.h's schedule):
// request1, collect1, [duty1?], request2, collect2, [duty2?].
//
// --- How this harness proves the 093 hazard is structurally absent ---
// runCycleOnce() services each motor with its OWN serviceMotor() (request ->
// settle -> collect), motor1 fully before motor2 begins — there is no code
// path by which a duty write (which only ever originates inside
// NezhaMotor::tick(), only ever called from serviceMotor() AFTER that same
// motor's collect) can land between any one motor's own request and that
// SAME motor's own collect. This harness demonstrates the OBSERVABLE
// consequence of that
// structural guarantee two ways: (1) scenarioScheduleOrderDeterministicBaseline()
// proves the exact, minimal per-cycle transaction count (2 requests + 2
// collects, zero duty) when no motion is staged; (2) scenario
// NoDutyWriteBetweenRequestAndCollect() drives BOTH motors toward a real PID
// target across many cycles (so real duty writes DO land) and proves the
// scripted request/collect FIFO pairing NEVER desyncs (zero errCount, exact
// position decode every cycle) — a genuine hazard (a duty write stealing a
// request's queue slot) would eventually manifest as a FIFO under-run or a
// garbled decode; neither ever happens because the code has no path to
// produce one.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include "devices/device_bus.h"

namespace {

// --- Hand-rolled assertion plumbing (mirrors devices_motor_harness.cpp /
// devices_sensors_harness.cpp) ---

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

void checkU64Eq(uint64_t actual, uint64_t expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %llu, got %llu", what.c_str(),
                  static_cast<unsigned long long>(expected),
                  static_cast<unsigned long long>(actual));
    fail(buf);
  }
}

// --- Fixture helpers -------------------------------------------------------

constexpr uint16_t kNezhaWireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);
constexpr uint16_t kOtosWireAddr = static_cast<uint16_t>(Devices::kOtosDeviceAddr << 1);
constexpr uint16_t kAltWireAddr = static_cast<uint16_t>(Devices::kColorDeviceAddrAlt << 1);
constexpr uint16_t kLineWireAddr = static_cast<uint16_t>(Devices::kLineDeviceAddr << 1);

Devices::MotorConfig baseMotorConfig(uint32_t port) {
  Devices::MotorConfig cfg;
  cfg.port = port;
  cfg.fwdSign = 1;
  cfg.wheelTravelCalib = 1.0f;
  cfg.velFiltAlpha = 1.0f;  // no smoothing -- velocity() reflects each tick's raw difference-quotient exactly
  return cfg;
}

// Packs positionMm into the little-endian int32 tenths-of-degree raw
// encoder reading NezhaMotor::collectEncoder() decodes -- mirrors
// devices_motor_harness.cpp's scriptEncoderRequestCollect() encode step.
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

// Scripts ONE runCycleOnce()'s worth of motor bus traffic: motor1's
// requestSample() write, motor2's requestSample() write, up to
// `extraDutySlack` possible same-cycle duty writes (write-on-change/
// throttle-gated -- may or may not actually land; unconsumed slack is
// harmless, drained by a later cycle -- devices_motor_harness.cpp's own
// established precedent), then motor1's and motor2's collectEncoder() reads
// carrying the given positions. Both motors share ONE wire address, so
// writes/reads are pushed as one combined FIFO push per kind -- the fake is
// count-only, so this push order need not mirror runCycleOnce()'s actual
// (now alternating: request1, collect1, request2, collect2) call order; only
// the per-kind counts (2 requests + slack writes; 2 collect reads) must match.
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

// primeOtos() -- scripts Otos::begin()'s full successful-detect sequence (7
// writes + 1 read -- devices_otos_harness.cpp's own kBeginTxnCount=8 comment)
// then calls it. Default OtosConfig (zero offsets, unit scales) so this
// harness never needs to reason about the lever-arm/mounting-yaw transform --
// DB-005's own job, not this ticket's.
void primeOtos(Devices::I2CBus& bus, Devices::Otos& leaf) {
  bus.scriptWrite(kOtosWireAddr, 0);  // readReg8(kRegProductId)'s reg-select write
  uint8_t productId[1] = {0x5F};      // Otos::kExpectedProductId (otos.h)
  bus.scriptRead(kOtosWireAddr, productId, 1, 0);
  for (int i = 0; i < 6; ++i) {
    bus.scriptWrite(kOtosWireAddr, 0);  // init()x3 + setLinearScalar + setAngularScalar + writeXYH(zero)
  }
  leaf.begin();
}

// Scripts one Otos::readPositionVelocity()-shaped 12-byte burst read (device
// units -- LSB-scaled, not mm/rad; see otos.h's kPosMmPerLsb/kHdgRadPerLsb).
// This harness never asserts on the DECODED pose value -- only that a
// publish happened, at the expected stamp -- so the actual numbers are
// arbitrary/don't-care beyond being a valid successful read.
void scriptOtosBurst(Devices::I2CBus& bus, int16_t x, int16_t y, int16_t h,
                      int16_t vx, int16_t vy, int16_t vh) {
  bus.scriptWrite(kOtosWireAddr, 0);  // register-select write
  uint8_t raw[12] = {
      static_cast<uint8_t>(x & 0xFF), static_cast<uint8_t>((x >> 8) & 0xFF),
      static_cast<uint8_t>(y & 0xFF), static_cast<uint8_t>((y >> 8) & 0xFF),
      static_cast<uint8_t>(h & 0xFF), static_cast<uint8_t>((h >> 8) & 0xFF),
      static_cast<uint8_t>(vx & 0xFF), static_cast<uint8_t>((vx >> 8) & 0xFF),
      static_cast<uint8_t>(vy & 0xFF), static_cast<uint8_t>((vy >> 8) & 0xFF),
      static_cast<uint8_t>(vh & 0xFF), static_cast<uint8_t>((vh >> 8) & 0xFF),
  };
  bus.scriptRead(kOtosWireAddr, raw, 12, 0);
}

// primeLine() -- scripts LineSensorLeaf::beginStep()'s single successful
// readRaw() detection probe (4 write/read pairs -- devices_sensors_harness.cpp's
// own precedent) then calls it once (detection succeeds on the first
// attempt, so one beginStep() call is always enough here).
void primeLine(Devices::I2CBus& bus, Devices::LineSensorLeaf& leaf, uint64_t nowUs) {
  for (int ch = 0; ch < 4; ++ch) {
    bus.scriptWrite(kLineWireAddr, 0);
    uint8_t val[1] = {5};
    bus.scriptRead(kLineWireAddr, val, 1, 0);
  }
  leaf.beginStep(nowUs);
}

void scriptLineFrame(Devices::I2CBus& bus, const uint16_t raw[4]) {
  for (int ch = 0; ch < 4; ++ch) {
    bus.scriptWrite(kLineWireAddr, 0);
    uint8_t val[1] = {static_cast<uint8_t>(raw[ch])};
    bus.scriptRead(kLineWireAddr, val, 1, 0);
  }
}

// primeColorAlt() -- scripts ColorSensorLeaf::beginStep()'s ALT-chip
// detection succeeding on the FIRST attempt with a non-zero probe value
// (devices_sensors_harness.cpp's own note: absent-device modeling needs an
// EXPLICIT non-zero/zero register value scripted, never an unscripted call
// left to the fake's zero-init default -- not needed here since this
// harness always wants a PRESENT chip, but the same "always script the
// decoded value explicitly" discipline applies).
void primeColorAlt(Devices::I2CBus& bus, Devices::ColorSensorLeaf& leaf, uint64_t nowUs) {
  bus.scriptWrite(kAltWireAddr, 0);  // writeReg8(0x81, 0xCA)
  bus.scriptWrite(kAltWireAddr, 0);  // writeReg8(0x80, 0x17)
  bus.scriptWrite(kAltWireAddr, 0);  // readReg16Alt(0xA4) lo write
  bus.scriptWrite(kAltWireAddr, 0);  // readReg16Alt(0xA4) hi write
  uint8_t lo[1] = {0x34};
  uint8_t hi[1] = {0x12};  // probe = 0x1234, non-zero -- found
  bus.scriptRead(kAltWireAddr, lo, 1, 0);
  bus.scriptRead(kAltWireAddr, hi, 1, 0);
  leaf.beginStep(nowUs);
}

// Scripts one steady-state ALT color frame: 0xA6 probe (non-zero -- ready)
// then 0xA0/0xA2/0xA4 (r/g/b) -- 4 readReg16Alt()-shaped calls, each 2
// writes + 2 reads (8 writes + 8 reads total). Mirrors
// devices_sensors_harness.cpp's scriptAltReg16()/scenario 1 frame script.
void scriptColorAltFrame(Devices::I2CBus& bus, uint16_t c, uint16_t r,
                          uint16_t g, uint16_t b) {
  auto push16 = [&bus](uint16_t value) {
    bus.scriptWrite(kAltWireAddr, 0);
    bus.scriptWrite(kAltWireAddr, 0);
    uint8_t lo[1] = {static_cast<uint8_t>(value & 0xFF)};
    uint8_t hi[1] = {static_cast<uint8_t>((value >> 8) & 0xFF)};
    bus.scriptRead(kAltWireAddr, lo, 1, 0);
    bus.scriptRead(kAltWireAddr, hi, 1, 0);
  };
  push16(c);
  push16(r);
  push16(g);
  push16(b);
}

Devices::Gains testVelGains() {
  // Matches devices_motor_harness.cpp's own scenarioPidOnChasesVelocityTarget()
  // gain set -- large enough that a fresh error immediately produces an
  // above-deadband duty on the very first tick.
  return Devices::Gains{/*kp=*/0.01f, /*ki=*/0.05f, /*kff=*/0.002f,
                         /*iMax=*/1.0f, /*kaw=*/2.0f};
}

// --- Scenarios ---------------------------------------------------------

// 1. Deterministic no-motion baseline: exactly 2 requests + 2 collects per
//    cycle, zero duty writes (mode_ never commanded on either motor), proves
//    the minimal schedule's transaction COUNT is exactly as documented.
void scenarioScheduleOrderDeterministicBaseline() {
  beginScenario(
      "runCycleOnce(): request->settle->collect, exact transaction count per "
      "cycle (no motion staged)");

  Devices::I2CBus::setClock(0);
  Devices::DeviceBus deviceBus(baseMotorConfig(1), baseMotorConfig(2),
                                Devices::OtosConfig{}, Devices::ColorConfig{},
                                Devices::LineConfig{});

  const int kCycles = 3;
  float pos1 = 0.0f, pos2 = 0.0f;
  uint64_t nowUs = 0;
  for (int i = 0; i < kCycles; ++i) {
    deviceBus.clock().setMicros(nowUs);
    uint32_t before = deviceBus.bus().txnCount(Devices::kNezhaDeviceAddr);
    uint32_t sleepsBefore = static_cast<uint32_t>(deviceBus.sleeper().sleepCount());

    scriptMotorCycle(deviceBus.bus(), pos1, pos2, /*extraDutySlack=*/0);
    deviceBus.runCycleOnce();

    uint32_t after = deviceBus.bus().txnCount(Devices::kNezhaDeviceAddr);
    checkUintEq(after - before, 4u,
                "exactly 2 requests + 2 collects, zero duty writes, this cycle");
    checkUintEq(deviceBus.bus().errCount(Devices::kNezhaDeviceAddr), 0,
                "no script under-run/mismatch");
    checkFloatEq(deviceBus.motor(1).latest().value.position, pos1,
                 "motor1 position reflects this cycle's collected sample");
    checkFloatEq(deviceBus.motor(2).latest().value.position, pos2,
                 "motor2 position reflects this cycle's collected sample");

    // Exactly THREE sleeps per cycle: one settle sleep INSIDE each motor's
    // own serviceMotor() (between that motor's own request and its own
    // collect -- the brick holds only ONE pending 0x46 request, so the two
    // motors cannot share a settle window; see device_bus.h's alternating
    // note), then the single pace sleep. The settle sleep's PLACEMENT between
    // a motor's request and its collect is a structural property of
    // runCycleOnce()'s fixed call order (device_bus.cpp); this harness
    // verifies the observable consequence available through the Sleeper
    // seam: exactly three sleepMillis() calls per cycle, the last of which is
    // the pace-sleep (kCyclePaceMs = 12, device_bus.h).
    uint32_t sleepsAfter = static_cast<uint32_t>(deviceBus.sleeper().sleepCount());
    checkUintEq(sleepsAfter - sleepsBefore, 3u,
                "exactly three sleeps this cycle (settle x2, one per motor, then pace)");
    checkUintEq(deviceBus.sleeper().lastSleepMillis(), 12u,
                "the cycle's LAST sleep is the pace-sleep (kCyclePaceMs)");

    pos1 += 5.0f;
    pos2 += 3.0f;
    nowUs += 20000;
  }
}

// 2. Active-drive, many-cycle stress: proves the scripted request/collect
//    FIFO pairing never desyncs while BOTH motors are actively chasing a PID
//    target (so real duty writes DO land, unpredictably, across the run) --
//    the observable proof that the 093 hazard cannot occur (see this file's
//    own header comment for the full argument).
void scenarioNoDutyWriteBetweenRequestAndCollect() {
  beginScenario(
      "runCycleOnce(): a duty write can never land between a motor's own "
      "request and its own collect (093 hazard structurally absent)");

  Devices::I2CBus::setClock(0);
  Devices::MotorConfig cfg1 = baseMotorConfig(1);
  cfg1.velGains = testVelGains();
  cfg1.velDeadband = 5.0f;
  Devices::MotorConfig cfg2 = baseMotorConfig(2);
  cfg2.velGains = testVelGains();
  cfg2.velDeadband = 5.0f;

  Devices::DeviceBus deviceBus(cfg1, cfg2, Devices::OtosConfig{},
                                Devices::ColorConfig{}, Devices::LineConfig{});
  deviceBus.motor(1).setVelocity(300.0f);
  deviceBus.motor(2).setVelocity(-250.0f);

  const int kCycles = 10;
  float pos1 = 0.0f, pos2 = 0.0f;
  uint64_t nowUs = 0;
  for (int i = 0; i < kCycles; ++i) {
    deviceBus.clock().setMicros(nowUs);
    scriptMotorCycle(deviceBus.bus(), pos1, pos2, /*extraDutySlack=*/2);
    deviceBus.runCycleOnce();

    char label[64];
    std::snprintf(label, sizeof(label), "cycle %d", i);
    checkUintEq(deviceBus.bus().errCount(Devices::kNezhaDeviceAddr), 0,
                std::string("no script under-run/mismatch through active-drive ") + label);
    checkFloatEq(deviceBus.motor(1).latest().value.position, pos1,
                 std::string("motor1 position still decodes exactly (read-FIFO alignment held), ") + label);
    checkFloatEq(deviceBus.motor(2).latest().value.position, pos2,
                 std::string("motor2 position still decodes exactly (read-FIFO alignment held), ") + label);
    checkTrue(deviceBus.motor(1).connected(), std::string("motor1 stays connected, ") + label);
    checkTrue(deviceBus.motor(2).connected(), std::string("motor2 stays connected, ") + label);

    pos1 += 6.0f;   // well under kMaxPlausibleStepSpeed at 20ms spacing
    pos2 -= 5.0f;
    nowUs += 20000;
  }
}

// 3. A staged setVelocity() reaches an armored duty write within one cycle.
void scenarioStagedVelocityReachesArmoredWriteWithinOneCycle() {
  beginScenario("a staged setVelocity() reaches an armored duty write within one cycle");

  Devices::I2CBus::setClock(0);
  Devices::MotorConfig cfg1 = baseMotorConfig(1);
  cfg1.velGains = testVelGains();
  cfg1.velDeadband = 5.0f;
  Devices::DeviceBus deviceBus(cfg1, baseMotorConfig(2), Devices::OtosConfig{},
                                Devices::ColorConfig{}, Devices::LineConfig{});

  checkFalse(deviceBus.motor(1).latest().valid,
             "precondition: no cycle has run yet -- ring has no published sample");

  // Start the clock at a nonzero baseline BEFORE staging setVelocity() --
  // two independent reasons: (1) Motor::setVelocity() stamps
  // velocityStagedUs_ from the clock at CALL time (handles.h), so staging it
  // before advancing the clock would make the very first drainStagedInputs()
  // see a target that is ALREADY older than kVelocityStaleUs and neutralize
  // it before this scenario ever gets to observe a real duty write; (2)
  // NezhaMotor::writeRawDuty()'s write-rate-limit throttle compares against
  // lastWriteTimeUs_'s own zero-init default, so a first write landing at
  // EXACTLY nowUs==0 would read as "0us since the (never-happened) last
  // write" and get throttled away -- devices_motor_harness.cpp's own
  // scenarios avoid the identical coincidence via `I2CBus::setClock(1000000)`;
  // this is the same fix, applied to the Devices::Clock instance DeviceBus
  // actually times its cycles from.
  deviceBus.clock().setMicros(1000000);
  deviceBus.motor(1).setVelocity(300.0f);  // staged -- this call itself touches no bus

  scriptMotorCycle(deviceBus.bus(), 0.0f, 0.0f, /*extraDutySlack=*/2);
  deviceBus.runCycleOnce();

  checkUintEq(deviceBus.bus().errCount(Devices::kNezhaDeviceAddr), 0, "no script under-run");
  Devices::Sample<Devices::MotorReading> latest = deviceBus.motor(1).latest();
  checkTrue(latest.valid, "one cycle published a sample");
  checkTrue(std::fabs(latest.value.appliedDuty) > 0.03f,
            "the staged velocity target reached an armored (above-deadband) duty write within this one cycle");
}

// 4. publishSamples() populates each ring with a monotonic [us] stamp.
void scenarioPublishSamplesMonotonicStamps() {
  beginScenario("publishSamples() populates each ring with a monotonic [us] stamp");

  Devices::I2CBus::setClock(0);
  Devices::DeviceBus deviceBus(baseMotorConfig(1), baseMotorConfig(2),
                                Devices::OtosConfig{}, Devices::ColorConfig{},
                                Devices::LineConfig{});

  const int kCycles = 5;
  uint64_t nowUs = 1000;
  const uint64_t dt = 15000;
  uint64_t lastNowUs = nowUs;
  for (int i = 0; i < kCycles; ++i) {
    deviceBus.clock().setMicros(nowUs);
    scriptMotorCycle(deviceBus.bus(), static_cast<float>(i), static_cast<float>(i), 0);
    deviceBus.runCycleOnce();
    lastNowUs = nowUs;
    nowUs += dt;
  }

  uint64_t prevStamp = UINT64_MAX;
  for (uint8_t age = 0; age < 5; ++age) {
    Devices::Sample<Devices::MotorReading> s = deviceBus.motor(1).sample(age);
    checkTrue(s.valid, "motor1 ring has a published sample at this age");
    checkTrue(s.stamp < prevStamp,
              "motor1 ring stamps strictly decrease as age increases (monotonic publish order)");
    prevStamp = s.stamp;
  }

  checkU64Eq(deviceBus.motor(1).updatedAt(), lastNowUs,
             "updatedAt() reflects the most recent cycle's nowUs");
  checkU64Eq(deviceBus.motor(2).updatedAt(), lastNowUs,
             "motor2 updatedAt() also reflects the most recent cycle's nowUs");
}

// 5. sampleAt(otosStamp) on a motor handle returns a bracket-interpolated
//    reading -- both the literal exact-stamp query the acceptance criterion
//    names, and a genuine fractional midpoint query proving a real blend.
void scenarioMotorSampleAtOtosStamp() {
  beginScenario("sampleAt(otosStamp) on a motor handle returns a bracket-interpolated reading");

  Devices::I2CBus::setClock(0);
  Devices::DeviceBus deviceBus(baseMotorConfig(1), baseMotorConfig(2),
                                Devices::OtosConfig{}, Devices::ColorConfig{},
                                Devices::LineConfig{});
  primeOtos(deviceBus.bus(), deviceBus.otosLeaf());

  const uint64_t dt = 20000;
  uint64_t nowUs = 0;

  // Cycle 1 -- perceptionSlot = Line (absent leaf: no-op, no bus traffic).
  deviceBus.clock().setMicros(nowUs);
  scriptMotorCycle(deviceBus.bus(), 0.0f, 0.0f, 0);
  deviceBus.runCycleOnce();
  nowUs += dt;

  // Cycle 2 -- perceptionSlot = Color (absent leaf: no-op).
  deviceBus.clock().setMicros(nowUs);
  scriptMotorCycle(deviceBus.bus(), 10.0f, 0.0f, 0);
  deviceBus.runCycleOnce();
  nowUs += dt;

  // Cycle 3 -- perceptionSlot = Otos: scripted burst read publishes a pose.
  deviceBus.clock().setMicros(nowUs);
  scriptMotorCycle(deviceBus.bus(), 20.0f, 0.0f, 0);
  scriptOtosBurst(deviceBus.bus(), /*x=*/100, /*y=*/0, /*h=*/0, /*vx=*/0, /*vy=*/0, /*vh=*/0);
  deviceBus.runCycleOnce();
  const uint64_t otosStamp = nowUs;

  checkTrue(deviceBus.odometer().latest().valid, "OTOS published on its round-robin turn (cycle 3)");
  checkU64Eq(deviceBus.odometer().updatedAt(), otosStamp, "OTOS ring stamp matches this cycle's nowUs");
  checkUintEq(deviceBus.bus().errCount(Devices::kOtosDeviceAddr), 0, "no script under-run (otos)");
  checkUintEq(deviceBus.bus().errCount(Devices::kNezhaDeviceAddr), 0, "no script under-run (motors)");

  // Exact-stamp query -- every ring in a cycle shares the SAME nowUs
  // (device_bus.cpp's runCycleOnce() reads the clock once per cycle), so
  // otosStamp coincides exactly with motor1's OWN cycle-3 sample --
  // exercises bracket()'s frac==1.0 boundary case end to end.
  Devices::MotorReading exact{};
  checkTrue(deviceBus.motor(1).sampleAt(otosStamp, exact),
            "motor1.sampleAt(otosStamp) finds a bracketing pair");
  checkFloatEq(exact.position, 20.0f, "exact-stamp sampleAt() returns cycle 3's own position");

  // Genuine fractional query -- the midpoint between cycle 2 (10mm) and
  // cycle 3 (20mm) -- proves sampleAt() performs a REAL linear blend, not
  // just an endpoint snap.
  Devices::MotorReading mid{};
  checkTrue(deviceBus.motor(1).sampleAt(otosStamp - dt / 2, mid),
            "motor1.sampleAt() finds a bracketing pair at the midpoint instant");
  checkFloatEq(mid.position, 15.0f,
               "midpoint sampleAt() linearly interpolates position (10mm, 20mm -> 15mm)");
}

// 6. PID-off setDuty() drives the armored write.
void scenarioPidOffSetDutyDrivesArmoredWrite() {
  beginScenario("PID-off setDuty() drives the armored write");

  Devices::I2CBus::setClock(0);
  Devices::MotorConfig cfg1 = baseMotorConfig(1);
  cfg1.slewRate = 100.0f;  // no slew clamping -- isolates this scenario, matches devices_motor_harness.cpp's own precedent
  Devices::DeviceBus deviceBus(cfg1, baseMotorConfig(2), Devices::OtosConfig{},
                                Devices::ColorConfig{}, Devices::LineConfig{});

  deviceBus.motor(1).setPidEnabled(false);
  deviceBus.motor(1).setDuty(0.6f);

  uint64_t nowUs = 0;
  for (int i = 0; i < 5; ++i) {
    deviceBus.clock().setMicros(nowUs);
    scriptMotorCycle(deviceBus.bus(), 0.0f, 0.0f, /*extraDutySlack=*/2);
    deviceBus.runCycleOnce();
    nowUs += 50000;  // 50ms -- clears the 40ms write-rate throttle every cycle
  }

  checkUintEq(deviceBus.bus().errCount(Devices::kNezhaDeviceAddr), 0, "no script under-run");
  checkFloatEq(deviceBus.motor(1).latest().value.appliedDuty, 0.6f,
               "raw staged duty reaches the armored write path while PID is disabled");
}

// 7. Stale staged targets cause the cycle to write neutral.
void scenarioStaleStagedTargetCausesNeutral() {
  beginScenario("stale staged targets cause the cycle to write neutral");

  Devices::I2CBus::setClock(0);
  Devices::MotorConfig cfg1 = baseMotorConfig(1);
  cfg1.velGains = testVelGains();
  cfg1.velDeadband = 5.0f;
  Devices::DeviceBus deviceBus(cfg1, baseMotorConfig(2), Devices::OtosConfig{},
                                Devices::ColorConfig{}, Devices::LineConfig{});

  // Nonzero clock baseline -- see scenarioStagedVelocityReachesArmoredWriteWithinOneCycle()'s
  // own comment for why a first write at EXACTLY nowUs==0 hits a
  // write-rate-limit-throttle coincidence unrelated to what this scenario
  // is testing.
  const uint64_t baseUs = 1000000;
  deviceBus.clock().setMicros(baseUs);
  deviceBus.motor(1).setVelocity(300.0f);
  scriptMotorCycle(deviceBus.bus(), 0.0f, 0.0f, /*extraDutySlack=*/2);
  deviceBus.runCycleOnce();

  checkTrue(std::fabs(deviceBus.motor(1).latest().value.appliedDuty) > 0.03f,
            "precondition: the motor is actively driving before it's allowed to go stale");

  // Advance well past kVelocityStaleUs (300000us / 300ms, device_bus.h)
  // WITHOUT another setVelocity() call -- the deadman condition.
  deviceBus.clock().setMicros(baseUs + 400000);
  scriptMotorCycle(deviceBus.bus(), 0.0f, 0.0f, /*extraDutySlack=*/2);
  deviceBus.runCycleOnce();

  checkUintEq(deviceBus.bus().errCount(Devices::kNezhaDeviceAddr), 0,
              "no script under-run across the watchdog cycle");
  checkFloatEq(deviceBus.motor(1).latest().value.appliedDuty, 0.0f,
               "stale staged velocity target -- the cycle wrote neutral instead of chasing the stale target");
}

// 8. Round-robin completeness: perceptionSlotStep() services line, then
//    color, then OTOS -- exactly one per cycle, in that order.
void scenarioPerceptionRoundRobinServicesEachLeafOnItsTurn() {
  beginScenario("perceptionSlotStep(): round robin services line, then color, then OTOS -- one per cycle");

  Devices::I2CBus::setClock(0);
  Devices::DeviceBus deviceBus(baseMotorConfig(1), baseMotorConfig(2),
                                Devices::OtosConfig{}, Devices::ColorConfig{},
                                Devices::LineConfig{});

  primeLine(deviceBus.bus(), deviceBus.lineLeaf(), 0);
  primeColorAlt(deviceBus.bus(), deviceBus.colorLeaf(), 0);
  primeOtos(deviceBus.bus(), deviceBus.otosLeaf());

  uint64_t nowUs = 0;
  const uint64_t dt = 20000;

  // Cycle 1 -- Line's turn.
  deviceBus.clock().setMicros(nowUs);
  scriptMotorCycle(deviceBus.bus(), 0.0f, 0.0f, 0);
  uint16_t lineRaw[4] = {10, 20, 30, 40};
  scriptLineFrame(deviceBus.bus(), lineRaw);
  deviceBus.runCycleOnce();
  checkTrue(deviceBus.line().latest().valid, "line ring published on cycle 1 (its round-robin turn)");
  checkFalse(deviceBus.color().latest().valid, "color ring NOT yet published (not its turn)");
  checkFalse(deviceBus.odometer().latest().valid, "OTOS ring NOT yet published (not its turn)");
  nowUs += dt;

  // Cycle 2 -- Color's turn.
  deviceBus.clock().setMicros(nowUs);
  scriptMotorCycle(deviceBus.bus(), 0.0f, 0.0f, 0);
  scriptColorAltFrame(deviceBus.bus(), /*c=*/0x0111, /*r=*/0x0222, /*g=*/0x0333, /*b=*/0x0444);
  deviceBus.runCycleOnce();
  checkTrue(deviceBus.color().latest().valid, "color ring published on cycle 2 (its round-robin turn)");
  checkFalse(deviceBus.odometer().latest().valid, "OTOS ring still not published (not its turn yet)");
  nowUs += dt;

  // Cycle 3 -- OTOS's turn.
  deviceBus.clock().setMicros(nowUs);
  scriptMotorCycle(deviceBus.bus(), 0.0f, 0.0f, 0);
  scriptOtosBurst(deviceBus.bus(), 50, 0, 0, 0, 0, 0);
  deviceBus.runCycleOnce();
  checkTrue(deviceBus.odometer().latest().valid, "OTOS ring published on cycle 3 (its round-robin turn)");

  checkUintEq(deviceBus.bus().errCount(Devices::kLineDeviceAddr), 0, "no script under-run (line)");
  checkUintEq(deviceBus.bus().errCount(Devices::kColorDeviceAddrAlt), 0, "no script under-run (color)");
  checkUintEq(deviceBus.bus().errCount(Devices::kOtosDeviceAddr), 0, "no script under-run (otos)");
}

}  // namespace

int main() {
  scenarioScheduleOrderDeterministicBaseline();
  scenarioNoDutyWriteBetweenRequestAndCollect();
  scenarioStagedVelocityReachesArmoredWriteWithinOneCycle();
  scenarioPublishSamplesMonotonicStamps();
  scenarioMotorSampleAtOtosStamp();
  scenarioPidOffSetDutyDrivesArmoredWrite();
  scenarioStaleStagedTargetCausesNeutral();
  scenarioPerceptionRoundRobinServicesEachLeafOnItsTurn();

  if (g_failureCount == 0) {
    std::printf("OK: all device_bus cycle scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the device_bus cycle scenarios\n", g_failureCount);
  return 1;
}
