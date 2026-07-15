// app_robot_loop_harness.cpp -- off-hardware acceptance harness for ticket
// 105-001 (SUC-018), App::RobotLoop (source/app/robot_loop.{h,cpp}), the
// boot loop + main cycle body extracted from source/main.cpp.
//
// Proves the extraction is genuinely host-buildable and runnable: constructs
// every leaf/app module RobotLoop needs over a scripted Devices::I2CBus
// (i2c_bus_host.cpp) and a scripted Devices::Clock/Sleeper pair
// (clock_host.cpp), a minimal App::Transport stub in place of real
// serial/radio, drives boot() to completion, then cycle() a few times, and
// asserts: no bus script under/over-run (proves cycle ordering matches what
// main.cpp's pre-extraction body actually issued), encoder-derived
// position()/velocity() reflect the scripted samples (proves the
// request/settle/collect timing survived the move), the Sleeper recorded a
// sleep for every runAndWait/pace point (proves markTime()/sleepUntil() now
// route through Devices::Clock/Sleeper, not system_timer_current_time()/
// uBit.sleep()), and Telemetry emitted real bytes on both the primary and
// (given enough cycles) secondary cadence. No MicroBit.h is included by
// this file, robot_loop.h, or robot_loop.cpp -- compiled with -DHOST_BUILD
// against the same headers the ARM build uses.
//
// Mirrors app_preamble_harness.cpp's exact scripting/assertion-plumbing
// conventions (this codebase's established per-harness-file style).
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include "app/comms.h"
#include "app/deadman.h"
#include "app/drive.h"
#include "app/odometry.h"
#include "app/preamble.h"
#include "app/robot_loop.h"
#include "app/telemetry.h"
#include "devices/clock.h"
#include "devices/color_sensor.h"
#include "devices/device_config.h"
#include "devices/device_types.h"
#include "devices/i2c_bus.h"
#include "devices/line_sensor.h"
#include "devices/nezha_motor.h"
#include "devices/otos.h"

namespace {

// --- Hand-rolled assertion plumbing (see app_preamble_harness.cpp) --------

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

void checkFloatEq(float actual, float expected, const std::string& what, float tol = 1e-3f) {
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

// --- Minimal App::Transport stub -- no real serial/radio, ever. readLine()
// always reports "nothing ready" (this harness does not need to inject
// commands to prove the extraction runs); send()/sendReliable() just record
// whether anything was ever sent, proving Telemetry's own emit() path is
// reachable end-to-end through Comms with no MicroBit.h dependency. -------
class NullTransport : public App::Transport {
 public:
  bool readLine(char*, uint16_t) override { return false; }
  void send(const char*) override { ++sendCount; }
  void sendReliable(const char*) override { ++sendCount; }

  int sendCount = 0;
};

// --- Fixture helpers (mirrors app_preamble_harness.cpp) --------------------

constexpr uint16_t kMotorWireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);
constexpr uint16_t kOtosWireAddr = static_cast<uint16_t>(Devices::kOtosDeviceAddr << 1);
constexpr uint16_t kColorAltWireAddr = static_cast<uint16_t>(Devices::kColorDeviceAddrAlt << 1);
constexpr uint16_t kLineWireAddr = static_cast<uint16_t>(Devices::kLineDeviceAddr << 1);

Devices::MotorConfig baseMotorConfig(uint32_t port) {
  Devices::MotorConfig cfg;
  cfg.port = port;
  cfg.fwdSign = 1;
  cfg.wheelTravelCalib = 1.0f;
  cfg.velFiltAlpha = 1.0f;  // no smoothing -- velocity() reflects the raw difference-quotient exactly
  return cfg;
}

void scriptEncoderCall(Devices::I2CBus& bus, int writeStatus, int readStatus) {
  bus.scriptWrite(kMotorWireAddr, writeStatus);
  uint8_t data[4] = {0, 0, 0, 0};
  bus.scriptRead(kMotorWireAddr, data, 4, readStatus);
}

// begin()'s hardReset(): 4 calls (3 median snapshots + 1 readback), all
// succeeding, all reading back raw=0 -- see app_preamble_harness.cpp's
// identical helper for the full derivation.
void scriptMotorBeginSuccess(Devices::I2CBus& bus) {
  for (int i = 0; i < 4; ++i) scriptEncoderCall(bus, /*writeStatus=*/0, /*readStatus=*/0);
}

void scriptOtosBeginSuccess(Devices::I2CBus& bus) {
  for (int i = 0; i < 7; ++i) bus.scriptWrite(kOtosWireAddr, 0);
  uint8_t id[1] = {0x5F};
  bus.scriptRead(kOtosWireAddr, id, 1, 0);
}

void scriptColorBeginSuccess(Devices::I2CBus& bus) {
  bus.scriptWrite(kColorAltWireAddr, 0);  // writeReg8(0x81, 0xCA)
  bus.scriptWrite(kColorAltWireAddr, 0);  // writeReg8(0x80, 0x17)
  bus.scriptWrite(kColorAltWireAddr, 0);  // readReg16Alt(0xA4) lo-byte select
  bus.scriptWrite(kColorAltWireAddr, 0);  // readReg16Alt(0xA4) hi-byte select
  uint8_t lo[1] = {0x34};
  uint8_t hi[1] = {0x12};
  bus.scriptRead(kColorAltWireAddr, lo, 1, 0);
  bus.scriptRead(kColorAltWireAddr, hi, 1, 0);
}

void scriptLineBeginSuccess(Devices::I2CBus& bus) {
  for (int ch = 0; ch < 4; ++ch) {
    bus.scriptWrite(kLineWireAddr, 0);
    uint8_t data[1] = {100};
    bus.scriptRead(kLineWireAddr, data, 1, 0);
  }
}

// One request(write)+collect(4-byte read) pair for a single motor, plus
// `extraDutyWrites` additional writes (0 or 1) IF this exact call is
// expected to also land a duty write this cycle.
//
// Unlike a single-device harness (e.g. devices_motor_harness.cpp's own
// scriptEncoderRequestCollect(), which over-provisions one "slack" write
// per cycle because an unconsumed entry simply rolls forward into that
// SAME motor's own next cycle, harmlessly), RobotLoop::cycle() interleaves
// TWO motors AND the OTOS burst read on the SAME global write/read FIFO
// (i2c_bus_host.cpp's own file-header: scriptWrite()/scriptRead() are each
// ONE queue shared across every address, matched by address only) -- an
// over-provisioned "maybe" write left unconsumed at a motor's own address
// does NOT roll forward harmlessly here; it gets wrongly popped by the
// NEXT real call to any address, including OTOS's own burst-read write,
// corrupting that leaf's errCount(). Exact counts are required.
//
// Empirically derived (via a standalone NezhaMotor+Drive-free probe, then
// confirmed against this exact scenario's own dumpRecent() log) and
// explained by RobotLoop::cycle()'s own real ordering: drive_.tick() (which
// calls both leaves' setVelocity(), transitioning mode_ from its
// pre-Drive-wiring default to Active) runs BETWEEN motorL_.tick() and
// motorR_.tick() within a SINGLE cycle() call (see robot_loop.cpp's own
// cycle() body). So each leaf's PID/duty dispatch activates ONE CYCLE
// APART: R's mode_ is already Active by the time motorR_.tick() runs on
// cycle 0 (drive_.tick() ran moments earlier, same cycle) -- R issues its
// OWN one-time "first write" (lastWrittenPct_'s -128 sentinel, pct=0) THAT
// cycle. L's motorL_.tick() runs BEFORE drive_.tick() has EVER executed on
// cycle 0 (mode_ still at its pre-Drive default, no dispatch at all) -- L's
// own one-time first write is deferred to cycle 1, once mode_ is Active
// from cycle 0's drive_.tick() call. Both gains are zero (baseMotorConfig()
// leaves velGains defaulted), so duty is deterministically 0 forever after
// each leaf's own first write -- write-on-change then skips every
// subsequent cycle for that leaf, matching this scenario's own 3/3/2
// write-count schedule below.
void scriptMotorCycle(Devices::I2CBus& bus, float positionMm, int extraDutyWrites) {
  bus.scriptWrite(kMotorWireAddr, /*status=*/0);  // requestEncoder()'s 0x46 write
  for (int i = 0; i < extraDutyWrites; ++i) {
    bus.scriptWrite(kMotorWireAddr, /*status=*/0);  // this leaf's own one-time first duty write
  }
  int32_t raw = static_cast<int32_t>(positionMm * 10.0f);
  uint8_t data[4] = {
      static_cast<uint8_t>(raw & 0xFF),
      static_cast<uint8_t>((raw >> 8) & 0xFF),
      static_cast<uint8_t>((raw >> 16) & 0xFF),
      static_cast<uint8_t>((raw >> 24) & 0xFF),
  };
  bus.scriptRead(kMotorWireAddr, data, 4, /*status=*/0);
}

// One 1-write + 12-byte-read OTOS burst (Otos::readPositionVelocity()) --
// only needed on the FIRST cycle() call after begin() (hasRead_ starts
// false, so the very next tick() is unconditionally due); every later
// cycle() call in this harness runs at the SAME frozen fake-Clock value, so
// (nowUs - lastReadUs_) stays 0 < kReadPeriod and Otos::tick() performs zero
// bus traffic -- no further scripting needed for a "few cycles" smoke test.
void scriptOtosReadZeroPose(Devices::I2CBus& bus) {
  bus.scriptWrite(kOtosWireAddr, /*status=*/0);
  uint8_t raw[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  bus.scriptRead(kOtosWireAddr, raw, 12, /*status=*/0);
}

// ===========================================================================
// Boot resolves all 5 devices, then a few main cycles run to completion with
// no bus script under/over-run, encoder-derived position()/velocity()
// reflecting the scripted samples, and the Sleeper/Telemetry seams both
// exercised -- the ticket's own acceptance criterion ("steps it through
// boot + a few cycles, and confirms it runs to completion with no
// MicroBit.h dependency anywhere in the compiled translation units").
// ===========================================================================

void scenarioBootThenAFewCyclesRunToCompletion() {
  beginScenario("RobotLoop: boot() resolves all devices, cycle() runs a few passes cleanly");

  Devices::I2CBus::setClock(1000000);
  Devices::I2CBus bus;
  Devices::Clock clock;
  Devices::Sleeper sleeper;

  Devices::NezhaMotor motorL(bus, baseMotorConfig(1));
  Devices::NezhaMotor motorR(bus, baseMotorConfig(2));
  Devices::Otos otos(bus, Devices::OtosConfig{});
  Devices::ColorSensorLeaf color(bus, Devices::ColorConfig{});
  Devices::LineSensorLeaf line(bus, Devices::LineConfig{});

  NullTransport serialLink;
  NullTransport radioLink;
  App::Comms comms(serialLink, radioLink, "DEVICE:NEZHA2:robot:test:0");
  App::Telemetry tlm(comms, serialLink, radioLink);
  App::Deadman deadman(clock);
  App::Drive drive(motorL, motorR, /*trackWidth=*/120.0f);
  App::Odometry odom(motorL, motorR, /*trackWidth=*/120.0f);
  App::Preamble preamble(motorL, motorR, otos, color, line, clock);

  // --- Drive Preamble to done() BEFORE constructing/calling RobotLoop's
  // own boot() -- see this file's own derivation (robot_loop.cpp's boot()
  // has no way to advance a HOST_BUILD fake Clock itself, exactly like the
  // real ARM Clock never self-advances except via the hardware timer; the
  // power-settle gate's "elapsed since Preamble's OWN first step() call"
  // check would otherwise never pass inside one un-interrupted boot() call
  // against a Clock that only moves when a test explicitly steps it). The
  // FIRST step() call below reproduces app_preamble_harness.cpp's own
  // "power-settle no-op" call at nowUs=0; advancing the clock to 50000
  // (>= Preamble's kPowerSettle) BEFORE robotLoop.boot() lets every
  // remaining slot resolve on its first attempt, all at that SAME frozen
  // timestamp -- exactly app_preamble_harness.cpp's own all-present
  // happy-path scenario, just driven through RobotLoop::boot() instead of
  // Preamble::step() directly. ---
  clock.setMicros(0);
  preamble.step();  // power-settle no-op; latches Preamble's own start time
  checkTrue(!preamble.done(), "not done after the power-settle no-op call");

  clock.setMicros(50000);
  scriptMotorBeginSuccess(bus);  // Left
  scriptMotorBeginSuccess(bus);  // Right
  scriptOtosBeginSuccess(bus);
  scriptColorBeginSuccess(bus);
  scriptLineBeginSuccess(bus);

  App::RobotLoop robotLoop(bus, motorL, motorR, otos, comms, tlm, drive, odom,
                            deadman, preamble, clock, sleeper);

  int sleepsBeforeBoot = sleeper.sleepCount();
  robotLoop.boot();

  checkTrue(preamble.done(), "boot() returns once preamble.done() is true");
  checkTrue(preamble.leftConnected(), "leftConnected() true after boot()");
  checkTrue(preamble.rightConnected(), "rightConnected() true after boot()");
  checkTrue(preamble.otosConnected(), "otosConnected() true after boot()");
  checkTrue(sleeper.sleepCount() > sleepsBeforeBoot,
            "boot()'s own pacing sleep (kPreamblePace) went through Devices::Sleeper, "
            "not uBit.sleep()");
  checkUintEq(bus.errCount(Devices::kNezhaDeviceAddr), 0, "no script under-run: motor (boot)");
  checkUintEq(bus.errCount(Devices::kOtosDeviceAddr), 0, "no script under-run: otos (boot)");

  // --- A few main cycles. Encoder positions climb by 10mm per cycle on
  // both wheels (a plain forward roll) -- proves requestSample()/tick()'s
  // request->settle->collect timing survived the extraction unchanged, and
  // that Odometry/applyOtosSample still run every cycle in the same order
  // main.cpp's pre-extraction for(;;) body did. ---
  constexpr int kCycles = 3;
  for (int i = 0; i < kCycles; ++i) {
    float positionMm = static_cast<float>(i + 1) * 10.0f;
    // R gets its own one-time first duty write on cycle 0; L gets its own
    // one-time first duty write on cycle 1 (one cycle later than R) -- see
    // scriptMotorCycle()'s own header comment for the full derivation.
    scriptMotorCycle(bus, positionMm, /*extraDutyWrites=*/(i == 1) ? 1 : 0);  // Left
    scriptMotorCycle(bus, positionMm, /*extraDutyWrites=*/(i == 0) ? 1 : 0);  // Right
    if (i == 0) scriptOtosReadZeroPose(bus);  // Otos::tick()'s hasRead_==false on cycle 1 only

    int sleepsBefore = sleeper.sleepCount();
    robotLoop.cycle();
    checkTrue(sleeper.sleepCount() > sleepsBefore,
              "cycle() paces via Devices::Sleeper (three runAndWait blocks + final "
              "sleepUntil), not uBit.sleep()");
  }

  checkFloatEq(motorL.position(), 30.0f, "left position reflects the 3rd scripted cycle's sample");
  checkFloatEq(motorR.position(), 30.0f, "right position reflects the 3rd scripted cycle's sample");
  checkTrue(motorL.connected(), "left motor still connected after 3 cycles");
  checkTrue(motorR.connected(), "right motor still connected after 3 cycles");

  checkUintEq(bus.errCount(Devices::kNezhaDeviceAddr), 0, "no script under-run: motor (cycles)");
  checkUintEq(bus.errCount(Devices::kOtosDeviceAddr), 0, "no script under-run: otos (cycles)");

  checkTrue(tlm.primaryEmitCount() > 0, "Telemetry emitted at least one primary frame");
  checkTrue(serialLink.sendCount > 0, "Comms actually sent bytes over the (fake) transport");
}

}  // namespace

int main() {
  scenarioBootThenAFewCyclesRunToCompletion();

  if (g_failureCount == 0) {
    std::printf("OK: all App::RobotLoop scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the App::RobotLoop scenarios\n", g_failureCount);
  return 1;
}
