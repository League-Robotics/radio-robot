// devices_color_sensor_apds_probe_harness.cpp -- regression harness for
// ticket 108-008 (clasi/issues/color-sensor-apds-probe-success-on-failure.md,
// 2026-07-13 code review finding M4).
//
// Proves Devices::ColorSensorLeaf::beginStep()'s APDS probe (color_sensor.cpp
// ApdsProbe phase) no longer latches present()==true on a NAK'd bus read.
// Pre-fix, the probe used readReg8() (status-ignoring); a NAK left its
// out-buffer at its zero-initialized default, and en==0x00 is exactly the
// leaf's own "detected" condition -- a robot with NO color sensor at all
// would latch present()==true and keep issuing failing APDS transactions at
// every due perception slot forever. Post-fix, the probe uses
// readReg8Status() and requires the transaction to report OK before trusting
// the register value.
//
// This is the Stage-4-adjacent Python-hook test pattern's C++ half: this
// sprint's ticket 108-002 real Devices::I2CBus implementation, TestSim::
// SimPlant (tests/_infra/sim/sim_plant.{h,cpp}), already NAKs every
// unrecognized/absent-device address by default -- including the color
// sensor's 0x39/0x43 addresses (sim_plant.cpp's own header comment names
// this exact ticket as one of its consumers). ticket 108-008's own
// acceptance criteria call for a "Python hook test," but as of this ticket
// nothing in source/app/ surfaces ColorSensorLeaf::present()/connected() to
// telemetry (grepped: no color-sensor field reaches App::Telemetry or the
// wire), so there is no host-observable signal a Python/ctypes-level SimLoop
// test could assert on without scope-creeping a telemetry change into this
// ticket. Per this ticket's own "what this ticket does" fallback (b): a
// small C++ hook-driven test using SimPlant directly, registering a
// SimPlant::ReadHook that NAKs vs. returns a valid enable-register readback
// for the APDS probe address, driving ColorSensorLeaf::beginStep() and
// asserting present(). Ticket 009's broader tests/sim/unit -> Python-hook
// migration is the natural place to also wire a telemetry-level version of
// this same scenario once that surface exists.
//
// Compiled by test_devices_color_sensor_apds_probe.py against the REAL
// source/devices/color_sensor.cpp and tests/_infra/sim/sim_plant.cpp (plus
// its own wheel_plant.cpp/otos_plant.cpp dependencies) with -DHOST_BUILD,
// the same "compile a throwaway subprocess binary" pattern every other
// tests/sim/{unit,plant,system} harness uses (see test_plant.py, this
// ticket's own header comment, for the closest precedent: SimPlant plus a
// handful of source/devices/*.cpp files, no full App::RobotLoop needed).

#include <cstdint>
#include <cstdio>
#include <string>

#include "devices/color_sensor.h"
#include "devices/device_config.h"
#include "sim_plant.h"

namespace {

// --- Hand-rolled assertion plumbing (mirrors devices_sensors_harness.cpp) --

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

void checkIntEq(int actual, int expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %d, got %d", what.c_str(), expected, actual);
    fail(buf);
  }
}

// color_sensor.h's own private constants, duplicated here per this
// codebase's established per-file fixture-duplication convention (see
// devices_sensors_harness.cpp's identical kAltRetryPeriodUs/kMaxAltAttempts
// duplication, and sim_plant.cpp's own header comment on the same
// convention) -- paces this harness's own nowUs advances through
// beginStep()'s AltProbe phase without depending on the leaf's private
// members.
constexpr uint64_t kAltRetryPeriodUs = 50000;
constexpr int kMaxAltAttempts = 20;

// CODAL's well-known success convention, matching SimPlant's/color_sensor.
// cpp's own identical local kOk.
constexpr int kOk = 0;

// Drives beginStep() until detectDone() latches or a generous bound is hit
// (never an unbounded loop -- mirrors devices_sensors_harness.cpp's own
// bounded-termination scenario). AltProbe never answers under either
// scenario below (SimPlant's default handler NAKs 0x43 unconditionally, and
// this harness never hooks it), so every run walks the full kMaxAltAttempts
// retries before beginStep() ever reaches the ApdsProbe phase this test is
// actually about.
int runDetectionToCompletion(Devices::ColorSensorLeaf& sensor) {
  const int kBound = kMaxAltAttempts + 2;  // 20 ALT attempts + 1 APDS attempt + margin
  int callsUsed = 0;
  for (int i = 0; i < kBound; ++i) {
    sensor.beginStep(static_cast<uint64_t>(i) * kAltRetryPeriodUs);
    callsUsed = i + 1;
    if (sensor.detectDone()) break;
  }
  return callsUsed;
}

// --- Scenarios --------------------------------------------------------

// 1. Absent color sensor (the bug's exact repro): SimPlant's DEFAULT read
//    handler NAKs the APDS probe address (0x39<<1) -- no hook registered at
//    all, proving the fix against the simulator's own "no chip present"
//    baseline, not a hand-scripted stand-in for one. Pre-fix, this scenario
//    latches present()==true (the bug); post-fix it must latch false.
void scenarioApdsProbeNakLatchesAbsent() {
  beginScenario("ColorSensorLeaf APDS probe: SimPlant default (NAK) -- present() must latch false");

  TestSim::SimPlant plant;
  int readCalls = 0;
  plant.setReadHook([&](uint16_t address, uint8_t* data, int len) {
    ++readCalls;
    return plant.defaultRead(address, data, len);  // pass-through, just counting
  });

  Devices::ColorConfig cfg;
  Devices::ColorSensorLeaf sensor(plant, cfg);

  int callsUsed = runDetectionToCompletion(sensor);
  checkTrue(sensor.detectDone(), "detectDone() reached within the bounded call count");
  checkTrue(callsUsed <= kMaxAltAttempts + 2, "termination bounded (never hangs)");

  // The load-bearing assertion: a NAK'd probe must NOT be mistaken for
  // "device answered with enable-register value 0."
  checkFalse(sensor.present(), "present() false -- the APDS9960 never actually answered (NAK)");
  checkFalse(sensor.connected(), "connected() false -- the APDS9960 never actually answered (NAK)");

  // "the perception slot skips it" (the issue's own bench-verification
  // wording): once beginStep() has latched present()==false, tick() must be
  // a total bus-traffic no-op forever after, at any nowUs -- no recurring
  // failing APDS transactions.
  int readCallsAfterDetect = readCalls;
  for (uint64_t nowUs = 0; nowUs < 5000000; nowUs += 100000) {
    sensor.tick(nowUs);
  }
  checkIntEq(readCalls, readCallsAfterDetect,
             "tick() issues zero further bus reads once present() has latched false "
             "(no recurring bus errors -- perception slot skip)");
  checkFalse(sensor.readingFresh(), "readingFresh() stays false -- tick() never runs on an absent leaf");
}

// 2. Present color sensor: a registered hook returns a valid, transaction-OK
//    enable-register readback (en==0x00) for the APDS probe address --
//    present() must latch true. Proves the fix doesn't just flip every
//    outcome to "absent" -- a genuinely-answering APDS9960 is still detected.
void scenarioApdsProbeOkLatchesPresent() {
  beginScenario("ColorSensorLeaf APDS probe: hook returns OK + en==0x00 -- present() must latch true");

  TestSim::SimPlant plant;
  constexpr uint16_t kApdsWireAddr = static_cast<uint16_t>(Devices::kColorDeviceAddrApds << 1);

  // Writes to the APDS address (the ENABLE-register writes beginStep()'s
  // ApdsProbe phase and initApds() both issue) always ACK.
  plant.setWriteHook([&](uint16_t address, uint8_t* data, int len) {
    if (address == kApdsWireAddr) return kOk;
    return plant.defaultWrite(address, data, len);
  });
  // Reads to the APDS address return a valid, OK'd enable-register readback
  // of 0x00 -- exactly the "chip answered, currently powered off" value the
  // probe treats as detected. Every other address falls through to
  // SimPlant's own default (NAK for color/line, real handling for motor/
  // OTOS).
  plant.setReadHook([&](uint16_t address, uint8_t* data, int len) {
    if (address == kApdsWireAddr) {
      for (int i = 0; i < len; ++i) data[i] = 0x00;
      return kOk;
    }
    return plant.defaultRead(address, data, len);
  });

  Devices::ColorConfig cfg;
  Devices::ColorSensorLeaf sensor(plant, cfg);

  int callsUsed = runDetectionToCompletion(sensor);
  checkTrue(sensor.detectDone(), "detectDone() reached within the bounded call count");
  checkIntEq(callsUsed, kMaxAltAttempts + 1,
             "detection took exactly kMaxAltAttempts ALT attempts plus one successful APDS attempt");

  checkTrue(sensor.present(), "present() true -- the APDS9960 answered OK with en==0x00");
  checkTrue(sensor.connected(), "connected() true -- the APDS9960 answered OK with en==0x00");
}

}  // namespace

int main() {
  scenarioApdsProbeNakLatchesAbsent();
  scenarioApdsProbeOkLatchesPresent();

  if (g_failureCount == 0) {
    std::printf("OK: all ColorSensorLeaf APDS-probe regression scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the ColorSensorLeaf APDS-probe scenarios\n", g_failureCount);
  return 1;
}
