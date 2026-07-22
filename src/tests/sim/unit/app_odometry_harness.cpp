// app_odometry_harness.cpp -- off-hardware acceptance harness for ticket
// 103-006 (SUC-006), App::Odometry + applyOtosSample()
// (src/firm/app/odometry.{h,cpp}). Proves: Odometry::integrate() accumulates
// world x/y/theta correctly for a straight-line case (equal wheel deltas)
// and a pure-rotation case (equal-and-opposite wheel deltas), reading the
// REAL Devices::NezhaMotor leaves' own position() (no shadow copy); and
// applyOtosSample() copies a REAL Devices::Otos leaf's sample into a
// Telemetry::Frame, respecting the leaf's own read-rate throttle and never
// clobbering the frame when the chip was never detected.
//
// Reuses devices_motor_harness.cpp's NezhaMotor-scripting convention and
// devices_otos_harness.cpp's Otos-scripting convention, duplicated here per
// this codebase's established per-harness-file fixture convention.
// Compiled by test_app_odometry.py with -DHOST_BUILD against odometry.cpp,
// nezha_motor.cpp, velocity_pid.cpp, otos.cpp, sim_plant.cpp,
// {wheel,otos}_plant.cpp, body_kinematics.cpp.
//
// Migrated by sprint 108 ticket 009 off the deleted src/firm/devices/
// i2c_bus_host.cpp scripted-FIFO Devices::I2CBus fake (ticket 001 reduced
// Devices::I2CBus to a pure interface and removed it) onto a
// TestSim::SimPlant scripted deterministically via TestSim::ScriptedI2CHook
// -- see devices_motor_harness.cpp's/scripted_i2c_hook.h's own header for
// the migration rationale. Every scenario below is otherwise UNCHANGED from
// the pre-migration harness -- only the bus/scripting plumbing moved.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include "app/odometry.h"
#include "app/telemetry.h"
#include "devices/device_config.h"
#include "devices/device_types.h"
#include "devices/nezha_motor.h"
#include "devices/otos.h"
#include "kinematics/body_kinematics.h"
#include "scripted_i2c_hook.h"
#include "sim_plant.h"

namespace {

// --- Hand-rolled assertion plumbing (see app_telemetry_harness.cpp) ------

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

void checkUintEq(uint32_t actual, uint32_t expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %u, got %u", what.c_str(),
                  static_cast<unsigned>(expected), static_cast<unsigned>(actual));
    fail(buf);
  }
}

void checkNear(float actual, float expected, float tol, const std::string& what) {
  if (std::fabs(static_cast<double>(actual - expected)) > tol) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %g, got %g (tol %g)", what.c_str(),
                  static_cast<double>(expected), static_cast<double>(actual),
                  static_cast<double>(tol));
    fail(buf);
  }
}

// --- Devices::NezhaMotor scripting helpers (duplicated from
// devices_motor_harness.cpp) ------------------------------------------------

void scriptEncoderRequestCollect(TestSim::ScriptedI2CHook& bus, uint16_t wireAddr,
                                  float positionMm) {
  bus.queueWrite(wireAddr, /*status=*/0);   // requestEncoder()'s 0x46 write
  bus.queueWrite(wireAddr, /*status=*/0);   // slack: a possible same-cycle duty write (0x60)

  int32_t raw = static_cast<int32_t>(std::lround(positionMm * 10.0f));
  uint8_t data[4] = {
      static_cast<uint8_t>(raw & 0xFF),
      static_cast<uint8_t>((raw >> 8) & 0xFF),
      static_cast<uint8_t>((raw >> 16) & 0xFF),
      static_cast<uint8_t>((raw >> 24) & 0xFF),
  };
  bus.queueRead(wireAddr, data, 4, /*status=*/0);   // collectEncoder()'s 4-byte read
}

Devices::MotorConfig baseNezhaConfig(uint32_t port) {
  Devices::MotorConfig cfg;
  cfg.port = port;
  cfg.fwdSign = 1;
  cfg.wheelTravelCalib = 1.0f;
  cfg.velFiltAlpha = 1.0f;
  return cfg;
}

void driveToPosition(Devices::NezhaMotor& motor, TestSim::ScriptedI2CHook& bus,
                      uint16_t wireAddr, float positionMm, uint64_t nowUs) {
  scriptEncoderRequestCollect(bus, wireAddr, positionMm);
  motor.requestSample();
  motor.tick(nowUs);
}

// --- Devices::Otos scripting helpers (duplicated from
// devices_otos_harness.cpp) --------------------------------------------------

constexpr uint16_t kOtosAddr7 = Devices::kOtosDeviceAddr;                      // 0x17
constexpr uint16_t kOtosWireAddr = static_cast<uint16_t>(kOtosAddr7 << 1);     // 0x2E

// Same LSB scale factors as otos.cpp -- duplicated here per this codebase's
// established per-file convention (devices_otos_harness.cpp's own
// precedent). Valid as a DIRECT scale check only because every scenario
// below uses zero mounting offset/yaw (identity lever-arm/rotation).
constexpr float kPosMmPerLsb = 0.305f;
constexpr float kHdgRadPerLsb = 0.00549f * (3.14159265f / 180.0f);

void scriptGenerousWrites(TestSim::ScriptedI2CHook& bus, int count) {
  for (int i = 0; i < count; ++i) bus.queueWrite(kOtosWireAddr, /*status=*/0);
}

void scriptProductId(TestSim::ScriptedI2CHook& bus, uint8_t id, int status = 0) {
  uint8_t data[1] = {id};
  bus.queueRead(kOtosWireAddr, data, 1, status);
}

void scriptPosVel(TestSim::ScriptedI2CHook& bus, int16_t x, int16_t y, int16_t h,
                   int16_t vx, int16_t vy, int16_t vh, int status = 0) {
  uint8_t raw[12];
  raw[0]  = static_cast<uint8_t>(x & 0xFF);
  raw[1]  = static_cast<uint8_t>((x >> 8) & 0xFF);
  raw[2]  = static_cast<uint8_t>(y & 0xFF);
  raw[3]  = static_cast<uint8_t>((y >> 8) & 0xFF);
  raw[4]  = static_cast<uint8_t>(h & 0xFF);
  raw[5]  = static_cast<uint8_t>((h >> 8) & 0xFF);
  raw[6]  = static_cast<uint8_t>(vx & 0xFF);
  raw[7]  = static_cast<uint8_t>((vx >> 8) & 0xFF);
  raw[8]  = static_cast<uint8_t>(vy & 0xFF);
  raw[9]  = static_cast<uint8_t>((vy >> 8) & 0xFF);
  raw[10] = static_cast<uint8_t>(vh & 0xFF);
  raw[11] = static_cast<uint8_t>((vh >> 8) & 0xFF);
  bus.queueRead(kOtosWireAddr, raw, 12, status);
}

// ===========================================================================
// 1. Straight-line case: equal wheel deltas -> theta unchanged, x
//    accumulates the common distance, y unchanged. Cross-checked against
//    BodyKinematics::forward()'s own direct output for the same deltas
//    (AC's "against BodyKinematics::forward()'s own known-correct output").
// ===========================================================================

void scenarioStraightLineAccumulatesDistanceNoHeadingChange() {
  beginScenario("Odometry::integrate(): straight-line (equal wheel deltas) accumulates x, leaves theta at 0");

  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  const uint16_t wireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);

  Devices::NezhaMotor left(plant, baseNezhaConfig(1));
  Devices::NezhaMotor right(plant, baseNezhaConfig(2));

  const float trackWidth = 200.0f;  // [mm]
  App::Odometry odom(left, right, trackWidth);

  // Both wheels advance the SAME 50mm -- vL == vR analog.
  driveToPosition(left, bus, wireAddr, 50.0f, 20000);
  driveToPosition(right, bus, wireAddr, 50.0f, 20000);
  odom.integrate();

  float expectedDist = 0.0f, expectedHeadingDelta = 0.0f;
  BodyKinematics::forward(50.0f, 50.0f, trackWidth, expectedDist, expectedHeadingDelta);
  checkNear(expectedDist, 50.0f, 1e-3f, "sanity: independent forward() gives distance == 50 for equal deltas");
  checkNear(expectedHeadingDelta, 0.0f, 1e-6f, "sanity: independent forward() gives headingDelta == 0 for equal deltas");

  checkNear(odom.x(), 50.0f, 1e-3f, "x accumulates the common wheel distance");
  checkNear(odom.y(), 0.0f, 1e-3f, "y stays 0 -- no heading change means no lateral component");
  checkNear(odom.theta(), 0.0f, 1e-6f, "theta stays 0 for equal wheel deltas");
}

// ===========================================================================
// 2. Pure-rotation case: equal-and-opposite wheel deltas -> distance stays
//    0 (x/y unchanged), theta accumulates the rotation exactly.
// ===========================================================================

void scenarioPureRotationAccumulatesHeadingNoTranslation() {
  beginScenario("Odometry::integrate(): pure-rotation (vL == -vR analog) accumulates theta, leaves x/y at 0");

  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  const uint16_t wireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);

  Devices::NezhaMotor left(plant, baseNezhaConfig(1));
  Devices::NezhaMotor right(plant, baseNezhaConfig(2));

  const float trackWidth = 200.0f;  // [mm]
  App::Odometry odom(left, right, trackWidth);

  // Left goes -d, right goes +d -- vL == -vR analog. d is chosen exactly
  // representable at the leaf's own 0.1mm encoder-decode resolution
  // (scriptEncoderRequestCollect() round-trips positionMm through
  // lround(positionMm * 10) -- see devices_motor_harness.cpp's identical
  // convention) so the independent forward() oracle below isn't thrown off
  // by quantization the leaf itself would also apply on real hardware.
  const float d = 31.4f;  // [mm]
  driveToPosition(left, bus, wireAddr, -d, 20000);
  driveToPosition(right, bus, wireAddr, d, 20000);
  odom.integrate();

  float expectedDist = 0.0f, expectedHeadingDelta = 0.0f;
  BodyKinematics::forward(-d, d, trackWidth, expectedDist, expectedHeadingDelta);
  checkNear(expectedDist, 0.0f, 1e-3f, "sanity: independent forward() gives distance == 0 for equal-and-opposite deltas");
  checkNear(expectedHeadingDelta, (d - (-d)) / trackWidth, 1e-4f,
            "sanity: independent forward() gives headingDelta == (dR-dL)/b");

  checkNear(odom.x(), 0.0f, 1e-3f, "x stays 0 -- zero net distance for a pure rotation");
  checkNear(odom.y(), 0.0f, 1e-3f, "y stays 0 -- zero net distance for a pure rotation");
  checkNear(odom.theta(), expectedHeadingDelta, 1e-4f, "theta accumulates exactly the rotation forward() computed");
}

// ===========================================================================
// 3. No shadow copy: the delta baseline is seeded from the leaves' own
//    position() at construction, so a leaf already at a nonzero position
//    when Odometry is constructed produces a ZERO delta on the first
//    integrate() call, not a phantom jump.
// ===========================================================================

void scenarioBaselineSeededFromLeafPositionAtConstruction() {
  beginScenario("Odometry constructor seeds the delta baseline from the leaves' own position() -- no phantom first jump");

  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  const uint16_t wireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);

  Devices::NezhaMotor left(plant, baseNezhaConfig(1));
  Devices::NezhaMotor right(plant, baseNezhaConfig(2));

  // Advance both leaves to a nonzero position BEFORE constructing Odometry.
  driveToPosition(left, bus, wireAddr, 500.0f, 20000);
  driveToPosition(right, bus, wireAddr, 500.0f, 20000);

  App::Odometry odom(left, right, 200.0f);
  odom.integrate();  // no further motion since construction -- delta must be 0

  checkNear(odom.x(), 0.0f, 1e-3f, "x stays 0 -- the pre-existing leaf position is NOT counted as first-cycle motion");
  checkNear(odom.theta(), 0.0f, 1e-6f, "theta stays 0 for the same reason");
}

// ===========================================================================
// 4. applyOtosSample(): a present+connected chip copies its pose into the
//    frame; a burst-read failure holds the stale pose but reports
//    disconnected; a never-detected chip leaves the frame's otos field
//    untouched.
// ===========================================================================

void scenarioApplyOtosSamplePresentAndConnectedCopiesPose() {
  beginScenario("applyOtosSample(): present+connected -- frame carries otosPresent/otosConnected/otos pose");

  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  Devices::OtosConfig cfg;  // zero offsets, scale 1.0 -- identity transform
  Devices::Otos otos(plant, cfg);

  scriptGenerousWrites(bus, 20);
  scriptProductId(bus, 0x5F);
  otos.begin();
  checkTrue(otos.present(), "setup: begin() detected the chip");

  scriptPosVel(bus, /*x=*/100, /*y=*/50, /*h=*/1000, /*vx=*/0, /*vy=*/0, /*vh=*/0);

  App::Telemetry::Frame frame;
  App::applyOtosSample(otos, /*now=*/20000, frame);

  checkTrue(frame.otosPresent, "otosPresent mirrors present()");
  checkTrue(frame.otosConnected, "otosConnected mirrors this tick's connected()");
  checkNear(frame.otos.x, 100.0f * kPosMmPerLsb, 1e-3f, "otos.x matches the scripted burst read, scaled");
  checkNear(frame.otos.y, 50.0f * kPosMmPerLsb, 1e-3f, "otos.y matches the scripted burst read, scaled");
  checkNear(frame.otos.heading, 1000.0f * kHdgRadPerLsb, 1e-4f, "otos.heading matches the scripted burst read, scaled");
}

void scenarioApplyOtosSampleBurstFailureHoldsStalePoseReportsDisconnected() {
  beginScenario("applyOtosSample(): a failed burst read holds the stale pose but reports otosConnected=false");

  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  Devices::OtosConfig cfg;
  Devices::Otos otos(plant, cfg);

  scriptGenerousWrites(bus, 20);
  scriptProductId(bus, 0x5F);
  otos.begin();

  scriptPosVel(bus, 200, 300, 500, 0, 0, 0);
  App::Telemetry::Frame frame;
  App::applyOtosSample(otos, /*now=*/0, frame);
  float expectedX = 200.0f * kPosMmPerLsb;
  checkNear(frame.otos.x, expectedX, 1e-3f, "setup: first sample populated a real pose");
  checkTrue(frame.otosConnected, "setup: first sample reports connected");

  // Second read, past kReadPeriod (20000us), scripted to FAIL.
  scriptPosVel(bus, 0, 0, 0, 0, 0, 0, /*status=*/-5);
  App::applyOtosSample(otos, /*now=*/20000, frame);

  // 115-005: otosPresent is now "OtosReading fresh THIS frame" (tighter than
  // the old pre-115 hasOtos, which mirrored present() -- see
  // applyOtosSample()'s own doc comment in odometry.h). A failed burst read
  // means no fresh pose this frame, so otosPresent is false here even
  // though the chip is still detected (present() stays true, unchecked by
  // this scenario -- otosConnected below is the live per-tick signal that
  // actually reflects this cycle's I2C failure).
  checkFalse(frame.otosPresent, "otosPresent reflects THIS cycle's failed read -- no fresh pose this frame");
  checkFalse(frame.otosConnected, "otosConnected reflects THIS cycle's failed read");
  checkNear(frame.otos.x, expectedX, 1e-3f, "the stale pose is held, not clobbered by the failed read");
}

void scenarioApplyOtosSampleNeverDetectedLeavesFrameUntouched() {
  beginScenario("applyOtosSample(): a never-detected chip leaves frame.otos untouched, both bools false");

  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  Devices::OtosConfig cfg;
  Devices::Otos otos(plant, cfg);

  scriptGenerousWrites(bus, 20);
  scriptProductId(bus, 0x00);  // wrong id -- real chip reports 0x5F
  otos.begin();
  checkFalse(otos.present(), "setup: begin() did not detect a chip");

  App::Telemetry::Frame frame;
  frame.otos = {7.0f, 8.0f, 9.0f};  // sentinel -- must survive untouched

  App::applyOtosSample(otos, /*now=*/1000, frame);

  checkFalse(frame.otosPresent, "otosPresent reflects present() == false");
  checkFalse(frame.otosConnected, "otosConnected reflects connected() == false");
  checkNear(frame.otos.x, 7.0f, 1e-6f, "otos.x left untouched when the chip was never detected");
  checkNear(frame.otos.y, 8.0f, 1e-6f, "otos.y left untouched");
  checkNear(frame.otos.heading, 9.0f, 1e-6f, "otos.heading left untouched");
  checkUintEq(bus.txnCount(kOtosAddr7), 2, "applyOtosSample() adds zero bus traffic -- Otos::tick() itself no-ops");
}

// ===========================================================================
// 5. Rate-limit: a too-soon second call issues no extra bus traffic (Otos's
//    own kReadPeriod throttle, unchanged) yet the frame still carries the
//    last real reading -- "sampled at least once per cycle" means the call
//    happens every cycle, not that a fresh bus transaction does.
// ===========================================================================

void scenarioApplyOtosSampleRateLimitSkipStillReachesFrame() {
  beginScenario("applyOtosSample(): a too-soon call issues zero extra bus traffic but still reaches the frame");

  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  Devices::OtosConfig cfg;
  Devices::Otos otos(plant, cfg);

  scriptGenerousWrites(bus, 20);
  scriptProductId(bus, 0x5F);
  otos.begin();

  scriptPosVel(bus, 100, 50, 1000, 0, 0, 0);
  App::Telemetry::Frame frame;
  App::applyOtosSample(otos, /*now=*/0, frame);  // first call always reads (hasRead_ starts false)
  uint32_t txnAfterFirst = bus.txnCount(kOtosAddr7);

  // Second call only 5000us later -- well under kReadPeriod (20000us). No
  // read is scripted for it; a bus-traffic attempt would surface as a
  // script-mismatch error.
  App::applyOtosSample(otos, /*now=*/5000, frame);

  checkUintEq(bus.txnCount(kOtosAddr7), txnAfterFirst,
              "a too-soon call issues zero additional bus traffic");
  checkUintEq(bus.errCount(kOtosAddr7), 0, "no script-mismatch error -- confirms no unexpected bus call was attempted");
  // 115-005: otosPresent is "fresh THIS frame" (see the burst-failure
  // scenario's own comment above) -- a rate-limited (too-soon) call means
  // no read happened THIS call, so no fresh pose this frame either, even
  // though frame.otos still carries the last real reading below.
  checkFalse(frame.otosPresent, "otosPresent is false on a rate-limited cycle -- no fresh pose read this call");
  checkTrue(frame.otosConnected, "otosConnected still reflects the last REAL read's health, not falsely cleared");
  checkNear(frame.otos.x, 100.0f * kPosMmPerLsb, 1e-3f, "otos pose still carries the last real reading on a rate-limited cycle");
}

}  // namespace

int main() {
  scenarioStraightLineAccumulatesDistanceNoHeadingChange();
  scenarioPureRotationAccumulatesHeadingNoTranslation();
  scenarioBaselineSeededFromLeafPositionAtConstruction();
  scenarioApplyOtosSamplePresentAndConnectedCopiesPose();
  scenarioApplyOtosSampleBurstFailureHoldsStalePoseReportsDisconnected();
  scenarioApplyOtosSampleNeverDetectedLeavesFrameUntouched();
  scenarioApplyOtosSampleRateLimitSkipStillReachesFrame();

  if (g_failureCount == 0) {
    std::printf("OK: all App::Odometry / applyOtosSample scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the App::Odometry scenarios\n", g_failureCount);
  return 1;
}
