// devices_otos_harness.cpp — off-hardware acceptance harness for ticket
// DB-005 (device-bus-tickets.md): exercises the REAL Devices::Otos leaf
// (source/devices/otos.cpp) against DB-003's HOST_BUILD scripted
// Devices::I2CBus fake -- no MicroBitI2C, no CODAL, no real hardware.
//
// Modeled on tests/sim/unit/otos_odometer_harness.cpp (that file's own
// header comment is this harness's explicit test precedent) -- compiles the
// ACTUAL source/devices/otos.cpp against the SAME source/devices/otos.h
// every ARM build compiles, with -DHOST_BUILD selecting
// source/devices/i2c_bus_host.cpp's scripted fake in place of the real
// MicroBitI2C-backed i2c_bus.cpp. Hand-rolled assertions, PASS/FAIL per
// scenario, nonzero exit on any failure. Run by test_devices_otos.py, which
// compiles and runs this binary via subprocess. Includes ONLY devices/
// headers plus plain C/C++ stdlib (isolation invariant) -- no messages/*.h,
// no config/boot_config.h, no com/i2c_bus.h.
//
// --- Why these scenarios can't inspect exact written register bytes ---
// The HOST_BUILD scripted fake (i2c_bus_host.cpp) does not record a write()
// call's payload bytes -- only the address and per-device txnCount()/
// errCount() are observable. So these scenarios prove behavior through
// txnCount() deltas, connected()/present()/poseFresh() (the leaf's own
// observable state), and scripted READ payloads (which the fake DOES
// deliver back), letting these scenarios verify the read-side register
// scaling + mounting-rotation + lever-arm math end to end.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include "devices/device_config.h"
#include "devices/device_types.h"
#include "devices/i2c_bus.h"
#include "devices/otos.h"

namespace {

// --- Hand-rolled assertion plumbing (mirrors otos_odometer_harness.cpp) ---

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

// --- Fixture helpers ---------------------------------------------------

constexpr uint16_t kAddr7 = Devices::kOtosDeviceAddr;                     // 0x17
constexpr uint16_t kWireAddr = static_cast<uint16_t>(kAddr7 << 1);        // 0x2E

// Same LSB scale factors as otos.cpp -- duplicated here so this harness can
// construct scripted register payloads and compute the expected post-
// transform pose from first principles (this codebase's existing per-file
// convention for scale-constant duplication -- see otos_odometer_harness.cpp's
// own precedent).
constexpr float kPosMmPerLsb = 0.305f;
constexpr float kHdgRadPerLsb = 0.00549f * (3.14159265f / 180.0f);

// kReadPeriod duplicated from otos.h's private constant -- this file's own
// established convention for restating a private leaf constant (matches
// otos_odometer_harness.cpp's identical kReadPeriodMs precedent).
constexpr uint64_t kReadPeriodUs = 20000;   // [us]

// testSensorToCentre()/testCentreToSensor() -- a LOCAL, independent
// re-implementation of Devices::Otos's private sensorToCentre()/
// centreToSensor() methods, duplicated the same way kPosMmPerLsb/
// kHdgRadPerLsb above already are -- this file's own established convention
// for a test oracle that can't reach a production symbol directly.
void testSensorToCentre(float sensorX, float sensorY, float sensorHeading,
                         float offsetX, float offsetY,
                         float& centreXOut, float& centreYOut) {
  float c = cosf(sensorHeading);
  float s = sinf(sensorHeading);
  float offsetXWorld = c * offsetX - s * offsetY;
  float offsetYWorld = s * offsetX + c * offsetY;
  centreXOut = sensorX - offsetXWorld;
  centreYOut = sensorY - offsetYWorld;
}

void testCentreToSensor(float centreX, float centreY, float centreHeading,
                         float offsetX, float offsetY,
                         float& sensorXOut, float& sensorYOut) {
  float c = cosf(centreHeading);
  float s = sinf(centreHeading);
  sensorXOut = centreX + (c * offsetX - s * offsetY);
  sensorYOut = centreY + (s * offsetX + c * offsetY);
}

Devices::OtosConfig makeConfig(float offsetX, float offsetY, float offsetYaw,
                                float linearScale, float angularScale) {
  Devices::OtosConfig cfg;
  cfg.offsetX = offsetX;
  cfg.offsetY = offsetY;
  cfg.offsetYaw = offsetYaw;
  cfg.linearScale = linearScale;
  cfg.angularScale = angularScale;
  return cfg;
}

void scriptGenerousWrites(Devices::I2CBus& bus, int count) {
  for (int i = 0; i < count; ++i) bus.scriptWrite(kWireAddr, /*status=*/0);
}

void scriptProductId(Devices::I2CBus& bus, uint8_t id, int status = 0) {
  uint8_t data[1] = {id};
  bus.scriptRead(kWireAddr, data, 1, status);
}

// Queues one scripted 12-byte burst read (X_L X_H Y_L Y_H H_L H_H, then
// VX_L VX_H VY_L VY_H VH_L VH_H, all LE) for the next readPositionVelocity()
// call.
void scriptPosVel(Devices::I2CBus& bus, int16_t x, int16_t y, int16_t h,
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
  bus.scriptRead(kWireAddr, raw, 12, status);
}

// begin()'s full successful-detect transaction count: 1 write + 1 read
// (product-ID probe) + 3 writes (init(): signal-process-cfg, reset,
// imu-calibration) + 1 write (setLinearScalar) + 1 write (setAngularScalar)
// + 1 write (zero position/heading) = 7 writes + 1 read = 8 total.
constexpr uint32_t kBeginTxnCount = 8;

// --- Scenarios ------------------------------------------------------------

// 1. PRODUCT_ID detect gates ALL traffic: a mismatch leaves the leaf
//    un-initialized after only the failed probe; never calling begin() at
//    all means every primitive (including tick()/setPose()'s drain) is a
//    total no-op; a successful detect runs the full init sequence.
void scenarioProductIdGatesAllTraffic() {
  beginScenario("PRODUCT_ID detect gates all bus traffic");

  // Case A: mismatch -- only the failed probe touches the bus.
  {
    Devices::I2CBus::setClock(1000000);
    Devices::I2CBus bus;
    scriptGenerousWrites(bus, 20);
    scriptProductId(bus, 0x00);   // wrong id -- real chip reports 0x5F

    Devices::Otos odom(bus, makeConfig(0.0f, 0.0f, 0.0f, 1.0f, 1.0f));
    odom.begin();

    checkFalse(odom.present(), "mismatch: present() false");
    checkFalse(odom.connected(), "mismatch: connected() false");
    checkUintEq(bus.txnCount(kAddr7), 2, "mismatch: only the failed probe (1 write + 1 read) touched the bus");
    checkUintEq(bus.errCount(kAddr7), 0, "mismatch: the probe's own status was OK -- just the ID didn't match");
  }

  // Case B: begin() never called at all -- every primitive is a total no-op.
  {
    Devices::I2CBus::setClock(1000000);
    Devices::I2CBus bus;   // no scripts queued -- any traffic surfaces as an error

    Devices::Otos odom(bus, makeConfig(-47.7f, 3.5f, 0.0f, 1.067f, 0.987f));

    float ox = 0, oy = 0, oh = 0;
    odom.init();
    odom.resetTracking();
    odom.setLinearScalar(50.0f);
    odom.setAngularScalar(-13.0f);
    odom.setOffset(1.0f, 2.0f, 0.1f);
    odom.getOffset(ox, oy, oh);
    odom.setSignalProcessConfig(0x0F);
    uint8_t signalCfg = odom.signalProcessConfig();
    uint8_t imuRemaining = odom.imuCalibrationSamplesRemaining();
    odom.setPose(10.0f, 20.0f, 0.5f);   // stages only
    odom.tick(1000);                    // would drain the staged pose IF initialized

    checkFalse(odom.present(), "never begun: present() stays false");
    checkFalse(odom.connected(), "never begun: connected() stays false");
    checkUintEq(bus.txnCount(kAddr7), 0, "never begun: zero bus traffic from any primitive, including tick()'s drain");
    checkUintEq(bus.errCount(kAddr7), 0, "never begun: zero traffic means zero script-mismatch errors too");
    checkNear(ox, 0.0f, 1e-6f, "never begun: getOffset() returns zero");
    checkUintEq(signalCfg, 0, "never begun: signalProcessConfig() returns 0");
    checkUintEq(imuRemaining, 0, "never begun: imuCalibrationSamplesRemaining() returns 0");

    Devices::PoseReading pose = odom.pose();
    checkNear(pose.x, 0.0f, 1e-6f, "never begun: pose() stays the zero default");
    checkFalse(odom.poseFresh(), "never begun: poseFresh() stays false");
  }

  // Case C: successful detect -- runs the full init sequence.
  {
    Devices::I2CBus::setClock(1000000);
    Devices::I2CBus bus;
    scriptGenerousWrites(bus, 20);
    scriptProductId(bus, 0x5F);

    Devices::Otos odom(bus, makeConfig(0.0f, 0.0f, 0.0f, 1.0f, 1.0f));
    odom.begin();

    checkTrue(odom.present(), "match: present() true");
    checkTrue(odom.connected(), "match: connected() true");
    checkUintEq(bus.txnCount(kAddr7), kBeginTxnCount,
                "match: begin() issued exactly the expected probe+init+scalar+zero-pose transactions");
    checkUintEq(bus.errCount(kAddr7), 0, "match: no script under-run");
  }
}

// 2. readDue()/tick() rate-limiting: true before any real read, false right
//    after one, true again once kReadPeriod elapses -- and a tick() call
//    that arrives too soon issues ZERO bus traffic and marks the sample
//    stale rather than re-publishing.
void scenarioReadDueRateLimitsRealReads() {
  beginScenario("readDue()/tick(): rate-limits real bus reads to kReadPeriod");
  Devices::I2CBus::setClock(1000000);
  Devices::I2CBus bus;
  scriptGenerousWrites(bus, 20);
  scriptProductId(bus, 0x5F);

  Devices::Otos odom(bus, makeConfig(0.0f, 0.0f, 0.0f, 1.0f, 1.0f));
  checkTrue(odom.readDue(0), "readDue() true before begin() is ever called");

  odom.begin();
  checkTrue(odom.readDue(1000000),
            "readDue() still true right after begin() -- the probe is not tick()'s own real read");

  // Tick 1: hasRead_ starts false -- always issues a real read.
  scriptPosVel(bus, 1000, 500, 0, 300, -100, 50);
  odom.tick(1000000);
  checkTrue(odom.poseFresh(), "tick 1: real read -- poseFresh() true");
  uint32_t txnAfterTick1 = bus.txnCount(kAddr7);

  checkFalse(odom.readDue(1000000), "readDue() false immediately after a real read (same now)");
  checkFalse(odom.readDue(1000000 + kReadPeriodUs - 1), "readDue() false just inside the kReadPeriod window");
  checkTrue(odom.readDue(1000000 + kReadPeriodUs), "readDue() true exactly at the kReadPeriod boundary");

  // Tick 2, just inside the window: too soon -- zero further bus traffic,
  // sample marked stale, prior pose held.
  Devices::PoseReading afterTick1 = odom.pose();
  odom.tick(1000000 + kReadPeriodUs - 1);
  checkUintEq(bus.txnCount(kAddr7), txnAfterTick1, "tick 2 (too soon): issues NO bus traffic");
  checkFalse(odom.poseFresh(), "tick 2 (too soon): poseFresh() false -- stale, not re-fused");
  checkNear(odom.pose().x, afterTick1.x, 1e-6f, "tick 2 (too soon): pose().x held unchanged");

  // Tick 3, exactly at the boundary: due again -- one write + one 12-byte read.
  scriptPosVel(bus, 1100, 550, 0, 300, -100, 50);
  odom.tick(1000000 + kReadPeriodUs);
  checkTrue(odom.poseFresh(), "tick 3 (period elapsed): real read -- poseFresh() true");
  checkUintEq(bus.txnCount(kAddr7) - txnAfterTick1, 2,
              "tick 3: issues exactly one write + one read (the combined burst)");

  checkUintEq(bus.errCount(kAddr7), 0, "no script under-run");
}

// 3a. tick(): lever-arm-only transform (offsetYaw == 0, non-zero mounting
//     offset) -- isolates the read-side scaling + sensorToCentre() wiring
//     from the mounting-yaw rotation step (3b isolates that one instead).
void scenarioTickLeverArmOnlyTransform() {
  beginScenario("tick(): a burst decodes to the expected pose -- lever-arm-only transform");
  Devices::I2CBus::setClock(1000000);
  Devices::I2CBus bus;
  scriptGenerousWrites(bus, 20);
  scriptProductId(bus, 0x5F);

  constexpr float kOffsetX = -47.7f;   // [mm] tovez.json-realistic
  constexpr float kOffsetY = 3.5f;     // [mm]
  Devices::Otos odom(bus, makeConfig(kOffsetX, kOffsetY, 0.0f, 1.0f, 1.0f));
  odom.begin();

  constexpr int16_t kRx = 2000, kRy = 1000, kRh = 5217;
  constexpr int16_t kRvx = 300, kRvy = -100, kRvh = 50;
  scriptPosVel(bus, kRx, kRy, kRh, kRvx, kRvy, kRvh);

  odom.tick(2000000);

  checkTrue(odom.connected(), "connected() true after a clean burst read");
  checkTrue(odom.poseFresh(), "poseFresh() true after a clean burst read");

  float xF = static_cast<float>(kRx) * kPosMmPerLsb;
  float yF = static_cast<float>(kRy) * kPosMmPerLsb;
  float hF = static_cast<float>(kRh) * kHdgRadPerLsb;
  float expectedCentreX = 0.0f, expectedCentreY = 0.0f;
  testSensorToCentre(xF, yF, hF, kOffsetX, kOffsetY, expectedCentreX, expectedCentreY);

  Devices::PoseReading pose = odom.pose();
  checkNear(pose.x, expectedCentreX, 1e-2f, "pose().x matches testSensorToCentre()");
  checkNear(pose.y, expectedCentreY, 1e-2f, "pose().y matches testSensorToCentre()");
  checkNear(pose.heading, hF, 1e-5f, "pose().heading passes the raw heading through unmodified");

  float vxF = static_cast<float>(kRvx) * kPosMmPerLsb;
  float vyF = static_cast<float>(kRvy) * kPosMmPerLsb;
  float whF = static_cast<float>(kRvh) * kHdgRadPerLsb;
  checkNear(pose.v_x, vxF, 1e-3f, "twist.v_x is the scaled velocity-register X (no mount rotation)");
  checkNear(pose.v_y, vyF, 1e-3f, "twist.v_y is the scaled velocity-register Y (no mount rotation)");
  checkNear(pose.omega, whF, 1e-6f, "twist.omega passes through unmodified");

  checkUintEq(bus.errCount(kAddr7), 0, "no script under-run");
}

// 3b. tick(): mounting-yaw-rotation-only transform (zero mounting offset,
//     non-zero offsetYaw) -- isolates the rotation step from the lever arm.
void scenarioTickMountingYawRotationOnlyTransform() {
  beginScenario("tick(): a burst decodes to the expected pose -- mounting-yaw-only transform");
  Devices::I2CBus::setClock(1000000);
  Devices::I2CBus bus;
  scriptGenerousWrites(bus, 20);
  scriptProductId(bus, 0x5F);

  constexpr float kOffsetYaw = 0.3f;   // [rad] a hypothetical rotated mount
  Devices::Otos odom(bus, makeConfig(0.0f, 0.0f, kOffsetYaw, 1.0f, 1.0f));
  odom.begin();

  constexpr int16_t kRx = 1500, kRy = -800, kRh = 2000;
  scriptPosVel(bus, kRx, kRy, kRh, 0, 0, 0);

  odom.tick(3000000);

  float xF = static_cast<float>(kRx) * kPosMmPerLsb;
  float yF = static_cast<float>(kRy) * kPosMmPerLsb;
  float hF = static_cast<float>(kRh) * kHdgRadPerLsb;
  float ang = -kOffsetYaw;
  float expectedX = cosf(ang) * xF - sinf(ang) * yF;   // zero offset -- lever-arm is a no-op
  float expectedY = sinf(ang) * xF + cosf(ang) * yF;

  Devices::PoseReading pose = odom.pose();
  checkNear(pose.x, expectedX, 1e-2f, "pose().x reflects the mounting-yaw rotation");
  checkNear(pose.y, expectedY, 1e-2f, "pose().y reflects the mounting-yaw rotation");
  checkNear(pose.heading, hF, 1e-5f, "pose().heading still takes no mounting-rotation adjustment");

  checkUintEq(bus.errCount(kAddr7), 0, "no script under-run");
}

// 4. Lever-arm compensation cancels on a scripted PURE SPIN: the chassis
//    centre stays fixed at the origin while heading sweeps through a wide
//    spread of values; the SENSOR-frame readings a real chip would report
//    for that motion are scripted (via the SAME testCentreToSensor()
//    formula, at each spin sample's SAME-INSTANT heading), and every
//    resulting pose() must stay near (0,0) -- no phantom translation (the
//    db11b7c regression signature this contract exists to prevent). Runs
//    through the REAL tick()/sensorToCentre() wiring, not a local oracle.
void scenarioLeverArmCancelsOnPureSpin() {
  beginScenario("tick(): lever-arm compensation cancels on a pure spin -- no phantom translation");
  Devices::I2CBus::setClock(1000000);
  Devices::I2CBus bus;
  scriptGenerousWrites(bus, 200);
  scriptProductId(bus, 0x5F);

  constexpr float kOffsetX = -47.7f;   // [mm] tovez.json-realistic
  constexpr float kOffsetY = 3.5f;     // [mm]
  Devices::Otos odom(bus, makeConfig(kOffsetX, kOffsetY, 0.0f, 1.0f, 1.0f));
  odom.begin();

  // Spin sweep -- spread across most of the chip's representable heading
  // range (avoiding the exact +/-pi wrap boundary, which is DB-002's
  // wrap-aware-lerp concern, not this leaf's).
  const float headings[] = {0.0f, 0.5f, 1.0f, 1.5f, 2.0f, 2.5f, -2.5f, -2.0f, -1.5f, -1.0f, -0.5f};
  uint64_t now = 2000000;

  for (float heading : headings) {
    // The centre is fixed at (0,0) -- a pure spin. testCentreToSensor()
    // computes what the SENSOR itself reports for a chassis-centre pose of
    // (0,0,heading) with this same-instant heading.
    float sensorX = 0.0f, sensorY = 0.0f;
    testCentreToSensor(0.0f, 0.0f, heading, kOffsetX, kOffsetY, sensorX, sensorY);

    int16_t rx = static_cast<int16_t>(lroundf(sensorX / kPosMmPerLsb));
    int16_t ry = static_cast<int16_t>(lroundf(sensorY / kPosMmPerLsb));
    int16_t rh = static_cast<int16_t>(lroundf(heading / kHdgRadPerLsb));

    scriptPosVel(bus, rx, ry, rh, 0, 0, 0);
    now += kReadPeriodUs;
    odom.tick(now);

    checkTrue(odom.poseFresh(), "spin sample: poseFresh() true");
    Devices::PoseReading pose = odom.pose();

    char label[128];
    std::snprintf(label, sizeof(label), "spin sample at heading=%.3frad", static_cast<double>(heading));
    // Tolerance: one LSB of quantization on each axis (~0.31mm) plus a
    // small float-rounding margin.
    checkNear(pose.x, 0.0f, 1.0f, std::string(label) + " -- x must stay near the fixed centre (no phantom translation)");
    checkNear(pose.y, 0.0f, 1.0f, std::string(label) + " -- y must stay near the fixed centre (no phantom translation)");
    checkNear(pose.heading, heading, 1e-3f, std::string(label) + " -- heading tracks the spin");
  }

  checkUintEq(bus.errCount(kAddr7), 0, "no script under-run across the spin sweep");
}

// 5. A burst-read failure holds the previously-cached pose, marks it stale
//    (poseFresh() false), and flips connected() false -- then a THIRD, clean
//    tick() proves the failure did not permanently latch.
void scenarioBurstFailureHoldsPriorPoseAndMarksStale() {
  beginScenario("tick(): a burst-read failure holds the last-good pose, marks it stale, and recovers");
  Devices::I2CBus::setClock(1000000);
  Devices::I2CBus bus;
  scriptGenerousWrites(bus, 20);
  scriptProductId(bus, 0x5F);

  Devices::Otos odom(bus, makeConfig(0.0f, 0.0f, 0.0f, 1.0f, 1.0f));
  odom.begin();

  // Tick 1: clean burst -- establishes a known-good cached pose.
  scriptPosVel(bus, 1000, 500, 0, 0, 0, 0);
  odom.tick(1000000);
  checkTrue(odom.connected(), "tick 1: clean burst -- connected() true");
  checkTrue(odom.poseFresh(), "tick 1: poseFresh() true");
  Devices::PoseReading afterGood = odom.pose();

  // Tick 2 (kReadPeriodUs later, so the read is due): the combined 12-byte
  // burst read fails (induced status) -- the bogus 9999 values must NOT be
  // adopted.
  scriptPosVel(bus, 9999, 9999, 9999, 0, 0, 0, /*status=*/-1);
  odom.tick(1000000 + kReadPeriodUs);

  checkFalse(odom.connected(), "tick 2: induced failure -- connected() false");
  checkFalse(odom.poseFresh(), "tick 2: poseFresh() false -- must not be re-fused downstream");
  Devices::PoseReading afterFail = odom.pose();
  checkNear(afterFail.x, afterGood.x, 1e-6f, "tick 2: the failed burst's bogus reading must NOT overwrite pose().x");
  checkNear(afterFail.y, afterGood.y, 1e-6f, "tick 2: the failed burst's bogus reading must NOT overwrite pose().y");
  checkNear(afterFail.heading, afterGood.heading, 1e-6f, "tick 2: the failed burst's bogus reading must NOT overwrite pose().heading");

  // Tick 3: clean again -- proves the failure did not permanently latch
  // connected() false (always-retry semantics).
  scriptPosVel(bus, 1200, 600, 0, 0, 0, 0);
  odom.tick(1000000 + 2 * kReadPeriodUs);
  checkTrue(odom.connected(), "tick 3: a subsequent clean burst recovers connected() -- no permanent latch");
  checkTrue(odom.poseFresh(), "tick 3: poseFresh() recovers true");

  checkUintEq(bus.errCount(kAddr7), 1, "exactly the one induced failure in tick 2");
}

// 6. present() is a permanent, boot-time-only flag -- false before begin()
//    is ever called, false after a begin() whose detect fails, true after a
//    successful detect, and -- the whole point of the present()/connected()
//    split -- STAYS true even after a subsequent tick() whose own bus read
//    fails (only connected() tracks that).
void scenarioPresentTracksDetectionOnlyIndependentOfConnected() {
  beginScenario("present(): permanent boot-time detection flag, independent of connected()'s live per-tick health");
  Devices::I2CBus::setClock(1000000);

  // Case A: never begin()'d at all.
  Devices::I2CBus busNeverBegun;
  Devices::Otos odomNeverBegun(busNeverBegun, makeConfig(0.0f, 0.0f, 0.0f, 1.0f, 1.0f));
  checkFalse(odomNeverBegun.present(), "present() false -- begin() was never called");
  checkUintEq(busNeverBegun.errCount(kAddr7), 0, "no bus traffic at all when begin() is never called");

  // Case B: begin() called, product-ID probe returns the wrong id.
  Devices::I2CBus busWrongId;
  scriptGenerousWrites(busWrongId, 20);
  scriptProductId(busWrongId, 0x00);
  Devices::Otos odomWrongId(busWrongId, makeConfig(0.0f, 0.0f, 0.0f, 1.0f, 1.0f));
  odomWrongId.begin();
  checkFalse(odomWrongId.present(), "present() false -- begin()'s product-ID detect failed");

  // Case C: begin() succeeds -- present() true, and STAYS true across a
  // subsequent failed tick() (connected() itself goes false, present() must
  // not).
  Devices::I2CBus busPresent;
  scriptGenerousWrites(busPresent, 20);
  scriptProductId(busPresent, 0x5F);
  Devices::Otos odomPresent(busPresent, makeConfig(0.0f, 0.0f, 0.0f, 1.0f, 1.0f));
  odomPresent.begin();
  checkTrue(odomPresent.present(), "present() true -- begin()'s product-ID detect succeeded");

  scriptPosVel(busPresent, 9999, 9999, 9999, 0, 0, 0, /*status=*/-1);   // induced burst-read failure
  odomPresent.tick(1000000);
  checkFalse(odomPresent.connected(), "sanity: the induced failure DOES flip connected() false");
  checkTrue(odomPresent.present(),
            "present() stays true after a transient tick() failure -- must NOT track connected_");
  checkUintEq(busPresent.errCount(kAddr7), 1, "exactly the one induced failure");
}

// 7. The staged setPose() re-anchor cell: setPose() alone issues no bus
//    traffic; the NEXT tick() call drains it as exactly one write (the
//    anchor write), skipping this cycle's read (poseFresh() false, pose()
//    unchanged -- mirrors the pre-port file's own setPose(), which only
//    wrote registers and let a later tick() read confirm it); a subsequent
//    tick() then resumes the normal rate-limited read cycle.
void scenarioSetPoseStagedReanchorAppliesAtNextTick() {
  beginScenario("setPose(): stages a re-anchor request; tick() drains it as exactly one write");
  Devices::I2CBus::setClock(1000000);
  Devices::I2CBus bus;
  scriptGenerousWrites(bus, 20);
  scriptProductId(bus, 0x5F);

  Devices::Otos odom(bus, makeConfig(0.0f, 0.0f, 0.0f, 1.0f, 1.0f));
  odom.begin();

  // Establish a known-good cached pose first.
  scriptPosVel(bus, 1000, 500, 0, 0, 0, 0);
  odom.tick(1000000);
  Devices::PoseReading beforeReanchor = odom.pose();
  uint32_t base = bus.txnCount(kAddr7);

  odom.setPose(123.0f, -45.0f, 0.3f);
  checkUintEq(bus.txnCount(kAddr7) - base, 0, "setPose() stages only -- zero bus traffic");

  odom.tick(1000000 + kReadPeriodUs);   // due for a read, but the staged pose takes priority
  checkUintEq(bus.txnCount(kAddr7) - base, 1, "tick()'s drain issues exactly one write");
  checkFalse(odom.poseFresh(), "the drain tick performs no read -- poseFresh() false");
  Devices::PoseReading afterReanchor = odom.pose();
  checkNear(afterReanchor.x, beforeReanchor.x, 1e-6f, "pose() is unchanged by the drain itself (write-only)");
  checkNear(afterReanchor.y, beforeReanchor.y, 1e-6f, "pose() is unchanged by the drain itself (write-only)");

  // A later tick() resumes the normal read cycle.
  base = bus.txnCount(kAddr7);
  scriptPosVel(bus, 2000, -1000, 0, 0, 0, 0);
  odom.tick(1000000 + 2 * kReadPeriodUs);
  checkUintEq(bus.txnCount(kAddr7) - base, 2, "the following tick() issues a normal one-write + one-read burst");
  checkTrue(odom.poseFresh(), "the following tick() is fresh again");

  checkUintEq(bus.errCount(kAddr7), 0, "no script under-run");
}

// 8. Secondary primitives (setOffset()/getOffset() scaling round-trip,
//    setSignalProcessConfig()/signalProcessConfig(), imuCalibrationSamples-
//    Remaining(), resetTracking()) -- proves the full ported surface (not
//    just the tick()/readDue() hot path) still functions end to end after
//    the msg::Pose2D -> plain-float port.
void scenarioSecondaryPrimitivesRoundTrip() {
  beginScenario("secondary primitives: setOffset/getOffset, signal-cfg, imu-calib, resetTracking");
  Devices::I2CBus::setClock(1000000);
  Devices::I2CBus bus;
  scriptGenerousWrites(bus, 20);
  scriptProductId(bus, 0x5F);

  Devices::Otos odom(bus, makeConfig(10.0f, -5.0f, 0.0f, 1.0f, 1.0f));
  odom.begin();
  uint32_t base = bus.txnCount(kAddr7);

  odom.resetTracking();
  checkUintEq(bus.txnCount(kAddr7) - base, 1, "resetTracking() issues exactly one write");
  base = bus.txnCount(kAddr7);

  constexpr float kOffX = 42.7f, kOffY = -13.2f, kOffH = 0.15f;
  odom.setOffset(kOffX, kOffY, kOffH);
  checkUintEq(bus.txnCount(kAddr7) - base, 1, "setOffset() issues exactly one write");
  base = bus.txnCount(kAddr7);

  int16_t rx = static_cast<int16_t>(lroundf(kOffX / kPosMmPerLsb));
  int16_t ry = static_cast<int16_t>(lroundf(kOffY / kPosMmPerLsb));
  int16_t rh = static_cast<int16_t>(lroundf(kOffH / kHdgRadPerLsb));
  uint8_t raw[6] = {
      static_cast<uint8_t>(rx & 0xFF), static_cast<uint8_t>((rx >> 8) & 0xFF),
      static_cast<uint8_t>(ry & 0xFF), static_cast<uint8_t>((ry >> 8) & 0xFF),
      static_cast<uint8_t>(rh & 0xFF), static_cast<uint8_t>((rh >> 8) & 0xFF),
  };
  bus.scriptRead(kWireAddr, raw, 6, /*status=*/0);

  float gx = 0, gy = 0, gh = 0;
  odom.getOffset(gx, gy, gh);
  checkUintEq(bus.txnCount(kAddr7) - base, 2, "getOffset() issues exactly one write + one 6-byte read");
  checkNear(gx, kOffX, 0.5f, "getOffset().x round-trips within one LSB");
  checkNear(gy, kOffY, 0.5f, "getOffset().y round-trips within one LSB");
  checkNear(gh, kOffH, 1e-4f, "getOffset().h round-trips within one LSB");
  base = bus.txnCount(kAddr7);

  uint8_t signalRaw[1] = {0x0F};
  bus.scriptRead(kWireAddr, signalRaw, 1, /*status=*/0);
  uint8_t signalCfg = odom.signalProcessConfig();
  checkUintEq(bus.txnCount(kAddr7) - base, 2, "signalProcessConfig() issues exactly one write + one read");
  checkUintEq(signalCfg, 0x0F, "signalProcessConfig() returns the raw scripted byte unmodified");
  base = bus.txnCount(kAddr7);

  uint8_t imuRaw[1] = {37};
  bus.scriptRead(kWireAddr, imuRaw, 1, /*status=*/0);
  uint8_t imuRemaining = odom.imuCalibrationSamplesRemaining();
  checkUintEq(bus.txnCount(kAddr7) - base, 2, "imuCalibrationSamplesRemaining() issues exactly one write + one read");
  checkUintEq(imuRemaining, 37, "imuCalibrationSamplesRemaining() returns the raw scripted byte unmodified");

  checkUintEq(bus.errCount(kAddr7), 0, "no script under-run");
}

}  // namespace

int main() {
  scenarioProductIdGatesAllTraffic();
  scenarioReadDueRateLimitsRealReads();
  scenarioTickLeverArmOnlyTransform();
  scenarioTickMountingYawRotationOnlyTransform();
  scenarioLeverArmCancelsOnPureSpin();
  scenarioBurstFailureHoldsPriorPoseAndMarksStale();
  scenarioPresentTracksDetectionOnlyIndependentOfConnected();
  scenarioSetPoseStagedReanchorAppliesAtNextTick();
  scenarioSecondaryPrimitivesRoundTrip();

  if (g_failureCount == 0) {
    std::printf("OK: all Devices::Otos scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the Devices::Otos scenarios\n", g_failureCount);
  return 1;
}
