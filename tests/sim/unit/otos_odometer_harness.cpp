// otos_odometer_harness.cpp -- off-hardware acceptance harness for ticket
// 086-006 (SUC-005/SUC-006/SUC-007): exercises the REAL Hal::OtosOdometer
// leaf (source/hal/otos/otos_odometer.cpp) against ticket 001's HOST_BUILD
// scripted I2CBus fake -- no MicroBitI2C, no CODAL, no real hardware.
//
// Mirrors nezha_flipflop_harness.cpp's shape exactly (that file's own header
// comment is this ticket's explicit test precedent): compiles the ACTUAL
// source/hal/otos/otos_odometer.cpp against the SAME source/hal/otos/
// otos_odometer.h every ARM build compiles, with -DHOST_BUILD selecting
// i2c_bus_host.cpp's scripted fake in place of the real MicroBitI2C-backed
// i2c_bus.cpp. Hand-rolled assertions, PASS/FAIL per scenario, nonzero exit
// on any failure. Run by test_otos_odometer.py, which compiles and runs this
// binary via subprocess.
//
// --- Why these scenarios can't inspect exact written register bytes ---
// The HOST_BUILD scripted fake (i2c_bus_host.cpp) does not record a write()
// call's payload bytes at all (see its write()'s `(void)data; (void)len;`)
// -- only the ADDRESS and per-device txnCount()/errCount() are observable.
// So these scenarios prove behavior through txnCount() deltas (how many
// transactions a given call issued), connected()/pose() (the leaf's own
// observable state), and scripted READ payloads (which this fake DOES
// deliver back to the caller, letting these scenarios verify the
// read-side register-scaling + mounting-rotation + lever-arm math end to
// end) -- exactly the same split nezha_flipflop_harness.cpp's own header
// comment documents for the identical fake.
//
// --- 086-007 update ---
// The real-hardware HITL fix (086-007) changed tick()'s bus traffic shape:
// (1) the former two separate 6-byte readXYH() bursts (position, then
// velocity) are now ONE combined 12-byte burst read (position+velocity
// together -- readPositionVelocity()), so scenarios below script that one
// combined read (scriptPosVel()) instead of two scriptXYH() calls; (2) every
// bus_.write()/bus_.read() call in the real leaf now carries a non-zero
// preClear/postClear (kBusClearance = 4000us) -- this fake's write()/read()
// still don't expose the exact clearance argument passed, but DO enforce it
// behaviorally via the lastEnd/readyAt entry-spin (see i2c_bus_host.cpp),
// so a test that wanted to observe it could bracket a call with
// I2CBus::clock() before/after; these scenarios don't need to (txnCount()/
// pose() already prove the read sequencing and math), so none do; (3)
// tick()'s own real bus read is now rate-limited to kReadPeriod (20ms) --
// scenario 8 below (scenarioTickRateLimitsBusReads) covers this directly.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include "com/i2c_bus.h"
#include "config/boot_config.h"
#include "hal/lever_arm.h"
#include "hal/otos/otos_odometer.h"

namespace {

// --- Hand-rolled assertion plumbing (see nezha_flipflop_harness.cpp) ---

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

constexpr uint16_t kAddr7 = Hal::kOtosDeviceAddr;                         // 0x17
constexpr uint16_t kWireAddr = static_cast<uint16_t>(kAddr7 << 1);        // 0x2E

// Same LSB scale factors as otos_odometer.cpp -- duplicated here so this
// harness can construct scripted register payloads and compute the expected
// post-transform pose from first principles (this codebase's existing
// per-file convention for scale-constant duplication -- see otos_commands.cpp's
// own kCdegToRad comment).
constexpr float kPosMmPerLsb = 0.305f;
constexpr float kHdgRadPerLsb = 0.00549f * (3.14159265f / 180.0f);

Config::OtosBootConfig makeConfig(float offsetX, float offsetY, float offsetYaw,
                                   float linearScale, float angularScale) {
  Config::OtosBootConfig cfg;
  cfg.offsetX = offsetX;
  cfg.offsetY = offsetY;
  cfg.offsetYaw = offsetYaw;
  cfg.linearScale = linearScale;
  cfg.angularScale = angularScale;
  return cfg;
}

void scriptGenerousWrites(I2CBus& bus, int count) {
  for (int i = 0; i < count; ++i) bus.scriptWrite(kWireAddr, /*status=*/0);
}

void scriptProductId(I2CBus& bus, uint8_t id, int status = 0) {
  uint8_t data[1] = {id};
  bus.scriptRead(kWireAddr, data, 1, status);
}

// Queues one scripted 12-byte burst read (X_L X_H Y_L Y_H H_L H_H, then
// VX_L VX_H VY_L VY_H VH_L VH_H, all LE) for the next
// readPositionVelocity() call -- 086-007 combined the former two separate
// 6-byte position/velocity reads into this one 12-byte burst (the two
// register blocks are contiguous: kRegPositionXl then kRegVelocityXl).
void scriptPosVel(I2CBus& bus, int16_t x, int16_t y, int16_t h,
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

// --- Scenarios ----------------------------------------------------------

// 1. Successful product-ID detect: begin() runs its full init sequence and
//    the leaf reports connected().
void scenarioBeginDetectsAndInitializes() {
  beginScenario("begin(): product-ID match runs the full init sequence, connected() true");
  I2CBus::setClock(1000000);
  I2CBus bus;
  scriptGenerousWrites(bus, 20);
  scriptProductId(bus, 0x5F);

  Hal::OtosOdometer odom(bus, makeConfig(0.0f, 0.0f, 0.0f, 1.0f, 1.0f));
  odom.begin();

  checkTrue(odom.connected(), "connected() true after a successful detect+begin");
  checkUintEq(bus.txnCount(kAddr7), kBeginTxnCount,
              "begin() issued exactly the expected probe+init+scalar+zero-pose transactions");
  checkUintEq(bus.errCount(kAddr7), 0, "no script under-run");
}

// 2. Product-ID mismatch: begin() leaves the leaf uninitialized after only
//    the probe itself -- no init/scalar/zero-pose traffic follows.
void scenarioBeginProductIdMismatchStaysUninitialized() {
  beginScenario("begin(): product-ID mismatch -- no further bus traffic, connected() false");
  I2CBus::setClock(1000000);
  I2CBus bus;
  scriptGenerousWrites(bus, 20);
  scriptProductId(bus, 0x00);   // wrong id -- real chip reports 0x5F

  Hal::OtosOdometer odom(bus, makeConfig(0.0f, 0.0f, 0.0f, 1.0f, 1.0f));
  odom.begin();

  checkFalse(odom.connected(), "connected() false after a product-ID mismatch");
  checkUintEq(bus.txnCount(kAddr7), 2,
              "only the failed product-ID probe (1 write + 1 read) touched the bus");
  checkUintEq(bus.errCount(kAddr7), 0, "the probe's own status was OK -- just the ID didn't match");
}

// 3. Never begin()'d (or begin() never detected the chip): every primitive
//    setter AND tick() is a no-op -- zero bus traffic, matching the
//    never-detected gate every one of them shares. 092-003 extends this to
//    the new setOffset()/getOffset()/setSignalProcessConfig()/
//    signalProcessConfig()/imuCalibrationSamplesRemaining() primitives too.
void scenarioNeverInitializedEverySetterIsNoop() {
  beginScenario("never initialized: init/resetTracking/setPose/setLinearScalar/"
                "setAngularScalar/setOffset/getOffset/setSignalProcessConfig/"
                "signalProcessConfig/imuCalibrationSamplesRemaining/tick() are all no-ops");
  I2CBus::setClock(1000000);
  I2CBus bus;
  // No scripts queued at all -- any bus traffic would immediately surface as
  // a script-mismatch error, caught by the errCount() == 0 check below.

  Hal::OtosOdometer odom(bus, makeConfig(-47.7f, 3.5f, 0.0f, 1.067f, 0.987f));
  // Deliberately never call begin() -- initialized_ defaults false.

  odom.init();
  odom.resetTracking();
  odom.setPose(msg::Pose2D{});
  odom.setLinearScalar(50.0f);
  odom.setAngularScalar(-13.0f);
  odom.setOffset(msg::Pose2D{});
  msg::Pose2D offset = odom.getOffset();
  odom.setSignalProcessConfig(0x0F);
  uint8_t signalCfg = odom.signalProcessConfig();
  uint8_t imuRemaining = odom.imuCalibrationSamplesRemaining();
  odom.tick(1000);

  checkFalse(odom.connected(), "never initialized -- connected() stays false");
  checkUintEq(bus.txnCount(kAddr7), 0, "zero bus traffic from any primitive when never initialized");
  checkUintEq(bus.errCount(kAddr7), 0, "zero bus traffic means zero script-mismatch errors too");

  checkNear(offset.x, 0.0f, 1e-6f, "getOffset() returns a zero Pose2D when never initialized");
  checkNear(offset.y, 0.0f, 1e-6f, "getOffset() returns a zero Pose2D when never initialized");
  checkNear(offset.h, 0.0f, 1e-6f, "getOffset() returns a zero Pose2D when never initialized");
  checkUintEq(signalCfg, 0, "signalProcessConfig() returns 0 when never initialized");
  checkUintEq(imuRemaining, 0, "imuCalibrationSamplesRemaining() returns 0 when never initialized");

  msg::PoseEstimate pose = odom.pose();
  checkFalse(pose.stamp.valid, "pose().stamp.valid stays false with no successful tick() ever");
}

// 4. tick(): lever-arm-only transform (offsetYaw == 0, non-zero mounting
//    offset) -- the read-side position/velocity scaling plus
//    LeverArm::sensorToCentre() wiring, isolated from the mounting-yaw
//    rotation step (scenario 5 isolates that one instead).
void scenarioTickLeverArmOnlyTransform() {
  beginScenario("tick(): lever-arm-only transform matches LeverArm::sensorToCentre() directly");
  I2CBus::setClock(1000000);
  I2CBus bus;
  scriptGenerousWrites(bus, 20);
  scriptProductId(bus, 0x5F);

  constexpr float kOffsetX = -47.7f;   // [mm] tovez.json-realistic
  constexpr float kOffsetY = 3.5f;     // [mm]
  Hal::OtosOdometer odom(bus, makeConfig(kOffsetX, kOffsetY, 0.0f, 1.0f, 1.0f));
  odom.begin();

  constexpr int16_t kRx = 2000, kRy = 1000, kRh = 5217;      // position burst
  constexpr int16_t kRvx = 300, kRvy = -100, kRvh = 50;      // velocity burst
  scriptPosVel(bus, kRx, kRy, kRh, kRvx, kRvy, kRvh);

  odom.tick(2000);

  checkTrue(odom.connected(), "connected() true after a clean burst read");

  float xF = static_cast<float>(kRx) * kPosMmPerLsb;
  float yF = static_cast<float>(kRy) * kPosMmPerLsb;
  float hF = static_cast<float>(kRh) * kHdgRadPerLsb;
  float expectedCentreX = 0.0f, expectedCentreY = 0.0f;
  LeverArm::sensorToCentre(xF, yF, hF, kOffsetX, kOffsetY, expectedCentreX, expectedCentreY);

  msg::PoseEstimate pose = odom.pose();
  checkTrue(pose.stamp.valid, "pose().stamp.valid true after a clean tick()");
  checkUintEq(pose.stamp.last_upd, 2000, "pose().stamp.last_upd is this tick()'s now");
  checkNear(pose.pose.x, expectedCentreX, 1e-2f, "pose().pose.x matches LeverArm::sensorToCentre()");
  checkNear(pose.pose.y, expectedCentreY, 1e-2f, "pose().pose.y matches LeverArm::sensorToCentre()");
  checkNear(pose.pose.h, hF, 1e-5f, "pose().pose.h passes the raw heading through unmodified");

  float vxF = static_cast<float>(kRvx) * kPosMmPerLsb;
  float vyF = static_cast<float>(kRvy) * kPosMmPerLsb;
  float whF = static_cast<float>(kRvh) * kHdgRadPerLsb;
  checkNear(pose.twist.v_x, vxF, 1e-3f, "twist.v_x is the scaled velocity-register X (no mount rotation)");
  checkNear(pose.twist.v_y, vyF, 1e-3f, "twist.v_y is the scaled velocity-register Y (no mount rotation)");
  checkNear(pose.twist.omega, whF, 1e-6f, "twist.omega passes through unmodified");

  checkUintEq(bus.errCount(kAddr7), 0, "no script under-run");
}

// 5. tick(): mounting-yaw-rotation-only transform (zero mounting offset,
//    non-zero offsetYaw) -- isolates the rotation step from the lever arm.
void scenarioTickMountingYawRotationOnlyTransform() {
  beginScenario("tick(): mounting-yaw-rotation-only transform matches the rotation formula directly");
  I2CBus::setClock(1000000);
  I2CBus bus;
  scriptGenerousWrites(bus, 20);
  scriptProductId(bus, 0x5F);

  constexpr float kOffsetYaw = 0.3f;   // [rad] a hypothetical rotated mount (tovez.json's own is 0.0)
  Hal::OtosOdometer odom(bus, makeConfig(0.0f, 0.0f, kOffsetYaw, 1.0f, 1.0f));
  odom.begin();

  constexpr int16_t kRx = 1500, kRy = -800, kRh = 2000;
  scriptPosVel(bus, kRx, kRy, kRh, 0, 0, 0);   // velocity burst -- unused by this scenario's assertions

  odom.tick(3000);

  float xF = static_cast<float>(kRx) * kPosMmPerLsb;
  float yF = static_cast<float>(kRy) * kPosMmPerLsb;
  float hF = static_cast<float>(kRh) * kHdgRadPerLsb;
  float ang = -kOffsetYaw;
  float expectedX = cosf(ang) * xF - sinf(ang) * yF;   // zero offset -- lever-arm is a no-op
  float expectedY = sinf(ang) * xF + cosf(ang) * yF;

  msg::PoseEstimate pose = odom.pose();
  checkNear(pose.pose.x, expectedX, 1e-2f, "pose().pose.x reflects the mounting-yaw rotation");
  checkNear(pose.pose.y, expectedY, 1e-2f, "pose().pose.y reflects the mounting-yaw rotation");
  checkNear(pose.pose.h, hF, 1e-5f, "pose().pose.h still takes no mounting-rotation adjustment");

  checkUintEq(bus.errCount(kAddr7), 0, "no script under-run");
}

// 6. tick(): a burst-read failure holds the previously-cached pose but
//    marks stamp.valid false (so PoseEstimator::tick() skips fusion) and
//    flips connected() false -- then a THIRD, clean tick() proves the
//    failure did not permanently latch (always-retry semantics).
void scenarioTickFailureHoldsLastGoodPoseAndRecovers() {
  beginScenario("tick(): a burst-read failure holds the last-good pose, marks it stale, and recovers");
  I2CBus::setClock(1000000);
  I2CBus bus;
  scriptGenerousWrites(bus, 20);
  scriptProductId(bus, 0x5F);

  Hal::OtosOdometer odom(bus, makeConfig(0.0f, 0.0f, 0.0f, 1.0f, 1.0f));
  odom.begin();

  // Tick 1: clean burst -- establishes a known-good cached pose.
  scriptPosVel(bus, 1000, 500, 0, 0, 0, 0);
  odom.tick(1000);
  checkTrue(odom.connected(), "tick 1: clean burst -- connected() true");
  msg::PoseEstimate afterGood = odom.pose();
  checkTrue(afterGood.stamp.valid, "tick 1: stamp.valid true");

  // Tick 2: the combined 12-byte burst read fails (induced status) --
  // 086-007 combined position+velocity into ONE read, so a single induced
  // failure status now covers what used to be "position ok, velocity
  // fails" -- the whole burst is treated as failed either way.
  scriptPosVel(bus, 9999, 9999, 9999, 0, 0, 0, /*status=*/-1);   // would-be-fresh values, must NOT be adopted
  odom.tick(2000);

  checkFalse(odom.connected(), "tick 2: induced failure -- connected() false");
  msg::PoseEstimate afterFail = odom.pose();
  checkFalse(afterFail.stamp.valid, "tick 2: stamp.valid false -- PoseEstimator must skip fusion");
  checkNear(afterFail.pose.x, afterGood.pose.x, 1e-6f,
            "tick 2: the failed burst's bogus 9999 reading must NOT overwrite the held pose.x");
  checkNear(afterFail.pose.y, afterGood.pose.y, 1e-6f,
            "tick 2: the failed burst's bogus 9999 reading must NOT overwrite the held pose.y");

  // Tick 3: clean again -- proves the failure did not permanently latch
  // connected() false (always-retry semantics, mirrors Hal::NezhaMotor).
  scriptPosVel(bus, 1200, 600, 0, 0, 0, 0);
  odom.tick(3000);
  checkTrue(odom.connected(), "tick 3: a subsequent clean burst recovers connected() -- no permanent latch");
  checkTrue(odom.pose().stamp.valid, "tick 3: stamp.valid recovers true");

  checkUintEq(bus.errCount(kAddr7), 1, "exactly the one induced failure in tick 2");
}

// 7. setPose()/setLinearScalar()/setAngularScalar()/resetTracking()/
//    setOffset()/setSignalProcessConfig(): each issues exactly one write
//    when initialized (txnCount delta), zero when not (already covered by
//    scenario 3 for the not-initialized side).
void scenarioSetterTxnCounts() {
  beginScenario("setPose()/setLinearScalar()/setAngularScalar()/resetTracking()/"
                "setOffset()/setSignalProcessConfig(): one write each");
  I2CBus::setClock(1000000);
  I2CBus bus;
  scriptGenerousWrites(bus, 20);
  scriptProductId(bus, 0x5F);

  Hal::OtosOdometer odom(bus, makeConfig(10.0f, -5.0f, 0.0f, 1.0f, 1.0f));
  odom.begin();
  uint32_t base = bus.txnCount(kAddr7);

  odom.setPose(msg::Pose2D{});
  checkUintEq(bus.txnCount(kAddr7) - base, 1, "setPose() issues exactly one write");
  base = bus.txnCount(kAddr7);

  odom.setLinearScalar(50.0f);
  checkUintEq(bus.txnCount(kAddr7) - base, 1, "setLinearScalar() issues exactly one write");
  base = bus.txnCount(kAddr7);

  odom.setAngularScalar(-13.0f);
  checkUintEq(bus.txnCount(kAddr7) - base, 1, "setAngularScalar() issues exactly one write");
  base = bus.txnCount(kAddr7);

  odom.resetTracking();
  checkUintEq(bus.txnCount(kAddr7) - base, 1, "resetTracking() issues exactly one write");
  base = bus.txnCount(kAddr7);

  odom.setOffset(msg::Pose2D{});
  checkUintEq(bus.txnCount(kAddr7) - base, 1, "setOffset() issues exactly one write");
  base = bus.txnCount(kAddr7);

  odom.setSignalProcessConfig(0x0F);
  checkUintEq(bus.txnCount(kAddr7) - base, 1, "setSignalProcessConfig() issues exactly one write");

  checkUintEq(bus.errCount(kAddr7), 0, "no script under-run");
}

// 8. tick(): 086-007 rate limiting -- a second tick() call within
//    kReadPeriod of the last REAL bus read issues NO further bus traffic
//    and marks the cached sample stale (so PoseEstimator does not re-fuse
//    the same reading every main-loop pass); a subsequent tick() at/after
//    the period boundary is due again and issues a fresh real read.
void scenarioTickRateLimitsBusReads() {
  beginScenario("tick(): rate-limits real bus reads to kReadPeriod (086-007)");
  I2CBus::setClock(1000000);
  I2CBus bus;
  scriptGenerousWrites(bus, 20);
  scriptProductId(bus, 0x5F);

  Hal::OtosOdometer odom(bus, makeConfig(0.0f, 0.0f, 0.0f, 1.0f, 1.0f));
  odom.begin();

  // Mirrors otos_odometer.h's private kReadPeriod (086-007) -- duplicated
  // here the same way kPosMmPerLsb/kHdgRadPerLsb already are (this file's
  // own established convention for restating a private leaf constant).
  constexpr uint32_t kReadPeriodMs = 20;

  // Tick 1 at now=1000: hasRead_ starts false -- always issues a real read
  // regardless of kReadPeriod.
  scriptPosVel(bus, 1000, 500, 0, 300, -100, 50);
  odom.tick(1000);
  checkTrue(odom.pose().stamp.valid, "tick 1: real read -- stamp.valid true");
  uint32_t txnAfterTick1 = bus.txnCount(kAddr7);

  // Tick 2 at now=1000+kReadPeriodMs-1 (just inside the window): too soon
  // -- must issue ZERO further bus traffic and mark the sample stale.
  odom.tick(1000 + kReadPeriodMs - 1);
  checkUintEq(bus.txnCount(kAddr7), txnAfterTick1, "tick 2 (too soon): issues NO bus traffic");
  checkFalse(odom.pose().stamp.valid, "tick 2 (too soon): stamp.valid false -- stale, not re-fused");

  // Tick 3 at now=1000+kReadPeriodMs (exactly at the boundary, not "<"
  // kReadPeriod): due again -- issues a fresh real read (one write + one
  // 12-byte read -- txnCount delta of 2).
  scriptPosVel(bus, 1100, 550, 0, 300, -100, 50);
  odom.tick(1000 + kReadPeriodMs);
  checkTrue(odom.pose().stamp.valid, "tick 3 (period elapsed): real read -- stamp.valid true");
  checkUintEq(bus.txnCount(kAddr7) - txnAfterTick1, 2,
              "tick 3: issues exactly one write + one read (the combined burst)");

  checkUintEq(bus.errCount(kAddr7), 0, "no script under-run");
}

// 9. setOffset()/getOffset() (092-003, SUC-003): register scaling round
//    trip -- setOffset() writes a known offset, a scripted read-back
//    (encoded with the SAME kPosMmPerLsb/kHdgRadPerLsb math setOffset()
//    itself uses) proves getOffset() correctly inverts it. Also proves the
//    expected txn shape: setOffset() is one write; getOffset() is one
//    write (register-select) + one 6-byte read.
void scenarioSetOffsetGetOffsetScalingRoundTrip() {
  beginScenario("setOffset()/getOffset(): REG_OFFSET register scaling round-trip");
  I2CBus::setClock(1000000);
  I2CBus bus;
  scriptGenerousWrites(bus, 20);
  scriptProductId(bus, 0x5F);

  Hal::OtosOdometer odom(bus, makeConfig(0.0f, 0.0f, 0.0f, 1.0f, 1.0f));
  odom.begin();
  uint32_t base = bus.txnCount(kAddr7);

  msg::Pose2D offset;
  offset.x = 42.7f;    // [mm]
  offset.y = -13.2f;   // [mm]
  offset.h = 0.15f;    // [rad]

  odom.setOffset(offset);
  checkUintEq(bus.txnCount(kAddr7) - base, 1, "setOffset() issues exactly one write");
  base = bus.txnCount(kAddr7);

  // Script a read-back encoded with the EXACT SAME round-to-nearest-LSB math
  // setOffset() itself performs (kPosMmPerLsb/kHdgRadPerLsb) -- proves
  // getOffset() decodes REG_OFFSET's int16 triple with the inverse of that
  // SAME scaling (Decision 6: same helpers, same scale as position).
  int16_t rx = static_cast<int16_t>(lroundf(offset.x / kPosMmPerLsb));
  int16_t ry = static_cast<int16_t>(lroundf(offset.y / kPosMmPerLsb));
  int16_t rh = static_cast<int16_t>(lroundf(offset.h / kHdgRadPerLsb));
  uint8_t raw[6] = {
      static_cast<uint8_t>(rx & 0xFF), static_cast<uint8_t>((rx >> 8) & 0xFF),
      static_cast<uint8_t>(ry & 0xFF), static_cast<uint8_t>((ry >> 8) & 0xFF),
      static_cast<uint8_t>(rh & 0xFF), static_cast<uint8_t>((rh >> 8) & 0xFF),
  };
  bus.scriptRead(kWireAddr, raw, 6, /*status=*/0);

  msg::Pose2D readBack = odom.getOffset();
  checkUintEq(bus.txnCount(kAddr7) - base, 2, "getOffset() issues exactly one write + one 6-byte read");

  checkNear(readBack.x, offset.x, 0.5f, "getOffset().x round-trips within one LSB (~0.3mm)");
  checkNear(readBack.y, offset.y, 0.5f, "getOffset().y round-trips within one LSB");
  checkNear(readBack.h, offset.h, 1e-4f, "getOffset().h round-trips within one LSB");

  checkUintEq(bus.errCount(kAddr7), 0, "no script under-run");
}

// 10. signalProcessConfig()/imuCalibrationSamplesRemaining() (092-003,
//     SUC-003): each is a plain one-write + one-read register access
//     (register-select then a single-byte read), returning the raw byte
//     the fake scripts back -- no scaling, so this proves the pass-through
//     is exact, not just the txn shape.
void scenarioSignalProcessConfigAndImuCalibrationProgressReads() {
  beginScenario("signalProcessConfig()/imuCalibrationSamplesRemaining(): raw register read-back");
  I2CBus::setClock(1000000);
  I2CBus bus;
  scriptGenerousWrites(bus, 20);
  scriptProductId(bus, 0x5F);

  Hal::OtosOdometer odom(bus, makeConfig(0.0f, 0.0f, 0.0f, 1.0f, 1.0f));
  odom.begin();
  uint32_t base = bus.txnCount(kAddr7);

  uint8_t signalRaw[1] = {0x0F};
  bus.scriptRead(kWireAddr, signalRaw, 1, /*status=*/0);
  uint8_t signalCfg = odom.signalProcessConfig();
  checkUintEq(bus.txnCount(kAddr7) - base, 2, "signalProcessConfig() issues exactly one write + one read");
  checkUintEq(signalCfg, 0x0F, "signalProcessConfig() returns the raw scripted byte unmodified");
  base = bus.txnCount(kAddr7);

  uint8_t imuRaw[1] = {37};
  bus.scriptRead(kWireAddr, imuRaw, 1, /*status=*/0);
  uint8_t imuRemaining = odom.imuCalibrationSamplesRemaining();
  checkUintEq(bus.txnCount(kAddr7) - base, 2,
              "imuCalibrationSamplesRemaining() issues exactly one write + one read");
  checkUintEq(imuRemaining, 37, "imuCalibrationSamplesRemaining() returns the raw scripted byte unmodified");

  checkUintEq(bus.errCount(kAddr7), 0, "no script under-run");
}

}  // namespace

int main() {
  scenarioBeginDetectsAndInitializes();
  scenarioBeginProductIdMismatchStaysUninitialized();
  scenarioNeverInitializedEverySetterIsNoop();
  scenarioTickLeverArmOnlyTransform();
  scenarioTickMountingYawRotationOnlyTransform();
  scenarioTickFailureHoldsLastGoodPoseAndRecovers();
  scenarioSetterTxnCounts();
  scenarioTickRateLimitsBusReads();
  scenarioSetOffsetGetOffsetScalingRoundTrip();
  scenarioSignalProcessConfigAndImuCalibrationProgressReads();

  if (g_failureCount == 0) {
    std::printf("OK: all OtosOdometer scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the OtosOdometer scenarios\n", g_failureCount);
  return 1;
}
