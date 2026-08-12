// app_odometry_harness.cpp -- off-hardware acceptance harness for
// Motion::Odometry (src/firm/motion/odometry.{h,cpp}). Proves:
// Odometry::integrate() accumulates world x/y/theta correctly for a
// straight-line case (equal wheel deltas) and a pure-rotation case
// (equal-and-opposite wheel deltas), reading the REAL Hardware::NezhaMotor
// leaves' own position() (no shadow copy), plus the pathLength() odometer
// semantics.
//
// (The former applyOtosSample() scenarios moved out: that base-side
// perception step is now a private RobotLoop method -- its OTOS behaviors
// are covered directly in devices_otos_harness.cpp and end-to-end in
// app_robot_loop_harness.cpp.)
//
// Reuses devices_motor_harness.cpp's NezhaMotor-scripting convention,
// duplicated here per this codebase's established per-harness-file fixture
// convention. Compiled by test_app_odometry.py with -DHOST_BUILD against
// odometry.cpp, nezha_motor.cpp, velocity_pid.cpp, sim_plant.cpp,
// wheel_plant.cpp, body_kinematics.cpp.
//
// Migrated by sprint 108 ticket 009 off the deleted src/firm/devices/
// i2c_bus_host.cpp scripted-FIFO Platform::I2CBus fake (ticket 001 reduced
// Platform::I2CBus to a pure interface and removed it) onto a
// TestSim::SimPlant scripted deterministically via TestSim::ScriptedI2CHook
// -- see devices_motor_harness.cpp's/scripted_i2c_hook.h's own header for
// the migration rationale. Every scenario below is otherwise UNCHANGED from
// the pre-migration harness -- only the bus/scripting plumbing moved.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include "hal/device_config.h"
#include "hal/device_types.h"
#include "hardware/nezha/nezha_motor.h"
#include "kinematics/differential.h"
#include "motion/odometry.h"
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

// --- Hardware::NezhaMotor scripting helpers (duplicated from
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

Hal::MotorConfig baseNezhaConfig(uint32_t port) {
  Hal::MotorConfig cfg;
  cfg.port = port;
  cfg.fwdSign = 1;
  cfg.wheelTravelCalib = 1.0f;
  return cfg;
}

void driveToPosition(Hardware::NezhaMotor& motor, TestSim::ScriptedI2CHook& bus,
                      uint16_t wireAddr, float positionMm, uint64_t nowUs) {
  scriptEncoderRequestCollect(bus, wireAddr, positionMm);
  motor.requestSample();
  motor.tick(nowUs);
}

// ===========================================================================
// 1. Straight-line case: equal wheel deltas -> theta unchanged, x
//    accumulates the common distance, y unchanged. Cross-checked against
//    Kinematics::Differential::forward()'s own direct output for the same deltas
//    (AC's "against Kinematics::Differential::forward()'s own known-correct output").
// ===========================================================================

void scenarioStraightLineAccumulatesDistanceNoHeadingChange() {
  beginScenario("Odometry::integrate(): straight-line (equal wheel deltas) accumulates x, leaves theta at 0");

  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  const uint16_t wireAddr = static_cast<uint16_t>(Hardware::kNezhaDeviceAddr << 1);

  Hardware::NezhaMotor left(plant, baseNezhaConfig(1));
  Hardware::NezhaMotor right(plant, baseNezhaConfig(2));

  const float trackWidth = 200.0f;  // [mm]
  Motion::Odometry odom(trackWidth, left.position(), right.position());

  // Both wheels advance the SAME 50mm -- vL == vR analog.
  driveToPosition(left, bus, wireAddr, 50.0f, 20000);
  driveToPosition(right, bus, wireAddr, 50.0f, 20000);
  odom.integrate(left.position(), right.position(), 0, 0);

  float expectedDist = 0.0f, expectedHeadingDelta = 0.0f;
  Kinematics::Differential::forward(50.0f, 50.0f, trackWidth, expectedDist, expectedHeadingDelta);
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
  const uint16_t wireAddr = static_cast<uint16_t>(Hardware::kNezhaDeviceAddr << 1);

  Hardware::NezhaMotor left(plant, baseNezhaConfig(1));
  Hardware::NezhaMotor right(plant, baseNezhaConfig(2));

  const float trackWidth = 200.0f;  // [mm]
  Motion::Odometry odom(trackWidth, left.position(), right.position());

  // Left goes -d, right goes +d -- vL == -vR analog. d is chosen exactly
  // representable at the leaf's own 0.1mm encoder-decode resolution
  // (scriptEncoderRequestCollect() round-trips positionMm through
  // lround(positionMm * 10) -- see devices_motor_harness.cpp's identical
  // convention) so the independent forward() oracle below isn't thrown off
  // by quantization the leaf itself would also apply on real hardware.
  const float d = 31.4f;  // [mm]
  driveToPosition(left, bus, wireAddr, -d, 20000);
  driveToPosition(right, bus, wireAddr, d, 20000);
  odom.integrate(left.position(), right.position(), 0, 0);

  float expectedDist = 0.0f, expectedHeadingDelta = 0.0f;
  Kinematics::Differential::forward(-d, d, trackWidth, expectedDist, expectedHeadingDelta);
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
  const uint16_t wireAddr = static_cast<uint16_t>(Hardware::kNezhaDeviceAddr << 1);

  Hardware::NezhaMotor left(plant, baseNezhaConfig(1));
  Hardware::NezhaMotor right(plant, baseNezhaConfig(2));

  // Advance both leaves to a nonzero position BEFORE constructing Odometry.
  driveToPosition(left, bus, wireAddr, 500.0f, 20000);
  driveToPosition(right, bus, wireAddr, 500.0f, 20000);

  Motion::Odometry odom(200.0f, left.position(), right.position());
  odom.integrate(left.position(), right.position(), 0, 0);  // no further motion since construction -- delta must be 0

  checkNear(odom.x(), 0.0f, 1e-3f, "x stays 0 -- the pre-existing leaf position is NOT counted as first-cycle motion");
  checkNear(odom.theta(), 0.0f, 1e-6f, "theta stays 0 for the same reason");
}

// ===========================================================================
// 3b. 116-003: pathLength() accumulates fabsf(distance) across every
//     integrate() call -- straight-line travel accumulates approximately the
//     true distance traveled; an in-place turn (zero net forward travel)
//     contributes approximately 0; reverse travel over the same ground still
//     accumulates positively (uses |distance|, so forward-then-reverse ADDS,
//     it doesn't cancel back toward 0 the way x()/y()/theta() would); and
//     reset() does NOT zero pathLength() (the chosen reset() interaction --
//     see odometry.h's pathLength() doc comment for the rationale).
// ===========================================================================

void scenarioPathLengthAccumulatesStraightLineTravel() {
  beginScenario("Odometry::pathLength(): straight-line travel accumulates approximately the true distance");

  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  const uint16_t wireAddr = static_cast<uint16_t>(Hardware::kNezhaDeviceAddr << 1);

  Hardware::NezhaMotor left(plant, baseNezhaConfig(1));
  Hardware::NezhaMotor right(plant, baseNezhaConfig(2));

  const float trackWidth = 200.0f;  // [mm]
  Motion::Odometry odom(trackWidth, left.position(), right.position());

  checkNear(odom.pathLength(), 0.0f, 1e-6f, "pathLength() starts at 0 before any integrate() call");

  // Both wheels advance the same 50mm, in two integrate() calls of 25mm
  // each, to also prove accumulation ACROSS calls (not just within one).
  // The second call's timestamp is spaced far enough past the first
  // (40000us, well under kMaxPlausibleStepSpeed's 1200mm/s implied-speed
  // gate for a 25mm step -- nezha_motor.cpp) that it registers as a fresh
  // sample rather than being rejected as an implausible glitch.
  driveToPosition(left, bus, wireAddr, 25.0f, 20000);
  driveToPosition(right, bus, wireAddr, 25.0f, 20000);
  odom.integrate(left.position(), right.position(), 0, 0);
  driveToPosition(left, bus, wireAddr, 50.0f, 60000);
  driveToPosition(right, bus, wireAddr, 50.0f, 60000);
  odom.integrate(left.position(), right.position(), 0, 0);

  checkNear(odom.pathLength(), 50.0f, 1e-3f, "pathLength() accumulates the true straight-line distance across two integrate() calls");
}

void scenarioPathLengthInPlaceTurnContributesApproximatelyZero() {
  beginScenario("Odometry::pathLength(): an in-place turn (zero net forward travel) contributes approximately 0");

  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  const uint16_t wireAddr = static_cast<uint16_t>(Hardware::kNezhaDeviceAddr << 1);

  Hardware::NezhaMotor left(plant, baseNezhaConfig(1));
  Hardware::NezhaMotor right(plant, baseNezhaConfig(2));

  const float trackWidth = 200.0f;  // [mm]
  Motion::Odometry odom(trackWidth, left.position(), right.position());

  // Left goes -d, right goes +d -- vL == -vR analog, same as the
  // pure-rotation scenario above: forward()'s distance output is exactly 0
  // for equal-and-opposite deltas, so pathLength_ accumulates fabsf(0) == 0.
  const float d = 31.4f;  // [mm]
  driveToPosition(left, bus, wireAddr, -d, 20000);
  driveToPosition(right, bus, wireAddr, d, 20000);
  odom.integrate(left.position(), right.position(), 0, 0);

  checkNear(odom.pathLength(), 0.0f, 1e-3f, "an in-place turn contributes ~0 to pathLength() -- zero net forward travel");
}

void scenarioPathLengthReverseTravelAccumulatesNotCancels() {
  beginScenario("Odometry::pathLength(): forward then reverse over the same ground ADDS, doesn't cancel");

  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  const uint16_t wireAddr = static_cast<uint16_t>(Hardware::kNezhaDeviceAddr << 1);

  Hardware::NezhaMotor left(plant, baseNezhaConfig(1));
  Hardware::NezhaMotor right(plant, baseNezhaConfig(2));

  const float trackWidth = 200.0f;  // [mm]
  Motion::Odometry odom(trackWidth, left.position(), right.position());

  // Forward 40mm, then back to 0 -- net displacement is 0 (x() returns to
  // ~0), but pathLength() must read ~80mm (40 out + 40 back), not ~0. The
  // second call's timestamp is spaced far enough past the first (60000us)
  // to clear kMaxPlausibleStepSpeed's implied-speed gate for a 40mm step
  // (nezha_motor.cpp) -- see the straight-line scenario's comment above.
  driveToPosition(left, bus, wireAddr, 40.0f, 20000);
  driveToPosition(right, bus, wireAddr, 40.0f, 20000);
  odom.integrate(left.position(), right.position(), 0, 0);
  driveToPosition(left, bus, wireAddr, 0.0f, 60000);
  driveToPosition(right, bus, wireAddr, 0.0f, 60000);
  odom.integrate(left.position(), right.position(), 0, 0);

  checkNear(odom.x(), 0.0f, 1e-3f, "sanity: net x() returns to ~0 after forward-then-reverse over the same ground");
  checkNear(odom.pathLength(), 80.0f, 1e-3f, "pathLength() accumulates |distance| -- forward+reverse ADDS to ~80mm, doesn't cancel to 0");
}

void scenarioPathLengthNotZeroedByReset() {
  beginScenario("Odometry::pathLength(): reset() does NOT zero pathLength() -- the chosen reset() interaction");

  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  const uint16_t wireAddr = static_cast<uint16_t>(Hardware::kNezhaDeviceAddr << 1);

  Hardware::NezhaMotor left(plant, baseNezhaConfig(1));
  Hardware::NezhaMotor right(plant, baseNezhaConfig(2));

  const float trackWidth = 200.0f;  // [mm]
  Motion::Odometry odom(trackWidth, left.position(), right.position());

  driveToPosition(left, bus, wireAddr, 60.0f, 20000);
  driveToPosition(right, bus, wireAddr, 60.0f, 20000);
  odom.integrate(left.position(), right.position(), 0, 0);
  checkNear(odom.pathLength(), 60.0f, 1e-3f, "setup: pathLength() accumulated 60mm before reset()");

  odom.reset(0.0f, 0.0f, 0.0f, left.position(), right.position());

  checkNear(odom.x(), 0.0f, 1e-6f, "sanity: reset() snaps x() to the given pose");
  checkNear(odom.pathLength(), 60.0f, 1e-3f, "reset() leaves pathLength() untouched -- it is NOT zeroed by reset()");

  // reset() also re-anchors the delta baseline -- confirm pathLength() keeps
  // accumulating correctly (no phantom jump) for motion AFTER the reset().
  driveToPosition(left, bus, wireAddr, 90.0f, 60000);
  driveToPosition(right, bus, wireAddr, 90.0f, 60000);
  odom.integrate(left.position(), right.position(), 0, 0);
  checkNear(odom.pathLength(), 90.0f, 1e-3f, "pathLength() continues accumulating correctly after reset() re-anchors the delta baseline");
}

// ===========================================================================
// 4. 131-004 (position-rebaseline-destroys-the-pose.md): a per-wheel
//    positionEpoch change re-anchors THAT wheel's delta baseline (crediting
//    zero delta for the rebaseline call), instead of differencing across
//    Core::RobotLoop::publishWheel()'s ~30,000mm software rebaseline jump.
//    Left and right are tracked independently -- one wheel rebaselining
//    does not perturb the other's normal diff the same cycle.
// ===========================================================================

void scenarioEpochChangeReAnchorsThatWheelOnlyLeavesTheOtherDiffingNormally() {
  beginScenario("Odometry::integrate(): a wheel's positionEpoch change re-anchors ONLY that wheel, "
                "crediting zero delta -- the other wheel keeps differencing normally");

  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  const uint16_t wireAddr = static_cast<uint16_t>(Hardware::kNezhaDeviceAddr << 1);

  Hardware::NezhaMotor left(plant, baseNezhaConfig(1));
  Hardware::NezhaMotor right(plant, baseNezhaConfig(2));

  const float trackWidth = 200.0f;  // [mm]
  Motion::Odometry odom(trackWidth, left.position(), right.position());

  // Ordinary cycle, epoch unchanged (0, 0): both wheels advance 40mm.
  driveToPosition(left, bus, wireAddr, 40.0f, 20000);
  driveToPosition(right, bus, wireAddr, 40.0f, 20000);
  odom.integrate(left.position(), right.position(), 0, 0);
  checkNear(odom.x(), 40.0f, 1e-3f, "ordinary cycle: x accumulates the real 40mm common delta");

  // Left wheel "rebaselines" -- Core::RobotLoop::publishWheel() re-anchors
  // Hal::Motor::position() near 0 the SAME cycle it bumps positionEpoch
  // (odometry.h's own file header); driveToPosition() here stands in for
  // that (this harness's real NezhaMotor leaves have no rebaseline concept
  // of their own -- Odometry only ever sees the (position, epoch) pair
  // RobotLoop hands it, regardless of how that pair came to be). Right
  // wheel continues its OWN normal travel (40 -> 70, a genuine +30mm) with
  // its epoch unchanged. Without the epoch-aware re-anchor this would
  // compute deltaLeft = 0 - 40 = -40 (a phantom -40mm on the left wheel,
  // the same class of corruption the real ~30,000mm jump produces, just
  // scaled down for a tractable test).
  driveToPosition(left, bus, wireAddr, 0.0f, 60000);
  driveToPosition(right, bus, wireAddr, 70.0f, 60000);
  odom.integrate(left.position(), right.position(), /*leftEpoch=*/1, /*rightEpoch=*/0);

  float expectedDist = 0.0f, expectedHeadingDelta = 0.0f;
  // deltaLeft credited as 0 (re-anchored), deltaRight the real +30mm.
  Kinematics::Differential::forward(0.0f, 30.0f, trackWidth, expectedDist, expectedHeadingDelta);
  // Independent midpoint-arc integration (odometry.cpp's own formula):
  // theta_ was exactly 0 before this call (the first cycle was straight),
  // so midTheta is just half this call's own headingDelta.
  const float midTheta = expectedHeadingDelta * 0.5f;
  const float expectedX = 40.0f + expectedDist * std::cos(midTheta);
  checkNear(odom.x(), expectedX, 1e-3f,
            "epoch-change cycle: x advances by ONLY the right wheel's real delta -- no phantom "
            "jump from the left wheel's rebaseline");
  checkNear(odom.theta(), expectedHeadingDelta, 1e-3f,
            "epoch-change cycle: theta reflects ONLY the real (0, +30) delta pair, not the raw "
            "(-40, +30) pair the rebaseline would otherwise have produced");

  // Next, ordinary cycle: left's baseline is now anchored at 0.0f (the
  // rebaselined value), so a genuine continuing move from there diffs
  // normally again -- no "catch-up" double-count of the eaten cycle.
  const float thetaBeforeNextCycle = odom.theta();
  driveToPosition(left, bus, wireAddr, 10.0f, 100000);
  driveToPosition(right, bus, wireAddr, 100.0f, 100000);
  odom.integrate(left.position(), right.position(), 1, 0);
  float nextDist = 0.0f, nextHeadingDelta = 0.0f;
  Kinematics::Differential::forward(10.0f, 30.0f, trackWidth, nextDist, nextHeadingDelta);  // 10-0, 100-70
  checkNear(odom.theta(), thetaBeforeNextCycle + nextHeadingDelta, 1e-3f,
            "post-rebaseline cycle: theta advances by the ordinary (10, 30) delta pair off the "
            "re-anchored baseline -- no phantom catch-up from the eaten cycle");
}

}  // namespace

int main() {
  scenarioStraightLineAccumulatesDistanceNoHeadingChange();
  scenarioPureRotationAccumulatesHeadingNoTranslation();
  scenarioBaselineSeededFromLeafPositionAtConstruction();
  scenarioPathLengthAccumulatesStraightLineTravel();
  scenarioPathLengthInPlaceTurnContributesApproximatelyZero();
  scenarioPathLengthReverseTravelAccumulatesNotCancels();
  scenarioPathLengthNotZeroedByReset();
  scenarioEpochChangeReAnchorsThatWheelOnlyLeavesTheOtherDiffingNormally();

  if (g_failureCount == 0) {
    std::printf("OK: all Motion::Odometry scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the Motion::Odometry scenarios\n", g_failureCount);
  return 1;
}
