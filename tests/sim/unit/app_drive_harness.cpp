// app_drive_harness.cpp -- off-hardware acceptance harness for ticket
// 103-006 (SUC-006), App::Drive (source/app/drive.{h,cpp}). Proves: tick()
// computes vL/vR via BodyKinematics::inverse() and stages them onto the two
// REAL Devices::NezhaMotor leaves via setVelocity() -- for BOTH directions
// (straight-line, equal-sign wheel targets; pure rotation, opposite-sign
// wheel targets) -- and stop() zeroes both targets within one cycle of the
// next NezhaMotor::tick().
//
// setVelocity()'s staged target isn't itself directly observable (it's a
// private NezhaMotor field) -- these scenarios drive the REAL leaf through
// exactly one more scripted request/collect + tick() cycle afterward and
// read back appliedDuty(), which the embedded PID computes deterministically
// from the staged target when kp=0/ki=0 (rawDuty = sign(target)*kff*|target|
// = kff*target -- velocity_pid.cpp's own compute()) against a KNOWN zero
// measured velocity (every cycle here reports the SAME encoder position, so
// filteredVelocity_ stays exactly 0). This isolates "did Drive stage the
// value inverse() computed" from the PID's own convergence behavior
// (already proved by devices_motor_harness.cpp's scenarioPidOnChasesVelocityTarget).
//
// Mirrors devices_motor_harness.cpp's own NezhaMotor-scripting helpers
// (scriptEncoderRequestCollect/baseNezhaConfig) -- duplicated here per this
// codebase's established per-harness-file fixture convention (see that
// file's own header note, and otos_odometer_harness.cpp's precedent).
// Compiled by test_app_drive.py with -DHOST_BUILD against drive.cpp,
// nezha_motor.cpp, velocity_pid.cpp, i2c_bus_host.cpp, body_kinematics.cpp.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include "app/drive.h"
#include "devices/device_config.h"
#include "devices/device_types.h"
#include "devices/i2c_bus.h"
#include "devices/nezha_motor.h"
#include "kinematics/body_kinematics.h"

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

void checkFloatEq(float actual, float expected, const std::string& what,
                   float tol = 1e-3f) {
  if (std::fabs(actual - expected) > tol) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %g, got %g", what.c_str(),
                  static_cast<double>(expected), static_cast<double>(actual));
    fail(buf);
  }
}

// --- Devices::NezhaMotor scripting helpers (duplicated from
// devices_motor_harness.cpp -- see this file's own header note) ----------

void scriptEncoderRequestCollect(Devices::I2CBus& bus, uint16_t wireAddr,
                                  float positionMm) {
  bus.scriptWrite(wireAddr, /*status=*/0);   // requestEncoder()'s 0x46 write
  bus.scriptWrite(wireAddr, /*status=*/0);   // slack: a possible same-cycle duty write (0x60)

  int32_t raw = static_cast<int32_t>(std::lround(positionMm * 10.0f));
  uint8_t data[4] = {
      static_cast<uint8_t>(raw & 0xFF),
      static_cast<uint8_t>((raw >> 8) & 0xFF),
      static_cast<uint8_t>((raw >> 16) & 0xFF),
      static_cast<uint8_t>((raw >> 24) & 0xFF),
  };
  bus.scriptRead(wireAddr, data, 4, /*status=*/0);   // collectEncoder()'s 4-byte read
}

// writeRawDuty()'s own write-rate limiter (nezha_motor.cpp) throttles any
// NON-stop write to at most one per 40000us since the leaf's last actual
// bus write (lastWriteTimeUs_ starts at 0) -- every scenario's single
// post-prime verification cycle below therefore runs at nowUs >= 50000 (a
// safe margin past that threshold) so the write under test is not silently
// dropped by the throttle. A stop write (pct == 0) is explicitly exempt
// (writeRawDuty()'s own `stopping` branch), so scenario 3's post-stop cycle
// does not need the same margin.
constexpr uint64_t kPastWriteThrottleUs = 50000;

Devices::MotorConfig baseNezhaConfig(uint32_t port) {
  Devices::MotorConfig cfg;
  cfg.port = port;
  cfg.fwdSign = 1;
  cfg.wheelTravelCalib = 1.0f;
  cfg.velFiltAlpha = 1.0f;  // no smoothing -- exact difference-quotient velocity
  // kp=0, ki=0 isolates the PID's proportional/feedforward term to a single
  // deterministic linear relation (rawDuty == kff * target, see file
  // header) so this harness can predict appliedDuty() exactly from
  // Drive's staged target without simulating multi-cycle convergence.
  cfg.velGains = Devices::Gains{/*kp=*/0.0f, /*ki=*/0.0f, /*kff=*/0.002f,
                                 /*iMax=*/1.0f, /*kaw=*/2.0f};
  cfg.velDeadband = 0.0f;
  return cfg;
}

// Primes a fresh leaf at position 0 (one request->collect cycle at nowUs=0)
// so lastPosition_/lastTickUs_ are established before any staged target is
// executed -- mirrors devices_motor_harness.cpp's own "prime cycle"
// convention.
void primeAtZero(Devices::NezhaMotor& motor, Devices::I2CBus& bus, uint16_t wireAddr) {
  scriptEncoderRequestCollect(bus, wireAddr, 0.0f);
  motor.requestSample();
  motor.tick(0);
}

// Runs one more request->collect + tick() cycle at the given time, holding
// position at 0 (so filteredVelocity_ stays exactly 0 -- isolates the
// staged target's effect on appliedDuty() from any plant/PID convergence
// dynamics).
void runOneCycleAtZeroPosition(Devices::NezhaMotor& motor, Devices::I2CBus& bus,
                                uint16_t wireAddr, uint64_t nowUs) {
  scriptEncoderRequestCollect(bus, wireAddr, 0.0f);
  motor.requestSample();
  motor.tick(nowUs);
}

// ===========================================================================
// 1. Straight-line twist (omega=0): both wheel targets equal and same sign
//    as v_x -- proves Drive::tick() stages inverse()'s vL/vR onto the two
//    leaves with no additional sign/scaling logic (AC #1).
// ===========================================================================

void scenarioStraightLineStagesEqualSameSignTargets() {
  beginScenario("Drive::tick(): straight twist stages equal, same-sign wheel targets via inverse()");

  Devices::I2CBus::setClock(1000000);
  Devices::I2CBus bus;
  const uint16_t wireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);

  Devices::NezhaMotor left(bus, baseNezhaConfig(1));
  Devices::NezhaMotor right(bus, baseNezhaConfig(2));
  primeAtZero(left, bus, wireAddr);
  primeAtZero(right, bus, wireAddr);

  const float trackWidth = 200.0f;  // [mm]
  App::Drive drive(left, right, trackWidth);

  const float v_x = 100.0f;    // [mm/s]
  const float omega = 0.0f;    // [rad/s]
  drive.setTwist(v_x, omega);
  drive.tick();

  runOneCycleAtZeroPosition(left, bus, wireAddr, kPastWriteThrottleUs);
  runOneCycleAtZeroPosition(right, bus, wireAddr, kPastWriteThrottleUs);

  float expectedVL = 0.0f, expectedVR = 0.0f;
  BodyKinematics::inverse(v_x, omega, trackWidth, expectedVL, expectedVR);
  checkFloatEq(expectedVL, 100.0f, "sanity: independent inverse() gives vL == 100 for a straight twist");
  checkFloatEq(expectedVR, 100.0f, "sanity: independent inverse() gives vR == 100 for a straight twist");

  const float kff = 0.002f;
  checkFloatEq(left.appliedDuty(), kff * expectedVL, "left appliedDuty() reflects the staged vL via kff (kp=ki=0)");
  checkFloatEq(right.appliedDuty(), kff * expectedVR, "right appliedDuty() reflects the staged vR via kff (kp=ki=0)");
  checkTrue(left.pidEnabled(), "left leaf's pidEnabled_ untouched by Drive -- stays default true");
  checkTrue(right.pidEnabled(), "right leaf's pidEnabled_ untouched by Drive -- stays default true");
}

// ===========================================================================
// 2. Pure-rotation twist (v_x=0): wheel targets are equal magnitude,
//    OPPOSITE sign -- proves signs work in both directions (AC #1).
// ===========================================================================

void scenarioPureRotationStagesOppositeSignTargets() {
  beginScenario("Drive::tick(): pure-rotation twist stages opposite-sign wheel targets via inverse()");

  Devices::I2CBus::setClock(1000000);
  Devices::I2CBus bus;
  const uint16_t wireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);

  Devices::NezhaMotor left(bus, baseNezhaConfig(1));
  Devices::NezhaMotor right(bus, baseNezhaConfig(2));
  primeAtZero(left, bus, wireAddr);
  primeAtZero(right, bus, wireAddr);

  const float trackWidth = 200.0f;  // [mm]
  App::Drive drive(left, right, trackWidth);

  const float v_x = 0.0f;      // [mm/s]
  const float omega = 0.5f;    // [rad/s] CCW-positive
  drive.setTwist(v_x, omega);
  drive.tick();

  runOneCycleAtZeroPosition(left, bus, wireAddr, kPastWriteThrottleUs);
  runOneCycleAtZeroPosition(right, bus, wireAddr, kPastWriteThrottleUs);

  float expectedVL = 0.0f, expectedVR = 0.0f;
  BodyKinematics::inverse(v_x, omega, trackWidth, expectedVL, expectedVR);
  checkFloatEq(expectedVL, -50.0f, "sanity: independent inverse() gives vL == -50 for this pure rotation");
  checkFloatEq(expectedVR, 50.0f, "sanity: independent inverse() gives vR == +50 for this pure rotation");

  const float kff = 0.002f;
  checkFloatEq(left.appliedDuty(), kff * expectedVL, "left appliedDuty() reflects the staged NEGATIVE vL");
  checkFloatEq(right.appliedDuty(), kff * expectedVR, "right appliedDuty() reflects the staged POSITIVE vR");
  checkTrue(left.appliedDuty() < 0.0f, "left duty is negative for this rotation direction");
  checkTrue(right.appliedDuty() > 0.0f, "right duty is positive for this rotation direction -- opposite sign from left");
}

// ===========================================================================
// 3. stop(): both wheel targets reach 0 within one cycle of the next
//    NezhaMotor::tick() (AC #2), transitioning from a previously nonzero
//    staged target.
// ===========================================================================

void scenarioStopZeroesBothTargetsWithinOneCycle() {
  beginScenario("Drive::stop(): both wheel targets reach 0 within one cycle of the next tick()");

  Devices::I2CBus::setClock(1000000);
  Devices::I2CBus bus;
  const uint16_t wireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);

  Devices::NezhaMotor left(bus, baseNezhaConfig(1));
  Devices::NezhaMotor right(bus, baseNezhaConfig(2));
  primeAtZero(left, bus, wireAddr);
  primeAtZero(right, bus, wireAddr);

  const float trackWidth = 200.0f;  // [mm]
  App::Drive drive(left, right, trackWidth);

  // Stage a nonzero twist and actually execute it once, so appliedDuty() is
  // demonstrably nonzero before stop() -- proves the transition, not just
  // "duty was never nonzero to begin with."
  drive.setTwist(100.0f, 0.0f);
  drive.tick();
  runOneCycleAtZeroPosition(left, bus, wireAddr, kPastWriteThrottleUs);
  runOneCycleAtZeroPosition(right, bus, wireAddr, kPastWriteThrottleUs);
  checkTrue(left.appliedDuty() != 0.0f, "setup: left duty is nonzero before stop()");
  checkTrue(right.appliedDuty() != 0.0f, "setup: right duty is nonzero before stop()");

  drive.stop();
  drive.tick();

  runOneCycleAtZeroPosition(left, bus, wireAddr, kPastWriteThrottleUs + 20000);
  runOneCycleAtZeroPosition(right, bus, wireAddr, kPastWriteThrottleUs + 20000);

  checkFloatEq(left.appliedDuty(), 0.0f, "left appliedDuty() reaches 0 within one cycle of stop()");
  checkFloatEq(right.appliedDuty(), 0.0f, "right appliedDuty() reaches 0 within one cycle of stop()");
}

}  // namespace

int main() {
  scenarioStraightLineStagesEqualSameSignTargets();
  scenarioPureRotationStagesOppositeSignTargets();
  scenarioStopZeroesBothTargetsWithinOneCycle();

  if (g_failureCount == 0) {
    std::printf("OK: all App::Drive scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the App::Drive scenarios\n", g_failureCount);
  return 1;
}
