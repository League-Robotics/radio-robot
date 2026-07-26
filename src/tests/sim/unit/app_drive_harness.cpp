// app_drive_harness.cpp -- off-hardware acceptance harness for ticket
// 103-006 (SUC-006), App::Drive (src/firm/app/drive.{h,cpp}). Proves:
// setDuty() stages the raw left/right values directly and tick() applies
// the last-staged target onto the two REAL Devices::NezhaMotor leaves, and
// stop() zeroes both targets within one cycle of the next
// NezhaMotor::tick().
//
// 125-003 (sprint 125 Decision 2, "protection vs. control" -- see
// sprint.md): Devices::NezhaMotor's embedded velocity PID is DELETED --
// App::Drive itself now holds the INTERIM closed loop (two
// Motion::WheelVelocityPid instances, drive.h's own header) that converts
// the staged mm/s target into real duty before calling
// Devices::Motor::setDuty(). This harness's own scenarios are updated
// in-place (not deleted): they still isolate "did Drive's staged target
// reach appliedDuty() through a KNOWN deterministic computation" from the
// PID's own convergence behavior, exactly as before -- only WHERE that
// computation runs changed (App::Drive::tick() instead of
// NezhaMotor::tick()). kp=0/ki=0 (this harness's own gains, applied via
// Drive::applyGainsLeft/Right()) isolates the feedforward-only term
// (rawDuty == kff * target), which is independent of both `dt` and the
// plant's `measured` velocity -- so this harness's assertions are
// numerically IDENTICAL to their pre-125-003 values, despite the
// relocation.
//
// 122-002 (motion-library extraction, ticket 2): App::Drive shrank to a bare
// wheel-target sink -- setDuty()/stop()/tick() only, implementing
// Motion::WheelSink. setTwist()/the BodyKinematics::inverse() staging this
// harness used to test moved to Motion::MoveQueue (which now calls
// BodyKinematics::inverse() itself and hands the result down through the
// sink) -- that coverage lives in app_move_queue_harness.cpp's own
// TWIST-Move scenarios.
//
// 125-002 (RETOOL, MECHANICAL RENAME ONLY): Motion::WheelSink's own contract
// retooled from a velocity sink to a duty sink -- setWheels() -> setDuty().
// "v_left"/"v_right" in the scenario names/locals below still describe what
// these values ARE today (raw mm/s velocities staged under the duty-shaped
// name), not yet real duty at the Drive::setDuty() boundary -- Drive's own
// interim closed loop (this file's own header) is what turns them into real
// duty before they reach the Motor leaves.
//
// Mirrors devices_motor_harness.cpp's own NezhaMotor-scripting helpers
// (scriptEncoderRequestCollect/baseNezhaConfig) -- duplicated here per this
// codebase's established per-harness-file fixture convention.
// Compiled by test_app_drive.py with -DHOST_BUILD against drive.cpp,
// nezha_motor.cpp, wheel_velocity_pid.cpp (src/motion/), sim_plant.cpp,
// {wheel,otos}_plant.cpp, body_kinematics.cpp.
//
// Migrated by sprint 108 ticket 009 off the deleted src/firm/devices/
// i2c_bus_host.cpp scripted-FIFO Devices::I2CBus fake (ticket 001 reduced
// Devices::I2CBus to a pure interface and removed it) onto a
// TestSim::SimPlant scripted deterministically via TestSim::ScriptedI2CHook
// -- see devices_motor_harness.cpp's/scripted_i2c_hook.h's own header for
// the migration rationale.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include "app/drive.h"
#include "devices/device_config.h"
#include "devices/nezha_motor.h"
#include "motion/wheel_velocity_pid.h"
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

// writeRawDuty()'s own write-rate limiter (nezha_motor.cpp) throttles any
// NON-stop write to at most one per 35000us (118 ticket 003's jitter
// margin, kMinWriteIntervalUs) since the leaf's last actual bus write
// (lastWriteTimeUs_ starts at 0) -- every scenario's single post-prime
// verification cycle below therefore runs at nowUs >= 50000 (a safe margin
// past that threshold) so the write under test is not silently dropped by
// the throttle. A stop write (pct == 0) is explicitly exempt
// (writeRawDuty()'s own `stopping` branch), so the stop() scenario's
// post-stop cycle does not need the same margin.
constexpr uint64_t kPastWriteThrottleUs = 50000;

Devices::MotorConfig baseNezhaConfig(uint32_t port) {
  Devices::MotorConfig cfg;
  cfg.port = port;
  cfg.fwdSign = 1;
  cfg.wheelTravelCalib = 1.0f;
  return cfg;
}

// kp=0, ki=0 isolates App::Drive's interim PID's proportional/integral term
// to a single deterministic linear relation (rawDuty == kff * target, see
// this file's own header) so this harness can predict appliedDuty() exactly
// from Drive's staged target without simulating multi-cycle convergence.
Motion::Gains ffOnlyGains() {
  Motion::Gains gains;
  gains.kp = 0.0f;
  gains.ki = 0.0f;
  gains.kff = 0.002f;
  gains.iMax = 1.0f;
  gains.kaw = 2.0f;
  return gains;
}

// Primes a fresh leaf at position 0 (one request->collect cycle at nowUs=0)
// so lastPosition_/lastTickUs_ are established before any staged target is
// executed -- mirrors devices_motor_harness.cpp's own "prime cycle"
// convention.
void primeAtZero(Devices::NezhaMotor& motor, TestSim::ScriptedI2CHook& bus, uint16_t wireAddr) {
  scriptEncoderRequestCollect(bus, wireAddr, 0.0f);
  motor.requestSample();
  motor.tick(0);
}

// Runs one more request->collect + tick() cycle at the given time, holding
// position at 0 (so velocity() stays exactly 0 -- isolates the staged
// target's effect on appliedDuty() from any plant convergence dynamics).
void runOneCycleAtZeroPosition(Devices::NezhaMotor& motor, TestSim::ScriptedI2CHook& bus,
                                uint16_t wireAddr, uint64_t nowUs) {
  scriptEncoderRequestCollect(bus, wireAddr, 0.0f);
  motor.requestSample();
  motor.tick(nowUs);
}

// ===========================================================================
// 1. setDuty() stages the raw v_left/v_right values directly (AC #1) --
//    the ONE staging path Drive has left post-122-002. tick() runs the
//    interim PID (this file's own header) and forwards the resulting duty
//    to Devices::Motor::setDuty().
// ===========================================================================

void scenarioSetDutyStagesRawValues() {
  beginScenario("Drive::setDuty(): stages raw v_left/v_right, tick() applies them via the interim PID");

  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  const uint16_t wireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);

  Devices::NezhaMotor left(plant, baseNezhaConfig(1));
  Devices::NezhaMotor right(plant, baseNezhaConfig(2));
  primeAtZero(left, bus, wireAddr);
  primeAtZero(right, bus, wireAddr);

  const float trackWidth = 200.0f;  // [mm]
  App::Drive drive(left, right, trackWidth);
  drive.applyGainsLeft(ffOnlyGains());
  drive.applyGainsRight(ffOnlyGains());

  const float vLeft = 90.0f;   // [mm/s]
  const float vRight = 30.0f;  // [mm/s]
  drive.setDuty(vLeft, vRight);
  drive.tick();

  runOneCycleAtZeroPosition(left, bus, wireAddr, kPastWriteThrottleUs);
  runOneCycleAtZeroPosition(right, bus, wireAddr, kPastWriteThrottleUs);

  const float kff = 0.002f;
  checkFloatEq(left.appliedDuty(), kff * vLeft, "left appliedDuty() reflects the RAW staged v_left");
  checkFloatEq(right.appliedDuty(), kff * vRight, "right appliedDuty() reflects the RAW staged v_right");
}

// ===========================================================================
// 2. stop(): both wheel targets reach 0 within one cycle of the next
//    NezhaMotor::tick() (AC #2), transitioning from a previously nonzero
//    staged target.
// ===========================================================================

void scenarioStopZeroesBothTargetsWithinOneCycle() {
  beginScenario("Drive::stop(): both wheel targets reach 0 within one cycle of the next tick()");

  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  const uint16_t wireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);

  Devices::NezhaMotor left(plant, baseNezhaConfig(1));
  Devices::NezhaMotor right(plant, baseNezhaConfig(2));
  primeAtZero(left, bus, wireAddr);
  primeAtZero(right, bus, wireAddr);

  const float trackWidth = 200.0f;  // [mm]
  App::Drive drive(left, right, trackWidth);
  drive.applyGainsLeft(ffOnlyGains());
  drive.applyGainsRight(ffOnlyGains());

  // Stage a nonzero target and actually execute it once, so appliedDuty() is
  // demonstrably nonzero before stop() -- proves the transition, not just
  // "duty was never nonzero to begin with."
  drive.setDuty(60.0f, 45.0f);
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
  scenarioSetDutyStagesRawValues();
  scenarioStopZeroesBothTargetsWithinOneCycle();

  if (g_failureCount == 0) {
    std::printf("OK: all App::Drive scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the App::Drive scenarios\n", g_failureCount);
  return 1;
}
