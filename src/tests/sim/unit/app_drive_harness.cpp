// app_drive_harness.cpp -- off-hardware acceptance harness for ticket
// 103-006 (SUC-006), App::Drive (src/firm/app/drive.{h,cpp}). Proves:
// setDuty() stages the raw left/right values directly and tick() applies
// the last-staged target onto the two REAL Devices::NezhaMotor leaves, and
// stop() zeroes both targets within one cycle of the next
// NezhaMotor::tick().
//
// 128-015: App::Drive has no controller of its own at all -- duty is open
// loop from calibrated speed (drive.h's own file header); the interim,
// per-125-003 closed loop this file's header used to describe (two
// motion-local wheel-velocity-PID instances inside Drive) never survived
// past 122-002/125-002's own reshaping of Drive into a bare duty sink, and
// that PID class itself (a fully separate, zero-instantiation class from
// Motion::WheelPid/Planner::stageDuty()) is deleted outright this ticket --
// see src/motion/DESIGN.md's "wheel control generations" note. This
// harness's scenarios still isolate "did Drive's staged target reach
// appliedDuty() through a KNOWN deterministic computation," now simply
// Drive's own open-loop calibration (setDutyPerSpeed()) rather than a PID.
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
// open-loop calibration is what turns them into real duty before they
// reach the Motor leaves.
//
// Mirrors devices_motor_harness.cpp's own NezhaMotor-scripting helpers
// (scriptEncoderRequestCollect/baseNezhaConfig) -- duplicated here per this
// codebase's established per-harness-file fixture convention.
// Compiled by test_app_drive.py with -DHOST_BUILD against drive.cpp,
// nezha_motor.cpp, sim_plant.cpp, {wheel,otos}_plant.cpp,
// body_kinematics.cpp.
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
#include "firm/types/robot_state.h"
#include "scripted_i2c_hook.h"
#include "sim_plant.h"

namespace {

// 130-003: Drive::tick() now takes the whole Types::RobotState (so the
// forthcoming unified controller, ticket 004, can also read cmdAccel and
// each wheel's measured sections) rather than two loose speed floats. This
// harness's scenarios only ever need to stage a commanded velocity pair, so
// this helper is the one place that builds the RobotState tick() reads --
// every call site below stays exactly as readable as it was with the old
// two-float signature.
Types::RobotState wheelCmd(float vLeft, float vRight) {  // [mm/s] x2
  Types::RobotState state;
  state.wheelLeft.cmdVelocity = vLeft;
  state.wheelRight.cmdVelocity = vRight;
  return state;
}

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

// --- MockMotor (129-001) -------------------------------------------------
//
// A dependency-free Devices::Motor double, local to this harness, for the
// stop-re-assertion scenario below -- it needs direct, test-settable
// control over velocity() independent of any plant physics, which the real
// NezhaMotor leaves used by scenarios 1/2 above can't give without a much
// larger scripted-encoder sequence. Records setDuty() call COUNT (not just
// the last value, unlike the real leaves' appliedDuty()) -- that count is
// this scenario's own oracle for "did Drive actually re-issue the write,
// or did the quiet-at-zero shortcut swallow it."
class MockMotor : public Devices::Motor {
 public:
  void begin() override {}
  void requestSample() override {}
  void setDuty(float duty) override { lastDutyCmd = duty; ++setDutyCalls; }
  void setNeutral(Devices::Neutral) override {}
  void applyTravelCalib(float) override {}
  bool reconfigure(const Devices::MotorConfig&) override { return true; }
  void tick(uint64_t) override {}
  float position() const override { return 0.0f; }
  float velocity() const override { return mockVelocity_; }
  float appliedDuty() const override { return 0.0f; }
  bool connected() const override { return true; }
  uint64_t sampleTime() const override { return 0; }
  void resetPosition() override {}
  void rebaseline() override {}

  void setMockVelocity(float velocity) { mockVelocity_ = velocity; }  // [mm/s]

  int setDutyCalls = 0;
  float lastDutyCmd = 0.0f;

 private:
  float mockVelocity_ = 0.0f;  // [mm/s]
};

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
  beginScenario("Drive::tick(dutyL, dutyR): applies the duty pair to both leaves");

  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  const uint16_t wireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);

  Devices::NezhaMotor left(plant, baseNezhaConfig(1));
  Devices::NezhaMotor right(plant, baseNezhaConfig(2));
  primeAtZero(left, bus, wireAddr);
  primeAtZero(right, bus, wireAddr);

  const float trackWidth = 200.0f;  // [mm]
  App::Drive drive(left, right, trackWidth);

  drive.setDutyPerSpeed(1.0f, 1.0f);  // identity calibration: target IS duty
  const float dutyLeft = 0.18f;
  const float dutyRight = 0.06f;
  // tick() takes the commanded SPEEDS directly now (command-ingestion-ring-
  // buffered-comms-subsystem-routing-two-stops.md §4), via the whole
  // RobotState (130-003): the loop hands it whichever subsystem's targets
  // the blackboard carries, so there is one actuation path regardless of
  // who decided the motion.
  drive.tick(wheelCmd(dutyLeft, dutyRight));

  runOneCycleAtZeroPosition(left, bus, wireAddr, kPastWriteThrottleUs);
  runOneCycleAtZeroPosition(right, bus, wireAddr, kPastWriteThrottleUs);

  checkFloatEq(left.appliedDuty(), dutyLeft, "left appliedDuty() reflects the commanded duty");
  checkFloatEq(right.appliedDuty(), dutyRight, "right appliedDuty() reflects the commanded duty");
}

// ===========================================================================
// 2. stop(): both wheel targets reach 0 within one cycle of the next
//    NezhaMotor::tick() (AC #2), transitioning from a previously nonzero
//    staged target.
// ===========================================================================

void scenarioStopZeroesBothTargetsWithinOneCycle() {
  beginScenario("Drive::tick(0, 0): both wheels reach 0 duty after a nonzero pair");

  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  const uint16_t wireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);

  Devices::NezhaMotor left(plant, baseNezhaConfig(1));
  Devices::NezhaMotor right(plant, baseNezhaConfig(2));
  primeAtZero(left, bus, wireAddr);
  primeAtZero(right, bus, wireAddr);

  const float trackWidth = 200.0f;  // [mm]
  App::Drive drive(left, right, trackWidth);

  drive.setDutyPerSpeed(1.0f, 1.0f);
  // Drive a nonzero pair first, so the zero write below is a real
  // transition, not "duty was never nonzero to begin with."
  drive.tick(wheelCmd(0.12f, 0.09f));
  runOneCycleAtZeroPosition(left, bus, wireAddr, kPastWriteThrottleUs);
  runOneCycleAtZeroPosition(right, bus, wireAddr, kPastWriteThrottleUs);
  checkTrue(left.appliedDuty() != 0.0f, "setup: left duty is nonzero before the zero pair");
  checkTrue(right.appliedDuty() != 0.0f, "setup: right duty is nonzero before the zero pair");

  drive.tick(wheelCmd(0.0f, 0.0f));

  runOneCycleAtZeroPosition(left, bus, wireAddr, kPastWriteThrottleUs + 20000);
  runOneCycleAtZeroPosition(right, bus, wireAddr, kPastWriteThrottleUs + 20000);

  checkFloatEq(left.appliedDuty(), 0.0f, "left appliedDuty() reaches 0 within one cycle of stop()");
  checkFloatEq(right.appliedDuty(), 0.0f, "right appliedDuty() reaches 0 within one cycle of stop()");
}

// ===========================================================================
// 3. 129-001 (issue 07, the 2026-07-31 runaway): estop() arms a stop-re-
//    assertion window (kStopEnforceTicks, drive.h) -- tick() must keep
//    explicitly re-issuing the zero duty write to a wheel that is still
//    measurably moving (above kRestVelocity), bypassing the quiet-at-zero
//    shortcut that would otherwise silently stop calling setDuty() once
//    writtenLeft_/Right_ first reach 0. This is Drive's own mirror of
//    NezhaMotor's stopNotTaken write-on-change exemption
//    (devices_motor_harness.cpp's own dropped-stop-write scenario covers
//    that layer) -- "a stop is asserted until it is OBSERVED, not until it
//    is sent" (drive.h's own header).
// ===========================================================================

void scenarioEstopReassertsStopWhileWheelsStillMoving() {
  beginScenario("estop() re-asserts a commanded stop while a wheel is still measurably "
                "moving, bypassing the quiet-at-zero shortcut (129-001)");

  MockMotor left;
  MockMotor right;
  const float trackWidth = 200.0f;  // [mm]
  App::Drive drive(left, right, trackWidth);
  drive.setDutyPerSpeed(1.0f, 1.0f);  // identity calibration: target IS duty

  // Drive a nonzero pair first -- one real setDuty() call each, landing a
  // nonzero writtenLeft_/writtenRight_ baseline.
  drive.tick(wheelCmd(0.5f, 0.5f));
  checkTrue(left.setDutyCalls == 1 && right.setDutyCalls == 1,
            "setup: the nonzero pair actually reached both leaves");

  // ESTOP -- arms the re-assertion window. RobotLoop::handleEstop() zeroes
  // the blackboard's own cmdVelocity in the SAME cycle (robot_loop.cpp);
  // this harness drives that same contract directly via tick(0, 0).
  drive.estop();

  // The wheels are still coasting well above kRestVelocity (8 mm/s) -- every
  // tick() call while that holds must explicitly re-issue setDuty(0), not
  // silently take the quiet shortcut once writtenLeft_/Right_ first land at
  // 0 (which happens on the very first of these calls below).
  left.setMockVelocity(60.0f);
  right.setMockVelocity(60.0f);
  const int callsBeforeReassert = left.setDutyCalls;
  for (int i = 0; i < 5; ++i) {
    drive.tick(wheelCmd(0.0f, 0.0f));
  }
  checkTrue(left.setDutyCalls == callsBeforeReassert + 5,
            "each tick() while still moving re-issues setDuty(0) on the left leaf -- the "
            "quiet-at-zero shortcut never engages while wheelsMoving holds");
  checkTrue(right.setDutyCalls == callsBeforeReassert + 5,
            "same for the right leaf");

  // Once genuinely at rest AND the fixed-length countdown armed by estop()
  // has also fully elapsed (well past kStopEnforceTicks == 30 tick() calls
  // total since estop(), including the 5 above), the quiet-at-zero shortcut
  // resumes -- Drive stops re-calling setDuty() for a command that hasn't
  // changed, matching the ORIGINAL (pre-129-001) steady-state behavior.
  left.setMockVelocity(0.0f);
  right.setMockVelocity(0.0f);
  for (int i = 0; i < 40; ++i) {
    drive.tick(wheelCmd(0.0f, 0.0f));
  }
  const int callsAfterSettle = left.setDutyCalls;
  drive.tick(wheelCmd(0.0f, 0.0f));
  checkTrue(left.setDutyCalls == callsAfterSettle,
            "once at rest and the enforce window has elapsed, the quiet shortcut resumes on "
            "the left leaf");
  checkTrue(right.setDutyCalls == callsAfterSettle, "same for the right leaf");
}

}  // namespace

int main() {
  scenarioSetDutyStagesRawValues();
  scenarioStopZeroesBothTargetsWithinOneCycle();
  scenarioEstopReassertsStopWhileWheelsStillMoving();

  if (g_failureCount == 0) {
    std::printf("OK: all App::Drive scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the App::Drive scenarios\n", g_failureCount);
  return 1;
}
