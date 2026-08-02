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
#include <utility>
#include <vector>

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

// ===========================================================================
// 4-6. 130-004 (wheel-speed-controller-moves-into-drive.md Phase 2): Stage
// C's bias adaptation -- convergence under a known plant-gain error, the
// biasMax clamp holding against a divergent error, and bumpless (duty-
// continuous) transfer across a bias update. All three drive App::Drive
// directly, one cycle at a time, hand-simulating a plant with a KNOWN
// multiplicative gain error on top of an identity Stage A calibration
// (setDutyPerSpeed(1,1), the SAME "identity calibration: target IS duty"
// convention scenario 1 above already establishes for this harness) --
// duty == corrected command == cmd + bias, so the arithmetic is legible
// without a real NezhaMotor/SimPlant round trip. state.wheelLeft.velocity
// is hand-set each tick to `plantGain * (cmd + bias_from_previous_tick)`,
// modeling a one-cycle-delayed plant response consistent with how every
// other value tick() reads is "as of last cycle" (see tick()'s own doc
// comment, drive.h). ControlGains (Stage B) stay all-zero throughout, in
// isolation from Stage C.
// ===========================================================================

void scenarioBiasConvergesUnderKnownPlantGainError() {
  beginScenario("Stage C: bias converges to close a known plant-gain error at cruise");

  MockMotor left;
  MockMotor right;
  const float trackWidth = 200.0f;  // [mm]
  App::Drive drive(left, right, trackWidth);
  drive.setDutyPerSpeed(1.0f, 1.0f);  // identity: duty == corrected command

  App::Drive::AdaptationBounds bounds;
  bounds.biasMax = 60.0f;    // [mm/s] comfortably above the 50 needed to close the gap
  bounds.tauAdapt = 0.5f;    // [s] fast enough to converge within this test's tick budget
  bounds.aSteady = 100.0f;   // [mm/s^2] cmdAccel is always 0 below -- always "steady"
  drive.setAdaptationBounds(bounds);
  // ControlGains left at the all-zero default -- isolates Stage C from Stage B.

  const float cmd = 200.0f;      // [mm/s] commanded cruise speed
  const float plantGain = 0.8f;  // the known error: actual = plantGain * duty

  uint32_t now = 1000;
  float measured = plantGain * cmd;  // first tick's measured value, no bias adapted yet
  for (int i = 0; i < 400; ++i) {
    Types::RobotState state;
    state.wheelLeft.cmdVelocity = cmd;
    state.wheelLeft.velocity = measured;
    state.wheelLeft.connected = true;
    state.wheelLeft.sampleTime = now - 10;  // 10ms old -- comfortably fresh
    state.time.cycleStart = now;
    state.time.cyclePeriod = 50000;  // [us] -- dt = 0.05s
    drive.tick(state);

    // The plant's own response to the duty this tick just computed,
    // consumed as next tick's measured velocity.
    measured = plantGain * (cmd + drive.biasLeft());
    now += 50;
  }

  // bias should converge to cmd*(1/plantGain - 1) = 200*(1.25-1) = 50 --
  // the exact correction that makes plantGain*(cmd+bias) == cmd.
  checkFloatEq(drive.biasLeft(), cmd * (1.0f / plantGain - 1.0f),
              "bias converges to close the known plant-gain error", 2.0f);
  checkFloatEq(measured, cmd, "measured velocity converges to the commanded cruise speed", 5.0f);
}

void scenarioBiasClampHoldsAgainstDivergentError() {
  beginScenario("Stage C: the biasMax clamp holds against an error too large to fully close");

  MockMotor left;
  MockMotor right;
  const float trackWidth = 200.0f;  // [mm]
  App::Drive drive(left, right, trackWidth);
  drive.setDutyPerSpeed(1.0f, 1.0f);

  App::Drive::AdaptationBounds bounds;
  bounds.biasMax = 60.0f;   // [mm/s]
  bounds.tauAdapt = 0.2f;   // [s] deliberately fast, to reach the clamp quickly
  bounds.aSteady = 100.0f;  // [mm/s^2]
  drive.setAdaptationBounds(bounds);

  const float cmd = 200.0f;
  const float plantGain = 0.5f;  // needs bias=200 to fully close -- unreachable under biasMax=60

  uint32_t now = 1000;
  float measured = plantGain * cmd;
  for (int i = 0; i < 400; ++i) {
    Types::RobotState state;
    state.wheelLeft.cmdVelocity = cmd;
    state.wheelLeft.velocity = measured;
    state.wheelLeft.connected = true;
    state.wheelLeft.sampleTime = now - 10;
    state.time.cycleStart = now;
    state.time.cyclePeriod = 50000;
    drive.tick(state);

    checkTrue(drive.biasLeft() <= bounds.biasMax + 1e-3f,
             "bias never exceeds +biasMax, even under a persistently divergent error");
    measured = plantGain * (cmd + drive.biasLeft());
    now += 50;
  }
  checkFloatEq(drive.biasLeft(), bounds.biasMax,
              "bias settles PINNED at the clamp (not partway, not oscillating)", 0.5f);

  // Same clamp, the other sign: a plant that over-delivers needs a
  // NEGATIVE bias to correct, and that clamp must hold too.
  MockMotor left2;
  MockMotor right2;
  App::Drive drive2(left2, right2, trackWidth);
  drive2.setDutyPerSpeed(1.0f, 1.0f);
  drive2.setAdaptationBounds(bounds);
  const float overDeliverGain = 2.0f;  // needs bias=-100 to fully close -- unreachable
  now = 1000;
  measured = overDeliverGain * cmd;
  for (int i = 0; i < 400; ++i) {
    Types::RobotState state;
    state.wheelLeft.cmdVelocity = cmd;
    state.wheelLeft.velocity = measured;
    state.wheelLeft.connected = true;
    state.wheelLeft.sampleTime = now - 10;
    state.time.cycleStart = now;
    state.time.cyclePeriod = 50000;
    drive2.tick(state);

    checkTrue(drive2.biasLeft() >= -bounds.biasMax - 1e-3f,
             "bias never exceeds -biasMax on the negative side either");
    measured = overDeliverGain * (cmd + drive2.biasLeft());
    now += 50;
  }
  checkFloatEq(drive2.biasLeft(), -bounds.biasMax,
              "bias settles PINNED at the negative clamp", 0.5f);
}

void scenarioBumplessTransferAcrossBiasUpdate() {
  beginScenario("Stage C: duty is continuous (bumpless) across a bias update");

  MockMotor left;
  MockMotor right;
  const float trackWidth = 200.0f;  // [mm]
  App::Drive drive(left, right, trackWidth);
  drive.setDutyPerSpeed(1.0f, 1.0f);

  App::Drive::AdaptationBounds bounds;
  bounds.biasMax = 60.0f;
  bounds.tauAdapt = 5.0f;   // [s] slow -- bias moves by a small amount each tick
  bounds.aSteady = 100.0f;  // [mm/s^2]
  drive.setAdaptationBounds(bounds);

  const float cmd = 200.0f;
  const float plantGain = 0.8f;

  uint32_t now = 1000;
  float measured = plantGain * cmd;
  float previousDuty = 0.0f;
  bool first = true;
  for (int i = 0; i < 100; ++i) {
    Types::RobotState state;
    state.wheelLeft.cmdVelocity = cmd;
    state.wheelLeft.velocity = measured;
    state.wheelLeft.connected = true;
    state.wheelLeft.sampleTime = now - 10;
    state.time.cycleStart = now;
    state.time.cyclePeriod = 50000;
    drive.tick(state);

    if (!first) {
      checkTrue(std::fabs(left.lastDutyCmd - previousDuty) < 5.0f,
               "duty steps by only a small amount across one tick's bias update -- no bump");
    }
    first = false;
    previousDuty = left.lastDutyCmd;
    measured = plantGain * (cmd + drive.biasLeft());
    now += 50;
  }
  // The bias genuinely moved over the run (not a trivially-passing test
  // where nothing changed at all).
  checkTrue(drive.biasLeft() > 1.0f, "bias actually adapted over the run (not a no-op check)");
}

// ===========================================================================
// 7. 130-005: Stage B's steady gate and anti-windup -- the coverage
// Motion::WheelTrim's own deleted test suite (wheel_trim_test.cpp's
// testIntegratorEngagesOnlyInHold()/testUnitBehaviorOfTheLaw()) provided
// for the OLD planner-side trim, carried over onto Drive's fastPid() (the
// mechanism that now lives here instead -- drive.h's own header). Drives
// tick() directly with nonzero ControlGains and AdaptationBounds.tauAdapt
// left at its all-zero default (Stage C stays off, isolating Stage B).
// ===========================================================================

void scenarioFastPidSteadyGateAndAntiWindup() {
  beginScenario("Stage B: the fast PID's integrator engages only when steady (frozen while ramping, "
                "not reset) and anti-winds-up against pidMax (130-005 coverage carried over from the "
                "deleted Motion::WheelTrim suite)");

  MockMotor left;
  MockMotor right;
  const float trackWidth = 200.0f;  // [mm]
  App::Drive drive(left, right, trackWidth);
  drive.setDutyPerSpeed(1.0f, 1.0f);  // identity calibration: target IS duty

  App::Drive::ControlGains gains;
  gains.kp = 0.05f;
  gains.ki = 0.5f;
  gains.iMax = 50.0f;
  gains.kaff = 0.05f;
  gains.pidMax = 20.0f;  // deliberately small -- reachable within this test's tick budget
  drive.setControlGains(gains);

  App::Drive::AdaptationBounds bounds;
  bounds.aSteady = 50.0f;  // [mm/s^2]
  // tauAdapt left at 0 (the all-zero default) -- Stage C stays off.
  drive.setAdaptationBounds(bounds);

  const float cmd = 200.0f;  // [mm/s] commanded speed
  uint32_t now = 1000;

  // Phase 1: RAMPING (|cmdAccel| >= aSteady) with a persistent, large
  // error (velocity held at exactly 0) -- the integrator must stay
  // EXACTLY zero throughout, mirroring Motion::WheelTrim's own accel-phase
  // gate. p+feed alone (0.05*200 + 0.05*100 = 15) stays under pidMax (20),
  // so if the integral is genuinely frozen, pidLeft() is IDENTICAL and
  // exact on every single tick -- no closed-form assumption needed beyond
  // "the inputs don't change and neither does the gate".
  const float kRampAccel = 100.0f;  // [mm/s^2] >= aSteady -- ramping
  const float kExpectedRampPid = gains.kp * cmd + gains.kaff * kRampAccel;  // 15.0
  for (int i = 0; i < 20; ++i) {
    Types::RobotState state;
    state.wheelLeft.cmdVelocity = cmd;
    state.wheelLeft.cmdAccel = kRampAccel;
    state.wheelLeft.velocity = 0.0f;  // stalled: a large, persistent error
    state.wheelLeft.connected = true;
    state.wheelLeft.sampleTime = now - 10;
    state.time.cycleStart = now;
    state.time.cyclePeriod = 50000;  // [us] -- dt = 0.05s
    drive.tick(state);
    checkFloatEq(drive.pidLeft(), kExpectedRampPid,
                "ramping: pid is p+feed only, no integral growth (the integrator stayed frozen)");
    now += 50;
  }

  // Phase 2: STEADY (|cmdAccel| < aSteady), same persistent error -- the
  // integrator now engages and grows, and the total pid output clamps at
  // pidMax without ever exceeding it (anti-windup): once the provisional
  // sum is pinned against the clamp, fastPid() stops integrating further
  // (the same pushingIntoClamp gate WheelTrim::compute() used to implement).
  bool sawGrowth = false;
  float previousPid = drive.pidLeft();
  for (int i = 0; i < 50; ++i) {
    Types::RobotState state;
    state.wheelLeft.cmdVelocity = cmd;
    state.wheelLeft.cmdAccel = 0.0f;  // steady
    state.wheelLeft.velocity = 0.0f;  // same large, persistent error
    state.wheelLeft.connected = true;
    state.wheelLeft.sampleTime = now - 10;
    state.time.cycleStart = now;
    state.time.cyclePeriod = 50000;
    drive.tick(state);
    checkTrue(std::fabs(drive.pidLeft()) <= gains.pidMax + 1e-3f,
             "pid never exceeds pidMax, even sustained (anti-windup)");
    if (drive.pidLeft() > previousPid + 1e-6f) sawGrowth = true;
    previousPid = drive.pidLeft();
    now += 50;
  }
  checkTrue(sawGrowth,
           "the integrator actually grew once steady (it was frozen during the ramp, not disabled "
           "outright)");
  checkFloatEq(drive.pidLeft(), gains.pidMax,
              "pid settles PINNED at the clamp under a persistent error large enough to saturate it",
              0.5f);
}

// ===========================================================================
// 8. 130-005 AC#2: a WHEELS teleop command and a planner Move both reach the
// SAME Drive controller path -- Drive's tick() reads only state.wheelLeft/
// Right.cmdVelocity/cmdAccel/velocity, with NO knowledge of which subsystem
// staged them (drive.h's own header: "no privileged or degraded path").
// Exercises the REAL production write path for the WHEELS side
// (Drive::command()+Drive::update(), exactly as RobotLoop calls them) against
// a hand-staged blackboard for the Move side (state.wheelLeft.cmdVelocity/
// cmdAccel set directly, exactly what Motion::Planner::update() stages for a
// Move cruising at the same speed with zero accel -- MovePhase::Hold) --
// then asserts tick()'s bias/duty evolution is IDENTICAL, cycle for cycle,
// under the SAME imperfect plant.
// ===========================================================================

void scenarioWheelsAndMoveReachIdenticalDriveBehavior() {
  beginScenario("130-005 AC#2: a WHEELS command and a planner Move reach the SAME Drive controller "
                "path -- identical tick-by-tick bias/duty for the same commanded velocity");

  const float cmd = 200.0f;          // [mm/s] commanded cruise speed, both paths
  const float plantGain = 0.85f;     // a known, imperfect plant -- gives Stage C something to close
  constexpr int kCycles = 60;

  auto runPath = [&](bool viaWheels) {
    MockMotor left;
    MockMotor right;
    const float trackWidth = 200.0f;  // [mm]
    App::Drive drive(left, right, trackWidth);
    drive.setDutyPerSpeed(1.0f, 1.0f);  // identity: duty == corrected command

    App::Drive::ControlGains gains;
    gains.kp = 0.1f;
    gains.ki = 0.3f;
    gains.iMax = 40.0f;
    gains.kaff = 0.05f;
    gains.pidMax = 30.0f;
    drive.setControlGains(gains);

    App::Drive::AdaptationBounds bounds;
    bounds.biasMax = 40.0f;   // [mm/s]
    bounds.tauAdapt = 2.0f;   // [s]
    bounds.aSteady = 50.0f;   // [mm/s^2]
    drive.setAdaptationBounds(bounds);

    uint32_t now = 1000;
    float measured = 0.0f;  // both wheels start at rest, identically
    std::vector<float> dutyTrace;
    std::vector<float> biasTrace;
    dutyTrace.reserve(kCycles);
    biasTrace.reserve(kCycles);

    for (int i = 0; i < kCycles; ++i) {
      Types::RobotState state{};
      if (viaWheels) {
        // WHEELS: arm once (a real command() call), then Drive's own
        // update() republishes its targets onto the blackboard every
        // cycle -- the exact call RobotLoop makes each cycle while Drive
        // owns motion.
        if (i == 0) drive.command(cmd, cmd, /*duration=*/60000.0f, /*moveId=*/1, now);
        drive.update(state, now);
      } else {
        // Move: hand-stage exactly what Motion::Planner::update() would
        // publish for a Move cruising at `cmd` with zero accel (Hold
        // phase) -- Motion::Planner sheds all wheel-actuation code
        // (130-005), so this is bit-for-bit what tick() actually sees
        // from a real Move at cruise.
        state.wheelLeft.cmdVelocity = cmd;
        state.wheelLeft.cmdAccel = 0.0f;
      }
      state.wheelLeft.velocity = measured;
      state.wheelLeft.connected = true;
      state.wheelLeft.sampleTime = now - 10;
      state.time.cycleStart = now;
      state.time.cyclePeriod = 50000;  // [us] -- dt = 0.05s

      drive.tick(state);
      dutyTrace.push_back(left.lastDutyCmd);
      biasTrace.push_back(drive.biasLeft());

      // The plant's own response to the duty this tick just computed,
      // consumed as next tick's measured velocity (same convention as the
      // Stage C scenarios above).
      measured = plantGain * (cmd + drive.biasLeft());
      now += 50;
    }
    return std::make_pair(dutyTrace, biasTrace);
  };

  const auto wheels = runPath(/*viaWheels=*/true);
  const auto move = runPath(/*viaWheels=*/false);

  checkTrue(wheels.first.size() == move.first.size(), "both paths ran the same number of cycles");
  bool allMatched = true;
  for (size_t i = 0; i < wheels.first.size() && i < move.first.size(); ++i) {
    if (std::fabs(wheels.first[i] - move.first[i]) > 1e-4f ||
        std::fabs(wheels.second[i] - move.second[i]) > 1e-4f) {
      allMatched = false;
      char buf[256];
      std::snprintf(buf, sizeof(buf),
                    "tick %zu: WHEELS (duty=%g, bias=%g) != Move (duty=%g, bias=%g)", i,
                    static_cast<double>(wheels.first[i]), static_cast<double>(wheels.second[i]),
                    static_cast<double>(move.first[i]), static_cast<double>(move.second[i]));
      fail(buf);
    }
  }
  checkTrue(allMatched, "every cycle's duty/bias matched exactly between the WHEELS path and the Move "
                       "path -- one controller, no privileged or degraded writer");
  // Not a trivially-passing no-op: the plant gain error actually moved bias
  // over the run, on both paths.
  checkTrue(std::fabs(wheels.second.back()) > 1.0f,
           "bias actually adapted over the run on both paths (not a no-op check)");
}

}  // namespace

int main() {
  scenarioSetDutyStagesRawValues();
  scenarioStopZeroesBothTargetsWithinOneCycle();
  scenarioEstopReassertsStopWhileWheelsStillMoving();
  scenarioBiasConvergesUnderKnownPlantGainError();
  scenarioBiasClampHoldsAgainstDivergentError();
  scenarioBumplessTransferAcrossBiasUpdate();
  scenarioFastPidSteadyGateAndAntiWindup();
  scenarioWheelsAndMoveReachIdenticalDriveBehavior();

  if (g_failureCount == 0) {
    std::printf("OK: all App::Drive scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the App::Drive scenarios\n", g_failureCount);
  return 1;
}
