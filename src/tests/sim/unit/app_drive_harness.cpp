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
// 131-002 (issue A-commanded-zero-leaks-through-stage-b.md): commanded-zero
// through Stage B, and Stage B's own freshness gate. correctedCommand()'s
// `desired == 0.0f` guard (Stage A) has NO effect on Stage B -- fastPid() is
// a fully separate computation on MEASURED velocity, so a wheel that has not
// yet coasted to rest keeps a nonzero err even after its commanded speed
// reaches exactly 0. The two scenarios below prove: (1) a commanded-zero
// wheel writes exactly 0 duty every tick regardless of a pre-wound
// integrator, and that integrator is FROZEN (not reset) across the whole
// zero period; (2) a stale/disconnected/frozen wheel's integrator likewise
// stays frozen rather than winding against a manufactured zero-velocity
// reading (the same freshness conjunct Stage C's adaptBias() already uses).
// ===========================================================================

// 9. Commanded speed exactly 0.0f -> Stage B's contribution and the written
//    duty are exactly 0.0 every tick, no matter how long, regardless of the
//    integrator's retained value -- and the integrator is FROZEN (not
//    reset): re-commanding the ORIGINAL nonzero speed/error afterward
//    resumes from at least the pre-zero pid value, never from scratch.
void scenarioCommandedZeroForcesStageBToZeroAndFreezesIntegrator() {
  beginScenario("Stage B: commanded-zero forces the contribution and written duty to exactly 0.0, "
                "and FREEZES (does not reset) the integrator across the zero period (131-002)");

  MockMotor left;
  MockMotor right;
  const float trackWidth = 200.0f;  // [mm]
  App::Drive drive(left, right, trackWidth);
  drive.setDutyPerSpeed(1.0f, 1.0f);  // identity calibration: target IS duty

  App::Drive::ControlGains gains;
  gains.kp = 0.05f;
  gains.ki = 0.5f;
  gains.iMax = 50.0f;
  gains.kaff = 0.0f;
  gains.pidMax = 40.0f;  // reachable within this test's winding budget (see Phase 1 below)
  drive.setControlGains(gains);

  App::Drive::AdaptationBounds bounds;
  bounds.aSteady = 50.0f;  // [mm/s^2] -- cmdAccel always 0 below, so always "steady"
  // tauAdapt left at 0 (all-zero default) -- Stage C stays off, isolating Stage B.
  drive.setAdaptationBounds(bounds);

  const float cmd = 200.0f;  // [mm/s] commanded cruise speed
  uint32_t now = 1000;

  // Phase 1: wind the integrator to a known, comfortably-nonzero value
  // (pins at pidMax=40 well within 10 cycles -- see
  // scenarioFastPidSteadyGateAndAntiWindup()'s own arithmetic, identical
  // gains/error) under a persistent, steady error.
  for (int i = 0; i < 10; ++i) {
    Types::RobotState state;
    state.wheelLeft.cmdVelocity = cmd;
    state.wheelLeft.velocity = 0.0f;  // stalled: a large, persistent error
    state.wheelLeft.connected = true;
    state.wheelLeft.sampleTime = now - 10;
    state.time.cycleStart = now;
    state.time.cyclePeriod = 50000;  // [us] -- dt = 0.05s
    drive.tick(state);
    now += 50;
  }
  const float pidBeforeZero = drive.pidLeft();
  checkTrue(pidBeforeZero > 1.0f,
           "setup: the integrator actually wound up to a nonzero value before the zero period");

  // Phase 2: command exactly zero for many ticks -- Stage B's contribution
  // and the written duty must be EXACTLY zero every single tick, regardless
  // of the integrator's retained (nonzero) value.
  for (int i = 0; i < 20; ++i) {
    Types::RobotState state;
    state.wheelLeft.cmdVelocity = 0.0f;
    state.wheelLeft.velocity = 0.0f;
    state.wheelLeft.connected = true;
    state.wheelLeft.sampleTime = now - 10;
    state.time.cycleStart = now;
    state.time.cyclePeriod = 50000;
    drive.tick(state);
    checkFloatEq(drive.pidLeft(), 0.0f,
                "commanded-zero: Stage B's contribution is exactly 0.0, every tick");
    checkFloatEq(left.lastDutyCmd, 0.0f,
                "commanded-zero: written duty is exactly 0.0, every tick");
    now += 50;
  }

  // Phase 3: re-command the ORIGINAL nonzero speed/error for exactly one
  // tick. If the integrator had been RESET to 0 during the zero period, this
  // tick's pid would be far below pidBeforeZero (starting from scratch). It
  // is instead FROZEN at its pre-zero value throughout Phase 2, so this
  // tick's pid must be at least pidBeforeZero.
  Types::RobotState resumeState;
  resumeState.wheelLeft.cmdVelocity = cmd;
  resumeState.wheelLeft.velocity = 0.0f;
  resumeState.wheelLeft.connected = true;
  resumeState.wheelLeft.sampleTime = now - 10;
  resumeState.time.cycleStart = now;
  resumeState.time.cyclePeriod = 50000;
  drive.tick(resumeState);
  checkTrue(drive.pidLeft() >= pidBeforeZero - 1e-3f,
           "resumed motion: pid picks up from at least the pre-zero value -- the integrator was "
           "FROZEN across the commanded-zero period, not reset to 0");
}

// 10. A stale/disconnected/frozen wheel's Stage B integrator stays FROZEN
//     (never winds), sharing the SAME freshness conjunct Stage C's
//     adaptBias() already required (connected/wheelFrozenLeft/sampleTime
//     age) -- exercised as three independent failure modes, each proven by
//     the p-term-only value staying IDENTICAL across many ticks under a
//     persistent error (mirrors scenarioFastPidSteadyGateAndAntiWindup()'s
//     own ramping-phase proof technique).
void scenarioStaleDisconnectedOrFrozenWheelFreezesStageBIntegrator() {
  beginScenario("Stage B: a stale/disconnected/frozen wheel's integrator stays FROZEN, never "
                "winding against a manufactured reading (131-002)");

  App::Drive::ControlGains gains;
  gains.kp = 0.05f;
  gains.ki = 0.5f;
  gains.iMax = 50.0f;
  gains.kaff = 0.0f;
  gains.pidMax = 40.0f;

  App::Drive::AdaptationBounds bounds;
  bounds.aSteady = 50.0f;  // [mm/s^2]

  const float cmd = 200.0f;  // [mm/s]
  const float kExpectedPOnly = gains.kp * cmd;  // 10.0 -- integrator frozen at 0, no feed term

  // (a) disconnected -- state.wheelLeft.connected == false throughout.
  {
    MockMotor left;
    MockMotor right;
    App::Drive drive(left, right, 200.0f);
    drive.setDutyPerSpeed(1.0f, 1.0f);
    drive.setControlGains(gains);
    drive.setAdaptationBounds(bounds);

    uint32_t now = 1000;
    for (int i = 0; i < 20; ++i) {
      Types::RobotState state;
      state.wheelLeft.cmdVelocity = cmd;
      state.wheelLeft.velocity = 0.0f;  // manufactured zero on a failed collect -- large "error"
      state.wheelLeft.connected = false;  // disconnected -- NOT fresh
      state.wheelLeft.sampleTime = now - 10;
      state.time.cycleStart = now;
      state.time.cyclePeriod = 50000;
      drive.tick(state);
      checkFloatEq(drive.pidLeft(), kExpectedPOnly,
                  "disconnected: pid is p-term only, no integral growth -- Stage B never winds "
                  "against a disconnected wheel");
      now += 50;
    }
  }

  // (b) stale sample age -- connected, but sampleTime()'s implied age
  //     exceeds kMaxSampleAge (200ms), the same "not fresh" conjunct via a
  //     different failure mode.
  {
    MockMotor left;
    MockMotor right;
    App::Drive drive(left, right, 200.0f);
    drive.setDutyPerSpeed(1.0f, 1.0f);
    drive.setControlGains(gains);
    drive.setAdaptationBounds(bounds);

    uint32_t now = 1000;
    for (int i = 0; i < 20; ++i) {
      Types::RobotState state;
      state.wheelLeft.cmdVelocity = cmd;
      state.wheelLeft.velocity = 0.0f;
      state.wheelLeft.connected = true;
      state.wheelLeft.sampleTime = 0;  // ancient -- age (cycleStart - 0) always >> 200ms here
      state.time.cycleStart = now;
      state.time.cyclePeriod = 50000;
      drive.tick(state);
      checkFloatEq(drive.pidLeft(), kExpectedPOnly,
                  "stale sample age: pid is p-term only, no integral growth");
      now += 50;
    }
  }

  // (c) frozen (Health::wheelFrozenLeft) -- the third fresh conjunct.
  {
    MockMotor left;
    MockMotor right;
    App::Drive drive(left, right, 200.0f);
    drive.setDutyPerSpeed(1.0f, 1.0f);
    drive.setControlGains(gains);
    drive.setAdaptationBounds(bounds);

    uint32_t now = 1000;
    for (int i = 0; i < 20; ++i) {
      Types::RobotState state;
      state.wheelLeft.cmdVelocity = cmd;
      state.wheelLeft.velocity = 0.0f;
      state.wheelLeft.connected = true;
      state.wheelLeft.sampleTime = now - 10;  // fresh BY AGE ...
      state.health.wheelFrozenLeft = true;    // ... but frozen -- still NOT fresh
      state.time.cycleStart = now;
      state.time.cyclePeriod = 50000;
      drive.tick(state);
      checkFloatEq(drive.pidLeft(), kExpectedPOnly,
                  "frozen wheel: pid is p-term only, no integral growth");
      now += 50;
    }
  }
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

// ===========================================================================
// 9. 131-001 (SUC-131-001, sprint 131 Design Rationale Decisions 1/2):
// Drive::takeover() vs Drive::estop() -- takeover() preserves Stage B's
// integrator, Stage C's bias, and the deficit latch (an ownership handover,
// not a safety event); estop() still fully resets all three (a genuine
// panic path). Both verbs' distinct post-conditions are asserted side by
// side in this ONE scenario, per this ticket's own testing plan -- a
// regression guard against a future edit silently re-merging them.
// ===========================================================================

void scenarioTakeoverPreservesLearnedStateEstopFullyResets() {
  beginScenario("131-001: takeover() preserves Stage B's integrator, Stage C's bias, and the "
                "deficit latch; estop() still fully resets all three -- both verbs' distinct "
                "post-conditions side by side");

  MockMotor left;
  MockMotor right;
  const float trackWidth = 200.0f;  // [mm]
  App::Drive drive(left, right, trackWidth);
  drive.setDutyPerSpeed(1.0f, 1.0f);  // identity calibration: target IS duty

  App::Drive::ControlGains gains;
  gains.kp = 0.5f;
  gains.ki = 2.0f;
  gains.iMax = 40.0f;
  gains.pidMax = 40.0f;
  drive.setControlGains(gains);

  App::Drive::AdaptationBounds bounds;
  bounds.biasMax = 60.0f;
  bounds.tauAdapt = 0.3f;
  bounds.aSteady = 100.0f;
  bounds.deficitThreshold = 1.0f;  // deliberately tiny -- trips easily under a persistent error
  bounds.deficitWindow = 10.0f;    // [ms] -- a couple of 50ms ticks is already past this
  drive.setAdaptationBounds(bounds);

  // A real WHEELS-style armed command, so takeover()/estop()'s "disarm" half
  // has something genuine to disarm (owns() true beforehand).
  drive.command(200.0f, 200.0f, /*duration=*/100000.0f, /*moveId=*/1, /*now=*/500);
  checkTrue(drive.owns(), "setup: a WHEELS command is armed (owns() true) before either verb");

  // Drive a sustained, deliberately unclosable error (plant held at 0
  // velocity throughout, cmd=200) so Stage B's integrator winds up, Stage
  // C's bias saturates, and the deficit latch trips -- all THREE pieces of
  // "learned state" land at a known, nonzero/true value before either verb
  // is exercised.
  uint32_t now = 1000;
  for (int i = 0; i < 200; ++i) {
    Types::RobotState state;
    state.wheelLeft.cmdVelocity = 200.0f;
    state.wheelLeft.velocity = 0.0f;  // stalled -- persistent, unclosable error
    state.wheelLeft.connected = true;
    state.wheelLeft.sampleTime = now - 10;
    state.time.cycleStart = now;
    state.time.cyclePeriod = 50000;
    drive.tick(state);
    now += 50;
  }

  const float biasBefore = drive.biasLeft();
  checkTrue(std::fabs(biasBefore) > 1.0f, "setup: bias genuinely adapted away from 0");
  checkTrue(drive.deficitLeft(), "setup: the deficit flag latched under the sustained, unclosable error");
  checkTrue(std::fabs(drive.pidLeft()) > 1.0f, "setup: Stage B's fast PID output is genuinely nonzero");

  // --- Phase 1: takeover() -- learned state must be UNCHANGED. ---
  drive.takeover();
  checkFloatEq(drive.targetLeft(), 0.0f, "takeover() zeroes targetLeft_");
  checkFloatEq(drive.targetRight(), 0.0f, "takeover() zeroes targetRight_");
  checkTrue(!drive.owns(), "takeover() disarms the WHEELS command (owns() false)");
  checkFloatEq(drive.biasLeft(), biasBefore, "takeover() leaves biasLeft_ UNCHANGED");
  checkTrue(drive.deficitLeft(), "takeover() leaves the deficit latch UNCHANGED (still true)");

  // --- Phase 2: estop() -- the SAME learned state must now be fully reset,
  // in contrast to Phase 1 above. ---
  drive.estop();
  checkFloatEq(drive.targetLeft(), 0.0f, "estop() also zeroes targetLeft_");
  checkFloatEq(drive.targetRight(), 0.0f, "estop() also zeroes targetRight_");
  checkTrue(!drive.owns(), "estop() also disarms the WHEELS command");
  checkFloatEq(drive.biasLeft(), 0.0f, "estop() resets biasLeft_ to 0 -- distinct from takeover()");
  checkTrue(!drive.deficitLeft(), "estop() clears the deficit latch -- distinct from takeover()");

  // A subsequent tick() under the SAME stalled/unreachable conditions
  // confirms Stage B's integrator was genuinely zeroed too: the very next
  // tick's pid is p-term only (integral fresh at 0, feed/kaff==0 here),
  // not still carrying whatever the pre-estop() integrator had wound up to.
  Types::RobotState state;
  state.wheelLeft.cmdVelocity = 200.0f;
  state.wheelLeft.velocity = 0.0f;
  state.wheelLeft.connected = true;
  state.wheelLeft.sampleTime = now - 10;
  state.time.cycleStart = now;
  state.time.cyclePeriod = 50000;
  drive.tick(state);
  // p-only (integral freshly 0) is kp*200 == 100, which exceeds pidMax (40)
  // and is clamped there -- the SAME clamp the pre-estop() saturated
  // integrator was already pinned against, so this single-tick check can't
  // distinguish "integral still 40-ish" from "integral genuinely 0" by
  // magnitude alone. checkIntegralGenuinelyZeroed() below does that
  // properly, by comparing against a FRESH Drive that never wound up at
  // all.
  checkFloatEq(drive.pidLeft(), gains.pidMax,
              "estop() leaves the fast PID pinned at +pidMax on the very next tick (p-term alone "
              "already exceeds pidMax under this persistent error)");

  // The real proof that pidIntegralLeft_ was genuinely zeroed (not just
  // "still clamped, coincidentally at the same value"): a FRESH Drive,
  // whose integrator was NEVER wound up, produces the IDENTICAL pid output
  // under the identical one-tick input -- if estop() had left the old
  // integral in place, the post-estop() drive would clamp at the same
  // +pidMax too (since pidMax dominates either way at this error size), so
  // this comparison instead uses a smaller error where the unclamped p+i
  // sum is observable.
  MockMotor freshLeft;
  MockMotor freshRight;
  App::Drive freshDrive(freshLeft, freshRight, trackWidth);
  freshDrive.setDutyPerSpeed(1.0f, 1.0f);
  freshDrive.setControlGains(gains);
  freshDrive.setAdaptationBounds(bounds);
  Types::RobotState smallErrState;
  smallErrState.wheelLeft.cmdVelocity = 10.0f;  // small enough that p+i stays well under pidMax
  smallErrState.wheelLeft.velocity = 0.0f;
  smallErrState.wheelLeft.connected = true;
  smallErrState.wheelLeft.sampleTime = now - 10;
  smallErrState.time.cycleStart = now;
  smallErrState.time.cyclePeriod = 50000;
  freshDrive.tick(smallErrState);

  drive.estop();  // re-zero drive's integrator again (Phase 2 already did this once above)
  drive.tick(smallErrState);
  checkFloatEq(drive.pidLeft(), freshDrive.pidLeft(),
              "post-estop() pid under a small error matches a NEVER-wound-up fresh Drive exactly -- "
              "the integrator was genuinely zeroed, not merely re-clamped to the same ceiling");
}

// ===========================================================================
// 10. 131-001: takeover() must not touch the estop()-armed stop-reassertion
// countdown (stopEnforceCountdown_, 129-001) either -- it has no public
// getter, so this scenario observes it INDIRECTLY through tick()'s own
// re-assertion behavior (scenario 3 above establishes that mechanism):
// consume part of an estop()-armed countdown, call takeover(), then confirm
// EXACTLY the remaining ticks still force a write (neither reset to 0 --
// which would go quiet immediately -- nor re-armed to the full
// kStopEnforceTicks -- which would force MORE writes than remain).
// ===========================================================================

void scenarioTakeoverDoesNotTouchStopReassertionCountdown() {
  beginScenario("131-001: takeover() leaves the estop()-armed stop-reassertion countdown "
                "untouched -- neither reset to 0 nor re-armed");

  MockMotor left;
  MockMotor right;
  const float trackWidth = 200.0f;  // [mm]
  App::Drive drive(left, right, trackWidth);
  drive.setDutyPerSpeed(1.0f, 1.0f);

  drive.tick(wheelCmd(0.5f, 0.5f));  // a real nonzero write, so estop() below is a genuine transition
  drive.estop();                    // arms stopEnforceCountdown_ = kStopEnforceTicks (30, drive.h)

  left.setMockVelocity(0.0f);   // at rest throughout -- isolates the COUNTDOWN half of enforceStop
  right.setMockVelocity(0.0f);  // from the wheelsMoving half (scenario 3 above covers that half)

  constexpr int kStopEnforceTicks = 30;  // drive.h's own kStopEnforceTicks -- duplicated per this
                                          // codebase's established fixture convention (private, no
                                          // accessor) -- see e.g. sim_api_harness.cpp's kSettle/kClear.
  const int kConsumed = 5;
  for (int i = 0; i < kConsumed; ++i) drive.tick(wheelCmd(0.0f, 0.0f));

  drive.takeover();  // must NOT touch stopEnforceCountdown_

  // If takeover() left the countdown alone, exactly (kStopEnforceTicks -
  // kConsumed) more tick() calls re-issue setDuty(0) before the
  // quiet-at-zero shortcut resumes; landing anywhere else (0 more -- reset
  // to 0; kStopEnforceTicks more -- re-armed) falsifies the AC.
  const int kRemaining = kStopEnforceTicks - kConsumed;
  int callsBefore = left.setDutyCalls;
  for (int i = 0; i < kRemaining; ++i) drive.tick(wheelCmd(0.0f, 0.0f));
  checkTrue(left.setDutyCalls == callsBefore + kRemaining,
           "the remaining (kStopEnforceTicks - already-consumed) re-assertion ticks still fire "
           "after takeover() -- the countdown was neither reset nor re-armed");

  callsBefore = left.setDutyCalls;
  drive.tick(wheelCmd(0.0f, 0.0f));  // one more, past the countdown -- quiet shortcut should resume
  checkTrue(left.setDutyCalls == callsBefore,
           "the quiet-at-zero shortcut resumes exactly when the ORIGINAL estop()-armed countdown "
           "would have elapsed -- confirms takeover() didn't re-arm it");
}

// ===========================================================================
// 11. 131-001 (review C3, Design Rationale Decision 2): a bias converged
// under sustained FORWARD motion must not REDUCE a subsequent REVERSE
// command's magnitude relative to bias==0 -- the sign-aware, magnitude-
// domain bias fix in correctedCommand().
// ===========================================================================

void scenarioReversalAfterConvergedForwardBiasDoesNotReduceMagnitude() {
  beginScenario("131-001 (review C3, Decision 2): a bias converged under sustained FORWARD motion "
                "does not reduce a subsequent REVERSE command's magnitude relative to bias==0");

  MockMotor left;
  MockMotor right;
  const float trackWidth = 200.0f;  // [mm]
  App::Drive drive(left, right, trackWidth);
  drive.setDutyPerSpeed(1.0f, 1.0f);  // identity: duty == corrected command

  App::Drive::AdaptationBounds bounds;
  bounds.biasMax = 60.0f;
  bounds.tauAdapt = 0.3f;
  bounds.aSteady = 100.0f;
  drive.setAdaptationBounds(bounds);

  // Converge bias forward under a known under-delivering plant (mirrors
  // scenarioBiasConvergesUnderKnownPlantGainError above).
  const float cmd = 200.0f;
  const float plantGain = 0.8f;
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
    measured = plantGain * (cmd + drive.biasLeft());
    now += 50;
  }
  const float convergedBias = drive.biasLeft();
  checkTrue(convergedBias > 5.0f, "setup: a genuinely positive bias converged under forward motion");

  // Freeze Stage C (tauAdapt=0, the "0 = off" convention) so the reverse
  // command below exercises ONLY correctedCommand()'s bias application,
  // not further adaptation.
  App::Drive::AdaptationBounds frozen;  // tauAdapt=0 default -- adaptation off, bias held
  drive.setAdaptationBounds(frozen);
  checkFloatEq(drive.biasLeft(), convergedBias,
              "setup: freezing adaptationBounds does not itself reset the already-converged bias");

  // A second, otherwise-identical Drive with bias == 0 (adaptation was
  // never on) -- the uncorrected baseline this scenario compares against.
  MockMotor left0;
  MockMotor right0;
  App::Drive driveZeroBias(left0, right0, trackWidth);
  driveZeroBias.setDutyPerSpeed(1.0f, 1.0f);
  driveZeroBias.setAdaptationBounds(frozen);

  const float reverseCmd = -150.0f;
  Types::RobotState reverseState;
  reverseState.wheelLeft.cmdVelocity = reverseCmd;
  reverseState.wheelLeft.velocity = 0.0f;
  reverseState.wheelLeft.connected = true;
  reverseState.wheelLeft.sampleTime = now - 10;
  reverseState.time.cycleStart = now;
  reverseState.time.cyclePeriod = 50000;

  drive.tick(reverseState);
  driveZeroBias.tick(reverseState);

  const float reverseMagnitudeWithBias = std::fabs(left.lastDutyCmd);
  const float reverseMagnitudeUncorrected = std::fabs(left0.lastDutyCmd);
  checkTrue(reverseMagnitudeWithBias >= reverseMagnitudeUncorrected - 1e-3f,
           "the reverse command's magnitude with the forward-converged bias is NOT reduced "
           "relative to bias==0 -- the fix must not make reverse magnitude worse than uncorrected");
  // Not a trivially-passing check: the corrected magnitude is measurably
  // BOOSTED (bias > 0 helps, matching Decision 2's own physical claim), not
  // merely "equal by coincidence."
  checkTrue(reverseMagnitudeWithBias > reverseMagnitudeUncorrected + 1.0f,
           "the bias genuinely BOOSTS the reverse command's magnitude (not just a no-op pass)");
}

// ===========================================================================
// 12. 131-001 (sprint success criteria): a converged Stage C bias survives
// MULTIPLE chained takeover() calls (2+ leg boundaries, equivalent to a
// tour) -- never resets to 0 at any boundary. See also
// src/tests/sim/system/sim_api_harness.cpp's own chained-MOVE scenario for
// the same property proven through the REAL RobotLoop::handleMove() call
// site over the full composition root.
// ===========================================================================

void scenarioBiasPersistsAcrossChainedTakeoverBoundaries() {
  beginScenario("131-001: a converged Stage C bias survives THREE chained takeover() calls (2+ "
                "leg boundaries, equivalent to a tour) -- never resets to 0 at any boundary");

  MockMotor left;
  MockMotor right;
  const float trackWidth = 200.0f;  // [mm]
  App::Drive drive(left, right, trackWidth);
  drive.setDutyPerSpeed(1.0f, 1.0f);

  App::Drive::AdaptationBounds bounds;
  bounds.biasMax = 60.0f;
  bounds.tauAdapt = 0.4f;
  bounds.aSteady = 100.0f;
  drive.setAdaptationBounds(bounds);

  const float cmd = 200.0f;
  const float plantGain = 0.8f;
  uint32_t now = 1000;
  float measured = plantGain * cmd;

  auto runLeg = [&](int cycles) {
    for (int i = 0; i < cycles; ++i) {
      Types::RobotState state;
      state.wheelLeft.cmdVelocity = cmd;
      state.wheelLeft.velocity = measured;
      state.wheelLeft.connected = true;
      state.wheelLeft.sampleTime = now - 10;
      state.time.cycleStart = now;
      state.time.cyclePeriod = 50000;
      drive.tick(state);
      measured = plantGain * (cmd + drive.biasLeft());
      now += 50;
    }
  };

  // Leg 1: converge bias away from 0.
  runLeg(300);
  const float biasAfterLeg1 = drive.biasLeft();
  checkTrue(std::fabs(biasAfterLeg1) > 5.0f, "leg 1: bias converged away from 0");

  // Leg boundary 1 -- a MOVE arrives; RobotLoop::handleMove() calls
  // drive_.takeover() (this IS that call, hand-invoked here).
  drive.takeover();
  checkFloatEq(drive.biasLeft(), biasAfterLeg1, "leg boundary 1: bias UNCHANGED across takeover()");

  // Leg 2: bias holds (it does not re-converge from 0 -- it was already there).
  runLeg(20);
  checkTrue(std::fabs(drive.biasLeft()) > 5.0f,
           "leg 2: bias still away from 0 (never dropped to 0 at the boundary)");
  const float biasAfterLeg2 = drive.biasLeft();

  // Leg boundary 2 -- a second chained MOVE.
  drive.takeover();
  checkFloatEq(drive.biasLeft(), biasAfterLeg2,
              "leg boundary 2: bias UNCHANGED across the SECOND takeover()");

  // Leg 3.
  runLeg(20);
  checkTrue(std::fabs(drive.biasLeft()) > 5.0f,
           "leg 3: bias still away from 0 across BOTH leg boundaries -- 2+ legs, equivalent to a "
           "tour, no reset anywhere");
}

// ===========================================================================
// 13-17. 131-003 REVISED post-shipment (SUC-131-003, issue
// A-speed-floor-snaps-the-planner-differential.md): applySpeedFloor() is a
// RATIO-PRESERVING SCALE on the raw wheel pair directly -- no common-mode/
// differential decomposition (the originally-shipped design, commit
// `29578345`, superseded because it measurably regressed turn accuracy; see
// sprint 131 sprint.md's "Revision -- Ticket 003's speed-floor semantics").
// `dominantMag = max(|rawLeft|, |rawRight|)`; below vMin and nonzero, BOTH
// wheels scale by `vMin / dominantMag`. All scenarios below drive tick()
// directly with dutyPerSpeed=(1,1) (identity: duty == corrected command,
// this file's own established convention) and all-zero ControlGains/
// AdaptationBounds.tauAdapt (Stage B/C both inert -- see
// scenarioSetDutyStagesRawValues()'s own precedent), so MockMotor::
// lastDutyCmd reflects the post-floor speedLeft/speedRight EXACTLY,
// isolating the floor's own arithmetic from every other stage. Only
// AdaptationBounds.vMin is set (a test-local 100.0f mm/s -- NOT this
// robot's real, LOW-CONFIDENCE 99.7 figure; ticket 131-003 is
// semantics-only and must not entangle a test constant with the
// production one).
// ===========================================================================

// 13. AC#1 (unchanged numbers from the superseded implementation -- see
// sprint.md Revision point 2): a small differential trim riding on a
// common-mode speed AT the vMin boundary produces a PROPORTIONAL per-wheel
// split (the differential passes through whole). The dominant wheel (103)
// already clears vMin (100), so NO scaling applies at all -- the raw pair
// passes through completely unchanged, unlike the OLD (pre-131-003)
// per-wheel-independent floor, which would have independently boosted the
// sub-floor left wheel (97) and eaten half the trim.
void scenarioDifferentialTrimAtFloorBoundaryStaysProportional() {
  beginScenario("131-003 AC#1: a differential trim on a common-mode speed AT the vMin boundary "
                "produces a proportional per-wheel split (the OLD per-wheel floor ate half the "
                "trim here; the ratio-preserving scale doesn't engage at all since the dominant "
                "wheel already clears vMin)");

  MockMotor left;
  MockMotor right;
  const float trackWidth = 200.0f;  // [mm]
  App::Drive drive(left, right, trackWidth);
  drive.setDutyPerSpeed(1.0f, 1.0f);  // identity calibration: target IS duty

  App::Drive::AdaptationBounds bounds;
  bounds.vMin = 100.0f;  // [mm/s] test-local floor -- NOT tovez.json's real 99.7
  drive.setAdaptationBounds(bounds);

  // Deliberately chosen so the OLD per-wheel floor bites: raw left (97) is
  // BELOW vMin (100) and would have been independently boosted to exactly
  // 100 -- eating the correction down to a 3 mm/s split (100 vs 103) instead
  // of the commanded 6 mm/s split (97 vs 103). The dominant wheel (103, the
  // larger magnitude of the two raw commands) is already AT/above vMin, so
  // the ratio-preserving scale is a no-op here -- the fix is entirely in
  // WHAT the floor looks at (the raw pair's dominant wheel, not each wheel
  // independently), not in re-tuning vMin itself.
  const float commonMode = 100.0f;    // [mm/s] -- AT the floor
  const float differential = 3.0f;    // [mm/s] -- "a few mm/s" heading-hold trim
  const float cmdLeft = commonMode - differential;   // 97
  const float cmdRight = commonMode + differential;  // 103 -- the dominant wheel

  drive.tick(wheelCmd(cmdLeft, cmdRight));

  checkFloatEq(left.lastDutyCmd, cmdLeft,
              "left wheel: the differential passes through proportionally -- duty matches the "
              "RAW (unscaled) commanded value, the dominant (right) wheel was already at the "
              "floor so no scaling applied");
  checkFloatEq(right.lastDutyCmd, cmdRight,
              "right wheel: same -- proportional, not floor-snapped");
  checkFloatEq(right.lastDutyCmd - left.lastDutyCmd, 2.0f * differential,
              "the per-wheel split equals the full commanded differential (6 mm/s) -- NOT the "
              "3 mm/s split the OLD per-wheel-independent floor would have left after boosting "
              "only the sub-floor left wheel");

  // A second, healthier-margin case (dominant wheel comfortably above the
  // floor) as a plain sanity check that pass-through holds in the ordinary
  // case too, not just at the adversarial boundary above.
  MockMotor left2;
  MockMotor right2;
  App::Drive drive2(left2, right2, trackWidth);
  drive2.setDutyPerSpeed(1.0f, 1.0f);
  drive2.setAdaptationBounds(bounds);
  const float cruiseCommon = 150.0f;  // [mm/s] comfortably above vMin
  drive2.tick(wheelCmd(cruiseCommon - differential, cruiseCommon + differential));
  checkFloatEq(left2.lastDutyCmd, cruiseCommon - differential,
              "healthy-margin case: left wheel duty matches the raw commanded value exactly");
  checkFloatEq(right2.lastDutyCmd, cruiseCommon + differential,
              "healthy-margin case: right wheel duty matches the raw commanded value exactly");
}

// 14. NEW, the direct regression-fix proof at the unit level (sprint.md
// Revision point 1): a symmetric pivot -- equal-and-opposite wheel commands,
// the EXACT shape Planner::planWheels() emits for every Angle Move -- with
// both wheels below vMin is boosted to exactly (-vMin, +vMin). This is
// bit-identical to what the pre-131-003 per-wheel-independent floor produced
// for this case (both wheels have equal magnitude, so flooring the pair as a
// whole and flooring each wheel independently coincide), and is exactly the
// case the superseded common-mode-only floor got wrong: a pure pivot's
// common mode is EXACTLY zero, so that design's floor never engaged here at
// all.
void scenarioSymmetricPivotBelowFloorBoostsToExactVMin() {
  beginScenario("131-003 (revised): a symmetric pivot (equal-and-opposite wheel commands, the "
                "shape planWheels() emits for an Angle Move) with both wheels below vMin is "
                "boosted to exactly (-vMin, +vMin) -- the direct regression-fix proof; the "
                "superseded common-mode-only floor never engaged here since a pivot's common "
                "mode is exactly zero");

  MockMotor left;
  MockMotor right;
  App::Drive drive(left, right, 200.0f);
  drive.setDutyPerSpeed(1.0f, 1.0f);

  App::Drive::AdaptationBounds bounds;
  bounds.vMin = 100.0f;  // [mm/s] test-local floor
  drive.setAdaptationBounds(bounds);

  const float lambda = 40.0f;  // [mm/s] -- a pivot's own sub-floor ramp magnitude
  drive.tick(wheelCmd(-lambda, lambda));  // unit = (-1, +1): a pure in-place rotation

  checkFloatEq(left.lastDutyCmd, -bounds.vMin,
              "left wheel boosted to exactly -vMin -- bit-identical to the pre-131-003 "
              "per-wheel-independent floor for a symmetric pivot");
  checkFloatEq(right.lastDutyCmd, bounds.vMin,
              "right wheel boosted to exactly +vMin, same");
  checkFloatEq(right.lastDutyCmd, -left.lastDutyCmd,
              "the ratio is preserved exactly (equal and opposite) -- not flattened, not "
              "distorted, just scaled up to the floor");
}

// 15. NEW (sprint.md Revision point 3): an asymmetric pair below vMin is
// scaled to preserve its ratio exactly, not flattened toward 1:1 (what the
// OLD pre-131-003 per-wheel-independent floor would have produced: both
// wheels independently boosted to vMin, i.e. 100/100) and not left
// arbitrarily distorted (a failure mode the superseded common-mode-only
// floor could produce depending on sign, since it never bounded how much it
// distorted an asymmetric differential riding on a sub-floor common mode).
void scenarioAsymmetricPairBelowFloorPreservesRatio() {
  beginScenario("131-003 (revised): an asymmetric pair below vMin (20/80 at a test-local "
                "vMin=100) is scaled to preserve its ratio exactly (25/100), not flattened "
                "toward 1:1 like the OLD per-wheel floor, and not left distorted like the "
                "superseded common-mode-only floor");

  MockMotor left;
  MockMotor right;
  App::Drive drive(left, right, 200.0f);
  drive.setDutyPerSpeed(1.0f, 1.0f);

  App::Drive::AdaptationBounds bounds;
  bounds.vMin = 100.0f;  // [mm/s] test-local floor
  drive.setAdaptationBounds(bounds);

  const float rawLeft = 20.0f;   // [mm/s] -- the sub-dominant wheel
  const float rawRight = 80.0f;  // [mm/s] -- the dominant wheel, still below vMin
  drive.tick(wheelCmd(rawLeft, rawRight));

  // scale = vMin / dominantMag = 100 / 80 = 1.25
  checkFloatEq(left.lastDutyCmd, 25.0f,
              "left wheel scaled from 20 to 25 (x1.25) -- NOT flattened to 100 (the old "
              "per-wheel floor) and not arbitrarily distorted (the superseded floor)");
  checkFloatEq(right.lastDutyCmd, 100.0f,
              "right (dominant) wheel lands at exactly vMin");
  checkFloatEq(right.lastDutyCmd / left.lastDutyCmd, rawRight / rawLeft,
              "the commanded ratio (4:1) is preserved exactly after scaling");
}

// 16. REVISED (sprint.md Revision's "Acknowledged, deliberate tradeoff",
// flips the superseded implementation's own AC#2): a differential trim
// riding on an exactly-zero common-mode component NO LONGER passes through
// unfloored -- it is now ALSO scaled up toward vMin, same as any other
// sub-floor pair, because a zero-common-mode differential is
// indistinguishable (as a raw wheel pair) from a symmetric pivot, and this
// is exactly the case scenario 14 above proves must be boosted. This is an
// intentional, accepted reversion for the one sub-case that is currently
// UNREACHABLE on any shipped robot profile (no robot JSON combines a
// nonzero wheel_v_min with a nonzero heading_hold_gain) -- see this
// function's own beginScenario() text and sprint.md's Revision section for
// the full acceptance reasoning. Sub-case B (a plain sub-floor travel
// command, no differential) is unchanged from the superseded implementation
// -- it was always boosted to vMin and still is.
void scenarioDifferentialWithNearZeroCommonModeNowBoostedTowardVMin() {
  beginScenario("131-003 (revised, was AC#2): a differential trim with a near-zero common mode "
                "is now ALSO boosted toward vMin -- reverting to pre-131-003 behavior for this "
                "one sub-case, accepted because it is unreachable on any shipped robot profile "
                "(see sprint.md's Revision section) and because Drive cannot tell a zero-common- "
                "mode differential apart from a symmetric pivot (scenario 14) from the raw wheel "
                "pair alone");

  App::Drive::AdaptationBounds bounds;
  bounds.vMin = 100.0f;  // [mm/s] test-local floor -- NOT tovez.json's real 99.7

  // Sub-case A: common mode EXACTLY zero, small differential trim -- raw
  // pair (-3, +3) is bit-identical in shape to scenario 14's symmetric
  // pivot (just a smaller magnitude), so it is boosted the same way: both
  // wheels scaled by vMin/3 to land at exactly (-vMin, +vMin).
  {
    MockMotor left;
    MockMotor right;
    App::Drive drive(left, right, 200.0f);
    drive.setDutyPerSpeed(1.0f, 1.0f);
    drive.setAdaptationBounds(bounds);

    const float differential = 3.0f;  // [mm/s]
    drive.tick(wheelCmd(-differential, differential));

    checkFloatEq(left.lastDutyCmd, -bounds.vMin,
                "zero common mode: the differential is now boosted to exactly -vMin, same "
                "treatment as a symmetric pivot (scenario 14) -- this is the accepted, "
                "unreachable-on-any-shipped-profile tradeoff, not an oversight");
    checkFloatEq(right.lastDutyCmd, bounds.vMin,
                "zero common mode: same, right wheel boosted to exactly +vMin");
  }

  // Sub-case B: a small NONZERO common mode (no differential at all -- a
  // plain sub-floor travel command) still gets boosted to vMin, sign
  // preserved -- UNCHANGED from the superseded implementation and from
  // applySpeedFloor()'s original per-wheel behavior alike, since both
  // wheels carry the same magnitude here regardless of which algorithm is
  // in effect.
  {
    MockMotor left;
    MockMotor right;
    App::Drive drive(left, right, 200.0f);
    drive.setDutyPerSpeed(1.0f, 1.0f);
    drive.setAdaptationBounds(bounds);

    const float subFloorCommon = 5.0f;  // [mm/s] -- below vMin, no differential
    drive.tick(wheelCmd(subFloorCommon, subFloorCommon));

    checkFloatEq(left.lastDutyCmd, bounds.vMin,
                "no differential: boost-to-vMin is UNCHANGED -- both wheels boosted to +vMin");
    checkFloatEq(right.lastDutyCmd, bounds.vMin,
                "same, right wheel");
  }
}

// 17. REVISED (sprint.md Revision point 5, the ratio-preserving scale's own
// "stop is stop" strengthening): a wheel whose RAW cmdVelocity is exactly
// 0.0f -- because it is the incidental product of vector cancellation, part
// of an active, asymmetric command -- now stays at EXACTLY 0.0f after
// scaling (`0 * scale == 0` in IEEE-754 for any finite scale), where the
// superseded common-mode/differential design would have driven it nonzero
// and had to specially justify that as "intentional." This is a
// simplification, not a loss of coverage: the OTHER wheel of the same pair
// still gets boosted normally, proving the scale genuinely ran (this isn't
// "nothing happened"), while the raw-zero wheel itself provably never
// moves. A GENUINE full stop (BOTH raw wheel commands exactly 0.0f) is
// unaffected and still recombines to exactly (0.0f, 0.0f) -- "stop is stop"
// holds, same as always.
void scenarioRawZeroWheelStaysZeroUnderRatioPreservingScale() {
  beginScenario("131-003 (revised): a wheel whose RAW command is exactly 0.0f from vector "
                "cancellation now STAYS at exactly 0.0f even though the pair is scaled up (0 * "
                "scale == 0) -- simpler than the superseded implementation's own "
                "intentionally-driven-nonzero case -- while a GENUINE full stop (both wheels "
                "raw zero) still recombines to exactly (0.0, 0.0)");

  App::Drive::AdaptationBounds bounds;
  bounds.vMin = 100.0f;  // [mm/s] test-local floor

  // Sub-case A: raw pair (0, 50) -- the left wheel's raw command happens to
  // be exactly zero (e.g. the incidental result of some upstream vector
  // cancellation), the right wheel is nonzero but below vMin. The dominant
  // wheel (50) is below vMin, so the pair IS scaled (by vMin/50 == 2) -- but
  // 0 * 2 == 0 exactly, so the left wheel stays at exactly zero while the
  // right wheel is boosted to exactly vMin. Under the superseded
  // common-mode/differential design this same raw pair (common mode 25,
  // differential 25) would have driven the left wheel to a nonzero 50 --
  // this scenario proves the new algorithm does NOT do that.
  {
    MockMotor left;
    MockMotor right;
    App::Drive drive(left, right, 200.0f);
    drive.setDutyPerSpeed(1.0f, 1.0f);
    drive.setAdaptationBounds(bounds);

    const float rawLeft = 0.0f;   // [mm/s] -- exactly zero, from vector cancellation
    const float rawRight = 50.0f;  // [mm/s] -- the dominant wheel, below vMin

    drive.tick(wheelCmd(rawLeft, rawRight));

    checkFloatEq(left.lastDutyCmd, 0.0f,
                "the raw-zero left wheel stays at EXACTLY 0.0 duty even though the pair is "
                "scaled up -- 0 * scale == 0 always, unlike the superseded implementation which "
                "drove this wheel nonzero");
    checkFloatEq(right.lastDutyCmd, bounds.vMin,
                "the right (dominant) wheel IS boosted to exactly vMin, proving the scale "
                "genuinely ran -- this isn't a case where nothing happened");
  }

  // Sub-case B: contrast -- a GENUINE full stop (both wheels' raw command
  // exactly 0.0f) still recombines to exactly (0.0, 0.0): dominantMag is
  // exactly 0.0f, so applySpeedFloor()'s own "dominantMag <= 0.0f" guard
  // passes the pair through completely unscaled -- unaffected by this
  // ticket (see also scenarioStopZeroesBothTargetsWithinOneCycle() above,
  // which proves this same invariant through a full NezhaMotor round trip).
  {
    MockMotor left;
    MockMotor right;
    App::Drive drive(left, right, 200.0f);
    drive.setDutyPerSpeed(1.0f, 1.0f);
    drive.setAdaptationBounds(bounds);

    drive.tick(wheelCmd(0.0f, 0.0f));

    checkFloatEq(left.lastDutyCmd, 0.0f,
                "genuine full stop: left wheel writes EXACTLY 0.0 duty, unaffected by the "
                "ratio-preserving scale");
    checkFloatEq(right.lastDutyCmd, 0.0f,
                "genuine full stop: right wheel writes EXACTLY 0.0 duty");
  }
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
  scenarioCommandedZeroForcesStageBToZeroAndFreezesIntegrator();
  scenarioStaleDisconnectedOrFrozenWheelFreezesStageBIntegrator();
  scenarioWheelsAndMoveReachIdenticalDriveBehavior();
  scenarioTakeoverPreservesLearnedStateEstopFullyResets();
  scenarioTakeoverDoesNotTouchStopReassertionCountdown();
  scenarioReversalAfterConvergedForwardBiasDoesNotReduceMagnitude();
  scenarioBiasPersistsAcrossChainedTakeoverBoundaries();
  scenarioDifferentialTrimAtFloorBoundaryStaysProportional();
  scenarioSymmetricPivotBelowFloorBoostsToExactVMin();
  scenarioAsymmetricPairBelowFloorPreservesRatio();
  scenarioDifferentialWithNearZeroCommonModeNowBoostedTowardVMin();
  scenarioRawZeroWheelStaysZeroUnderRatioPreservingScale();

  if (g_failureCount == 0) {
    std::printf("OK: all App::Drive scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the App::Drive scenarios\n", g_failureCount);
  return 1;
}
