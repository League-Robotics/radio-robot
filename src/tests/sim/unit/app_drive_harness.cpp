// app_drive_harness.cpp -- off-hardware acceptance harness for ticket
// 103-006 (SUC-006), App::Drive (src/firm/app/drive.{h,cpp}). Proves:
// setDuty() stages the raw left/right values directly and tick() applies
// the last-staged target onto the two REAL Hardware::NezhaMotor leaves, and
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
// i2c_bus_host.cpp scripted-FIFO Platform::I2CBus fake (ticket 001 reduced
// Platform::I2CBus to a pure interface and removed it) onto a
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
#include "hal/device_config.h"
#include "hardware/nezha/nezha_motor.h"
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

// 134-002: applySpeedFloor() is a TELEOP affordance -- it engages only
// while Drive itself owns motion (Drive::owns()). A freshly-constructed
// Drive owns nothing (commandActive_ == false, the planner's state), so
// every floor scenario below arms a WHEELS command first and only then
// ticks the same pair through the blackboard -- which is also what
// robot_loop.cpp does for real, one cycle apart: Drive::update() publishes
// its armed targets onto state, Drive::tick() reads them back. The
// duration is deliberately enormous so the deadman never expires inside a
// single-tick scenario; the expiry edge has its own coverage in
// scenarioSpeedFloorIsATeleopAffordanceNotAGlobalOne() below, which also
// pins the un-owned (planner) side of the gate.
void armTeleop(App::Drive& drive, float vLeft, float vRight) {  // [mm/s] x2
  drive.command(vLeft, vRight, /*duration=*/1.0e6f, /*moveId=*/1, /*now=*/0);
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
// A dependency-free Hal::Motor double, local to this harness, for the
// stop-re-assertion scenario below -- it needs direct, test-settable
// control over velocity() independent of any plant physics, which the real
// NezhaMotor leaves used by scenarios 1/2 above can't give without a much
// larger scripted-encoder sequence. Records setDuty() call COUNT (not just
// the last value, unlike the real leaves' appliedDuty()) -- that count is
// this scenario's own oracle for "did Drive actually re-issue the write,
// or did the quiet-at-zero shortcut swallow it."
class MockMotor : public Hal::Motor {
 public:
  void begin() override {}
  void requestSample() override {}
  void setDuty(float duty) override { lastDutyCmd = duty; ++setDutyCalls; }
  void setNeutral(Hal::Neutral) override {}
  void applyTravelCalib(float) override {}
  bool reconfigure(const Hal::MotorConfig&) override { return true; }
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

// --- Hardware::NezhaMotor scripting helpers (duplicated from
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

Hal::MotorConfig baseNezhaConfig(uint32_t port) {
  Hal::MotorConfig cfg;
  cfg.port = port;
  cfg.fwdSign = 1;
  cfg.wheelTravelCalib = 1.0f;
  return cfg;
}

// Primes a fresh leaf at position 0 (one request->collect cycle at nowUs=0)
// so lastPosition_/lastTickUs_ are established before any staged target is
// executed -- mirrors devices_motor_harness.cpp's own "prime cycle"
// convention.
void primeAtZero(Hardware::NezhaMotor& motor, TestSim::ScriptedI2CHook& bus, uint16_t wireAddr) {
  scriptEncoderRequestCollect(bus, wireAddr, 0.0f);
  motor.requestSample();
  motor.tick(0);
}

// Runs one more request->collect + tick() cycle at the given time, holding
// position at 0 (so velocity() stays exactly 0 -- isolates the staged
// target's effect on appliedDuty() from any plant convergence dynamics).
void runOneCycleAtZeroPosition(Hardware::NezhaMotor& motor, TestSim::ScriptedI2CHook& bus,
                                uint16_t wireAddr, uint64_t nowUs) {
  scriptEncoderRequestCollect(bus, wireAddr, 0.0f);
  motor.requestSample();
  motor.tick(nowUs);
}

// ===========================================================================
// 1. setDuty() stages the raw v_left/v_right values directly (AC #1) --
//    the ONE staging path Drive has left post-122-002. tick() runs the
//    interim PID (this file's own header) and forwards the resulting duty
//    to Hal::Motor::setDuty().
// ===========================================================================

void scenarioSetDutyStagesRawValues() {
  beginScenario("Drive::tick(dutyL, dutyR): applies the duty pair to both leaves");

  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  const uint16_t wireAddr = static_cast<uint16_t>(Hardware::kNezhaDeviceAddr << 1);

  Hardware::NezhaMotor left(plant, baseNezhaConfig(1));
  Hardware::NezhaMotor right(plant, baseNezhaConfig(2));
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
// STALL DETECTION -- 2026-08-08, stakeholder directive after the robot drove
// into the playfield rails repeatedly and ground there with nothing noticing.
//
// The condition is deliberately three-part -- a real demand for motion AND no
// measured motion AND an encoder healthy enough to be believed -- and the
// scenarios below pin each part by REMOVING it and proving the stall does not
// latch. The encoder-health part matters most: without it a wedged encoder
// reads zero speed forever and would halt a perfectly healthy robot.
// ===========================================================================

// Mirrors tovez.json's shipped stall values so the harness exercises the
// geometry the real robot runs, not a convenient made-up one.
App::Drive::AdaptationBounds stallBounds() {
  App::Drive::AdaptationBounds bounds;
  bounds.stallSpeed = 15.0f;    // [mm/s]
  bounds.stallDemand = 40.0f;   // [mm/s]
  bounds.stallWindow = 500.0f;  // [ms]
  return bounds;
}

// One cycle of blackboard state: what was commanded, what the OTOS measured
// of ACTUAL ground travel, and what the encoders read. The OTOS is what the
// stall test believes -- see drive.cpp's stall block for the measured reason.
Types::RobotState stallState(float cmd, float otosVx, uint32_t now,  // [mm/s] [mm/s] [ms]
                             bool otosOk = true, float otosOmega = 0.0f,
                             float wheelVel = 0.0f, bool connected = true,
                             bool frozen = false) {
  Types::RobotState state;
  state.time.cycleStart = now;
  state.wheelLeft.cmdVelocity = cmd;
  state.wheelRight.cmdVelocity = cmd;
  state.wheelLeft.velocity = wheelVel;
  state.wheelRight.velocity = wheelVel;
  state.wheelLeft.connected = connected;
  state.wheelRight.connected = connected;
  state.wheelLeft.sampleTime = now;  // age 0, inside kMaxSampleAge
  state.wheelRight.sampleTime = now;
  state.health.wheelFrozenLeft = frozen;
  state.health.wheelFrozenRight = frozen;
  state.otos.present = otosOk;
  state.otos.connected = otosOk;
  state.otos.v_x = otosVx;
  state.otos.omega = otosOmega;
  state.otos.sampleTime = now;
  return state;
}

// Drives `cycles` ticks of the same condition, 32ms apart (kCycle), starting
// at a deliberately nonzero time -- updateStall() uses `since == 0` as its
// unarmed sentinel, so a scenario starting at t=0 would never arm.
void tickStall(App::Drive& drive, float cmd, float otosVx, uint32_t cycles,
               bool otosOk = true, float otosOmega = 0.0f,
               float wheelVel = 0.0f, bool connected = true,
               bool frozen = false) {
  for (uint32_t i = 0; i < cycles; ++i) {
    const uint32_t now = 1000u + i * 32u;  // [ms]
    Types::RobotState state = stallState(cmd, otosVx, now, otosOk, otosOmega,
                                         wheelVel, connected, frozen);
    drive.tick(state);
  }
}

// A Drive calibrated and armed so tick() actually runs its body: tick()
// early-returns unless setDutyPerSpeed() has been called, and the stall test
// reads the blackboard regardless of who owns motion.
struct StallRig {
  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus{plant};
  Hardware::NezhaMotor left{plant, baseNezhaConfig(1)};
  Hardware::NezhaMotor right{plant, baseNezhaConfig(2)};
  App::Drive drive{left, right, 200.0f};

  StallRig() {
    const uint16_t wireAddr = static_cast<uint16_t>(Hardware::kNezhaDeviceAddr << 1);
    primeAtZero(left, bus, wireAddr);
    primeAtZero(right, bus, wireAddr);
    drive.setDutyPerSpeed(1.0f, 1.0f);
    drive.setAdaptationBounds(stallBounds());
  }
};

void scenarioStallLatchesOnCommandedMotionThatIsNotHappening() {
  beginScenario("stall: commanded 150mm/s, OTOS says the robot is not moving");
  StallRig rig;
  // 500ms window at 32ms cycles: 16 ticks spans 480ms (not yet), 17 spans 512.
  tickStall(rig.drive, 150.0f, 0.0f, 16);
  checkTrue(!rig.drive.stallLeft(), "no stall at 480ms -- still inside the window");
  tickStall(rig.drive, 150.0f, 0.0f, 17);
  checkTrue(rig.drive.stallLeft(), "stall latches once the window elapses");
  checkTrue(rig.drive.stallRight(), "both wheels report it -- the ROBOT is stuck");
}

void scenarioWheelsSlippingAgainstAWallStillStalls() {
  beginScenario("stall: wheels SPINNING against a wall is still a stall");
  StallRig rig;
  // The rail case measured on tovez: 17cm of encoder travel while the robot
  // sat still. An encoder-only test reads this as healthy driving; the OTOS
  // reads the truth. This is the scenario the first design got wrong.
  tickStall(rig.drive, 150.0f, /*otosVx=*/2.0f, 40, /*otosOk=*/true,
            /*otosOmega=*/0.0f, /*wheelVel=*/150.0f);
  checkTrue(rig.drive.stallLeft(), "slipping wheels must not hide a jammed robot");
}

void scenarioWedgedEncoderWithRobotMovingIsNotAStall() {
  beginScenario("stall: a wedged encoder on a MOVING robot is not a stall");
  StallRig rig;
  // Encoders frozen and flagged wedged, but the OTOS says the robot is
  // travelling. This is the false positive the OTOS gate exists to prevent,
  // and it is why the encoder reading cannot be the primary signal.
  tickStall(rig.drive, 150.0f, /*otosVx=*/140.0f, 40, /*otosOk=*/true,
            /*otosOmega=*/0.0f, /*wheelVel=*/0.0f, /*connected=*/true,
            /*frozen=*/true);
  checkTrue(!rig.drive.stallLeft(), "a moving robot must never latch a stall");
}

void scenarioPivotIsNotAStall() {
  beginScenario("stall: an in-place PIVOT translates nothing but is not stalled");
  StallRig rig;
  // v_x is ~0 through a pivot, so a v_x-only test would call every turn a
  // stall. 1.0 rad/s across a 200mm track is 100mm/s at the rim.
  tickStall(rig.drive, 150.0f, /*otosVx=*/0.0f, 40, /*otosOk=*/true,
            /*otosOmega=*/1.0f);
  checkTrue(!rig.drive.stallLeft(), "rotation counts as moving");
}

void scenarioNoOtosFallsBackToEncoders() {
  beginScenario("stall: with no usable OTOS the encoders are the fallback");
  StallRig rig;
  tickStall(rig.drive, 150.0f, /*otosVx=*/0.0f, 40, /*otosOk=*/false,
            /*otosOmega=*/0.0f, /*wheelVel=*/0.0f);
  checkTrue(rig.drive.stallLeft(), "encoder fallback still protects the robot");
  App::Drive::AdaptationBounds b = stallBounds();
  rig.drive.setAdaptationBounds(b);
}

void scenarioStandingSubDemandCommandIsNotAStall() {
  beginScenario("stall: a command below stallDemand is not a demand for motion");
  StallRig rig;
  // 30mm/s is under the 40mm/s demand floor: a standing sub-breakaway command
  // legitimately does not move the robot, and is not a jam.
  tickStall(rig.drive, 30.0f, 0.0f, 40);
  checkTrue(!rig.drive.stallLeft(), "sub-demand command must not latch a stall");
}

void scenarioMovingRobotIsNotAStall() {
  beginScenario("stall: a robot travelling above stallSpeed is not stalled");
  StallRig rig;
  tickStall(rig.drive, 150.0f, 60.0f, 40);
  checkTrue(!rig.drive.stallLeft(), "a moving robot must not latch a stall");
}

void scenarioZeroWindowDisablesTheDetector() {
  beginScenario("stall: stall_window = 0 disables the detector entirely");
  StallRig rig;
  App::Drive::AdaptationBounds off = stallBounds();
  off.stallWindow = 0.0f;
  rig.drive.setAdaptationBounds(off);
  tickStall(rig.drive, 150.0f, 0.0f, 40);
  checkTrue(!rig.drive.stallLeft(), "zero window is off, matching the 0-is-disabled convention");
}

void scenarioEstopClearsTheStallLatch() {
  beginScenario("stall: estop() clears the latch along with the rest of Drive");
  StallRig rig;
  tickStall(rig.drive, 150.0f, 0.0f, 40);
  checkTrue(rig.drive.stallLeft(), "precondition: the stall latched");
  rig.drive.estop();
  checkTrue(!rig.drive.stallLeft(), "estop() clears the left stall latch");
  checkTrue(!rig.drive.stallRight(), "estop() clears the right stall latch");
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
  const uint16_t wireAddr = static_cast<uint16_t>(Hardware::kNezhaDeviceAddr << 1);

  Hardware::NezhaMotor left(plant, baseNezhaConfig(1));
  Hardware::NezhaMotor right(plant, baseNezhaConfig(2));
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

void scenarioStageBITermIsAPositionTermWithNoSteadyGate() {
  beginScenario("Stage B (133-002): the I term reads POSITION -- it tracks the position error "
                "directly (no accumulator), it is NOT gated by aSteady, posErrMax clamps its INPUT "
                "in mm and iMax clamps its OUTPUT in mm/s, and iMax == 0 turns it off entirely");

  const float cmd = 200.0f;         // [mm/s] commanded speed
  const float dt = 0.05f;           // [s] one 50000us cycle
  const float perCycle = cmd * dt;  // [mm] commanded travel per cycle == 10mm

  App::Drive::ControlGains gains;
  gains.kp = 0.0f;    // P silenced: pidLeft() reads out the I term alone
  gains.ki = 0.5f;    // [1/s] -- 0.5 mm/s of correction per mm of position error
  gains.iMax = 500.0f;
  gains.kaff = 0.0f;
  gains.pidMax = 0.0f;  // no total clamp -- isolate the I path's own two bounds

  App::Drive::AdaptationBounds bounds;
  bounds.aSteady = 50.0f;  // [mm/s^2]
  // tauAdapt left at 0 (the all-zero default) -- Stage C stays off.

  // Runs `cycles` ticks of a stalled wheel (position pinned at 0 while cmd
  // is nonzero, so the position error is exactly the commanded travel so
  // far) and returns the final Stage B output. `cmdAccel` is a parameter
  // because the WHOLE POINT of one of the checks below is that it no
  // longer matters to this stage.
  auto runStalled = [&](int cycles, float cmdAccel, float posErrMax) -> float {
    MockMotor left;
    MockMotor right;
    App::Drive drive(left, right, /*trackWidth=*/200.0f);
    drive.setDutyPerSpeed(1.0f, 1.0f);  // identity calibration: target IS duty
    drive.setControlGains(gains);
    App::Drive::AdaptationBounds b = bounds;
    b.posErrMax = posErrMax;
    drive.setAdaptationBounds(b);

    uint32_t now = 1000;
    for (int i = 0; i < cycles; ++i) {
      Types::RobotState state;
      state.wheelLeft.cmdVelocity = cmd;
      state.wheelLeft.cmdAccel = cmdAccel;
      state.wheelLeft.position = 0.0f;  // stalled -- travels nothing
      state.wheelLeft.velocity = 0.0f;
      state.wheelLeft.connected = true;
      state.wheelLeft.sampleTime = now - 10;
      state.time.cycleStart = now;
      state.time.cyclePeriod = 50000;  // [us] -- dt = 0.05s
      drive.tick(state);
      now += 50;
    }
    return drive.pidLeft();
  };

  // (a) The I term IS the position error, times ki -- exactly, with no
  //     per-cycle accumulation constant of its own. The FIRST tick anchors
  //     the reference (armed == false until then) and reports zero error,
  //     so after N ticks the reference has advanced (N-1) cycles' worth.
  //     This is a closed form, not a trend: it can only hold if the term
  //     is a direct function of position, not a sum of velocity samples.
  for (int cycles : {2, 5, 11, 21}) {
    const float expectedError = static_cast<float>(cycles - 1) * perCycle;  // [mm]
    checkFloatEq(runStalled(cycles, /*cmdAccel=*/0.0f, /*posErrMax=*/0.0f),
                gains.ki * expectedError,
                "the I term equals ki * (commanded travel since the anchor) exactly -- it reads "
                "position, it does not accumulate velocity",
                0.01f);
  }

  // (b) NOT gated by aSteady. The old accumulator froze while
  //     |cmdAccel| >= aSteady, which is where the residual distance error
  //     was measurably being banked (a velocity loop can never repay
  //     banked distance afterwards). A ramping wheel now produces the
  //     IDENTICAL I term as a steady one.
  const float steadyPid = runStalled(21, /*cmdAccel=*/0.0f, /*posErrMax=*/0.0f);
  const float rampingPid = runStalled(21, /*cmdAccel=*/500.0f, /*posErrMax=*/0.0f);
  checkFloatEq(rampingPid, steadyPid,
              "a RAMPING wheel (|cmdAccel| well above aSteady) gets the same I term as a steady "
              "one -- there is no steady gate left to freeze, which is the point");
  checkTrue(std::fabs(steadyPid) > 1.0f,
           "setup: the I term is genuinely nonzero, so (b) is not comparing two zeros");

  // (c) posErrMax clamps the INPUT, in millimetres. With ki = 0.5 and a
  //     3mm clamp the output pins at 1.5 mm/s no matter how far behind the
  //     wheel falls -- and crucially, RAISING ki now raises the output
  //     (see (e)) instead of shrinking the remembered distance, which is
  //     the unit-domain bug this replaced.
  const float kPosClamp = 3.0f;  // [mm]
  checkFloatEq(runStalled(51, /*cmdAccel=*/0.0f, kPosClamp), gains.ki * kPosClamp,
              "posErrMax clamps the position error itself, in mm: the I term pins at ki*posErrMax "
              "however long the wheel stays behind");

  // (d) iMax clamps the OUTPUT, in mm/s -- a DIFFERENT domain from (c),
  //     and both are live at once. Deliberately asserted in the same
  //     scenario as (c): "posErrMax replaces iMax" is the misreading this
  //     ticket exists to make impossible.
  {
    MockMotor left;
    MockMotor right;
    App::Drive drive(left, right, /*trackWidth=*/200.0f);
    drive.setDutyPerSpeed(1.0f, 1.0f);
    App::Drive::ControlGains bothClamped = gains;
    bothClamped.iMax = 1.0f;  // [mm/s] -- tighter than ki*posErrMax below
    drive.setControlGains(bothClamped);
    App::Drive::AdaptationBounds b = bounds;
    b.posErrMax = 100.0f;  // [mm] -- deliberately generous, so iMax is what binds
    drive.setAdaptationBounds(b);

    uint32_t now = 1000;
    for (int i = 0; i < 51; ++i) {
      Types::RobotState state;
      state.wheelLeft.cmdVelocity = cmd;
      state.wheelLeft.position = 0.0f;
      state.wheelLeft.connected = true;
      state.wheelLeft.sampleTime = now - 10;
      state.time.cycleStart = now;
      state.time.cyclePeriod = 50000;
      drive.tick(state);
      now += 50;
    }
    checkFloatEq(drive.pidLeft(), 1.0f,
                "iMax clamps the I term's OUTPUT in mm/s, independently of posErrMax's mm clamp "
                "on its input -- both bounds are live, neither replaces the other");
  }

  // (e) The direction that used to be backwards: with the position error
  //     clamped in mm, MORE ki gives MORE correction. Under the old iMax-
  //     clamps-ki*error form, the remembered distance was iMax/ki, so
  //     raising ki SHRANK the memory and the sweep plateaued.
  {
    App::Drive::ControlGains lowKi = gains;
    App::Drive::ControlGains highKi = gains;
    lowKi.ki = 0.5f;
    highKi.ki = 4.0f;

    auto pidWith = [&](const App::Drive::ControlGains& g) -> float {
      MockMotor left;
      MockMotor right;
      App::Drive drive(left, right, /*trackWidth=*/200.0f);
      drive.setDutyPerSpeed(1.0f, 1.0f);
      drive.setControlGains(g);
      App::Drive::AdaptationBounds b = bounds;
      b.posErrMax = kPosClamp;
      drive.setAdaptationBounds(b);
      uint32_t now = 1000;
      for (int i = 0; i < 51; ++i) {
        Types::RobotState state;
        state.wheelLeft.cmdVelocity = cmd;
        state.wheelLeft.position = 0.0f;
        state.wheelLeft.connected = true;
        state.wheelLeft.sampleTime = now - 10;
        state.time.cycleStart = now;
        state.time.cyclePeriod = 50000;
        drive.tick(state);
        now += 50;
      }
      return drive.pidLeft();
    };

    checkFloatEq(pidWith(lowKi), lowKi.ki * kPosClamp, "low ki: saturated I term is ki*posErrMax");
    checkFloatEq(pidWith(highKi), highKi.ki * kPosClamp,
                "HIGH ki: the saturated I term GROWS with ki -- the position memory is posErrMax "
                "mm regardless of ki, not iMax/ki mm as the old clamp made it");
  }

  // (f) iMax == 0 turns the I term OFF ENTIRELY (fail closed), unchanged
  //     from the accumulator era's "0 disables integration". Every shipped
  //     robot JSON carries iMax = 0 today, so re-reading 0 as "unclamped"
  //     would hand all of them unbounded I authority the first time a
  //     nonzero ki was pushed.
  {
    App::Drive::ControlGains noIMax = gains;
    noIMax.iMax = 0.0f;
    MockMotor left;
    MockMotor right;
    App::Drive drive(left, right, /*trackWidth=*/200.0f);
    drive.setDutyPerSpeed(1.0f, 1.0f);
    drive.setControlGains(noIMax);
    drive.setAdaptationBounds(bounds);
    uint32_t now = 1000;
    for (int i = 0; i < 51; ++i) {
      Types::RobotState state;
      state.wheelLeft.cmdVelocity = cmd;
      state.wheelLeft.position = 0.0f;
      state.wheelLeft.connected = true;
      state.wheelLeft.sampleTime = now - 10;
      state.time.cycleStart = now;
      state.time.cyclePeriod = 50000;
      drive.tick(state);
      now += 50;
    }
    checkFloatEq(drive.pidLeft(), 0.0f,
                "iMax == 0: the I term contributes exactly nothing, however large the position "
                "error grows (fail closed, NOT 'unclamped')");
  }
}

// ===========================================================================
// 133-002: positionError()'s three re-anchor guards, each proven
// independently. All three must return EXACTLY zero error on the guarded
// cycle and re-establish the origin, rather than correcting against a
// reference that no longer means anything.
// ===========================================================================

void scenarioPositionErrorGuardsReAnchorWithoutCorrecting() {
  beginScenario("133-002: positionError() re-anchors without correcting on commanded zero, an "
                "epoch change, and a disconnect -- each guard proven on its own");

  const float cmd = 200.0f;  // [mm/s]
  const float perCycle = 10.0f;  // [mm] cmd * 0.05s

  App::Drive::ControlGains gains;
  gains.kp = 0.0f;    // P silenced: pidLeft() reads out the I term alone
  gains.ki = 1.0f;    // [1/s] -- pidLeft() is then numerically the position error in mm
  gains.iMax = 5000.0f;
  gains.pidMax = 0.0f;

  App::Drive::AdaptationBounds bounds;
  bounds.aSteady = 50.0f;

  // Drives one tick with fully-specified wheel state; returns Stage B's
  // output, which equals the position error [mm] under the gains above.
  struct Rig {
    MockMotor left;
    MockMotor right;
    App::Drive drive;
    uint32_t now = 1000;
    Rig(const App::Drive::ControlGains& g, const App::Drive::AdaptationBounds& b)
        : drive(left, right, 200.0f) {
      drive.setDutyPerSpeed(1.0f, 1.0f);
      drive.setControlGains(g);
      drive.setAdaptationBounds(b);
    }
    float step(float cmdVelocity, float position, bool connected, uint8_t epoch) {
      Types::RobotState state;
      state.wheelLeft.cmdVelocity = cmdVelocity;
      state.wheelLeft.position = position;
      state.wheelLeft.velocity = 0.0f;
      state.wheelLeft.connected = connected;
      state.wheelLeft.positionEpoch = epoch;
      state.wheelLeft.sampleTime = now - 10;
      state.time.cycleStart = now;
      state.time.cyclePeriod = 50000;  // [us]
      drive.tick(state);
      now += 50;
      return drive.pidLeft();
    }
  };

  // (a) COMMANDED ZERO -- stop is stop. A commanded-zero wheel reports no
  //     error at all, and the reference it had built is DISCARDED: on
  //     re-command, the error restarts from zero rather than resuming the
  //     debt from before the stop. (Holding the old reference would make
  //     Stage B a servo that creeps a stopped wheel back onto it -- the
  //     runaway class 133-001 exists to prevent.)
  {
    Rig rig(gains, bounds);
    rig.step(cmd, /*position=*/0.0f, true, 0);          // anchor
    rig.step(cmd, /*position=*/0.0f, true, 0);          // 10mm behind
    const float behind = rig.step(cmd, 0.0f, true, 0);  // 20mm behind
    checkFloatEq(behind, 2.0f * perCycle, "setup: the wheel is genuinely 20mm behind its reference");

    checkFloatEq(rig.step(/*cmdVelocity=*/0.0f, 0.0f, true, 0), 0.0f,
                "commanded zero: Stage B contributes exactly 0.0 -- stop is stop");
    rig.step(cmd, 0.0f, true, 0);  // re-command: this cycle re-arms the anchor
    checkFloatEq(rig.step(cmd, 0.0f, true, 0), perCycle,
                "after a commanded-zero period the reference is DISCARDED, not resumed: the error "
                "restarts at one cycle's travel, not the 20mm+ owed before the stop");
  }

  // (b) EPOCH CHANGE -- RobotLoop rebaselined the register underneath us,
  //     so `position` just stepped discontinuously. A step is not travel:
  //     the loop must re-anchor on the new value, never charge the step to
  //     the wheel as a huge instantaneous over-travel.
  {
    Rig rig(gains, bounds);
    rig.step(cmd, /*position=*/1000.0f, true, /*epoch=*/0);  // anchor at 1000mm
    checkFloatEq(rig.step(cmd, 1000.0f, true, 0), perCycle, "setup: 10mm behind before the rebaseline");

    // The rebaseline: position drops to 0 AND the epoch increments. Read
    // naively that is 1000mm of travel in one cycle.
    checkFloatEq(rig.step(cmd, /*position=*/0.0f, true, /*epoch=*/1), 0.0f,
                "epoch change: exactly 0.0 error -- the 1000mm discontinuity is NOT read as "
                "travel, and the pre-step debt is not carried across it either");
    // The re-anchor RE-ARMS in the same cycle here (unlike the
    // commanded-zero case above, which leaves `armed` false because the
    // wheel is not being commanded anywhere): the command is still live
    // and the new reading is trustworthy, so no cycle is wasted.
    checkFloatEq(rig.step(cmd, 0.0f, true, 1), perCycle,
                "and the reference resumes cleanly from the new origin on the very next cycle");
  }

  // (c) DISCONNECT -- Hardware::NezhaMotor::collectEncoder() hands back its
  //     last good raw value on a failed read, so a disconnected wheel's
  //     position is MANUFACTURED, not measured. Anchoring against it would
  //     bank a fictitious deficit for as long as the bus stays down.
  {
    Rig rig(gains, bounds);
    rig.step(cmd, /*position=*/0.0f, /*connected=*/true, 0);
    checkFloatEq(rig.step(cmd, 0.0f, true, 0), perCycle, "setup: 10mm behind while connected");

    checkFloatEq(rig.step(cmd, 0.0f, /*connected=*/false, 0), 0.0f,
                "disconnected: exactly 0.0 error -- Stage B does not correct against a "
                "manufactured position");
    checkFloatEq(rig.step(cmd, 0.0f, /*connected=*/false, 0), 0.0f,
                "and stays at 0.0 for as long as the wheel is disconnected (armed never latches "
                "while connected is false)");
    rig.step(cmd, 0.0f, /*connected=*/true, 0);  // reconnect: this cycle re-arms
    checkFloatEq(rig.step(cmd, 0.0f, true, 0), perCycle,
                "on reconnect the reference re-anchors on the live reading and starts fresh");
  }
}

// ===========================================================================
// 133-002, the headline defect: a DROPPED encoder sample must not
// permanently delete real distance from the loop's position estimate.
//
// The old accumulator summed a derived velocity, gated by a freshness test
// that FROZE the sum on exactly the ticks where distance was being
// travelled unobserved -- so travel during a dropped sample was lost for
// good. The position REGISTER carries that travel across the gap
// (Hardware::NezhaMotor::collectEncoder() returns the last good value, then
// the next successful read reports the full advance), so the position-
// domain term simply sees the truth again the moment it can look.
//
// Proven by equality, not by a trend: a run with dropped samples in the
// middle must end at the IDENTICAL position error as a run with none, for
// the same true travel.
// ===========================================================================

void scenarioDroppedSampleDoesNotDeleteDistanceFromThePositionEstimate() {
  beginScenario("133-002: a stale/dropped encoder sample does not permanently delete distance from "
                "Stage B's position estimate -- the register carries the truth across the gap");

  const float cmd = 200.0f;         // [mm/s]
  const float perCycle = 10.0f;     // [mm] commanded travel per 50ms cycle
  const float trueRate = 9.0f;      // [mm] the wheel actually travels 90% of commanded

  App::Drive::ControlGains gains;
  gains.kp = 0.0f;    // P silenced: pidLeft() reads out the I term alone
  gains.ki = 1.0f;    // [1/s] -- pidLeft() is then numerically the position error in mm
  gains.iMax = 5000.0f;
  gains.pidMax = 0.0f;

  App::Drive::AdaptationBounds bounds;
  bounds.aSteady = 50.0f;
  // posErrMax left at 0 (unclamped) so the full accumulated deficit is
  // observable -- the clamp is scenario 7's subject, not this one's.

  // Runs `cycles` ticks at a constant command. The wheel truly travels
  // trueRate per cycle throughout. When `dropFrom <= i < dropUntil`, the
  // REPORTED position is held at its pre-drop value (what a failed collect
  // does) and then jumps to the true value on the next good read -- the
  // wheel never stops moving, only the reporting does.
  auto finalError = [&](int cycles, int dropFrom, int dropUntil) -> float {
    MockMotor left;
    MockMotor right;
    App::Drive drive(left, right, /*trackWidth=*/200.0f);
    drive.setDutyPerSpeed(1.0f, 1.0f);
    drive.setControlGains(gains);
    drive.setAdaptationBounds(bounds);

    uint32_t now = 1000;
    float heldPosition = 0.0f;  // [mm] what a stale register keeps reporting
    for (int i = 0; i < cycles; ++i) {
      const float truePosition = static_cast<float>(i) * trueRate;  // [mm]
      const bool dropped = (i >= dropFrom && i < dropUntil);
      if (!dropped) heldPosition = truePosition;

      Types::RobotState state;
      state.wheelLeft.cmdVelocity = cmd;
      state.wheelLeft.position = heldPosition;
      state.wheelLeft.velocity = 0.0f;
      // connected stays TRUE: a stale read is not a disconnect. The
      // register's value is the last TRUE position, held -- distinct from
      // the manufactured value a disconnected wheel reports, which
      // positionError() does re-anchor on (scenario 8c).
      state.wheelLeft.connected = true;
      state.wheelLeft.sampleTime = now - 10;
      state.time.cycleStart = now;
      state.time.cyclePeriod = 50000;  // [us]
      drive.tick(state);
      now += 50;
    }
    return drive.pidLeft();
  };

  const int kCycles = 41;
  // Anchored on tick 0; the reference accumulates over the remaining 40
  // cycles while the wheel truly travels 9mm per cycle from position 0.
  const float expected = static_cast<float>(kCycles - 1) * (perCycle - trueRate);  // 40mm

  const float clean = finalError(kCycles, /*dropFrom=*/-1, /*dropUntil=*/-1);
  checkFloatEq(clean, expected,
              "baseline (no dropped samples): the position error is the full accumulated deficit",
              0.05f);

  for (const auto& window : std::vector<std::pair<int, int>>{{5, 8}, {12, 20}, {30, 39}}) {
    const float withDrops = finalError(kCycles, window.first, window.second);
    checkFloatEq(withDrops, clean,
                "a run with dropped samples ends at the IDENTICAL position error as a clean run -- "
                "the distance travelled unobserved is recovered in full, not deleted",
                0.05f);
  }

  // Not a trivially-passing check: the deficit being tracked is real and
  // large compared with the tolerance above.
  checkTrue(clean > 30.0f, "setup: the accumulated deficit is genuinely large (not ~0)");
}

// ===========================================================================
// 133-002 (§7-E), LANDING UNVALIDATED: Drive::update() now publishes a
// commanded-accel estimate on the WHEELS path, which nothing ever did
// before -- the field held a stale planner value or zero while Drive owned
// motion, so Stage B's feedforward multiplied zero and Stage C's steady
// gate was pinned true, and the kff and aSteady sweeps each measured
// nothing as a result.
//
// This scenario asserts exactly two things and deliberately no more,
// because no measurement supports more: the estimate IS published and
// tracks the sign/rough magnitude of a real command change, and it is
// INERT at kaff == 0 (which every shipped robot JSON carries, and which
// is the only reason this is safe to land ahead of ticket 004's bench
// session). Sprint 133 Open Question 2 -- do not read this scenario as
// closing it.
// ===========================================================================

void scenarioUpdatePublishesCmdAccelOnTheWheelsPathInertAtZeroKaff() {
  beginScenario("133-002 (UNVALIDATED, Open Question 2): update() publishes a smoothed cmdAccel on "
                "the WHEELS path, and it is INERT at kaff == 0");

  MockMotor left;
  MockMotor right;
  App::Drive drive(left, right, /*trackWidth=*/200.0f);
  drive.setDutyPerSpeed(1.0f, 1.0f);

  Types::RobotState state;
  state.time.cyclePeriod = 50000;  // [us] -- dt = 0.05s

  // Baseline: an idle Drive publishes exactly zero accel.
  drive.command(0.0f, 0.0f, /*duration=*/100000.0f, /*moveId=*/1, /*now=*/500);
  drive.update(state, /*now=*/1000);
  checkFloatEq(state.wheelLeft.cmdAccel, 0.0f, "an unchanging command publishes zero cmdAccel");

  // A step up in commanded velocity produces a POSITIVE accel estimate,
  // bounded above by the raw difference quotient (the first-order filter
  // approaches it, never overshoots it on a single step).
  drive.command(200.0f, 200.0f, /*duration=*/100000.0f, /*moveId=*/2, /*now=*/1000);
  drive.update(state, /*now=*/1050);
  const float rawStep = 200.0f / 0.05f;  // [mm/s^2] 4000
  checkTrue(state.wheelLeft.cmdAccel > 0.0f,
           "a step UP in commanded velocity publishes a positive cmdAccel");
  checkTrue(state.wheelLeft.cmdAccel < rawStep,
           "and it is SMOOTHED, strictly below the raw difference quotient -- the host re-arms "
           "slower than kCycle, so a bare finite difference alternates a double-size spike with a "
           "zero");
  checkFloatEq(state.wheelRight.cmdAccel, state.wheelLeft.cmdAccel,
              "both wheels get their own estimate, equal here because both were commanded equally");

  // A step DOWN produces a negative estimate -- it is a signed accel, not
  // a magnitude.
  drive.command(0.0f, 0.0f, /*duration=*/100000.0f, /*moveId=*/3, /*now=*/1050);
  drive.update(state, /*now=*/1100);
  checkTrue(state.wheelLeft.cmdAccel < 0.0f,
           "a step DOWN publishes a negative cmdAccel -- signed, not a magnitude");

  // INERT at kaff == 0: Stage B's output is bit-identical whether the
  // published accel is a large positive number or exactly zero. This is
  // the property that makes landing an unvalidated estimator safe on
  // every currently-shipped robot.
  auto pidWithCmdAccel = [](float cmdAccel) -> float {
    MockMotor l;
    MockMotor r;
    App::Drive d(l, r, /*trackWidth=*/200.0f);
    d.setDutyPerSpeed(1.0f, 1.0f);
    App::Drive::ControlGains gains;
    gains.kp = 0.05f;
    gains.ki = 0.5f;
    gains.iMax = 50.0f;
    gains.kaff = 0.0f;  // the shipped value on every robot JSON today
    d.setControlGains(gains);

    uint32_t now = 1000;
    for (int i = 0; i < 10; ++i) {
      Types::RobotState s;
      s.wheelLeft.cmdVelocity = 200.0f;
      s.wheelLeft.cmdAccel = cmdAccel;
      s.wheelLeft.position = 0.0f;
      s.wheelLeft.velocity = 0.0f;
      s.wheelLeft.connected = true;
      s.wheelLeft.sampleTime = now - 10;
      s.time.cycleStart = now;
      s.time.cyclePeriod = 50000;
      d.tick(s);
      now += 50;
    }
    return d.pidLeft();
  };
  checkFloatEq(pidWithCmdAccel(4000.0f), pidWithCmdAccel(0.0f),
              "at kaff == 0 a large published cmdAccel changes Stage B's output by exactly nothing "
              "-- the estimator is inert on every shipped robot, which is the only reason it lands "
              "before its own measurement (Open Question 2)");
}

// ===========================================================================
// 131-002 (issue A-commanded-zero-leaks-through-stage-b.md), carried
// forward to the position domain by 133-002: commanded-zero through Stage
// B, and what Stage B does with a reading it cannot trust.
// correctedCommand()'s `desired == 0.0f` guard (Stage A) has NO effect on
// Stage B -- it is a fully separate computation, so a wheel that has not
// yet coasted to rest keeps a nonzero err even after its commanded speed
// reaches exactly 0. The two scenarios below prove: (1) a commanded-zero
// wheel writes exactly 0 duty every tick and its position reference is
// DISCARDED rather than resumed (133-002 inverted 131-002's "frozen, not
// reset" contract here on purpose -- see the scenario's own Phase 3); (2) a
// disconnected wheel still contributes exactly nothing, while a merely
// STALE or FROZEN reading is now permitted to correct, bounded.
// ===========================================================================

// 9. Commanded speed exactly 0.0f -> Stage B's contribution and the written
//    duty are exactly 0.0 every tick, no matter how long -- and the
//    position reference is DISCARDED across the zero period, so motion
//    resumes with no debt carried over from before the stop.
void scenarioCommandedZeroForcesStageBToZeroAndDiscardsTheReference() {
  beginScenario("Stage B: commanded-zero forces the contribution and written duty to exactly 0.0, "
                "and DISCARDS the position reference across the zero period (131-002, 133-002)");

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
  gains.pidMax = 40.0f;
  drive.setControlGains(gains);

  App::Drive::AdaptationBounds bounds;
  bounds.aSteady = 50.0f;  // [mm/s^2] -- no longer read by Stage B; Stage C is off (tauAdapt 0)
  drive.setAdaptationBounds(bounds);

  const float cmd = 200.0f;  // [mm/s] commanded cruise speed
  uint32_t now = 1000;

  // Phase 1: build a genuinely large position error -- the wheel is
  // stalled (position pinned at 0) while 200 mm/s is commanded, so it
  // falls 10mm further behind every cycle.
  for (int i = 0; i < 10; ++i) {
    Types::RobotState state;
    state.wheelLeft.cmdVelocity = cmd;
    state.wheelLeft.position = 0.0f;  // stalled -- travels nothing
    state.wheelLeft.velocity = 0.0f;
    state.wheelLeft.connected = true;
    state.wheelLeft.sampleTime = now - 10;
    state.time.cycleStart = now;
    state.time.cyclePeriod = 50000;  // [us] -- dt = 0.05s
    drive.tick(state);
    now += 50;
  }
  const float pidBeforeZero = drive.pidLeft();
  checkTrue(pidBeforeZero > gains.kp * cmd + 1.0f,
           "setup: the I term genuinely contributed on top of the p term before the zero period");

  // Phase 2: command exactly zero for many ticks -- Stage B's contribution
  // and the written duty must be EXACTLY zero every single tick, however
  // far behind the wheel was when the stop arrived.
  for (int i = 0; i < 20; ++i) {
    Types::RobotState state;
    state.wheelLeft.cmdVelocity = 0.0f;
    state.wheelLeft.position = 0.0f;
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

  // Phase 3: re-command the ORIGINAL speed. 133-002 DELIBERATELY INVERTED
  // 131-002's contract here. There is no accumulator to freeze; there is a
  // position reference, and a reference held across a stop would make
  // Stage B a servo that drives a stopped wheel back onto a position it
  // was told to abandon -- the runaway class 133-001 exists to prevent.
  // So: the reference re-anchors, the debt is dropped, and the first cycle
  // back reports the p term alone.
  auto resume = [&]() {
    Types::RobotState state;
    state.wheelLeft.cmdVelocity = cmd;
    state.wheelLeft.position = 0.0f;
    state.wheelLeft.velocity = 0.0f;
    state.wheelLeft.connected = true;
    state.wheelLeft.sampleTime = now - 10;
    state.time.cycleStart = now;
    state.time.cyclePeriod = 50000;
    drive.tick(state);
    now += 50;
  };
  resume();
  checkFloatEq(drive.pidLeft(), gains.kp * cmd,
              "resumed motion: the first cycle back reports the p term alone -- the pre-stop "
              "position debt was DISCARDED with the reference, never repaid after the stop");
  resume();
  checkTrue(drive.pidLeft() > gains.kp * cmd,
           "and the reference then starts accumulating again from the new anchor (the loop is "
           "re-armed, not disabled)");
}

// 10. What Stage B does with a reading it cannot fully trust, in the
//     POSITION domain (133-002 -- this replaces 131-002's velocity-domain
//     freshness gate, which guarded a hazard that no longer reaches this
//     term):
//       (a) DISCONNECTED -- still contributes exactly nothing. The
//           position a disconnected wheel reports is MANUFACTURED
//           (collectEncoder() hands back lastGoodRawEnc_), so correcting
//           against it would bank a fictitious deficit.
//       (b) STALE BY AGE, connected -- now DOES correct. A stale register
//           holds the last TRUE position; the travel it has not yet
//           reported is not lost, it lands on the next good read (scenario
//           7's own subject). Freezing here is what used to delete real
//           distance permanently.
//       (c) FROZEN (Health::wheelFrozenLeft) -- likewise corrects, and is
//           BOUNDED: posErrMax caps how far behind a wheel may be counted
//           as being, so a wedged encoder or a physically blocked wheel
//           cannot bank unbounded catch-up debt and then sprint to repay
//           it. Bounded authority, not frozen authority, is the new
//           contract; the bound is asserted here explicitly.
void scenarioDisconnectedContributesNothingStaleAndFrozenCorrectBounded() {
  beginScenario("Stage B (133-002): a DISCONNECTED wheel contributes exactly nothing, while a "
                "merely stale or frozen reading corrects -- bounded by posErrMax, not frozen");

  App::Drive::ControlGains gains;
  gains.kp = 0.05f;
  gains.ki = 0.5f;
  gains.iMax = 50.0f;
  gains.kaff = 0.0f;
  gains.pidMax = 40.0f;

  App::Drive::AdaptationBounds bounds;
  bounds.aSteady = 50.0f;   // [mm/s^2]
  bounds.posErrMax = 4.0f;  // [mm] -- the bound under test in (b)/(c)

  const float cmd = 200.0f;  // [mm/s]
  const float kPOnly = gains.kp * cmd;                          // 10.0 -- I term contributes 0
  const float kBounded = kPOnly + gains.ki * bounds.posErrMax;  // 12.0 -- I term pinned at its clamp

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
      state.wheelLeft.position = 0.0f;   // manufactured on a failed collect, not measured
      state.wheelLeft.velocity = 0.0f;
      state.wheelLeft.connected = false;  // disconnected
      state.wheelLeft.sampleTime = now - 10;
      state.time.cycleStart = now;
      state.time.cyclePeriod = 50000;
      drive.tick(state);
      checkFloatEq(drive.pidLeft(), kPOnly,
                  "disconnected: pid is p-term only on EVERY tick -- Stage B never corrects "
                  "against a manufactured position, and never latches armed while disconnected");
      now += 50;
    }
  }

  // (b) stale sample age -- connected, but the age implied by sampleTime
  //     exceeds kMaxSampleAge (200ms). Stage C's adaptBias() still freezes
  //     on this; Stage B deliberately does not.
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
      state.wheelLeft.position = 0.0f;
      state.wheelLeft.velocity = 0.0f;
      state.wheelLeft.connected = true;
      state.wheelLeft.sampleTime = 0;  // ancient -- age (cycleStart - 0) always >> 200ms here
      state.time.cycleStart = now;
      state.time.cyclePeriod = 50000;
      drive.tick(state);
      checkTrue(drive.pidLeft() <= kBounded + 1e-3f,
               "stale sample age: the correction never exceeds p + ki*posErrMax -- a stale read "
               "is allowed to correct, but only up to the configured position bound");
      now += 50;
    }
    checkFloatEq(drive.pidLeft(), kBounded,
                "and it does reach that bound: a stale reading is NOT frozen out, because the "
                "register holds the last TRUE position rather than a manufactured one");
  }

  // (c) frozen (Health::wheelFrozenLeft) -- a wedged encoder, or a wheel
  //     physically blocked. Same contract as (b): bounded, not frozen.
  {
    MockMotor left;
    MockMotor right;
    App::Drive drive(left, right, 200.0f);
    drive.setDutyPerSpeed(1.0f, 1.0f);
    drive.setControlGains(gains);
    drive.setAdaptationBounds(bounds);

    uint32_t now = 1000;
    for (int i = 0; i < 200; ++i) {
      Types::RobotState state;
      state.wheelLeft.cmdVelocity = cmd;
      state.wheelLeft.position = 0.0f;        // wedged: the register never advances
      state.wheelLeft.velocity = 0.0f;
      state.wheelLeft.connected = true;
      state.wheelLeft.sampleTime = now - 10;  // fresh BY AGE ...
      state.health.wheelFrozenLeft = true;    // ... but diagnosed frozen
      state.time.cycleStart = now;
      state.time.cyclePeriod = 50000;
      drive.tick(state);
      checkTrue(drive.pidLeft() <= kBounded + 1e-3f,
               "frozen wheel: the correction is capped at p + ki*posErrMax for as long as the "
               "register stays wedged -- unbounded catch-up debt is what posErrMax exists to stop");
      now += 50;
    }
    checkFloatEq(drive.pidLeft(), kBounded,
                "sustained over 200 cycles it sits exactly ON the bound and never climbs past it");
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

  // 133-001 shifted the ledger by exactly one tick, and this scenario's
  // arithmetic has to say so out loud. The FIRST of the kConsumed ticks
  // above is itself a commanded nonzero->zero transition of the duty pair
  // (writtenLeft_/Right_ were still the 0.5 pair from the setup tick, so
  // alreadyQuiet is false on it), which now RE-ARMS the window to a full
  // kStopEnforceTicks after its own decrement. Only the remaining
  // (kConsumed - 1) ticks are net consumption. What this scenario actually
  // asserts is unchanged: takeover() neither reset the countdown (0 more
  // ticks) nor re-armed it (kStopEnforceTicks more) -- it left whatever was
  // there alone.
  const int kRemaining = kStopEnforceTicks - (kConsumed - 1);
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
// 10b. 133-001 (the 2026-08-03 runaway): the stop re-assertion window arms
//    on the COMMANDED nonzero->zero transition of the duty pair, while the
//    encoder reports at rest -- the exact condition that disarmed BOTH
//    prior defences. Pre-133 the window was armed only by estop(), and its
//    other half (`wheelsMoving`) reads the encoder, so a wheel reporting
//    at rest left a plain commanded stop with exactly ONE write. The brick
//    latches its last speed, so losing that one write is permanent.
//
//    MockMotor's velocity is pinned at 0 throughout -- if this scenario
//    ever passes because a wheel was measurably moving, it is testing the
//    OTHER half and proves nothing about this one.
// ===========================================================================

void scenarioCommandedStopArmsReassertionWithEncoderAtRest() {
  beginScenario("133-001: a commanded nonzero->zero duty transition arms the stop-reassertion "
                "window even with the encoder reading exactly at rest (no estop() involved)");

  MockMotor left;
  MockMotor right;
  const float trackWidth = 200.0f;  // [mm]
  App::Drive drive(left, right, trackWidth);
  drive.setDutyPerSpeed(1.0f, 1.0f);  // identity calibration: target IS duty

  // At rest for the WHOLE scenario -- the encoder half of enforceStop
  // (wheelsMoving) is false on every tick below, by construction.
  left.setMockVelocity(0.0f);
  right.setMockVelocity(0.0f);

  drive.tick(wheelCmd(0.5f, 0.5f));  // a real nonzero write -> writtenLeft_/Right_ nonzero
  checkTrue(left.setDutyCalls == 1 && right.setDutyCalls == 1,
            "setup: the nonzero pair actually reached both leaves");

  // The transition tick: commanded zero, nothing else changed, no estop().
  drive.tick(wheelCmd(0.0f, 0.0f));
  checkTrue(left.setDutyCalls == 2 && right.setDutyCalls == 2,
            "setup: the first commanded zero is written once (true before 133-001 too)");
  checkTrue(left.lastDutyCmd == 0.0f && right.lastDutyCmd == 0.0f,
            "setup: that write really was a zero duty");

  // THE REGRESSION. Pre-133-001 this loop produced ZERO further writes:
  // alreadyQuiet held, stopEnforceCountdown_ was never armed (no estop()),
  // and wheelsMoving was false -- so a stop whose single write was dropped
  // on the wire was never re-issued, and the brick kept its latched speed
  // with no witness. Post-133-001 the transition tick above armed the
  // window, so the next kStopEnforceTicks ticks each re-issue setDuty(0).
  constexpr int kStopEnforceTicks = 30;  // drive.h's own private constant -- duplicated per this
                                          // harness's established fixture convention.
  const int callsBeforeWindow = left.setDutyCalls;
  for (int i = 0; i < kStopEnforceTicks; ++i) drive.tick(wheelCmd(0.0f, 0.0f));
  checkTrue(left.setDutyCalls == callsBeforeWindow + kStopEnforceTicks,
            "every tick inside the armed window re-issues setDuty(0) on the left leaf, with the "
            "encoder at rest throughout");
  checkTrue(right.setDutyCalls == callsBeforeWindow + kStopEnforceTicks, "same for the right leaf");
  checkTrue(left.lastDutyCmd == 0.0f && right.lastDutyCmd == 0.0f,
            "every re-issued write is a ZERO -- the window adds writes, it never invents a duty");

  // The window is bounded, not a permanent re-write loop: once it elapses
  // and the command is still an unchanged zero, the quiet-at-zero shortcut
  // resumes (bus budget -- drive.h's own kStopEnforceTicks comment).
  const int callsAfterWindow = left.setDutyCalls;
  for (int i = 0; i < 10; ++i) drive.tick(wheelCmd(0.0f, 0.0f));
  checkTrue(left.setDutyCalls == callsAfterWindow,
            "past the window, an unchanged commanded zero stops being re-issued -- the arm is "
            "one-shot per transition, not a standing re-write");
  checkTrue(right.setDutyCalls == callsAfterWindow, "same for the right leaf");

  // ... and it re-arms on the NEXT transition, not only the first one.
  drive.tick(wheelCmd(0.5f, 0.5f));  // motion again
  const int callsBeforeSecondStop = left.setDutyCalls;
  drive.tick(wheelCmd(0.0f, 0.0f));  // second nonzero->zero transition
  for (int i = 0; i < kStopEnforceTicks; ++i) drive.tick(wheelCmd(0.0f, 0.0f));
  checkTrue(left.setDutyCalls == callsBeforeSecondStop + 1 + kStopEnforceTicks,
            "a SECOND commanded stop re-arms the window the same way the first did");
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
//
// 134-002: every scenario in this block now calls armTeleop() (above)
// before its tick(), because the floor engages only while Drive owns
// motion. That is not scaffolding to keep old tests green -- it is what
// these scenarios always meant. Each one measures the floor's ARITHMETIC,
// which only ever ran on the teleop path's behalf; the ownership gate
// itself is scenario 18's subject, and it pins the planner-owned side
// these five deliberately do not cover.
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

  armTeleop(drive, cmdLeft, cmdRight);  // 134-002: the floor only runs for its owner
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
  armTeleop(drive2, cruiseCommon - differential, cruiseCommon + differential);
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
  armTeleop(drive, -lambda, lambda);      // 134-002: the floor only runs for its owner
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
  armTeleop(drive, rawLeft, rawRight);  // 134-002: the floor only runs for its owner
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
    armTeleop(drive, -differential, differential);  // 134-002: floor runs for its owner only
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
    armTeleop(drive, subFloorCommon, subFloorCommon);  // 134-002: floor runs for its owner only
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

    armTeleop(drive, rawLeft, rawRight);  // 134-002: the floor only runs for its owner
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

    // Owned, deliberately: a teleop wheels(0, 0) IS how a standing command
    // stops, and it is the case where the floor is live and must still not
    // manufacture motion. The un-owned stop is trivially covered too --
    // the gate returns before the scale is even computed.
    armTeleop(drive, 0.0f, 0.0f);
    drive.tick(wheelCmd(0.0f, 0.0f));

    checkFloatEq(left.lastDutyCmd, 0.0f,
                "genuine full stop: left wheel writes EXACTLY 0.0 duty, unaffected by the "
                "ratio-preserving scale");
    checkFloatEq(right.lastDutyCmd, 0.0f,
                "genuine full stop: right wheel writes EXACTLY 0.0 duty");
  }
}

// 18. NEW, 134-002 (SUC-003, sprint 134 Design Rationale D2): the speed
// floor is a TELEOP AFFORDANCE and runs only while Drive owns motion. The
// SAME sub-vMin wheel pair is ticked through twice -- once planner-owned,
// once teleop-owned -- and must come out differently: untouched for the
// planner, boosted to vMin (ratio preserved) for teleop. Both halves matter
// and neither is decorative. Half (a) is the fix: a deliberately-dying
// profile tail and (134-003) a terminal alignment nudge are shaped small on
// purpose, and boosting them to 20 mm/s on `tovez` makes terminal authority
// undeliverable -- two independent bench sessions hit this on 2026-08-04
// (docs/bench-reports/motion-planning-lab-2026-08-04.md Section 5.3; the
// companion turn-tuning session had to push `DBG vmin 0` to measure a turn
// at all). Half (b) is the regression guard, and is the one that matters
// most on review: the whole risk of this change is silently disabling the
// floor for EVERYONE, and a sub-breakaway standing teleop command that is
// no longer boosted stalls and buzzes (sprint 114's dead-zone defect, one
// layer down in the duty domain).
void scenarioSpeedFloorIsATeleopAffordanceNotAGlobalOne() {
  beginScenario("134-002: the speed floor is a TELEOP affordance -- the same sub-vMin pair "
                "passes through UNMODIFIED while the planner owns motion and is still boosted "
                "to vMin while a WHEELS command owns it");

  App::Drive::AdaptationBounds bounds;
  bounds.vMin = 100.0f;  // [mm/s] test-local floor -- NOT tovez.json's real 20.0

  // A planner-shaped decelerating tail: a symmetric pivot's ramp-out, well
  // below the floor. This is the exact pair scenario 14 above proves gets
  // boosted for teleop -- the ONLY difference here is who owns motion.
  const float tailLeft = -12.0f;   // [mm/s]
  const float tailRight = 12.0f;   // [mm/s]

  // (a) Planner owns motion. handleMove() calls takeover() before every
  // planner Move (robot_loop.cpp), and takeover() clears commandActive_ --
  // so takeover() here is the real ownership transition, not a test poke at
  // some private flag.
  {
    MockMotor left;
    MockMotor right;
    App::Drive drive(left, right, 200.0f);
    drive.setDutyPerSpeed(1.0f, 1.0f);
    drive.setAdaptationBounds(bounds);

    armTeleop(drive, tailLeft, tailRight);  // teleop first, so takeover() has something to take
    checkTrue(drive.owns(), "setup: an armed WHEELS command means Drive owns motion");
    drive.takeover();  // a planner Move arrives
    checkTrue(!drive.owns(), "setup: takeover() hands motion to the planner");

    drive.tick(wheelCmd(tailLeft, tailRight));

    checkFloatEq(left.lastDutyCmd, tailLeft,
                "planner-owned sub-vMin tail reaches actuation UNMODIFIED -- not boosted to "
                "-vMin, which is what made terminal authority undeliverable");
    checkFloatEq(right.lastDutyCmd, tailRight,
                "planner-owned sub-vMin tail, right wheel: same, unmodified");
  }

  // (b) Teleop owns motion -- the 131-003 behaviour must be intact. THIS is
  // the regression guard: the floor's own justification (a standing command
  // below breakaway stalls and buzzes instead of crawling) is untouched by
  // this ticket.
  {
    MockMotor left;
    MockMotor right;
    App::Drive drive(left, right, 200.0f);
    drive.setDutyPerSpeed(1.0f, 1.0f);
    drive.setAdaptationBounds(bounds);

    armTeleop(drive, tailLeft, tailRight);
    drive.tick(wheelCmd(tailLeft, tailRight));

    checkFloatEq(left.lastDutyCmd, -bounds.vMin,
                "teleop-owned sub-vMin command is STILL boosted to exactly -vMin -- 131-003's "
                "behaviour is intact for the path it was written for");
    checkFloatEq(right.lastDutyCmd, bounds.vMin,
                "teleop-owned sub-vMin command, right wheel: still boosted to exactly +vMin");
    checkFloatEq(right.lastDutyCmd, -left.lastDutyCmd,
                "and the boost is still ratio-preserving (equal and opposite in, equal and "
                "opposite out)");
  }

  // (c) An asymmetric teleop pair, to prove the ratio-preserving property
  // itself survived the gate rather than only the symmetric special case.
  {
    MockMotor left;
    MockMotor right;
    App::Drive drive(left, right, 200.0f);
    drive.setDutyPerSpeed(1.0f, 1.0f);
    drive.setAdaptationBounds(bounds);

    const float rawLeft = 20.0f;   // [mm/s]
    const float rawRight = 80.0f;  // [mm/s] dominant, still below vMin
    armTeleop(drive, rawLeft, rawRight);
    drive.tick(wheelCmd(rawLeft, rawRight));

    checkFloatEq(left.lastDutyCmd, 25.0f,
                "teleop asymmetric pair: scaled by vMin/dominantMag (x1.25), ratio preserved");
    checkFloatEq(right.lastDutyCmd, bounds.vMin,
                "teleop asymmetric pair: the dominant wheel still lands at exactly vMin");
  }

  // (d) "Stop is stop" under the gate, both ownerships: a genuine full stop
  // writes exactly (0.0, 0.0) whether or not the floor is allowed to run.
  {
    MockMotor leftOwned;
    MockMotor rightOwned;
    App::Drive driveOwned(leftOwned, rightOwned, 200.0f);
    driveOwned.setDutyPerSpeed(1.0f, 1.0f);
    driveOwned.setAdaptationBounds(bounds);
    armTeleop(driveOwned, 0.0f, 0.0f);
    driveOwned.tick(wheelCmd(0.0f, 0.0f));
    checkFloatEq(leftOwned.lastDutyCmd, 0.0f, "teleop-owned full stop: exactly 0.0 duty left");
    checkFloatEq(rightOwned.lastDutyCmd, 0.0f, "teleop-owned full stop: exactly 0.0 duty right");

    MockMotor leftPlanner;
    MockMotor rightPlanner;
    App::Drive drivePlanner(leftPlanner, rightPlanner, 200.0f);
    drivePlanner.setDutyPerSpeed(1.0f, 1.0f);
    drivePlanner.setAdaptationBounds(bounds);
    drivePlanner.tick(wheelCmd(0.0f, 0.0f));
    checkFloatEq(leftPlanner.lastDutyCmd, 0.0f, "planner-owned full stop: exactly 0.0 duty left");
    checkFloatEq(rightPlanner.lastDutyCmd, 0.0f,
                "planner-owned full stop: exactly 0.0 duty right");
  }

  // (e) The deadman-expiry edge, decided and documented in
  // applySpeedFloor()'s own doc comment: an EXPIRING command is not
  // floored. update() clears commandActive_ the moment the deadline passes,
  // and tick() runs BEFORE update() within a cycle -- so the floor covers
  // every cycle the command was live and no cycle after it. A command on
  // its way out is allowed to reach zero rather than being boosted back up
  // to vMin. The pair ticked here is deliberately the STALE nonzero one
  // (update() itself zeroes the targets at expiry, so on the real loop this
  // arm is belt-and-braces -- but the ordering is pinned rather than
  // assumed).
  {
    MockMotor left;
    MockMotor right;
    App::Drive drive(left, right, 200.0f);
    drive.setDutyPerSpeed(1.0f, 1.0f);
    drive.setAdaptationBounds(bounds);

    drive.command(tailLeft, tailRight, /*duration=*/100.0f, /*moveId=*/9, /*now=*/1000);
    checkTrue(drive.owns(), "setup: the armed command owns motion before its deadline");

    Types::RobotState state = wheelCmd(tailLeft, tailRight);
    drive.update(state, /*now=*/1200);  // past the deadline: the deadman expires
    checkTrue(!drive.owns(), "an expired WHEELS command no longer owns motion");
    checkFloatEq(state.wheelLeft.cmdVelocity, 0.0f,
                "expiry publishes a zeroed left target, as it always did");
    checkFloatEq(state.wheelRight.cmdVelocity, 0.0f,
                "expiry publishes a zeroed right target, as it always did");

    drive.tick(wheelCmd(tailLeft, tailRight));
    checkFloatEq(left.lastDutyCmd, tailLeft,
                "after expiry the floor is OFF: a still-nonzero command on its way out reaches "
                "zero instead of being boosted back up to vMin");
    checkFloatEq(right.lastDutyCmd, tailRight, "after expiry, right wheel: same");
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
  scenarioUpdatePublishesCmdAccelOnTheWheelsPathInertAtZeroKaff();
  scenarioStageBITermIsAPositionTermWithNoSteadyGate();
  scenarioPositionErrorGuardsReAnchorWithoutCorrecting();
  scenarioDroppedSampleDoesNotDeleteDistanceFromThePositionEstimate();
  scenarioCommandedZeroForcesStageBToZeroAndDiscardsTheReference();
  scenarioDisconnectedContributesNothingStaleAndFrozenCorrectBounded();
  scenarioWheelsAndMoveReachIdenticalDriveBehavior();
  scenarioTakeoverPreservesLearnedStateEstopFullyResets();
  scenarioTakeoverDoesNotTouchStopReassertionCountdown();
  scenarioCommandedStopArmsReassertionWithEncoderAtRest();
  scenarioReversalAfterConvergedForwardBiasDoesNotReduceMagnitude();
  scenarioBiasPersistsAcrossChainedTakeoverBoundaries();
  scenarioDifferentialTrimAtFloorBoundaryStaysProportional();
  scenarioSymmetricPivotBelowFloorBoostsToExactVMin();
  scenarioAsymmetricPairBelowFloorPreservesRatio();
  scenarioDifferentialWithNearZeroCommonModeNowBoostedTowardVMin();
  scenarioRawZeroWheelStaysZeroUnderRatioPreservingScale();
  scenarioSpeedFloorIsATeleopAffordanceNotAGlobalOne();
  scenarioStallLatchesOnCommandedMotionThatIsNotHappening();
  scenarioWheelsSlippingAgainstAWallStillStalls();
  scenarioWedgedEncoderWithRobotMovingIsNotAStall();
  scenarioPivotIsNotAStall();
  scenarioNoOtosFallsBackToEncoders();
  scenarioStandingSubDemandCommandIsNotAStall();
  scenarioMovingRobotIsNotAStall();
  scenarioZeroWindowDisablesTheDetector();
  scenarioEstopClearsTheStallLatch();

  if (g_failureCount == 0) {
    std::printf("OK: all App::Drive scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the App::Drive scenarios\n", g_failureCount);
  return 1;
}
