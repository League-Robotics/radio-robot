// app_move_queue_harness.cpp -- off-hardware acceptance harness for ticket
// 116-005 (SUC-050/SUC-051/SUC-052), App::MoveQueue
// (src/firm/app/move_queue.{h,cpp}). Proves the queue-lifecycle contract
// move_queue.h's own file header documents: enqueue()/replace()/flush()/
// ERR_FULL bookkeeping, activation staging a Move's velocity through Drive
// and capturing a Motion::StopCondition baseline from REAL App::Odometry
// readings, tick()'s Continue/StopConditionMet/TimedOut handling (including
// the SAME-CALL chain hand-off to the next pending Move), and the
// flushed-while-pending no-completion-ack convention (sprint.md
// Architecture Open Question 2).
//
// Combines app_drive_harness.cpp's Drive-staging verification (real
// Devices::NezhaMotor leaves via TestSim::SimPlant/ScriptedI2CHook, kp=ki=0
// so appliedDuty() == kff*target exactly) with app_odometry_harness.cpp's
// real-encoder-driven Odometry progression (straight travel for DISTANCE,
// pure rotation for ANGLE) and app_deadman_harness.cpp's/
// motion_stop_condition_harness.cpp's TestSim::SimClock stepping (for TIME/
// timeout). Every scripting helper below is duplicated from those three
// harnesses per this codebase's established per-harness-file fixture
// convention (see app_drive_harness.cpp's own header note).
//
// Scenarios 13-14 (decel-into-the-goal campaign) additionally prove
// App::MoveQueue's ShaperLimits/Motion::VelocityShaper integration:
// scenarios 1-12 above are UNCHANGED and continue to construct MoveQueue
// with the default ShaperLimits{} (shaping OFF) -- they are this file's
// own regression guard that shaping is truly opt-in/byte-identical when
// disabled, not just documented as such.
//
// Compiled by test_app_move_queue.py with -DHOST_BUILD against
// move_queue.cpp, stop_condition.cpp, velocity_shaper.cpp, drive.cpp,
// odometry.cpp, nezha_motor.cpp, velocity_pid.cpp, sim_plant.cpp,
// {wheel,otos}_plant.cpp, body_kinematics.cpp, sim_clock.cpp.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include "app/drive.h"
#include "app/move_queue.h"
#include "app/odometry.h"
#include "app/state_estimator.h"
#include "devices/device_config.h"
#include "devices/device_types.h"
#include "devices/nezha_motor.h"
#include "kinematics/body_kinematics.h"
#include "messages/envelope.h"
#include "scripted_i2c_hook.h"
#include "sim_clock.h"
#include "sim_plant.h"

namespace {

// --- Hand-rolled assertion plumbing (see app_drive_harness.cpp) ---------

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

void checkFloatEq(float actual, float expected, const std::string& what, float tol = 1e-3f) {
  if (std::fabs(actual - expected) > tol) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %g, got %g", what.c_str(),
                  static_cast<double>(expected), static_cast<double>(actual));
    fail(buf);
  }
}

// checkLe -- decel-into-the-goal campaign scenarios (13-14 below) only.
void checkLe(float actual, float bound, const std::string& what) {
  if (actual > bound) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected <= %g, got %g", what.c_str(),
                  static_cast<double>(bound), static_cast<double>(actual));
    fail(buf);
  }
}

// --- Devices::NezhaMotor scripting helpers (duplicated from
// app_drive_harness.cpp / app_odometry_harness.cpp) ----------------------

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

// See app_drive_harness.cpp's own comment: writeRawDuty()'s write-rate
// limiter throttles any non-stop write to at most one per 40000us since the
// leaf's last actual bus write -- every scenario below spaces its
// duty-observing cycles at least this far apart.
constexpr uint64_t kPastWriteThrottleUs = 50000;

Devices::MotorConfig baseNezhaConfig(uint32_t port) {
  Devices::MotorConfig cfg;
  cfg.port = port;
  cfg.fwdSign = 1;
  cfg.wheelTravelCalib = 1.0f;
  cfg.velFiltAlpha = 1.0f;
  // kp=0, ki=0 isolates appliedDuty() to a single deterministic linear
  // relation (rawDuty == kff * target) -- see app_drive_harness.cpp's own
  // header comment.
  cfg.velGains = Devices::Gains{/*kp=*/0.0f, /*ki=*/0.0f, /*kff=*/0.002f,
                                 /*iMax=*/1.0f, /*kaw=*/2.0f};
  cfg.velDeadband = 0.0f;
  return cfg;
}

void primeAtZero(Devices::NezhaMotor& motor, TestSim::ScriptedI2CHook& bus, uint16_t wireAddr) {
  scriptEncoderRequestCollect(bus, wireAddr, 0.0f);
  motor.requestSample();
  motor.tick(0);
}

void runOneCycleAtZeroPosition(Devices::NezhaMotor& motor, TestSim::ScriptedI2CHook& bus,
                                uint16_t wireAddr, uint64_t nowUs) {
  scriptEncoderRequestCollect(bus, wireAddr, 0.0f);
  motor.requestSample();
  motor.tick(nowUs);
}

// driveToPosition -- same shape as app_odometry_harness.cpp's own helper of
// the same name: scripts an ABSOLUTE position read (not a delta), so it
// doubles as both "advance the encoder for Odometry::integrate()" and "run
// one more duty-write cycle for appliedDuty() observation" without
// triggering NezhaMotor's own implausible-step-speed rejection (holding
// position steady between calls is always a zero-delta, always-plausible
// step).
void driveToPosition(Devices::NezhaMotor& motor, TestSim::ScriptedI2CHook& bus,
                      uint16_t wireAddr, float positionMm, uint64_t nowUs) {
  scriptEncoderRequestCollect(bus, wireAddr, positionMm);
  motor.requestSample();
  motor.tick(nowUs);
}

// --- msg::Move test fixtures ---------------------------------------------

void setStopValue(msg::Move& m, msg::Move::StopKind kind, float value) {
  m.stop_kind = kind;
  switch (kind) {
    case msg::Move::StopKind::TIME:
      m.stop.time = value;
      break;
    case msg::Move::StopKind::DISTANCE:
      m.stop.distance = value;
      break;
    case msg::Move::StopKind::ANGLE:
      m.stop.angle = value;
      break;
    case msg::Move::StopKind::NONE:
      break;
  }
}

msg::Move makeTwistMove(uint32_t id, float v_x, float v_y, float omega,
                         msg::Move::StopKind stopKind, float stopValue, float timeout,
                         bool replace) {
  msg::Move m;
  m.velocity_kind = msg::Move::VelocityKind::TWIST;
  m.velocity.twist = msg::MoveTwist{v_x, v_y, omega};
  setStopValue(m, stopKind, stopValue);
  m.timeout = timeout;
  m.replace = replace;
  m.id = id;
  return m;
}

msg::Move makeWheelsMove(uint32_t id, float vLeft, float vRight, msg::Move::StopKind stopKind,
                          float stopValue, float timeout, bool replace) {
  msg::Move m;
  m.velocity_kind = msg::Move::VelocityKind::WHEELS;
  m.velocity.wheels = msg::MoveWheels{vLeft, vRight};
  setStopValue(m, stopKind, stopValue);
  m.timeout = timeout;
  m.replace = replace;
  m.id = id;
  return m;
}

// checkMovesEqual -- field-by-field comparison (never a raw memcmp: `move`
// carries a union, whose non-active member's slack bytes are unspecified,
// so a byte-compare could spuriously fail/pass independent of the fields
// that actually matter). This is SUC-052's own "byte-for-byte, not just
// 'still 4 pending'" rigor bar, applied per meaningful field.
void checkMovesEqual(const msg::Move& a, const msg::Move& b, const std::string& what) {
  if (a.velocity_kind != b.velocity_kind) {
    fail(what + " -- velocity_kind differs");
  } else if (a.velocity_kind == msg::Move::VelocityKind::WHEELS) {
    checkFloatEq(a.velocity.wheels.v_left, b.velocity.wheels.v_left, what + " -- v_left");
    checkFloatEq(a.velocity.wheels.v_right, b.velocity.wheels.v_right, what + " -- v_right");
  } else {
    checkFloatEq(a.velocity.twist.v_x, b.velocity.twist.v_x, what + " -- v_x");
    checkFloatEq(a.velocity.twist.v_y, b.velocity.twist.v_y, what + " -- v_y");
    checkFloatEq(a.velocity.twist.omega, b.velocity.twist.omega, what + " -- omega");
  }

  if (a.stop_kind != b.stop_kind) {
    fail(what + " -- stop_kind differs");
  } else {
    switch (a.stop_kind) {
      case msg::Move::StopKind::TIME:
        checkFloatEq(a.stop.time, b.stop.time, what + " -- stop.time");
        break;
      case msg::Move::StopKind::DISTANCE:
        checkFloatEq(a.stop.distance, b.stop.distance, what + " -- stop.distance");
        break;
      case msg::Move::StopKind::ANGLE:
        checkFloatEq(a.stop.angle, b.stop.angle, what + " -- stop.angle");
        break;
      case msg::Move::StopKind::NONE:
        break;
    }
  }

  checkFloatEq(a.timeout, b.timeout, what + " -- timeout");
  checkTrue(a.replace == b.replace, what + " -- replace");
  checkUintEq(a.id, b.id, what + " -- id");
}

constexpr float kTrackWidth = 200.0f;  // [mm]
constexpr uint16_t kWireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);
constexpr float kKff = 0.002f;

// ===========================================================================
// 1. Activation (AC #2): enqueue() on an empty queue activates immediately,
//    stages a TWIST Move's velocity through Drive::setTwist() (verified via
//    the REAL leaves' appliedDuty(), same technique as
//    app_drive_harness.cpp), and reports ERR_NONE with the corrId echoed.
// ===========================================================================

void scenarioEnqueueOnEmptyQueueActivatesTwistImmediately() {
  beginScenario("MoveQueue::enqueue(): empty queue -- TWIST Move activates immediately via Drive::setTwist()");

  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  Devices::NezhaMotor left(plant, baseNezhaConfig(1));
  Devices::NezhaMotor right(plant, baseNezhaConfig(2));
  primeAtZero(left, bus, kWireAddr);
  primeAtZero(right, bus, kWireAddr);

  TestSim::SimClock clock;
  App::Drive drive(left, right, kTrackWidth);
  App::Odometry odom(left, right, kTrackWidth);
  // Turn-prediction campaign: MoveQueue now holds a const StateEstimator&
  // (bodyAt()-driven stop-condition anticipation) -- default-constructed
  // (body peer stays valid=false, never updated by this harness) and
  // stopLead left at its own constructor default (0, anticipation OFF),
  // so every scenario below is unaffected: the anticipation branch in
  // tick() is gated on stopLead_ > 0, matching this file's own pre-
  // existing raw-Odometry-baseline assertions unchanged.
  App::StateEstimator estimator;
  App::MoveQueue queue(drive, odom, clock, estimator);

  msg::Move move = makeTwistMove(/*id=*/7, /*v_x=*/100.0f, /*v_y=*/0.0f, /*omega=*/0.0f,
                                  msg::Move::StopKind::TIME, /*stopValue=*/500.0f,
                                  /*timeout=*/5000.0f, /*replace=*/false);

  App::MoveQueue::EnqueueResult result = queue.enqueue(move, /*corrId=*/42);
  checkUintEq(result.corrId, 42, "EnqueueResult echoes corrId unchanged");
  checkTrue(result.err == msg::ErrCode::ERR_NONE, "activation on an empty queue is ERR_NONE, never ERR_FULL");
  checkTrue(queue.active(), "queue is active immediately after enqueue() on an empty queue");
  checkUintEq(queue.activeMoveId(), 7, "activeMoveId() reflects the just-activated Move's id");

  drive.tick();
  runOneCycleAtZeroPosition(left, bus, kWireAddr, kPastWriteThrottleUs);
  runOneCycleAtZeroPosition(right, bus, kWireAddr, kPastWriteThrottleUs);

  float expectedVL = 0.0f, expectedVR = 0.0f;
  BodyKinematics::inverse(100.0f, 0.0f, kTrackWidth, expectedVL, expectedVR);
  checkFloatEq(left.appliedDuty(), kKff * expectedVL, "left duty reflects the TWIST-staged target via inverse()");
  checkFloatEq(right.appliedDuty(), kKff * expectedVR, "right duty reflects the TWIST-staged target via inverse()");
}

// ===========================================================================
// 2. Activation stages a WHEELS Move raw (AC #2) and a DISTANCE stop
//    condition ends using REAL App::Odometry::pathLength() progression
//    (straight-line travel, both wheels advancing equally) -- proves
//    MoveQueue correctly captured the activation-time baseline and reads
//    live odometry at each tick(), not just that StopCondition's own math
//    is right (already proven by motion_stop_condition_harness.cpp).
// ===========================================================================

void scenarioWheelsDistanceMoveUsesRealOdometryBaseline() {
  beginScenario("MoveQueue: WHEELS+DISTANCE Move stages raw wheel targets, ends via REAL Odometry::pathLength()");

  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  Devices::NezhaMotor left(plant, baseNezhaConfig(1));
  Devices::NezhaMotor right(plant, baseNezhaConfig(2));
  primeAtZero(left, bus, kWireAddr);
  primeAtZero(right, bus, kWireAddr);

  TestSim::SimClock clock;
  App::Drive drive(left, right, kTrackWidth);
  App::Odometry odom(left, right, kTrackWidth);
  // Turn-prediction campaign: MoveQueue now holds a const StateEstimator&
  // (bodyAt()-driven stop-condition anticipation) -- default-constructed
  // (body peer stays valid=false, never updated by this harness) and
  // stopLead left at its own constructor default (0, anticipation OFF),
  // so every scenario below is unaffected: the anticipation branch in
  // tick() is gated on stopLead_ > 0, matching this file's own pre-
  // existing raw-Odometry-baseline assertions unchanged.
  App::StateEstimator estimator;
  App::MoveQueue queue(drive, odom, clock, estimator);

  msg::Move move = makeWheelsMove(/*id=*/9, /*vLeft=*/90.0f, /*vRight=*/30.0f,
                                   msg::Move::StopKind::DISTANCE, /*stopValue=*/80.0f /*[mm]*/,
                                   /*timeout=*/60000.0f, /*replace=*/false);
  App::MoveQueue::EnqueueResult enqRes = queue.enqueue(move, /*corrId=*/1);
  checkTrue(enqRes.err == msg::ErrCode::ERR_NONE, "enqueue on empty queue is ERR_NONE");

  drive.tick();
  runOneCycleAtZeroPosition(left, bus, kWireAddr, kPastWriteThrottleUs);
  runOneCycleAtZeroPosition(right, bus, kWireAddr, kPastWriteThrottleUs);
  checkFloatEq(left.appliedDuty(), kKff * 90.0f, "left duty reflects the RAW staged v_left -- no inverse() involved");
  checkFloatEq(right.appliedDuty(), kKff * 30.0f, "right duty reflects the RAW staged v_right -- no inverse() involved");

  // Advance both wheels forward 50mm (equal deltas -- straight travel,
  // distance == 50mm) -- under the 80mm threshold, motion should CONTINUE.
  driveToPosition(left, bus, kWireAddr, 50.0f, 200000);
  driveToPosition(right, bus, kWireAddr, 50.0f, 200000);
  odom.integrate();
  checkFloatEq(odom.pathLength(), 50.0f, "sanity: pathLength() reflects the 50mm straight-line travel");

  App::MoveQueue::TickResult tick1 = queue.tick(clock.nowMicros(), odom);
  checkFalse(tick1.completed, "DISTANCE Move continues at 50mm traveled -- under the 80mm threshold");
  checkTrue(queue.active(), "queue is still active after a Continue tick");

  // Advance a further 40mm (total 90mm traveled) -- over the 80mm threshold.
  driveToPosition(left, bus, kWireAddr, 90.0f, 260000);
  driveToPosition(right, bus, kWireAddr, 90.0f, 260000);
  odom.integrate();

  App::MoveQueue::TickResult tick2 = queue.tick(clock.nowMicros(), odom);
  checkTrue(tick2.completed, "DISTANCE Move ends once 90mm >= the 80mm threshold");
  checkUintEq(tick2.completion.moveId, 9, "completion reports the ended Move's id");
  checkFalse(tick2.completion.timedOut, "ended via the DISTANCE condition, not the timeout backstop");
  checkFalse(queue.active(), "queue drains empty -- nothing was pending");

  // Empty-queue drain calls Drive::stop() -- verify both leaves reach 0.
  drive.tick();
  driveToPosition(left, bus, kWireAddr, 90.0f, 320000);
  driveToPosition(right, bus, kWireAddr, 90.0f, 320000);
  checkFloatEq(left.appliedDuty(), 0.0f, "left duty reaches 0 -- empty-queue drain called Drive::stop()");
  checkFloatEq(right.appliedDuty(), 0.0f, "right duty reaches 0 -- empty-queue drain called Drive::stop()");
}

// ===========================================================================
// 3. ANGLE stop condition ends using REAL Odometry::theta() progression
//    (pure rotation -- equal-and-opposite wheel deltas, zero pathLength()
//    contribution, matching app_odometry_harness.cpp's own pure-rotation
//    convention).
// ===========================================================================

void scenarioAngleMoveUsesRealOdometryHeadingBaseline() {
  beginScenario("MoveQueue: TWIST+ANGLE Move ends via REAL Odometry::theta() progression (pure rotation)");

  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  Devices::NezhaMotor left(plant, baseNezhaConfig(1));
  Devices::NezhaMotor right(plant, baseNezhaConfig(2));
  primeAtZero(left, bus, kWireAddr);
  primeAtZero(right, bus, kWireAddr);

  TestSim::SimClock clock;
  App::Drive drive(left, right, kTrackWidth);
  App::Odometry odom(left, right, kTrackWidth);
  // Turn-prediction campaign: MoveQueue now holds a const StateEstimator&
  // (bodyAt()-driven stop-condition anticipation) -- default-constructed
  // (body peer stays valid=false, never updated by this harness) and
  // stopLead left at its own constructor default (0, anticipation OFF),
  // so every scenario below is unaffected: the anticipation branch in
  // tick() is gated on stopLead_ > 0, matching this file's own pre-
  // existing raw-Odometry-baseline assertions unchanged.
  App::StateEstimator estimator;
  App::MoveQueue queue(drive, odom, clock, estimator);

  msg::Move move = makeTwistMove(/*id=*/11, /*v_x=*/0.0f, /*v_y=*/0.0f, /*omega=*/0.5f,
                                  msg::Move::StopKind::ANGLE, /*stopValue=*/0.5f /*[rad]*/,
                                  /*timeout=*/60000.0f, /*replace=*/false);
  queue.enqueue(move, /*corrId=*/2);

  // Left goes -d, right goes +d -- headingDelta = 2d/trackWidth, distance
  // stays 0 (BodyKinematics::forward() for equal-and-opposite deltas).
  driveToPosition(left, bus, kWireAddr, -30.0f, 200000);
  driveToPosition(right, bus, kWireAddr, 30.0f, 200000);
  odom.integrate();
  checkFloatEq(odom.theta(), 0.3f, "sanity: theta() reflects the pure-rotation headingDelta", 1e-3f);
  checkFloatEq(odom.pathLength(), 0.0f, "sanity: pathLength() stays ~0 for a pure rotation");

  App::MoveQueue::TickResult tick1 = queue.tick(clock.nowMicros(), odom);
  checkFalse(tick1.completed, "ANGLE Move continues at 0.3rad turned -- under the 0.5rad threshold");

  driveToPosition(left, bus, kWireAddr, -60.0f, 260000);
  driveToPosition(right, bus, kWireAddr, 60.0f, 260000);
  odom.integrate();

  App::MoveQueue::TickResult tick2 = queue.tick(clock.nowMicros(), odom);
  checkTrue(tick2.completed, "ANGLE Move ends once 0.6rad >= the 0.5rad threshold");
  checkUintEq(tick2.completion.moveId, 11, "completion reports the ended Move's id");
  checkFalse(tick2.completion.timedOut, "ended via the ANGLE condition, not the timeout backstop");
}

// ===========================================================================
// 4. TIME stop condition: Continue before the deadline, completes AT/after
//    it (>=, not >), then an empty-queue drain stops Drive.
// ===========================================================================

void scenarioTimeMoveContinuesThenCompletesAndDrainsEmptyToStop() {
  beginScenario("MoveQueue: TIME Move Continue before the deadline, completes at/after it, empty-queue drain stops Drive");

  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  Devices::NezhaMotor left(plant, baseNezhaConfig(1));
  Devices::NezhaMotor right(plant, baseNezhaConfig(2));
  primeAtZero(left, bus, kWireAddr);
  primeAtZero(right, bus, kWireAddr);

  TestSim::SimClock clock;
  App::Drive drive(left, right, kTrackWidth);
  App::Odometry odom(left, right, kTrackWidth);
  // Turn-prediction campaign: MoveQueue now holds a const StateEstimator&
  // (bodyAt()-driven stop-condition anticipation) -- default-constructed
  // (body peer stays valid=false, never updated by this harness) and
  // stopLead left at its own constructor default (0, anticipation OFF),
  // so every scenario below is unaffected: the anticipation branch in
  // tick() is gated on stopLead_ > 0, matching this file's own pre-
  // existing raw-Odometry-baseline assertions unchanged.
  App::StateEstimator estimator;
  App::MoveQueue queue(drive, odom, clock, estimator);

  clock.setMicros(0);
  msg::Move move = makeTwistMove(/*id=*/5, /*v_x=*/120.0f, /*v_y=*/0.0f, /*omega=*/0.0f,
                                  msg::Move::StopKind::TIME, /*stopValue=*/100.0f /*[ms]*/,
                                  /*timeout=*/5000.0f, /*replace=*/false);
  queue.enqueue(move, /*corrId=*/3);

  drive.tick();
  runOneCycleAtZeroPosition(left, bus, kWireAddr, kPastWriteThrottleUs);
  runOneCycleAtZeroPosition(right, bus, kWireAddr, kPastWriteThrottleUs);
  checkTrue(left.appliedDuty() != 0.0f, "setup: left duty nonzero -- the Move is genuinely driving before completion");

  clock.setMicros(99000);  // 99ms -- under the 100ms TIME threshold
  App::MoveQueue::TickResult tick1 = queue.tick(clock.nowMicros(), odom);
  checkFalse(tick1.completed, "not completed at 99ms -- under the 100ms threshold");
  checkTrue(queue.active(), "still active at 99ms");

  clock.setMicros(100000);  // 100ms -- AT the threshold
  App::MoveQueue::TickResult tick2 = queue.tick(clock.nowMicros(), odom);
  checkTrue(tick2.completed, "completed AT the 100ms threshold (>=, not >)");
  checkUintEq(tick2.completion.moveId, 5, "completion reports the ended Move's id");
  checkFalse(tick2.completion.timedOut, "ended via the TIME condition, not the timeout backstop");
  checkFalse(queue.active(), "queue drains empty -- nothing was pending");

  drive.tick();  // flush the stop() staged by tick()'s empty-queue drain
  runOneCycleAtZeroPosition(left, bus, kWireAddr, kPastWriteThrottleUs + 200000);
  runOneCycleAtZeroPosition(right, bus, kWireAddr, kPastWriteThrottleUs + 200000);
  checkFloatEq(left.appliedDuty(), 0.0f, "left duty reaches 0 -- empty-queue drain called Drive::stop()");
  checkFloatEq(right.appliedDuty(), 0.0f, "right duty reaches 0 -- empty-queue drain called Drive::stop()");
}

// ===========================================================================
// 5. Chain hand-off (SUC-051): a pending Move B activates the SAME tick()
//    call that ends active Move A -- no intervening zero/stopped cycle.
// ===========================================================================

void scenarioChainedMoveActivatesSameCycleNoInterveningStop() {
  beginScenario("MoveQueue: chained MOVE B activates the SAME tick() call A ends -- no intervening stop (SUC-051)");

  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  Devices::NezhaMotor left(plant, baseNezhaConfig(1));
  Devices::NezhaMotor right(plant, baseNezhaConfig(2));
  primeAtZero(left, bus, kWireAddr);
  primeAtZero(right, bus, kWireAddr);

  TestSim::SimClock clock;
  App::Drive drive(left, right, kTrackWidth);
  App::Odometry odom(left, right, kTrackWidth);
  // Turn-prediction campaign: MoveQueue now holds a const StateEstimator&
  // (bodyAt()-driven stop-condition anticipation) -- default-constructed
  // (body peer stays valid=false, never updated by this harness) and
  // stopLead left at its own constructor default (0, anticipation OFF),
  // so every scenario below is unaffected: the anticipation branch in
  // tick() is gated on stopLead_ > 0, matching this file's own pre-
  // existing raw-Odometry-baseline assertions unchanged.
  App::StateEstimator estimator;
  App::MoveQueue queue(drive, odom, clock, estimator);

  clock.setMicros(0);
  msg::Move moveA = makeWheelsMove(21, 60.0f, 60.0f, msg::Move::StopKind::TIME, 100.0f, 5000.0f, false);
  App::MoveQueue::EnqueueResult resA = queue.enqueue(moveA, 10);
  checkTrue(resA.err == msg::ErrCode::ERR_NONE, "A activates on the empty queue");

  msg::Move moveB = makeWheelsMove(22, -40.0f, 40.0f, msg::Move::StopKind::TIME, 200.0f, 5000.0f, false);
  App::MoveQueue::EnqueueResult resB = queue.enqueue(moveB, 11);
  checkTrue(resB.err == msg::ErrCode::ERR_NONE, "B enqueues behind A -- queue has room");
  checkUintEq(static_cast<uint32_t>(queue.pendingCount()), 1, "B is pending, 1 slot occupied");

  clock.setMicros(100000);  // A's TIME threshold
  App::MoveQueue::TickResult tick1 = queue.tick(clock.nowMicros(), odom);
  checkTrue(tick1.completed, "A ends at its 100ms threshold");
  checkUintEq(tick1.completion.moveId, 21, "completion reports A's id");
  checkTrue(queue.active(), "queue is STILL active -- B activated the SAME call (seamless hand-off)");
  checkUintEq(queue.activeMoveId(), 22, "B is now the active Move");
  checkUintEq(static_cast<uint32_t>(queue.pendingCount()), 0, "B moved out of pending into active");

  // Drive is staged with B's (raw, unequal-sign) wheel targets, not A's and
  // not zero -- proves no intervening Drive::stop() ran between A ending
  // and B activating.
  drive.tick();
  runOneCycleAtZeroPosition(left, bus, kWireAddr, kPastWriteThrottleUs);
  runOneCycleAtZeroPosition(right, bus, kWireAddr, kPastWriteThrottleUs);
  checkFloatEq(left.appliedDuty(), kKff * -40.0f, "left duty reflects B's staged v_left -- seamless hand-off, not A's or zero");
  checkFloatEq(right.appliedDuty(), kKff * 40.0f, "right duty reflects B's staged v_right");
}

// ===========================================================================
// 6. replace=true (SUC-051): flushes pending and preempts the active Move
//    immediately, same call -- the flushed pending Move never activates.
// ===========================================================================

void scenarioReplaceTruePreemptsActiveAndFlushesPending() {
  beginScenario("MoveQueue::enqueue(replace=true): flushes pending and preempts the active Move, same call");

  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  Devices::NezhaMotor left(plant, baseNezhaConfig(1));
  Devices::NezhaMotor right(plant, baseNezhaConfig(2));
  primeAtZero(left, bus, kWireAddr);
  primeAtZero(right, bus, kWireAddr);

  TestSim::SimClock clock;
  App::Drive drive(left, right, kTrackWidth);
  App::Odometry odom(left, right, kTrackWidth);
  // Turn-prediction campaign: MoveQueue now holds a const StateEstimator&
  // (bodyAt()-driven stop-condition anticipation) -- default-constructed
  // (body peer stays valid=false, never updated by this harness) and
  // stopLead left at its own constructor default (0, anticipation OFF),
  // so every scenario below is unaffected: the anticipation branch in
  // tick() is gated on stopLead_ > 0, matching this file's own pre-
  // existing raw-Odometry-baseline assertions unchanged.
  App::StateEstimator estimator;
  App::MoveQueue queue(drive, odom, clock, estimator);

  clock.setMicros(0);
  msg::Move moveA = makeWheelsMove(31, 50.0f, 50.0f, msg::Move::StopKind::TIME, 1000.0f, 5000.0f, false);
  queue.enqueue(moveA, 1);
  msg::Move moveB = makeWheelsMove(32, 20.0f, 20.0f, msg::Move::StopKind::TIME, 1000.0f, 5000.0f, false);
  queue.enqueue(moveB, 2);
  checkUintEq(static_cast<uint32_t>(queue.pendingCount()), 1, "setup: B is pending behind A");

  msg::Move moveC = makeWheelsMove(33, -70.0f, 70.0f, msg::Move::StopKind::TIME, 150.0f, 5000.0f, /*replace=*/true);
  App::MoveQueue::EnqueueResult resC = queue.enqueue(moveC, 3);
  checkTrue(resC.err == msg::ErrCode::ERR_NONE, "replace=true is always accepted, never ERR_FULL");
  checkTrue(queue.active(), "C is active immediately");
  checkUintEq(queue.activeMoveId(), 33, "C preempted A -- C is now active");
  checkUintEq(static_cast<uint32_t>(queue.pendingCount()), 0, "B was flushed -- pending is empty");

  drive.tick();
  runOneCycleAtZeroPosition(left, bus, kWireAddr, kPastWriteThrottleUs);
  runOneCycleAtZeroPosition(right, bus, kWireAddr, kPastWriteThrottleUs);
  checkFloatEq(left.appliedDuty(), kKff * -70.0f, "Drive is staged with C's targets, not A's");
  checkFloatEq(right.appliedDuty(), kKff * 70.0f, "Drive is staged with C's targets, not A's");

  // C ends -- B never appears (it was flushed, not merely deprioritized).
  clock.setMicros(150000);
  App::MoveQueue::TickResult tickC = queue.tick(clock.nowMicros(), odom);
  checkTrue(tickC.completed, "C ends at its own 150ms threshold");
  checkUintEq(tickC.completion.moveId, 33, "completion reports C's id");
  checkFalse(queue.active(), "queue drains fully empty -- B never activates (flushed, not just pending)");
}

// ===========================================================================
// 7. ERR_FULL overflow (SUC-052): a 5th pending Move is rejected and the
//    existing active + 4 pending Moves are byte-for-byte unchanged.
// ===========================================================================

void scenarioOverflowRejectedErrFullQueueByteForByteUnchanged() {
  beginScenario("MoveQueue::enqueue(): a 5th pending Move is ERR_FULL, active+4 pending are byte-for-byte unchanged");

  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  Devices::NezhaMotor left(plant, baseNezhaConfig(1));
  Devices::NezhaMotor right(plant, baseNezhaConfig(2));
  primeAtZero(left, bus, kWireAddr);
  primeAtZero(right, bus, kWireAddr);

  TestSim::SimClock clock;
  App::Drive drive(left, right, kTrackWidth);
  App::Odometry odom(left, right, kTrackWidth);
  // Turn-prediction campaign: MoveQueue now holds a const StateEstimator&
  // (bodyAt()-driven stop-condition anticipation) -- default-constructed
  // (body peer stays valid=false, never updated by this harness) and
  // stopLead left at its own constructor default (0, anticipation OFF),
  // so every scenario below is unaffected: the anticipation branch in
  // tick() is gated on stopLead_ > 0, matching this file's own pre-
  // existing raw-Odometry-baseline assertions unchanged.
  App::StateEstimator estimator;
  App::MoveQueue queue(drive, odom, clock, estimator);

  msg::Move moveA = makeTwistMove(41, 10.0f, 0.0f, 0.0f, msg::Move::StopKind::TIME, 1000.0f, 5000.0f, false);
  queue.enqueue(moveA, 100);

  msg::Move moveB = makeTwistMove(42, 20.0f, 0.0f, 0.0f, msg::Move::StopKind::DISTANCE, 30.0f, 5000.0f, false);
  msg::Move moveC = makeWheelsMove(43, 5.0f, 5.0f, msg::Move::StopKind::ANGLE, 0.4f, 5000.0f, false);
  msg::Move moveD = makeTwistMove(44, -15.0f, 0.0f, 0.2f, msg::Move::StopKind::TIME, 250.0f, 5000.0f, false);
  msg::Move moveE = makeWheelsMove(45, -8.0f, 8.0f, msg::Move::StopKind::TIME, 400.0f, 5000.0f, false);
  queue.enqueue(moveB, 101);
  queue.enqueue(moveC, 102);
  queue.enqueue(moveD, 103);
  queue.enqueue(moveE, 104);
  checkUintEq(static_cast<uint32_t>(queue.pendingCount()), 4, "setup: 4 Moves pending, queue at capacity");

  msg::Move moveF = makeTwistMove(46, 99.0f, 0.0f, 0.0f, msg::Move::StopKind::TIME, 1000.0f, 5000.0f, false);
  App::MoveQueue::EnqueueResult resF = queue.enqueue(moveF, 105);
  checkTrue(resF.err == msg::ErrCode::ERR_FULL, "a 5th pending Move is rejected ERR_FULL");
  checkUintEq(resF.corrId, 105, "EnqueueResult still echoes the rejected command's corrId");

  checkUintEq(static_cast<uint32_t>(queue.pendingCount()), 4, "pending count is UNCHANGED at 4 after the rejection");
  checkUintEq(queue.activeMoveId(), 41, "the active Move (A) is unchanged after the rejection");
  checkMovesEqual(queue.pendingAt(0), moveB, "pending[0] (B) is byte-for-byte unchanged after the rejection");
  checkMovesEqual(queue.pendingAt(1), moveC, "pending[1] (C) is byte-for-byte unchanged after the rejection");
  checkMovesEqual(queue.pendingAt(2), moveD, "pending[2] (D) is byte-for-byte unchanged after the rejection");
  checkMovesEqual(queue.pendingAt(3), moveE, "pending[3] (E) is byte-for-byte unchanged after the rejection");
}

// ===========================================================================
// 8. flush() (used by STOP): drains active+pending with NO completion ack
//    (structurally -- flush() returns void) and stops Drive.
// ===========================================================================

void scenarioFlushDrainsAllPendingAndActiveWithNoCompletionAckAndStopsDrive() {
  beginScenario("MoveQueue::flush(): drains active+pending with NO completion ack (void return), stops Drive");

  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  Devices::NezhaMotor left(plant, baseNezhaConfig(1));
  Devices::NezhaMotor right(plant, baseNezhaConfig(2));
  primeAtZero(left, bus, kWireAddr);
  primeAtZero(right, bus, kWireAddr);

  TestSim::SimClock clock;
  App::Drive drive(left, right, kTrackWidth);
  App::Odometry odom(left, right, kTrackWidth);
  // Turn-prediction campaign: MoveQueue now holds a const StateEstimator&
  // (bodyAt()-driven stop-condition anticipation) -- default-constructed
  // (body peer stays valid=false, never updated by this harness) and
  // stopLead left at its own constructor default (0, anticipation OFF),
  // so every scenario below is unaffected: the anticipation branch in
  // tick() is gated on stopLead_ > 0, matching this file's own pre-
  // existing raw-Odometry-baseline assertions unchanged.
  App::StateEstimator estimator;
  App::MoveQueue queue(drive, odom, clock, estimator);

  msg::Move moveA = makeWheelsMove(51, 60.0f, 60.0f, msg::Move::StopKind::TIME, 5000.0f, 10000.0f, false);
  queue.enqueue(moveA, 1);
  msg::Move moveB = makeWheelsMove(52, 30.0f, 30.0f, msg::Move::StopKind::TIME, 5000.0f, 10000.0f, false);
  msg::Move moveC = makeWheelsMove(53, 15.0f, 15.0f, msg::Move::StopKind::TIME, 5000.0f, 10000.0f, false);
  queue.enqueue(moveB, 2);
  queue.enqueue(moveC, 3);
  checkTrue(queue.active(), "setup: A is active");
  checkUintEq(static_cast<uint32_t>(queue.pendingCount()), 2, "setup: B, C pending");

  drive.tick();
  runOneCycleAtZeroPosition(left, bus, kWireAddr, kPastWriteThrottleUs);
  runOneCycleAtZeroPosition(right, bus, kWireAddr, kPastWriteThrottleUs);
  checkTrue(left.appliedDuty() != 0.0f, "setup: left duty nonzero -- A is genuinely driving before flush()");

  queue.flush();  // void return -- structurally, no completion is ever reported for a flush()

  checkFalse(queue.active(), "flush() ends the active Move -- queue is no longer active");
  checkUintEq(static_cast<uint32_t>(queue.pendingCount()), 0, "flush() drains every pending slot");

  drive.tick();
  runOneCycleAtZeroPosition(left, bus, kWireAddr, kPastWriteThrottleUs + 200000);
  runOneCycleAtZeroPosition(right, bus, kWireAddr, kPastWriteThrottleUs + 200000);
  checkFloatEq(left.appliedDuty(), 0.0f, "flush() calls Drive::stop() -- left duty reaches 0");
  checkFloatEq(right.appliedDuty(), 0.0f, "flush() calls Drive::stop() -- right duty reaches 0");

  // After flush(), the queue behaves as freshly empty -- proves B/C are
  // truly gone, not merely deprioritized ("still 2 pending" residue).
  msg::Move moveD = makeTwistMove(54, 5.0f, 0.0f, 0.0f, msg::Move::StopKind::TIME, 100.0f, 5000.0f, false);
  App::MoveQueue::EnqueueResult resD = queue.enqueue(moveD, 4);
  checkTrue(resD.err == msg::ErrCode::ERR_NONE, "post-flush() enqueue succeeds on a genuinely empty queue");
  checkUintEq(queue.activeMoveId(), 54, "D activates immediately -- B/C are truly gone, not just deprioritized");
}

// ===========================================================================
// 9. Timeout (AC #3): a DISTANCE Move whose threshold is unreachable ends
//    AT `timeout`, with completion.timedOut == true (distinguishing it from
//    a kind-specific StopConditionMet).
// ===========================================================================

void scenarioTimeoutEndsStalledDistanceMoveWithTimedOutTrue() {
  beginScenario("MoveQueue: a DISTANCE Move whose threshold is unreachable ends at `timeout` with timedOut == true");

  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  Devices::NezhaMotor left(plant, baseNezhaConfig(1));
  Devices::NezhaMotor right(plant, baseNezhaConfig(2));
  primeAtZero(left, bus, kWireAddr);
  primeAtZero(right, bus, kWireAddr);

  TestSim::SimClock clock;
  App::Drive drive(left, right, kTrackWidth);
  App::Odometry odom(left, right, kTrackWidth);  // pathLength() stays 0 -- wheels never move in this scenario
  // Turn-prediction campaign: MoveQueue now holds a const StateEstimator&
  // (bodyAt()-driven stop-condition anticipation) -- default-constructed
  // (body peer stays valid=false, never updated by this harness) and
  // stopLead left at its own constructor default (0, anticipation OFF),
  // so every scenario below is unaffected: the anticipation branch in
  // tick() is gated on stopLead_ > 0, matching this file's own pre-
  // existing raw-Odometry-baseline assertions unchanged.
  App::StateEstimator estimator;
  App::MoveQueue queue(drive, odom, clock, estimator);

  clock.setMicros(0);
  msg::Move move = makeWheelsMove(/*id=*/61, 40.0f, 40.0f, msg::Move::StopKind::DISTANCE,
                                   /*stopValue=*/500.0f /*[mm], never reached -- no encoder motion below*/,
                                   /*timeout=*/200.0f /*[ms]*/, false);
  queue.enqueue(move, 6);

  clock.setMicros(199000);  // 199ms -- under the 200ms timeout, pathLength still 0
  App::MoveQueue::TickResult tick1 = queue.tick(clock.nowMicros(), odom);
  checkFalse(tick1.completed, "not yet timed out at 199ms, DISTANCE condition nowhere near met (pathLength still 0)");

  clock.setMicros(200000);  // 200ms -- AT the timeout
  App::MoveQueue::TickResult tick2 = queue.tick(clock.nowMicros(), odom);
  checkTrue(tick2.completed, "ends AT the 200ms timeout");
  checkUintEq(tick2.completion.moveId, 61, "completion reports the ended Move's id");
  checkTrue(tick2.completion.timedOut, "ended via the timeout backstop, NOT the DISTANCE condition -- timedOut must be true");
}

// ===========================================================================
// 10. tick() with no active Move is a no-op.
// ===========================================================================

void scenarioTickWithNoActiveMoveIsANoOp() {
  beginScenario("MoveQueue::tick(): a no-op (not completed) when no Move is active");

  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  Devices::NezhaMotor left(plant, baseNezhaConfig(1));
  Devices::NezhaMotor right(plant, baseNezhaConfig(2));

  TestSim::SimClock clock;
  App::Drive drive(left, right, kTrackWidth);
  App::Odometry odom(left, right, kTrackWidth);
  // Turn-prediction campaign: MoveQueue now holds a const StateEstimator&
  // (bodyAt()-driven stop-condition anticipation) -- default-constructed
  // (body peer stays valid=false, never updated by this harness) and
  // stopLead left at its own constructor default (0, anticipation OFF),
  // so every scenario below is unaffected: the anticipation branch in
  // tick() is gated on stopLead_ > 0, matching this file's own pre-
  // existing raw-Odometry-baseline assertions unchanged.
  App::StateEstimator estimator;
  App::MoveQueue queue(drive, odom, clock, estimator);

  checkFalse(queue.active(), "fresh MoveQueue starts with no active Move");
  App::MoveQueue::TickResult result = queue.tick(clock.nowMicros(), odom);
  checkFalse(result.completed, "tick() on an empty queue reports no completion");
  checkFalse(queue.active(), "still not active after a no-op tick()");
}

// ===========================================================================
// 11. Turn-prediction campaign: a positive `stopLead` evaluates an ANGLE
//    stop condition against StateEstimator::bodyAt(now + stopLead), not raw
//    Odometry::theta() -- proven by making the RAW odom reading stay well
//    under threshold (theta() untouched at 0.0) while the estimator's own
//    predicted heading alone crosses it. If MoveQueue fell back to reading
//    odom directly here, this Move would never complete.
// ===========================================================================

void scenarioAngleMoveAnticipatesViaStateEstimatorPredictedHeading() {
  beginScenario("MoveQueue: positive stopLead evaluates ANGLE stop against "
               "StateEstimator::bodyAt(now+stopLead), not raw Odometry::theta()");

  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  Devices::NezhaMotor left(plant, baseNezhaConfig(1));
  Devices::NezhaMotor right(plant, baseNezhaConfig(2));
  primeAtZero(left, bus, kWireAddr);
  primeAtZero(right, bus, kWireAddr);

  TestSim::SimClock clock;
  App::Drive drive(left, right, kTrackWidth);
  App::Odometry odom(left, right, kTrackWidth);  // theta() stays 0.0 -- wheels never move this scenario
  App::StateEstimator estimator;
  App::MoveQueue queue(drive, odom, clock, estimator, /*stopLead=*/200);

  clock.setMicros(10000000);  // 10s -- arbitrary nonzero base, matches update()'s own [ms] domain below
  msg::Move move = makeTwistMove(/*id=*/71, /*v_x=*/0.0f, /*v_y=*/0.0f, /*omega=*/0.5f,
                                  msg::Move::StopKind::ANGLE, /*stopValue=*/0.5f /*[rad]*/,
                                  /*timeout=*/60000.0f, /*replace=*/false);
  queue.enqueue(move, /*corrId=*/7);

  // Feed the estimator a body basis: heading=0.42rad, omega=0.5rad/s, basis
  // time == this tick's own `now` (so bodyAt(now+200ms)'s own age is
  // EXACTLY stopLead -- see state_estimator.h's own age math). frame.pose.h
  // is DELIBERATELY 0.42, NOT odom.theta() (0.0) -- this scenario's whole
  // point is that MoveQueue reads the ESTIMATOR's prediction, never odom,
  // once stopLead_ > 0.
  App::Telemetry::Frame frame;
  frame.pose = msg::Pose2D{0.0f, 0.0f, 0.42f};
  frame.twist = msg::BodyTwist3{0.0f, 0.0f, 0.5f};
  uint32_t nowMs = static_cast<uint32_t>(clock.nowMicros() / 1000);
  estimator.update(frame, nowMs);

  // Predicted heading at now+200ms = 0.42 + 0.5*0.2 = 0.52rad -- OVER the
  // 0.5rad threshold, despite odom.theta() reading exactly 0.0.
  checkFloatEq(odom.theta(), 0.0f, "sanity: raw odom.theta() is untouched (0.0) -- wheels never moved");
  App::MoveQueue::TickResult tick = queue.tick(clock.nowMicros(), odom);
  checkTrue(tick.completed, "ANGLE Move completes via the PREDICTED heading (0.52rad >= 0.5rad threshold), "
                            "even though raw odom.theta() (0.0rad) never came close");
  checkUintEq(tick.completion.moveId, 71, "completion reports the ended Move's id");
  checkFalse(tick.completion.timedOut, "ended via the (anticipated) ANGLE condition, not the timeout backstop");
}

// ===========================================================================
// 12. Turn-prediction campaign: the SAME anticipation, DISTANCE kind -- a
//    positive `stopLead` predicts pathLength forward using the estimator's
//    own held body-frame speed (StateEstimator::bodyAt()'s v_x/v_y), added
//    to the CURRENT odom.pathLength() baseline. Raw odom.pathLength() stays
//    at 0 (wheels never move) -- only the predicted increment crosses the
//    threshold.
// ===========================================================================

void scenarioDistanceMoveAnticipatesViaStateEstimatorPredictedSpeed() {
  beginScenario("MoveQueue: positive stopLead evaluates DISTANCE stop against a StateEstimator-"
               "predicted pathLength, not raw Odometry::pathLength() alone");

  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  Devices::NezhaMotor left(plant, baseNezhaConfig(1));
  Devices::NezhaMotor right(plant, baseNezhaConfig(2));
  primeAtZero(left, bus, kWireAddr);
  primeAtZero(right, bus, kWireAddr);

  TestSim::SimClock clock;
  App::Drive drive(left, right, kTrackWidth);
  App::Odometry odom(left, right, kTrackWidth);  // pathLength() stays 0 -- wheels never move this scenario
  App::StateEstimator estimator;
  App::MoveQueue queue(drive, odom, clock, estimator, /*stopLead=*/200);

  clock.setMicros(10000000);
  msg::Move move = makeTwistMove(/*id=*/72, /*v_x=*/200.0f, /*v_y=*/0.0f, /*omega=*/0.0f,
                                  msg::Move::StopKind::DISTANCE, /*stopValue=*/80.0f /*[mm]*/,
                                  /*timeout=*/60000.0f, /*replace=*/false);
  queue.enqueue(move, /*corrId=*/8);

  // v_x=500mm/s held basis -> predicted increment over 200ms = 100mm, added
  // to odom.pathLength() (0) = 100mm -- OVER the 80mm threshold, despite
  // odom.pathLength() itself never moving.
  App::Telemetry::Frame frame;
  frame.pose = msg::Pose2D{0.0f, 0.0f, 0.0f};
  frame.twist = msg::BodyTwist3{500.0f, 0.0f, 0.0f};
  uint32_t nowMs = static_cast<uint32_t>(clock.nowMicros() / 1000);
  estimator.update(frame, nowMs);

  checkFloatEq(odom.pathLength(), 0.0f, "sanity: raw odom.pathLength() is untouched (0.0) -- wheels never moved");
  App::MoveQueue::TickResult tick = queue.tick(clock.nowMicros(), odom);
  checkTrue(tick.completed, "DISTANCE Move completes via the PREDICTED pathLength (100mm >= 80mm threshold), "
                            "even though raw odom.pathLength() (0mm) never came close");
  checkUintEq(tick.completion.moveId, 72, "completion reports the ended Move's id");
  checkFalse(tick.completion.timedOut, "ended via the (anticipated) DISTANCE condition, not the timeout backstop");
}

// ===========================================================================
// 13. Decel-into-the-goal campaign: a Distance-kind TWIST Move with LINEAR
//    shaping enabled (angular disabled) ramps v_x up from 0 (never jumps
//    straight to cruise at activation), holds/continues ramping while
//    `remaining` is large, then TAPERS below what pure acceleration would
//    produce once `remaining` shrinks -- the stakeholder's own "speed
//    drops as you approach the target." The stop condition itself still
//    fires correctly with shaping active, and the LAST shaped speed staged
//    before stop-fire is well below cruise.
// ===========================================================================

void scenarioDistanceMoveShapesLinearSpeedRampUpThenTaperNearGoal() {
  beginScenario("MoveQueue: Distance-kind TWIST Move shapes v_x -- ramps up from 0, tapers near the goal");

  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  Devices::NezhaMotor left(plant, baseNezhaConfig(1));
  Devices::NezhaMotor right(plant, baseNezhaConfig(2));
  primeAtZero(left, bus, kWireAddr);
  primeAtZero(right, bus, kWireAddr);

  TestSim::SimClock clock;
  App::Drive drive(left, right, kTrackWidth);
  App::Odometry odom(left, right, kTrackWidth);
  App::StateEstimator estimator;
  App::ShaperLimits limits;
  limits.aMax = 1000.0f;    // [mm/s^2]
  limits.aDecel = 800.0f;   // [mm/s^2]
  limits.alphaMax = 0.0f;   // angular shaping disabled -- irrelevant, v_x==300/omega==0 here anyway
  limits.alphaDecel = 0.0f;
  App::MoveQueue queue(drive, odom, clock, estimator, /*stopLead=*/0, limits);

  // Two INDEPENDENT timelines, matching this file's own established
  // convention (see scenario 2's own driveToPosition() calls): `clock`
  // (StopCondition/VelocityShaper's own [us] `now`, small realistic
  // ~20ms-cycle increments below) and `nowUs` (the NezhaMotor encoder
  // cycle's own plausibility-check clock, passed directly to
  // runOneCycleAtZeroPosition()/driveToPosition() -- large, well-spaced
  // round numbers, monotonically increasing, >= kPastWriteThrottleUs apart
  // per this file's own write-rate-throttle note). nowUs increments by
  // 100000 (100ms) each step below -- generous headroom over both the
  // 40000us write-rate throttle and NezhaMotor's own implausible-step
  // rejection.
  uint64_t nowUs = 100000;

  clock.setMicros(0);
  msg::Move move = makeTwistMove(/*id=*/81, /*v_x=*/300.0f, /*v_y=*/0.0f, /*omega=*/0.0f,
                                  msg::Move::StopKind::DISTANCE, /*stopValue=*/50.0f /*[mm]*/,
                                  /*timeout=*/600000.0f, /*replace=*/false);
  queue.enqueue(move, /*corrId=*/1);

  // Activation itself stages the CARRIED-OVER shaped state (0 -- a fresh
  // queue's own resting value), NOT the raw 300mm/s cruise target --
  // proves activate() doesn't jump straight to cruise when shaping is on.
  drive.tick();
  runOneCycleAtZeroPosition(left, bus, kWireAddr, nowUs);
  runOneCycleAtZeroPosition(right, bus, kWireAddr, nowUs);
  checkFloatEq(left.appliedDuty(), 0.0f, "activation stages the carried-over shaped state (0), not raw cruise");
  checkFloatEq(right.appliedDuty(), 0.0f, "activation stages the carried-over shaped state (0), not raw cruise");

  // t=20ms, no travel yet (remaining stays the full 50mm): accel-ramp
  // clamp binds (aMax*dt=1000*0.02=20mm/s); decel taper
  // (sqrt(2*800*50)~=282.8) does not.
  clock.setMicros(20000);
  App::MoveQueue::TickResult tick1 = queue.tick(clock.nowMicros(), odom);
  checkFalse(tick1.completed, "still far from the 50mm threshold");
  drive.tick();
  nowUs += 100000;
  runOneCycleAtZeroPosition(left, bus, kWireAddr, nowUs);
  runOneCycleAtZeroPosition(right, bus, kWireAddr, nowUs);
  checkFloatEq(left.appliedDuty(), kKff * 20.0f, "tick 1 -- accel-ramped to aMax*dt (1000*0.02), never straight to cruise");
  checkFloatEq(right.appliedDuty(), kKff * 20.0f, "tick 1 -- accel-ramped to aMax*dt (1000*0.02), never straight to cruise");

  // t=40ms, still no travel: accel ramp continues (20 -> 40), monotonic
  // rise -- the stakeholder's own "speed rises smoothly" half of the
  // shaper, not just the taper half.
  clock.setMicros(40000);
  App::MoveQueue::TickResult tick2 = queue.tick(clock.nowMicros(), odom);
  checkFalse(tick2.completed, "still far from the 50mm threshold");
  drive.tick();
  nowUs += 100000;
  runOneCycleAtZeroPosition(left, bus, kWireAddr, nowUs);
  runOneCycleAtZeroPosition(right, bus, kWireAddr, nowUs);
  checkFloatEq(left.appliedDuty(), kKff * 40.0f, "tick 2 -- accel ramp continues monotonically (20 -> 40)");
  checkFloatEq(right.appliedDuty(), kKff * 40.0f, "tick 2 -- accel ramp continues monotonically (20 -> 40)");

  // Now travel to within 2mm of the 50mm threshold (48mm traveled): the
  // decel taper (sqrt(2*800*2)~=56.57) now binds BELOW what pure
  // acceleration would have produced (40+1000*0.02=60) -- the taper
  // itself, not just the ramp.
  nowUs += 100000;
  driveToPosition(left, bus, kWireAddr, 48.0f, nowUs);
  driveToPosition(right, bus, kWireAddr, 48.0f, nowUs);
  odom.integrate();
  checkFloatEq(odom.pathLength(), 48.0f, "sanity: 48mm traveled, 2mm remaining");
  clock.setMicros(60000);
  App::MoveQueue::TickResult tick3 = queue.tick(clock.nowMicros(), odom);
  checkFalse(tick3.completed, "48mm < 50mm threshold -- still Continue");
  float expectedTaper = std::sqrt(2.0f * 800.0f * 2.0f);  // ~56.57mm/s
  checkLe(expectedTaper, 300.0f, "sanity: the taper ceiling is below cruise this close to the goal");
  drive.tick();
  nowUs += 100000;
  runOneCycleAtZeroPosition(left, bus, kWireAddr, nowUs);
  runOneCycleAtZeroPosition(right, bus, kWireAddr, nowUs);
  checkFloatEq(left.appliedDuty(), kKff * expectedTaper,
               "tick 3 -- decel taper CAPS speed below the pure accel-ramp value (60) near the goal", 0.05f);
  checkFloatEq(right.appliedDuty(), kKff * expectedTaper,
               "tick 3 -- decel taper CAPS speed below the pure accel-ramp value (60) near the goal", 0.05f);
  checkLe(expectedTaper, 100.0f, "acceptance: approach speed near the goal is well below cruise (< 1/3 of 300mm/s)");

  // Finally, cross the threshold (50mm traveled) -- the stop condition
  // still fires correctly with shaping active.
  nowUs += 100000;
  driveToPosition(left, bus, kWireAddr, 50.0f, nowUs);
  driveToPosition(right, bus, kWireAddr, 50.0f, nowUs);
  odom.integrate();
  clock.setMicros(80000);
  App::MoveQueue::TickResult tick4 = queue.tick(clock.nowMicros(), odom);
  checkTrue(tick4.completed, "the DISTANCE stop condition still fires once 50mm is reached");
  checkUintEq(tick4.completion.moveId, 81, "completion reports the shaped Move's own id");
}

// ===========================================================================
// 14. Decel-into-the-goal campaign: an Angle-kind TWIST Move with ANGULAR
//    shaping enabled (linear disabled) shapes omega the SAME way scenario
//    13 shapes v_x -- ramps up, then tapers near the heading target.
//    Pure rotation (equal-and-opposite wheel deltas, app_odometry_
//    harness.cpp's own convention) via REAL Odometry::theta() progression.
// ===========================================================================

void scenarioAngleMoveShapesAngularSpeedRampUpThenTaperNearGoal() {
  beginScenario("MoveQueue: Angle-kind TWIST Move shapes omega -- ramps up, tapers near the goal");

  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  Devices::NezhaMotor left(plant, baseNezhaConfig(1));
  Devices::NezhaMotor right(plant, baseNezhaConfig(2));
  primeAtZero(left, bus, kWireAddr);
  primeAtZero(right, bus, kWireAddr);

  TestSim::SimClock clock;
  App::Drive drive(left, right, kTrackWidth);
  App::Odometry odom(left, right, kTrackWidth);
  App::StateEstimator estimator;
  App::ShaperLimits limits;
  limits.aMax = 0.0f;       // linear shaping disabled -- irrelevant, v_x==0 here anyway
  limits.aDecel = 0.0f;
  limits.alphaMax = 6.0f;    // [rad/s^2]
  limits.alphaDecel = 0.5f;  // [rad/s^2] -- deliberately weak (a unit-test value, not
                              // a production default): makes the taper ceiling reachable
                              // by a small, easy-to-drive `remaining` below (see tick 2's
                              // own comment for the arithmetic this depends on)
  App::MoveQueue queue(drive, odom, clock, estimator, /*stopLead=*/0, limits);

  // Two INDEPENDENT timelines -- see scenario 13's own comment for why
  // (`clock`'s small ~20ms-cycle increments below drive VelocityShaper's
  // own dt; `nowUs`'s large, well-spaced, monotonically increasing values
  // drive NezhaMotor's own encoder-cycle plausibility checks).
  //
  // NezhaMotor duty tolerance: NezhaMotor internally represents duty as an
  // INTEGER PWM percent (nezha_motor.cpp's own writeShapedDuty()/
  // clampStep() -- a real, pre-existing quantization step, not a shaping
  // artifact) -- a fraction-of-a-percent duty (this scenario's small
  // rad/s-scale omega targets, unlike scenario 13's much larger mm/s-scale
  // ones) can round to the nearest whole percentage point (0.01 duty).
  // kDutyQuantization below is that one-percentage-point bound, used as
  // the tolerance for every appliedDuty() comparison in this scenario.
  constexpr float kDutyQuantization = 0.011f;
  uint64_t nowUs = 100000;

  clock.setMicros(0);
  msg::Move move = makeTwistMove(/*id=*/82, /*v_x=*/0.0f, /*v_y=*/0.0f, /*omega=*/2.0f,
                                  msg::Move::StopKind::ANGLE, /*stopValue=*/1.5f /*[rad]*/,
                                  /*timeout=*/600000.0f, /*replace=*/false);
  queue.enqueue(move, /*corrId=*/2);

  // Activation stages the carried-over shaped state (0), not raw omega.
  drive.tick();
  runOneCycleAtZeroPosition(left, bus, kWireAddr, nowUs);
  runOneCycleAtZeroPosition(right, bus, kWireAddr, nowUs);
  checkFloatEq(left.appliedDuty(), 0.0f, "activation stages the carried-over shaped state (0), not raw omega");
  checkFloatEq(right.appliedDuty(), 0.0f, "activation stages the carried-over shaped state (0), not raw omega");

  // t=20ms, no rotation yet (remaining stays the full 1.5rad): accel-ramp
  // clamp binds (alphaMax*dt=6*0.02=0.12rad/s); decel taper
  // (sqrt(2*8*1.5)~=4.9rad/s) does not.
  clock.setMicros(20000);
  App::MoveQueue::TickResult tick1 = queue.tick(clock.nowMicros(), odom);
  checkFalse(tick1.completed, "still far from the 1.5rad threshold");
  float expectedVL = 0.0f, expectedVR = 0.0f;
  BodyKinematics::inverse(0.0f, 0.12f, kTrackWidth, expectedVL, expectedVR);
  drive.tick();
  nowUs += 100000;
  runOneCycleAtZeroPosition(left, bus, kWireAddr, nowUs);
  runOneCycleAtZeroPosition(right, bus, kWireAddr, nowUs);
  checkFloatEq(left.appliedDuty(), kKff * expectedVL,
               "tick 1 -- omega accel-ramped to alphaMax*dt (6*0.02), never straight to cruise", kDutyQuantization);
  checkFloatEq(right.appliedDuty(), kKff * expectedVR,
               "tick 1 -- omega accel-ramped to alphaMax*dt (6*0.02), never straight to cruise", kDutyQuantization);

  // Now rotate to within 0.03rad of the 1.5rad threshold (1.47rad turned):
  // with alphaDecel=0.5, the decel taper (sqrt(2*0.5*0.03)~=0.173rad/s) now
  // binds BELOW what pure acceleration would have produced
  // (0.12+6*0.02=0.24) -- the taper itself, not just the ramp. The step
  // itself (147mm from 0) needs >=~123ms of elapsed nowUs to stay under
  // NezhaMotor's own kMaxPlausibleStepSpeed=1200mm/s outlier-rejection
  // gate (nezha_motor.cpp) -- 200ms (735mm/s implied) is comfortably under.
  nowUs += 200000;
  driveToPosition(left, bus, kWireAddr, -1.47f * (kTrackWidth / 2.0f), nowUs);
  driveToPosition(right, bus, kWireAddr, 1.47f * (kTrackWidth / 2.0f), nowUs);
  odom.integrate();
  checkFloatEq(odom.theta(), 1.47f, "sanity: 1.47rad turned, 0.03rad remaining", 1e-3f);
  clock.setMicros(40000);
  App::MoveQueue::TickResult tick2 = queue.tick(clock.nowMicros(), odom);
  checkFalse(tick2.completed, "1.47rad < 1.5rad threshold -- still Continue");
  float expectedTaper = std::sqrt(2.0f * 0.5f * 0.03f);  // ~0.173rad/s
  checkLe(expectedTaper, 0.24f, "sanity: the taper ceiling is below the pure-accel-ramp value (0.24) here");
  BodyKinematics::inverse(0.0f, expectedTaper, kTrackWidth, expectedVL, expectedVR);
  drive.tick();
  nowUs += 100000;
  runOneCycleAtZeroPosition(left, bus, kWireAddr, nowUs);
  runOneCycleAtZeroPosition(right, bus, kWireAddr, nowUs);
  checkFloatEq(left.appliedDuty(), kKff * expectedVL,
               "tick 2 -- decel taper CAPS omega below the pure accel-ramp value (0.24) near the goal", kDutyQuantization);
  checkFloatEq(right.appliedDuty(), kKff * expectedVR,
               "tick 2 -- decel taper CAPS omega below the pure accel-ramp value (0.24) near the goal", kDutyQuantization);
  checkLe(expectedTaper, 1.0f, "acceptance: approach omega near the goal is well below cruise (< half of 2.0rad/s)");
}

}  // namespace

int main() {
  scenarioEnqueueOnEmptyQueueActivatesTwistImmediately();
  scenarioWheelsDistanceMoveUsesRealOdometryBaseline();
  scenarioAngleMoveUsesRealOdometryHeadingBaseline();
  scenarioTimeMoveContinuesThenCompletesAndDrainsEmptyToStop();
  scenarioChainedMoveActivatesSameCycleNoInterveningStop();
  scenarioReplaceTruePreemptsActiveAndFlushesPending();
  scenarioOverflowRejectedErrFullQueueByteForByteUnchanged();
  scenarioFlushDrainsAllPendingAndActiveWithNoCompletionAckAndStopsDrive();
  scenarioTimeoutEndsStalledDistanceMoveWithTimedOutTrue();
  scenarioTickWithNoActiveMoveIsANoOp();
  scenarioAngleMoveAnticipatesViaStateEstimatorPredictedHeading();
  scenarioDistanceMoveAnticipatesViaStateEstimatorPredictedSpeed();
  scenarioDistanceMoveShapesLinearSpeedRampUpThenTaperNearGoal();
  scenarioAngleMoveShapesAngularSpeedRampUpThenTaperNearGoal();

  if (g_failureCount == 0) {
    std::printf("OK: all App::MoveQueue scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the App::MoveQueue scenarios\n", g_failureCount);
  return 1;
}
