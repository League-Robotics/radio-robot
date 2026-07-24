// motion_move_queue_chained_harness.cpp -- sprint 122 ticket 002's own
// end-to-end SUC-001 scenario: enqueue two chained WHEELS Moves against the
// model plant (TestSim::WheelPlant, src/tests/sim/plant/wheel_plant.{h,cpp})
// and verify the completion/chain-advance sequence, proving the
// velocity-sink boundary (motion/wheel_sink.h) is sufficient on its own --
// NO TestSim::SimHarness, NO libfirmware_host, NO Python, NO Devices::/App::
// anything, anywhere in this binary's own build or run path. Built and run
// by src/motion/CMakeLists.txt's `motion_tests` target (Design Rationale
// Decision 4) -- see that file for the plain-CMake build recipe.
//
// TestWheelSink below is deliberately NOT a real PID/motor-controller model
// -- Motion::MoveQueue's own contract is a VELOCITY sink (Design Rationale
// Decision 1; the real velocity PID stays in the base, Devices::NezhaMotor,
// untouched this sprint). This test-only sink maps a commanded wheel
// velocity onto TestSim::WheelPlant's own duty input via a simple linear
// open-loop relation (duty = v / dutyVelMax, clamped to [-1,1]) -- close
// enough that the plant's own first-order lag converges toward the
// commanded speed, which is all this scenario needs to exercise real
// StopCondition-against-real-Odometry completion timing. It is scaffolding
// for THIS test, not a claim about the real motor's own duty response.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include "messages/envelope.h"
#include "motion/move_queue.h"
#include "motion/odometry.h"
#include "motion/wheel_sink.h"
#include "wheel_plant.h"

namespace {

// --- Hand-rolled assertion plumbing (see motion_stop_condition_harness.cpp) --

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

// TestWheelSink -- see file header. Owns two model plants (left/right) and
// steps both by dt every tick() call using whichever target setWheels()/
// stop() most recently staged -- mirrors App::Drive's own "stage now, apply
// at tick()" shape (drive.h), just against the model plant instead of real
// hardware.
class TestWheelSink : public Motion::WheelSink {
 public:
  TestWheelSink(float dutyVelMax, float tau)
      : dutyVelMax_(dutyVelMax), left_(dutyVelMax, tau), right_(dutyVelMax, tau) {}

  void setWheels(float v_left, float v_right) override {
    vLeft_ = v_left;
    vRight_ = v_right;
  }

  void stop() override {
    vLeft_ = 0.0f;
    vRight_ = 0.0f;
  }

  void tick(float dt) {  // [s]
    left_.step(clampDuty(vLeft_), dt);
    right_.step(clampDuty(vRight_), dt);
  }

  float leftPosition() const { return left_.position(); }    // [mm]
  float rightPosition() const { return right_.position(); }  // [mm]

 private:
  float clampDuty(float v) const {
    float duty = v / dutyVelMax_;
    if (duty > 1.0f) duty = 1.0f;
    if (duty < -1.0f) duty = -1.0f;
    return duty;
  }

  float dutyVelMax_;
  float vLeft_ = 0.0f;
  float vRight_ = 0.0f;
  TestSim::WheelPlant left_;
  TestSim::WheelPlant right_;
};

msg::Move makeWheelsMove(uint32_t id, float vLeft, float vRight, msg::Move::StopKind stopKind,
                          float stopValue, float timeout) {
  msg::Move m;
  m.velocity_kind = msg::Move::VelocityKind::WHEELS;
  m.velocity.wheels = msg::MoveWheels{vLeft, vRight};
  m.stop_kind = stopKind;
  switch (stopKind) {
    case msg::Move::StopKind::DISTANCE:
      m.stop.distance = stopValue;
      break;
    case msg::Move::StopKind::ANGLE:
      m.stop.angle = stopValue;
      break;
    case msg::Move::StopKind::TIME:
    case msg::Move::StopKind::NONE:
    default:
      m.stop.time = stopValue;
      break;
  }
  m.timeout = timeout;
  m.replace = false;
  m.id = id;
  return m;
}

// ===========================================================================
// Two chained WHEELS Moves, both DISTANCE-stop, straight travel: Move A
// (100mm) enqueues, activates immediately; Move B (another 100mm) enqueues
// behind it as pending. Stepping the REAL model plant (TestSim::WheelPlant)
// through Motion::MoveQueue::tick() against a REAL Motion::Odometry (fed
// from the plant's own positions) must: (1) complete A once ~100mm is
// traveled, (2) chain-advance to B the SAME tick() call (SUC-051, no
// intervening stop), (3) complete B once its OWN ~100mm threshold is
// reached, measured from B's own activation baseline (not a cumulative
// 200mm from the start). No SimHarness, no libfirmware_host, no Python
// anywhere in this scenario's own build/run path -- proves the boundary
// (motion/wheel_sink.h) is sufficient on its own.
// ===========================================================================

void scenarioTwoChainedWheelsMovesCompleteInSequence() {
  beginScenario(
      "Motion::MoveQueue: two chained WHEELS+DISTANCE Moves complete in sequence against the model plant");

  const float kTrackWidth = 200.0f;                    // [mm]
  const float kDt = 0.04f;                              // [s] matches App::RobotLoop::kCycle (40ms)
  const uint64_t kDtUs = static_cast<uint64_t>(kDt * 1.0e6f);  // [us]

  TestWheelSink sink(TestSim::kDefaultDutyVelMax, TestSim::kDefaultTau);
  Motion::Odometry odom(kTrackWidth, sink.leftPosition(), sink.rightPosition());
  Motion::MoveQueue queue(sink, odom, kTrackWidth);

  uint64_t now = 0;

  msg::Move moveA = makeWheelsMove(/*id=*/1, /*vLeft=*/150.0f, /*vRight=*/150.0f,
                                    msg::Move::StopKind::DISTANCE, /*stopValue=*/100.0f,
                                    /*timeout=*/10000.0f);
  Motion::MoveQueue::EnqueueResult enqA = queue.enqueue(moveA, /*corrId=*/10, now);
  checkTrue(enqA.err == msg::ErrCode::ERR_NONE, "Move A activates on the empty queue");
  checkTrue(queue.active(), "queue is active immediately after A enqueues");
  checkUintEq(queue.activeMoveId(), 1, "A is the active Move");

  msg::Move moveB = makeWheelsMove(/*id=*/2, /*vLeft=*/150.0f, /*vRight=*/150.0f,
                                    msg::Move::StopKind::DISTANCE, /*stopValue=*/100.0f,
                                    /*timeout=*/10000.0f);
  Motion::MoveQueue::EnqueueResult enqB = queue.enqueue(moveB, /*corrId=*/11, now);
  checkTrue(enqB.err == msg::ErrCode::ERR_NONE, "Move B enqueues behind A -- queue has room");
  checkUintEq(static_cast<uint32_t>(queue.pendingCount()), 1, "B is pending, 1 slot occupied");

  bool completedA = false;
  bool completedB = false;
  uint32_t completedAMoveId = 0;
  uint32_t completedBMoveId = 0;

  // Generous bound: at 150mm/s cruise (once the plant's own first-order lag
  // converges), 100mm takes well under 1s; 500 cycles at 40ms is 20s of
  // virtual time, comfortably past both Moves' own completion even
  // accounting for the plant's tau=0.13s spin-up lag on each activation.
  for (int i = 0; i < 500 && !completedB; ++i) {
    now += kDtUs;
    sink.tick(kDt);
    odom.integrate(sink.leftPosition(), sink.rightPosition());

    Motion::MoveQueue::TickResult result = queue.tick(now, odom);
    if (result.completed && !completedA) {
      completedA = true;
      completedAMoveId = result.completion.moveId;
      checkFalse(result.completion.timedOut, "A completes via its DISTANCE condition, not the timeout backstop");
      checkTrue(queue.active(), "B chain-advances the SAME tick() call A completes on (SUC-051)");
      checkUintEq(queue.activeMoveId(), 2, "B is now the active Move immediately after A's completion");
      checkUintEq(static_cast<uint32_t>(queue.pendingCount()), 0, "B moved out of pending into active");
    } else if (result.completed && completedA) {
      completedB = true;
      completedBMoveId = result.completion.moveId;
      checkFalse(result.completion.timedOut, "B completes via its DISTANCE condition, not the timeout backstop");
      checkFalse(queue.active(), "queue drains empty -- nothing was pending behind B");
    }
  }

  checkTrue(completedA, "Move A completes within the bounded step budget");
  checkUintEq(completedAMoveId, 1, "the FIRST completion reports A's own id");
  checkTrue(completedB, "Move B completes within the bounded step budget, chained after A");
  checkUintEq(completedBMoveId, 2, "the SECOND completion reports B's own id, not A's again");
}

}  // namespace

int main() {
  scenarioTwoChainedWheelsMovesCompleteInSequence();

  if (g_failureCount == 0) {
    std::printf("OK: all Motion::MoveQueue chained-moves end-to-end scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the Motion::MoveQueue chained-moves scenarios\n",
              g_failureCount);
  return 1;
}
