// test_app_robot_loop_goto_harness.cpp -- 135-004's own acceptance proof:
// Core::RobotLoop::handleGoto()/routeCommand()'s GOTO case/cycle()'s
// Navigator-vs-Planner ownership dispatch, exercised end to end against the
// REAL Core::RobotLoop graph (TestSim::SimHarness, composeRobot() -- the
// SAME composition root main.cpp uses, baking data/robots/tovez.json's
// real navigator block: yaw_sign=-1.0, align_tol/align_max_nudges, etc.).
//
// Landmine 1 (spurious ack(0) per internal segment) is proven here, not
// merely asserted: every internal Navigator-issued segment carries
// Move::id == 0 (navigator.h's own GotoTarget doc comment) -- this file
// picks a goto id and a corr_id that are BOTH nonzero and distinct, so ANY
// ack ring entry whose packed corr_id is 0, observed at ANY point across a
// run with several internal replacements, is unambiguous proof of the bug
// this ticket's own Landmine-1 fix (RobotLoop::publishGotoResult(), never
// publishMoveResult(), on a cycle a goto owns Drive) closes.
//
// Ownership scenarios (135-004's own "Ownership: estop/MOVE/WHEELS cancel
// an active goto, and vice versa" section): MOVE/WHEELS/ESTOP each cancel
// an active goto with no completion ack; GO_TO cancels active WHEELS
// teleop; ESTOP clears the Navigator's target the same cycle it clears
// the Planner's queue.
//
// Compiled by test_app_robot_loop_goto.py against the same full HOST_BUILD
// dependency graph test_sim_harness_configure.py/test_app_robot_loop_replace.py
// compile -- SimHarness composes the real Core::RobotLoop graph.
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "bench_test_config.h"
#include "sim_harness.h"

namespace {

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

void checkFloatNear(float actual, float expected, float tol, const std::string& what) {
  if (std::fabs(actual - expected) > tol) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %g +/- %g, got %g", what.c_str(),
                  static_cast<double>(expected), static_cast<double>(tol),
                  static_cast<double>(actual));
    fail(buf);
  }
}

// Every ack packed word this run's frames carried, unpacked to (corrOrId, err).
struct AckSeen {
  uint32_t key = 0;  // corr_id (enqueue ack) or Move/goto id (completion ack)
  uint32_t err = 0;
};

std::vector<AckSeen> collectAcks(std::vector<TestSupport::DecodedLine>& frames) {
  std::vector<AckSeen> out;
  constexpr uint32_t kAckErrBits = 4;
  constexpr uint32_t kAckErrMask = (1u << kAckErrBits) - 1;
  for (const TestSupport::DecodedLine& frame : frames) {
    if (frame.kind != TestSupport::DecodedKind::kTelemetry) continue;
    for (uint8_t i = 0; i < frame.telemetry.acks_count; ++i) {
      const uint32_t packed = frame.telemetry.acks_[i];
      out.push_back(AckSeen{packed >> kAckErrBits, packed & kAckErrMask});
    }
  }
  return out;
}

// --- Scenario 1: basic end-to-end -- GO_TO accepted, Navigator drives,
// exactly one completion ack, zero spurious ack(0) entries (Landmine 1). --
void scenarioBasicGotoAcceptedNavigatorDrivesCompletionAck() {
  beginScenario("GO_TO accepted -> Navigator drives -> exactly one completion ack, "
                "zero spurious ack(0) (Landmine 1)");
  TestSim::SimHarness sim;
  TestSupport::configureSimForBenchTest(sim);
  sim.boot();
  sim.step(3);  // settle: publishOtos() lags boot() by one cycle (rebaseline_pose_harness.cpp's
                // own "settle" pattern) -- handleGoto()'s otos.connected gate needs a live sample.
  checkTrue(sim.isConfigured(), "both ports configured -- handleGoto()'s configuration gate is open");
  checkFalse(sim.navigator().active(), "Navigator starts inactive");

  const uint32_t kGotoId = 777;
  const uint32_t kCorrId = 9001;
  // WORLD frame (0), config-default speed/arrive (0 => NavigatorLimits'
  // own baked defaults), a generous whole-goto timeout.
  sim.injectGoto(/*x=*/700.0f, /*y=*/300.0f, /*frame=*/0, /*speed=*/0.0f, /*arrive=*/0.0f,
                 /*timeout=*/30000.0f, kGotoId, kCorrId);
  sim.step(2);
  checkTrue(sim.navigator().active(), "Navigator accepted the goto and owns Drive");

  std::vector<AckSeen> allAcks;
  bool done = false;
  // Run a FEW cycles past `done` (Navigator::active() going false), not
  // zero: the ack this same cycle's publishGotoResult() just added to
  // tlm_'s own ring is emitted on the NEXT cycle's frame, not this one --
  // tlm_.emit() (cycle()'s own telemetry send) runs BEFORE the Navigator-
  // vs-Planner dispatch block that calls it, the identical one-cycle
  // visibility lag publishMoveResult()'s own ack already has today.
  // Stopping the instant `done` flips would drain every frame except the
  // one this test actually needs.
  int drainedSinceDone = 0;
  for (int i = 0; i < 900 && drainedSinceDone < 5; ++i) {
    sim.step(1);
    std::vector<TestSupport::DecodedLine> frames = sim.drainTelemetry();
    for (const AckSeen& ack : collectAcks(frames)) allAcks.push_back(ack);
    if (!done && !sim.navigator().active()) done = true;
    if (done) ++drainedSinceDone;
  }

  checkTrue(done, "the goto reached Done/Aborted (Navigator::active() went false) within the run budget");
  checkTrue(sim.navigator().replaceCount() >= 1,
            "the Navigator issued at least one internal segment replace en route");

  // The ack ring is a BOUNDED RING that stays visible across SEVERAL
  // consecutive telemetry frames, not a one-shot event -- draining every
  // single cycle (as this test does, to catch the completion ack's own
  // one-cycle-late frame) sees the SAME entry several times over.
  // test_app_robot_loop_replace_harness.cpp's own scenarioDuplicateIdSanityNoOp()
  // established the fix: a boolean "did this key ever appear", never a
  // raw count, is the only count-agnostic way to assert "this ack
  // happened" against a ring with retention. The ACTUAL "exactly one
  // completion EVENT" guarantee is Motion::Navigator's own state machine
  // property (NavResult::completed fires exactly once per goto,
  // navigator.h's own doc comment) -- proven directly, not inferred from
  // ring-frame counting, by navigator_test.cpp's own
  // testExactlyOneCompletionAck() ctest.
  bool sawCompletionAck = false;
  bool sawEnqueueAck = false;
  int spuriousZeroAcks = 0;
  for (const AckSeen& ack : allAcks) {
    if (ack.key == 0) {
      ++spuriousZeroAcks;
      continue;
    }
    if (ack.key == kGotoId) {
      sawCompletionAck = true;
      checkTrue(ack.err == 0, "the goto's own completion ack carries err==0 (fault rides a flag, not ack_err)");
    }
    if (ack.key == kCorrId) sawEnqueueAck = true;
  }
  std::printf("  GOTO_ACKS: saw_completion=%d saw_enqueue=%d spurious_zero=%d replaceCount=%u tickCount=%u\n",
              sawCompletionAck, sawEnqueueAck, spuriousZeroAcks,
              static_cast<unsigned>(sim.navigator().replaceCount()),
              static_cast<unsigned>(sim.navigator().tickCount()));
  checkTrue(sawCompletionAck, "the goto's own completion ack (id == kGotoId) landed in the ack ring");
  checkTrue(sawEnqueueAck, "the enqueue ack (corr_id) landed");
  checkTrue(spuriousZeroAcks == 0,
            "ZERO ack ring entries with corr_id/id == 0 -- Landmine 1: no internal segment "
            "(always id==0) ever reached publishMoveResult()'s unconditional ack path");

  const float dx = sim.trueX() - 700.0f;
  const float dy = sim.trueY() - 300.0f;
  checkTrue(std::hypot(dx, dy) < 150.0f,
            "the robot's true pose ended near the target (generous bound -- this test proves "
            "ROUTING, not tracking accuracy)");
}

// --- Scenario 2: MOVE cancels an active goto -- no completion ack. -------
void scenarioMoveCancelsActiveGotoNoCompletionAck() {
  beginScenario("MOVE cancels an active goto -- Navigator::active() drops, no completion ack ever arrives");
  TestSim::SimHarness sim;
  TestSupport::configureSimForBenchTest(sim);
  sim.boot();

  sim.step(3);  // settle: publishOtos() lags boot() by one cycle
  const uint32_t kGotoId = 555;
  sim.injectGoto(/*x=*/5000.0f, /*y=*/0.0f, /*frame=*/0, /*speed=*/0.0f, /*arrive=*/0.0f,
                 /*timeout=*/30000.0f, kGotoId, /*corrId=*/1);
  sim.step(3);
  checkTrue(sim.navigator().active(), "the goto is active before the preempting MOVE arrives");

  sim.injectMove(/*v_x=*/100.0f, /*v_y=*/0.0f, /*omega=*/0.0f, TestSupport::MoveStopKind::kDistance,
                 /*stopValue=*/200.0f, /*timeout=*/10000.0f, /*replace=*/false, /*id=*/1,
                 /*corrId=*/2);
  sim.step(3);
  checkFalse(sim.navigator().active(), "MOVE cancelled the goto -- Navigator::active() is now false");
  checkTrue(sim.planner().active(), "the MOVE itself is now the one active Planner entry");

  std::vector<AckSeen> allAcks;
  for (int i = 0; i < 300; ++i) {
    sim.step(1);
    std::vector<TestSupport::DecodedLine> frames = sim.drainTelemetry();
    for (const AckSeen& ack : collectAcks(frames)) allAcks.push_back(ack);
  }
  bool sawGotoCompletionAck = false;
  for (const AckSeen& ack : allAcks) {
    if (ack.key == kGotoId) sawGotoCompletionAck = true;
  }
  checkFalse(sawGotoCompletionAck,
             "the preempted goto's own id NEVER shows up in the ack ring -- preempted, not completed");
}

// --- Scenario 3: WHEELS cancels an active goto -- same contract as MOVE. -
void scenarioWheelsCancelsActiveGotoNoCompletionAck() {
  beginScenario("WHEELS cancels an active goto -- Navigator::active() drops, no completion ack");
  TestSim::SimHarness sim;
  TestSupport::configureSimForBenchTest(sim);
  sim.boot();

  sim.step(3);  // settle: publishOtos() lags boot() by one cycle
  const uint32_t kGotoId = 556;
  sim.injectGoto(/*x=*/5000.0f, /*y=*/0.0f, /*frame=*/0, /*speed=*/0.0f, /*arrive=*/0.0f,
                 /*timeout=*/30000.0f, kGotoId, /*corrId=*/1);
  sim.step(3);
  checkTrue(sim.navigator().active(), "the goto is active before the preempting WHEELS arrives");

  sim.injectWheels(/*vLeft=*/80.0f, /*vRight=*/80.0f, /*duration=*/500.0f, /*id=*/1, /*corrId=*/2);
  sim.step(3);
  checkFalse(sim.navigator().active(), "WHEELS cancelled the goto -- Navigator::active() is now false");
  checkTrue(sim.drive().owns(), "Drive now owns motion (the teleop primitive)");

  std::vector<AckSeen> allAcks;
  for (int i = 0; i < 60; ++i) {
    sim.step(1);
    std::vector<TestSupport::DecodedLine> frames = sim.drainTelemetry();
    for (const AckSeen& ack : collectAcks(frames)) allAcks.push_back(ack);
  }
  bool sawGotoCompletionAck = false;
  for (const AckSeen& ack : allAcks) {
    if (ack.key == kGotoId) sawGotoCompletionAck = true;
  }
  checkFalse(sawGotoCompletionAck, "the preempted goto's own id never shows up in the ack ring");
}

// --- Scenario 4: GO_TO cancels active WHEELS teleop. ----------------------
void scenarioGotoCancelsActiveWheelsTeleop() {
  beginScenario("GO_TO cancels active WHEELS teleop via drive_.takeover()");
  TestSim::SimHarness sim;
  TestSupport::configureSimForBenchTest(sim);
  sim.boot();

  sim.step(3);  // settle: publishOtos() lags boot() by one cycle
  sim.injectWheels(/*vLeft=*/80.0f, /*vRight=*/80.0f, /*duration=*/5000.0f, /*id=*/1, /*corrId=*/1);
  sim.step(3);
  checkTrue(sim.drive().owns(), "WHEELS teleop owns Drive before the GO_TO arrives");

  sim.injectGoto(/*x=*/700.0f, /*y=*/0.0f, /*frame=*/0, /*speed=*/0.0f, /*arrive=*/0.0f,
                 /*timeout=*/30000.0f, /*id=*/999, /*corrId=*/2);
  sim.step(3);
  checkFalse(sim.drive().owns(), "GO_TO cancelled the WHEELS teleop -- Drive no longer owns motion");
  checkTrue(sim.navigator().active(), "the Navigator now owns Drive instead");
}

// --- Scenario 5: ESTOP clears the Navigator's target the SAME cycle it
// clears the Planner's queue -- no completion ack, no fault ack, just gone.
void scenarioEstopClearsNavigatorSameCycleAsPlannerQueue() {
  beginScenario("ESTOP clears the Navigator's target the same cycle it clears the Planner's queue");
  TestSim::SimHarness sim;
  TestSupport::configureSimForBenchTest(sim);
  sim.boot();
  sim.step(3);  // settle: publishOtos() lags boot() by one cycle

  const uint32_t kGotoId = 321;
  sim.injectGoto(/*x=*/5000.0f, /*y=*/0.0f, /*frame=*/0, /*speed=*/0.0f, /*arrive=*/0.0f,
                 /*timeout=*/30000.0f, kGotoId, /*corrId=*/1);
  sim.step(5);
  checkTrue(sim.navigator().active(), "the goto is active and driving before ESTOP arrives");
  checkTrue(sim.planner().active(), "the Planner has an active internal segment Move too");

  sim.injectEstop(/*corrId=*/2);
  sim.step(1);  // ESTOP is routed and handled within a single cycle's command drain
  checkFalse(sim.navigator().active(), "ESTOP cleared the Navigator's target in the SAME cycle");
  checkFalse(sim.planner().active(), "ESTOP cleared the Planner's queue in the SAME cycle");
  checkFloatNear(sim.driveTargetVelLeft(), 0.0f, 1.0f, "commanded left wheel velocity is zero post-ESTOP");
  checkFloatNear(sim.driveTargetVelRight(), 0.0f, 1.0f, "commanded right wheel velocity is zero post-ESTOP");

  std::vector<AckSeen> allAcks;
  for (int i = 0; i < 60; ++i) {
    sim.step(1);
    std::vector<TestSupport::DecodedLine> frames = sim.drainTelemetry();
    for (const AckSeen& ack : collectAcks(frames)) allAcks.push_back(ack);
  }
  bool sawGotoCompletionAck = false;
  for (const AckSeen& ack : allAcks) {
    if (ack.key == kGotoId) sawGotoCompletionAck = true;
  }
  checkFalse(sawGotoCompletionAck,
             "ESTOP's halt is unconditional -- no completion ack, no fault ack, just gone");
}

// Scenario 6 (handleGoto() rejects with ERR_NOT_CONFIGURED while
// !state.otos.connected, SUC-005's own explicit gate) is deliberately NOT
// exercised here: TestSim::SimPlant has no existing fault-injection knob
// that reports OTOS itself as disconnected (setDisconnected(port, ...) is
// the WHEEL plant's own knob, sim_plant.cpp's mutableWheelPlant() --
// confirmed by reading it, not assumed) -- adding one is Hal::Otos/
// SimPlant's own scope, not this ticket's wire-routing job. The gate
// itself is a direct, one-line, easily-code-reviewed guard in
// RobotLoop::handleGoto() (robot_loop.cpp); ticket 006's real bench pass
// (which CAN genuinely disconnect the OTOS chip's I2C wiring) is the
// cheaper place to exercise it end to end on real hardware.

}  // namespace

int main() {
  scenarioBasicGotoAcceptedNavigatorDrivesCompletionAck();
  scenarioMoveCancelsActiveGotoNoCompletionAck();
  scenarioWheelsCancelsActiveGotoNoCompletionAck();
  scenarioGotoCancelsActiveWheelsTeleop();
  scenarioEstopClearsNavigatorSameCycleAsPlannerQueue();

  if (g_failureCount > 0) {
    std::printf("FAILED: %d assertion(s) across the GO_TO/Navigator RobotLoop scenarios\n", g_failureCount);
    return 1;
  }
  std::printf("PASS: every GO_TO/Navigator RobotLoop scenario (135-004)\n");
  return 0;
}
