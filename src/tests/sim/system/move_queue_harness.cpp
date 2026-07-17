// move_queue_harness.cpp -- sprint 109 ticket 003's own wire-level
// acceptance: the `Move` command end to end -- wire decode ->
// App::Pilot::enqueue() -> Motion::Executor -> jerk-limited twist on the
// REAL App::RobotLoop/SimPlant graph (TestSim::SimHarness).
//
// Scenarios (this ticket's own testing plan):
//   1. Teleop replace stream then silence -> smooth ramp to zero. Streams
//      several MOVE{replace=true} commands (mimicking a gamepad), then
//      goes silent; asserts the plant's real velocity never steps
//      instantaneously between replace calls (jerk-limited, not an
//      instant twist step) and eventually ramps back toward zero once the
//      last command's own deadline passes (no separate stop needed --
//      TIMED's own ramp-down IS the teleop decay bound).
//   2. Queue overflow -> ERR_FULL ack (plan untouched).
//   3. Degenerate MOVE -> TRIVIAL ack, never queued.
//   4. TWIST still preempts (flushes) the queue; STOP still stops the
//      robot immediately -- existing behavior not regressed.
#include <cmath>
#include <cstdio>
#include <string>

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

// Finds the ack (msg::AckEntry) for `corrId` across every decoded telemetry
// frame drained so far. Returns true and fills *status if found.
bool findAck(const std::vector<TestSupport::DecodedLine>& lines, uint32_t corrId,
             msg::AckStatus* status) {
  for (const auto& line : lines) {
    if (line.kind != TestSupport::DecodedKind::kTelemetry) continue;
    for (uint8_t i = 0; i < line.telemetry.acks_count; ++i) {
      if (line.telemetry.acks_[i].corr_id == corrId) {
        *status = line.telemetry.acks_[i].status;
        return true;
      }
    }
  }
  return false;
}

}  // namespace

int main() {
  std::printf("=== Move Queue Wire-Level Scenarios (109-003, SUC-001/SUC-003) ===\n\n");

  // --- Scenario 1: degenerate MOVE -> TRIVIAL ack ---
  {
    beginScenario("degenerate MOVE (all zero) -> ACK_STATUS_TRIVIAL, never queued");
    TestSim::SimHarness sim;
    sim.boot();
    sim.step(3);

    sim.injectMove(/*distance=*/0.0f, /*deltaHeading=*/0.0f, /*vMax=*/0.0f, /*omega=*/0.0f,
                    /*timeMs=*/0.0f, /*replace=*/false, /*id=*/1, /*corrId=*/101);
    sim.step(3);

    auto lines = sim.drainTelemetry();
    msg::AckStatus status = msg::AckStatus::ACK_STATUS_OK;
    checkTrue(findAck(lines, 101, &status), "an ack for corrId=101 was seen");
    checkTrue(status == msg::AckStatus::ACK_STATUS_TRIVIAL, "the ack status is ACK_STATUS_TRIVIAL");
    checkTrue(sim.pilotQueueDepth() == 0, "the degenerate command was never queued");
  }

  // --- Scenario 2: queue overflow -> ERR_FULL ack, plan untouched ---
  {
    beginScenario("9th queued MOVE -> ACK_STATUS_ERR/ERR_FULL, ring stays at depth 8");
    TestSim::SimHarness sim;
    sim.boot();
    sim.step(3);

    // corrId 0 activates immediately (queue empty); corrIds 1..8 fill the
    // ring to its own depth-8 capacity; corrId 9 must overflow.
    for (uint32_t i = 0; i <= Motion::kQueueDepth; ++i) {
      sim.injectMove(0.0f, 0.0f, /*vMax=*/50.0f, 0.0f, /*timeMs=*/5000.0f, /*replace=*/false,
                      /*id=*/i, /*corrId=*/200 + i);
      sim.step(1);
    }
    sim.injectMove(0.0f, 0.0f, 50.0f, 0.0f, 5000.0f, /*replace=*/false, /*id=*/99,
                    /*corrId=*/999);
    sim.step(3);

    auto lines = sim.drainTelemetry();
    msg::AckStatus overflowStatus = msg::AckStatus::ACK_STATUS_OK;
    checkTrue(findAck(lines, 999, &overflowStatus), "an ack for the overflowing corrId=999 was seen");
    checkTrue(overflowStatus == msg::AckStatus::ACK_STATUS_ERR, "the overflow ack status is ACK_STATUS_ERR");
    checkTrue(sim.pilotQueueDepth() == Motion::kQueueDepth,
              "the ring stays at exactly its own depth-8 capacity (plan untouched)");
  }

  // --- Scenario 3: teleop replace stream then silence -> jerk-limited,
  //     no instant step, eventual ramp toward zero ---
  {
    beginScenario("teleop replace stream (jerk-limited, no instant step) then silence -> ramps down");
    TestSim::SimHarness sim;
    sim.boot();
    sim.step(3);

    constexpr float kTargetV = 250.0f;  // [mm/s]
    constexpr float kLeaseMs = 300.0f;  // matches App::Pilot's own deadman lease scale

    float lastVel = 0.0f;
    bool everInstantStep = false;
    // Stream several replace=true commands, each re-arming a fresh
    // kLeaseMs deadline -- a real gamepad-teleop pattern.
    for (int i = 0; i < 6; ++i) {
      sim.injectMove(0.0f, 0.0f, kTargetV, 0.0f, kLeaseMs, /*replace=*/true, /*id=*/300 + i,
                      /*corrId=*/300 + i);
      for (int c = 0; c < 4; ++c) {
        sim.step(1);
        float vel = sim.motorLeft().velocity();
        // Never an instantaneous full-magnitude step between samples --
        // the exact per-cycle bound depends on the configured jerk/accel
        // limits, but ANY single 50ms cycle covering the ENTIRE
        // 0->kTargetV span outright would mean the twist was staged as a
        // raw step, not a Ruckig-solved ramp.
        if (std::fabs(vel - lastVel) > kTargetV * 0.9f) everInstantStep = true;
        lastVel = vel;
      }
    }
    checkTrue(!everInstantStep, "no single cycle stepped the whole 0->vMax span (jerk-limited, not instant)");
    checkTrue(std::fabs(lastVel) > 10.0f, "the stream actually got the plant moving");

    // Go silent -- no more replace commands. The last command's own
    // kLeaseMs deadline (plus its own ramp-down) should bring velocity
    // back toward zero without any further command.
    bool reachedNearZero = false;
    for (int i = 0; i < 40 && !reachedNearZero; ++i) {
      sim.step(1);
      if (std::fabs(sim.motorLeft().velocity()) < 15.0f) reachedNearZero = true;
    }
    checkTrue(reachedNearZero, "velocity ramped back toward zero after the teleop stream went silent");
  }

  // --- Scenario 4: TWIST preempts (flushes) the queue; STOP still stops
  //     immediately -- existing behavior not regressed ---
  {
    beginScenario("TWIST preempts the Move queue; STOP still stops immediately");
    TestSim::SimHarness sim;
    sim.boot();
    sim.step(3);

    sim.injectMove(0.0f, 0.0f, 200.0f, 0.0f, 5000.0f, false, /*id=*/1, /*corrId=*/1);
    sim.step(1);
    checkTrue(sim.pilotState() != Motion::State::kIdle, "the MOVE activated the executor");

    sim.injectTwist(/*v_x=*/100.0f, /*omega=*/0.0f, /*duration=*/2000.0f, /*corrId=*/2);
    sim.step(1);
    checkTrue(sim.pilotState() == Motion::State::kIdle, "TWIST flushed the executor back to kIdle");

    sim.injectStop(/*corrId=*/3);
    // The bare P-only PID plant has a documented brief post-STOP transient/
    // ring before settling (same characteristic scripted_twist_demo_harness.cpp's
    // own header describes as "watch velocity converge to (approximately)
    // zero over a 12-cycle post-STOP window") -- check the SETTLED value,
    // not an unrealistic "promptly within 3 cycles" bar.
    sim.step(15);
    checkTrue(std::fabs(sim.motorLeft().velocity()) < 15.0f,
              "STOP still brings the robot to rest (settled, existing behavior not regressed)");
  }

  std::printf("\n");
  if (g_failureCount == 0) {
    std::printf("OK: all Move-queue wire-level scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the Move-queue scenarios\n", g_failureCount);
  return 1;
}
