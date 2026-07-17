// motion_executor_harness.cpp -- off-hardware acceptance proof for sprint
// 109 ticket 003's Motion::Executor: the ring queue (depth 8), the TIMED
// state machine (activation -> jerk-limited ramp -> deadline-driven
// RAMP_TO_REST -> DONE), replace (tail-supersede and active-retarget), the
// degenerate/DISTANCE classification, and queue overflow.
//
// Drives Motion::Executor directly (no App::Pilot, no RobotLoop, no wire)
// -- a pure logic test of the queue/state-machine/solve-budget contract,
// mirroring jerk_trajectory_harness.cpp's own "compile+run, hand-rolled
// PASS/FAIL, nonzero exit on failure" shape. Compiled together with
// motion/jerk_trajectory.cpp and the vendored Ruckig sources by
// test_motion_executor.py, exactly like test_jerk_trajectory.py's own
// jerk_trajectory_harness.cpp precedent.
#include <cmath>
#include <cstdio>
#include <string>

#include "motion/executor.h"

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

msg::PlannerConfig makeConfig() {
  msg::PlannerConfig cfg;
  cfg.a_max = 800.0f;
  cfg.a_decel = 1000.0f;
  cfg.v_body_max = 600.0f;
  cfg.yaw_rate_max = 4.0f;
  cfg.yaw_acc_max = 20.0f;
  cfg.j_max = 8000.0f;
  cfg.yaw_jerk_max = 80.0f;
  return cfg;
}

Motion::Cmd makeTimedCmd(float vMax, float omega, float timeMs, uint32_t id,
                         bool replace = false) {
  Motion::Cmd cmd;
  cmd.vMax = vMax;
  cmd.omega = omega;
  cmd.time = timeMs;
  cmd.id = id;
  cmd.replace = replace;
  return cmd;
}

constexpr uint32_t kDtMs = 40;

}  // namespace

int main() {
  std::printf("=== Motion::Executor Scenarios (109-003, SUC-001/SUC-003) ===\n\n");

  // --- Scenario 1: degenerate Move -> TRIVIAL, never queued ---
  {
    beginScenario("degenerate command (zero distance+heading, time<=0) -> kTrivial, never queued");
    Motion::Executor exec;
    exec.configure(makeConfig());
    Motion::Cmd degenerate;  // all-zero defaults: distance=0, deltaHeading=0, time=0
    auto outcome = exec.enqueue(degenerate);
    checkTrue(outcome == Motion::EnqueueOutcome::kTrivial, "enqueue() returned kTrivial");
    checkTrue(exec.state() == Motion::State::kIdle, "executor stayed kIdle");
    checkTrue(exec.queueDepth() == 0, "nothing was queued");
  }

  // --- Scenario 2: DISTANCE mode (time<=0, non-degenerate) -> kUnimplemented ---
  {
    beginScenario("DISTANCE-mode command (time<=0, nonzero distance) -> kUnimplemented this ticket");
    Motion::Executor exec;
    exec.configure(makeConfig());
    Motion::Cmd distanceCmd;
    distanceCmd.distance = 500.0f;
    auto outcome = exec.enqueue(distanceCmd);
    checkTrue(outcome == Motion::EnqueueOutcome::kUnimplemented, "enqueue() returned kUnimplemented");
    checkTrue(exec.state() == Motion::State::kIdle, "executor stayed kIdle");
  }

  // --- Scenario 3: fresh TIMED activation ramps jerk-limited (no instant
  //     step), reaches the deadline, and completes DONE ---
  {
    beginScenario("TIMED command: jerk-limited ramp, deadline-driven RAMP_TO_REST, DONE");
    Motion::Executor exec;
    exec.configure(makeConfig());

    constexpr float kVMax = 300.0f;    // [mm/s]
    constexpr float kTimeMs = 2000.0f;  // [ms] total duration from activation
    auto outcome = exec.enqueue(makeTimedCmd(kVMax, 0.0f, kTimeMs, /*id=*/7));
    checkTrue(outcome == Motion::EnqueueOutcome::kAccepted, "fresh TIMED enqueue -> kAccepted (activates immediately)");
    checkTrue(exec.state() == Motion::State::kRunning, "activated into kRunning");
    checkTrue(exec.activeId() == 7, "activeId() reports the new command's id");

    float firstSampleV = 0.0f;
    float peakV = 0.0f;
    bool sawDone = false;
    Motion::CompletionEvent doneEvent;

    // Run well past the 2000ms deadline (50 cycles * 40ms = 2000ms, give a
    // margin) so RAMP_TO_REST's own decel window has room to complete.
    for (int i = 0; i < 80 && !sawDone; ++i) {
      exec.plan();  // <=1 solve/cycle -- may take 2 cycles to plan both channels
      Motion::Executor::Twist twist = exec.tick(kDtMs);
      if (i == 0) firstSampleV = twist.v;
      peakV = std::max(peakV, std::fabs(twist.v));

      Motion::CompletionEvent event;
      while (exec.popEvent(&event)) {
        if (event.status == Motion::CompletionStatus::kDone) {
          sawDone = true;
          doneEvent = event;
        }
      }
    }

    // Jerk-limited ramp: the very first 40ms sample must be far below
    // vMax (an instantaneous step would read ~kVMax on sample 0; a
    // jerk-limited ramp under these limits can cover at most a few tens of
    // mm/s in one 40ms tick).
    checkTrue(std::fabs(firstSampleV) < kVMax * 0.5f,
              "first sampled velocity is well below vMax (ramp, not an instant step)");
    checkTrue(peakV > kVMax * 0.8f, "the command actually reached close to its commanded vMax");
    checkTrue(sawDone, "a kDone completion event was eventually popped");
    checkTrue(doneEvent.id == 7, "the kDone event echoes the command's own id");
    checkTrue(exec.state() == Motion::State::kIdle, "executor returned to kIdle after completion");
  }

  // --- Scenario 4: ring queue depth + overflow ---
  {
    beginScenario("ring queue: 8 pending commands admitted, 9th -> kFull, plan untouched");
    Motion::Executor exec;
    exec.configure(makeConfig());

    // First enqueue activates immediately (queue empty, kIdle) -- the
    // remaining 8 fill the ring to its own stated depth.
    exec.enqueue(makeTimedCmd(100.0f, 0.0f, 5000.0f, /*id=*/0));
    checkTrue(exec.state() == Motion::State::kRunning, "first command activated immediately");

    for (uint32_t id = 1; id <= Motion::kQueueDepth; ++id) {
      auto outcome = exec.enqueue(makeTimedCmd(100.0f, 0.0f, 5000.0f, id));
      checkTrue(outcome == Motion::EnqueueOutcome::kAccepted,
                "queue slot " + std::to_string(id) + " accepted");
    }
    checkTrue(exec.queueDepth() == Motion::kQueueDepth, "ring reports full depth");

    auto overflowOutcome = exec.enqueue(makeTimedCmd(100.0f, 0.0f, 5000.0f, /*id=*/99));
    checkTrue(overflowOutcome == Motion::EnqueueOutcome::kFull, "9th queued command -> kFull");
    checkTrue(exec.queueDepth() == Motion::kQueueDepth, "plan untouched -- depth still at capacity");
  }

  // --- Scenario 5: replace -- tail supersede ---
  {
    beginScenario("replace=true with a non-empty queue supersedes the ring's own tail");
    Motion::Executor exec;
    exec.configure(makeConfig());

    exec.enqueue(makeTimedCmd(100.0f, 0.0f, 5000.0f, /*id=*/0));  // activates
    exec.enqueue(makeTimedCmd(150.0f, 0.0f, 5000.0f, /*id=*/1));  // queued (tail)

    auto outcome = exec.enqueue(makeTimedCmd(200.0f, 0.0f, 5000.0f, /*id=*/2, /*replace=*/true));
    checkTrue(outcome == Motion::EnqueueOutcome::kReplaced, "replace against a queued tail -> kReplaced");
    checkTrue(exec.queueDepth() == 1, "ring depth unchanged (replaced in place, not appended)");

    bool sawSuperseded = false;
    Motion::CompletionEvent event;
    while (exec.popEvent(&event)) {
      if (event.status == Motion::CompletionStatus::kSuperseded && event.id == 1) sawSuperseded = true;
    }
    checkTrue(sawSuperseded, "the superseded tail command (id=1) got a kSuperseded event");
  }

  // --- Scenario 6: replace -- active in-place retarget (no instant step) ---
  {
    beginScenario("replace=true with an empty queue retargets the ACTIVE command in place");
    Motion::Executor exec;
    exec.configure(makeConfig());

    exec.enqueue(makeTimedCmd(100.0f, 0.0f, 5000.0f, /*id=*/0));
    for (int i = 0; i < 10; ++i) {
      exec.plan();
      exec.tick(kDtMs);
    }
    float vBeforeReplace = exec.tick(0).v;  // re-sample at the same elapsed time, no advance

    auto outcome = exec.enqueue(makeTimedCmd(400.0f, 0.0f, 5000.0f, /*id=*/1, /*replace=*/true));
    checkTrue(outcome == Motion::EnqueueOutcome::kReplaced, "replace against the active command -> kReplaced");
    checkTrue(exec.activeId() == 1, "activeId() now reports the replacing command's id");

    // Immediately after the retarget, the very next sample must still be
    // close to the PRE-replace velocity (a smooth in-place retarget,
    // seeded from the channel's own last sample -- never an instant jump
    // to the new 400mm/s target).
    exec.plan();
    float vAfterReplace = exec.tick(kDtMs).v;
    checkTrue(std::fabs(vAfterReplace - vBeforeReplace) < 50.0f,
              "the sample immediately after replace() is close to the pre-replace velocity (no instant step)");

    bool sawSupersededActive = false;
    Motion::CompletionEvent event;
    while (exec.popEvent(&event)) {
      if (event.status == Motion::CompletionStatus::kSuperseded && event.id == 0) sawSupersededActive = true;
    }
    checkTrue(sawSupersededActive, "the superseded active command (id=0) got a kSuperseded event");
  }

  // --- Scenario 7: flush() (TWIST/STOP preemption) ---
  {
    beginScenario("flush() empties the ring and clears the active command, both FLUSHED");
    Motion::Executor exec;
    exec.configure(makeConfig());

    exec.enqueue(makeTimedCmd(100.0f, 0.0f, 5000.0f, /*id=*/0));  // active
    exec.enqueue(makeTimedCmd(100.0f, 0.0f, 5000.0f, /*id=*/1));  // queued

    exec.flush();
    checkTrue(exec.state() == Motion::State::kIdle, "flush() returns to kIdle");
    checkTrue(exec.queueDepth() == 0, "flush() empties the ring");

    int flushedCount = 0;
    Motion::CompletionEvent event;
    while (exec.popEvent(&event)) {
      if (event.status == Motion::CompletionStatus::kFlushed) ++flushedCount;
    }
    checkTrue(flushedCount == 2, "both the active AND the queued command got a kFlushed event");
  }

  std::printf("\n");
  if (g_failureCount == 0) {
    std::printf("OK: all Motion::Executor scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the Motion::Executor scenarios\n", g_failureCount);
  return 1;
}
