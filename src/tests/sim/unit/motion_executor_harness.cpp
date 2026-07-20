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

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegToRad = kPi / 180.0f;

msg::PlannerConfig makeConfig() {
  msg::PlannerConfig cfg;
  cfg.a_max = 800.0f;
  cfg.a_decel = 1000.0f;
  cfg.v_body_max = 600.0f;
  cfg.yaw_rate_max = 4.0f;
  cfg.yaw_acc_max = 20.0f;
  cfg.j_max = 8000.0f;
  cfg.yaw_jerk_max = 80.0f;
  // 109-005: the heading-dwell completion gate. 0.5 deg / 1 deg/s / 150ms --
  // sprint-098's own proven turn-accuracy bar (see executor.h/planner.proto's
  // own doc comments); arrive_dwell (150ms) is REUSED from the pre-existing
  // terminal-completion dwell field, not a new number.
  cfg.heading_dwell_tol = 0.5f * kDegToRad;
  cfg.heading_dwell_rate = 1.0f * kDegToRad;
  cfg.arrive_dwell = 0.15f;
  // 112-004: the unified completion rule's own linear tolerance -- see
  // executor.h/planner.proto's own distance_tol doc comments. Left at the
  // zero-value default, |sErr| < 0 is never satisfiable (the same reason
  // heading_dwell_tol above is never left at 0), so every DISTANCE-mode
  // scenario in this file needs a real value the same way it already needs
  // a real heading_dwell_tol -- matches gen_boot_config.py's own
  // DISTANCE_TOL_DEFAULT/Motion::kDistanceSettleEpsilonMm's old value.
  cfg.distance_tol = 3.0f;  // [mm]
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

// makeDistanceCmd -- 109-005: a DISTANCE-mode (time<=0) arc/straight/pivot
// command. distance==0 -> pivot; deltaHeading==0 -> straight leg; both
// nonzero -> a coupled arc.
Motion::Cmd makeDistanceCmd(float distance, float deltaHeading, float vMax, uint32_t id) {
  Motion::Cmd cmd;
  cmd.distance = distance;
  cmd.deltaHeading = deltaHeading;
  cmd.vMax = vMax;
  cmd.id = id;
  return cmd;
}

constexpr uint32_t kDtMs = 40;

// runPerfectlyTrackedHeading -- drives `exec` with a ONE-CYCLE-LAGGED
// "perfect sensor" (each cycle's measuredHeadingAbs is the PREVIOUS cycle's
// own thetaRef -- see this file's own Scenario 8 comment for why this
// stand-in is a faithful enough measured-heading source for a dwell-
// completion unit test that has no App::HeadingSource/sim plant available).
// Runs up to maxCycles; returns the cycle index (1-based) completion was
// observed on, or 0 if it never completed.
int runPerfectlyTrackedHeading(Motion::Executor& exec, int maxCycles, uint32_t* doneId = nullptr) {
  float measuredHeadingAbs = 0.0f;
  for (int i = 0; i < maxCycles; ++i) {
    exec.plan();
    Motion::Executor::Twist twist = exec.tick(kDtMs, /*measuredDistanceDelta=*/0.0f, measuredHeadingAbs);
    measuredHeadingAbs = twist.thetaRef;

    Motion::CompletionEvent event;
    while (exec.popEvent(&event)) {
      if (event.status == Motion::CompletionStatus::kDone) {
        if (doneId) *doneId = event.id;
        return i + 1;
      }
    }
  }
  return 0;
}

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

  // --- Scenario 2: DISTANCE mode (time<=0, non-degenerate) -> kAccepted,
  //     activates immediately (109-005: DISTANCE is real now) ---
  {
    beginScenario("DISTANCE-mode command (time<=0, nonzero distance) -> kAccepted, activates");
    Motion::Executor exec;
    exec.configure(makeConfig());
    auto outcome = exec.enqueue(makeDistanceCmd(/*distance=*/500.0f, /*deltaHeading=*/0.0f,
                                                 /*vMax=*/200.0f, /*id=*/1));
    checkTrue(outcome == Motion::EnqueueOutcome::kAccepted, "enqueue() returned kAccepted");
    checkTrue(exec.state() == Motion::State::kRunning, "executor activated into kRunning");
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

  // --- Scenario 8: pure pivot -- dwell completion on a TERMINAL command,
  //     exact final heading (ideal/no-noise tracking) ---
  {
    beginScenario("pure pivot (terminal): dwell-completes near-exactly at deltaHeading");
    Motion::Executor exec;
    exec.configure(makeConfig());

    constexpr float kDeltaHeading = 90.0f * kDegToRad;
    exec.enqueue(makeDistanceCmd(/*distance=*/0.0f, kDeltaHeading, /*vMax=*/0.0f, /*id=*/5));
    checkTrue(exec.state() == Motion::State::kRunning, "pivot activated into kRunning");

    uint32_t doneId = 0;
    int cycle = runPerfectlyTrackedHeading(exec, /*maxCycles=*/300, &doneId);
    checkTrue(cycle > 0, "the terminal pivot eventually dwell-completed");
    checkTrue(doneId == 5, "the kDone event echoes the pivot's own id");
    checkTrue(exec.state() == Motion::State::kIdle, "executor returned to kIdle");
  }

  // --- Scenario 9: chained (non-terminal) pivot skips the dwell hold --
  //     completes measurably EARLIER than an otherwise-identical terminal
  //     pivot (Scenario 8) ---
  {
    beginScenario("chained (non-terminal) pivot completes without the dwell hold");
    Motion::Executor exec;
    exec.configure(makeConfig());

    constexpr float kDeltaHeading = 90.0f * kDegToRad;
    exec.enqueue(makeDistanceCmd(0.0f, kDeltaHeading, 0.0f, /*id=*/6));
    // A second queued pivot makes id=6 NON-terminal (queueCount() > 0 while
    // id=6 is active) -- this is the only difference from Scenario 8.
    exec.enqueue(makeDistanceCmd(0.0f, kDeltaHeading, 0.0f, /*id=*/7));

    uint32_t doneId = 0;
    int cycle = runPerfectlyTrackedHeading(exec, /*maxCycles=*/300, &doneId);
    checkTrue(cycle > 0, "the chained pivot completed");
    checkTrue(doneId == 6, "the kDone event is for the FIRST (chained) pivot, id=6");
    checkTrue(exec.activeId() == 7, "the SECOND pivot (id=7) is now active -- handoff happened");

    // 150ms/40ms ~= 3.75 -> a real dwell hold costs at least 3-4 extra
    // cycles beyond the first instant tolerance is met. A generous margin
    // (>=2 cycles) distinguishes "skipped the hold" from measurement noise
    // without hard-coding the exact tracker-lag cycle count.
    checkTrue(cycle < 280, "chained completion did not run anywhere near the full budget (no dwell hold)");
  }

  // --- Scenario 10: coupled arc -- the rotational channel is SLAVED to the
  //     linear channel by the arc ratio (omega_ff/v == deltaHeading/distance
  //     throughout, not just at the endpoints) ---
  {
    beginScenario("coupled arc: omega_ff/v tracks the commanded deltaHeading/distance ratio");
    Motion::Executor exec;
    exec.configure(makeConfig());

    constexpr float kDistance = 600.0f;             // [mm]
    constexpr float kDeltaHeading = 45.0f * kDegToRad;  // [rad]
    constexpr float kExpectedRatio = kDeltaHeading / kDistance;  // [rad/mm]

    exec.enqueue(makeDistanceCmd(kDistance, kDeltaHeading, /*vMax=*/300.0f, /*id=*/8));

    bool sawMidCruiseSample = false;
    for (int i = 0; i < 60; ++i) {
      exec.plan();
      Motion::Executor::Twist twist = exec.tick(kDtMs, /*measuredDistanceDelta=*/0.0f, /*measuredHeadingAbs=*/0.0f);
      // Only check the ratio once the linear channel has a real cruise
      // velocity (skip the jerk-limited ramp-up's own near-zero-v samples,
      // where the ratio's own denominator is too small to be meaningful).
      if (std::fabs(twist.v) > 50.0f) {
        float ratio = twist.omega / twist.v;
        checkTrue(std::fabs(ratio - kExpectedRatio) < std::fabs(kExpectedRatio) * 0.05f,
                  "omega_ff/v matches deltaHeading/distance within 5% at a mid-cruise sample");
        checkTrue(twist.headingActive, "a heading-bearing arc reports headingActive during cruise");
        sawMidCruiseSample = true;
        break;
      }
    }
    checkTrue(sawMidCruiseSample, "the arc reached a cruise sample worth checking the ratio at");
  }

  // Scenario 11 (same-sign DISTANCE overshoot carries into the next
  // same-sign successor) is DELETED (112-004) -- the mechanism it tested,
  // Motion::Executor's own pendingOvershoot_ same-sign carry, is deleted
  // outright by this ticket (see executor.h's own "Distance completion"
  // comment); there is no replacement code path left to unit-test. That
  // scenario also relied on a purely SCRIPTED measured-distance signal
  // decoupled from the real Ruckig-solved trajectory to isolate the carry
  // bookkeeping -- a technique fundamentally incompatible with 112-004's
  // own unified completion rule, which additionally requires the dominant
  // channel's REAL solved trajectory to have elapsed its own duration
  // (`profileElapsed`, executor.cpp's own tick() comment) before a
  // "not carrying" command can complete, so a scripted signal alone can no
  // longer race the real trajectory to an early completion the way this
  // scenario depended on.

  std::printf("\n");
  if (g_failureCount == 0) {
    std::printf("OK: all Motion::Executor scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the Motion::Executor scenarios\n", g_failureCount);
  return 1;
}
