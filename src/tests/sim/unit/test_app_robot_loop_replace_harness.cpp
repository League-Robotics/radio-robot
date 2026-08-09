// test_app_robot_loop_replace_harness.cpp -- 127-001's own off-hardware
// acceptance proof: characterizes the firmware's `Move` `replace=True`
// preemption path against the five cases the sprint 127 design issue's
// investigation identified (clasi/sprints/127-host-side-path-planner-
// goto-and-path-following/issues/sprint-127-host-side-path-planner-goto-
// path-following.md, "Finding 1"), plus a duplicate-id sanity smoke check.
//
// CHARACTERIZATION, NOT A FIX: this file writes tests against existing
// firmware behavior. It does not modify src/motion/planner/planner.cpp,
// src/firm/app/robot_loop.cpp, any wire message, or any .proto definition
// -- planner.cpp's axis-carry logic (handleMove()'s activateNext() call
// path, ~planner.cpp:723-729), axisOf() (~:835-843), and the
// axisPerLambda conversion in planWheels() (~:1075) are read-only
// references here, exactly as the ticket requires.
//
// Two tiers of construction, deliberately different, each matched to what
// it needs to observe:
//
//   Cases 1, 2, 5 (scenarioSameCurvature.../scenarioHighRateReplacement)
//   drive a bare Motion::Planner directly (TestPlanner::benchLimits() +
//   TestPlanner::PerfectPlant/NoisyPlant, src/motion/planner/tests/
//   test_support.h -- the SAME zero-Python, no-RobotLoop scaffolding
//   src/motion/planner/tests/planner_scenarios_test.cpp's own
//   testReplacePreempts() already uses for the "does it preempt at all"
//   smoke case). This is deliberate, not a shortcut: Finding 1's
//   profileVelocity_ axis-unit carry is internal to Planner::move()/tick(),
//   and reaching it through the full wire codec/RobotLoop graph would add
//   nothing but noise. It also matters that these run under REALISTIC,
//   finite accel/decel ceilings (TestPlanner::benchLimits(), the same
//   bench-plausible limits the sketch-verification suite already uses)
//   rather than TestSim::SimHarness's own "effectively unshaped" sim
//   defaults (aMax ~1e6, src/firm/platform/host/sim_harness.h's simPlannerLimits()) -- an
//   unshaped ceiling would let the profiler jump straight to cruise in one
//   step regardless of the carried-velocity mismatch, hiding exactly the
//   discontinuity ticket 005 needs measured.
//
//   130-005 REMOVAL NOTE: Finding 1's SECOND edge -- the WheelTrim
//   integrator's phase-gated (not replace-gated) reset surviving a
//   mid-motion axis change -- was characterized here as cases 3/4
//   (scenarioEdgeAAxisChangeAtSpeed/scenarioAxisChangeFromRestSanity,
//   `planner.applyTrimGains()`/`trimIntegralLeft()`/`trimLeft()`/
//   `trimRight()`). Per wheel-speed-controller-moves-into-drive.md Phase 3
//   (DECIDED, stakeholder 2026-08-01), Motion::WheelTrim is deleted
//   outright -- Motion::Planner carries no wheel-actuation state of any
//   kind any more, so there is no integral left for a replace to disturb,
//   and the question those two cases characterized no longer has a
//   subject. Removed together (case 4 measured itself against case 3's
//   own peak, so neither survives alone) rather than repointed: the
//   equivalent question for App::Drive's OWN bias/fast-PID state
//   surviving a Move replace is a different scenario, against a
//   different (RobotLoop-level) harness tier, and is not part of this
//   ticket's acceptance criteria -- see app_drive_harness.cpp for Drive's
//   own Stage B/C coverage instead.
//
//   The duplicate-id sanity check (scenarioDuplicateIdSanityNoOp) drives
//   the real TestSim::SimHarness (src/firm/platform/host/sim_harness.h) instead, because
//   the dedup short-circuit it exercises is NOT inside Motion::Planner at
//   all -- it lives one layer up, in App::RobotLoop::handleMove()
//   (alreadyAccepted()/recordAccepted(), src/firm/app/robot_loop.cpp
//   ~216-247), and a bare Planner has no id memory whatsoever to exercise.
//
// Mirrors this tier's established hand-rolled assertion plumbing (see
// motion_stop_condition_harness.cpp / app_robot_loop_harness.cpp): compile
// with the system C++ compiler, run the resulting binary, assert it exits
// 0. Collected under src/tests/sim/unit/ -- already within pyproject.toml's
// testpaths = ["src/tests/sim"].
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include "devices/device_config.h"
#include "motion/planner/planner.h"
#include "motion/planner/tests/test_support.h"
#include "sim_harness.h"
#include "support/bench_test_config.h"
#include "types/robot_state.h"

namespace {

using Motion::Move;
using Motion::MovePhase;
using Motion::Planner;
using TestPlanner::benchLimits;
using TestPlanner::cycle;
using TestPlanner::PerfectPlant;

// --- Hand-rolled assertion plumbing (see motion_stop_condition_harness.cpp /
// app_robot_loop_harness.cpp -- this tier's established convention:
// accumulate every failure and keep running, rather than CHECK()'s
// exit-on-first-failure, so one scenario's report is complete). ---------

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

void checkUintEq(uint32_t actual, uint32_t expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %u, got %u", what.c_str(),
                  static_cast<unsigned>(expected), static_cast<unsigned>(actual));
    fail(buf);
  }
}

void checkFloatLe(float actual, float bound, const std::string& what) {
  if (!(actual <= bound)) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected <= %g, got %g", what.c_str(),
                  static_cast<double>(bound), static_cast<double>(actual));
    fail(buf);
  }
}

// --- Shared scenario helpers ------------------------------------------

constexpr float kPeriod = 50.0f;  // [ms] matches TestPlanner::benchLimits()'s own controlPeriod -- 20 Hz

Move distanceTwistMove(uint32_t id, float threshold, float v_x, float omega = 0.0f,
                        float timeout = 60000.0f) {
  Move m;
  m.id = id;
  m.kind = Move::Kind::Distance;
  m.threshold = threshold;
  m.v_x = v_x;
  m.omega = omega;
  m.timeout = timeout;
  return m;
}

Move angleTwistMove(uint32_t id, float threshold, float omega, float timeout = 60000.0f) {
  Move m;
  m.id = id;
  m.kind = Move::Kind::Angle;
  m.threshold = threshold;
  m.omega = omega;
  m.timeout = timeout;
  return m;
}

// active() + pendingCount() -- the protocol-v5 5-deep queue's own total
// occupancy (planner.h's own kQueueDepth comment: "1 active + 4 pending").
int queueDepth(const Planner& p) { return (p.active() ? 1 : 0) + p.pendingCount(); }

// ===========================================================================
// Case 1 -- same-curvature-at-speed (baseline). Replace a Linear move with
// another Linear move at the SAME unitLeft/unitRight ratio while at speed.
// Expect profile continuity -- no discontinuity beyond measurement noise.
// ===========================================================================
void scenarioSameCurvatureAtSpeedBaseline() {
  beginScenario("Case 1: same-curvature-at-speed (baseline) -- replace with a matching ratio, "
                "expect profile continuity");
  Planner planner(benchLimits());
  PerfectPlant plant;
  Types::RobotState state;
  uint32_t now = 0;

  const float kSpeed = 150.0f;  // [mm/s]
  CHECK(planner.move(distanceTwistMove(1, 5000.0f, kSpeed), false));
  for (int i = 0; i < 60; ++i) cycle(planner, state, plant, now, kPeriod);
  checkTrue(planner.phase() == MovePhase::Hold, "Move A reaches Hold (cruise) before the replace");

  const float leftBefore = planner.commandedLeft();
  const float rightBefore = planner.commandedRight();

  CHECK(planner.move(distanceTwistMove(2, 5000.0f, kSpeed), true));  // same ratio (straight), replace=true
  cycle(planner, state, plant, now, kPeriod);  // the tick that activates the replacement

  const float leftAfter = planner.commandedLeft();
  const float rightAfter = planner.commandedRight();
  const float discontinuity =
      std::max(std::fabs(leftAfter - leftBefore), std::fabs(rightAfter - rightBefore));  // [mm/s]

  std::printf("  CASE1_SAME_CURVATURE_DISCONTINUITY_MM_S=%.4f (left %.4f -> %.4f, right %.4f -> %.4f)\n",
              static_cast<double>(discontinuity), static_cast<double>(leftBefore),
              static_cast<double>(leftAfter), static_cast<double>(rightBefore),
              static_cast<double>(rightAfter));
  // Noise floor: generous relative to float roundoff, far tighter than any
  // ratio difference a host would deliberately command.
  checkFloatLe(discontinuity, 1.0f, "same-curvature replace stays within the noise floor (<= 1 mm/s)");
}

// ===========================================================================
// Case 2 (Edge B) -- large-curvature-step-at-speed. Replace a straight
// (axisPerLambda = 1.0) with a tight arc (unitLeft=-0.5, unitRight=1.0,
// axisPerLambda = 0.25) while at speed. THE measurement ticket 005 consumes
// to size its curvature slew limit.
// ===========================================================================
void scenarioEdgeBLargeCurvatureStepAtSpeed() {
  beginScenario("Case 2 (Edge B): large-curvature-step-at-speed -- straight replaced by a tight arc "
                "while cruising");
  Planner planner(benchLimits());
  PerfectPlant plant;
  Types::RobotState state;
  uint32_t now = 0;
  const float trackWidth = benchLimits().plant.trackWidth;  // [mm]

  const float kSpeed = 150.0f;  // [mm/s] -- the design issue's own worked example
  CHECK(planner.move(distanceTwistMove(10, 5000.0f, kSpeed), false));
  for (int i = 0; i < 60; ++i) cycle(planner, state, plant, now, kPeriod);
  checkTrue(planner.phase() == MovePhase::Hold, "straight Move A reaches Hold (cruise) before the replace");

  const float leftBefore = planner.commandedLeft();
  const float rightBefore = planner.commandedRight();

  // Tight arc: unitLeft=-0.5, unitRight=1.0 (axisPerLambda=0.25) at the SAME
  // dominant-wheel peak speed (150 mm/s) the straight was cruising at --
  // vRight = 150 (dominant, peak), vLeft = -75. v_x/omega derived from the
  // differential-drive decomposition shape.cpp itself uses (vLeft =
  // v_x - omega*trackWidth/2, vRight = v_x + omega*trackWidth/2).
  const float vRight = kSpeed;
  const float vLeft = -0.5f * kSpeed;
  const float arcVx = 0.5f * (vLeft + vRight);              // [mm/s]
  const float arcOmega = (vRight - vLeft) / trackWidth;     // [rad/s]
  Move arc = distanceTwistMove(11, 200.0f, arcVx, arcOmega);
  CHECK(planner.move(arc, true));
  cycle(planner, state, plant, now, kPeriod);  // the tick that activates the replacement

  const float leftAfter = planner.commandedLeft();
  const float rightAfter = planner.commandedRight();
  const float discontinuityLeft = std::fabs(leftAfter - leftBefore);    // [mm/s]
  const float discontinuityRight = std::fabs(rightAfter - rightBefore);  // [mm/s]
  const float discontinuity = std::max(discontinuityLeft, discontinuityRight);  // [mm/s]

  // ===== THE labeled number ticket 005 consumes (sprint.md Decision 4). =====
  std::printf(
      "  CASE2_EDGE_B_DISCONTINUITY_MM_S=%.4f  (left %.4f -> %.4f [d=%.4f], right %.4f -> %.4f [d=%.4f])\n",
      static_cast<double>(discontinuity), static_cast<double>(leftBefore), static_cast<double>(leftAfter),
      static_cast<double>(discontinuityLeft), static_cast<double>(rightBefore),
      static_cast<double>(rightAfter), static_cast<double>(discontinuityRight));
  // ============================================================================

  // No pass/fail threshold on the size of the discontinuity itself -- there
  // is no firmware fix in this ticket's scope to make it smaller (that is
  // ticket 005's job, sizing a HOST-side curvature slew limit against this
  // very number). The only structural assertions here: the measurement is
  // real (finite) and it is a genuine step, not accidentally absorbed to
  // ~0 by this scenario's own setup.
  checkTrue(std::isfinite(discontinuity), "Edge B discontinuity is a finite measurement");
  checkTrue(discontinuity > 10.0f,
            "Edge B discontinuity is a REAL step (> 10 mm/s), not accidentally near-zero -- confirms "
            "the design issue's own reasoning (carried profileVelocity_ / new axisPerLambda != the "
            "wheel's actual measured speed) is observable, not just theoretical");
}

// Cases 3/4 (axis-change-at-speed / axis-change-from-rest), which
// characterized Motion::WheelTrim's phase-gated integrator surviving a
// mid-motion replace, are REMOVED (130-005) -- see this file's own header
// note for why: the trim they characterized is deleted outright, and
// Motion::Planner carries no wheel-actuation state left for a replace to
// disturb.

// ===========================================================================
// Case 5 -- high-rate replacement (~20 Hz for >=5s). Issue replace=True
// twists faster than the ~1-tick activation latency. Measures the largest
// step between consecutive commanded wheel velocities, and confirms the
// planner queue depth never exceeds 1 -- no runaway growth from
// replacements arriving faster than they drain.
// ===========================================================================
void scenarioHighRateReplacement() {
  beginScenario("Case 5: high-rate (~20 Hz) replacement for >=5s -- largest inter-command step + queue "
                "depth never exceeds 1");
  Planner planner(benchLimits());
  PerfectPlant plant;
  Types::RobotState state;
  uint32_t now = 0;

  // kPeriod (50ms) IS 20 Hz -- replacing every single cycle is exactly the
  // "faster than the ~1-tick activation latency" case the ticket asks for.
  const int kCycles = 110;  // 110 * 50ms = 5.5s
  float maxStep = 0.0f;     // [mm/s]
  int maxQueueDepth = 0;
  float prevLeft = 0.0f;
  float prevRight = 0.0f;
  uint32_t moveId = 100;
  for (int i = 0; i < kCycles; ++i) {
    // A small curvature wobble each cycle -- plausible per-cycle correction
    // traffic a host-side path planner's outer loop would actually emit
    // (a small omega change at roughly constant speed), not an adversarial
    // huge step every single tick (that is case 2's own, isolated job).
    const float omega = 0.3f * std::sin(static_cast<float>(i) * 0.2f);  // [rad/s]
    CHECK(planner.move(distanceTwistMove(moveId++, 5000.0f, 150.0f, omega), true));
    maxQueueDepth = std::max(maxQueueDepth, queueDepth(planner));
    cycle(planner, state, plant, now, kPeriod);
    maxQueueDepth = std::max(maxQueueDepth, queueDepth(planner));

    const float left = planner.commandedLeft();
    const float right = planner.commandedRight();
    if (i > 0) {
      maxStep = std::max(maxStep, std::max(std::fabs(left - prevLeft), std::fabs(right - prevRight)));
    }
    prevLeft = left;
    prevRight = right;
  }

  // ===== THE labeled numbers: case 5's own measurements. =====
  std::printf("  CASE5_HIGH_RATE_MAX_STEP_MM_S=%.4f  CASE5_HIGH_RATE_MAX_QUEUE_DEPTH=%d  "
              "(ran %d cycles / %.1fs at 20 Hz)\n",
              static_cast<double>(maxStep), maxQueueDepth, kCycles,
              static_cast<double>(kCycles) * (kPeriod * 0.001f));
  // =============================================================

  checkTrue(std::isfinite(maxStep), "max inter-command step is a finite measurement");
  checkTrue(maxQueueDepth <= 1,
            "planner queue depth never exceeds 1 across the whole 20 Hz replacement run -- no runaway "
            "growth from replacements arriving faster than they drain");
}

// ===========================================================================
// Duplicate-id sanity check (precondition smoke check only -- the FULL
// four-rule dedup verification contract is ticket 002's scope). Resends an
// already-accepted Move.id and confirms no additional plan change results.
// This is App::RobotLoop::handleMove()'s own dedup short-circuit
// (alreadyAccepted()/recordAccepted(), robot_loop.cpp ~216-247) -- NOT
// inside Motion::Planner at all, so this scenario drives the real
// TestSim::SimHarness (the RobotLoop graph), unlike cases 1-5 above.
// ===========================================================================
void scenarioDuplicateIdSanityNoOp() {
  beginScenario("Duplicate-id sanity: RobotLoop::handleMove() short-circuits a resent already-accepted "
                "Move.id BEFORE honoring replace -- no additional plan change (robot_loop.cpp ~216-225)");
  TestSim::SimHarness sim;
  TestSupport::configureSimForBenchTest(sim);
  sim.boot();
  checkTrue(sim.isConfigured(), "both ports configured -- handleMove()'s configuration gate is open");

  const uint32_t kMoveId = 500;
  sim.injectMove(/*v_x=*/150.0f, /*v_y=*/0.0f, /*omega=*/0.0f, TestSupport::MoveStopKind::kDistance,
                 /*stopValue=*/5000.0f, /*timeout=*/60000.0f, /*replace=*/false, kMoveId, /*corrId=*/9001);
  sim.step(5);
  checkTrue(sim.planner().active(), "the original Move is active after its first accept");
  checkUintEq(sim.planner().activeMoveId(), kMoveId, "the original id is the one active");
  const int depthBefore = queueDepth(sim.planner());
  const float leftBefore = sim.planner().commandedLeft();
  const float rightBefore = sim.planner().commandedRight();

  // Resend the EXACT SAME id (a different corr_id, as a retried enqueue
  // whose original ack was lost would carry -- robot_loop.cpp's own comment
  // above alreadyAccepted()) with utterly DIFFERENT content (a different
  // stop condition, a different curvature) and replace=true: if the dedup
  // short-circuit did not run first, this would look exactly like a real
  // replace and change the plan. It must not.
  sim.injectMove(/*v_x=*/0.0f, /*v_y=*/0.0f, /*omega=*/3.0f, TestSupport::MoveStopKind::kAngle,
                 /*stopValue=*/3.14f, /*timeout=*/60000.0f, /*replace=*/true, kMoveId, /*corrId=*/9002);
  sim.step(3);

  const int depthAfter = queueDepth(sim.planner());
  const float leftAfter = sim.planner().commandedLeft();
  const float rightAfter = sim.planner().commandedRight();
  checkUintEq(sim.planner().activeMoveId(), kMoveId,
              "still the SAME original Move active -- no replacement landed");
  checkTrue(depthAfter == depthBefore, "planner queue depth unchanged by the duplicate-id resend");
  std::printf("  DUP_ID_QUEUE_DEPTH_BEFORE=%d AFTER=%d  cmdLeft %.3f->%.3f  cmdRight %.3f->%.3f\n",
              depthBefore, depthAfter, static_cast<double>(leftBefore), static_cast<double>(leftAfter),
              static_cast<double>(rightBefore), static_cast<double>(rightAfter));

  // robot_loop.cpp's own comment: "Ack it as success -- the move genuinely
  // is enqueued, running, or done" -- confirm the resend's OWN corr_id
  // still acks OK via the telemetry ack ring, not just that the plan held.
  std::vector<TestSupport::DecodedLine> frames = sim.drainTelemetry();
  bool sawDupAck = false;
  for (const TestSupport::DecodedLine& frame : frames) {
    if (frame.kind != TestSupport::DecodedKind::kTelemetry) continue;
    for (uint8_t i = 0; i < frame.telemetry.acks_count; ++i) {
      const uint32_t packed = frame.telemetry.acks_[i];
      constexpr uint32_t kAckErrBits = 4;
      constexpr uint32_t kAckErrMask = (1u << kAckErrBits) - 1;
      if ((packed >> kAckErrBits) == 9002u) {
        sawDupAck = true;
        checkUintEq(packed & kAckErrMask, 0u, "the duplicate-id resend still acks OK (err==0)");
      }
    }
  }
  checkTrue(sawDupAck, "the duplicate resend's own corr_id shows up, acked OK, in the telemetry ack ring");
}

}  // namespace

int main() {
  scenarioSameCurvatureAtSpeedBaseline();
  scenarioEdgeBLargeCurvatureStepAtSpeed();
  scenarioHighRateReplacement();
  scenarioDuplicateIdSanityNoOp();

  if (g_failureCount == 0) {
    std::printf("OK: all 127-001 replace-preemption characterization scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the 127-001 replace-preemption scenarios\n", g_failureCount);
  return 1;
}
