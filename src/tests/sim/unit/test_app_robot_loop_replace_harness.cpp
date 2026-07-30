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
//   Cases 1-5 (scenarioSameCurvature.../scenarioHighRateReplacement) drive
//   a bare Motion::Planner directly (TestPlanner::benchLimits() +
//   TestPlanner::PerfectPlant/NoisyPlant, src/motion/planner/tests/
//   test_support.h -- the SAME zero-Python, no-RobotLoop scaffolding
//   src/motion/planner/tests/planner_scenarios_test.cpp's own
//   testReplacePreempts() already uses for the "does it preempt at all"
//   smoke case). This is deliberate, not a shortcut: Finding 1's two edges
//   are BOTH internal to Planner::move()/tick() -- profileVelocity_'s
//   axis-unit carry and the WheelTrim integrator's phase-gated (not
//   replace-gated) reset -- and reaching them through the full wire
//   codec/RobotLoop graph would add nothing but noise. It also matters
//   that these run under REALISTIC, finite accel/decel ceilings
//   (TestPlanner::benchLimits(), the same bench-plausible limits the
//   sketch-verification suite already uses) rather than TestSim::
//   SimHarness's own "effectively unshaped" sim defaults (aMax ~1e6,
//   src/sim/sim_harness.h's simPlannerLimits()) -- an unshaped ceiling
//   would let the profiler jump straight to cruise in one step regardless
//   of the carried-velocity mismatch, hiding exactly the discontinuity
//   ticket 005 needs measured.
//
//   The duplicate-id sanity check (scenarioDuplicateIdSanityNoOp) drives
//   the real TestSim::SimHarness (src/sim/sim_harness.h) instead, because
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
using TestPlanner::NoisyPlant;
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

// Case 3's own peak transient wheel-speed error, stashed here so case 4 can
// report the SAME metric measured under the SAME trim gains/plant and show
// it is materially smaller -- the direct, data-driven demonstration that
// the extra error in case 3 is attributable to the surviving integral, not
// to the ordinary kp response every fresh ramp gets (see case 4's own
// header comment). 0 until scenarioEdgeAAxisChangeAtSpeed() runs; main()
// below runs case 3 before case 4, in file order.
float g_case3PeakTransientErrorMmS = 0.0f;  // [mm/s]

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
  const float trackWidth = benchLimits().trackWidth;  // [mm]

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

// ===========================================================================
// Case 3 (Edge A) -- axis-change-at-speed. Replace a Distance-stopped
// (Linear axis) move with an Angle-stopped (Angular axis) move while at
// speed. profileVelocity_/profileAccel_ zero on the axis change
// (planner.cpp ~723-729) -- but the WheelTrim integrator does NOT (it is
// phase-gated -- frozen outside MovePhase::Hold, reset only when cmd hits
// EXACTLY 0.0f, wheel_trim.h/planner.cpp's stageTrim()). A replace at speed
// never passes through cmd==0, so a Move A-learned integral survives
// straight into Move B's own command. Measures the resulting transient
// wheel-speed error and gives an explicit benign/hazardous verdict.
// ===========================================================================
void scenarioEdgeAAxisChangeAtSpeed() {
  beginScenario("Case 3 (Edge A): axis-change-at-speed -- Distance replaced by Angle while cruising; "
                "the trim integrator is NOT reset by replace");
  Planner planner(benchLimits());
  // Nonzero trim gains so the integrator has something to learn from Move
  // A's own (gain-mismatched) straight leg -- at the default all-zero
  // gains the trim is exactly 0 always and this scenario would prove
  // nothing.
  planner.applyTrimGains(/*kp=*/0.3f, /*ki=*/0.5f, /*iMax=*/150.0f, /*kaff=*/0.0f, /*trimMax=*/150.0f);
  NoisyPlant plant;
  plant.gainLeft = 0.85f;   // left wheel runs 15% slow -- gives the integrator a persistent, real error
  plant.gainRight = 1.0f;
  plant.trackingLag = 1.0f;  // instant response -- isolates the gain-mismatch error from any lag confound
  Types::RobotState state;
  uint32_t now = 0;

  const float kSpeed = 150.0f;  // [mm/s]
  CHECK(planner.move(distanceTwistMove(20, 5000.0f, kSpeed), false));
  for (int i = 0; i < 200; ++i) cycle(planner, state, plant, now, kPeriod);
  checkTrue(planner.phase() == MovePhase::Hold, "straight Move A reaches Hold before the replace");

  const float integralBeforeLeft = planner.trimIntegralLeft();
  std::printf("  CASE3_TRIM_INTEGRAL_BEFORE_REPLACE_MM_S=%.4f\n", static_cast<double>(integralBeforeLeft));
  checkTrue(std::fabs(integralBeforeLeft) > 1.0f,
            "the trim integrator has learned a real (nonzero) correction from the gain-mismatched "
            "straight leg -- precondition for this scenario to mean anything");

  const float kOmega = 2.0f;  // [rad/s]
  // Axis change: Linear (Distance) -> Angular (Angle), replace=true, MID-MOTION.
  CHECK(planner.move(angleTwistMove(21, 3.14159265f, kOmega), true));
  cycle(planner, state, plant, now, kPeriod);  // activation tick

  const float integralAfterLeft = planner.trimIntegralLeft();
  const bool integralSurvived = std::fabs(integralAfterLeft - integralBeforeLeft) < 0.5f;
  std::printf("  CASE3_TRIM_INTEGRAL_AFTER_REPLACE_MM_S=%.4f (survives replace unreset: %s)\n",
              static_cast<double>(integralAfterLeft), integralSurvived ? "yes" : "no");
  checkTrue(integralSurvived,
            "the trim integrator is NOT reset by replace -- it carries its pre-replace value into the "
            "new (Angular) axis, confirming the design issue's own claim structurally, not just by "
            "downstream effect");

  // The transient wheel-speed ERROR the stale integrator injects into the
  // NEW Move over the next N control cycles: trimLeft()/trimRight() is the
  // correction ADDED on top of the freshly-profiled (from-rest, since the
  // axis changed) command -- exactly the extra velocity a stale,
  // off-axis-learned integral asks for on a move it was never tuned
  // against.
  constexpr int kTransientCycles = 10;  // 10 * 50ms = 500ms immediately after the replace
  float peakTrimLeft = 0.0f;
  float peakTrimRight = 0.0f;
  for (int i = 0; i < kTransientCycles; ++i) {
    cycle(planner, state, plant, now, kPeriod);
    peakTrimLeft = std::max(peakTrimLeft, std::fabs(planner.trimLeft()));
    peakTrimRight = std::max(peakTrimRight, std::fabs(planner.trimRight()));
  }
  const float peakTransientError = std::max(peakTrimLeft, peakTrimRight);  // [mm/s]
  g_case3PeakTransientErrorMmS = peakTransientError;  // case 4 reports against this

  // ===== THE labeled number: Case 3's measured transient wheel-speed error. =====
  std::printf(
      "  CASE3_EDGE_A_TRANSIENT_WHEEL_SPEED_ERROR_MM_S=%.4f (peak over %d cycles / %.2fs post-replace; "
      "left=%.4f right=%.4f)\n",
      static_cast<double>(peakTransientError), kTransientCycles,
      static_cast<double>(kTransientCycles) * (kPeriod * 0.001f), static_cast<double>(peakTrimLeft),
      static_cast<double>(peakTrimRight));
  // ================================================================================

  // Verdict, backed by that number: compare the stale-integrator error
  // against the NEW Move's own commanded wheel speed (omega * trackWidth/2)
  // -- the yardstick for "is this misapplied correction big enough to
  // meaningfully distort (or exceed, or reverse) the turn it landed on."
  const float kNewMoveCommandedWheelSpeed = kOmega * benchLimits().trackWidth * 0.5f;  // [mm/s]
  const bool hazardous = peakTransientError > kNewMoveCommandedWheelSpeed;
  std::printf(
      "  CASE3_EDGE_A_VERDICT=%s (transient %.4f mm/s vs the new Move's own commanded wheel speed "
      "%.4f mm/s)\n",
      hazardous ? "HAZARDOUS" : "BENIGN", static_cast<double>(peakTransientError),
      static_cast<double>(kNewMoveCommandedWheelSpeed));
  checkTrue(std::isfinite(peakTransientError), "Case 3's transient wheel-speed error is a finite measurement");
}

// ===========================================================================
// Case 4 -- axis-change-from-rest (sanity). Same shape as case 3 but the
// Distance Move is allowed to COMPLETE and drain to rest first (the
// sanctioned "stop, then turn" path), rather than being replaced
// mid-motion. Coming to rest passes cmd through EXACTLY 0.0f, which is
// stageTrim()'s own reset trigger -- so the SAME trim gains that let case 3
// carry a stale integral here start the new Angle Move at integral == 0.
// Expect zero surprises.
// ===========================================================================
void scenarioAxisChangeFromRestSanity() {
  beginScenario("Case 4: axis-change-from-rest (sanity) -- Move A completes and drains to rest, "
                "THEN the Angle Move: zero surprises");
  Planner planner(benchLimits());
  // Same trim gains as case 3 -- if coming to rest did NOT reset the
  // integrator, this scenario would show the identical carry-over case 3
  // does, and it must not.
  planner.applyTrimGains(/*kp=*/0.3f, /*ki=*/0.5f, /*iMax=*/150.0f, /*kaff=*/0.0f, /*trimMax=*/150.0f);
  NoisyPlant plant;
  plant.gainLeft = 0.85f;
  plant.gainRight = 1.0f;
  plant.trackingLag = 1.0f;
  Types::RobotState state;
  uint32_t now = 0;

  // A SHORT Distance Move that actually reaches its threshold and drains to
  // rest on its own (not replaced mid-motion) -- the sanctioned path.
  CHECK(planner.move(distanceTwistMove(30, 120.0f, 150.0f), false));
  bool completed = false;
  for (int i = 0; i < 300 && !completed; ++i) {
    completed = cycle(planner, state, plant, now, kPeriod).completed;
  }
  checkTrue(completed, "Move A (short straight) completes on its own before the axis change");
  // A few more drain cycles so the ramp-to-zero actually lands on cmd==0
  // (stageTrim()'s own reset trigger) and settles.
  for (int i = 0; i < 20; ++i) cycle(planner, state, plant, now, kPeriod);
  checkTrue(!planner.active(), "planner is idle (drained) before the axis change");

  const float integralBeforeLeft = planner.trimIntegralLeft();
  std::printf("  CASE4_TRIM_INTEGRAL_AT_REST_MM_S=%.4f (expect ~0 -- drain-to-zero resets it, unlike a "
              "mid-motion replace)\n",
              static_cast<double>(integralBeforeLeft));
  checkTrue(std::fabs(integralBeforeLeft) < 1e-3f,
            "coming to rest naturally resets the trim integrator (the cmd==0.0f branch) -- the "
            "structural difference from case 3's mid-motion replace");

  // The raw commanded-value jump, same metric case 1 measures (before vs.
  // one activation tick after) -- reported for direct comparison, NOT
  // gated at case 1's sub-mm/s noise floor: from a genuine standstill the
  // profile legitimately ramps up over its first tick (the ordinary accel
  // step every fresh Move gets, axis change or not) -- that is not a
  // "surprise" in the sense this ticket characterizes, so the assertion
  // below only bounds it by the shape's own one-tick accel ceiling, not by
  // case 1's "nothing should have changed at all" floor.
  const float leftBeforeCmd = planner.commandedLeft();
  const float rightBeforeCmd = planner.commandedRight();

  const float kOmega = 2.0f;  // [rad/s] -- identical to case 3, for a direct comparison
  CHECK(planner.move(angleTwistMove(31, 3.14159265f, kOmega), false));  // from rest; queue is empty either way
  cycle(planner, state, plant, now, kPeriod);  // activation tick
  const float rawDiscontinuity = std::max(std::fabs(planner.commandedLeft() - leftBeforeCmd),
                                          std::fabs(planner.commandedRight() - rightBeforeCmd));  // [mm/s]
  std::printf("  CASE4_FROM_REST_DISCONTINUITY_MM_S=%.4f (informational -- the ordinary first-tick accel "
              "ramp from a genuine standstill, not a stale-state jump)\n",
              static_cast<double>(rawDiscontinuity));

  constexpr int kTransientCycles = 10;
  float peakTrimLeft = 0.0f;
  float peakTrimRight = 0.0f;
  for (int i = 0; i < kTransientCycles; ++i) {
    cycle(planner, state, plant, now, kPeriod);
    peakTrimLeft = std::max(peakTrimLeft, std::fabs(planner.trimLeft()));
    peakTrimRight = std::max(peakTrimRight, std::fabs(planner.trimRight()));
  }
  const float peakTransientError = std::max(peakTrimLeft, peakTrimRight);  // [mm/s]

  // ===== THE labeled number: case 4's own counterpart to case 3's transient error. =====
  std::printf("  CASE4_FROM_REST_TRANSIENT_WHEEL_SPEED_ERROR_MM_S=%.4f (peak over %d cycles; left=%.4f "
              "right=%.4f; case 3's own peak was %.4f)\n",
              static_cast<double>(peakTransientError), kTransientCycles, static_cast<double>(peakTrimLeft),
              static_cast<double>(peakTrimRight), static_cast<double>(g_case3PeakTransientErrorMmS));
  // ===================================================================================

  // This nonzero value is NOT the hazard case 3 characterizes -- it is
  // ordinary kp reacting to the SAME plant tracking imperfection (gainLeft
  // != gainRight) case 3's own plant has, present on any fresh ramp with or
  // without a prior Move (kp is not phase-gated; only the INTEGRAL is --
  // wheel_trim.h's own header). The literal "discontinuity is zero"
  // criterion is the INTEGRAL check above (CASE4_TRIM_INTEGRAL_AT_REST_MM_S,
  // asserted ~0), which IS the state a replace could have left stale. What
  // this comparison confirms instead: case 4's peak transient is
  // meaningfully SMALLER than case 3's own (measured under identical trim
  // gains and an identical plant) -- the gap between the two is exactly
  // the surviving-integral contribution case 3 carries and case 4 does not.
  checkTrue(g_case3PeakTransientErrorMmS > 0.0f,
            "case 3 ran first and recorded a nonzero peak transient to compare against");
  checkTrue(peakTransientError < 0.75f * g_case3PeakTransientErrorMmS,
            "axis-change-from-rest's transient error is meaningfully smaller than case 3's own "
            "(same trim gains, same plant) -- the missing piece is the stale integral case 3 alone "
            "carries, confirming case 3's hazard is specific to replacing MID-MOTION");
}

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
  scenarioEdgeAAxisChangeAtSpeed();
  scenarioAxisChangeFromRestSanity();
  scenarioHighRateReplacement();
  scenarioDuplicateIdSanityNoOp();

  if (g_failureCount == 0) {
    std::printf("OK: all 127-001 replace-preemption characterization scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the 127-001 replace-preemption scenarios\n", g_failureCount);
  return 1;
}
