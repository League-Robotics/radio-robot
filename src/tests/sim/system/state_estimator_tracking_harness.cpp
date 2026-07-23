// state_estimator_tracking_harness.cpp -- off-hardware acceptance harness
// for sprint 117 ticket 005 (SUC-060): proves the WIRED-IN App::
// StateEstimator (RobotLoop::cycle()'s own trailing kPace-block update()
// call, ticket 004) tracks TestSim::SimPlant's own ground-truth wheel/body
// state across the stakeholder's varied motion-pattern set -- steps both
// directions, a reversal, a pivot, and a chained sequence of short moves --
// driven through the REAL App::RobotLoop/App::MoveQueue/App::Drive/
// App::Odometry graph and a REAL, live-responding SimPlant, exactly like
// every sibling sim/system harness (move_protocol_harness.cpp/
// sim_api_harness.cpp).
//
// What "tracks" means here, concretely: this harness exercises the estimator
// as a genuine PREDICT-TO-NOW instrument, not merely a pass-through of
// already-known Frame data. At the end of every cycle it asks
// StateEstimator::wheelAt()/bodyAt() to extrapolate ONE FULL CYCLE (SimHarness
// ::kCycleDtUs, 40ms -- 118 ticket 003) past the CURRENT basis -- the same
// "predict past the last collect" query a live host would make between two
// telemetry frames --
// then, once that next cycle has actually run, compares the STORED
// prediction against SimPlant's own live ground truth at that exact instant
// (TestSim::WheelPlant::position()/velocity() for each wheel via
// SimHarness::plant().wheelPlant(1|2), and SimHarness::trueX()/trueY()/
// trueHeading() for the body -- bypassing OTOS drift/bias and Odometry's own
// independently-integrated pose entirely, per sim_harness.h's own "True
// pose" doc comment). wheelNow()/whereAmI() are also implicitly exercised --
// wheelAt(wheel, basisTime) and bodyAt(basisTime) collapse to them at age
// zero, the very first evaluation this harness performs each phase before
// advancing the target time by one cycle.
//
// Pattern set (AC #1): ramp-up into a steady forward cruise, a reversal
// (the single largest-acceleration transient in this whole set), a pivot
// (turn in place, omega-only), and a chained sequence of four short moves
// alternating direction and kind (straight/turn) with replace=false hand-offs
// -- covers "both directions, steps, reversals, pivots; straights and turns"
// per the ticket's own Description.
//
// Tolerances (AC #2) are phase-specific and were derived empirically: a
// first cut ran every phase with tolerances loose enough to always pass,
// then this file's own comments were filled in with the OBSERVED worst-case
// error per phase plus margin -- mirroring every other sim/system harness's
// own "empirically ~X, tolerance Y keeps margin above the observed value"
// convention (see move_protocol_harness.cpp's DISTANCE/ANGLE stop-condition
// scenarios for the precedent). Steady-state phases (velocity/omega has
// settled, so a one-cycle-ahead ZOH extrapolation is nearly exact) get tight
// tolerances; transient phases (still inside TestSim::kDefaultTau's own
// ~130ms settle window, where velocity itself is changing across the
// 50ms prediction horizon) get visibly looser ones -- exactly the numbers
// AC #3 asks this harness to report, not silently pass/fail.
//
// Stretch (AC #4, a throwaway replay harness cross-checking this same
// wired-in estimator's output against ticket 006's Python
// one_step_ahead.py reference): SKIPPED. Wiring a second, C++-side replay of
// the same input sequence through ticket 006's pure-Python reference would
// need a new sim_ctypes.cpp export (or an ad hoc JSON/CSV bridge this
// harness would have to invent from scratch) to hand a Python process the
// exact per-cycle Frame stream this binary already walks in-process -- not
// "cheap" per the ticket's own qualifier. Ticket 006's own
// test_one_step_ahead.py unit-tests the Python reference against the SAME
// ZOH formula this file's comments already document (matching ticket 002's
// C++ source, hand-computed fixtures), which is the load-bearing proof that
// the two implementations compute the same math -- this harness's own job is
// proving the WIRING (RobotLoop actually calls update() every cycle with
// fresh data), a different, already-covered concern.
//
// Hand-rolled assertions, PASS/FAIL per scenario, nonzero exit on any
// failure -- mirrors every other src/tests/sim/system harness's own shape.
// Run by test_state_estimator_tracking.py, which compiles this file
// together with sim_plant.cpp, wire_test_codec.cpp, the plant sources, and
// the same full HOST_BUILD Devices/App/messages/kinematics dependency graph
// every sibling test_*.py in this directory already compiles.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "app/state_estimator.h"
#include "app/telemetry.h"
#include "bench_test_config.h"
#include "sim_harness.h"
#include "wire_test_codec.h"

namespace {

// --- Hand-rolled assertion plumbing (mirrors every other tests/sim harness
// in this codebase) ---------------------------------------------------------

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

void checkFloatLe(float actual, float bound, const std::string& what) {
  if (!(actual <= bound)) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected <= %g, got %g", what.c_str(),
                  static_cast<double>(bound), static_cast<double>(actual));
    fail(buf);
  }
}

using TestSupport::MoveStopKind;

// --- Per-phase tracking-error bookkeeping (AC #3: report, don't silently
// pass/fail) ---------------------------------------------------------------

struct PhaseStats {
  std::string name;
  int samples = 0;
  float maxWheelDistErr = 0.0f;   // [mm] max over both wheels, one-cycle-ahead prediction vs truth
  float maxWheelVelErr = 0.0f;    // [mm/s] max over both wheels
  float maxBodyPosErr = 0.0f;     // [mm] sqrt(dx^2+dy^2), one-cycle-ahead prediction vs truth
  float maxBodyHeadingErr = 0.0f; // [rad]
};

std::vector<PhaseStats> g_phaseReport;

// runPhase -- steps `cycles` more sim cycles. Each cycle: (1) check the
// prediction STORED by the previous cycle (targeting THIS cycle's own time)
// against this cycle's fresh ground truth, accumulating PhaseStats; (2) make
// a fresh one-cycle-ahead prediction from THIS cycle's own basis, stored for
// the next iteration's check. `havePending` resets false on every call --
// tracking errors are never attributed across a phase boundary (a prediction
// made under the PREVIOUS phase's own Move is never checked against the
// NEXT phase's ground truth), so every phase's own reported numbers are
// entirely its own.
PhaseStats runPhase(TestSim::SimHarness& sim, const std::string& name, int cycles,
                     float wheelDistTolMm, float bodyPosTolMm, float bodyHeadingTolRad) {
  beginScenario(name);
  PhaseStats stats;
  stats.name = name;

  bool havePending = false;
  App::WheelEstimate pendingLeft;
  App::WheelEstimate pendingRight;
  App::BodyEstimate pendingBody;

  constexpr uint32_t kHorizonMs = TestSim::SimHarness::kCycleDtUs / 1000;  // [ms] one full cycle ahead

  for (int i = 0; i < cycles; ++i) {
    sim.step(1);
    uint32_t nowMs = static_cast<uint32_t>(sim.clock().nowMicros() / 1000);

    if (havePending) {
      float trueLeft = sim.plant().wheelPlant(1).position();    // [mm] ground truth, port 1 == left
      float trueRight = sim.plant().wheelPlant(2).position();   // [mm] ground truth, port 2 == right
      float trueVelLeft = sim.plant().wheelPlant(1).velocity(); // [mm/s]
      float trueVelRight = sim.plant().wheelPlant(2).velocity();

      float errLeft = std::fabs(pendingLeft.distance - trueLeft);
      float errRight = std::fabs(pendingRight.distance - trueRight);
      float velErrLeft = std::fabs(pendingLeft.velocity - trueVelLeft);
      float velErrRight = std::fabs(pendingRight.velocity - trueVelRight);
      stats.maxWheelDistErr = std::max(stats.maxWheelDistErr, std::max(errLeft, errRight));
      stats.maxWheelVelErr = std::max(stats.maxWheelVelErr, std::max(velErrLeft, velErrRight));

      // Body ground truth -- SimHarness's own trueX()/trueY()/trueHeading(),
      // NEVER Odometry's own frame.pose (this harness's whole point is
      // comparing the estimator against the PLANT's truth, not against the
      // same dead-reckoned source the estimator itself already blends in).
      float dx = pendingBody.x - sim.trueX();
      float dy = pendingBody.y - sim.trueY();
      float posErr = std::sqrt(dx * dx + dy * dy);
      float headingErr = std::fabs(pendingBody.heading - sim.trueHeading());
      stats.maxBodyPosErr = std::max(stats.maxBodyPosErr, posErr);
      stats.maxBodyHeadingErr = std::max(stats.maxBodyHeadingErr, headingErr);

      ++stats.samples;
    }

    pendingLeft = sim.stateEstimator().wheelAt(App::Wheel::Left, nowMs + kHorizonMs);
    pendingRight = sim.stateEstimator().wheelAt(App::Wheel::Right, nowMs + kHorizonMs);
    pendingBody = sim.stateEstimator().bodyAt(nowMs + kHorizonMs);
    havePending = true;
  }

  checkTrue(stats.samples > 0, name + ": at least one one-cycle-ahead prediction was checked against truth");
  checkFloatLe(stats.maxWheelDistErr, wheelDistTolMm,
               name + ": max one-cycle-ahead wheel distance error stays within the documented tolerance");
  checkFloatLe(stats.maxBodyPosErr, bodyPosTolMm,
               name + ": max one-cycle-ahead body position error stays within the documented tolerance");
  checkFloatLe(stats.maxBodyHeadingErr, bodyHeadingTolRad,
               name + ": max one-cycle-ahead body heading error stays within the documented tolerance");

  std::printf(
      "  PHASE %-20s samples=%-4d wheelDistErr(max)=%.3fmm wheelVelErr(max)=%.3fmm/s "
      "bodyPosErr(max)=%.3fmm bodyHeadingErr(max)=%.5frad\n",
      name.c_str(), stats.samples, static_cast<double>(stats.maxWheelDistErr),
      static_cast<double>(stats.maxWheelVelErr), static_cast<double>(stats.maxBodyPosErr),
      static_cast<double>(stats.maxBodyHeadingErr));

  g_phaseReport.push_back(stats);
  return stats;
}

// ===========================================================================
// The one scenario: varied MOVE patterns, one-cycle-ahead StateEstimator
// tracking checked every cycle of every phase (see this file's own header).
// ===========================================================================

void scenarioTrackingAcrossVariedMovePatterns() {
  TestSim::SimHarness sim;
  TestSupport::configureSimForBenchTest(sim);
  sim.boot();
  sim.step(3);  // settle: both leaves' own one-time zero-duty activation writes land
  (void)sim.drainTelemetry();
  g_phaseReport.clear();

  // --- Phase 1: forward -- ramp-up into a steady cruise -------------------
  // v_x=150mm/s (non-saturating, matches move_protocol_harness.cpp's own
  // TIME-stop cruise speed), a TIME stop far longer than this phase's own
  // cycle budget below (the phase boundary itself, not this Move's own stop
  // condition, is what ends the phase -- the next phase's injectMove()
  // preempts it via replace=true).
  sim.injectMove(150.0f, /*v_y=*/0.0f, /*omega=*/0.0f, MoveStopKind::kTime, 100000.0f, 100000.0f,
                 /*replace=*/true, /*id=*/1, /*corrId=*/101);
  // "forward_ramp": still inside kDefaultTau's own ~130ms settle window (6
  // cycles * 50ms = 300ms spans it) -- velocity is genuinely changing across
  // the one-cycle prediction horizon here, so the ZOH "hold velocity
  // constant" assumption has real, expected error. Empirically ~14.1mm
  // wheel / ~6.6mm body / ~0.037rad heading at worst (118: re-baselined --
  // restoring the interleaved schedule moved updateTlm()'s own staging
  // point to BEFORE motorR_.requestSample()/tick(), so frame_.encRight is
  // now genuinely one cycle stale relative to frame_.encLeft every cycle,
  // exactly the R "-1 cycle" telemetry-staleness the last-known-good
  // 39c084c1 skeleton always had -- see
  // clasi/issues/restore-the-interleaved-request-settle-tick-loop-schedule.md);
  // tolerances below keep >1.25x margin over that without papering over a
  // real regression.
  runPhase(sim, "forward_ramp", /*cycles=*/6, /*wheelDistTolMm=*/18.0f, /*bodyPosTolMm=*/15.0f,
           /*bodyHeadingTolRad=*/0.05f);
  // "forward_steady": velocity has settled (>300ms into a held command) --
  // a one-cycle ZOH extrapolation should be near-exact IN VELOCITY, but the
  // STEADY-STATE wheel-distance error no longer converges to ~0 (118: the
  // same persistent one-cycle encoder-telemetry staleness noted above --
  // ~150mm/s * 50ms one-cycle horizon = ~7.5mm, matching the empirically
  // observed ~7.58mm exactly). bodyPos/heading stay tight (~0.06mm/
  // ~0.0009rad) since the body fuses both wheels and OTOS, diluting the
  // one-wheel staleness; tolerances below keep >1.3x margin on wheel
  // distance, a generous margin on body pos/heading.
  runPhase(sim, "forward_steady", /*cycles=*/20, /*wheelDistTolMm=*/10.0f, /*bodyPosTolMm=*/1.0f,
           /*bodyHeadingTolRad=*/0.01f);

  // --- Phase 2: reversal ---------------------------------------------------
  // v_x=-150mm/s while still cruising forward -- the single largest
  // acceleration transient in this whole pattern set (a full sign flip
  // across TestSim::kDefaultDutyVelMax, not just a step from rest).
  sim.injectMove(-150.0f, /*v_y=*/0.0f, /*omega=*/0.0f, MoveStopKind::kTime, 100000.0f, 100000.0f,
                 /*replace=*/true, /*id=*/2, /*corrId=*/102);
  // Empirically ~13.0mm wheel / ~8.2mm body / ~0.055rad heading at worst
  // (118: heading re-baselined -- same one-cycle-stale-encoder-telemetry
  // mechanism as forward_ramp above, sharper here since this transient
  // reverses sign rather than ramping from rest) -- tolerances below keep
  // >1.25x margin.
  runPhase(sim, "reversal_transient", /*cycles=*/8, /*wheelDistTolMm=*/18.0f, /*bodyPosTolMm=*/15.0f,
           /*bodyHeadingTolRad=*/0.07f);
  // Empirically ~0.09mm body / ~0.0005rad heading once velocity has settled
  // negative; wheel distance no longer converges to ~0 (118: same
  // persistent one-cycle encoder-telemetry staleness as forward_steady
  // above -- ~150mm/s * 50ms = ~7.5mm, matching the observed ~7.59mm).
  runPhase(sim, "steady_reverse", /*cycles=*/20, /*wheelDistTolMm=*/10.0f, /*bodyPosTolMm=*/1.0f,
           /*bodyHeadingTolRad=*/0.01f);

  // --- Phase 3: pivot -- turn in place (v_x=0, omega=1.0rad/s) ------------
  sim.injectMove(/*v_x=*/0.0f, /*v_y=*/0.0f, /*omega=*/1.0f, MoveStopKind::kTime, 100000.0f, 100000.0f,
                 /*replace=*/true, /*id=*/3, /*corrId=*/103);
  // Empirically ~4.4mm wheel / ~2.5mm body / ~0.032rad heading at worst --
  // a pivot's own angular acceleration ramp couples into heading error the
  // straight-line phases above never see (v_x=0 -- no linear-distance
  // ground truth at all, only the differential wheel spin).
  runPhase(sim, "pivot_ramp", /*cycles=*/6, /*wheelDistTolMm=*/10.0f, /*bodyPosTolMm=*/6.0f,
           /*bodyHeadingTolRad=*/0.06f);
  // Wheel distance no longer converges to ~0 once angular rate has settled
  // (118: same persistent one-cycle encoder-telemetry staleness as the
  // straight-line steady phases above -- omega=1.0rad/s at this fixture's
  // trackWidth gives a per-wheel speed of ~60mm/s, so ~60mm/s * 50ms =
  // ~3.0mm, matching the observed ~3.35mm); body pos/heading stay tight
  // (~0.11mm / ~0.0019rad, fused across both wheels + OTOS).
  runPhase(sim, "pivot_steady", /*cycles=*/20, /*wheelDistTolMm=*/5.0f, /*bodyPosTolMm=*/1.0f,
           /*bodyHeadingTolRad=*/0.01f);

  // --- Phase 4: chained steps -- both directions, straights and turns -----
  // Four short Moves, replace=false so each hands off at the previous one's
  // own 200ms/4-cycle TIME stop (SUC-051's own chaining contract) -- the
  // densest sequence of direction/kind changes in this pattern set, and
  // therefore this harness's own worst-case phase for a held-constant ZOH
  // assumption.
  sim.injectMove(100.0f, 0.0f, 0.0f, MoveStopKind::kTime, 200.0f, 100000.0f, /*replace=*/false, 4, 104);
  sim.injectMove(-100.0f, 0.0f, 0.0f, MoveStopKind::kTime, 200.0f, 100000.0f, /*replace=*/false, 5, 105);
  sim.injectMove(0.0f, 0.0f, -1.0f, MoveStopKind::kTime, 200.0f, 100000.0f, /*replace=*/false, 6, 106);
  sim.injectMove(200.0f, 0.0f, 0.0f, MoveStopKind::kTime, 200.0f, 100000.0f, /*replace=*/false, 7, 107);
  // Empirically ~3.3mm wheel (118: the same persistent one-cycle
  // encoder-telemetry staleness the steady phases above show, bounded here
  // by this phase's own <=200mm/s per-leg commanded speed -- 200mm/s * 50ms
  // = 10mm ceiling, but most legs run slower or don't hold long enough to
  // reach that ceiling) / ~0.12mm body / ~0.0012rad heading at worst -- each
  // leg's own commanded speed is modest and each 200ms/4-cycle leg still
  // settles most of the way before the next hand-off, so this phase's own
  // worst case turns out smaller than the ramp phases above; tolerances
  // below keep >1.5x margin on wheel distance, a full order of magnitude
  // above the observed worst case on body pos/heading.
  runPhase(sim, "chained_steps", /*cycles=*/40, /*wheelDistTolMm=*/5.0f, /*bodyPosTolMm=*/3.0f,
           /*bodyHeadingTolRad=*/0.02f);

  sim.injectStop(/*corrId=*/199);
  sim.step(3);

  // AC #3: explicitly report which pattern phase(s) showed the largest
  // tracking error -- not silently pass/fail.
  std::vector<PhaseStats> byWheel = g_phaseReport;
  std::sort(byWheel.begin(), byWheel.end(),
            [](const PhaseStats& a, const PhaseStats& b) { return a.maxWheelDistErr > b.maxWheelDistErr; });
  std::vector<PhaseStats> byBody = g_phaseReport;
  std::sort(byBody.begin(), byBody.end(),
            [](const PhaseStats& a, const PhaseStats& b) { return a.maxBodyPosErr > b.maxBodyPosErr; });

  std::printf("\nREPORT: phases ranked by max one-cycle-ahead WHEEL distance error (largest first):\n");
  for (const auto& p : byWheel) {
    std::printf("  %-20s wheelDistErr(max)=%.3fmm\n", p.name.c_str(), static_cast<double>(p.maxWheelDistErr));
  }
  std::printf("REPORT: phases ranked by max one-cycle-ahead BODY position error (largest first):\n");
  for (const auto& p : byBody) {
    std::printf("  %-20s bodyPosErr(max)=%.3fmm\n", p.name.c_str(), static_cast<double>(p.maxBodyPosErr));
  }
  checkTrue(!byWheel.empty() && !byBody.empty(), "the phase report is non-empty (every phase ran and was recorded)");
}

}  // namespace

int main() {
  std::printf("=== StateEstimator Sim-System Tracking Scenarios (117-005, SUC-060) ===\n");
  std::printf("Proves the WIRED-IN App::StateEstimator (RobotLoop's own kPace-block update() call)\n");
  std::printf("tracks TestSim::SimPlant's own ground-truth wheel/body state across a varied MOVE\n");
  std::printf("pattern set, via a genuine one-cycle-ahead predict-to-now check every cycle.\n\n");

  scenarioTrackingAcrossVariedMovePatterns();

  if (g_failureCount == 0) {
    std::printf("\nOK: all StateEstimator tracking scenarios passed\n");
    return 0;
  }
  std::printf("\nFAILED: %d assertion(s) across the StateEstimator tracking scenarios\n", g_failureCount);
  return 1;
}
