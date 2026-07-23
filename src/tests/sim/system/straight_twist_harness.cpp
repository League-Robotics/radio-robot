// straight_twist_harness.cpp -- ticket 108-004's own headline acceptance:
// the direct regression proof that the divergence bug motivating sprint 108
// (an arbitrary twist stream could drift the deleted SimApi::DutyPredictor's
// prediction and the firmware's real write sequence apart -- observed as
// "left encoder freezes, right runs away") is gone against the REAL,
// live-responding TestSim::SimPlant/TestSim::SimHarness.
//
// Scenario: command a straight twist (v_x only, omega=0) at
// kCruiseVx=150mm/s -- this sprint's own SUC-042 tour cruise speed
// (host/robot_radio/planner/tour.py's own DEFAULT_V_MAX), NOT a saturating
// speed -- run it for kRunCycles cycles (kRunCycles*SimHarness::kCycleDtUs
// == a tour-leg-scale duration, several seconds of virtual time -- 4.8s at
// the current 40ms cycle, 118 ticket 003), and at
// EVERY sampled cycle (not just the final one) assert:
//   (a) both wheels are tracking together -- neither motor's velocity()
//       is frozen (pinned at exactly the same value for the whole run,
//       the "left freezes" flavor) nor diverging wildly from the other
//       (the "right runs away" flavor);
//   (b) trueHeading() stays within a small, DOCUMENTED tolerance of zero.
//
// --- The ~11deg open-loop heading offset investigation (this ticket's own
// "investigate and report" requirement) ---
// Ticket 003's own testing observed ~11deg of heading drift at a
// SATURATING v_x=600mm/s. This harness's own diagnostic run (kept here as
// a comment, not code, since the finding is now understood and does not
// need a live throwaway path) measured, cycle by cycle, at BOTH v_x=600 and
// this file's own v_x=150:
//
//   v_x=600 (saturating): heading rises monotonically during the ~20-cycle
//   ramp-to-saturation window, then LOCKS at a flat ~11.19deg for the rest
//   of the run (both wheels pinned at the SAME saturated duty from cycle
//   ~18 onward, so no further differential accumulates).
//
//   v_x=150 (this file's own target, non-saturating): heading peaks at
//   ~6.0deg within the first 2 cycles, then partially unwinds as the
//   REACHABLE-target PID's own natural give-and-take partially cancels the
//   accumulated asymmetry, settling into a persistent ~2.7-2.9deg residual
//   (oscillating by a few tenths of a degree around that value) for the
//   rest of the run -- never growing further, never returning fully to
//   zero.
//
// ROOT CAUSE, AS IT STOOD THROUGH 118 (verified by reading
// src/firm/app/robot_loop.cpp's own RobotLoop::cycle() schedule, not
// guessed): drive_.tick() -- the call that converts the currently-injected
// twist into fresh L/R wheel velocity targets -- ran BETWEEN
// motorR_.requestSample() and motorR_.tick(), i.e. strictly AFTER
// motorL_.tick() had already run for that same cycle (robot_loop.cpp's own
// cycle() body: motorL_.requestSample() -> motorL_.tick() -> ... ->
// drive_.tick() -> motorR_.requestSample() -> motorR_.tick()). So on the
// very cycle a fresh twist (or a fresh profiled setpoint) was dispatched,
// the RIGHT motor picked up the new target immediately, but the LEFT motor
// still ticked against the OLD (stale) target for one more cycle -- a
// genuine, one-cycle sequencing asymmetry baked into the firmware's own
// schedule, not a simulator artifact. TestSim::SimPlant's own tick()
// (sim_plant.cpp) steps BOTH WheelPlants and the OtosPlant together,
// symmetrically, every single call -- there is no left/right ordering
// asymmetry anywhere in the plant itself to fix.
//
// FIXED by 119 ticket 005 (straight-leg-crab actuation/telemetry pairing
// skew -- clasi/issues/straight-leg-crab-118-001-actuation-and-telemetry-pairing-skew.md):
// drive_.tick() now runs at the very TOP of cycle(), before EITHER motor's
// own select/collect, so both motorL_.tick() and motorR_.tick() apply the
// SAME staged target every cycle -- the one-cycle L/R actuation skew this
// comment used to document is gone. src/tests/sim/unit/
// app_robot_loop_harness.cpp's own scriptMotorCycle() call sites were
// independently re-derived to match (both leaves' one-time first duty
// write now lands on the SAME cycle, not staggered). Empirically
// re-measured against this exact scenario post-fix: maxAbsHeadingDeg ==
// 0.0000 for the entire run (was ~6.0deg peak / ~2.7-2.9deg settled) --
// kHeadingToleranceDeg below is tightened accordingly (was 8deg).
// kHeadingToleranceDeg is set with margin over the measured 0.0000 to
// tolerate legitimate floating-point/PID noise, not to paper over a real
// drift -- see the straight-leg-crab issue's own repro
// (docs/code_review/2026-07-22-turn-execution-review-scripts/straight_drift_repro.py)
// for the independent v_x=150mm/s, stop_distance=700mm confirmation
// (cruise heading +2.685deg -> 0.000deg, final y +32.5mm -> +0.0mm).
//
// Hand-rolled assertions, PASS/FAIL, nonzero exit on any failure -- mirrors
// every other src/tests/sim/system harness's own shape. Run by
// test_straight_twist.py, which compiles this file together with
// sim_plant.cpp, wire_test_codec.cpp, the plant sources, and the same full
// HOST_BUILD Devices/App/messages/kinematics dependency graph every sibling
// test_*.py in this directory already compiles.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

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

}  // namespace

int main() {
  std::printf("=== Straight-Twist Stays-Straight Regression (108-004, SUC-041) ===\n");
  std::printf("Direct regression proof for the divergence bug (left encoder freezes, right\n");
  std::printf("runs away) that motivated sprint 108 -- against the live-responding SimPlant.\n\n");

  // Tour cruise speed (host/robot_radio/planner/tour.py's own
  // DEFAULT_V_MAX) -- deliberately NOT a saturating speed (contrast every
  // OTHER scenario's v_x=1000, TestSim::kDefaultDutyVelMax==500).
  constexpr float kCruiseVx = 150.0f;  // [mm/s]
  constexpr float kOmega = 0.0f;       // [rad/s]

  // kRunCycles*SimHarness::kCycleDtUs(40ms, 118 ticket 003) == 4.8s -- order
  // of magnitude of a tour leg's own duration at this cruise speed (e.g. a
  // 600-900mm leg, this sprint's own SUC-042 profiled-leg scale, tests
  // 4-6s of travel).
  constexpr int kRunCycles = 120;

  // See this file's own header for the full derivation: 119 ticket 005
  // fixed the one-cycle L/R actuation skew that used to produce a ~6.0deg
  // peak transient here (measured post-fix: maxAbsHeadingDeg == 0.0000 for
  // the entire run). 0.5deg sits with margin over that measured 0.0000 for
  // legitimate floating-point/PID noise -- two orders of magnitude tighter
  // than the pre-fix 8deg bound, not loosened to paper over any drift.
  constexpr float kHeadingToleranceDeg = 0.5f;

  // The startup transient itself (this file's header) legitimately holds
  // one wheel well below the other for the first several cycles after the
  // twist is dispatched (empirically: |velL-velR| peaks at ~86mm/s at
  // cycle 3, decaying to single digits by cycle 5) -- kSettleCycles skips
  // only the wheel-tracking check across that window (never the heading
  // check, which must hold from sample 0), matching every sibling
  // scenario's own settle convention.
  constexpr int kSettleCycles = 6;

  // Generous bound: catches genuine runaway divergence (the bug this test
  // exists to catch) while comfortably clearing the observed few-mm/s
  // steady-state oscillation a bare P-only harness PID produces (see
  // profiled_motion_harness.cpp's own "legitimately OSCILLATES" comment).
  constexpr float kMaxWheelDivergence = 60.0f;  // [mm/s] |velLeft - velRight|

  // Proves neither wheel is frozen at (or near) zero once past the startup
  // settle window -- well below the ~120-130mm/s this P-only PID actually
  // holds at this cruise target, comfortably above 0.
  constexpr float kMinTrackingVelocity = 20.0f;  // [mm/s]

  TestSim::SimHarness sim;
  TestSupport::configureSimForBenchTest(sim);
  sim.boot();
  sim.step(3);  // settle: both leaves' own one-time zero-duty activation writes land

  beginScenario("straight twist: v_x=150mm/s, omega=0, held for a tour-leg-scale run");
  // 116-006 (MOVE protocol cutover): bare TWIST/injectTwist() is gone --
  // a TIME-stop MOVE with a stop value/timeout far longer than this run
  // is the equivalent "hold this twist indefinitely" injection.
  sim.injectMove(kCruiseVx, /*v_y=*/0.0f, kOmega, TestSupport::MoveStopKind::kTime,
                 /*stopValue=*/100000.0f, /*timeout=*/100000.0f, /*replace=*/true, /*id=*/1,
                 /*corrId=*/1);

  float maxAbsHeadingDeg = 0.0f;
  bool everFrozenOrDiverged = false;
  bool everHeadingOutOfBound = false;

  std::printf("%6s  %9s %9s  %10s\n", "cycle", "velL", "velR", "headingDeg");
  for (int i = 0; i < kRunCycles; ++i) {
    sim.step(1);

    float velL = sim.motorLeft().velocity();
    float velR = sim.motorRight().velocity();
    float headingDeg = sim.trueHeading() * 180.0f / static_cast<float>(M_PI);
    maxAbsHeadingDeg = std::max(maxAbsHeadingDeg, std::fabs(headingDeg));

    if (i % 10 == 0 || i == kRunCycles - 1) {
      std::printf("%6d  %9.2f %9.2f  %10.4f\n", i, static_cast<double>(velL), static_cast<double>(velR),
                  static_cast<double>(headingDeg));
    }

    // (b) Heading bound -- checked at EVERY sample, from cycle 0.
    if (std::fabs(headingDeg) > kHeadingToleranceDeg) {
      everHeadingOutOfBound = true;
      char buf[256];
      std::snprintf(buf, sizeof(buf), "cycle %d: |heading|=%.3fdeg exceeds the %.1fdeg tolerance", i,
                    static_cast<double>(std::fabs(headingDeg)), static_cast<double>(kHeadingToleranceDeg));
      fail(buf);
    }

    // (a) Wheel-tracking bound -- checked at every sample past the startup
    // settle window.
    if (i >= kSettleCycles) {
      bool diverged = std::fabs(velL - velR) > kMaxWheelDivergence;
      bool frozen = (std::fabs(velL) < kMinTrackingVelocity) || (std::fabs(velR) < kMinTrackingVelocity);
      if (diverged || frozen) {
        everFrozenOrDiverged = true;
        char buf[256];
        std::snprintf(buf, sizeof(buf),
                      "cycle %d: velL=%.2f velR=%.2f -- %s%s%s", i, static_cast<double>(velL),
                      static_cast<double>(velR), diverged ? "diverged" : "", diverged && frozen ? " AND " : "",
                      frozen ? "frozen" : "");
        fail(buf);
      }
    }
  }
  std::printf("\n");

  checkTrue(!everHeadingOutOfBound,
            "trueHeading() stayed within the documented tolerance of zero for the ENTIRE run");
  checkTrue(!everFrozenOrDiverged,
            "both wheels tracked together (neither frozen nor diverging) for the entire post-settle run");

  std::printf("  RESULT: maxAbsHeadingDeg=%.4f (tolerance %.1fdeg)\n", static_cast<double>(maxAbsHeadingDeg),
              static_cast<double>(kHeadingToleranceDeg));

  if (g_failureCount == 0) {
    std::printf("OK: straight-twist stays-straight regression passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the straight-twist regression\n", g_failureCount);
  return 1;
}
