// rebaseline_pose_harness.cpp -- 131-004 (position-rebaseline-destroys-the-
// pose.md, SUC-131-004): pose-sanity across Core::RobotLoop::publishWheel()'s
// software position rebaseline (kPositionRebaselineMargin==30,000mm,
// robot_loop.cpp), against the REAL, live-responding TestSim::SimHarness/
// TestSim::SimPlant -- the same composition root main.cpp boots (130-002).
//
// Root defect this proves fixed: publishWheel() re-anchors a wheel's raw
// position (a ~30,000mm software-only jump) and bumps positionEpoch the
// SAME cycle; Motion::Odometry::integrate() and the planner's
// Motion::PoseTracker::integrate() used to difference blindly across that
// jump (heading jumps ~234rad, x/y by ~15m). Both now take each wheel's
// positionEpoch and re-anchor instead of differencing when it changes
// (odometry.h/estimation.h's own doc comments).
//
// Scenario 1 (pose-sanity + soak, AC "no discontinuity anywhere past 30m"):
// drives BOTH wheels straight past the margin via the WHEELS teleop
// primitive (bypasses Motion::Planner entirely for ACTUATION, but
// Motion::Planner::tick()/update() still run every cycle regardless of who
// owns motion -- robot_loop.cpp's cycle() -- so this also exercises
// PoseTracker/WheelChannel's continuous ingestion throughout). Logs
// state.pose.x/y/heading (Motion::Odometry, the wire/world pose) every
// cycle and asserts no cycle-to-cycle jump beyond a generous bound anywhere
// in the run, not just at the triggering cycle, plus total travel matches
// commanded.
//
// Scenario 2 (Move in flight AT the boundary): pre-drives both wheels to
// just under the margin, then issues a fresh 90deg ANGLE-stop MOVE whose
// own wheel rotation crosses the margin mid-turn. Asserts the Move
// completes close to the commanded angle (state.estimate.body.heading --
// the planner's OWN PoseTracker-derived pose, exactly the ledger this
// ticket's fix touches and exactly what the Move's own completion math in
// Planner::measure() reads for an Angle Move), not corrupted by the
// mid-move epoch change. The uncorrected defect corrupts heading by
// roughly 234rad (~13,400deg) -- orders of magnitude past any normal
// turn-settle error -- so a coarse few-degree tolerance cleanly
// distinguishes "the fix holds" from "the epoch change corrupted this
// Move" without needing PerfectPlant-level exactness against the real sim
// plant.
//
// Run by test_rebaseline_pose.py, which compiles this file together with
// sim_plant.cpp, wire_test_codec.cpp, the plant sources, and the same full
// HOST_BUILD Devices/App/messages/kinematics dependency graph every sibling
// test_*.py in this directory already compiles.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include "bench_test_config.h"
#include "sim_harness.h"
#include "types/robot_state.h"

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

void checkNear(float actual, float expected, float tol, const std::string& what) {
  if (std::fabs(actual - expected) > tol) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %g, got %g (tol %g)", what.c_str(),
                  static_cast<double>(expected), static_cast<double>(actual),
                  static_cast<double>(tol));
    fail(buf);
  }
}

// Mirrors robot_loop.cpp's own file-local (anonymous-namespace)
// kPositionRebaselineMargin -- not exported, so this test hardcodes the
// same documented value (30,000mm) rather than reaching into firmware
// internals.
constexpr float kRebaselineMarginMm = 30000.0f;  // [mm]

}  // namespace

void scenarioPoseSanityAcrossRebaselineMarginAndSoak() {
  beginScenario(
      "pose-sanity across the rebaseline margin: no discontinuity anywhere past 30m of travel, "
      "total odometry travel matches commanded (SUC-131-004)");

  TestSim::SimHarness sim;
  TestSupport::configureSimForBenchTest(sim);
  sim.boot();
  sim.step(3);  // settle: both leaves' own one-time zero-duty activation writes land

  constexpr float kSpeed = 400.0f;  // [mm/s] -- matches the issue's own "~75s at 400mm/s" trigger
  constexpr float kWheelsDuration = 200000.0f;  // [ms] -- ample hold window for the whole run
  sim.injectWheels(kSpeed, kSpeed, kWheelsDuration, /*id=*/1, /*corrId=*/1);

  // Run 36,000mm at kSpeed -- comfortably past the 30,000mm margin, so the
  // "no discontinuity anywhere past 30m" soak requirement is genuinely
  // exercised across many post-boundary cycles, not just the single
  // triggering one.
  //
  // The cycle COUNT is derived from the loop period, never hardcoded: what
  // this scenario needs is a DISTANCE, and a fixed count silently stops
  // delivering it whenever kCycle changes. The literal 1800 here was
  // "90s @ 50ms"; at kCycle=32 the same 1800 cycles is 57.6s == 23,040mm,
  // short of the 30,000mm margin, so the run never crossed the boundary it
  // exists to test and the scenario failed on its own setup check.
  constexpr float kRunDistance = 36000.0f;                      // [mm]
  constexpr float kCyclePeriod = Core::RobotLoop::kCycle * 0.001f;  // [s]
  const int kRunCycles = static_cast<int>(kRunDistance / (kSpeed * kCyclePeriod)) + 1;

  const Types::RobotState* state = &sim.robotLoop().state();
  float prevX = state->pose.x;
  float prevY = state->pose.y;
  float prevHeading = state->pose.heading;
  uint8_t prevEpochLeft = state->wheelLeft.positionEpoch;
  uint8_t prevEpochRight = state->wheelRight.positionEpoch;

  // Generous per-cycle bounds: ordinary travel is ~kSpeed*50ms/1000 ==
  // 20mm/cycle; heading should stay ~0 (straight, equal wheel commands).
  // The uncorrected defect produces a jump on the order of ~15,000mm
  // (position, averaged over one wheel's ~30,000mm swing) or ~234rad
  // (heading) -- both enormous relative to these bounds, so there is no
  // risk of a legitimate settle transient tripping them.
  constexpr float kMaxPerCyclePositionJump = 250.0f;  // [mm]
  constexpr float kMaxPerCycleHeadingJump = 0.2f;      // [rad] (~11.5deg)

  bool sawEpochChange = false;
  int firstEpochChangeCycle = -1;

  for (int i = 0; i < kRunCycles; ++i) {
    sim.step(1);
    state = &sim.robotLoop().state();

    const float dx = state->pose.x - prevX;
    const float dy = state->pose.y - prevY;
    const float dHeading = state->pose.heading - prevHeading;

    if (state->wheelLeft.positionEpoch != prevEpochLeft ||
        state->wheelRight.positionEpoch != prevEpochRight) {
      sawEpochChange = true;
      if (firstEpochChangeCycle < 0) firstEpochChangeCycle = i;
    }

    if (std::fabs(dx) > kMaxPerCyclePositionJump || std::fabs(dy) > kMaxPerCyclePositionJump) {
      char buf[256];
      std::snprintf(buf, sizeof(buf),
                    "cycle %d: pose position jumped (dx=%.2f dy=%.2f) beyond the %.0fmm "
                    "per-cycle bound -- a rebaseline discontinuity",
                    i, static_cast<double>(dx), static_cast<double>(dy),
                    static_cast<double>(kMaxPerCyclePositionJump));
      fail(buf);
    }
    if (std::fabs(dHeading) > kMaxPerCycleHeadingJump) {
      char buf[256];
      std::snprintf(buf, sizeof(buf),
                    "cycle %d: pose.heading jumped by %.4frad beyond the %.2frad per-cycle bound "
                    "-- a rebaseline discontinuity",
                    i, static_cast<double>(dHeading), static_cast<double>(kMaxPerCycleHeadingJump));
      fail(buf);
    }

    prevX = state->pose.x;
    prevY = state->pose.y;
    prevHeading = state->pose.heading;
    prevEpochLeft = state->wheelLeft.positionEpoch;
    prevEpochRight = state->wheelRight.positionEpoch;
  }

  checkTrue(sawEpochChange,
            "the run actually crossed the rebaseline margin (positionEpoch incremented on at "
            "least one wheel) -- otherwise this scenario would prove nothing");

  const float expectedTravel =
      kSpeed * (static_cast<float>(kRunCycles) *
                static_cast<float>(TestSim::SimHarness::kCycleDtUs) / 1.0e6f);  // [mm]
  const float actualTravel = state->pose.x;  // straight run: x IS the travel
  // Generous 5% tolerance -- absorbs the real plant's ramp-up settle time
  // (a handful of cycles at less than full speed); a genuine ~30,000mm-scale
  // rebaseline corruption would miss by orders of magnitude more than this.
  checkNear(actualTravel, expectedTravel, 0.05f * expectedTravel,
            "total odometry travel (pose.x, a straight run) matches the commanded distance "
            "within 5% -- no ~30,000mm-scale rebaseline corruption");

  std::printf("  positionEpoch first changed at cycle %d; final pose.x=%.1f expected~%.1f\n",
              firstEpochChangeCycle, static_cast<double>(actualTravel),
              static_cast<double>(expectedTravel));
}

constexpr float kOmega = 2.0f;                              // [rad/s]
constexpr float kAngle = static_cast<float>(M_PI) * 0.5f;  // [rad] 90deg

// Runs one fresh SimHarness through an identical 90deg ANGLE-stop pivot,
// optionally pre-driving both wheels to just under the rebaseline margin
// first so the pivot's own wheel travel crosses it mid-turn. Returns the
// planner's own final state.estimate.body.heading, in degrees, once the
// Move has had a generous cycle budget to settle. `outSawRebaseline` (if
// non-null) reports whether a rebaseline was observed during the pivot
// itself.
float runAngleMoveAndGetFinalHeadingDeg(bool preDriveNearMargin, bool* outSawRebaseline) {
  TestSim::SimHarness sim;
  TestSupport::configureSimForBenchTest(sim);
  sim.boot();
  sim.step(3);

  if (preDriveNearMargin) {
    constexpr float kPreDriveSpeed = 400.0f;  // [mm/s]
    // Park both wheels just under the margin so the upcoming pivot's own
    // ~100.5mm dominant-wheel travel (trackWidth/2 * pi/2, at
    // TestSim::kDefaultTrackWidth==128mm) crosses it roughly mid-turn, not
    // at the very start or end.
    constexpr float kPreDriveTarget = kRebaselineMarginMm - 50.0f;  // [mm]

    sim.injectWheels(kPreDriveSpeed, kPreDriveSpeed, /*duration=*/200000.0f, /*id=*/1,
                      /*corrId=*/1);

    constexpr int kMaxPreDriveCycles = 3000;  // 150s of virtual time -- ample for ~75s needed
    int preDriveCycles = 0;
    while (preDriveCycles < kMaxPreDriveCycles &&
           sim.robotLoop().state().wheelLeft.position < kPreDriveTarget) {
      sim.step(1);
      ++preDriveCycles;
    }
    checkTrue(sim.robotLoop().state().wheelLeft.position >= kPreDriveTarget,
              "pre-drive reached the target position within the cycle budget");
    checkTrue(sim.robotLoop().state().wheelLeft.positionEpoch == 0 &&
                  sim.robotLoop().state().wheelRight.positionEpoch == 0,
              "pre-drive itself stayed below the rebaseline margin -- no rebaseline fired yet");
  }

  const uint8_t epochLeftBefore = sim.robotLoop().state().wheelLeft.positionEpoch;
  const uint8_t epochRightBefore = sim.robotLoop().state().wheelRight.positionEpoch;

  // omega > 0 -> vRight = +half (forward, continuing to increase -- the
  // wheel the pre-drive above parked near the margin), vLeft = -half
  // (reverse, moving AWAY from its own margin) -- shape.cpp's own
  // vLeft = v_x - half, vRight = v_x + half decomposition for a Twist
  // Move. omega=2.0 puts |half|==128mm/s, comfortably above the
  // wheel-speed floor (vMin, ticket 003's own territory) so this scenario
  // stays isolated to the epoch/rebaseline mechanism under test here; it
  // also matches test_angle_stop_rotation_calibration.py's own
  // square_tour.py-derived rate, so the two harnesses' plant behavior is
  // directly comparable.
  sim.injectMove(/*v_x=*/0.0f, /*v_y=*/0.0f, kOmega, TestSupport::MoveStopKind::kAngle,
                 /*stopValue=*/kAngle, /*timeout=*/30000.0f, /*replace=*/true, /*id=*/2,
                 /*corrId=*/2);

  bool sawRebaseline = false;
  // Generous budget: an unshaped (configureSimForBenchTest()'s effectively-
  // infinite ceilings) 90deg pivot at these gains settles in well under 5s
  // (100 cycles); 400 cycles (20s) leaves ample margin.
  constexpr int kMoveRunCycles = 400;
  for (int i = 0; i < kMoveRunCycles; ++i) {
    sim.step(1);
    if (sim.robotLoop().state().wheelLeft.positionEpoch != epochLeftBefore ||
        sim.robotLoop().state().wheelRight.positionEpoch != epochRightBefore) {
      sawRebaseline = true;
    }
  }

  if (outSawRebaseline) *outSawRebaseline = sawRebaseline;
  return sim.robotLoop().state().estimate.body.heading * 180.0f / static_cast<float>(M_PI);
}

void scenarioMoveInFlightAtRebaselineBoundaryCompletesCorrectly() {
  beginScenario(
      "a MOVE in flight AT the rebaseline boundary completes near the commanded angle, not "
      "corrupted by the mid-move epoch change (SUC-131-004)");

  // CONTROL first: the identical 90deg pivot, from position ~0, with NO
  // rebaseline anywhere in it. TestSim::WheelPlant's own first-order
  // actuation lag produces a real, separately-characterized systematic
  // overshoot at this omega (test_angle_stop_rotation_calibration.py's own
  // header: "~13 deg constant overshoot" when uncalibrated, exactly this
  // harness's configuration) that has NOTHING to do with this ticket's
  // fix -- measuring it here, uncontaminated by any rebaseline, is what
  // lets the rebaseline case below be checked against a TIGHT delta rather
  // than a loose absolute bound that could silently hide a real, smaller
  // regression.
  bool controlSawRebaseline = false;
  const float controlHeadingDeg = runAngleMoveAndGetFinalHeadingDeg(
      /*preDriveNearMargin=*/false, &controlSawRebaseline);
  checkTrue(!controlSawRebaseline,
            "control run: no rebaseline anywhere (sanity on the control itself)");

  bool sawRebaselineDuringMove = false;
  const float headingDeg =
      runAngleMoveAndGetFinalHeadingDeg(/*preDriveNearMargin=*/true, &sawRebaselineDuringMove);

  checkTrue(sawRebaselineDuringMove,
            "the rebaseline actually fired WHILE this Move was active -- otherwise this "
            "scenario would prove nothing");

  const float expectedDeg = kAngle * 180.0f / static_cast<float>(M_PI);

  // Primary check: the REBASELINE's own contribution to error, isolated
  // from the shared (unrelated, uncalibrated-plant) systematic bias by
  // comparing directly against the control run above. The uncorrected
  // defect corrupts heading by ~234rad (~13,400deg) -- a tight delta here
  // cleanly distinguishes "the fix holds" (rebaseline case tracks the
  // control) from "the epoch change corrupted this Move" (rebaseline case
  // diverges wildly from the control).
  checkNear(headingDeg, controlHeadingDeg, 8.0f,
            "the rebaseline case's final heading matches the no-rebaseline CONTROL's within "
            "8deg -- the mid-move rebaseline itself contributes negligible error, isolated from "
            "the shared uncalibrated-plant overshoot both runs equally inherit");

  // Secondary, absolute sanity floor: comfortably clears the KNOWN,
  // separately-characterized ~13deg uncalibrated-plant bias (see the
  // control comment above) while staying three orders of magnitude
  // tighter than the ~13,400deg an uncorrected epoch-change corruption
  // would produce.
  checkNear(headingDeg, expectedDeg, 20.0f,
            "the planner's own pose (state.estimate.body.heading, the ledger this ticket's fix "
            "touches, and exactly what Planner::measure()'s Angle-kind residual reads) lands "
            "within 20deg of the commanded 90deg turn as an absolute sanity floor");

  std::printf(
      "  final heading: rebaseline case=%.3fdeg, control (no rebaseline)=%.3fdeg, commanded=%.1fdeg\n",
      static_cast<double>(headingDeg), static_cast<double>(controlHeadingDeg),
      static_cast<double>(expectedDeg));
}

int main() {
  std::printf("=== Position-rebaseline pose sanity (131-004, SUC-131-004) ===\n");
  std::printf("Direct proof that Motion::Odometry/Motion::PoseTracker re-anchor across "
              "Core::RobotLoop::publishWheel()'s software position rebaseline, instead of "
              "differencing across the ~30,000mm jump.\n\n");

  scenarioPoseSanityAcrossRebaselineMarginAndSoak();
  scenarioMoveInFlightAtRebaselineBoundaryCompletesCorrectly();

  if (g_failureCount == 0) {
    std::printf("OK: rebaseline pose-sanity scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the rebaseline pose-sanity scenarios\n",
              g_failureCount);
  return 1;
}
