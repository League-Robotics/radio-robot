// deadband_terminal_correction_harness.cpp -- sim SYSTEM acceptance proof
// for sprint 114 ticket 005 (deadband-compensation-small-commands-must-
// produce-real-motion.md): a real terminal correction that falls inside
// NezhaMotor's outputDeadband_-equivalent floor (~15mm/s == outputDeadband_/
// kff == 0.03/0.002, TestSupport::benchTestMotorConfig()'s own values)
// produces real motion and completes promptly, instead of stalling until
// Motion::Executor's own stopTimeBackstopMs() (~8s for a short move -- see
// that function's own derivation, executor.cpp: (durationS*2 + 6.0)*1000)
// gives up -- the exact ~8s hardware-trace symptom the deadband-
// compensation issue documents (an ~11mm/s terminal correction held flat
// until an arrive-timeout).
//
// Two scenarios, both driving the REAL App::RobotLoop/App::Pilot/Motion::
// Executor/Devices::NezhaMotor graph (TestSim::SimHarness) against
// TestSupport::configureSimForBenchTest()'s own UNMODIFIED bench values:
//
//   1. A plain pivot MOVE. Per pilot.cpp's own comment (114-005 does not
//      touch this file, but its reasoning explains WHY this scenario
//      naturally lands in the dead zone): the deleted min-speed floor
//      existed because `heading_kp * heading_dwell_tol` must be >=
//      the deadband-equivalent omega for the terminal PD correction to
//      never fall inside the dead zone -- true for the real robot's
//      shipped heading_kp=6.0, but bench_test_config.cpp's own bench
//      heading_kp=2.5 does NOT clear that bound (2.5 * 3deg-in-rad *
//      trackWidth/2 ~= 8.4mm/s, comfortably under the ~15mm/s floor) --
//      so an ORDINARY pivot naturally exercises the exact defect, no
//      special-case scaffolding needed. Asserts the move reaches
//      ACK_STATUS_DONE well inside a budget far short of the ~8s backstop,
//      never ACK_STATUS_TIMEOUT, and that measured wheel velocity stayed
//      nonzero while a genuinely sub-deadband correction was in flight.
//
//   2. A directly-injected ~11mm/s wheel-differential TWIST (the exact
//      residual value the hardware trace found frozen) -- isolates "does
//      the boost make the plant move, within about one tick" from
//      scenario 1's own natural-pivot timing. Uses the raw TWIST wire
//      command (App::RobotLoop::handleTwist()), which App::Pilot
//      deliberately leaves untouched while idle (pilot.h's own "survive
//      untouched" contract) -- NOT a direct NezhaMotor::setVelocity() call,
//      which App::Drive::tick() would silently overwrite back to 0 every
//      cycle (Drive::tick() always restages both wheels from its own
//      cached v_x_/omega_ -- see drive.cpp).
#include <cmath>
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

constexpr float kDegToRad = 3.14159265f / 180.0f;

// outputDeadband_/kff -- the exact minimum-commandable-speed floor
// writeShapedDuty() now boosts a sub-deadband nonzero duty up TO. Matches
// TestSupport::benchTestMotorConfig()'s own 0.03/0.002 values
// (bench_test_config.cpp) -- kept as an independent literal here (not a
// derived-from-the-harness value) so this test pins the actual bench
// numbers rather than silently tracking them.
constexpr float kMinCommandableSpeed = 0.03f / 0.002f;   // [mm/s] == 15.0

}  // namespace

int main() {
  std::printf("=== Deadband-boosted terminal correction (sprint 114 ticket 005) ===\n\n");

  // --- Scenario 1: a real pivot's own terminal correction (naturally
  //     sub-deadband under the bench config) completes well inside the ~8s
  //     backstop, via ACK_STATUS_DONE, never ACK_STATUS_TIMEOUT. ---
  {
    beginScenario(
        "a real pivot's terminal correction (sub-deadband per bench config) completes "
        "well inside the 8s backstop, not via timeout");

    TestSim::SimHarness sim;
    TestSupport::configureSimForBenchTest(sim);
    sim.boot();

    constexpr float kDeltaHeading = 30.0f * kDegToRad;
    constexpr uint32_t kMoveId = 1;
    sim.injectMove(/*distance=*/0.0f, kDeltaHeading, /*vMax=*/0.0f, /*omega=*/0.0f,
                   /*timeMs=*/0.0f, /*replace=*/false, /*id=*/kMoveId, /*corrId=*/1);

    // Budget: 100 cycles * 50ms/cycle (SimHarness::kCycleDtUs) == 5s of
    // virtual time -- comfortably above any legitimate completion time for
    // a 30deg pivot (profile duration well under 1s at this bench config's
    // yaw_rate_max/yaw_acc_max, plus PD settle plus the 150ms arrive_dwell
    // hold), comfortably BELOW the ~8s backstop, so a run that only
    // "completes" by riding out to the backstop fails this budget outright
    // instead of silently passing late.
    constexpr int kMaxCycles = 100;
    bool sawAckDone = false;
    bool sawAckTimeout = false;
    bool sawNonzeroMeasuredVelocityWhileSubDeadband = false;
    int completionCycle = -1;

    for (int i = 0; i < kMaxCycles && !sawAckDone && !sawAckTimeout; ++i) {
      sim.step(1);

      // The terminal-correction differential this scenario targets: near
      // completion the planned per-wheel reference has already decayed
      // toward 0 (the jerk-limited profile's own ramp-down), so the
      // COMMANDED wheel speed is dominated by the heading PD's own trim --
      // exactly the residual under test.
      float commandedLeft = sim.driveTargetVelLeft();
      bool subDeadbandInFlight =
          std::fabs(commandedLeft) > 0.5f && std::fabs(commandedLeft) < kMinCommandableSpeed;

      for (const auto& line : sim.drainTelemetry()) {
        if (line.kind != TestSupport::DecodedKind::kTelemetry) continue;

        if (subDeadbandInFlight && line.telemetry.has_vel &&
            (std::fabs(line.telemetry.vel_left) > 1.0f || std::fabs(line.telemetry.vel_right) > 1.0f)) {
          sawNonzeroMeasuredVelocityWhileSubDeadband = true;
        }

        for (uint8_t a = 0; a < line.telemetry.acks_count; ++a) {
          if (line.telemetry.acks_[a].corr_id != kMoveId) continue;
          if (line.telemetry.acks_[a].status == msg::AckStatus::ACK_STATUS_DONE) {
            sawAckDone = true;
            completionCycle = i;
          } else if (line.telemetry.acks_[a].status == msg::AckStatus::ACK_STATUS_TIMEOUT) {
            sawAckTimeout = true;
            completionCycle = i;
          }
        }
      }
    }

    checkTrue(sawAckDone, "the pivot reached ACK_STATUS_DONE within " + std::to_string(kMaxCycles) +
                              " cycles (" +
                              std::to_string(kMaxCycles * TestSim::SimHarness::kCycleDtUs / 1000) +
                              "ms virtual time) -- not stalled at the deadband, not the ~8s arrive-timeout");
    checkTrue(!sawAckTimeout, "the pivot never reached ACK_STATUS_TIMEOUT");
    checkTrue(sawNonzeroMeasuredVelocityWhileSubDeadband,
              "measured wheel velocity was nonzero while the commanded correction was genuinely "
              "sub-deadband -- the boost is actually engaged in this scenario, not merely absent");

    if (completionCycle >= 0) {
      std::printf("  completed at cycle %d (~%dms virtual time)\n", completionCycle,
                  completionCycle * static_cast<int>(TestSim::SimHarness::kCycleDtUs / 1000));
    }
  }

  // --- Scenario 2: a directly-injected ~11mm/s wheel-differential TWIST
  //     produces nonzero MEASURED velocity within one tick of the boosted
  //     write landing, and does not decay back toward 0 once the initial
  //     error-driven transient fades (steady-state feedforward alone,
  //     kff*11mm/s ~= 0.022 duty, is itself sub-deadband -- the ONLY thing
  //     keeping the command above outputDeadband_ once the transient decays
  //     is the boost). ---
  {
    beginScenario("an ~11mm/s wheel-differential TWIST produces nonzero measured velocity within one tick");

    TestSim::SimHarness sim;
    TestSupport::configureSimForBenchTest(sim);
    sim.boot();
    sim.step(3);   // settle past boot

    checkTrue(sim.motorLeft().velocity() == 0.0f && sim.motorRight().velocity() == 0.0f,
              "setup: both wheels start at rest");

    // Pure rotation twist -- BodyKinematics::inverse() maps omega to a
    // symmetric +/- per-wheel differential of omega*trackWidth/2. Picks the
    // SAME ~11mm/s the hardware trace found frozen; kDefaultTrackWidth is
    // SimHarness's own default (128mm, sim_plant.h).
    constexpr float kResidual = 11.0f;   // [mm/s]
    const float halfTrack = TestSim::kDefaultTrackWidth / 2.0f;
    const float omega = kResidual / halfTrack;   // [rad/s]
    // duration is msg::Twist.duration's own wire unit -- [ms], the Deadman
    // ARM window (app/deadman.h), NOT a scenario-runtime bound. 100000ms
    // (100s) matches every other injectTwist() call in this test domain
    // (scripted_twist_demo_harness.cpp/straight_twist_harness.cpp) --
    // comfortably longer than this scenario's own ~30-cycle (1.5s) run, so
    // the twist stays live throughout rather than expiring after one cycle.
    sim.injectTwist(/*v_x=*/0.0f, omega, /*duration=*/100000.0f, /*corrId=*/2);

    // Poll for dispatch rather than assume a fixed cycle count: a SECOND
    // TestSim::SimHarness constructed in the same process as an earlier one
    // (scenario 1 above) can take a few extra cycles for FakeTransport/
    // App::Comms to actually dispatch an injected command -- an unrelated,
    // pre-existing cross-instance latency quirk in the test harness
    // plumbing (confirmed by direct comparison: a lone SimHarness dispatches
    // on the very first cycle; a second one in-process can take 1-2 more),
    // not anything to do with this ticket's own fix. Anchoring "within one
    // tick" to when the write actually LANDS (driveTargetVelLeft reflects
    // the commanded differential), rather than to an assumed dispatch cycle,
    // keeps this assertion honest about what it is actually proving.
    int dispatchCycle = -1;
    for (int i = 0; i < 10 && dispatchCycle < 0; ++i) {
      sim.step(1);
      if (std::fabs(sim.driveTargetVelLeft()) > 1.0f) dispatchCycle = i;
    }
    checkTrue(dispatchCycle >= 0,
              "the TWIST was dispatched (driveTargetVelLeft reflects the commanded differential) "
              "within 10 cycles");

    sim.step(1);   // one further tick for SimPlant's own physics to integrate the just-landed write
    checkTrue(std::fabs(sim.motorLeft().velocity()) > 0.5f,
              "left wheel: measured velocity is nonzero within one tick of the boosted write landing "
              "(would stay pinned at 0.000 under the old zeroing dead zone)");
    checkTrue(std::fabs(sim.motorRight().velocity()) > 0.5f, "right wheel: same");

    // Steady state: once the initial kp*err transient decays, the ONLY
    // thing keeping the command above outputDeadband_ is the boost -- the
    // pure feedforward term alone (kff*11mm/s ~= 0.022 duty) is itself
    // sub-deadband. Confirms the wheel does NOT decay back toward 0 the way
    // the old single-branch dead zone would once the transient faded.
    sim.step(30);
    checkTrue(std::fabs(sim.motorLeft().velocity()) > 5.0f,
              "left wheel: settles near the boosted floor, does not decay back to 0 once the "
              "kp*err transient fades (steady-state feedforward alone is sub-deadband)");
    checkTrue(std::fabs(sim.motorRight().velocity()) > 5.0f, "right wheel: same");
  }

  std::printf("\n");
  if (g_failureCount == 0) {
    std::printf("OK: all deadband terminal-correction scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the deadband terminal-correction scenarios\n",
              g_failureCount);
  return 1;
}
