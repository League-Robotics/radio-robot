// pose_ownership_test.cpp -- SUC-002 / 128-016 regression
// (robot-state-pose-needs-exactly-one-writer.md): Motion::Odometry::
// integrate() is Types::RobotState::pose's ONE writer. Before this
// ticket, Planner::update() ALSO wrote state.pose from its own internal
// PoseTracker (`pose_`), which blends a fresh OTOS heading whenever
// limits_.headingOtosWeight > 0 -- a second writer, ordering-dependent on
// which of Odometry/Planner ran last a given cycle (App::RobotLoop's own
// publishPose() vs. planner_.update(), robot_loop.cpp). At the shipped
// default (headingOtosWeight == 0.0) the two writers happened to agree
// closely enough that the bug was invisible; a nonzero weight made
// telemetry's pose source silently flip.
//
// This test drives the SAME per-cycle order App::RobotLoop::cycle() uses
// -- Odometry integrates and publishes pose FIRST, the Planner
// tick()s/update()s SECOND, both against one shared Types::RobotState --
// with a fresh OTOS heading present every cycle, clearly divergent from
// the encoder-only heading, and headingOtosWeight configured > 0 (the
// exact hazard this ticket closes). It asserts state.pose stays
// bit-for-bit what Odometry alone computed, cycle after cycle, proving
// Planner::update() no longer touches it at all.
//
// Odometry/BodyKinematics are not linked into the standalone `planner`
// library (this directory's own CMakeLists.txt) -- this test target adds
// odometry.cpp/body_kinematics.cpp explicitly (see this directory's
// CMakeLists.txt PLANNER_TESTS-with-extra-sources handling).
#include <cstdio>

#include "motion/odometry.h"
#include "planner.h"
#include "tests/test_support.h"

using Motion::Move;
using Motion::Odometry;
using Motion::Planner;
using Motion::PlannerLimits;
using TestPlanner::PerfectPlant;

namespace {

constexpr float kPeriod = 50.0f;       // [ms]
constexpr float kTrackWidth = 100.0f;  // [mm]

Move distanceMove(uint32_t id, float threshold, float v_x) {
  Move m;
  m.id = id;
  m.kind = Move::Kind::Distance;
  m.threshold = threshold;
  m.v_x = v_x;
  m.timeout = 60000.0f;
  return m;
}

// Mirrors App::RobotLoop::publishPose()'s own write, minus the
// BodyKinematics::forward() twist (not needed for this test's
// pose-ownership assertion, and pulling body_kinematics.h in here would
// add a messages/ dependency this standalone planner test tree otherwise
// has none of).
void publishPose(Types::RobotState& state, const Odometry& odom) {
  state.pose.x = odom.x();
  state.pose.y = odom.y();
  state.pose.heading = odom.theta();
}

void testPlannerNeverWritesStatePoseEvenWithOtosBlendConfigured() {
  PlannerLimits limits;
  limits.vMax = 600.0f;
  limits.aMax = 400.0f;
  limits.aDecel = 300.0f;
  limits.omegaMax = 8.0f;
  limits.alphaMax = 12.0f;
  limits.alphaDecel = 10.0f;
  limits.trackWidth = kTrackWidth;
  limits.controlPeriod = kPeriod;
  limits.otosStaleness = 200;
  // The exact hazard this ticket closes: a nonzero blend weight used to
  // make Planner::update() overwrite state.pose with pose_'s OTOS-blended
  // heading instead of leaving Odometry's write alone.
  limits.headingOtosWeight = 0.6f;

  Planner planner(limits);
  Odometry odom(kTrackWidth, 0.0f, 0.0f);
  PerfectPlant plant;
  Types::RobotState state;
  uint32_t now = 0;

  CHECK(planner.move(distanceMove(1, 2000.0f, 150.0f), false));

  for (int i = 0; i < 200; ++i) {
    // --- Odometry FIRST, exactly App::RobotLoop::cycle()'s own order. ---
    odom.integrate(state.wheelLeft.position, state.wheelRight.position);
    publishPose(state, odom);

    // A fresh, clearly divergent OTOS heading every cycle -- present and
    // within staleness every tick, so the (deleted) blend path would have
    // fired on every single cycle if it still existed.
    state.otos.present = true;
    state.otos.heading = 1.0f;  // [rad] -- straight-line odometry heading is ~0
    state.otos.sampleTime = now;

    // --- Planner SECOND. ---
    state.time.cycleStart = now;
    planner.tick(state);
    planner.update(state);

    // state.pose must be EXACTLY what Odometry alone just computed --
    // Planner::update() must not have touched it, regardless of the
    // configured OTOS blend weight.
    CHECK(state.pose.x == odom.x());
    CHECK(state.pose.y == odom.y());
    CHECK(state.pose.heading == odom.theta());

    now += static_cast<uint32_t>(kPeriod);
    plant.step(state, kPeriod * 0.001f, now);
  }
}

}  // namespace

int main() {
  testPlannerNeverWritesStatePoseEvenWithOtosBlendConfigured();
  std::printf("pose_ownership_test: all checks passed\n");
  return 0;
}
