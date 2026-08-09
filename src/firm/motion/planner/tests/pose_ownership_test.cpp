// pose_ownership_test.cpp -- SUC-002 / 128-016 regression
// (robot-state-pose-needs-exactly-one-writer.md): Motion::Odometry::
// integrate() is Types::RobotState::pose's ONE writer. Before that
// ticket, Planner::update() ALSO wrote state.pose from its own internal
// PoseTracker (`pose_`), which used to blend a fresh OTOS heading
// whenever `PlannerLimits::headingOtosWeight > 0` -- a second writer,
// ordering-dependent on which of Odometry/Planner ran last a given cycle
// (App::RobotLoop's own publishPose() vs. planner_.update(),
// robot_loop.cpp). At the shipped default (headingOtosWeight == 0.0) the
// two writers happened to agree closely enough that the bug was
// invisible; a nonzero weight made telemetry's pose source silently flip.
//
// 130-009 deletes `headingOtosWeight`/`otosStaleness` (and the blend call
// site in Planner::tick() they gated) from PlannerLimits outright -- the
// feature was live code but configured off in every robot JSON, and the
// sprint scoped OTOS-heading fusion out to a from-scratch estimator-v2
// design (clasi/issues/later/estimator-v2-otos-fusion-sim-first.md)
// rather than reshaping this ad hoc blend. There is consequently no way
// to construct "OTOS blend configured" any more; this test keeps the
// underlying invariant it always proved -- Planner::update() never
// writes state.pose, regardless of what state.otos carries -- which is
// now true unconditionally rather than only at the shipped default.
//
// This test drives the SAME per-cycle order App::RobotLoop::cycle() uses
// -- Odometry integrates and publishes pose FIRST, the Planner
// tick()s/update()s SECOND, both against one shared Types::RobotState --
// with a fresh OTOS heading present every cycle, clearly divergent from
// the encoder-only heading, so a resurrected blend call site would be
// caught immediately. It asserts state.pose stays bit-for-bit what
// Odometry alone computed, cycle after cycle, proving Planner::update()
// never touches it.
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
// Kinematics::DifferentialKinematics::forward() twist (not needed for this test's
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
  limits.ceilings.vMax = 600.0f;
  limits.ceilings.aMax = 400.0f;
  limits.ceilings.aDecel = 300.0f;
  limits.ceilings.omegaMax = 8.0f;
  limits.ceilings.alphaMax = 12.0f;
  limits.ceilings.alphaDecel = 10.0f;
  limits.plant.trackWidth = kTrackWidth;
  limits.plant.controlPeriod = kPeriod;
  // No headingOtosWeight/otosStaleness to configure any more -- 130-009
  // deleted both fields along with the blend call site they gated (see
  // this file's own header). The fresh, clearly-divergent OTOS heading
  // fed into state.otos below is what used to make a nonzero blend
  // weight overwrite state.pose; it is retained here as a belt-and-braces
  // check that no blend of any kind fires, configured or not.

  Planner planner(limits);
  Odometry odom(kTrackWidth, 0.0f, 0.0f);
  PerfectPlant plant;
  Types::RobotState state;
  uint32_t now = 0;

  CHECK(planner.move(distanceMove(1, 2000.0f, 150.0f), false));

  for (int i = 0; i < 200; ++i) {
    // --- Odometry FIRST, exactly App::RobotLoop::cycle()'s own order. ---
    odom.integrate(state.wheelLeft.position, state.wheelRight.position,
                   state.wheelLeft.positionEpoch, state.wheelRight.positionEpoch);
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
