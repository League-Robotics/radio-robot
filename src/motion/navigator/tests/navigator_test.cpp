// navigator_test.cpp -- ctest coverage for Motion::Navigator (135-003).
// Constructs a REAL Motion::Planner (never a mock) and drives it through
// Motion::Navigator against a hand-scripted or ideal-plant Types::
// RobotState sequence, per the ticket's own Testing section. Each test
// names the acceptance criterion it exercises in its own title comment.
#include <cmath>
#include <cstdio>

#include "navigator.h"
#include "planner.h"
#include "test_support.h"

using Motion::GotoTarget;
using Motion::Move;
using Motion::MoveLifecycle;
using Motion::Navigator;
using Motion::NavigatorLimits;
using Motion::NavResult;
using Motion::Planner;
using TestNav::cycle;
using TestNav::defaultNavLimits;
using TestNav::defaultPlannerLimits;
using TestNav::IdealPlant;

namespace {

constexpr float kPeriod = 50.0f;  // [ms] firmware control-loop cadence
constexpr double kPi = 3.14159265358979323846;

// --- SUC-001: converges from rest, at rest, within tolerance -----------

void testConvergesFromRestAndSettles() {
  Planner planner(defaultPlannerLimits());
  NavigatorLimits limits = defaultNavLimits();
  Navigator nav(limits, planner);
  IdealPlant plant;
  plant.trackWidth = limits.trackWidth;
  Types::RobotState state{};
  uint32_t now = 0;

  GotoTarget target;
  target.id = 100;
  target.x = 1000.0f;
  target.y = 300.0f;
  target.tolerance = 20.0f;
  nav.start(target);

  bool done = false;
  bool fault = false;
  for (int i = 0; i < 600 && !done; ++i) {
    const NavResult r = cycle(nav, state, plant, now, kPeriod);
    if (r.completed) {
      done = true;
      fault = r.fault;
    }
  }

  CHECK(done);
  CHECK(!fault);
  CHECK(!nav.active());  // "Drive ownership released" signal, ticket 004
  const float distance = std::hypot(target.x - plant.x, target.y - plant.y);
  CHECK(distance <= target.tolerance + 5.0f);
  CHECK(std::fabs(planner.commandedLeft()) < 5.0f);
  CHECK(std::fabs(planner.commandedRight()) < 5.0f);
}

// --- Velocity continuity, asserted numerically --------------------------
//
// Mirrors solver.py's own discontinuity guard (CASE2_EDGE_B_DISCONTINUITY_
// MM_S = 433.3333 was the FAILURE mode being guarded against). The bound
// asserted here is the SAME budget arc_solver.h's own curvature slew
// clamp is derived from (kArcSolverMaxWheelStepDefault, the max per-wheel
// step ONE replace may induce) plus the profiler's own per-tick accel
// ceiling (PlannerLimits::Ceilings::aMax * one control period) -- i.e. the
// worst case where a replace's curvature-reinterpretation step and the
// profiler's own accel step land in the SAME tick. Comfortably below the
// measured 433.3 mm/s hazard figure.
void testVelocityContinuityBounded() {
  const Motion::PlannerLimits plannerLimits = defaultPlannerLimits();
  Planner planner(plannerLimits);
  NavigatorLimits limits = defaultNavLimits();
  Navigator nav(limits, planner);
  IdealPlant plant;
  plant.trackWidth = limits.trackWidth;
  Types::RobotState state{};
  uint32_t now = 0;

  GotoTarget target;
  target.id = 101;
  target.x = 3000.0f;
  target.y = 1200.0f;  // bearing ~21.8 deg -- under turnFirstAngle (~50 deg)
  target.tolerance = 20.0f;
  nav.start(target);

  const float bound =
      limits.maxWheelStep + plannerLimits.ceilings.aMax * (kPeriod / 1000.0f) + 1.0f;

  float prevLeft = 0.0f;
  float prevRight = 0.0f;
  bool done = false;
  bool fault = false;
  // Target distance is ~3231 mm at a 150 mm/s cruise -- ~21.5 s ideal, so
  // 800 ticks (40 s) leaves generous margin for the accel/decel ramps and
  // the arc's own curvature without the loop itself timing out first.
  for (int i = 0; i < 800 && !done; ++i) {
    const NavResult r = cycle(nav, state, plant, now, kPeriod);
    CHECK_NEAR(planner.commandedLeft(), prevLeft, bound);
    CHECK_NEAR(planner.commandedRight(), prevRight, bound);
    prevLeft = planner.commandedLeft();
    prevRight = planner.commandedRight();
    if (r.completed) {
      done = true;
      fault = r.fault;
    }
  }

  CHECK(done);
  CHECK(!fault);
}

// --- Material-change throttle: does NOT replace every tick -------------

void testMaterialChangeThrottleNotEveryTick() {
  Planner planner(defaultPlannerLimits());
  NavigatorLimits limits = defaultNavLimits();
  limits.speed = 20.0f;  // slow: 1 mm of travel per 50 ms tick
  Navigator nav(limits, planner);

  Types::RobotState state{};
  state.otos.present = true;
  state.otos.connected = true;
  state.otos.heading = 0.0f;  // wire sign; encoder-sign heading = 0, facing +x

  GotoTarget target;
  target.id = 102;
  target.x = 5000.0f;  // far ahead, exactly on-heading throughout
  target.y = 0.0f;
  target.tolerance = 10.0f;
  nav.start(target);

  uint32_t now = 0;
  const int kTicks = 200;
  const float perTick = limits.speed * (kPeriod / 1000.0f);  // [mm] 1.0 mm/tick
  float traveled = 0.0f;
  for (int i = 0; i < kTicks; ++i) {
    state.time.cycleStart = now;
    state.otos.x = traveled;
    state.otos.y = 0.0f;
    state.otos.sampleTime = now;  // fresh every cycle
    nav.tick(state);
    traveled += perTick;
    now += static_cast<uint32_t>(kPeriod);
  }

  CHECK(nav.tickCount() == static_cast<uint32_t>(kTicks));
  // arcLengthThreshold (15 mm) / 1 mm-per-tick drift -> a replace roughly
  // every 15 ticks; over 200 ticks that is on the order of a dozen, far
  // fewer than one per tick.
  CHECK(nav.replaceCount() >= 1);
  CHECK(nav.replaceCount() < nav.tickCount() / 4);
}

// --- Mandatory half-arc refresh, independent of material change --------
//
// Constructs a target the robot orbits at a CONSTANT distance and body-
// frame bearing (a circle centred on the target, with heading rotating in
// lockstep) -- by this construction, EVERY solve returns byte-identical
// omega/arcLength (both are pure functions of distance and bearing, which
// never change), so the material-change throttle can never be what fires
// a replace. The robot's WORLD position still moves substantially around
// the circle, so the half-arc-consumed refresh is the ONLY mechanism that
// can explain any replace after the first (bootstrap) one.
void testHalfArcMandatoryRefreshFiresWithoutMaterialChange() {
  Planner planner(defaultPlannerLimits());
  NavigatorLimits limits = defaultNavLimits();
  limits.maxWheelStep = 1.0e9f;  // disable the slew clamp -- see the header
                                  // comment above for why this test wants
                                  // the raw, unclamped geometry from the
                                  // very first solve.
  Navigator nav(limits, planner);

  Types::RobotState state{};
  state.otos.present = true;
  state.otos.connected = true;

  const float kR = 10000.0f;                                     // [mm]
  const float kBearing = static_cast<float>(20.0 * kPi / 180.0);  // [rad]

  GotoTarget target;
  target.id = 103;
  target.x = 0.0f;
  target.y = 0.0f;
  target.tolerance = 50.0f;  // << kR: never "arrives" in this test window
  nav.start(target);

  uint32_t now = 0;
  const int kTicks = 400;
  const float totalSweep = static_cast<float>(2.0 * kPi / 3.0);  // 120 deg
  const float dTheta = totalSweep / static_cast<float>(kTicks);

  for (int i = 0; i < kTicks; ++i) {
    const float theta = static_cast<float>(i) * dTheta;
    const float x = kR * std::cos(theta);
    const float y = kR * std::sin(theta);
    const float heading = (theta + static_cast<float>(kPi)) - kBearing;

    state.time.cycleStart = now;
    state.otos.x = x;
    state.otos.y = y;
    state.otos.heading = -heading;  // wire/hardware-mounted sign, ticket 008
    state.otos.sampleTime = now;
    nav.tick(state);
    now += static_cast<uint32_t>(kPeriod);
  }

  CHECK(nav.active());  // never froze/arrived -- distance stayed == kR throughout
  CHECK(nav.replaceCount() >= 3);
  CHECK(nav.replaceCount() <= 8);
  CHECK(nav.replaceCount() * 4 < nav.tickCount());  // still materially fewer than every tick
}

// --- SUC-004: target behind -> stop-then-pivot-then-arc ----------------

void testTargetBehindStopThenPivotThenArc() {
  Planner planner(defaultPlannerLimits());
  NavigatorLimits limits = defaultNavLimits();
  Navigator nav(limits, planner);
  IdealPlant plant;
  plant.trackWidth = limits.trackWidth;
  Types::RobotState state{};
  uint32_t now = 0;

  GotoTarget target1;
  target1.id = 1;
  target1.x = 300.0f;
  target1.y = 0.0f;
  target1.tolerance = 20.0f;
  nav.start(target1);

  // A few cycles so the robot is genuinely moving (Planner has an ACTIVE
  // Move) before the redirect below -- this is what makes the redirect
  // exercise the plannedStop()-queues-behind-the-in-flight-arc path
  // rather than the skip-straight-to-pivot "already at rest" path.
  for (int i = 0; i < 4; ++i) cycle(nav, state, plant, now, kPeriod);
  CHECK(planner.active());

  // Redirect to a target BEHIND the robot's current heading (heading is
  // still ~0, facing +x -- a target at -x from here is >90 deg behind,
  // past even the behind-guard, let alone turnFirstAngle).
  GotoTarget target2;
  target2.id = 2;
  target2.x = plant.x - 500.0f;
  target2.y = plant.y;
  target2.tolerance = 20.0f;
  nav.start(target2);

  bool sawStopping = false;
  bool done = false;
  bool fault = false;
  for (int i = 0; i < 400 && !done; ++i) {
    if (planner.lifecycle() == MoveLifecycle::Stopping) sawStopping = true;
    const NavResult r = cycle(nav, state, plant, now, kPeriod);
    if (r.completed) {
      done = true;
      fault = r.fault;
    }
  }

  CHECK(sawStopping);  // the plannedStop() sequencing genuinely engaged
  CHECK(done);
  CHECK(!fault);
  const float distance = std::hypot(target2.x - plant.x, target2.y - plant.y);
  CHECK(distance <= target2.tolerance + 5.0f);
}

// --- Small bearing, just under turnFirstAngle: no pivot, no oscillation -

void testSmallBearingNoPivotNoOscillation() {
  Planner planner(defaultPlannerLimits());
  NavigatorLimits limits = defaultNavLimits();
  limits.maxWheelStep = 1.0e9f;
  Navigator nav(limits, planner);

  Types::RobotState state{};
  state.otos.present = true;
  state.otos.connected = true;

  const float kR = 10000.0f;
  const float kBearing = limits.turnFirstAngle - 0.02f;  // just under the threshold

  GotoTarget target;
  target.id = 104;
  target.x = 0.0f;
  target.y = 0.0f;
  target.tolerance = 50.0f;
  nav.start(target);

  uint32_t now = 0;
  const int kTicks = 300;
  const float totalSweep = static_cast<float>(kPi);  // 180 deg, generous
  const float dTheta = totalSweep / static_cast<float>(kTicks);
  bool sawStopping = false;

  for (int i = 0; i < kTicks; ++i) {
    const float theta = static_cast<float>(i) * dTheta;
    const float x = kR * std::cos(theta);
    const float y = kR * std::sin(theta);
    const float heading = (theta + static_cast<float>(kPi)) - kBearing;

    state.time.cycleStart = now;
    state.otos.x = x;
    state.otos.y = y;
    state.otos.heading = -heading;
    state.otos.sampleTime = now;
    nav.tick(state);
    if (planner.lifecycle() == MoveLifecycle::Stopping) sawStopping = true;
    now += static_cast<uint32_t>(kPeriod);
  }

  CHECK(!sawStopping);  // never entered the pivot machinery at all
  CHECK(nav.active());
}

// --- SUC-005: a single stale-but-connected cycle skips re-solving ------

void testOtosStalenessSkipsResolve() {
  Planner planner(defaultPlannerLimits());
  NavigatorLimits limits = defaultNavLimits();
  Navigator nav(limits, planner);

  Types::RobotState state{};
  state.otos.present = true;
  state.otos.connected = true;
  state.otos.heading = 0.0f;

  GotoTarget target;
  target.id = 105;
  target.x = 5000.0f;
  target.y = 0.0f;
  target.tolerance = 10.0f;
  nav.start(target);

  uint32_t now = 0;

  // Bootstrap: the first tick always issues unconditionally.
  state.time.cycleStart = now;
  state.otos.x = 0.0f;
  state.otos.sampleTime = now;
  nav.tick(state);
  now += static_cast<uint32_t>(kPeriod);
  CHECK(nav.replaceCount() == 1);

  // A STALE cycle (sampleTime unchanged) with a pose that would otherwise
  // be an obviously material change -- must NOT replace.
  state.time.cycleStart = now;
  state.otos.x = 500.0f;  // sampleTime left untouched -- stale
  nav.tick(state);
  now += static_cast<uint32_t>(kPeriod);
  CHECK(nav.replaceCount() == 1);

  // A FRESH cycle afterward can replace again -- the skip was one cycle,
  // not a permanent freeze.
  state.time.cycleStart = now;
  state.otos.x = 500.0f;
  state.otos.sampleTime = now;
  nav.tick(state);
  CHECK(nav.replaceCount() >= 2);
}

// --- SUC-005: a sustained disconnect aborts within the bounded window --

void testOtosDisconnectAbortsWithinWindow() {
  Planner planner(defaultPlannerLimits());
  NavigatorLimits limits = defaultNavLimits();
  Navigator nav(limits, planner);
  IdealPlant plant;
  plant.trackWidth = limits.trackWidth;
  Types::RobotState state{};
  uint32_t now = 0;

  GotoTarget target;
  target.id = 106;
  target.x = 5000.0f;
  target.y = 0.0f;
  target.tolerance = 10.0f;
  nav.start(target);

  // A few connected cycles to establish a real fix.
  for (int i = 0; i < 5; ++i) cycle(nav, state, plant, now, kPeriod);
  CHECK(nav.active());

  bool done = false;
  bool fault = false;
  int disconnectedTicks = 0;
  for (int i = 0; i < 100 && !done; ++i) {
    state.time.cycleStart = now;
    state.otos.connected = false;  // force the dropout for THIS tick's read
    const NavResult r = nav.tick(state);
    ++disconnectedTicks;
    if (r.completed) {
      done = true;
      fault = r.fault;
    }
    now += static_cast<uint32_t>(kPeriod);
    plant.step(state, kPeriod / 1000.0f, now);  // re-marks connected=true; overridden again next loop
  }

  CHECK(done);
  CHECK(fault);           // Aborted, not Done
  CHECK(!nav.active());   // Drive ownership released
  // Bounded window == Motion::kNavOtosDisconnectAbortWindow (1000 ms) ==
  // 20 ticks at 50 ms -- must fire at/soon after that, never immediately
  // and never long after.
  CHECK(disconnectedTicks >= 20);
  CHECK(disconnectedTicks <= 25);
}

// --- Exactly one completion ack per goto id -----------------------------

void testExactlyOneCompletionAck() {
  Planner planner(defaultPlannerLimits());
  NavigatorLimits limits = defaultNavLimits();
  Navigator nav(limits, planner);
  IdealPlant plant;
  plant.trackWidth = limits.trackWidth;
  Types::RobotState state{};
  uint32_t now = 0;

  GotoTarget target;
  target.id = 107;
  target.x = 800.0f;
  target.y = 0.0f;
  target.tolerance = 20.0f;
  nav.start(target);

  int completions = 0;
  uint32_t completedId = 0;
  for (int i = 0; i < 600; ++i) {  // run well past arrival -- must still be exactly one
    const NavResult r = cycle(nav, state, plant, now, kPeriod);
    if (r.completed) {
      ++completions;
      completedId = r.id;
    }
  }

  CHECK(completions == 1);
  CHECK(completedId == target.id);
}

}  // namespace

int main() {
  testConvergesFromRestAndSettles();
  testVelocityContinuityBounded();
  testMaterialChangeThrottleNotEveryTick();
  testHalfArcMandatoryRefreshFiresWithoutMaterialChange();
  testTargetBehindStopThenPivotThenArc();
  testSmallBearingNoPivotNoOscillation();
  testOtosStalenessSkipsResolve();
  testOtosDisconnectAbortsWithinWindow();
  testExactlyOneCompletionAck();
  std::printf("PASS navigator_test (9 checks)\n");
  return 0;
}
