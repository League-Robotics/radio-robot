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

// --- Landmine 2 (135-004): a pivot's own terminal fine-align (MoveLifecycle
// ::Aligning) must not block the Navigator's next replace for ~2s --------
//
// defaultPlannerLimits() leaves landing.alignTol/alignMaxNudges at 0
// (disabled) -- exactly why ticket 003's own
// testTargetBehindStopThenPivotThenArc above never exercised this path
// (per this ticket's own text: "it may not have exercised the real
// Aligning path if its scripted RobotState never satisfied
// alignApplies()"). This test explicitly enables it with tovez.json's own
// tuned values (data/robots/tovez.json's planner.align_tol/
// align_max_nudges) to prove the fix, not just assert it.
void testPivotThenCruiseNotBlockedBehindAligning() {
  Motion::PlannerLimits plannerLimits = defaultPlannerLimits();
  plannerLimits.landing.alignTol = 0.017453f;  // [rad] ~1 deg, tovez.json's own value
  plannerLimits.landing.alignMaxNudges = 6;

  Planner planner(plannerLimits);
  NavigatorLimits limits = defaultNavLimits();
  Navigator nav(limits, planner);
  IdealPlant plant;
  plant.trackWidth = limits.trackWidth;
  Types::RobotState state{};
  uint32_t now = 0;

  // Target behind the robot's heading (0, facing +x) -- forces a pivot.
  // The robot starts at rest, so beginPivotSequence() issues the pivot
  // directly (Pivoting phase) -- no Stopping phase to wait through first.
  GotoTarget target;
  target.id = 1;
  target.x = -500.0f;
  target.y = 0.0f;
  target.tolerance = 20.0f;
  nav.start(target);

  int alignEnteredTick = -1;
  int cruiseReplaceTick = -1;
  for (int i = 0; i < 200 && cruiseReplaceTick < 0; ++i) {
    if (alignEnteredTick < 0 && planner.lifecycle() == MoveLifecycle::Aligning) {
      alignEnteredTick = i;
    }
    cycle(nav, state, plant, now, kPeriod);
    // replaceCount() only ever increments from the CRUISE mustIssue block
    // (navigator.cpp) -- the pivot's own issue is a different, uncounted
    // event (navigator.h's own replaceCount() doc comment) -- so the first
    // tick it goes nonzero is unambiguously the first cruise re-issue.
    if (alignEnteredTick >= 0 && nav.replaceCount() > 0) {
      cruiseReplaceTick = i;
    }
  }

  CHECK(alignEnteredTick >= 0);   // the pivot genuinely landed into Aligning
  CHECK(cruiseReplaceTick >= 0);  // the Navigator did replace with the cruise arc
  const int ticksToReplace = cruiseReplaceTick - alignEnteredTick;
  std::printf("  LANDMINE2_ALIGN_ENTERED_TICK=%d CRUISE_REPLACE_TICK=%d DELTA_TICKS=%d "
              "(bound: <=3, i.e. <=%.0fms; Aligning's own settle-and-trim budget is ~2000ms)\n",
              alignEnteredTick, cruiseReplaceTick, ticksToReplace,
              3.0f * kPeriod);
  // The whole point: the replace must land within a HANDFUL of cycles of
  // Aligning beginning, not after Aligning's own ~2s settle-and-trim
  // budget (up to alignMaxNudges nudges, each with its own settle window)
  // has run its course.
  CHECK(ticksToReplace >= 0);
  CHECK(ticksToReplace <= 3);
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

// --- Landmine 4 (135-004): NavigatorLimits::yawSign converts worldPose's
// own (true-world) omega convention to the wire's Move::omega convention,
// matching goto_otos.py's own YAW_SIGN = -1.0 exactly -- see navigator.h's
// own comment for the full derivation. Deliberately picks a raw heading
// that is NEITHER 0 nor 90 deg: both degenerate cases numerically hide a
// sign error that a generic heading exposes (0 deg: worldPose.heading ==
// pose.heading trivially, since -0 == 0; 90 deg: cos(heading) == 0
// trivially satisfies sin(A+B) == -sin(A-B) regardless of whether the
// underlying transform is otherwise correct -- both traps were hit and
// caught during this ticket's own derivation, see Completion Notes).
void testYawSignMatchesGotoOtosConvention() {
  Planner planner(defaultPlannerLimits());
  NavigatorLimits limits = defaultNavLimits();
  limits.yawSign = -1.0f;  // matches goto_otos.py's measured YAW_SIGN
  Navigator nav(limits, planner);

  Types::RobotState state{};
  state.otos.present = true;
  state.otos.connected = true;
  // Raw, un-negated wire heading == true-world CCW convention (ticket 008
  // settled this -- navigator.h's own OTOS sign convention comment).
  state.otos.heading = static_cast<float>(30.0 * kPi / 180.0);  // 30 deg
  state.otos.x = 0.0f;
  state.otos.y = 0.0f;
  state.otos.sampleTime = 1;

  // Target chosen so goto_otos.py's OWN solve_arc() + YAW_SIGN formula --
  // computed independently in Python, not re-derived here -- gives a
  // KNOWN, non-trivial NEGATIVE wire omega for this exact (heading,
  // target) pair: raw heading 30 deg, target bearing 50 deg (a 20 deg
  // world-frame correction), distance 1000 mm:
  //   error = 20 deg; omega_world = +0.10261 rad/s;
  //   wire_omega = YAW_SIGN * omega_world = -0.10261 rad/s.
  GotoTarget target;
  target.id = 1;
  target.x = 642.788f;   // 1000 * cos(50 deg)
  target.y = 766.044f;   // 1000 * sin(50 deg)
  target.tolerance = 20.0f;
  nav.start(target);

  // No plant: state.otos is held fixed on purpose (this checks Navigator's
  // ONE-SHOT computation against an external reference, not closed-loop
  // convergence -- see navigator.h's own comment on why NO plant, ideal
  // or real-sim, can validate this sign choice via convergence alone).
  // Time still advances so the Planner's own profiler ramps the staged
  // Move up from rest.
  uint32_t now = 0;
  for (int i = 0; i < 10; ++i) {
    state.time.cycleStart = now;
    nav.tick(state);
    now += static_cast<uint32_t>(kPeriod);
  }

  // wire Move::omega negative -> vRight < vLeft (shape.cpp's own
  // decomposition: omega == (vRight - vLeft) / trackWidth) ->
  // commandedRight() < commandedLeft(), with a clear margin so this is
  // not a rounding-noise pass.
  CHECK(planner.commandedRight() < planner.commandedLeft() - 1.0f);
}

// --- Per-goto cruise-speed override (135-004, wire parity with
// envelope.proto's GoTo.speed) -- a nonzero GotoTarget::speed overrides
// NavigatorLimits::speed for this goto's own cruise arc.
void testPerGotoSpeedOverrideAppliesToCruise() {
  Planner planner(defaultPlannerLimits());
  NavigatorLimits limits = defaultNavLimits();
  limits.speed = 150.0f;
  Navigator nav(limits, planner);
  IdealPlant plant;
  plant.trackWidth = limits.trackWidth;
  Types::RobotState state{};
  uint32_t now = 0;

  GotoTarget target;
  target.id = 108;
  target.x = 5000.0f;  // far ahead, exactly on-heading -- no pivot, clean cruise
  target.y = 0.0f;
  target.tolerance = 10.0f;
  target.timeout = 0;
  target.speed = 300.0f;  // override, well above limits.speed
  nav.start(target);

  // Enough ticks for the profiler to ramp to plateau (aMax=400mm/s^2 in
  // defaultPlannerLimits() reaches 300mm/s in <1s; 3s is generous).
  for (int i = 0; i < 60; ++i) cycle(nav, state, plant, now, kPeriod);

  // Both wheels should be commanding well above the UN-overridden
  // 150 mm/s config default -- proves the override actually reached
  // ArcSolver::solve(), not just GotoTarget's own storage.
  CHECK(planner.commandedLeft() > 250.0f);
  CHECK(planner.commandedRight() > 250.0f);
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
  testPivotThenCruiseNotBlockedBehindAligning();
  testSmallBearingNoPivotNoOscillation();
  testOtosStalenessSkipsResolve();
  testOtosDisconnectAbortsWithinWindow();
  testYawSignMatchesGotoOtosConvention();
  testPerGotoSpeedOverrideAppliesToCruise();
  testExactlyOneCompletionAck();
  std::printf("PASS navigator_test (12 checks)\n");
  return 0;
}
