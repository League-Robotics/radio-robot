// planner_lifecycle_test.cpp -- state-transition coverage for tick()'s
// explicit Move lifecycle (130-008, planner-honesty-pass-50ms-period-
// tick-state-machine-limits-reduction.md item 2). planner_scenarios_test
// and planner_noise_test already gate the EXACTNESS/behavior this
// rewrite must preserve; this file instead pins the STATES and EVENTS
// Planner::tick()'s own doc comment (planner.cpp) documents -- each test
// below names the transition it exercises in its own title comment.
#include <cstdio>

#include "planner.h"
#include "tests/test_support.h"

using Motion::Move;
using Motion::MoveLifecycle;
using Motion::Planner;
using Motion::TickResult;
using TestPlanner::benchLimits;
using TestPlanner::cycle;
using TestPlanner::PerfectPlant;

namespace {

constexpr float kPeriod = 50.0f;  // [ms]

Move distanceMove(uint32_t id, float threshold, float v_x) {
  Move m;
  m.id = id;
  m.kind = Move::Kind::Distance;
  m.threshold = threshold;
  m.v_x = v_x;
  m.timeout = 60000.0f;
  return m;
}

// ---- Idle ----

void testIdleAtConstructionAndWithEmptyQueue() {
  Planner planner(benchLimits());
  CHECK(planner.lifecycle() == MoveLifecycle::Idle);

  // Ticking with nothing ever queued stays Idle, command at zero.
  Types::RobotState state;
  PerfectPlant plant;
  uint32_t now = 0;
  for (int i = 0; i < 5; ++i) cycle(planner, state, plant, now, kPeriod);
  CHECK(planner.lifecycle() == MoveLifecycle::Idle);
  CHECK_NEAR(planner.commandedLeft(), 0.0f, 1e-6);
  CHECK_NEAR(planner.commandedRight(), 0.0f, 1e-6);
}

// ---- Idle/Draining -> Breakaway; Breakaway -> Tracking ----

void testBreakawayThenTracking() {
  Planner planner(benchLimits());
  PerfectPlant plant;
  Types::RobotState state;
  uint32_t now = 0;
  CHECK(planner.move(distanceMove(1, 500.0f, 150.0f), false));

  // Activation tick: the stall backstop's own atRest test still reads the
  // STALE (pre-loop) zero sample, so arrival/stall detection has not
  // armed yet.
  cycle(planner, state, plant, now, kPeriod);
  CHECK(planner.lifecycle() == MoveLifecycle::Breakaway);

  // Next tick: a fresh, genuinely-moving sample lands (the accel ramp's
  // first step is well above the 5 mm/s rest floor on a perfect plant),
  // so the Move has measurably left rest -- one-way (unlike `decelLatched`,
  // which 131-006 made a conditional latch -- see planner.cpp's
  // Planner::planWheels() doc comment).
  cycle(planner, state, plant, now, kPeriod);
  CHECK(planner.lifecycle() == MoveLifecycle::Tracking);
}

// ---- {Breakaway,Tracking,Stopping} -> Draining -> Idle (queue-empty) ----

void testDrainingThenIdleAfterQueueEmpties() {
  Planner planner(benchLimits());
  PerfectPlant plant;
  Types::RobotState state;
  uint32_t now = 0;
  // A short timeout, not the distance itself, ends this Move -- forced to
  // fire mid-cruise (well before profile-complete/arrived would honestly
  // fire on this 300mm leg), so it completes with a genuinely nonzero
  // staged command still in force. 130-010's own profile-complete gate
  // (planner.cpp's Move::Kind::Distance/Angle cases) now REQUIRES the
  // commanded ramp to have already decayed near the rest floor before that
  // event can fire at all -- exactly so a normal (non-timeout) completion
  // no longer leaves a meaningful residual to drain, which is what this
  // test used to rely on `distanceMove()`'s own long (60s) timeout to
  // exercise by accident. `timedOut` is checked unconditionally, before
  // any of that gating, so it is still the reliable way to force a
  // completes-while-still-moving scenario here.
  Move move = distanceMove(1, 300.0f, 150.0f);
  move.timeout = 200.0f;  // [ms] -- a few ticks into the accel/cruise ramp
  CHECK(planner.move(move, false));

  // Nothing queued behind it: the same tick the Move completes, the
  // staged command it leaves behind (the second-to-last decel step, not
  // literally zero yet) reports Draining; drainToZero() then usually
  // closes it out within that same tick or the next. Watch continuously
  // from BEFORE completion through the drain -- checking only after a
  // separate "wait for completed" loop would miss the one tick Draining
  // is reported on entirely.
  bool completed = false;
  bool sawDraining = false;
  bool sawIdle = false;
  for (int i = 0; i < 400 && !sawIdle; ++i) {
    // cycle() must run EVERY iteration regardless of `completed`'s
    // current value -- `completed || cycle(...).completed` would
    // short-circuit and stop calling cycle() (and therefore tick()) the
    // instant `completed` first became true, silently freezing the
    // planner mid-drain.
    const bool tickCompleted = cycle(planner, state, plant, now, kPeriod).completed;
    completed = completed || tickCompleted;
    if (planner.lifecycle() == MoveLifecycle::Draining) sawDraining = true;
    if (planner.lifecycle() == MoveLifecycle::Idle) sawIdle = true;
  }
  CHECK(completed);
  CHECK(sawDraining);
  CHECK(sawIdle);
  CHECK_NEAR(planner.commandedLeft(), 0.0f, 1e-6);
}

// ---- {Tracking} -> {Breakaway} directly, no Draining/Idle in between
// (queue-empty does NOT fire when a next entry is found) ----

void testChainedHandoffNeverVisitsDrainingOrIdle() {
  Planner planner(benchLimits());
  PerfectPlant plant;
  Types::RobotState state;
  uint32_t now = 0;
  // Same shape (1:1 ratio, same cruise): the exact-ratio hand-off lets the
  // second leg activate and continue AT SPEED the instant the first
  // completes (motion-planner sketch's lookahead) -- planner_scenarios_
  // test's testSameAxisChainExactAndCarried() is the exactness gate this
  // mirrors; here the point is only the STATE reported around the splice.
  CHECK(planner.move(distanceMove(1, 500.0f, 150.0f), false));
  CHECK(planner.move(distanceMove(2, 300.0f, 150.0f), false));

  int completions = 0;
  bool sawDrainingOrIdleBeforeChainEnded = false;
  for (int i = 0; i < 800 && completions < 2; ++i) {
    const TickResult r = cycle(planner, state, plant, now, kPeriod);
    if (r.completed) ++completions;
    // Meaningful strictly BEFORE the whole chain ends: once the SECOND
    // Move completes with nothing queued behind it, Draining/Idle is the
    // correct next state, not a violation of this test's own claim.
    if (completions < 2 &&
        (planner.lifecycle() == MoveLifecycle::Draining ||
         planner.lifecycle() == MoveLifecycle::Idle)) {
      sawDrainingOrIdleBeforeChainEnded = true;
    }
  }
  CHECK(completions == 2);
  CHECK(!sawDrainingOrIdleBeforeChainEnded);
}

// ---- Idle/Draining -> Stopping; Stopping -> Idle ----

void testStoppingWhileCruisingThenIdle() {
  Planner planner(benchLimits());
  PerfectPlant plant;
  Types::RobotState state;
  uint32_t now = 0;
  CHECK(planner.move(distanceMove(1, 5000.0f, 300.0f), false));
  // Run well past the accel ramp (300 mm/s / 400 mm/s^2 = 0.75 s = 15
  // ticks) so the body is genuinely cruising, not still accelerating.
  for (int i = 0; i < 25; ++i) cycle(planner, state, plant, now, kPeriod);
  CHECK(planner.lifecycle() == MoveLifecycle::Tracking);
  CHECK(planner.commandedLeft() > 250.0f);

  // move(..., replace=true) drops the active/pending Moves WITHOUT
  // touching the staged command (unlike estop()) -- see Planner::move()'s
  // own doc comment -- so the Stop entry activates onto a body still
  // cruising, which is exactly the scenario Stopping exists to describe
  // (a Kind::Stop that has real work left to ramp down, not one that
  // completes on its own activation tick).
  Move stopEntry;
  stopEntry.id = 99;
  stopEntry.kind = Move::Kind::Stop;
  CHECK(planner.move(stopEntry, /*replace=*/true));

  cycle(planner, state, plant, now, kPeriod);
  CHECK(planner.lifecycle() == MoveLifecycle::Stopping);
  CHECK(planner.commandedLeft() > 0.0f);  // still ramping down, not yet at rest

  bool completed = false;
  TickResult last{};
  bool sawStopping = true;  // already true from the check above
  for (int i = 0; i < 60 && !completed; ++i) {
    last = cycle(planner, state, plant, now, kPeriod);
    if (planner.lifecycle() == MoveLifecycle::Stopping) sawStopping = true;
    completed = last.completed;
  }
  CHECK(sawStopping);
  CHECK(completed);
  CHECK(last.moveId == 99);
  CHECK(last.settled);  // Stop's own completion test IS settleReached()

  // Queue is empty behind it: Idle within a few more ticks.
  bool sawIdle = false;
  for (int i = 0; i < 10 && !sawIdle; ++i) {
    cycle(planner, state, plant, now, kPeriod);
    sawIdle = planner.lifecycle() == MoveLifecycle::Idle;
  }
  CHECK(sawIdle);
}

// ---- event: arrived (settleReached(), isolated from profile-complete) ----

void testArrivedEventIsolatedFromProfileComplete() {
  // Hand-picks a measurement comfortably within the coarse arrival
  // tolerance (settleEpsilonLinear, default 1.0 mm) but well OUTSIDE the
  // tight, predictive profile-complete epsilon (kDoneEpsilonLinear,
  // 1e-3 mm, planner.cpp) -- a body 0.5 mm short and at rest can only be
  // completing via `arrived`, without needing to reason about exactly
  // which tick a real plant's own lag would trigger it on.
  Planner planner(benchLimits());
  PerfectPlant plant;
  Types::RobotState state;
  uint32_t now = 0;
  CHECK(planner.move(distanceMove(1, 500.0f, 150.0f), false));

  // Run normally long enough to clear Breakaway -- `arrived` requires
  // lifecycle() == Tracking.
  for (int i = 0; i < 10; ++i) cycle(planner, state, plant, now, kPeriod);
  CHECK(planner.lifecycle() == MoveLifecycle::Tracking);

  const float residual = 0.5f;  // [mm] short of the 500 mm target
  now += static_cast<uint32_t>(kPeriod);
  state.time.cycleStart = now;
  state.wheelLeft.position = 500.0f - residual;
  state.wheelLeft.velocity = 0.0f;
  state.wheelLeft.sampleTime = now;
  state.wheelLeft.connected = true;
  state.wheelRight.position = 500.0f - residual;
  state.wheelRight.velocity = 0.0f;
  state.wheelRight.sampleTime = now;
  state.wheelRight.connected = true;
  const TickResult r = planner.tick(state);
  planner.update(state);

  CHECK(r.completed);
  CHECK(!r.timedOut);
  // `arrived` REQUIRES settleReached() to be true -- this is its
  // signature: a Move completed via profile-complete alone (the kind-
  // specific switch case in tick()) can legitimately report settled=false.
  CHECK(r.settled);
}

// ---- event: timeout ----

void testTimeoutEvent() {
  Planner planner(benchLimits());
  PerfectPlant plant;
  Types::RobotState state;
  uint32_t now = 0;
  Move m = distanceMove(1, 100000.0f, 150.0f);  // unreachable before timeout
  m.timeout = 2000.0f;  // [ms]
  CHECK(planner.move(m, false));

  bool completed = false;
  TickResult last{};
  for (int i = 0; i < 100 && !completed; ++i) {
    last = cycle(planner, state, plant, now, kPeriod);
    completed = last.completed;
  }
  CHECK(completed);
  CHECK(last.timedOut);
}

// ---- event: stall-window expiry ----

void testStallWindowExpiryEvent() {
  // A body that has genuinely left rest (clearing Breakaway) and is then
  // frozen -- a fresh, at-rest sample every tick, but the SAME position --
  // never closes on the target. kStallWindow (planner.cpp) is 0.5 s == 10
  // ticks at this 50 ms period.
  Planner planner(benchLimits());
  PerfectPlant plant;
  Types::RobotState state;
  uint32_t now = 0;
  CHECK(planner.move(distanceMove(1, 500.0f, 150.0f), false));
  for (int i = 0; i < 20; ++i) cycle(planner, state, plant, now, kPeriod);
  CHECK(planner.lifecycle() == MoveLifecycle::Tracking);

  const float frozenLeft = state.wheelLeft.position;
  const float frozenRight = state.wheelRight.position;
  bool completed = false;
  TickResult last{};
  int freezeTicks = 0;
  for (; freezeTicks < 30 && !completed; ++freezeTicks) {
    now += static_cast<uint32_t>(kPeriod);
    state.time.cycleStart = now;
    last = planner.tick(state);
    planner.update(state);
    // Publish a frozen, at-rest, but genuinely FRESH sample (new
    // sampleTime every tick) -- a stuck wheel, not merely a stale re-read.
    state.wheelLeft.position = frozenLeft;
    state.wheelLeft.velocity = 0.0f;
    state.wheelLeft.sampleTime = now;
    state.wheelLeft.connected = true;
    state.wheelRight.position = frozenRight;
    state.wheelRight.velocity = 0.0f;
    state.wheelRight.sampleTime = now;
    state.wheelRight.connected = true;
    completed = last.completed;
  }
  CHECK(completed);
  CHECK(!last.timedOut);   // the 60 s MOVE_TIMEOUT never came close
  CHECK(!last.settled);    // frozen well short (~380 mm) of the target
  std::printf("  stall-window expiry: completed after %d frozen ticks "
              "(kStallWindow == 10 ticks @ 50 ms)\n", freezeTicks);
  CHECK(freezeTicks >= 9);
  CHECK(freezeTicks <= 14);
}

// ---- event: estop (ANY -> Idle, unconditional and immediate) ----

void testEstopForcesIdleImmediately() {
  Planner planner(benchLimits());
  PerfectPlant plant;
  Types::RobotState state;
  uint32_t now = 0;
  CHECK(planner.move(distanceMove(1, 5000.0f, 300.0f), false));
  for (int i = 0; i < 20; ++i) cycle(planner, state, plant, now, kPeriod);
  CHECK(planner.lifecycle() == MoveLifecycle::Tracking);

  planner.estop();
  // No tick() needed: estop() is unconditional and this instant, so an
  // observer never sees a stale active-Move lifecycle after it returns.
  CHECK(planner.lifecycle() == MoveLifecycle::Idle);
  CHECK(!planner.active());
  CHECK(planner.pendingCount() == 0);
  CHECK_NEAR(planner.commandedLeft(), 0.0f, 1e-6);
  CHECK_NEAR(planner.commandedRight(), 0.0f, 1e-6);
}

// ---- event: replace (ANY -> {Breakaway,Tracking,Stopping} next tick()) ----

void testReplaceReactivatesBreakaway() {
  Planner planner(benchLimits());
  PerfectPlant plant;
  Types::RobotState state;
  uint32_t now = 0;
  CHECK(planner.move(distanceMove(1, 5000.0f, 300.0f), false));
  for (int i = 0; i < 20; ++i) cycle(planner, state, plant, now, kPeriod);
  CHECK(planner.lifecycle() == MoveLifecycle::Tracking);

  // Bring the body to rest first (estop()) rather than replacing while
  // still cruising: activateNext() always resets a fresh activation's
  // lifecycle_ to Breakaway, but the very next line of tick() re-derives
  // it from the MEASURED velocity, which is a purely physical reading
  // independent of which Move is nominally active. Replacing onto a body
  // still carrying the preempted Move's own momentum would read Tracking
  // again within that same tick (correctly -- the body plainly has not
  // just left rest, it never stopped) rather than demonstrating the fresh
  // activation this test is about.
  planner.estop();
  cycle(planner, state, plant, now, kPeriod);
  CHECK(planner.lifecycle() == MoveLifecycle::Idle);

  CHECK(planner.move(distanceMove(2, 200.0f, 150.0f), /*replace=*/true));
  // The replacement activates on the NEXT tick() (Planner::move()'s own
  // doc comment) as a fresh activation -- Breakaway, exactly as any other
  // freshly-popped Distance entry, regardless of what state the
  // preempted Move was in.
  cycle(planner, state, plant, now, kPeriod);
  CHECK(planner.lifecycle() == MoveLifecycle::Breakaway);
}

// ---- Tracking -> Aligning -> {Draining,Idle}: terminal fine-align -------
//
// 134-003. The trim that took `tovez`'s planner square tour from 25.8 mm
// of closure to 9.4 mm, moved into the Move (docs/bench-reports/
// motion-planning-lab-2026-08-04.md §5.2). These tests pin the STATE
// machine -- entry conditions, the nudge cap, the timeout backstop, and
// which Kinds are excluded. They cannot and do not pin the ACCURACY claim:
// the sim's corner behavior sign-flips against hardware (report §7), so
// the real gate is a bench run on `tovez`.

// benchLimits() plus the two fine-align fields the robot JSON supplies on
// a real boot. alignTol is [rad] -- 0.017453 rad IS the measured 1.0 deg
// operating point; a value near 1.0 here would be degrees in a radian
// field.
Motion::PlannerLimits alignLimits(int32_t maxNudges = 3) {
  Motion::PlannerLimits limits = benchLimits();
  limits.landing.alignTol = 0.017453f;  // [rad] 1.0 deg
  limits.landing.alignMaxNudges = maxNudges;
  return limits;
}

Move angleMove(uint32_t id, float threshold, float omega) {
  Move m;
  m.id = id;
  m.kind = Move::Kind::Angle;
  m.threshold = threshold;
  m.omega = omega;
  m.timeout = 60000.0f;
  return m;
}

// What an Aligning phase did, watched from outside the Planner: how many
// corrective pivots it emitted, whether it ever acked while still in the
// phase (it must not), and how it ended.
struct AlignWatch {
  bool sawAligning = false;
  bool ackedWhileAligning = false;
  int nudges = 0;
  bool completed = false;
  bool timedOut = false;
  uint32_t moveId = 0;
  int ticks = 0;
};

// Run until the active Move completes (or `maxTicks` runs out), watching
// the alignment. `disturb` is applied AFTER each cycle while the planner is
// aligning -- a test-only way to make a residual that cannot be closed,
// standing in for a plant with no corrective authority. It moves the
// encoder POSITIONS only, never the published velocities, so the phase's
// own at-rest test still reads an honestly-resting body.
template <typename Plant>
AlignWatch runAlignment(Planner& planner, Types::RobotState& state, Plant& plant,
                        uint32_t& now, int maxTicks, float disturb = 0.0f) {
  AlignWatch watch;
  // Seeded true so whatever command the landing itself left behind is never
  // miscounted as the first nudge.
  bool previousCommanding = true;
  for (int i = 0; i < maxTicks && !watch.completed; ++i) {
    const TickResult result = cycle(planner, state, plant, now, kPeriod);
    ++watch.ticks;
    const bool aligning = planner.lifecycle() == MoveLifecycle::Aligning;
    if (aligning) {
      watch.sawAligning = true;
      if (result.completed) watch.ackedWhileAligning = true;
      const bool commanding = planner.commandedLeft() != 0.0f ||
                              planner.commandedRight() != 0.0f;
      if (commanding && !previousCommanding) ++watch.nudges;
      previousCommanding = commanding;
      if (disturb != 0.0f) plant.disturbHeading(disturb, 100.0f);
    } else {
      previousCommanding = true;
    }
    if (result.completed) {
      watch.completed = true;
      watch.timedOut = result.timedOut;
      watch.moveId = result.moveId;
    }
  }
  return watch;
}

// Entry, the held-back ack, and a nudge that actually closes the residual.
//
// The residual is made the way a real corner makes one: `threshold` is the
// ACTUATION-sized command (what Core::RobotLoop::handleMove()'s rotation-
// calibration inversion produces) and `requestedThreshold` is the caller's
// own intent (134-001). A perfect plant lands the command exactly, so the
// gap between the two IS the residual the trim must close -- 0.05 rad, or
// 2.9 deg, comfortably outside the 1.0 deg tolerance.
void testFineAlignEntersTrimsAndAcksOnTheWayOut() {
  Planner planner(alignLimits());
  PerfectPlant plant;
  Types::RobotState state;
  uint32_t now = 0;

  Move move = angleMove(1, 0.50f, 2.0f);
  move.requestedThreshold = 0.55f;  // [rad] what the caller ASKED for
  CHECK(planner.move(move, false));

  const AlignWatch watch = runAlignment(planner, state, plant, now, 600);

  CHECK(watch.sawAligning);
  // The Move is not done until it has landed: no ack may ride out while
  // the trim is still running.
  CHECK(!watch.ackedWhileAligning);
  CHECK(watch.completed);
  CHECK(!watch.timedOut);
  // A nudge rewrites the ACTIVE Move in place; that must not change the
  // identity the host is waiting on.
  CHECK(watch.moveId == 1u);
  CHECK(watch.nudges >= 1);
  CHECK(watch.nudges <= 3);

  // Landed on the INTENT (0.55 rad), not on the command (0.50 rad), and
  // inside the tolerance. Without the trim this would sit at 0.50.
  const float heading =
      (plant.positionRight - plant.positionLeft) / 100.0f;  // [rad]
  std::printf("  fine-align: %d nudge(s), heading %.5f rad vs intent 0.55\n",
              watch.nudges, static_cast<double>(heading));
  CHECK(std::fabs(0.55f - heading) <= 0.017453f);

  // The queue is not wedged: with nothing pending, the planner drains out.
  CHECK(planner.lifecycle() == MoveLifecycle::Draining ||
        planner.lifecycle() == MoveLifecycle::Idle);
}

// A residual that nudges cannot close stops at alignMaxNudges, not at the
// Move timeout.
//
// The plant is stalled the moment the phase begins -- wheel gains to zero,
// so every nudge is commanded and delivers exactly nothing. That is the
// measured NO-BREAKAWAY case (report §3: 26% of 333 nudges delivered under
// 0.25 deg), and it is the one that has to spend the whole budget: the
// residual never improves, but it never gets WORSE either, so the
// made-it-worse guard correctly stays out of the way and the retries the
// measured 94% convergence depends on all happen.
void testFineAlignStopsAtTheNudgeCap() {
  constexpr int32_t kBudget = 3;
  Planner planner(alignLimits(kBudget));
  TestPlanner::NoisyPlant plant;  // defaults = a clean plant, until stalled below
  Types::RobotState state;
  uint32_t now = 0;

  Move move = angleMove(1, 0.50f, 2.0f);
  move.requestedThreshold = 0.55f;  // [rad] a 2.9 deg residual to chase
  CHECK(planner.move(move, false));

  bool completed = false;
  bool timedOut = false;
  bool sawAligning = false;
  bool previousCommanding = true;
  int nudges = 0;
  int ticks = 0;
  for (; ticks < 2000 && !completed; ++ticks) {
    const TickResult result = cycle(planner, state, plant, now, kPeriod);
    if (planner.lifecycle() == MoveLifecycle::Aligning) {
      sawAligning = true;
      // Stall the wheels for the whole phase: commanded, but delivering
      // nothing.
      plant.gainLeft = 0.0f;
      plant.gainRight = 0.0f;
      const bool commanding = planner.commandedLeft() != 0.0f ||
                              planner.commandedRight() != 0.0f;
      if (commanding && !previousCommanding) ++nudges;
      previousCommanding = commanding;
    } else {
      previousCommanding = true;
    }
    if (result.completed) {
      completed = true;
      timedOut = result.timedOut;
    }
  }

  CHECK(sawAligning);
  CHECK(completed);
  // The CAP ended it, not the 60 s timeout -- which is the whole point of
  // having a cap.
  CHECK(!timedOut);
  std::printf("  fine-align cap: %d nudges spent (budget %d), %d ticks\n",
              nudges, static_cast<int>(kBudget), ticks);
  CHECK(nudges == kBudget);
}

// A nudge that made the residual measurably WORSE ends the phase, well
// inside the budget.
//
// The corrective pivot has a coarse quantum of its own (report §3: median
// 1.72 deg delivered against a 1.0 deg tolerance), so a plant that
// consistently over-delivers cannot land inside the tolerance -- it
// oscillates across it, and without this guard it spends the whole budget
// to end no better than it started. Measured directly in the sim while
// building this ticket: a 1.9 deg residual, nudged, came back +3.3 deg the
// other way, and the loop hunted between the two for all six nudges.
//
// Reproduced here by yanking the heading further out after every aligning
// tick -- a residual that grows under correction, which is what an
// over-delivering nudge looks like from the phase's own side.
void testFineAlignStopsWhenANudgeMakesItWorse() {
  constexpr int32_t kBudget = 6;
  Planner planner(alignLimits(kBudget));
  PerfectPlant plant;
  Types::RobotState state;
  uint32_t now = 0;
  CHECK(planner.move(angleMove(1, 0.50f, 2.0f), false));

  const AlignWatch watch =
      runAlignment(planner, state, plant, now, 2000, /*disturb=*/0.004f);

  CHECK(watch.sawAligning);
  CHECK(watch.completed);
  CHECK(!watch.timedOut);
  std::printf("  fine-align worse-guard: stopped after %d nudge(s) of a %d "
              "budget, %d ticks\n", watch.nudges, static_cast<int>(kBudget),
              watch.ticks);
  // Bounded well under the budget -- that is the whole point.
  CHECK(watch.nudges >= 1);
  CHECK(watch.nudges < kBudget);
}

// The wall-clock timeout still fires from INSIDE Aligning. A non-
// converging alignment with a budget it cannot exhaust must not be able to
// wedge the queue -- this is the acceptance criterion the nudge cap alone
// does not cover.
//
// It ends the Move but does NOT raise the move-timeout fault: the Move met
// its ANGLE stop condition and only the trim overran, so the wire's
// kFlagFaultMoveTimeout would be a false fault. Asserted both ways below.
void testFineAlignTimeoutFiresFromInsideAligning() {
  Planner planner(alignLimits(/*maxNudges=*/1000));
  PerfectPlant plant;
  Types::RobotState state;
  uint32_t now = 0;
  Move move = angleMove(1, 0.50f, 2.0f);
  // Long enough for the 0.5 rad turn itself (a few hundred ms), far too
  // short for an alignment that can never converge.
  move.timeout = 3000.0f;  // [ms]
  CHECK(planner.move(move, false));

  const AlignWatch watch =
      runAlignment(planner, state, plant, now, 2000, /*disturb=*/0.004f);

  CHECK(watch.sawAligning);
  CHECK(watch.completed);
  CHECK(watch.moveId == 1u);
  // ...but NOT reported as a move-timeout fault. The Move met its ANGLE
  // stop condition; what expired was the trim's budget, and
  // `TickResult::timedOut` is the wire's kFlagFaultMoveTimeout -- a fault
  // on the MOTION. See tick()'s own `motionTimedOut` for the argument.
  CHECK(!watch.timedOut);
  // And the queue really is free again -- which is the property that
  // actually matters here.
  CHECK(!planner.active());
  CHECK(planner.lifecycle() == MoveLifecycle::Draining ||
        planner.lifecycle() == MoveLifecycle::Idle);
}

// The exclusions, one per rejected entry condition. None of these may ever
// enter Aligning: Distance and Time Moves have no heading intent to trim,
// a Wheels-velocityKind Move is structurally excluded from the ledger that
// supplies the target, and a Move that timed out mid-motion never
// delivered the motion there would be anything honest to trim toward.
void testFineAlignNeverEntersForExcludedMoves() {
  struct Case {
    const char* name;
    Move move;
  };
  Case cases[4];

  cases[0].name = "Distance";
  cases[0].move = distanceMove(1, 300.0f, 150.0f);

  cases[1].name = "Time";
  Move timeMove;
  timeMove.id = 2;
  timeMove.kind = Move::Kind::Time;
  timeMove.threshold = 400.0f;  // [ms]
  timeMove.omega = 2.0f;
  timeMove.timeout = 60000.0f;
  cases[1].move = timeMove;

  cases[2].name = "Wheels-velocity Angle";
  Move wheelsMove;
  wheelsMove.id = 3;
  wheelsMove.kind = Move::Kind::Angle;
  wheelsMove.velocityKind = Move::VelocityKind::Wheels;
  wheelsMove.threshold = 0.50f;  // [rad]
  wheelsMove.vLeft = -100.0f;
  wheelsMove.vRight = 100.0f;
  wheelsMove.timeout = 60000.0f;
  cases[2].move = wheelsMove;

  cases[3].name = "timed-out Angle";
  Move timedOutAngle = angleMove(4, 3.0f, 2.0f);  // a long turn...
  timedOutAngle.timeout = 200.0f;                 // ...cut off mid-motion
  cases[3].move = timedOutAngle;

  for (const Case& scenario : cases) {
    Planner planner(alignLimits());
    PerfectPlant plant;
    Types::RobotState state;
    uint32_t now = 0;
    CHECK(planner.move(scenario.move, false));

    bool completed = false;
    bool sawAligning = false;
    for (int i = 0; i < 600 && !completed; ++i) {
      completed = cycle(planner, state, plant, now, kPeriod).completed;
      if (planner.lifecycle() == MoveLifecycle::Aligning) sawAligning = true;
    }
    std::printf("  fine-align excluded: %s -- completed %d, aligned %d\n",
                scenario.name, completed ? 1 : 0, sawAligning ? 1 : 0);
    CHECK(completed);
    CHECK(!sawAligning);
  }
}

// A Move handing off AT SPEED does not align -- and the ledger still
// carries its intent through the successor's own alignment.
//
// Two same-ratio Angle Moves queued together hand off at speed
// (boundaryLambda()'s exact-ratio fast path), so the first is not supposed
// to come to a stop at all and is excluded exactly as `arrived` excludes
// it. The SECOND has an empty queue behind it, lands at zero, and aligns.
//
// THIS IS A REAL LIMITATION, NOT JUST A GUARD: in a PIPELINED tour every
// corner but the last hands off at speed and therefore gets no trim. The
// measured 25.8 -> 9.4 mm result came from a SEQUENTIAL tour
// (planner_square_tour.py --sequential --trim), which is the
// configuration this phase reproduces; a pipelined tour is a different
// arm with a different number.
//
// The final heading is the discriminating check. Each Move commands 0.50
// rad and intends 0.55, so a chain that carries INTENT lands on 1.10 --
// and it only lands there if the second Move's alignment aimed at the
// LATCHED ledger target rather than at a target recomputed from whatever
// baseline its own corrective nudges left behind.
void testFineAlignSkipsAnAtSpeedHandoffAndKeepsTheLedger() {
  Planner planner(alignLimits());
  PerfectPlant plant;
  Types::RobotState state;
  uint32_t now = 0;

  Move first = angleMove(1, 0.50f, 2.0f);
  first.requestedThreshold = 0.55f;  // [rad]
  Move second = angleMove(2, 0.50f, 2.0f);
  second.requestedThreshold = 0.55f;  // [rad]
  CHECK(planner.move(first, false));
  CHECK(planner.move(second, false));

  int aligningDuringFirst = 0;
  int aligningDuringSecond = 0;
  bool firstAcked = false;
  bool secondAcked = false;
  for (int i = 0; i < 800 && !secondAcked; ++i) {
    const TickResult result = cycle(planner, state, plant, now, kPeriod);
    if (planner.lifecycle() == MoveLifecycle::Aligning) {
      if (firstAcked) {
        ++aligningDuringSecond;
      } else {
        ++aligningDuringFirst;
      }
    }
    if (result.completed && result.moveId == 1u) firstAcked = true;
    if (result.completed && result.moveId == 2u) secondAcked = true;
  }

  const float heading =
      (plant.positionRight - plant.positionLeft) / 100.0f;  // [rad]
  std::printf("  fine-align at-speed handoff: aligning ticks %d (first) / %d "
              "(second), heading %.5f rad vs intent 1.10\n",
              aligningDuringFirst, aligningDuringSecond,
              static_cast<double>(heading));
  CHECK(firstAcked);
  CHECK(secondAcked);
  CHECK(aligningDuringFirst == 0);
  CHECK(aligningDuringSecond > 0);
  CHECK(std::fabs(1.10f - heading) <= 0.017453f);
}

// Unconfigured is OFF. With both fields at their structural zero -- which
// is what every direct caller that never sets them gets -- an Angle Move
// completes at its landing exactly as it did before 134-003 existed.
void testFineAlignDisabledWhenUnconfigured() {
  Planner planner(benchLimits());  // alignTol/alignMaxNudges both 0
  PerfectPlant plant;
  Types::RobotState state;
  uint32_t now = 0;
  Move move = angleMove(1, 0.50f, 2.0f);
  move.requestedThreshold = 0.55f;  // a residual the trim WOULD have closed
  CHECK(planner.move(move, false));

  const AlignWatch watch = runAlignment(planner, state, plant, now, 600);

  CHECK(watch.completed);
  CHECK(!watch.sawAligning);
  // Landed on the command, residual left standing for the ledger to repay.
  const float heading =
      (plant.positionRight - plant.positionLeft) / 100.0f;  // [rad]
  CHECK(std::fabs(0.50f - heading) <= 0.01f);
}

}  // namespace

int main() {
  std::printf("planner_lifecycle_test:\n");
  testIdleAtConstructionAndWithEmptyQueue();
  testBreakawayThenTracking();
  testDrainingThenIdleAfterQueueEmpties();
  testChainedHandoffNeverVisitsDrainingOrIdle();
  testStoppingWhileCruisingThenIdle();
  testArrivedEventIsolatedFromProfileComplete();
  testTimeoutEvent();
  testStallWindowExpiryEvent();
  testEstopForcesIdleImmediately();
  testReplaceReactivatesBreakaway();
  testFineAlignEntersTrimsAndAcksOnTheWayOut();
  testFineAlignStopsAtTheNudgeCap();
  testFineAlignStopsWhenANudgeMakesItWorse();
  testFineAlignTimeoutFiresFromInsideAligning();
  testFineAlignNeverEntersForExcludedMoves();
  testFineAlignSkipsAnAtSpeedHandoffAndKeepsTheLedger();
  testFineAlignDisabledWhenUnconfigured();
  std::printf("planner_lifecycle_test: all checks passed\n");
  return 0;
}
