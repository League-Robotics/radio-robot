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
  // so the Move has measurably left rest -- one-way, like decelLatched.
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
  std::printf("planner_lifecycle_test: all checks passed\n");
  return 0;
}
