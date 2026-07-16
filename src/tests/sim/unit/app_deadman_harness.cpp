// app_deadman_harness.cpp -- off-hardware acceptance harness for ticket
// 103-004 (SUC-004), App::Deadman (src/firm/app/deadman.{h,cpp}). Proves
// arm(duration)/disarm()/expired() against TestSim::SimClock
// (tests/_infra/sim/sim_clock.cpp, the TestSim::SimClock host-test fake --
// sprint 108 ticket 010) -- no wall clock, no real sleeps, every scenario
// steps time explicitly.
//
// Mirrors devices_clock_harness.cpp's exact shape: hand-rolled assertion
// plumbing, PASS/FAIL printf, exit nonzero on failure. Compiled by
// test_app_deadman.py with -DHOST_BUILD against deadman.cpp and
// sim_clock.cpp.
#include <cstdint>
#include <cstdio>
#include <string>

#include "app/deadman.h"
#include "sim_clock.h"

namespace {

// --- Hand-rolled assertion plumbing (see devices_clock_harness.cpp) ------

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

void checkFalse(bool condition, const std::string& what) {
  if (condition) fail(what + " -- expected false, got true");
}

// --- Scenarios -----------------------------------------------------------

// 1. arm(100) -> expired() false before 100ms of scripted clock advance,
//    true at/after.
void scenarioArmExpiresAtDeadline() {
  beginScenario("arm(100): expired() false before the 100ms deadline, true at/after");
  TestSim::SimClock clock;
  App::Deadman dm(clock);

  dm.arm(100.0f);  // [ms]
  checkFalse(dm.expired(), "not expired immediately after arm(100) at now=0");

  clock.advanceMicros(99000);  // now = 99ms
  checkFalse(dm.expired(), "not expired at 99ms (< 100ms deadline)");

  clock.advanceMicros(1000);  // now = 100ms -- exactly at the deadline
  checkTrue(dm.expired(), "expired AT the 100ms deadline (>=, not >)");

  clock.advanceMicros(50000);  // now = 150ms -- well past
  checkTrue(dm.expired(), "still expired well past the deadline");
}

// 2. disarm() -> expired() stays false regardless of further advance.
void scenarioDisarmCancelsUnconditionally() {
  beginScenario("disarm(): expired() stays false regardless of further clock advance");
  TestSim::SimClock clock;
  App::Deadman dm(clock);

  dm.arm(100.0f);
  clock.advanceMicros(150000);  // now = 150ms -- would be expired
  checkTrue(dm.expired(), "sanity: expired before disarm()");

  dm.disarm();
  checkFalse(dm.expired(), "not expired immediately after disarm()");

  clock.advanceMicros(1000000);  // now += 1s -- far past any prior deadline
  checkFalse(dm.expired(), "still not expired after a large further advance");
}

// 3. Re-arming resets the deadline (not stacking): arm(100), advance 50ms,
//    arm(100) again, advance 60ms more -> still not expired, since the
//    second arm() set a FRESH deadline from the now-current clock.
void scenarioReArmResetsDeadline() {
  beginScenario("arm() re-armed mid-window sets a FRESH deadline from now (re-arming, not stacking)");
  TestSim::SimClock clock;
  App::Deadman dm(clock);

  dm.arm(100.0f);              // deadline = 100ms
  clock.advanceMicros(50000);  // now = 50ms
  dm.arm(100.0f);               // re-arm -- deadline = 50ms + 100ms = 150ms
  clock.advanceMicros(60000);  // now = 110ms (< 150ms new deadline)

  checkFalse(dm.expired(), "not expired -- the second arm() reset the deadline to now+100ms");

  clock.advanceMicros(40000);  // now = 150ms -- at the reset deadline
  checkTrue(dm.expired(), "expired once the RESET deadline is reached");
}

// 4. Negative/NaN duration is malformed-wire-safety-clamped to 0 --
//    immediate expiry, not a crash or an unbounded window.
void scenarioNegativeAndNanDurationClampToImmediateExpiry() {
  beginScenario("arm(): negative/NaN duration clamps to 0 -- immediate expiry");
  TestSim::SimClock clock;
  App::Deadman dmNeg(clock);

  clock.setMicros(1000);
  dmNeg.arm(-50.0f);
  checkTrue(dmNeg.expired(), "arm(negative) expires immediately (clamped to 0)");

  TestSim::SimClock clock2;
  App::Deadman dmNan(clock2);
  clock2.setMicros(2000);
  float nan = 0.0f / 0.0f;
  dmNan.arm(nan);
  checkTrue(dmNan.expired(), "arm(NaN) expires immediately (clamped to 0)");
}

// 5. never-armed Deadman reads as not expired -- there is no window to have
//    elapsed yet.
void scenarioNeverArmedIsNotExpired() {
  beginScenario("a fresh, never-armed Deadman reads expired() == false");
  TestSim::SimClock clock;
  App::Deadman dm(clock);

  checkFalse(dm.expired(), "never-armed Deadman is not expired");
  clock.advanceMicros(1000000);
  checkFalse(dm.expired(), "still not expired after a clock advance with no arm() ever called");
}

}  // namespace

int main() {
  scenarioArmExpiresAtDeadline();
  scenarioDisarmCancelsUnconditionally();
  scenarioReArmResetsDeadline();
  scenarioNegativeAndNanDurationClampToImmediateExpiry();
  scenarioNeverArmedIsNotExpired();

  if (g_failureCount == 0) {
    std::printf("OK: all App::Deadman scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the App::Deadman scenarios\n", g_failureCount);
  return 1;
}
