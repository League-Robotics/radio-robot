// devices_clock_harness.cpp — off-hardware acceptance harness for ticket
// DB-003 (device-bus-tickets.md), migrated by sprint 108 ticket 010 to the
// pure-interface split: proves TestSim::SimClock (tests/_infra/sim/
// sim_clock.h, the Devices::Clock host-test fake) advances ONLY when a
// test steps it explicitly (setMicros()/advanceMicros()), never on its
// own, and that TestSim::SimSleeper (the Devices::Sleeper host-test fake)
// records every requested sleepMillis()/yield() call without blocking on a
// wall clock or sleeping for real.
//
// Plain C++ program, hand-rolled assertions — mirrors devices_i2c_bus_
// harness.cpp / motor_policy_harness.cpp's shape exactly: prints a
// PASS/FAIL line per scenario and exits nonzero if any assertion failed,
// run by the pytest wrapper in test_devices_clock.py.

#include <cstdint>
#include <cstdio>
#include <string>

#include "sim_clock.h"

namespace {

// --- Hand-rolled assertion plumbing (see devices_i2c_bus_harness.cpp) -----

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

void checkIntEq(int actual, int expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s — expected %d, got %d", what.c_str(),
                  expected, actual);
    fail(buf);
  }
}

void checkU32Eq(uint32_t actual, uint32_t expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s — expected %u, got %u", what.c_str(),
                  expected, actual);
    fail(buf);
  }
}

void checkU64Eq(uint64_t actual, uint64_t expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s — expected %llu, got %llu",
                  what.c_str(), static_cast<unsigned long long>(expected),
                  static_cast<unsigned long long>(actual));
    fail(buf);
  }
}

// --- Scenarios --------------------------------------------------------

// 1. A fresh Clock starts at 0 and does not move on repeated nowMicros()
//    reads — the host fake never self-advances (unlike I2CBus's own fake
//    clock, which self-advances during a live entry-spin; Clock has no such
//    spin).
void scenarioStartsAtZeroAndDoesNotSelfAdvance() {
  beginScenario("Clock starts at 0us and never advances on its own");
  TestSim::SimClock clock;

  checkU64Eq(clock.nowMicros(), 0, "fresh Clock reads 0us");
  checkU64Eq(clock.nowMicros(), 0, "a second read is still 0us — no self-advance");
  checkU64Eq(clock.nowMicros(), 0, "a third read is still 0us — no self-advance");
}

// 2. setMicros() sets the fake clock directly; advanceMicros() steps it
//    forward by exactly the requested delta, cumulatively.
void scenarioSetAndAdvanceMicros() {
  beginScenario("setMicros()/advanceMicros() step the fake clock exactly");
  TestSim::SimClock clock;

  clock.setMicros(5000);
  checkU64Eq(clock.nowMicros(), 5000, "setMicros(5000) reads back 5000us");

  clock.advanceMicros(250);
  checkU64Eq(clock.nowMicros(), 5250, "advanceMicros(250) adds exactly 250us");

  clock.advanceMicros(1);
  clock.advanceMicros(1);
  checkU64Eq(clock.nowMicros(), 5252, "repeated advanceMicros() calls accumulate");
}

// 3. Two Clock instances are independent — stepping one must not perturb
//    the other (a per-instance fake, unlike I2CBus's shared static one).
void scenarioInstancesAreIndependent() {
  beginScenario("separate Clock instances do not share state");
  TestSim::SimClock a;
  TestSim::SimClock b;

  a.setMicros(1000);
  b.setMicros(9000);

  checkU64Eq(a.nowMicros(), 1000, "clock A holds its own value");
  checkU64Eq(b.nowMicros(), 9000, "clock B holds its own, different value");

  a.advanceMicros(500);
  checkU64Eq(a.nowMicros(), 1500, "advancing A moves only A");
  checkU64Eq(b.nowMicros(), 9000, "B is unaffected by A's advance");
}

// 4. Sleeper.sleepMillis() records the requested duration and a running
//    count, without blocking (no wall-clock dependency: the test process
//    would visibly hang if this actually slept the sum of the requested
//    durations, and it does not).
void scenarioSleeperRecordsRequestedSleeps() {
  beginScenario("Sleeper.sleepMillis() records requests without blocking");
  TestSim::SimSleeper sleeper;

  checkIntEq(sleeper.sleepCount(), 0, "fresh Sleeper has made zero sleep requests");

  sleeper.sleepMillis(4);
  checkIntEq(sleeper.sleepCount(), 1, "sleepCount increments after one sleepMillis()");
  checkU32Eq(sleeper.lastSleepMillis(), 4, "lastSleepMillis reflects the requested duration");

  sleeper.sleepMillis(12);
  checkIntEq(sleeper.sleepCount(), 2, "sleepCount increments again");
  checkU32Eq(sleeper.lastSleepMillis(), 12, "lastSleepMillis tracks the MOST RECENT request");
}

// 5. Sleeper.yield() records a running count independent of sleepMillis()'s
//    own counter — the cycle's bare scheduling points (schedule()) are
//    distinguishable from its timed settle/pace sleeps (fiber_sleep()).
void scenarioYieldRecordedSeparatelyFromSleep() {
  beginScenario("Sleeper.yield() is counted separately from sleepMillis()");
  TestSim::SimSleeper sleeper;

  sleeper.yield();
  sleeper.yield();
  sleeper.sleepMillis(1);

  checkIntEq(sleeper.yieldCount(), 2, "two yield() calls counted");
  checkIntEq(sleeper.sleepCount(), 1, "one sleepMillis() call counted, independently");
}

// 6. A Sleeper never advances a Clock on its own — the two seams are
//    independent objects; a harness driving the loop's cycle must step the
//    Clock itself even while the cycle calls Sleeper.sleepMillis().
void scenarioSleeperNeverAdvancesClock() {
  beginScenario("Sleeper never touches a Clock instance");
  TestSim::SimClock clock;
  TestSim::SimSleeper sleeper;

  clock.setMicros(1234);
  sleeper.sleepMillis(4);
  sleeper.yield();

  checkU64Eq(clock.nowMicros(), 1234, "Clock is unchanged by unrelated Sleeper calls");
}

}  // namespace

int main() {
  scenarioStartsAtZeroAndDoesNotSelfAdvance();
  scenarioSetAndAdvanceMicros();
  scenarioInstancesAreIndependent();
  scenarioSleeperRecordsRequestedSleeps();
  scenarioYieldRecordedSeparatelyFromSleep();
  scenarioSleeperNeverAdvancesClock();

  if (g_failureCount == 0) {
    std::printf("OK: all TestSim::SimClock/Sleeper scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the TestSim::SimClock/Sleeper scenarios\n",
              g_failureCount);
  return 1;
}
