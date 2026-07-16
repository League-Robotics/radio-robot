// sim_clock.cpp -- TestSim::SimClock / TestSim::SimSleeper definitions.
// Moved verbatim from the deleted source/devices/clock_host.cpp (sprint 108
// ticket 010) -- no behavior change, only the class names and file
// location.
#include "sim_clock.h"

namespace TestSim {

// ---------------------------------------------------------------------------
// SimClock -- per-instance fake, advances ONLY when stepped.
// ---------------------------------------------------------------------------

uint64_t SimClock::nowMicros() const { return nowMicros_; }

void SimClock::setMicros(uint64_t us) { nowMicros_ = us; }

void SimClock::advanceMicros(uint64_t us) { nowMicros_ += us; }

// ---------------------------------------------------------------------------
// SimSleeper -- records requested sleeps/yields; never blocks.
// ---------------------------------------------------------------------------

void SimSleeper::sleepMillis(uint32_t duration) {
  ++sleepCount_;
  lastSleepMillis_ = duration;
  // No wall-clock block -- a harness advances the paired SimClock
  // explicitly (there is no implicit link between a requested sleep
  // duration and how far the fake clock moves; the harness decides).
}

void SimSleeper::yield() {
  ++yieldCount_;
  // No-op otherwise -- nothing to hand control to in a host test process.
}

}  // namespace TestSim
