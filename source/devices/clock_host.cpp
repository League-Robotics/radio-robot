// clock_host.cpp — HOST_BUILD scripted-fake implementation of
// Devices::Clock / Devices::Sleeper. Compiled ONLY when HOST_BUILD is
// defined; never linked alongside clock_real.cpp (see clock.h's file
// header). No CODAL, no wall clock, no real sleeps — every harness under
// tests/sim/unit/ steps time and inspects sleep/yield requests explicitly.
#include "devices/clock.h"

namespace Devices {

// ---------------------------------------------------------------------------
// Clock — per-instance fake, advances ONLY when stepped.
// ---------------------------------------------------------------------------

uint64_t Clock::nowMicros() const { return nowMicros_; }

void Clock::setMicros(uint64_t us) { nowMicros_ = us; }

void Clock::advanceMicros(uint64_t us) { nowMicros_ += us; }

// ---------------------------------------------------------------------------
// Sleeper — records requested sleeps/yields; never blocks.
// ---------------------------------------------------------------------------

void Sleeper::sleepMillis(uint32_t duration) {
  ++sleepCount_;
  lastSleepMillis_ = duration;
  // No wall-clock block — a harness advances the paired Clock explicitly
  // (there is no implicit link between a requested sleep duration and how
  // far the fake Clock moves; the harness decides).
}

void Sleeper::yield() {
  ++yieldCount_;
  // No-op otherwise — nothing to hand control to in a host test process.
}

}  // namespace Devices
