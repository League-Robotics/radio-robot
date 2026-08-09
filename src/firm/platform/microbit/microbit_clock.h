// microbit_clock.h — Platform::MicroBitClock / Platform::MicroBitSleeper: the
// real ARM implementations of Platform::Clock / Platform::Sleeper, wrapping
// the CODAL vendor primitives.
//
// MicroBitClock::nowMicros() wraps system_timer_current_time_us() (both
// cooperative, never spins). MicroBitSleeper::sleepMillis() wraps
// fiber_sleep(); MicroBitSleeper::yield() wraps schedule().
//
// Usage: one MicroBitClock and one MicroBitSleeper instance, owned by
// main() and passed to App::RobotLoop (and the modules it composes:
// App::Deadman, App::Preamble) as `Platform::Clock&`/`Platform::Sleeper&`.
#pragma once
#include "MicroBit.h"  // system_timer_current_time_us(), fiber_sleep(), schedule()
#include "platform/clock.h"
#include <cstdint>

namespace Platform {

class MicroBitClock : public Clock {
 public:
  MicroBitClock() = default;

  uint64_t nowMicros() const override;  // [us]
};

class MicroBitSleeper : public Sleeper {
 public:
  MicroBitSleeper() = default;

  void sleepMillis(uint32_t duration) override;  // [ms] settle/pace sleep
  void yield() override;  // hand the processor to another fiber
};

}  // namespace Platform
