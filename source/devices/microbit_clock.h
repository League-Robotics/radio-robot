// microbit_clock.h — Devices::MicroBitClock / Devices::MicroBitSleeper: the
// real ARM implementations of Devices::Clock / Devices::Sleeper, wrapping
// the CODAL vendor primitives.
//
// Sprint 108 ticket 010 split (mirrors source/devices/microbit_i2c_bus.h's
// own split for I2CBus, sprint 108 ticket 001). Moved verbatim from the old
// clock.h's `#ifndef HOST_BUILD` fork (clock_real.cpp) — no behavior
// change, only the class names and file split.
//
// MicroBitClock::nowMicros() wraps system_timer_current_time_us() (both
// cooperative, never spins — concurrency contract rule 4: "the device
// fiber's sleeps are what hand time back the other way"). MicroBitSleeper::
// sleepMillis() wraps fiber_sleep(); MicroBitSleeper::yield() wraps
// schedule().
//
// Usage: one MicroBitClock and one MicroBitSleeper instance, owned by
// main() and passed to App::RobotLoop (and the modules it composes:
// App::Deadman, App::Preamble) as `Devices::Clock&`/`Devices::Sleeper&` —
// mirrors main.cpp's own `static Devices::MicroBitI2CBus bus(uBit.i2c);`
// usage note for I2CBus.
#pragma once
#include "MicroBit.h"  // system_timer_current_time_us(), fiber_sleep(), schedule()
#include "devices/clock.h"
#include <cstdint>

namespace Devices {

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

}  // namespace Devices
