// microbit_clock.cpp — Devices::MicroBitClock / Devices::MicroBitSleeper
// real implementation. Compiled ONLY in the real ARM build (this file
// includes MicroBit.h, so it is never part of the HOST_BUILD graph).
#include "devices/microbit_clock.h"

namespace Devices {

uint64_t MicroBitClock::nowMicros() const {
  return system_timer_current_time_us();  // [us]
}

void MicroBitSleeper::sleepMillis(uint32_t duration) {
  // [ms] CODAL's fiber_sleep(): "the calling thread will be immediately
  // descheduled, and placed onto a wait queue until the requested amount of
  // time has elapsed" — a cooperative yield, never a spin.
  fiber_sleep(static_cast<unsigned long>(duration));
}

void MicroBitSleeper::yield() {
  // CODAL's schedule(): "yield control of the processor when you have
  // nothing more to do" — a bare scheduling point, no timed wait.
  schedule();
}

}  // namespace Devices
