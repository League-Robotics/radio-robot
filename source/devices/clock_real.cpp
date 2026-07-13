// clock_real.cpp — Devices::Clock / Devices::Sleeper real (non-HOST_BUILD)
// implementation. Compiled ONLY in the real ARM build; never linked
// alongside clock_host.cpp (both files define the same Devices::Clock::/
// Devices::Sleeper:: symbols — see clock.h's file header).
#include "devices/clock.h"
#include "MicroBit.h"  // system_timer_current_time_us(), fiber_sleep(), schedule()

namespace Devices {

uint64_t Clock::nowMicros() const {
  return system_timer_current_time_us();  // [us] vendor SDK call — excluded
                                           // from the no-units-in-identifiers
                                           // rename (coding-standards.md)
}

void Sleeper::sleepMillis(uint32_t duration) {
  // [ms] CODAL's fiber_sleep(): "the calling thread will be immediately
  // descheduled, and placed onto a wait queue until the requested amount of
  // time has elapsed" — a cooperative yield, never a spin.
  fiber_sleep(static_cast<unsigned long>(duration));
}

void Sleeper::yield() {
  // CODAL's schedule(): "yield control of the processor when you have
  // nothing more to do" — a bare scheduling point, no timed wait.
  schedule();
}

}  // namespace Devices
