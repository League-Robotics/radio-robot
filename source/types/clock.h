#pragma once

// clock.h -- Types::systemClockNow(): the one seam between "what time is
// it" and the CODAL vendor clock, so command handlers ask a project-owned
// function instead of calling system_timer_current_time() directly.
// commands/system_commands.cpp's handlePing was the one remaining call site
// in the host-clean command set that read the CODAL vendor clock directly
// (081-002) -- this closes it, mirroring the two-fork split
// source/com/i2c_bus.h/.cpp already uses for the same "one real
// implementation, one host-settable fake" shape.
//
//   source/types/clock.cpp      -- on-target: system_timer_current_time().
//   source/types/clock_host.cpp -- HOST_BUILD-only: a settable fake clock,
//                                   driven by setHostClockNow() below. Not
//                                   yet linked into any build as of 081-002
//                                   -- ticket 081-004's CMake source list is
//                                   what adds it to the host sim build,
//                                   mirroring i2c_bus_host.cpp/i2c_bus.cpp's
//                                   own split (see CMakeLists.txt's
//                                   i2c_bus_host.cpp EXCLUDE REGEX for the
//                                   ARM-build-exclusion precedent this
//                                   mirrors for clock_host.cpp).
#include <stdint.h>

namespace Types {

// systemClockNow() -- current time [ms]. Exactly one of clock.cpp (real
// vendor clock) or clock_host.cpp (fake clock) is ever linked into a given
// binary -- both define the same symbol, same as I2CBus's real/HOST_BUILD
// split.
uint32_t systemClockNow();   // [ms]

#ifdef HOST_BUILD
// HOST_BUILD-only fake-clock control (source/types/clock_host.cpp) -- not
// yet callable from any linked build as of 081-002; ticket 081-004 links
// clock_host.cpp into the host sim build, at which point this becomes
// usable the same way i2c_bus.h's setClock()/advanceClock() already are.
void setHostClockNow(uint32_t now);   // [ms]
#endif

}  // namespace Types
