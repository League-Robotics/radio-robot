// clock_host.cpp -- HOST_BUILD fake implementation of Types::systemClockNow().
//
// Compiled ONLY when HOST_BUILD is defined, and NEVER linked alongside the
// real source/types/clock.cpp (both files define the same Types::
// symbols; linking both into one binary is a build-configuration error, not
// a supported dual-build) -- exactly the same contract
// source/com/i2c_bus_host.cpp already documents for I2CBus's real/fake
// split. Not yet linked into any build as of ticket 081-002: ticket
// 081-004's CMake source list is what adds this file to the host sim build
// (tests/_infra/sim/CMakeLists.txt), mirroring how i2c_bus_host.cpp was
// written in 079-001 and consumed by later host harnesses.
//
// Clock: HOST_BUILD has no wall clock, so this is a plain settable global --
// no self-advancing spin logic is needed here (unlike I2CBus's clearance
// timers, nothing in this file blocks on the clock reaching a deadline).
#include "types/clock.h"

namespace {
uint32_t g_hostClockNow = 0;   // [ms]
}  // namespace

namespace Types {

uint32_t systemClockNow() {
    return g_hostClockNow;
}

void setHostClockNow(uint32_t now) {
    g_hostClockNow = now;
}

}  // namespace Types
