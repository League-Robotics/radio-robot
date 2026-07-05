// clock.cpp -- on-target implementation of Types::systemClockNow(). See
// clock.h for the real/host-fake split this mirrors from i2c_bus.h/.cpp.
#include "types/clock.h"

#include "MicroBit.h"

namespace Types {

uint32_t systemClockNow() {
    return system_timer_current_time();
}

}  // namespace Types
