// microbit_fiber.h — Platform::MicroBitFiberRunner: the real ARM
// implementation of Hal::FiberRunner, wrapping CODAL's create_fiber().
// Counterpart to microbit_clock.h's MicroBitClock/MicroBitSleeper — same
// seam pattern, same ownership (constructed once by main(), passed by
// reference).
#pragma once
#include "hal/fiber.h"

namespace Platform {

class MicroBitFiberRunner : public Hal::FiberRunner {
 public:
  void createFiber(void (*entry)(void*), void* context) override;
};

}  // namespace Platform
