// microbit_fiber.h — Platform::MicroBitFiberLauncher: the real ARM
// implementation of Hal::FiberLauncher, wrapping CODAL's create_fiber().
// Counterpart to microbit_clock.h's MicroBitClock/MicroBitSleeper — same
// seam pattern, same ownership (constructed once by main(), passed by
// reference).
#pragma once
#include "hal/fiber.h"

namespace Platform {

class MicroBitFiberLauncher : public Hal::FiberLauncher {
 public:
  void launch(void (*entry)(void*), void* context) override;
};

}  // namespace Platform
