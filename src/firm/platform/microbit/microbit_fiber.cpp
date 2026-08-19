// microbit_fiber.cpp — Platform::MicroBitFiberLauncher real implementation.
#include "platform/microbit/microbit_fiber.h"

#include "MicroBit.h"  // codal create_fiber()

namespace Platform {

void MicroBitFiberLauncher::launch(void (*entry)(void*), void* context) {
  // CODAL's parameterized overload: the new fiber runs entry(context) on
  // its own stack under the cooperative scheduler. The kernel's entry
  // never returns, so the completion_fn default (release_fiber) never
  // fires.
  ::create_fiber(entry, context);
}

}  // namespace Platform
