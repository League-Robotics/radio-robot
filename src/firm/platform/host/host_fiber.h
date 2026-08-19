// host_fiber.h -- TestSim::FailingFiberLauncher: the host build's
// Hal::FiberLauncher, which FAILS if it is ever used.
//
// There are no fibers in the host build, and there must not be. The host
// harness drives Control::DifferentialDrive::step() directly, one cycle
// per call, which is what makes host tests deterministic -- no scheduler,
// no interleaving, no wall-clock. A harness that called start() would
// quietly acquire a SECOND control loop racing the one the test is
// stepping, and the resulting flakiness would look like a control-law
// problem rather than a harness problem.
//
// So this does not no-op. A no-op launcher would let start() succeed,
// report running() == true, and leave the test subtly wrong. It aborts,
// loudly, at the exact call.
#pragma once

#include <cstdio>
#include <cstdlib>

#include "hal/fiber.h"

namespace TestSim {

class FailingFiberLauncher : public Hal::FiberLauncher {
 public:
  void launch(void (*)(void*), void*) override {
    std::fprintf(stderr,
                 "FATAL: Hal::FiberLauncher::launch() called in the HOST "
                 "build.\n"
                 "The host has no fibers by design -- a harness drives\n"
                 "Control::DifferentialDrive::step() directly, one cycle per\n"
                 "call. Something called start(), which would race a second\n"
                 "control loop against the one the test is stepping.\n"
                 "Fix the caller; do not make this a no-op.\n");
    std::abort();
  }
};

}  // namespace TestSim
