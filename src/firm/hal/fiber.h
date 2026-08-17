// fiber.h — Hal::FiberRunner: the create-a-fiber seam the wheel kernel's
// start() is parameterized on. Same shape as clock.h's Clock/Sleeper: a
// plain virtual base with the real ARM implementation in
// platform/microbit/microbit_fiber.h/.cpp (wrapping CODAL's
// create_fiber()) and a host-test fake in platform/host/sim_clock.h
// (TestSim::SimFiberRunner — records the request; a harness drives the
// kernel's cycle directly instead of running a real thread).
//
// One verb only: launch a fiber that runs `entry(context)` forever. There
// is deliberately no join/kill surface — CODAL fibers are cooperative and
// the kernel fiber is designed to run for the lifetime of the boot; a
// caller that needs the kernel quiescent uses the kernel's own estop/
// neutral surface, not fiber lifecycle.
#pragma once

namespace Hal {

class FiberRunner {
 public:
  virtual ~FiberRunner() = default;

  // Launch a cooperative fiber running entry(context). The entry function
  // is expected never to return.
  virtual void createFiber(void (*entry)(void*), void* context) = 0;
};

}  // namespace Hal
