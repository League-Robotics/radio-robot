// fiber.h — Hal::FiberLauncher: the create-a-fiber seam the wheel kernel's
// start() is parameterized on. Same shape as clock.h's Clock/Sleeper: a
// plain virtual base with the real ARM implementation in
// platform/microbit/microbit_fiber.h/.cpp (wrapping CODAL's
// create_fiber()) and a host-test fake in platform/host/sim_clock.h
// (TestSim::SimFiberLauncher — records the request; a harness drives the
// kernel's cycle directly instead of running a real thread).
//
// One verb only: launch a fiber that runs `entry(context)` forever. There
// is deliberately no join/kill surface — CODAL fibers are cooperative and
// the kernel fiber is designed to run for the lifetime of the boot; a
// caller that needs the kernel quiescent uses the kernel's own estop/
// neutral surface, not fiber lifecycle.
#pragma once

namespace Hal {

class FiberLauncher {
 public:
  virtual ~FiberLauncher() = default;

  // Launch a cooperative fiber running entry(context). The entry function
  // is expected never to return.
  //
  // The HOST implementation deliberately FAILS if this is ever called.
  // The host test harness drives the kernel's step() directly and must
  // never start a fiber -- making that a hard failure rather than a
  // convention means a harness that accidentally calls start() is caught
  // immediately instead of quietly running a second control loop.
  virtual void launch(void (*entry)(void*), void* context) = 0;
};

}  // namespace Hal
