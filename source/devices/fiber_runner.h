// fiber_runner.h — Devices::FiberRunner: the seam DeviceBus::start()/stop()
// (device_bus.h/.cpp, DB-008) use to run the fiber's body without hard-wiring
// CODAL's create_fiber() call into device_bus.* directly.
//
// Ticket DB-008 (device-bus-tickets.md). Implements clasi/issues/
// device-bus-fiber-owned-self-contained-device-subsystem.md's "The fiber and
// its cycle" step 2 (`while(!stopRequested_) runCycleOnce();`) and
// device-bus-tickets.md's own DB-008 "Cycle testability" resolution: "the
// real fiber is just while(!stopRequested_) runCycleOnce(); ... A FiberRunner
// seam lets host tests inject a synchronous runner that calls runCycleOnce()
// N times in place of create_fiber, so the lifecycle state machine and
// neutralize-on-exit ordering are host-verified. The real create_fiber path
// is exercised on hardware in DB-009."
//
// --- What run() covers, and what it deliberately does NOT ---
// run(bus) owns the ENTIRE fiber body: bus.runPreamble() once, THEN
// while (!bus.stopRequested()) bus.runCycleOnce(); THEN bus.markLoopExited().
// The preamble has to be INSIDE this call (never run synchronously by
// DeviceBus::start() itself before calling run()) — the issue's own "Because
// this runs in the fiber, retries no longer freeze the control loop or block
// boot" requirement means start() must return to its caller immediately even
// though detection retries can take up to ~1s (color_sensor.h's own
// kMaxAltAttempts * kAltRetryPeriod). run() does NOT call
// bus.neutralizeAllMotors() — that is DeviceBus::stop()'s OWN direct call,
// made AFTER stop() has confirmed (via markLoopExited()) that nothing else
// is still touching the bus. device-bus-tickets.md's own DB-008 wording,
// "stop(): request exit, join, and NEUTRALIZE ALL MOTORS before exit",
// attributes the neutralize step to stop() itself, not to the fiber body's
// own tail — see device_bus.cpp's stop() for the ordering proof.
//
// --- Real vs host, and why this is the one file gated on HOST_BUILD for the
// fiber lifecycle (mirrors i2c_bus.h's own #ifndef HOST_BUILD split) ---
// CodalFiberRunner (#ifndef HOST_BUILD): run() calls create_fiber() and
// returns IMMEDIATELY — the real fiber body keeps running asynchronously
// until bus.stopRequested() flips true and the (already-running, separate)
// fiber notices at its next while-condition check. Exercised on hardware in
// DB-009 (not this ticket) — device_bus.cpp's own comment on the trampoline
// flags the exact create_fiber() signature as DB-009's one thing to verify
// against real CODAL.
// HostFiberRunner (#ifdef HOST_BUILD): run() runs the ENTIRE body
// SYNCHRONOUSLY, in place, right here — but a bare
// `while (!bus.stopRequested())` would hang a single-threaded host test
// process forever (nothing else can call stop() concurrently to ever make
// that condition false). Bounded instead: calls bus.runCycleOnce() up to
// maxCycles_ times (a step budget fixed at construction — the "N times in
// place of create_fiber" device-bus-tickets.md's own DB-008 wording
// specifies), stopping early only if bus.stopRequested() is ALREADY true
// (defensive; never true in practice, since nothing sets it before start()
// returns in a single-threaded test). After the bounded loop, run() marks
// the loop exited immediately — as far as DeviceBus::stop()'s own join is
// concerned, a host-mode fiber has always ALREADY finished running
// everything it is ever going to run by the time start() returns, so
// stop()'s join loop (device_bus.cpp) never spins even once in a host test.
#pragma once
#ifndef HOST_BUILD
#include "MicroBit.h"
#endif

namespace Devices {

class DeviceBus;  // forward decl only — run() is DEFINED out-of-line in
                   // device_bus.cpp (DeviceBus must be a complete type
                   // there); device_bus.h includes THIS header before
                   // DeviceBus's own class body, so only a forward
                   // declaration is available here (mirrors handles.h's
                   // identical DeviceBus forward-decl, same reason).

class FiberRunner {
 public:
  virtual ~FiberRunner() = default;

  // Runs bus's entire fiber body — see this file's own header comment for
  // the exact contract and the real-vs-host split. Defined out-of-line in
  // device_bus.cpp for every concrete implementation below.
  virtual void run(DeviceBus& bus) = 0;
};

#ifndef HOST_BUILD
// Real (CODAL) implementation — exercised on hardware in DB-009.
class CodalFiberRunner : public FiberRunner {
 public:
  void run(DeviceBus& bus) override;

 private:
  // The trampoline create_fiber() actually invokes (device_bus.cpp). A
  // STATIC MEMBER of CodalFiberRunner, not a free function in an anonymous
  // namespace as device_bus.cpp originally had it (DB-008) — DB-009's first
  // real (non-HOST_BUILD) ARM compile of this file caught that the free-
  // function form cannot call DeviceBus::runPreamble()/stopRequested()/
  // markLoopExited() (all private): device_bus.h's own `friend class
  // CodalFiberRunner;` grants friendship to this CLASS, not to an unrelated
  // free function that merely happens to live in the same translation unit.
  // A static member function has an ordinary `void(*)(void*)` function-
  // pointer type (same as create_fiber() requires) while still being a
  // member the friend declaration actually covers — the minimal, root-cause
  // fix, not a workaround (e.g. widening the friend list to a free
  // function, or making the private members public/protected, would both
  // weaken the encapsulation the friend-based design deliberately chose).
  static void codalFiberEntry(void* arg);
};
#endif

#ifdef HOST_BUILD
// Host-synchronous implementation — device_bus_lifecycle_harness.cpp
// (DB-008's own acceptance harness) injects one of these (via
// DeviceBus::setFiberRunner()) configured with whatever step budget each
// scenario needs. DeviceBus also owns one internally (constructed with
// maxCycles == 0) as its own default so a test that only cares about the
// preamble/lifecycle transitions, and never stages a cycle, does not have
// to inject anything at all.
class HostFiberRunner : public FiberRunner {
 public:
  explicit HostFiberRunner(int maxCycles) : maxCycles_(maxCycles) {}

  void run(DeviceBus& bus) override;

 private:
  int maxCycles_;  // bounded step budget — see this file's own header note
};
#endif

}  // namespace Devices
