// clock.h — Devices::Clock / Devices::Sleeper: the time/yield seam the
// loop's own cycle is parameterized on.
//
// Ticket DB-003 (device-bus-tickets.md; issue "Sim / host-test story": "The
// cycle body is parameterized on a sleeper/clock interface: fiber_sleep +
// system_timer on hardware; the steppable fake clock in host tests.").
//
// Real impls (clock_real.cpp, compiled #ifndef HOST_BUILD) wrap the CODAL
// vendor primitives `system_timer_current_time_us()` (Clock) and
// `fiber_sleep()`/`schedule()` (Sleeper) — both cooperative, never spins
// (concurrency contract rule 4: "the device fiber's sleeps are what hand
// time back the other way"). HOST_BUILD impls (clock_host.cpp) are
// per-instance steppable fakes a harness advances/inspects explicitly — no
// wall-clock reads, no real sleeps.
//
// This mirrors i2c_bus.h's own #ifndef HOST_BUILD / #ifdef HOST_BUILD split
// exactly: one class declared here per seam, two mutually-exclusive .cpp
// forks selected by the HOST_BUILD build config, never linked together.
//
// Clock/Sleeper are a SEPARATE seam from I2CBus's own internal fake clock
// (I2CBus::setClock()/advanceClock(), used only for that class's per-
// transaction clearance-timer bookkeeping). The loop owns one Clock and one
// Sleeper instance and uses them for its own cycle-level time reads —
// publish() stamps, staleness deadlines, cycle pacing — not the bus's
// clearance windows.
#pragma once
#ifndef HOST_BUILD
#include "MicroBit.h"
#endif
#include <cstdint>

namespace Devices {

// Clock — [us] time source the fiber cycle reads via nowMicros(). Real impl
// wraps system_timer_current_time_us(); HOST_BUILD impl is a per-instance
// fake that only moves when a test calls setMicros()/advanceMicros() —
// never advances on its own (unlike I2CBus's fake, which self-advances
// during a live entry-spin; Clock has no such spin to self-advance out of).
class Clock {
 public:
  Clock() = default;

  uint64_t nowMicros() const;  // [us]

#ifdef HOST_BUILD
  // HOST_BUILD-only stepping surface.
  void setMicros(uint64_t us);      // [us] set the fake clock directly
  void advanceMicros(uint64_t us);  // [us] step the fake clock forward
#endif

 private:
#ifdef HOST_BUILD
  uint64_t nowMicros_ = 0;  // [us] advances ONLY when stepped
#endif
};

// Sleeper — the settle/pace-sleep and yield surface the fiber cycle sleeps
// through. Real impl wraps CODAL's fiber_sleep() (settle/pace sleeps) and
// schedule() (a bare yield — "hand control of the processor to another
// waiting fiber" per CodalFiber.h). HOST_BUILD impl records every requested
// sleep/yield without any wall-clock block, so a harness can step
// deterministically and assert what the cycle actually asked for.
class Sleeper {
 public:
  Sleeper() = default;

  void sleepMillis(uint32_t duration);  // [ms] settle/pace sleep
  void yield();                         // hand the processor to another fiber

#ifdef HOST_BUILD
  // HOST_BUILD-only inspection surface.
  int sleepCount() const { return sleepCount_; }
  uint32_t lastSleepMillis() const { return lastSleepMillis_; }  // [ms]
  int yieldCount() const { return yieldCount_; }
#endif

 private:
#ifdef HOST_BUILD
  int sleepCount_ = 0;
  uint32_t lastSleepMillis_ = 0;  // [ms]
  int yieldCount_ = 0;
#endif
};

}  // namespace Devices
