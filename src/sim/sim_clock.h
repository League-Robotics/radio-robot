// sim_clock.h -- TestSim::SimClock / TestSim::SimSleeper: the steppable
// host-test fakes for Devices::Clock / Devices::Sleeper.
//
// Sprint 108 ticket 010 (clasi/issues/plan-pure-i2cbus-clock-interfaces-a-
// real-simplant-simulator.md, Stage 5). Ticket 010 reduced
// `Devices::Clock`/`Devices::Sleeper` to pure interfaces
// (source/devices/clock.h); SimClock/SimSleeper are the SECOND concrete
// implementations, alongside `Devices::MicroBitClock`/`MicroBitSleeper`
// (source/devices/microbit_clock.h) on the ARM side -- mirrors
// tests/_infra/sim/sim_plant.h's own relationship to
// Devices::MicroBitI2CBus (ticket 001/002).
//
// Moved verbatim from the deleted source/devices/clock_host.cpp's own
// `#ifdef HOST_BUILD` fork -- no behavior change, only the class names and
// file location. SimClock is a per-instance fake that advances ONLY when a
// test calls setMicros()/advanceMicros() -- never on its own. SimSleeper
// records every requested sleepMillis()/yield() call without blocking on a
// wall clock or sleeping for real, so a harness can step deterministically
// and assert exactly what the cycle asked for.
//
// Source placement: HOST_BUILD-only test infrastructure -- this file does
// NOT live in source/ (architecture-update.md Decision 2, "source/ holds
// only interfaces + ARM impls"), matching sim_plant.h's own placement.
#pragma once

#include <cstdint>

#include "devices/clock.h"

namespace TestSim {

// SimClock -- per-instance fake, advances ONLY when stepped.
class SimClock : public Devices::Clock {
 public:
  SimClock() = default;

  uint64_t nowMicros() const override;  // [us]

  void setMicros(uint64_t us);      // [us] set the fake clock directly
  void advanceMicros(uint64_t us);  // [us] step the fake clock forward

 private:
  uint64_t nowMicros_ = 0;  // [us] advances ONLY when stepped
};

// SimSleeper -- records requested sleeps/yields; never blocks.
class SimSleeper : public Devices::Sleeper {
 public:
  SimSleeper() = default;

  void sleepMillis(uint32_t duration) override;
  void yield() override;

  // Inspection surface.
  int sleepCount() const { return sleepCount_; }
  uint32_t lastSleepMillis() const { return lastSleepMillis_; }  // [ms]
  int yieldCount() const { return yieldCount_; }

 private:
  int sleepCount_ = 0;
  uint32_t lastSleepMillis_ = 0;  // [ms]
  int yieldCount_ = 0;
};

}  // namespace TestSim
