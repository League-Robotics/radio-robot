// clock.h — Devices::Clock / Devices::Sleeper: the time/yield seam the
// loop's own cycle is parameterized on. Plain virtual bases (no
// preprocessor forks); the real ARM implementation lives in
// microbit_clock.h/.cpp, the host-test fake under tests/.
//
// Design/rationale: DESIGN.md.
//
// Clock/Sleeper are a SEPARATE seam from I2CBus's own internal clearance-
// timer clock. The loop owns one Clock and one Sleeper instance and uses
// them for its own cycle-level time reads — publish() stamps, staleness
// deadlines, cycle pacing — not the bus's clearance windows.
#pragma once
#include <cstdint>

namespace Devices {

// Clock — [us] time source the fiber cycle reads via nowMicros(). The real
// impl wraps system_timer_current_time_us(); the test-infra impl is a
// per-instance fake that only moves when a test calls setMicros()/
// advanceMicros() — never advances on its own (unlike I2CBus's fake, which
// self-advances during a live entry-spin; Clock has no such spin to
// self-advance out of).
class Clock {
 public:
  virtual ~Clock() = default;

  virtual uint64_t nowMicros() const = 0;  // [us]
};

// Sleeper — the settle/pace-sleep and yield surface the fiber cycle sleeps
// through. The real impl wraps CODAL's fiber_sleep() (settle/pace sleeps)
// and schedule() (a bare yield — "hand control of the processor to another
// waiting fiber" per CodalFiber.h). The test-infra impl records every
// requested sleep/yield without any wall-clock block, so a harness can step
// deterministically and assert what the cycle actually asked for.
class Sleeper {
 public:
  virtual ~Sleeper() = default;

  virtual void sleepMillis(uint32_t duration) = 0;  // [ms] settle/pace sleep
  virtual void yield() = 0;  // hand the processor to another fiber
};

}  // namespace Devices
