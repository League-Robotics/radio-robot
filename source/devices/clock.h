// clock.h — Devices::Clock / Devices::Sleeper: the time/yield seam the
// loop's own cycle is parameterized on.
//
// Sprint 108 ticket 010 (clasi/issues/plan-pure-i2cbus-clock-interfaces-a-
// real-simplant-simulator.md, Stage 5). These used to be concrete classes
// with an `#ifdef HOST_BUILD` fork inside the SAME header (a scripted-fake
// member/method surface compiled in for host tests, the real CODAL wrapper
// compiled in for ARM). They are now plain virtual bases — same style as
// `App::Transport` (source/app/comms.h), `Devices::MotorArmor`
// (source/devices/motor_armor.h), and `Devices::I2CBus`
// (source/devices/i2c_bus.h, sprint 108 ticket 001) — so this header never
// drags in MicroBit.h and has zero preprocessor forks.
//
// The real ARM implementation lives in `source/devices/microbit_clock.h/
// .cpp` (`Devices::MicroBitClock` wraps `system_timer_current_time_us()`;
// `Devices::MicroBitSleeper` wraps `fiber_sleep()`/`schedule()`). The
// steppable/inspectable host fake lives in `tests/_infra/sim/sim_clock.h/
// .cpp` (`TestSim::SimClock`/`TestSim::SimSleeper`) — test infrastructure,
// not a `source/` concern (mirrors i2c_bus.h's own split: `source/` holds
// only interfaces + ARM impls).
//
// Clock/Sleeper are a SEPARATE seam from I2CBus's own internal fake clock
// (I2CBus::setClock()/advanceClock(), used only for that class's per-
// transaction clearance-timer bookkeeping). The loop owns one Clock and one
// Sleeper instance and uses them for its own cycle-level time reads —
// publish() stamps, staleness deadlines, cycle pacing — not the bus's
// clearance windows.
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
