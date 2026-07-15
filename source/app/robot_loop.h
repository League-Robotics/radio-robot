// robot_loop.h -- App::RobotLoop: the boot loop and main per-cycle schedule
// extracted from source/main.cpp (sprint 103 ticket 008's original "single
// loop"), now parameterized on Devices::Clock&/Devices::Sleeper& instead of
// raw vendor timer/sleep calls, so it compiles under -DHOST_BUILD with no
// MicroBit.h dependency anywhere in the compiled translation units.
//
// This ticket (105-001) is a MECHANICAL extraction: main.cpp's boot loop
// (`while (!preamble.done())`) and main `for(;;)` cycle body (the
// runAndWait/markTime/sleepUntil schedule and the command-dispatch switch)
// move here VERBATIM in logic -- only the timing primitives themselves
// change from raw system_timer_current_time()/uBit.sleep() calls to
// clock_.nowMicros()/sleeper_.sleepMillis(). Zero intended behavior change
// on ARM; a diff-level review and the standing hardware bench gate
// (.claude/rules/hardware-bench-testing.md) both verify this.
//
// architecture-update.md (105) Step 3 "RobotLoop" boundary: inside -- the
// cycle's own fixed call sequence and timing primitives; outside -- device
// construction (the caller's job -- main.cpp on ARM, a future sim harness
// on host) and each called module's own internal behavior (unchanged).
// Serves SUC-018, and transitively every later ticket in sprint 105 (none
// of them can build a host-side harness until this class exists and is
// bench-proven identical to the pre-extraction main.cpp).
//
// --- Two entry points ---
// run() is what main.cpp calls: boot() once, then cycle() forever (never
// returns). A host test instead calls boot() and cycle() directly so it can
// step a bounded number of cycles and inspect state in between -- this is
// the "boot()/cycle() pair" shape the ticket's own implementation plan
// allows as an alternative to a single opaque run() when it "reads more
// naturally against the existing Preamble/main-loop split."
//
// --- Timing primitives (moved from main.cpp's anonymous namespace) ---
// runAndWait(gap, body) == markTime(); body(); sleepUntil(mark, gap): the
// block visibly scopes exactly the work that borrows the wait; the body
// itself never touches the bus and never sleeps. `grep 'runAndWait\|
// sleepUntil'` on this file is still the firmware's complete timing
// schedule -- unchanged by the move to source/app/robot_loop.cpp. Built on
// Devices::Clock::nowMicros() (converted [us] -> [ms]) and
// Devices::Sleeper::sleepMillis() -- the real impls wrap
// system_timer_current_time_us()/fiber_sleep() (clock_real.cpp), the
// HOST_BUILD impls are per-instance steppable fakes a harness advances
// explicitly (clock_host.cpp) -- see devices/clock.h's own file header.
#pragma once

#include <cstdint>

#include "app/comms.h"
#include "app/deadman.h"
#include "app/drive.h"
#include "app/odometry.h"
#include "app/preamble.h"
#include "app/telemetry.h"
#include "devices/clock.h"
#include "devices/i2c_bus.h"
#include "devices/nezha_motor.h"
#include "devices/otos.h"

namespace App {

class RobotLoop {
 public:
  // Every reference below is an already-constructed leaf/app module the
  // extracted cycle body itself touches by name (main.cpp on ARM, or a
  // future host harness, owns construction and wiring order -- unchanged
  // from the pre-extraction main.cpp's own "construction order matches
  // device_bus.h's own documented rationale" comment). bus is needed
  // directly for the cycle body's own bus.clearanceSafetyNetCount() fault
  // read; color/line leaves are NOT referenced here -- Preamble already
  // holds them and is called by name, never reached into directly, exactly
  // as in the pre-extraction main.cpp.
  RobotLoop(Devices::I2CBus& bus, Devices::NezhaMotor& motorL,
            Devices::NezhaMotor& motorR, Devices::Otos& otos, Comms& comms,
            Telemetry& tlm, Drive& drive, Odometry& odom, Deadman& deadman,
            Preamble& preamble, const Devices::Clock& clock,
            Devices::Sleeper& sleeper);

  // Runs boot() once, then cycle() forever. Never returns -- this is what
  // main.cpp's int main() calls after constructing real hardware.
  [[noreturn]] void run();

  // The pre-extraction main.cpp's own boot loop, byte-identical in
  // ordering: `preamble.step()` until `preamble.done()`, staging/emitting a
  // boot telemetry frame each pass and pacing via sleeper_.sleepMillis(
  // kPreamblePace) (was uBit.sleep(kPreamblePace)). Sets kEventBootReady on
  // the done() first-true transition, then returns.
  void boot();

  // One pass of the pre-extraction main.cpp's own `for(;;)` cycle body (the
  // runAndWait/markTime/sleepUntil schedule and the command-dispatch
  // switch), byte-identical in ordering. Call boot() first -- cycle()
  // assumes every device is already resolved, matching the pre-extraction
  // loop's own "no readiness checks below this line" comment.
  void cycle();

 private:
  uint32_t markTime() const;                    // [ms]
  void sleepUntil(uint32_t mark, uint32_t gap);  // [ms] [ms]

  template <typename Body>
  void runAndWait(uint32_t gap, Body body);  // [ms]

  Devices::I2CBus& bus_;
  Devices::NezhaMotor& motorL_;
  Devices::NezhaMotor& motorR_;
  Devices::Otos& otos_;
  Comms& comms_;
  Telemetry& tlm_;
  Drive& drive_;
  Odometry& odom_;
  Deadman& deadman_;
  Preamble& preamble_;
  const Devices::Clock& clock_;
  Devices::Sleeper& sleeper_;

  // Persists across cycle() calls -- moved verbatim from main.cpp's own
  // pre-loop locals. Each field is written by the part of the cycle that
  // owns it (encoder/vel/conn after motorL_/motorR_'s own tick(); pose
  // after odom_.integrate(); otos via applyOtosSample()) and read back
  // whole by the NEXT cycle's tlm_.setFrame()/emit() call -- Telemetry
  // itself always carries "the last staged snapshot" (telemetry.h's own
  // doc comment), so a field updated late in one cycle is simply one cycle
  // "stale" when it reaches the wire, never lost.
  bool driving_ = false;  // true once a Twist is applied, cleared on Stop/deadman
  Telemetry::Frame frame_;
};

}  // namespace App
