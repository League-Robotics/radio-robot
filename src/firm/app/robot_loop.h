// robot_loop.h -- App::RobotLoop: the boot loop and main per-cycle schedule.
// Compiles under -DHOST_BUILD (no MicroBit.h) via the Devices::Clock&/
// Devices::Sleeper& time seam instead of raw vendor timer/sleep calls.
//
// Two entry points: run() is what main.cpp calls -- boot() once, then
// cycle() forever (never returns). A host test instead calls boot() and
// cycle() directly so it can step a bounded number of cycles and inspect
// state in between.
//
// Timing primitives: runAndWait(gap, body) == markTime(); body();
// sleepUntil(mark, gap) -- the block visibly scopes exactly the work that
// borrows the wait; the body itself never touches the bus and never
// sleeps. `grep 'runAndWait\|sleepUntil'` on this file is the firmware's
// complete timing schedule. Built on Devices::Clock::nowMicros() (converted
// [us] -> [ms]) and Devices::Sleeper::sleepMillis() -- see devices/clock.h's
// own file header for the real vs. HOST_BUILD impls.
//
// Design/rationale: DESIGN.md.
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
  // cycle body touches by name (main.cpp on ARM, or a host harness, owns
  // construction and wiring order). bus is needed directly for the cycle
  // body's own bus.clearanceSafetyNetCount() fault read; color/line leaves
  // are NOT referenced here -- Preamble already holds them and is called by
  // name, never reached into directly.
  RobotLoop(Devices::I2CBus& bus, Devices::NezhaMotor& motorL,
            Devices::NezhaMotor& motorR, Devices::Otos& otos, Comms& comms,
            Telemetry& tlm, Drive& drive, Odometry& odom, Deadman& deadman,
            Preamble& preamble, const Devices::Clock& clock,
            Devices::Sleeper& sleeper);

  // Runs boot() once, then cycle() forever. Never returns -- this is what
  // main.cpp's int main() calls after constructing real hardware.
  [[noreturn]] void run();

  // Boot loop: `preamble.step()` until `preamble.done()`, staging/emitting
  // a boot telemetry frame each pass and pacing via
  // sleeper_.sleepMillis(kPreamblePace). Sets kEventBootReady on the
  // done() first-true transition, then returns.
  void boot();

  // One pass of the main cycle body (the runAndWait/markTime/sleepUntil
  // schedule and the command-dispatch switch). Call boot() first --
  // cycle() assumes every device is already resolved; no readiness checks
  // happen below this line.
  void cycle();

 private:
  uint32_t markTime() const;                    // [ms]
  void sleepUntil(uint32_t mark, uint32_t gap);  // [ms] [ms]

  template <typename Body>
  void runAndWait(uint32_t gap, Body body);  // [ms]



  // Update tlm_ from bus_/motorL_/motorR_/ comms_. 
  void updateTlm();

  // Dispatches the <=1 decoded command in cmd to its own handler by
  // cmd_kind (NONE is a no-op). Each handler applies its command and acks
  // via the telemetry ack ring.
  void processMessage(const Cmd& cmd);
  void handleTwist(const msg::CommandEnvelope& env);
  void handleConfig(const msg::CommandEnvelope& env);
  void handleStop(const msg::CommandEnvelope& env);

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

  // Persists across cycle() calls. Each field is written by the part of
  // the cycle that owns it (encoder/vel/conn after motorL_/motorR_'s own
  // tick(); pose after odom_.integrate(); otos via applyOtosSample()) and
  // read back whole by the NEXT cycle's tlm_.setFrame()/emit() call --
  // Telemetry always carries the last staged snapshot, so a field updated
  // late in one cycle is simply one cycle "stale" when it reaches the
  // wire, never lost.
  bool driving_ = false;  // true once a Twist is applied, cleared on Stop/deadman
  Telemetry::Frame frame_;
};

}  // namespace App
