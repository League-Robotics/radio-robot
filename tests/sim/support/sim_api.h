// sim_api.h -- TestSim::SimApi: the composed, steppable HOST_BUILD harness
// ticket 105-004 (SUC-021) builds. Wires the REAL App::RobotLoop (ticket
// 001) + the REAL plant (ticket 003, tests/sim/plant/{wheel,otos}_plant.h)
// + the REAL FakeTransport (ticket 002, tests/sim/support/fake_transport.h)
// + a scripted Devices::I2CBus + a fake Devices::Clock/Sleeper into ONE
// reusable object other test binaries link against instead of each
// re-deriving the composition -- this sprint's own ticket 006 pytest
// scenarios, and sprint 106's future profile-validation work
// (architecture-update.md Step 3 "sim_api" boundary).
//
// --- File placement decision (105-004 AC #5 / architecture-update.md Step 7
// Open Question 1) ---
// Lives at tests/sim/support/, NOT tests/sim/system/, even though the
// architecture doc's own Open Question 1 offered both as equally-acceptable
// options ("colocated with its primary consumer" vs "parallel to
// tests/sim/unit/'s per-module harnesses"). Decision: tests/sim/support/,
// because SimApi's own primary consumers are NOT colocated with it --
// ticket 006 (a DIFFERENT file, tests/sim/system/*) and sprint 106 (a
// DIFFERENT sprint) both consume it as a library, exactly the role
// TestSupport::FakeTransport (this same directory) already established for
// itself ("mirrors comms.h's own documentation density/style... later
// tickets (004's sim_api, 006's pytest scenarios) build on it too",
// fake_transport.h's own file header). tests/sim/system/'s own README
// scopes that directory to "whole-robot scenario tests" -- the scenario
// FILES themselves (this ticket's own sim_api_harness.cpp, ticket 006's
// scripted-twist pytest scenarios), not the shared library those scenario
// files link against.
//
// --- What this class does NOT do ---
// It does not decode/encode wire bytes itself (TestSupport::wire_test_codec,
// this same directory, owns that -- SimApi calls it, does not reimplement
// it) and it does not know anything about a SPECIFIC scenario's plant
// tuning beyond the fixed gains/config this file documents below (a
// scenario wanting different plant behavior constructs its own SimApi
// instance -- there is no shared mutable global state).
//
// --- Plant/PID tuning (why the numbers below) ---
// Both motors run with PID ENABLED (the default, matching what a twist
// command actually drives through App::Drive -- NezhaMotor::setVelocity()
// is only consumed when pidEnabled_ is true) and a deliberately large
// proportional gain (kp = 0.01, ki = kff = iMax = kaw = 0) plus a slew rate
// wide enough to reach full duty in ONE write (slewRate = 100, vs. the
// production-realistic 0.0f every other harness in this codebase uses,
// which -- see nezha_motor.cpp's clampStep() -- makes duty permanently
// stuck at whatever the FIRST write was; this harness's own scenarios need
// REAL duty movement, so slewRate must be nonzero). Any twist target this
// harness ever injects has |v_x| well above TestSim::kDefaultDutyVelMax (the
// plant's own achievable ceiling), so the velocity error the PID chases
// NEVER shrinks below (target - kDefaultDutyVelMax) for the life of any
// scenario -- with kp = 0.01 that error alone (>= 500 * 0.01 = 5.0) always
// clamps the PID's output to +-1.0. The practical consequence, verified by
// hand against nezha_motor.cpp's write-on-change/write-rate/slew gates
// (see this file's own sim_api.cpp scriptCycleBusResponses() comment): every
// actuation change this harness ever provokes (the initial mode-activation
// write, a fresh twist, an explicit stop, or a deadman expiry) is a SINGLE,
// immediately-saturated duty write, never a multi-write slew ramp or a
// write-rate-throttled sequence -- which keeps this harness's own required
// EXACT shared-I2CBus-FIFO script counts (105-003's own "CRITICAL prior
// finding") tractable by hand instead of needing a full control-law replica.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "app/comms.h"
#include "app/deadman.h"
#include "app/drive.h"
#include "app/odometry.h"
#include "app/preamble.h"
#include "app/robot_loop.h"
#include "app/telemetry.h"
#include "devices/clock.h"
#include "devices/color_sensor.h"
#include "devices/device_config.h"
#include "devices/i2c_bus.h"
#include "devices/line_sensor.h"
#include "devices/nezha_motor.h"
#include "devices/otos.h"
#include "fake_transport.h"
#include "otos_plant.h"
#include "wheel_plant.h"
#include "wire_test_codec.h"

namespace TestSim {

// The virtual-cycle-timing diagnostic's own output shape (105-004 AC #3) --
// see sim_api.cpp's measureOneCycle() for the derivation of virtualMillis.
struct CycleTimingReport {
  int sleepCount = 0;              // Devices::Sleeper::sleepCount() delta across the sampled cycle() call
  uint32_t lastSleepMillis = 0;    // [ms] Sleeper::lastSleepMillis() after the call -- the FINAL (cycle-pace) block
  int yieldCount = 0;              // Devices::Sleeper::yieldCount() delta (App::RobotLoop::cycle() never calls
                                    // Sleeper::yield() directly -- always 0; kept for completeness/future use)
  uint32_t virtualCycleMillis = 0; // [ms] derived total virtual schedule for one cycle() call -- see .cpp
};

class SimApi {
 public:
  SimApi();

  // Advances the sim. Two mutually exclusive phases (this ticket's own
  // implementation decision, per architecture-update.md Step 7 Open
  // Question 1's "or until done, per ticket 004's own implementation"):
  //   - Not yet booted: drives App::Preamble to done() (see sim_api.cpp's
  //     driveBootToDone() for why this can't happen INSIDE a single
  //     App::RobotLoop::boot() call), then calls robotLoop_.boot() itself
  //     (a real call -- by the time it runs, preamble_.done() is already
  //     true, so its own while-loop body never executes; the call still
  //     exercises the ACTUAL boot() method, including its
  //     setEvent(kEventBootReady, true) tail). Consumes this ENTIRE step()
  //     call regardless of `cycles`' value -- boot is atomic, not partially
  //     steppable.
  //   - Already booted: calls robotLoop_.cycle() `cycles` times, scripting
  //     this cycle's I2CBus responses from the plant's CURRENT state
  //     before each call (per ticket 003's own "plant driven BY the
  //     harness, between cycles" boundary), then advancing the fake Clock
  //     by kCycleDt before the call (see sim_api.cpp for why BEFORE, not
  //     after).
  void step(int cycles);

  // Pushes one complete armored ("*B...") line onto the inbound serial
  // FakeTransport -- App::Comms::pump() consumes at most one per cycle()
  // call (comms.h's own documented contract), so a line enqueued
  // immediately before a step(N) call is consumed on that call's FIRST
  // cycle.
  void injectCommand(const char* armoredLine);

  // Convenience wrappers over injectCommand() + TestSupport::armor*Command()
  // -- there is no encode(CommandEnvelope) in the generated codec (only a
  // host builds commands; see wire_test_codec.h's own file header), so
  // these are what every scenario in this ticket (and ticket 006's) actually
  // calls. Also registers the resulting actuation-change cycle for the
  // scripting helper below -- see sim_api.cpp.
  void injectTwist(float v_x, float omega, float duration, uint32_t corrId = 0);
  void injectStop(uint32_t corrId = 0);

  // A caller that provokes an actuation change WITHOUT injecting a new
  // command (this ticket's own deadman-expiry scenario: the change is the
  // ABSENCE of a fresh command over time, not a new one arriving) calls
  // this directly with the hand-computed cycle index the expiry will first
  // be observed on (see sim_api.cpp's scriptCycleBusResponses() comment for
  // why deadman expiry produces the identical single-cycle write-on-change
  // signature a fresh command does).
  void notePendingActuationChange(int atCycle);

  // Decodes and returns every outbound line captured on the serial
  // FakeTransport since the last call (both FakeTransport instances receive
  // an IDENTICAL broadcast -- App::Comms::sendReply()/App::Telemetry's own
  // secondary-frame send both fan out to serial AND radio -- so draining
  // just one is sufficient and avoids duplicate decoded frames).
  std::vector<TestSupport::DecodedLine> drainTelemetry();

  bool booted() const { return booted_; }
  int cycleCount() const { return cycleCount_; }  // total robotLoop_.cycle() calls made so far

  // Timing-diagnostic surface (105-004 AC #3) -- Devices::Sleeper's own
  // HOST_BUILD inspection surface, exposed read-only, plus a derived
  // one-cycle report. measureOneCycle() calls step(1) (must already be
  // booted) and returns the observed+derived breakdown for THAT cycle.
  int sleepCount() const { return sleeper_.sleepCount(); }
  uint32_t lastSleepMillis() const { return sleeper_.lastSleepMillis(); }
  int yieldCount() const { return sleeper_.yieldCount(); }
  CycleTimingReport measureOneCycle();

  Devices::NezhaMotor& motorLeft() { return motorL_; }
  Devices::NezhaMotor& motorRight() { return motorR_; }
  Devices::Clock& clock() { return clock_; }

  // [ms] the fixed per-cycle virtual-time advance step() applies before
  // every robotLoop_.cycle() call -- >=40ms so a fresh duty write is never
  // write-rate-throttled (nezha_motor.cpp's kMinWriteIntervalUs) and
  // comfortably >= Devices::Otos::kReadPeriod (20ms, otos.h) so OTOS is
  // due every single cycle (scriptCycleBusResponses() relies on this: no
  // "only cycle 0" special case the way app_robot_loop_harness.cpp's own
  // frozen-clock scenario needed).
  static constexpr uint32_t kCycleDtUs = 50000;  // [us]

 private:
  void driveBootToDone();
  void scriptCycleBusResponses();

  Devices::I2CBus bus_;
  Devices::Clock clock_;
  Devices::Sleeper sleeper_;

  Devices::NezhaMotor motorL_;
  Devices::NezhaMotor motorR_;
  Devices::Otos otos_;
  Devices::ColorSensorLeaf color_;
  Devices::LineSensorLeaf line_;

  TestSupport::FakeTransport serialLink_;
  TestSupport::FakeTransport radioLink_;

  App::Comms comms_;
  App::Telemetry tlm_;
  App::Deadman deadman_;
  App::Drive drive_;
  App::Odometry odom_;
  App::Preamble preamble_;

  App::RobotLoop robotLoop_;

  WheelPlant plantLeft_;
  WheelPlant plantRight_;
  OtosPlant otosPlant_;

  bool booted_ = false;
  int cycleCount_ = 0;              // total robotLoop_.cycle() calls made so far
  int pendingEventCycle_ = -1;      // cycleCount_ value at which R's duty write is expected to
                                     // change (L's follows one cycle later) -- -1 == none pending
  size_t telemetryDrainIndex_ = 0;  // index into serialLink_.sent() already returned by drainTelemetry()
};

}  // namespace TestSim
