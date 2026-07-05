// nezha_hardware.h — Subsystems::NezhaHardware: the top-level hardware
// subsystem for the Nezha controller. Owns the shared I2CBus plus one
// Hal::NezhaMotor per port (up to four), and orchestrates the split-phase bus
// schedule across them.
//
// This is a Subsystems-tier peer of Subsystems::Drivetrain — the
// aggregator/scheduler/distributor that genuinely IS a subsystem, as opposed
// to a per-device faceplate. It moved here (from namespace Hal /
// source/hal/nezha/) for exactly that reason; the individual hardware
// elements it owns — Hal::NezhaMotor and the Hal::Motor faceplate it hands
// back — stay in namespace Hal / source/hal/. The dependency direction is
// unchanged and un-inverted: Subsystems depends on Hal (this subsystem names
// Hal::NezhaMotor / Hal::Motor / Hal::*Command), never the reverse.
//
// This class has two roles on top of its 077 shape (Design Rationale 3,
// clasi/sprints/079-.../architecture-update.md): it is the BRICK FLIP-FLOP
// SEQUENCER — a small activePort_/phase_ state machine that issues at most one
// bus action (a 0x46 encoder request OR a settled collect) per tick() slice,
// cycling only the ports some command has actually addressed ("in-use" — see
// apply() below) — and the hardware DISTRIBUTION POINT — the two apply()
// overloads that mark ports in-use and forward an addressed msg::MotorCommand
// to the right concrete Hal::NezhaMotor(s), expanding broadcasts to every
// port. Neither role reintroduces left/right pairing or port-role
// special-casing: apply()'s addressing comes entirely from the caller
// (CommandProcessor's staged DEV M target, Drivetrain's own port binding) —
// NezhaHardware itself still only knows about ports, never which one is
// "left."
#pragma once

#include <stdint.h>

#include "com/i2c_bus.h"
#include "hal/capability/hal_command.h"
#include "hal/capability/motor.h"
#include "hal/nezha/nezha_motor.h"
#include "messages/motor.h"

namespace Subsystems {

class NezhaHardware {
 public:
  static constexpr uint32_t kPortCount = 4;

  // configs must supply exactly kPortCount entries; configs[i].port should
  // equal i+1 (1..4) — the constructing caller's (main.cpp, ticket 5's)
  // responsibility. NezhaHardware does not itself validate or force this,
  // consistent with "no NezhaHardware-level special-casing."
  NezhaHardware(I2CBus& bus, const msg::MotorConfig configs[kPortCount]);

  // Primes all four ports' encoders (see NezhaMotor::begin()).
  void begin();

  // The brick flip-flop sequencer (sprint 079-004; architecture-update.md
  // "The flip-flop and the 078 base-class contract"). Idle (no port
  // in-use): returns immediately, zero bus actions (decision 1). Otherwise
  // issues exactly one bus-facing action per call: REQUEST_DUE fires the
  // active in-use port's 0x46 encoder request (requestSample()) and
  // advances to COLLECT_DUE; COLLECT_DUE checks bus_.clear(Hal::kNezhaDeviceAddr)
  // — if the settle window has not yet elapsed, this call is a no-op pass;
  // once clear, it collects (the active port's full NezhaMotor::tick(),
  // the 078 base/leaf 5-step contract) and advances to the next in-use
  // port's REQUEST_DUE. Two calls per main-loop pass (the sanctioned
  // "slice 1 collects due, slice 2 requests/writes go out" double call,
  // ticket 005) drive one full request/collect pair per pass under typical
  // timing.
  void tick(uint32_t now);   // [ms]

  // Port-indexed accessor, port in [1, kPortCount]. Always returns the
  // Hal::Motor faceplate — callers (DEV commands, Drivetrain; both later
  // tickets) never see NezhaMotor's raw register verbs. Out-of-range ports
  // clamp to port 4 rather than trapping, since a bad port from a DEV
  // command should surface as ERR at the command layer, not crash the
  // firmware.
  Hal::Motor& motor(uint32_t port);

  // Distribution (sprint 079-004; architecture-update.md "The command-edge
  // types"). Both overloads forward the addressed msg::MotorCommand(s) to
  // the target NezhaMotor(s) via their own apply(); addressed (non-
  // broadcast) targets are also marked in-use, which is what brings them
  // into tick()'s cycling schedule (decision 1: sampling turns on because
  // someone commanded that port, never as a side effect of a broadcast —
  // see Design Rationale 5).
  //
  // allPorts==true never marks any port in-use, even though it still
  // forwards addressed[0].command to every port's setter.
  void apply(const Hal::CommandProcessorToHardwareCommand& cmd);

  // Both wheels are always addressed (never a broadcast) — the Drivetrain's
  // governed pair is exactly the ports its own DrivetrainConfig binds.
  void apply(const Hal::DrivetrainToHardwareCommand& cmd);

 private:
  // REQUEST_DUE: the next bus action is a fresh 0x46 request on
  // activePort_. COLLECT_DUE: the next bus action (once
  // bus_.clear(Hal::kNezhaDeviceAddr) confirms the settle window elapsed) is
  // that same port's collect + full tick().
  enum class Phase : uint8_t { REQUEST_DUE, COLLECT_DUE };

  // motorAt(): the concrete Hal::NezhaMotor& behind a port, for the
  // scheduler's and apply()'s internal use. motor() (public, above) returns
  // the same object narrowed to the Hal::Motor faceplate — implemented in
  // terms of this so the port-indexing switch exists exactly once.
  Hal::NezhaMotor& motorAt(uint32_t port);

  // The next in-use port at or after cur, wrapping 1..kPortCount. Only
  // ever called when anyPortInUse() is true (tick()'s idle-schedule guard),
  // so a match always exists; if none did, cur is returned unchanged
  // (defensive — should not be reached).
  uint32_t nextPortInUse(uint32_t cur) const;

  // True if at least one port has ever been individually addressed (see
  // apply()'s in-use marking) — the idle-schedule gate (decision 1).
  bool anyPortInUse() const;

  I2CBus& bus_;
  Hal::NezhaMotor motor1_;
  Hal::NezhaMotor motor2_;
  Hal::NezhaMotor motor3_;
  Hal::NezhaMotor motor4_;

  uint32_t activePort_ = 1;
  Phase phase_ = Phase::REQUEST_DUE;
  bool portInUse_[kPortCount] = {false, false, false, false};
};

}  // namespace Subsystems
