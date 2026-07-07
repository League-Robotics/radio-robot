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
// 081-002: implements the abstract Subsystems::Hardware owner base
// (subsystems/hardware.h) — see that file's header and
// architecture-update.md (081) Decision 1 for why the seam lives here, not
// in namespace Hal. kPortCount is now inherited from Hardware, not
// redeclared here.
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
//
// 086-006: also owns the real Hal::OtosOdometer leaf (source/hal/otos/
// otos_odometer.h) and overrides odometer() (Subsystems::Hardware's base
// default, previously nullptr on this class) to return its address. The
// OTOS chip (I2C address 0x17) is NOT folded into the flip-flop sequencer
// above — that sequencer is purely a Nezha (0x10) concern. dev_loop.cpp
// drives the OTOS leaf's own tick()/pose() separately, once per pass,
// exactly as it already does for Subsystems::SimHardware's Hal::SimOdometer
// (081-003) — this class needed no change to that calling convention,
// only to start returning a non-null leaf.
#pragma once

#include <stdint.h>

#include "com/i2c_bus.h"
#include "config/boot_config.h"
#include "hal/capability/hal_command.h"
#include "hal/capability/motor.h"
#include "hal/nezha/nezha_motor.h"
#include "hal/otos/otos_odometer.h"
#include "messages/motor.h"
#include "runtime/queue.h"
#include "subsystems/hardware.h"

namespace Subsystems {

class NezhaHardware : public Hardware {
 public:
  // configs must supply exactly kPortCount entries; configs[i].port should
  // equal i+1 (1..4) — the constructing caller's (main.cpp, ticket 5's)
  // responsibility. NezhaHardware does not itself validate or force this,
  // consistent with "no NezhaHardware-level special-casing." otosConfig
  // (086-006): ticket 086-005's boot-config values (mounting offset +
  // linear/angular scale multipliers), forwarded unchanged to the owned
  // Hal::OtosOdometer leaf's own constructor. Defaulted to
  // Config::OtosBootConfig()'s identity values (zero offset, 1.0 scale) so
  // every pre-086-006 two-argument call site (main.cpp aside, several
  // tests/sim/unit/*_harness.cpp fixtures that construct a NezhaHardware but
  // never call begin()/odometer() on it) keeps compiling unchanged — none of
  // those exercise the OTOS leaf at all, so the default is behaviorally
  // inert for them. Also copies configs[] verbatim into config_[] (087-004)
  // — config()'s backing store.
  NezhaHardware(I2CBus& bus, const msg::MotorConfig configs[kPortCount],
                const Config::OtosBootConfig& otosConfig = Config::OtosBootConfig());

  // Primes all four ports' encoders (see NezhaMotor::begin()) and the OTOS
  // leaf (product-ID detect + init — see Hal::OtosOdometer::begin()).
  void begin() override;

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
  //
  // 087-004: motorIn[]/motorResetIn[] (Subsystems::Hardware's own doc
  // comment has the full contract) are consumed FIRST, uniformly, before
  // the flip-flop's scheduling decision below — applying a motorIn[i]
  // command marks port i+1 in-use (the SAME side effect the apply()
  // overloads below already have — Design Rationale 5: sampling turns on
  // because someone commanded that port), so a port newly brought in-use
  // by this call's motorIn[]/motorResetIn[] is eligible for the SAME call's
  // bus action. motorResetIn[i] does NOT mark the port in-use (mirrors
  // today's direct `hardware->motor(port).resetPosition()` call sites,
  // e.g. pose_commands.cpp's ZERO handler, which never marked in-use
  // either) — this method's own resetPosition() call only stages the
  // reset; landing it still requires the port to already be (or separately
  // become) in-use.
  void tick(uint32_t now, Rt::Mailbox<msg::MotorCommand> motorIn[kPortCount],
            bool motorResetIn[kPortCount]) override;   // [ms]

  // Port-indexed accessor, port in [1, kPortCount]. Always returns the
  // Hal::Motor faceplate — callers (DEV commands, Drivetrain; both later
  // tickets) never see NezhaMotor's raw register verbs. Out-of-range ports
  // clamp to port 4 rather than trapping, since a bad port from a DEV
  // command should surface as ERR at the command layer, not crash the
  // firmware.
  Hal::Motor& motor(uint32_t port) override;

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
  void apply(const Hal::CommandProcessorToHardwareCommand& cmd) override;

  // Both wheels are always addressed (never a broadcast) — the Drivetrain's
  // governed pair is exactly the ports its own DrivetrainConfig binds.
  void apply(const Hal::DrivetrainToHardwareCommand& cmd) override;

  // 086-006: the real OTOS leaf's address — Subsystems::Hardware's base
  // default (nullptr) overridden. NOT folded into the flip-flop scheduler
  // above; dev_loop.cpp drives this leaf's own tick()/pose() separately,
  // once per pass, entirely outside this class's tick().
  Hal::Odometer* odometer() override;

  // config()/state() (087-004, Subsystems::Hardware's own doc comment has
  // the full contract). config(port) returns the constructor-supplied
  // config_[port-1] verbatim (the same value each port's own NezhaMotor
  // leaf was constructed with — see the constructor); state(port) returns
  // motor(port).state() unchanged. Out-of-range ports clamp to port 4,
  // matching motor()'s own convention.
  msg::MotorConfig config(uint32_t port) const override;
  msg::MotorState state(uint32_t port) const override;

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
  Hal::OtosOdometer otosOdometer_;   // 086-006 -- I2C address 0x17, a separate device slot from motorN_'s 0x10

  uint32_t activePort_ = 1;
  Phase phase_ = Phase::REQUEST_DUE;
  bool portInUse_[kPortCount] = {false, false, false, false};

  // config()'s own backing store (087-004) — a verbatim copy of the
  // constructor's configs[] argument, the SAME per-port config each port's
  // own NezhaMotor leaf was constructed with. This ticket adds no way to
  // change it after construction (no Hardware-level configure() exists
  // yet — see hardware.h's own file header); a future ticket that adds one
  // must keep this array and each NezhaMotor leaf's own cached config in
  // sync.
  msg::MotorConfig config_[kPortCount];
};

}  // namespace Subsystems
