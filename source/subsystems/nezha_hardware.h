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
// in namespace Hal. kMotorCount is now inherited from Hardware, not
// redeclared here.
//
// (0-based motor indices, OOP refactor) motors_ is addressed by a 0-based
// index [0, kMotorCount) — the index IS the motor's identity everywhere in
// this class. The 1-based number printed on the Nezha brick is a config
// input only (configs[i].port, a wire/serialized key — msg::MotorConfig.port
// stays 1-based by convention, unchanged): index i's motor was constructed
// from configs[i], whose own .port field happens to read i+1. No port math
// (`- 1`/`+ 1`/a port-keyed switch) remains anywhere in this class; the
// flip-flop's activeIndex_ cycles the same 0-based range.
//
// This class has two roles on top of its 077 shape (Design Rationale 3,
// clasi/sprints/079-.../architecture-update.md): it is the BRICK FLIP-FLOP
// SEQUENCER — a small activePort_/phase_ state machine that issues at most one
// bus action (a 0x46 encoder request OR a settled collect) per tick() slice,
// cycling only the ports the CONFIGURED poll-set marks polled_ (091-002: a
// constant mask read from configs[].polled at construction, mutable
// thereafter only through setPolled() — see that method's own doc comment)
// — and the hardware DISTRIBUTION POINT — the two apply() overloads that
// forward an addressed msg::MotorCommand to the right concrete
// Hal::NezhaMotor(s), expanding broadcasts to every port. Neither role
// reintroduces left/right pairing or port-role special-casing: apply()'s
// addressing comes entirely from the caller (CommandProcessor's staged DEV M
// target, Drivetrain's own port binding) — NezhaHardware itself still only
// knows about ports, never which one is "left."
//
// 091-002: `apply()`/`tick()` no longer mutate poll-schedule membership as a
// side effect of ordinary command flow — that command-derived, latch-forever
// "in-use" flag (never released, invisible to `SimHardware`) is gone.
// Schedule membership is now a fact fed in from config (`msg::MotorConfig.
// polled`, baked at boot for the drive pair by `gen_boot_config.py`) and
// changed only through the explicit `setPolled()` config-plane door (`DEV M
// <n> CFG polled=<bool>`, reached via `Rt::Configurator`'s `kMotor`
// `ConfigDelta` apply path) — never as a side effect of `DEV M <n> DUTY/VEL/
// POS` or any other motion command. See
// clasi/sprints/091-.../architecture-update.md Decision 1/2 for the full
// rationale (why a config-plane door, not a purely boot-fixed mask; why an
// unpolled port's motion verb is rejected `ERR nodev` rather than silently
// accepted-but-unsampled).
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
#include <array>

#include "hal/capability/motor.h"
#include "hal/nezha/nezha_motor.h"
#include "hal/otos/otos_odometer.h"
#include "messages/motor.h"
#include "subsystems/hardware.h"

namespace Subsystems {

class NezhaHardware : public Hardware {
 public:
  // configs must supply exactly kMotorCount entries; configs[i].port (a
  // wire/serialized key, msg::MotorConfig.port — unchanged, still 1-based)
  // should equal i+1 — the constructing caller's (main.cpp, ticket 5's)
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
  // inert for them. Also copies configs[] verbatim into motorConfigs_[] (087-004)
  // — config()'s backing store.
  NezhaHardware(I2CBus& bus, const msg::MotorConfig configs[kMotorCount],
                const Config::OtosBootConfig& otosConfig = Config::OtosBootConfig());

  // Primes all four motors' encoders (see NezhaMotor::begin()) and the OTOS
  // leaf (product-ID detect + init — see Hal::OtosOdometer::begin()).
  void begin() override;

  // The brick flip-flop sequencer (sprint 079-004; architecture-update.md
  // "The flip-flop and the 078 base-class contract"). Idle (no motor
  // polled_): returns immediately, zero bus actions (decision 1). Otherwise
  // issues exactly one bus-facing action per call: REQUEST_DUE fires the
  // active polled motor's 0x46 encoder request (requestSample()) and
  // advances to COLLECT_DUE; COLLECT_DUE checks bus_.clear(Hal::kNezhaDeviceAddr)
  // — if the settle window has not yet elapsed, this call is a no-op pass;
  // once clear, it collects (the active motor's full NezhaMotor::tick(),
  // the 078 base/leaf 5-step contract) and advances to the next polled
  // motor's REQUEST_DUE. Two calls per main-loop pass (the sanctioned
  // "slice 1 collects due, slice 2 requests/writes go out" double call,
  // ticket 005) drive one full request/collect pair per pass under typical
  // timing.
  //
  // (093/094 teardown) motorIn[]/motorResetIn[] consumption is gone —
  // Subsystems::Hardware's own tick() doc comment has the full contract.
  // tick() now runs ONLY the flip-flop's scheduling decision below;
  // whether motor i is eligible for THIS call's bus action depends
  // solely on motorPolled_[i], the constant (except via setPolled()) mask
  // established at construction.
  void tick(uint32_t now) override;   // [ms]

  // Index-addressed accessor, i in [0, kMotorCount). Always returns the
  // Hal::Motor faceplate — callers (DEV commands, Drivetrain; both later
  // tickets) never see NezhaMotor's raw register verbs. Out-of-range
  // indices clamp to kMotorCount-1 rather than trapping, since a bad index
  // derived from a DEV command should surface as ERR at the command layer,
  // not crash the firmware.
  Hal::Motor& motor(uint32_t i) override;

  // Distribution (sprint 079-004; architecture-update.md "The command-edge
  // types"). Both overloads forward the addressed msg::MotorCommand(s) to
  // the target NezhaMotor(s) via their own apply(). 091-002: neither
  // overload touches poll-schedule membership at all any more — a target's
  // eligibility for tick()'s cycling schedule depends solely on the
  // constant motorPolled_[] mask (config-established, setPolled()-mutable),
  // never on whether apply() was ever called for it. allPorts==true
  // forwards addressed[0].command to every motor's setter unconditionally,
  // regardless of each motor's own polled_ state.
  void apply(const Hal::CommandProcessorToHardwareCommand& cmd) override;

  // Both wheels are always addressed (never a broadcast) — the Drivetrain's
  // governed pair is exactly the motors its own DrivetrainConfig binds.
  void apply(const Hal::DrivetrainToHardwareCommand& cmd) override;

  // 086-006: the real OTOS leaf's address — Subsystems::Hardware's base
  // default (nullptr) overridden. NOT folded into the flip-flop scheduler
  // above; dev_loop.cpp drives this leaf's own tick()/pose() separately,
  // once per pass, entirely outside this class's tick().
  Hal::Odometer* odometer() override;

  // config()/state() (087-004, Subsystems::Hardware's own doc comment has
  // the full contract). config(i) returns the constructor-supplied
  // motorConfigs_[i] verbatim (the same value motor i's own NezhaMotor leaf was
  // constructed with — see the constructor); state(i) returns
  // motor(i).state() unchanged. Out-of-range indices clamp to
  // kMotorCount-1, matching motor()'s own convention.
  msg::MotorConfig motorConfig(uint32_t i) const override;
  msg::MotorState motorState(uint32_t i) const override;

  // setPolled() (091-002, Subsystems::Hardware's own doc comment has the
  // full contract) — the ONE way motorPolled_[] changes after construction.
  // Reached exclusively via Rt::Configurator's kMotor ConfigDelta apply
  // path (`DEV M <n> CFG polled=<bool>`). Out-of-range indices clamp to
  // kMotorCount-1, matching motor()'s own convention.
  void setMotorPolled(uint32_t i, bool polled) override;

 private:
  // REQUEST_DUE: the next bus action is a fresh 0x46 request on
  // activeIndex_. COLLECT_DUE: the next bus action (once
  // bus_.clear(Hal::kNezhaDeviceAddr) confirms the settle window elapsed) is
  // that same motor's collect + full tick().
  enum class Phase : uint8_t { REQUEST_DUE, COLLECT_DUE };

  // The next polled index at or after cur, wrapping [0, kMotorCount). Only
  // ever called when anyPolled() is true (tick()'s idle-schedule guard), so
  // a match always exists; if none did, cur is returned unchanged
  // (defensive — should not be reached).
  uint32_t nextPolled(uint32_t cur) const;

  // True if at least one motor is currently polled_ — the idle-schedule gate
  // (decision 1).
  bool anyPolled() const;

  // clampIndex() — the ONE place an out-of-range index clamps to
  // kMotorCount-1 (mirrors the pre-refactor motorAt()'s port-4 clamp,
  // preserved behavior, now expressed once instead of once per switch).
  static uint32_t clampIndex(uint32_t i) { return (i < kMotorCount) ? i : kMotorCount - 1; }

  I2CBus& bus_;

  // motors_[i] is motor index i's leaf — the index IS its identity. Index 0
  // is physical port 1 (the flip-flop's traditional first collect target),
  // index 1 is physical port 2 (the drive pair, same physical motors/order
  // as before this class stored motor1_.. motor4_ separately) — configs[]
  // is unchanged (already 0-based; configs[i].port is the wire label i+1).
  std::array<Hal::NezhaMotor, kMotorCount> motors_;
  Hal::OtosOdometer otosOdometer_;   // 086-006 -- I2C address 0x17, a separate device slot from motors_'s 0x10

  uint32_t activeIndex_ = 0;
  Phase phase_ = Phase::REQUEST_DUE;

  // 091-002: the configured poll-set — which motors the flip-flop schedules.
  // Established once at construction from configs[].polled (constant
  // thereafter except through setPolled(), above); never mutated by
  // tick()/apply() as a side effect of ordinary command flow (that was
  // the old "in-use" flag's bug — see this file's own header comment).
  bool motorPolled_[kMotorCount] = {false, false, false, false};

  // config()'s own backing store (087-004) — a verbatim copy of the
  // constructor's configs[] argument, the SAME per-motor config each
  // motor's own NezhaMotor leaf was constructed with. This ticket adds no
  // way to change it after construction (no Hardware-level configure()
  // exists yet — see hardware.h's own file header); a future ticket that
  // adds one must keep this array and each NezhaMotor leaf's own cached
  // config in sync.
  msg::MotorConfig motorConfigs_[kMotorCount];
};

}  // namespace Subsystems
