// sim_hardware.h — Subsystems::SimHardware: the simulated-device owner/
// scheduler (sprint 081-003), a Subsystems-tier peer of
// Subsystems::NezhaHardware implementing ticket 002's Subsystems::Hardware
// seam — NOT a Hal:: leaf beside Hal::SimMotor/Hal::SimOdometer. See
// architecture-update.md (081) Decision 2 for why this class lives here,
// under source/subsystems/, in namespace Subsystems, rather than under
// source/hal/sim/ in namespace Hal: it owns one Hal::PhysicsWorld plus four
// Hal::SimMotor leaves plus one Hal::SimOdometer leaf and runs a
// tick-cadence/scheduling policy over them (the dt=0 re-entry guard below)
// — it aggregates and schedules, it does not itself implement one device's
// primitive setters/getters, exactly the distinction
// subsystems/nezha_hardware.h's own header comment draws for its class.
//
// Port binding (default): port 1 -> plant LEFT, port 2 -> plant RIGHT (the
// robot's normal drive pair, matching docs/protocol-v2.md's `DEV DT PORTS`
// default `1 2`); ports 3/4 are unbound, trivial standalone integrators
// (Hal::SimMotor's own file header) — the coupled-bench-rig pair `3 4` a
// real robot might also drive. rebindPlantPorts() moves the plant binding
// to a different port pair at runtime, mirroring `DEV DT PORTS <left>
// <right>` reconfiguring which two ports Subsystems::Drivetrain governs.
//
// THE dt=0 re-entry guard (architecture-update.md (081) Decision 4): this
// is the ticket's single most important, non-obvious contract.
// devLoopTick() (source/dev_loop.cpp) calls `hardware.tick(now)` TWICE per
// ordinary pass with the SAME `now` (slice 1 + slice 2) — not only during
// a ctypes synchronous-command replay trick. Subsystems::Hardware's own
// file header already documents the general contract every concrete owner
// must satisfy ("safe to call tick() twice in the same pass with an
// UNCHANGED now"); Subsystems::NezhaHardware satisfies it only incidentally
// (the I2C bus's microsecond-resolution clearance timer naturally blocks a
// same-now second collect). SimHardware has no equivalent bus latency to
// lean on, so it tracks its own lastAdvancedNow_ and treats a repeated call
// with an unchanged `now` as a COMPLETE no-op: no Hal::SimMotor::tick()
// call, no Hal::PhysicsWorld::update() call, for ANY of the four motors —
// otherwise each SimMotor's embedded Hal::MotorVelocityPid::compute() would
// silently double-integrate every ordinary pass.
#pragma once

#include <stdint.h>

#include "hal/capability/hal_command.h"
#include "hal/capability/motor.h"
#include "hal/sim/physics_world.h"
#include "hal/sim/sim_motor.h"
#include "hal/sim/sim_odometer.h"
#include "messages/motor.h"
#include "subsystems/hardware.h"

namespace Subsystems {

class SimHardware : public Hardware {
 public:
  // configs must supply exactly kPortCount entries; configs[i].port should
  // equal i+1 (1..4) — mirrors Subsystems::NezhaHardware's constructor
  // contract exactly (the constructing caller's responsibility; SimHardware
  // does not itself validate or force this).
  explicit SimHardware(const msg::MotorConfig configs[kPortCount]);

  // Primes all four ports' encoders (see Hal::SimMotor::begin()).
  void begin() override;

  // The dt=0 re-entry guard (Decision 4) — see file header. A call with an
  // unchanged `now` is a complete no-op; otherwise every port's
  // Hal::SimMotor::tick(now) runs (in port order), then the plant advances
  // exactly once (Hal::PhysicsWorld::update()), then the odometer samples
  // the just-advanced true pose (Hal::SimOdometer::tick()).
  //
  // (093/094 teardown) motorIn[]/motorResetIn[] consumption is gone --
  // Subsystems::Hardware's own tick() doc comment has the full contract.
  void tick(uint32_t now) override;   // [ms]

  // Port-indexed accessor, port in [1, kPortCount]. Always returns the
  // Hal::Motor faceplate, exactly like Subsystems::NezhaHardware::motor() —
  // callers never see Hal::SimMotor's concrete surface through this seam.
  Hal::Motor& motor(uint32_t port) override;

  // Distribution — both overloads simply forward the addressed
  // msg::MotorCommand(s) to the target Hal::SimMotor(s)' own apply(); no
  // in-use/lazy-scheduling bookkeeping is needed here (unlike
  // NezhaHardware's brick flip-flop), since ticking all four simulated
  // motors every pass carries no bus cost to economize.
  void apply(const Hal::CommandProcessorToHardwareCommand& cmd) override;
  void apply(const Hal::DrivetrainToHardwareCommand& cmd) override;

  // config()/state() (087-004, Subsystems::Hardware's own doc comment has
  // the full contract). config(port) returns the constructor-supplied
  // config_[port-1] verbatim; state(port) returns motor(port).state()
  // unchanged. Out-of-range ports clamp to port 4, matching motor()'s own
  // convention.
  msg::MotorConfig config(uint32_t port) const override;
  msg::MotorState state(uint32_t port) const override;

  // The one Hal::Odometer leaf this owner has (082-003's Subsystems::Hardware
  // seam override) — never nullptr for SimHardware, unlike
  // Subsystems::NezhaHardware's inherited default. Returns the SAME
  // odometer_ member simOdometer() (below) exposes concretely; this overload
  // is reached through the abstract Subsystems::Hardware* base pointer
  // (devLoopTick()'s own call site), simOdometer() through the concrete
  // type (error-knob setters, ground-truth reads).
  Hal::Odometer* odometer() override { return &odometer_; }

  // --- Test / ctypes-facing surface (concrete type only — reached by
  // holding Subsystems::SimHardware directly, never through the abstract
  // Subsystems::Hardware* base pointer; architecture-update.md (081)
  // Decision 2's Consequences). sim_setters.h's free Hal:: functions take
  // these accessors' return types directly, e.g.
  // `Hal::setSimMotorScaleError(simHardware.plant(), 0, 0.05f)`. ---
  Hal::PhysicsWorld& plant() { return plant_; }

  // simOdometer() — the CONCRETE Hal::SimOdometer, needed for error-knob
  // setters and truth reads that use SimOdometer's own surface (e.g.
  // sim_setters.h's OTOS-noise functions), not just the Hal::Odometer*
  // faceplate seam odometer() (above) exposes. Renamed from this class's
  // pre-082-003 `odometer()` accessor the moment Hardware's own odometer()
  // became a real virtual seam that had to return Hal::Odometer* — same
  // "sim-prefixed concrete twin" naming already established by simMotor()
  // (below) for the identical motor()/simMotor() duality.
  Hal::SimOdometer& simOdometer() { return odometer_; }

  // Port-indexed accessor to the CONCRETE Hal::SimMotor, port in [1,
  // kPortCount] — for error-knob setters and truth reads that need
  // Hal::SimMotor's own surface (e.g. sim_setters.h's port-binding
  // functions), not just the Hal::Motor faceplate motor() (above) exposes.
  Hal::SimMotor& simMotor(uint32_t port);

  // Rebinds the plant's LEFT/RIGHT physical channels to a different port
  // pair, mirroring `DEV DT PORTS <left> <right>`. The two newly-bound
  // ports' Hal::SimMotor instances become plant-bound (Hal::SimMotor::Side::
  // LEFT/RIGHT respectively); the two ports vacated revert to their own
  // standalone trivial integrators. Uses sim_setters.h's
  // Hal::bindSimMotorToPlant()/Hal::unbindSimMotorFromPlant() — the
  // canonical Hal::-only call sites for this knob.
  void rebindPlantPorts(uint32_t leftPort, uint32_t rightPort);

 private:
  Hal::SimMotor& motorAt(uint32_t port);

  Hal::PhysicsWorld plant_;
  Hal::SimMotor motor1_;
  Hal::SimMotor motor2_;
  Hal::SimMotor motor3_;
  Hal::SimMotor motor4_;
  Hal::SimOdometer odometer_;

  // The two ports currently bound to the plant's LEFT/RIGHT channels —
  // defaults to {1, 2}; tracked so rebindPlantPorts() knows which
  // currently-bound ports to unbind first.
  uint32_t leftPort_ = 1;
  uint32_t rightPort_ = 2;

  // The dt=0 re-entry guard's own state (Decision 4) — see file header.
  uint32_t lastAdvancedNow_ = 0;   // [ms]
  bool hasAdvanced_ = false;

  // config()'s own backing store (087-004) — a verbatim copy of the
  // constructor's configs[] argument. This ticket adds no way to change it
  // after construction (no Hardware-level configure() exists yet).
  msg::MotorConfig config_[kPortCount];
};

}  // namespace Subsystems
