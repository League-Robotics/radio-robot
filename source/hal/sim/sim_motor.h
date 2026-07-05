// sim_motor.h — Hal::SimMotor: the simulated wheel-motor leaf (sprint
// 081-003), implementing the SAME Hal::Motor faceplate NezhaMotor implements
// (source/hal/capability/motor.h) — apply()/state()/the sprint-078 armor
// policy (armoredWrite/processResetIfPending/updateRestTracking) all come
// free from that shared base; this class supplies only the primitive
// setters/getters, tick(), and capabilities().
//
// DUTY mode stages the commanded duty straight into the plant (stiction gate
// + optional lag are applied INSIDE Hal::PhysicsWorld::update(), not here).
// VELOCITY mode calls Hal::MotorVelocityPid::compute() — ticket 081-001's
// exact class, the SAME one Hal::NezhaMotor calls — never a re-derived
// approximation (the design write-up's own highest-flagged correction).
// POSITION is unsupported (capabilities().position == false): Motor::apply()
// gates it before it ever reaches setPosition() (Motor::apply()'s capability
// gate, motor.h), so `DEV M <n> POS` answers `ERR unsupported`, matching a
// Nezha that lacked the capability.
//
// wedged()/wedgeSuspect() deliberately stay at Hal::Motor's base-class false
// default forever in v1: this class's tick() does NOT call
// updateWedgeDetector(). A real Nezha's raw wedge latch models a genuine
// hardware failure mode (a stuck I2C readback); the simulated plant has no
// analogous fault to detect without deliberate fault injection, which is out
// of this ticket's scope (see clasi/issues/later/sim-hardware-fault-
// injection.md, already filed — referenced here, not re-filed). Note this is
// NOT simply "an idle motor would falsely latch" — motor.h's own STATE
// semantics document that an idle real motor legitimately latches wedged()
// too (docs/protocol-v2.md's `wedged=` field is explicitly "benign... even
// for an idle motor at rest") — the reason to skip it here is that there is
// no injected-fault signal for the latch to ever mean anything beyond that
// same benign idle case, so running it would add cost with no informative
// value until fault injection lands.
//
// Two constructor shapes, matching architecture-update.md (081)'s port-
// binding description ("binds port 1->plant LEFT, port 2->plant RIGHT by
// default; ports 3/4 unbound trivial standalone integrators"):
//   - Plant-bound: this motor's actuator write and encoder read go through
//     one shared Hal::PhysicsWorld's LEFT or RIGHT channel (setActuator/
//     reportedEncL()/R()) — the two motors that are actually driving the
//     simulated chassis.
//   - Standalone: no PhysicsWorld reference at all; a trivial, dependency-
//     free integrator (position += (pwmPct/100)*PhysicsWorld::kNominalMaxSpeed
//     * dt, no slip/noise/stiction/lag) so an unbound port still reports
//     plausible, moving state instead of dead zeros. bindToPlant() converts
//     a standalone motor to plant-bound (or back) at runtime — this is what
//     Subsystems::SimHardware::rebindPlantPorts() (mirroring `DEV DT PORTS`)
//     uses via sim_setters.h's Hal::bindSimMotorToPlant()/
//     Hal::unbindSimMotorFromPlant().
#pragma once

#include <stdint.h>

#include "hal/capability/motor.h"
#include "hal/sim/physics_world.h"
#include "hal/velocity_pid.h"
#include "messages/motor.h"

namespace Hal {

class SimMotor : public Motor {
 public:
  enum class Side : uint8_t { LEFT = 0, RIGHT = 1 };

  // Plant-bound: reads/writes plant's LEFT or RIGHT channel.
  SimMotor(PhysicsWorld& plant, Side side, const msg::MotorConfig& config);

  // Standalone: a trivial, plant-free integrator (see file header).
  explicit SimMotor(const msg::MotorConfig& config);

  // Primes the encoder (parity with NezhaMotor::begin(), which calls
  // hardReset()) — zeroes whichever accumulator this motor is bound to.
  void begin() override;

  // Rebind this motor to a plant side, or unbind it (plant == nullptr) back
  // to the standalone trivial integrator. See sim_setters.h's
  // Hal::bindSimMotorToPlant()/Hal::unbindSimMotorFromPlant() — the
  // canonical call sites for the "plant port binding" knob.
  void bindToPlant(PhysicsWorld* plant, Side side);

  // --- Primitive setters (Hal::Motor) ---
  void setDutyCycle(float dutyCycle) override;      // [-1, 1]
  void setVoltage(float voltage) override;           // [V] unsupported — capabilities().voltage == false
  void setVelocity(float velocity) override;         // [mm/s] signed
  void setPosition(float position) override;         // [deg] unsupported — capabilities().position == false
  void setNeutral(msg::Neutral mode) override;
  void setFeedforward(float feedforward) override;   // [V]

  // --- Primitive getters (Hal::Motor) ---
  float position() const override;     // [mm]
  float velocity() const override;     // [mm/s] signed, filtered
  float appliedDuty() const override;  // [-1, 1]
  bool connected() const override;     // always true — no I2C link to fail

  // --- Faceplate verbs (Hal::Motor) ---
  void tick(uint32_t now) override;    // [ms]
  msg::MotorCapabilities capabilities() const override;

 protected:
  // --- Device-specific armor primitives (Hal::Motor, sprint 078 contract) ---
  void writeRawDuty(float duty) override;    // [-1, 1] stages straight into the plant (or the standalone integrator)
  void hardReset() override;                 // zeroes the bound/standalone encoder accumulator
  void softRebaseline() override;            // sim has no I2C timing race to avoid — same effect as hardReset(), different counter
  void configureDevice(const msg::MotorConfig& config) override;

 private:
  enum class Mode : uint8_t { NONE, DUTY, VELOCITY, NEUTRAL };

  int sideIndex() const { return static_cast<int>(side_); }

  // Reads this motor's current encoder position [mm]: the plant's reported
  // (errored) accumulator when bound, or the standalone integrator's own
  // accumulator when unbound.
  float encoderPosition() const;

  // Advances the standalone (unbound) trivial integrator by dt [ms]. No-op
  // when plant-bound (the shared PhysicsWorld::update() advances that path
  // instead, driven once per pass by Subsystems::SimHardware::tick()).
  void tickStandalone(uint32_t dt);  // [ms]

  PhysicsWorld* plant_ = nullptr;   // nullptr => standalone trivial integrator
  Side side_ = Side::LEFT;
  msg::MotorConfig config_;

  // ---- Staged command (set by the primitive setters; executed by tick()) ----
  Mode mode_ = Mode::NONE;
  float dutyTarget_ = 0.0f;                        // [-1, 1]
  float velocityTarget_ = 0.0f;                    // [mm/s]
  msg::Neutral neutralTarget_ = msg::Neutral::COAST;
  float feedforward_ = 0.0f;

  // ---- tick() encoder-sample cache (mirrors NezhaMotor's own) ----
  float lastPosition_ = 0.0f;          // [mm]
  float filteredVelocity_ = 0.0f;      // [mm/s] EMA-filtered; fed to pid_ and velocity()
  uint32_t lastTick_ = 0;              // [ms]
  bool hasLastTick_ = false;

  // ---- Embedded velocity PID — ticket 081-001's shared class, the SAME
  // one Hal::NezhaMotor embeds. config_ (vel_gains/min_duty) stays the
  // single source of truth for calibration; pid_ owns only the
  // integrator's persistent state. ----
  Hal::MotorVelocityPid pid_;

  // ---- Standalone (unbound) trivial integrator state — used only when
  // plant_ == nullptr. ----
  float standaloneEnc_ = 0.0f;   // [mm]
  int8_t standalonePwm_ = 0;     // [-100, 100]
};

}  // namespace Hal
