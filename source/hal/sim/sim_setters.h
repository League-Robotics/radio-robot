// sim_setters.h — one Hal:: free function per sim error/config knob (sprint
// 081-003), each with exactly one canonical call site naming the exact field
// it mutates — so ticket 004's ctypes ABI (sim_api.cpp) has an unambiguous
// 1:1 mapping from a wire-free, ctypes-only knob to the state it touches,
// and this ticket's own test harnesses use the SAME call sites rather than
// poking the underlying setters directly in two different styles.
//
// Namespace correction vs. the design write-up (architecture-update.md (081)
// Decision 2): the design write-up sketched these as `simsetters::` — a
// lowercase namespace, which would violate naming-and-style.md rule 3
// (namespaces are UpperCamelCase) for genuinely new code. There is no new
// namespace here at all: every function below is an ordinary free function
// directly in the existing, already-conforming `namespace Hal`.
//
// Every function here takes only Hal:: types (Hal::PhysicsWorld&,
// Hal::SimMotor&, Hal::SimOdometer&) — never a Subsystems:: type. This is
// deliberate, not an oversight: Subsystems depends on Hal, never the
// reverse (the same direction hardware.h/nezha_hardware.h already establish
// project-wide), so a Hal:: free function cannot take a
// Subsystems::SimHardware& parameter without inverting that direction.
// Ticket 004's sim_api.cpp reaches these the same way this ticket's own
// tests do: through Subsystems::SimHardware's own concrete (non-virtual)
// accessors — plant(), simMotor(port), odometer() — declared in
// subsystems/sim_hardware.h, e.g.
// `Hal::setSimMotorScaleError(simHardware.plant(), /*side=*/0, 0.05f)`.
//
// Headers-only, all-inline — mirrors capability/*.h's own headers-only
// convention (no sim_setters.cpp exists, or is needed).
#pragma once

#include "hal/sim/physics_world.h"
#include "hal/sim/sim_motor.h"
#include "hal/sim/sim_odometer.h"

namespace Hal {

// --- Per-wheel reported-encoder error (mutates PhysicsWorld's REPORTED
// accumulator error model only — the true/ground-truth accumulator and
// chassis pose are unaffected). side: 0 = left, 1 = right, 2 = both. ---

// Mutates PhysicsWorld's encScaleErrL_/encScaleErrR_ (fractional over/
// under-report; 0 = perfect).
inline void setSimMotorScaleError(PhysicsWorld& plant, int side, float err) {
  plant.setEncoderScaleError(side, err);
}

// Mutates PhysicsWorld's encSlipL_/encSlipR_ (fraction of motion not
// registered; 0 = perfect).
inline void setSimMotorSlip(PhysicsWorld& plant, int side, float fraction) {
  plant.setEncoderSlip(side, fraction);
}

// Mutates PhysicsWorld's encNoiseSigmaL_/encNoiseSigmaR_ (Gaussian noise
// sigma, mm per tick).
inline void setSimMotorNoise(PhysicsWorld& plant, int side, float sigma) {  // [mm]
  plant.setEncoderNoise(side, sigma);
}

// --- Motor stiction / lag (mutates the plant's per-wheel actuator-response
// model — applied inside PhysicsWorld::update(), ahead of both encoder
// accumulators). ---

// Mutates PhysicsWorld's stictionPwmL_/stictionPwmR_ (PWM dead-zone
// threshold, 0-100; 0 = never fires).
inline void setSimStiction(PhysicsWorld& plant, int side, float pwm) {
  plant.setStictionPwm(side, pwm);
}

// Mutates PhysicsWorld's motorLagL_/motorLagR_ (first-order response time
// constant; <= 0 = no-op).
inline void setSimMotorLag(PhysicsWorld& plant, int side, float tauMs) {  // [ms]
  plant.setMotorLag(side, tauMs);
}

// Mutates PhysicsWorld's nominalMaxSpeed_ (the duty->velocity plant gain:
// velocity at full duty). The real robot's effective gain differs from the
// compiled 400 mm/s default; exposing it lets a fit match measured cruise.
inline void setSimNominalMaxSpeed(PhysicsWorld& plant, float speed) {  // [mm/s]
  plant.setNominalMaxSpeed(speed);
}

// Mutates PhysicsWorld's coulombDecelL_/R_ (constant velocity-opposing dry
// friction [mm/s^2]; <= 0 = no-op). Damps the terminal reverse-creep.
inline void setSimCoulombFriction(PhysicsWorld& plant, int side, float decel) {  // [mm/s^2]
  plant.setCoulombFriction(side, decel);
}

// --- Chassis-level plant knobs. ---

// Mutates PhysicsWorld's trackwidth_.
inline void setSimTrackwidth(PhysicsWorld& plant, float trackwidth) {  // [mm]
  plant.setTrackwidth(trackwidth);
}

// Mutates PhysicsWorld's bodyRotationalScrub_ (default 1.0 = no-op).
inline void setSimBodyRotationalScrub(PhysicsWorld& plant, float scrub) {
  plant.setBodyRotationalScrub(scrub);
}

// Mutates PhysicsWorld's bodyLinearScrub_ (default 1.0 = no-op).
inline void setSimBodyLinearScrub(PhysicsWorld& plant, float scrub) {
  plant.setBodyLinearScrub(scrub);
}

// --- OTOS (Hal::SimOdometer) error knobs — an independent accumulator;
// never shares state with the encoder error model above. ---

// Mutates SimOdometer's linearNoiseSigma_.
inline void setSimOtosLinearNoise(SimOdometer& odometer, float sigma) {  // [mm]
  odometer.setLinearNoiseSigma(sigma);
}

// Mutates SimOdometer's yawNoiseSigma_.
inline void setSimOtosYawNoise(SimOdometer& odometer, float sigma) {  // [rad]
  odometer.setYawNoiseSigma(sigma);
}

// Mutates SimOdometer's linearScaleErr_.
inline void setSimOtosLinearScaleError(SimOdometer& odometer, float err) {
  odometer.setLinearScaleError(err);
}

// Mutates SimOdometer's angularScaleErr_.
inline void setSimOtosAngularScaleError(SimOdometer& odometer, float err) {
  odometer.setAngularScaleError(err);
}

// Mutates SimOdometer's linearDriftPerTick_.
inline void setSimOtosLinearDrift(SimOdometer& odometer, float driftPerTick) {  // [mm]
  odometer.setLinearDriftPerTick(driftPerTick);
}

// Mutates SimOdometer's yawDriftPerTick_.
inline void setSimOtosYawDrift(SimOdometer& odometer, float driftPerTick) {  // [rad]
  odometer.setYawDriftPerTick(driftPerTick);
}

// --- Plant port binding (mirrors `DEV DT PORTS`) — mutates a SimMotor's
// own plant_/side_ members (Hal::SimMotor::bindToPlant()). Subsystems::
// SimHardware::rebindPlantPorts() (subsystems/sim_hardware.{h,cpp}) is the
// Subsystems-tier convenience wrapper that resolves a port number to the
// right SimMotor instance and calls these; these two functions remain the
// canonical, Hal::-only call sites the port-to-object mapping itself is
// built on top of. ---

// Binds `motor` to `plant`'s LEFT or RIGHT channel.
inline void bindSimMotorToPlant(SimMotor& motor, PhysicsWorld& plant, SimMotor::Side side) {
  motor.bindToPlant(&plant, side);
}

// Unbinds `motor` back to its standalone trivial integrator.
inline void unbindSimMotorFromPlant(SimMotor& motor) {
  motor.bindToPlant(nullptr, SimMotor::Side::LEFT);
}

}  // namespace Hal
