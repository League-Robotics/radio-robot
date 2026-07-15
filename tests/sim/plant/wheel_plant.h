// wheel_plant.h -- TestSim::WheelPlant: a deterministic, seeded stand-in for
// one physical wheel + Nezha motor-controller channel on the bench.
//
// Ticket 105-003 (SUC-020). Per architecture-update.md Decision 2, this
// plant is LEAF-GETTER-DRIVEN, not bus-byte-driven: it never intercepts a
// raw Devices::I2CBus write payload (the HOST_BUILD scripted fake does not
// even record one -- i2c_bus.h's own comment). Instead it reads
// Devices::NezhaMotor::appliedDuty() (a public getter reflecting whatever
// armor/slew/write-on-change already decided was actually written to the
// simulated hardware) and integrates a first-order duty->velocity->position
// response, then SCHEDULES the resulting encoder reading onto the shared
// Devices::I2CBus for the leaf's NEXT requestSample()/tick() pair to
// consume -- the exact two-write-one-read convention
// devices_motor_harness.cpp's scriptEncoderRequestCollect() already
// establishes (scenario 6, "PID-on chases a velocity target").
//
// NOT PORTED FROM THE DELETED SIM: no formula here is carried over from the
// deleted `drive/` v2 sim plant (SimMotor/PhysicsWorld, removed sprint 102)
// -- this class is built fresh, per the sprint's own carried caution (the
// deleted sim's 180/360-degree pivot runs both converged on ~272-273
// degrees, a suspected angle-wrap attractor in ITS OWN heading math, never
// root-caused). This file carries NO heading state of its own at all --
// see otos_plant.h for where heading actually lives (Odometry's own
// integration, via BodyKinematics::forward() over two WheelPlant
// positions) and architecture-update.md Decision 3 for the full rationale.
//
// Deterministic/seeded: every quantity here is plain, non-random float
// arithmetic -- there is no RNG anywhere in this class's nominal path, so
// "seeded" reduces to "no hidden non-determinism source" (no wall-clock
// read, no Date/now dependency, no unordered-container iteration). A
// future fault-injection/noise extension (deferred -- Decision 3's own
// Consequences) would need to thread an explicit seed through; none is
// needed for this ticket's scope.
#pragma once

#include <cstdint>

#include "devices/i2c_bus.h"

namespace TestSim {

// Ship-default plant tuning, reused by every scenario in plant_harness.cpp.
// kDefaultTau sits in the bench-characterized ~120-140ms actuation-lag
// range (.clasi/knowledge/actuation-latency-delay-in-plan.md's cited
// figure; see also usecases.md SUC-020's own acceptance criterion).
constexpr float kDefaultTau = 0.13f;           // [s]
constexpr float kDefaultDutyVelMax = 500.0f;   // [mm/s] velocity at |duty|==1.0

class WheelPlant {
 public:
  // dutyVelMax: [mm/s] steady-state wheel speed at |appliedDuty|==1.0.
  // tau: [s] first-order duty->velocity time constant.
  WheelPlant(float dutyVelMax, float tau);

  // Advances the plant's own velocity/position state by dt [s] of virtual
  // time, given the duty ACTUALLY applied on the simulated hardware THIS
  // cycle (Devices::NezhaMotor::appliedDuty() -- never a raw bus byte).
  // Exact discretization of dv/dt = (dutyVelMax*appliedDuty - v) / tau:
  //   alpha = 1 - exp(-dt/tau); v += (dutyVelMax*appliedDuty - v) * alpha.
  // position integrates velocity forward by the same dt (position += v*dt).
  void step(float appliedDuty, float dt);   // [-1,1] [s]

  float velocity() const { return velocity_; }   // [mm/s] signed
  float position() const { return position_; }   // [mm]

  // Schedules the encoder response Devices::NezhaMotor's NEXT
  // requestSample()+tick() pair will consume, from this plant's CURRENT
  // position() -- mirrors devices_motor_harness.cpp's
  // scriptEncoderRequestCollect() two-write-one-read convention exactly
  // (wheelTravelCalib=1.0, fwdSign=+1 convention: raw == position()*10).
  //
  // writeCount selects how many writes to pre-queue for this device
  // address (1 or 2): requestEncoder() always issues exactly one 0x46
  // write; tick()'s OWN mode dispatch (armoredWrite()->writeRawDuty())
  // issues a SECOND write to the SAME address only on the specific cycle a
  // new duty value actually reaches the bus (write-on-change gates every
  // later repeat). Devices::I2CBus's scripted fake uses ONE global
  // write/read FIFO per direction shared across every device address, not
  // one per address (i2c_bus.h's own file header) -- so when this plant is
  // composed alongside a second wheel and/or an OtosPlant sharing the SAME
  // bus (a different device address), an unconsumed "slack" write here
  // would be wrongly popped by that OTHER device's own next write() call,
  // corrupting its address match. A single-wheel, single-address harness
  // (this ticket's own ramp scenario) can safely over-provision (push 2
  // unconditionally, matching scriptEncoderRequestCollect()'s own
  // documented "harmless slack" precedent -- there is no other address for
  // a stray entry to misalign); a MULTI-device harness (this ticket's
  // pivot/determinism scenarios, and every future full-loop composition)
  // must pass the EXACT count instead. Defaults to 1 (request-only) as the
  // safe choice for a multi-device caller; pass 2 explicitly on a leaf's
  // own first tick (the one cycle its write-on-change guard is certain to
  // let a duty write through).
  void scriptEncoderResponse(Devices::I2CBus& bus, uint16_t wireAddr,
                              int writeCount = 1) const;

 private:
  float dutyVelMax_;         // [mm/s]
  float tau_;                // [s]
  float velocity_ = 0.0f;    // [mm/s] signed
  float position_ = 0.0f;    // [mm]
};

}  // namespace TestSim
