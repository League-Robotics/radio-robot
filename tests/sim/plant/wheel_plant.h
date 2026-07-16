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

  // Selects the position this plant currently reports to a bus read,
  // applying the freeze/dropout fault knobs below (see each knob's own
  // comment) -- NEVER touches step()'s own duty->velocity->position
  // integration.
  //
  // Formerly packaged as scriptEncoderResponse(Devices::I2CBus&, ...),
  // which pushed the selected position (plus a scripted status) onto the
  // scripted-FIFO Devices::I2CBus fake sprint 108 ticket 001 deleted
  // (I2CBus is now a pure interface with no scriptWrite()/scriptRead()).
  // The fault-knob precedence logic itself is UNCHANGED -- only its
  // packaging moved: TestSim::SimPlant (tests/_infra/sim/sim_plant.cpp) is
  // now the sole caller, and packs the returned position into the wire's
  // own 4-byte LE tenths-of-mm frame itself. This keeps WheelPlant owning
  // only the physics + its own fault-injection state (architecture-
  // update.md Decision 3: "SimPlant owns the protocol, not the physics"),
  // never any bus/wire-format knowledge.
  //
  // NOT const (105-005): the dropout knob's own fractional accumulator and
  // "last reported position" bookkeeping (see below) are mutated on every
  // call -- this is the ONE piece of per-call, non-deterministic-LOOKING
  // (but still fully seeded/deterministic, per this file's own "seeded"
  // doc) state this class carries; step()'s own duty->velocity->position
  // integration remains completely unaffected by any knob here.
  float reportedPosition();   // [mm]

  // --- Fault-injection knobs (ticket 105-005, SUC-022) ---------------------
  // Each knob changes ONLY what reportedPosition() above returns (or, for
  // disconnect, how the caller -- SimPlant -- scripts the transaction
  // status) -- never step()'s own duty->velocity->position integration.
  // Three orthogonal toggles a scenario can flip mid-run.

  // Motor disconnect: SimPlant checks this directly (not through
  // reportedPosition()) and scripts a NAK status for the motor's wire
  // transactions instead of calling reportedPosition() at all --
  // Devices::NezhaMotor::connected() is recomputed fresh every
  // collectEncoder() call (nezha_motor.cpp -- never latched), so clearing
  // this knob recovers connected() to true on the very next cycle, no
  // separate "reconnect" step needed.
  void setDisconnected(bool disconnected) { disconnected_ = disconnected; }
  bool disconnected() const { return disconnected_; }

  // Encoder wedge / stuck value: while true, reportedPosition() keeps
  // returning the position CAPTURED at the moment this knob went true
  // (frozenPosition_) -- never step()'s own live, still-advancing
  // position_. The plant's internal velocity/position integration keeps
  // running underneath exactly as if nothing were wrong (this plant "knows"
  // it should be moving; only the SCRIPTED reading sticks) -- the exact
  // boundary-latch flavor Devices::MotorArmor::updateWedgeDetector() exists
  // to catch (kWedgeThreshold consecutive identical position() reads).
  // Clearing the knob resumes reporting the plant's live position_, which
  // has kept advancing the whole time -- the reported value jumps forward
  // to catch up, matching a real freed-up encoder's own behavior (and
  // exercising updateWedgeDetector()'s own clear-on-first-changed-read
  // semantics, robot_loop.cpp's live kFaultWedgeLatch bit right along with
  // it -- see robot_loop.cpp's own `tlm_.setFault(kFaultWedgeLatch, ...)`
  // call, re-evaluated fresh every cycle from wedged(), never a one-shot
  // latch at the wire level).
  void freezePosition(bool freeze);
  bool positionFrozen() const { return freezePosition_; }

  // Encoder dropout: `fraction` (0..1) of reportedPosition() calls return
  // the LAST reported position again (stale-not-fresh) instead of a fresh
  // sample off position_ -- deterministic (no RNG anywhere in this plant,
  // per this file's own "seeded" doc): a fixed fractional accumulator
  // advances by `fraction` every call and fires a hold whenever it crosses
  // 1.0, so e.g. a 0.25 dropout rate holds exactly every 4th call, on the
  // dot, run after run. 0.0 (the default) disables the knob -- every call
  // reports position_ fresh, matching the pre-105-005 behavior exactly.
  // Resets the accumulator's phase on every call (a rate change
  // mid-scenario does not inherit a stale phase from the previous rate).
  void setDropoutRate(float fraction);   // [0,1]
  float dropoutRate() const { return dropoutRate_; }

 private:
  float dutyVelMax_;         // [mm/s]
  float tau_;                // [s]
  float velocity_ = 0.0f;    // [mm/s] signed
  float position_ = 0.0f;    // [mm]

  // ---- Fault knob state (105-005) ----
  bool disconnected_ = false;
  bool freezePosition_ = false;
  float frozenPosition_ = 0.0f;      // [mm] captured on the freeze knob's rising edge
  float dropoutRate_ = 0.0f;         // [0,1]
  float dropoutAccum_ = 0.0f;        // fractional accumulator, see setDropoutRate()
  float lastReportedPosition_ = 0.0f;  // [mm] the last value reportedPosition() actually returned
};

}  // namespace TestSim
