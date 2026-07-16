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

// Rest-dither tuning (108-011, SUC-042). OPT-IN, default OFF (see
// setEncoderJitter() below): the very first cut of this ticket applied the
// dither unconditionally to EVERY WheelPlant, which broke every C++
// scenario test that asserts an exact, byte-stable stopped-wheel
// reportedPosition() (e.g. tests/sim/system/test_scripted_twist_demo.py,
// straight_twist, fault_knobs, plant_harness) -- WheelPlant's own "seeded,
// deterministic, run-A==run-B" contract (this file's header comment) is a
// documented default those tests correctly rely on. A real encoder never
// reports two consecutive byte-identical readings while sitting still -- it
// jitters by roughly a count. This plant's nominal (no-fault-knob, jitter
// OFF) reportedPosition() reports the exact same quantized position_ every
// call while at rest -- realistic-LOOKING but wrong ONLY for the
// hardware-realistic ctypes/tour path, which is why jitter is enabled
// there (tests/_infra/sim/sim_ctypes.cpp's sim_create(), see that file) and
// left off everywhere else. With jitter OFF, it starves
// Devices::MotorArmor::updateWedgeDetector() (kWedgeThreshold=10 consecutive
// identical reads) of the natural jitter that keeps a real, healthy encoder
// from ever latching the wedge fault while idle -- see
// clasi/issues/sim-mode-tour-1-fault-baseline-exclusion-mismatch.md. Below
// kRestVelocityThreshold (sub-LSB-per-cycle regime -- the wheel is not
// covering a full kDitherLsb of ground within one tick anyway) the reported
// position alternates by one wire LSB around the true position_, seeded by a
// per-instance phase bit (no RNG -- see this file's own "seeded" doc).
//
// kDitherPeriod (added after the first cut of this ticket flushed out a
// regression -- see wheel_plant.cpp's reportedPosition() and the ticket's
// own completion notes): the FIRST implementation flipped the dither's sign
// on EVERY rest-branch call (kDitherPeriod effectively 1). That satisfied
// the wedge detector (a change every single read, nowhere near
// kWedgeThreshold=10) but, driven through the REAL Devices::NezhaMotor
// velocity PID this harness wires up (tests/_infra/sim/sim_harness.h's
// makeMotorConfig(): velFiltAlpha=1.0, i.e. NO smoothing, plus a raw
// proportional term that is NEVER zeroed by MotorConfig.velDeadband --
// Devices::MotorVelocityPid::compute() only freezes the INTEGRAL in the
// deadband, never the kp*err term itself, source/devices/velocity_pid.cpp),
// produced a REAL, sustained ~4mm/s phantom velocity reading every single
// tick -- and App::Drive::tick() (source/app/drive.cpp) calls
// left_/right_.setVelocity(0) unconditionally every cycle from boot, so
// Mode::Active's PID chases that phantom reading continuously, even with no
// twist ever commanded. Once anything (e.g. a test's write hook) stops
// FURTHER duty writes from landing, whatever small corrective duty was
// in-flight at that instant is stuck forever, and the plant keeps
// integrating it -- see tests/testgui/test_sim_loop.py's
// test_write_hook_can_swallow_a_command, which caught this as sustained
// several-mm/s drift over less than a second.
//
// Fix: flip the dither's sign only once every kDitherPeriod calls (held
// steady the calls in between), not every call. A held value between flips
// means most rest-branch reads are BYTE-IDENTICAL to the last "fresh"
// sample Devices::NezhaMotor's own freshness gate anchored on
// (nezha_motor.cpp's `raw != lastFreshRawEnc_` check) -- exactly the real-
// hardware-realistic case that gate exists for -- so filteredVelocity_ is
// simply HELD (not recomputed, let alone driven to a new nonzero value) on
// every one of those calls, and the PID's proportional term sees 0 most of
// the time rather than a continuous alternating nonzero signal.
//
// kDitherPeriod=3 keeps the max byte-identical run at 3 -- comfortably
// under kWedgeThreshold=10, with a much wider margin than strictly
// required -- and was the smallest period, of {1..8} checked empirically
// against both regression harnesses, that satisfies BOTH: (a)
// test_write_hook_can_swallow_a_command's <1mm/~0.7s no-motion invariant
// (every write swallowed) and (b) straight_twist_harness's <8deg heading
// tolerance for a held 150mm/s straight run. Larger periods (4, 5, 8 were
// all tried) do NOT monotonically improve either check -- e.g. period=8
// passes the write-swallow test (worst-case phantom velocity per flip is
// smaller) but FAILS straight-twist (heading drifts to >20deg), because a
// longer hold means a LARGER one-time positional bias can accumulate
// between the two wheels' independent dither phases exactly during the
// brief below-kRestVelocityThreshold spin-up window at the start of a
// twist, before the wheels reach cruising speed -- a longer hold does not
// uniformly shrink risk, it just relocates it. Given that non-monotonic,
// aliasing-like sensitivity to the exact period (a product of this
// harness's specific tick cadence and PID gains, not a general law), the
// period was chosen by exhaustive empirical check across the acceptance
// suite rather than derived from a closed-form bound -- see the ticket's
// own completion notes for the raw pass/fail table across periods 1-8.
constexpr float kRestVelocityThreshold = 1.0f;  // [mm/s]
constexpr float kDitherLsb = 0.1f;              // [mm] one wire tenths-of-mm count
constexpr int kDitherPeriod = 3;                // [reportedPosition() calls]

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

  // Rest-encoder jitter (108-011, SUC-042, see this file's "Rest-dither
  // tuning" comment above). Default OFF -- reportedPosition() then returns
  // exactly position_ at rest, matching every C++ scenario test's
  // deterministic-stopped-wheel assumption. ON is opt-in per instance; the
  // ctypes/hardware-realistic path (tests/_infra/sim/sim_ctypes.cpp's
  // sim_create()) is the one caller that turns it on.
  void setEncoderJitter(bool enabled) { encoderJitter_ = enabled; }
  bool encoderJitter() const { return encoderJitter_; }

  // Plant teleport (sim command-surface fix, host TestGUI Sim "reset to
  // origin"/SI support): re-baselines this wheel to `pos` -- position_,
  // lastReportedPosition_, and frozenPosition_ all snap to it, velocity_
  // zeros (a teleported robot is at rest), and the dropout/dither
  // accumulators clear so the next reportedPosition() call starts a fresh
  // phase rather than inheriting one from before the jump. Does NOT touch
  // the disconnected_/freezePosition_/dropoutRate_/encoderJitter_ fault
  // knobs themselves -- only the physics state a fault knob reads from.
  void resetPosition(float pos = 0.0f) {  // [mm]
    position_ = pos;
    velocity_ = 0.0f;
    lastReportedPosition_ = pos;
    frozenPosition_ = pos;
    dropoutAccum_ = 0.0f;
    ditherCounter_ = 0;
  }

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
  bool encoderJitter_ = false;         // 108-011: opt-in, default OFF -- see setEncoderJitter()

  // Rest-dither phase (108-011): flips every kDitherPeriod dithered reads,
  // own per-instance state so left/right wheels dither independently. See
  // kRestVelocityThreshold/kDitherLsb/kDitherPeriod above.
  bool ditherPhase_ = false;
  int ditherCounter_ = 0;  // [reportedPosition() calls since the last flip]
};

}  // namespace TestSim
