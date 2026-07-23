// move_queue.h -- App::MoveQueue: owns the lifecycle of the robot's queued
// and active bounded motions (sprint 116, protocol-set-point issue).
//
// Boundary (sprint.md Architecture Step 3): inside -- the 5-slot array (1
// active + 4 pending), replace/flush/enqueue/ERR_FULL bookkeeping,
// advancing active->next-pending on stop/timeout, owning and driving one
// Motion::StopCondition for whichever Move is active; outside -- deciding
// what a VALID Move looks like (RobotLoop::handleMove()'s job, ticket 006:
// velocity variant present, stop variant present, timeout > 0, the
// config-completeness gate -- every Move this class's enqueue() ever sees
// is already permitted), how a velocity variant becomes wheel duty
// (Drive's job), what "traveled far enough" means numerically
// (Motion::StopCondition + App::Odometry's job). Constructor dependencies:
// Drive&, Odometry&, const Devices::Clock&, const StateEstimator& -- the
// StateEstimator& is a turn-prediction-campaign addition (see tick()'s own
// doc comment below); the other three are the same collaborators the
// deleted App::Deadman (clock only) and App::RobotLoop (drive+odom,
// already) depended on before.
//
// StateEstimator& dependency (turn-prediction campaign): this is a
// DELIBERATE new dependency edge (MoveQueue -> StateEstimator), superseding
// this file's own prior "no new dependency direction" claim -- state_
// estimator.h's own file header always named this as the eventual consumer
// ("consuming whereAmI()/wheelAt() to drive motion... a later, out-of-this-
// sprint trajectory controller"); this IS that later consumer. No cycle:
// StateEstimator depends on nothing in app/ beyond app/telemetry.h, and
// never reads MoveQueue.
//
// StopCondition storage: Motion::StopCondition has no default constructor
// (every baseline is captured at construction -- see stop_condition.h's
// own file header). Rather than a std::optional/placement-new wrapper,
// MoveQueue stores the active Move's StopCondition-construction ARGUMENTS
// (kind, threshold, timeout, and the activation-time now/pathLength/theta
// baseline) as plain fields on its own ActiveMove slot, and reconstructs a
// fresh, byte-identical Motion::StopCondition from them on every tick()
// call. This is behaviorally IDENTICAL to holding a persistent instance --
// StopCondition's constructor is pure precomputation from exactly these
// six values, and tick() itself is const/stateless -- while keeping
// MoveQueue's own storage a plain aggregate of scalars (no heap, no
// optional, no placement-new machinery), matching motion/DESIGN.md's own
// note that "MoveQueue's own construction cadence is out of [StopCondition's]
// boundary... entirely ticket 005's decision."
//
// tick(now, odom): both are the caller's CURRENT readings, passed in
// rather than read from the held Odometry& -- mirrors StopCondition's own
// "never read from a held reference for CURRENT readings" convention
// (stop_condition.h's file header), extended here for a second reason: a
// same-cycle chain-advance activation (the next pending Move taking over
// the instant the active one ends -- SUC-051's seamless hand-off) reuses
// these EXACT (now, odom) readings as the new Move's fresh StopCondition
// baseline, rather than issuing a second clock_.nowMicros()/odom_.
// pathLength() read mid-tick that could disagree with the one the caller
// already took this cycle.
#pragma once

#include <cstdint>

#include "app/drive.h"
#include "app/odometry.h"
#include "app/state_estimator.h"
#include "devices/clock.h"
#include "messages/envelope.h"
#include "motion/stop_condition.h"

namespace App {

// ShaperLimits -- Motion::VelocityShaper's own accel/decel magnitude
// ceilings (decel-into-the-goal campaign, follow-on to
// clasi/issues/angle-stop-overshoot-61-73-percent-on-hardware.md's own
// "Option 1... remains the path to closing that residual further").
// Field-for-field mirror of Config::ShaperBootConfig (config/
// boot_config.h), independently declared for the SAME reason
// App::FusionWeights is independently declared from Config::
// EstimatorBootConfig: config/ may depend only on messages/, never on
// app/ (docs/design/design.md's dependency diagram) -- main.cpp converts
// Config::ShaperBootConfig into this struct at the one composition-root
// place both types are visible. Declared at namespace scope (not nested
// inside MoveQueue), the SAME reason FusionWeights is declared at
// namespace scope rather than nested inside StateEstimator: a nested
// struct's own default member initializers cannot be referenced by a
// default ARGUMENT of the enclosing class's own constructor while that
// enclosing class is still being defined (a real clang/C++ parse
// restriction, not a style preference -- MoveQueue's constructor below
// needs exactly this default).
//
// Disabled-axis sentinel: aMax<=0 OR aDecel<=0 disables LINEAR shaping
// entirely (tick()/activate() stage a Move's raw v_x/v_left/v_right
// unchanged, byte-identical to this class's pre-shaping behavior);
// independently, alphaMax<=0 OR alphaDecel<=0 disables ANGULAR shaping
// (raw omega unchanged). This is the SAME "0 == off, matching this
// class's own pre-feature behavior" contract stopLead already
// established (MoveQueue's own constructor doc comment) -- the
// default-constructed ShaperLimits{} (every field 0) is therefore the
// exact identity/no-op configuration every pre-existing MoveQueue caller
// (every unit-test harness, TestSim::SimHarness) keeps getting without
// passing anything. Real firmware always supplies real positive values
// here (gen_boot_config.py's shaper_config_for_config() REQUIRES all four
// robot-JSON keys, config-as-truth) -- shaping is therefore
// unconditionally ON in production, opt-in only for a sim/test
// composition root that hasn't pushed a real config (mirrors
// FusionWeights{}/stopLead=0's own sim/production boundary precedent,
// sim_harness.h's own comment).
struct ShaperLimits {
  float aMax = 0.0f;        // [mm/s^2] linear accel-ramp ceiling
  float aDecel = 0.0f;      // [mm/s^2] linear decel-taper ceiling
  float alphaMax = 0.0f;    // [rad/s^2] angular accel-ramp ceiling
  float alphaDecel = 0.0f;  // [rad/s^2] angular decel-taper ceiling
};

class MoveQueue {
 public:
  static constexpr int kMaxPending = 4;

  // Result of an enqueue() call. corrId is echoed back unchanged so the
  // caller (RobotLoop::handleMove()) can ack the envelope's corr_id with
  // the returned err in one step (`tlm_.ack(result.corrId,
  // static_cast<uint32_t>(result.err))`) without separately re-threading
  // corr_id itself. err is msg::ErrCode::ERR_NONE (enqueued or activated)
  // or msg::ErrCode::ERR_FULL (rejected, queue provably unchanged -- see
  // enqueue()'s own doc comment).
  struct EnqueueResult {
    uint32_t corrId = 0;
    msg::ErrCode err = msg::ErrCode::ERR_NONE;
  };

  // Reported when a Move ends (StopConditionMet or TimedOut) -- what the
  // caller needs to send the completion ack (against moveId) and, when
  // timedOut is true, set kFlagFaultMoveTimeout on that cycle.
  struct Completion {
    uint32_t moveId = 0;
    bool timedOut = false;
  };

  // tick() reports AT MOST one completion per call -- only one Move is
  // ever active, so at most one can end on a given cycle.
  struct TickResult {
    bool completed = false;
    Completion completion{};  // valid iff completed
  };

  // stopLead -- [ms] initial anticipation lead (see tick()'s own doc
  // comment); defaults to 0 (anticipation OFF, IDENTICAL to this class's
  // pre-turn-prediction-campaign behavior) for a caller that doesn't source
  // one from boot config (e.g. src/sim/sim_harness.h's own documented
  // sim/production boundary, or a unit-test harness that doesn't care).
  // Live-retunable afterward via setStopLead() -- see that method's own
  // doc comment.
  //
  // shaperLimits -- Motion::VelocityShaper's own accel/decel ceilings (see
  // ShaperLimits's own doc comment above); defaults to ShaperLimits{}
  // (every field 0 -- shaping OFF, IDENTICAL to this class's pre-shaping
  // behavior), the SAME "opt-in for sim/test, always-on in production"
  // shape stopLead already established. Live-retunable via
  // setShaperLimits().
  MoveQueue(Drive& drive, Odometry& odom, const Devices::Clock& clock,
            const StateEstimator& stateEstimator, uint32_t stopLead = 0,
            ShaperLimits shaperLimits = ShaperLimits{});

  // Enqueues/replaces `move` (already shape-validated by the caller -- see
  // this file's own header comment).
  //
  // move.replace == true: flushes every pending slot (no completion ack
  // for any of them -- sprint.md Architecture Open Question 2's resolved
  // convention: only an activated-then-ended Move ever gets a completion
  // ack) and preempts the active Move immediately -- `move` itself
  // activates in this SAME call, staging its velocity through Drive and
  // capturing a fresh StopCondition baseline from `clock`/`odom` (the
  // collaborators this class was constructed with) at this exact moment.
  //
  // move.replace == false, queue empty (no active Move): `move` activates
  // immediately, identically to the replace==true activation above (there
  // is nothing to flush or preempt).
  //
  // move.replace == false, a Move is already active: `move` appends behind
  // it. If 4 are already pending, returns ERR_FULL and the call is a
  // complete no-op -- the existing active Move and all 4 pending Moves are
  // untouched, because nothing above this rejection path ever mutates any
  // queue state (the ERR_FULL check runs before any write).
  EnqueueResult enqueue(const msg::Move& move, uint32_t corrId);

  // Per-cycle tick. now/odom are the caller's CURRENT readings (see this
  // file's own header comment for why both are passed in rather than read
  // from the held collaborators). Ticks the active Move's StopCondition;
  // on StopConditionMet or TimedOut, ends the active Move (reported via
  // the returned TickResult) and either activates the next pending Move
  // THIS SAME CALL (seamless hand-off, SUC-051 -- no intervening call that
  // stages a zero/stopped target) or, if the queue is now empty, calls
  // Drive::stop(). A no-op (Continue, TickResult::completed == false) if
  // no Move is active.
  //
  // Anticipation lead (turn-prediction campaign, StateEstimator&
  // consumption): the kind-specific Distance/Angle comparison (never the
  // Time kind or the timeout backstop, both genuine elapsed-wall-clock
  // deadlines) is evaluated against `stateEstimator_.bodyAt(now + stopLead)`
  // instead of the caller's raw CURRENT `odom` reading, whenever stopLead_
  // > 0 AND the estimator's own body peer is valid (has seen at least one
  // update() call) -- falls back to the raw `odom` reading otherwise
  // (stopLead_ == 0, or too early after boot for the estimator to have a
  // basis yet). This closes two measured error sources at once: the
  // one-cycle basis staleness `odom` itself already carries at this call
  // site (robot_loop.cpp's own kPace-block ordering ticks MoveQueue BEFORE
  // stateEstimator_.update() runs each cycle) and the actuation/momentum-
  // tail overshoot a stop condition fired exactly AT threshold-crossing
  // still incurs (turn-prediction-campaign notebook,
  // src/tests/notebooks/turn_prediction.ipynb -- measured ~150-250ms lag,
  // ~18deg overshoot at omega=2rad/s in sim). Distance predicts pathLength
  // forward using the predicted body-frame speed (|v_x, v_y|) held
  // constant across the SAME age the heading prediction uses -- mirrors
  // pathLength()'s own "accumulate |distance| every cycle" contract
  // (odometry.h), extrapolated rather than measured. Activation baselines
  // (activationPathLength_/activationTheta_) are UNCHANGED by this --
  // still captured from the raw `odom` reading at activation time, exactly
  // as before; only the CURRENT-reading side of the comparison anticipates.
  //
  // Velocity shaping (decel-into-the-goal campaign, follow-on to the
  // anticipation lead above): on a Continue outcome (the Move keeps
  // running), this method ALSO re-stages the active Move's commanded
  // velocity through Drive::setTwist()/setWheels() -- Motion::
  // VelocityShaper::next() computes the next tick's speed for whichever
  // axis (linear v_x/v_left/v_right, angular omega) ShaperLimits enables,
  // using the SAME pathLength/theta (possibly anticipation-predicted, see
  // above) this same call already computed for the stop-condition
  // comparison -- never a second, independent prediction. "Remaining" for
  // the taper is threshold-minus-so-far-traveled for a Distance-kind Move
  // (linear axis only), threshold-minus-so-far-turned for an Angle-kind
  // Move (angular axis only), or +infinity for a Kind::Time Move on BOTH
  // axes (accel-limited ramp-up still applies; no taper, since a Time Move
  // ends on elapsed wall-clock time, not position -- there is no
  // "remaining distance" to taper against). A Move's non-primary axis
  // (e.g. a Distance-kind TWIST Move's own omega, if nonzero) is passed
  // through UNSHAPED -- this class only shapes the axis its own stop_kind
  // measures; see move_queue.cpp's own shapeAndStage() comment for the
  // full per-kind breakdown, including WHEELS (v_left/v_right shaped
  // independently, both linear, regardless of stop_kind). A no-op per
  // axis whenever ShaperLimits disables it (ShaperLimits's own doc
  // comment) -- Drive is never re-staged for a disabled axis, so a
  // caller with ShaperLimits{} (the default) sees IDENTICAL behavior to
  // this class before shaping existed (Drive is staged once, at
  // activate(), and never again until the Move ends).
  TickResult tick(uint64_t now, const Odometry& odom);  // [us]

  // setStopLead/stopLead -- the live-tuning entry point (RobotLoop::
  // handleConfig()'s own ESTIMATOR branch, turn-prediction campaign) mirrors
  // StateEstimator::setWeights()'s own shape: a plain in-memory write, no
  // I2C access, no bus transaction. A reboot always reverts to the boot
  // config's own stop_lead_ms bake (Config::EstimatorBootConfig::stopLead)
  // -- this class never persists it, matching EstimatorConfigPatch's own
  // "never persisted" contract for the fusion weights it already carries.
  void setStopLead(uint32_t stopLead) { stopLead_ = stopLead; }  // [ms]
  uint32_t stopLead() const { return stopLead_; }                // [ms]

  // setShaperLimits/shaperLimits -- the live-tuning entry point (RobotLoop::
  // handleConfig()'s own ESTIMATOR branch, decel-into-the-goal campaign),
  // the SAME shape as setStopLead()/stopLead() immediately above: a plain
  // in-memory write, never persisted (a reboot always reverts to the boot
  // config's own Config::ShaperBootConfig bake).
  void setShaperLimits(ShaperLimits limits) { shaperLimits_ = limits; }
  ShaperLimits shaperLimits() const { return shaperLimits_; }

  // Drains every pending slot and ends the active Move (if any) with NO
  // completion ack for any of them -- used by STOP (ticket 006), which
  // acks the STOP command itself via the envelope's own corr_id, not a
  // per-flushed-Move completion ack. Always calls Drive::stop() (STOP's
  // own "zero both motor velocity targets" contract), whether or not a
  // Move was active.
  void flush();

  // The caller's frame_.mode/driving_ derivation (ticket 006).
  bool active() const { return active_.occupied; }

  // --- Test/observability seam (mirrors Telemetry::primaryEmitCount()'s
  // own "measurement/test seam" precedent, telemetry.h) -- not called by
  // RobotLoop; lets a harness assert the queue's exact contents
  // byte-for-byte (SUC-052's own rigor bar: "not just 'still 4 pending'")
  // after an enqueue()/replace()/flush() call, without reaching into
  // private state. ---

  int pendingCount() const { return pendingCount_; }

  // index must be < pendingCount(); 0 is the NEXT Move to activate.
  const msg::Move& pendingAt(int index) const { return pending_[index]; }

  // Valid only when active() is true.
  uint32_t activeMoveId() const { return active_.moveId; }

 private:
  struct ActiveMove {
    bool occupied = false;
    uint32_t moveId = 0;
    Motion::StopCondition::Kind kind = Motion::StopCondition::Kind::Time;
    float threshold = 0.0f;             // [ms]/[mm]/[rad] depending on kind
    float timeout = 0.0f;               // [ms]
    uint64_t activationNow = 0;         // [us]
    float activationPathLength = 0.0f;  // [mm]
    float activationTheta = 0.0f;       // [rad]

    // Cruise-target velocity fields (decel-into-the-goal campaign) --
    // captured once at activation from `move`'s own velocity variant;
    // shapeAndStage() (move_queue.cpp) shapes TOWARD these every tick, the
    // "cruiseSpeed" argument to Motion::VelocityShaper::next(). Kept
    // alongside the stop-condition baseline above rather than re-reading
    // the original msg::Move (this class doesn't keep a copy of the
    // active Move itself, only pending_[] ones -- see this file's own
    // header, "Stages ... onto drive_" activate() comment).
    msg::Move::VelocityKind velocityKind = msg::Move::VelocityKind::NONE;
    float cruiseVX = 0.0f;      // [mm/s] TWIST only
    float cruiseVY = 0.0f;      // [mm/s] TWIST only (always 0 -- see Drive::setTwist()'s own doc comment)
    float cruiseOmega = 0.0f;   // [rad/s] TWIST only
    float cruiseVLeft = 0.0f;   // [mm/s] WHEELS only
    float cruiseVRight = 0.0f;  // [mm/s] WHEELS only
  };

  // Stages `move`'s velocity variant onto drive_ and populates active_ from
  // `move` plus the (now, pathLength, theta) activation baseline -- shared
  // by enqueue()'s two activation paths and tick()'s chain-advance path.
  void activate(const msg::Move& move, uint64_t now, float pathLength, float theta);

  // shapeAndStage -- decel-into-the-goal campaign. Computes this tick's
  // shaped speed for whichever axis ShaperLimits enables (see tick()'s own
  // doc comment) and re-stages it through drive_.setTwist()/setWheels().
  // Called from tick() ONLY on a Continue outcome (see tick()'s own doc
  // comment for why -- a Move that ends this same cycle is about to be
  // superseded by a chain-advance activate() or drive_.stop() regardless).
  // pathLength/theta are the SAME (possibly anticipation-predicted)
  // readings tick() already computed for the stop-condition comparison
  // this cycle -- never re-derived here.
  void shapeAndStage(uint64_t now, float pathLength, float theta);

  Drive& drive_;
  Odometry& odom_;
  const Devices::Clock& clock_;
  const StateEstimator& stateEstimator_;

  ActiveMove active_;
  msg::Move pending_[kMaxPending];
  int pendingCount_ = 0;
  uint32_t stopLead_ = 0;  // [ms] see tick()'s own doc comment

  // Velocity-shaping state (decel-into-the-goal campaign). shaperLimits_ is
  // the live-tunable config (setShaperLimits()); the four shaped* fields
  // are the actual RUNNING per-axis commanded-speed state
  // Motion::VelocityShaper::next()'s own "currentSpeed" argument reads and
  // writes each tick -- deliberately MoveQueue-level, not ActiveMove-level
  // (survives across a chain-advance/replace so a same-axis, same-
  // direction follow-on Move continues its ramp smoothly rather than
  // restarting from 0 -- see ShaperLimits's own doc comment for the
  // "byte-identical when disabled" contract this relies on). Initialized
  // to 0 (a fresh boot's own resting state); each axis is self-resetting
  // in practice, since the decel taper already drives it toward 0 as
  // ANY Move's own `remaining` approaches 0, before that Move's stop
  // condition fires.
  ShaperLimits shaperLimits_;
  float shapedVX_ = 0.0f;      // [mm/s]
  float shapedOmega_ = 0.0f;   // [rad/s]
  float shapedVLeft_ = 0.0f;   // [mm/s]
  float shapedVRight_ = 0.0f;  // [mm/s]
  uint64_t lastShapeNow_ = 0;  // [us] dt baseline for shapeAndStage(), reset at each activate()
};

}  // namespace App
