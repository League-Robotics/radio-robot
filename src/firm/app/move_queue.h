// move_queue.h -- App::MoveQueue: owns the lifecycle of the robot's queued
// and active bounded motions (sprint 116, protocol-set-point issue).
//
// Boundary (sprint.md Architecture Step 3): inside -- the 5-slot array (1
// active + 4 pending), replace/flush/enqueue/ERR_FULL bookkeeping,
// advancing active->next-pending on stop/timeout, owning and driving one
// Motion::StopCondition for whichever Move is active, and (118 ticket 004)
// the land-at-zero completion predicate (see tick()'s own doc comment
// below); outside -- deciding what a VALID Move looks like
// (RobotLoop::handleMove()'s job, ticket 006: velocity variant present,
// stop variant present, timeout > 0, the config-completeness gate -- every
// Move this class's enqueue() ever sees is already permitted), how a
// velocity variant becomes wheel duty (Drive's job), what "traveled far
// enough" means numerically (Motion::StopCondition + App::Odometry's job).
// Constructor dependencies: Drive&, Odometry&, const Devices::Clock& --
// the same collaborators the deleted App::Deadman (clock only) and
// App::RobotLoop (drive+odom, already) depended on before.
//
// StateEstimator& dependency -- REMOVED (118 ticket 004, land-at-zero
// completion; see land-at-zero-completion-delete-stop-lead.md). The
// turn-prediction campaign (117) had added a `const StateEstimator&` here
// to evaluate the stop condition against a predicted-forward heading/
// pathLength (`stateEstimator_.bodyAt()` evaluated at now plus a fixed
// millisecond lead) instead of the caller's raw current `odom` reading --
// a hand-tuned time-lead guess with a four-retune history and no stable
// value (see that issue's own Description). That anticipation block, the
// lead constant that drove it, and the
// StateEstimator& dependency it needed are all deleted: the taper
// (Motion::VelocityShaper) already brings the commanded speed to ~zero at
// the goal, so completion is now decided from `remaining` and the taper's
// own commanded speed, both already local to this class -- there is no
// tail left to predict, and no reason to reach into a peer module for one.
// `App::StateEstimator` itself is QUARANTINED, not deleted (module,
// `update()`, and its own tests all remain -- kept as the planned consumer
// for future fake-OTOS/fusion bench work); this class was its only
// firmware production consumer.
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
#include "devices/clock.h"
#include "messages/envelope.h"
#include "motion/stop_condition.h"
#include "motion/velocity_shaper.h"

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
// jMax/yawJerkMax (jerk-limited S-curve stage, 2026-07-22 stakeholder
// correction on top of this struct's own first accel-limited pass): the
// jerk magnitude ceilings Motion::VelocityShaper's own accel-slew clamp
// uses (velocity_shaper.h's own file header has the full algorithm).
// `j_max`/`yaw_jerk_max` already existed as REQUIRED, unread `control.*`
// robot-JSON keys since sprint 114 (098-001) -- this campaign is their
// first consumer, same "read again" story as aMax/aDecel above.
//
// Disabled-axis sentinel: aMax<=0 OR aDecel<=0 OR jMax<=0 disables LINEAR
// shaping entirely (tick()/activate() stage a Move's raw v_x/v_left/
// v_right unchanged, byte-identical to this class's pre-shaping
// behavior); independently, alphaMax<=0 OR alphaDecel<=0 OR
// yawJerkMax<=0 disables ANGULAR shaping (raw omega unchanged) -- and, per
// ticket 004, also disables the land-at-zero completion path on that axis
// (see tick()'s own doc comment): with no taper, the commanded speed never
// bleeds toward zero, so the threshold/timeout backstop stays the ONLY
// completion path, exactly as before this feature existed. The default-
// constructed ShaperLimits{} (every field 0) is therefore the exact
// identity/no-op configuration every pre-existing MoveQueue caller (every
// unit-test harness, TestSim::SimHarness) keeps getting without passing
// anything. Real firmware always supplies real positive values here
// (gen_boot_config.py's shaper_config_for_config() REQUIRES all six
// robot-JSON keys, config-as-truth) -- shaping is therefore
// unconditionally ON in production, opt-in only for a sim/test
// composition root that hasn't pushed a real config (mirrors
// FusionWeights{}'s own sim/production boundary precedent, sim_harness.h's
// own comment).
struct ShaperLimits {
  float aMax = 0.0f;         // [mm/s^2] linear accel-ramp ceiling
  float aDecel = 0.0f;       // [mm/s^2] linear decel-taper ceiling
  float alphaMax = 0.0f;     // [rad/s^2] angular accel-ramp ceiling
  float alphaDecel = 0.0f;   // [rad/s^2] angular decel-taper ceiling
  float jMax = 0.0f;         // [mm/s^3] linear jerk ceiling
  float yawJerkMax = 0.0f;   // [rad/s^3] angular jerk ceiling
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

  // shaperLimits -- Motion::VelocityShaper's own accel/decel ceilings (see
  // ShaperLimits's own doc comment above); defaults to ShaperLimits{}
  // (every field 0 -- shaping OFF, IDENTICAL to this class's pre-shaping
  // behavior) for a caller that doesn't source one from boot config (e.g.
  // src/sim/sim_harness.h's own documented sim/production boundary, or a
  // unit-test harness that doesn't care). Live-retunable via
  // setShaperLimits().
  MoveQueue(Drive& drive, Odometry& odom, const Devices::Clock& clock,
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
  // from the held collaborators). Ticks the active Move's StopCondition
  // (the always-armed threshold/timeout backstop -- unchanged, first to
  // fire wins); on StopConditionMet or TimedOut, ends the active Move
  // (reported via the returned TickResult) and either activates the next
  // pending Move THIS SAME CALL (seamless hand-off, SUC-051 -- no
  // intervening call that stages a zero/stopped target) or, if the queue
  // is now empty, calls Drive::stop(). A no-op (Continue, TickResult::
  // completed == false) if no Move is active.
  //
  // Land-at-zero completion (118 ticket 004,
  // land-at-zero-completion-delete-stop-lead.md -- supersedes the deleted
  // anticipation-lead mechanism this doc comment used to describe here,
  // see this file's own header). On a cycle the backstop above does NOT
  // already end the Move (Continue), an ADDITIONAL completion path is
  // checked for a TWIST Move whose stop_kind is Angle (omega axis) or
  // Distance (v_x axis) AND whose matching ShaperLimits axis is enabled
  // (angular/linear respectively -- see ShaperLimits's own doc comment):
  // the Move is declared done once `remaining` (threshold minus
  // so-far-traveled/turned, the SAME quantity shapeAndStage() computes for
  // the taper) has shrunk into the taper's OWN braking envelope for its
  // most-recently-commanded speed on that axis
  // (Motion::VelocityShaper::commandedSpeed()) -- a dynamic, self-
  // referential `remaining <= (commandedSpeed^2 / (2*decelCeiling)) *
  // marginFactor` check, not a static epsilon on either quantity alone; see
  // move_queue.cpp's own landAtZero() and its anonymous-namespace comment
  // for the full derivation, including why `marginFactor` takes one of two
  // values depending on pendingCount() (chain-advance about to take over
  // vs. this Move draining the queue to a genuine Drive::stop()). The
  // taper is DESIGNED to bring the commanded speed to ~zero as `remaining`
  // reaches zero (velocity_shaper.cpp's own `sqrt(2*aDecel*remaining)`
  // ceiling), so this is an emergent "let it finish" completion, not a
  // second tuned guess: once the taper has entered its own braking
  // envelope, there is nothing left to gain by waiting for the raw
  // threshold's own exact `remaining <= 0` crossing (which the output-
  // deadband boost can otherwise stall short of indefinitely, since a
  // sub-floor nonzero command gets boosted back UP to the floor rather
  // than allowed to taper all the way to true zero -- nezha_motor.cpp's
  // own writeShapedDuty()). WHEELS Moves and Kind::Time never qualify (no
  // matching stop_kind/axis pairing exists for either -- see
  // shapeAndStage()'s own per-kind breakdown). With the matching
  // ShaperLimits axis DISABLED (the default), this path can never fire --
  // the taper never exists, the commanded speed never bleeds toward zero,
  // and the threshold/timeout backstop is the ONLY completion path,
  // byte-identical to this class's behavior before this feature existed.
  //
  // Velocity shaping (decel-into-the-goal campaign): on a Continue outcome
  // (the Move keeps running -- neither the backstop nor land-at-zero ended
  // it this cycle), this method ALSO re-stages the active Move's commanded
  // velocity through Drive::setTwist()/setWheels() -- Motion::
  // VelocityShaper::next() computes the next tick's speed for whichever
  // axis (linear v_x/v_left/v_right, angular omega) ShaperLimits enables,
  // using the SAME pathLength/theta this same call already computed for
  // the stop-condition comparison above -- never a second, independent
  // prediction. "Remaining" for the taper is threshold-minus-so-far-
  // traveled for a Distance-kind Move (linear axis only), threshold-minus-
  // so-far-turned for an Angle-kind Move (angular axis only), or
  // +infinity for a Kind::Time Move on BOTH axes (accel-limited ramp-up
  // still applies; no taper, since a Time Move ends on elapsed wall-clock
  // time, not position -- there is no "remaining distance" to taper
  // against). A Move's non-primary axis (e.g. a Distance-kind TWIST Move's
  // own omega, if nonzero) is passed through UNSHAPED -- this class only
  // shapes the axis its own stop_kind measures; see move_queue.cpp's own
  // shapeAndStage() comment for the full per-kind breakdown, including
  // WHEELS (v_left/v_right shaped independently, both linear, regardless
  // of stop_kind). A no-op per axis whenever ShaperLimits disables it
  // (ShaperLimits's own doc comment) -- Drive is never re-staged for a
  // disabled axis, so a caller with ShaperLimits{} (the default) sees
  // IDENTICAL behavior to this class before shaping existed (Drive is
  // staged once, at activate(), and never again until the Move ends).
  TickResult tick(uint64_t now, const Odometry& odom);  // [us]

  // setShaperLimits/shaperLimits -- the live-tuning entry point (RobotLoop::
  // handleConfig()'s own ESTIMATOR branch, decel-into-the-goal campaign): a
  // plain in-memory write, no I2C access, no bus transaction, never
  // persisted (a reboot always reverts to the boot config's own
  // Config::ShaperBootConfig bake).
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

  // shapingDisabled -- true iff BOTH axes' ShaperLimits are disabled (the
  // SAME "aMax<=0 OR aDecel<=0 OR jMax<=0" / "alphaMax<=0 OR alphaDecel<=0
  // OR yawJerkMax<=0" gate shapeAndStage()'s own early-return condition
  // uses, move_queue.cpp -- ShaperLimits's own doc comment above has the
  // full disabled-axis sentinel rationale). Independent of active() --
  // this reads shaperLimits_ only, a config-level state that exists
  // whether or not a Move happens to be active right now. RobotLoop ANDs
  // this with active() to drive the new flags bit 16
  // (kFlagFaultShapingDisabled, 119 ticket 001,
  // kill-the-silent-off-shaping-config-boundary.md) -- see that
  // constant's own doc comment in telemetry.h.
  bool shapingDisabled() const;

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
  // pathLength/theta are the SAME readings tick() already computed for the
  // stop-condition comparison this cycle -- never re-derived here.
  void shapeAndStage(uint64_t now, float pathLength, float theta);

  // landAtZero -- the land-at-zero completion predicate (118 ticket 004,
  // see tick()'s own doc comment for the full contract). pathLength/theta
  // are the SAME CURRENT readings tick() already has this cycle -- never
  // re-derived here. dt is this Move's own elapsed time since its last
  // shaped tick (118 ticket 003 resolution -- the per-cycle discretization
  // term, see move_queue.cpp's own anonymous-namespace comment). Pure
  // query: reads shaperLimits_/shaperVX_/shaperOmega_/active_/
  // pendingCount_, mutates nothing.
  bool landAtZero(float pathLength, float theta, float dt) const;

  Drive& drive_;
  Odometry& odom_;
  const Devices::Clock& clock_;

  ActiveMove active_;
  msg::Move pending_[kMaxPending];
  int pendingCount_ = 0;

  // Velocity-shaping state (decel-into-the-goal campaign). shaperLimits_ is
  // the live-tunable config (setShaperLimits()); the four shaper* members
  // are the actual RUNNING per-axis Motion::VelocityShaper instances --
  // each one OWNS its own (commandedSpeed, commandedAccel) state pair
  // (jerk-limited S-curve stage: VelocityShaper became stateful, see its
  // own file header) -- deliberately MoveQueue-level, not ActiveMove-level
  // (survives across a chain-advance/replace so a same-axis, same-
  // direction follow-on Move continues its ramp smoothly rather than
  // restarting from 0 -- see ShaperLimits's own doc comment for the
  // "byte-identical when disabled" contract this relies on). Default-
  // constructed (both state fields 0 -- a fresh boot's own resting state);
  // each axis is self-resetting in practice, since the decel taper already
  // drives it toward 0 as ANY Move's own `remaining` approaches 0, before
  // that Move's stop condition fires (and tick()'s own empty-queue-drain/
  // flush() paths call .reset() explicitly regardless -- see their own
  // call sites in move_queue.cpp).
  ShaperLimits shaperLimits_;
  Motion::VelocityShaper shaperVX_;      // [mm/s] / [mm/s^2]
  Motion::VelocityShaper shaperOmega_;   // [rad/s] / [rad/s^2]
  Motion::VelocityShaper shaperVLeft_;   // [mm/s] / [mm/s^2]
  Motion::VelocityShaper shaperVRight_;  // [mm/s] / [mm/s^2]
  uint64_t lastShapeNow_ = 0;  // [us] dt baseline for shapeAndStage(), reset at each activate()
};

}  // namespace App
