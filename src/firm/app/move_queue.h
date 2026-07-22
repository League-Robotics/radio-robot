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
// Drive&, Odometry&, const Devices::Clock& -- the same three collaborators
// the deleted App::Deadman (clock only) and App::RobotLoop (drive+odom,
// already) depended on today; no new dependency direction. Fan-out stays
// at exactly these 3 injected collaborators plus the owned (not injected)
// Motion::StopCondition.
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

namespace App {

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

  MoveQueue(Drive& drive, Odometry& odom, const Devices::Clock& clock);

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
  TickResult tick(uint64_t now, const Odometry& odom);  // [us]

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
  };

  // Stages `move`'s velocity variant onto drive_ and populates active_ from
  // `move` plus the (now, pathLength, theta) activation baseline -- shared
  // by enqueue()'s two activation paths and tick()'s chain-advance path.
  void activate(const msg::Move& move, uint64_t now, float pathLength, float theta);

  Drive& drive_;
  Odometry& odom_;
  const Devices::Clock& clock_;

  ActiveMove active_;
  msg::Move pending_[kMaxPending];
  int pendingCount_ = 0;
};

}  // namespace App
