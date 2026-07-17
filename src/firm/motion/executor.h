// executor.h -- Motion::Executor: sequences Motion::Cmd arc commands into
// continuous motion. Owns the fixed ring queue (depth 8), the state
// machine (IDLE/RUNNING/RAMP_TO_REST/STOPPING), and per-command completion
// events; calls into Motion::JerkTrajectory for the actual solve, never
// does the solve math itself (jerk_trajectory.h's own boundary).
//
// 109-003 scope -- TIMED mode + replace only. This is the sprint's own
// staged scope (sprint.md's ticket table): TIMED (`Move.time > 0`) is the
// teleop primitive and is implemented end to end here (both linear and
// rotational channels driven directly and independently by
// `Cmd::vMax`/`Cmd::omega` -- unlike DISTANCE mode's dominant/slaved-
// channel coupling, TIMED mode has no heading reference to slave against,
// so both channels just each track their own commanded velocity). DISTANCE
// mode (`Move.time <= 0`, non-degenerate) is DECLARED on the wire
// (envelope.proto's Move message) but this ticket does not implement
// dominant-channel arc planning or the heading PD cascade for it --
// enqueue() returns EnqueueOutcome::kUnimplemented for a DISTANCE-mode
// Move rather than half-implement a mode ticket 005 replaces wholesale
// (dead-time re-derivation, heading reference, dwell completion all land
// together there). A degenerate Move (Cmd::isDegenerate()) is classified
// BEFORE the TIMED/DISTANCE branch and never reaches either.
//
// -- Deadline-driven RAMP_TO_REST (TIMED mode's "ramp down to finish at
// the deadline") --
// A TIMED command's own `time` is a total-duration deadline, ramps
// included (sprint.md's own stakeholder-decision note). Rather than
// pre-computing an exact three-segment (ramp-up/hold/ramp-down) plan up
// front, this executor treats "ramp to rest at the deadline" as ONE MORE
// reason to enter the SAME RAMP_TO_REST state the general state machine
// already has for "queue ran empty at speed" (sprint.md's own state-
// machine section) -- every cycle, `tick()` compares the command's
// remaining time against an ANALYTIC estimate of how long this channel's
// own configured decel would take from its current sampled velocity
// (`estimateStopDuration()` below); once remaining time is at or below
// that estimate, both channels get a fresh `solveToVelocity(0, ...)`
// request and the state flips to RAMP_TO_REST. The estimate is a v1
// approximation (trapezoidal decel time `|v|/aDecel`, plus one jerk-ramp
// term `aDecel/jerk` for the S-curve case) -- it decides WHEN to trigger
// the real (exact, jerk-limited) decel solve, not what shape that decel
// takes; Ruckig's own solve is what actually executes it. No acceptance
// criterion in this ticket's testing plan asserts exact-deadline landing
// precision (that is a natural follow-on refinement, not a defect).
//
// -- Solve budget --
// `plan()` performs AT MOST ONE JerkTrajectory solve per call (the
// `kPace`-block budget, per src/firm/DESIGN.md Sec 3 and sprint.md's own
// cycle-placement table) -- a fresh command that needs BOTH channels
// solved (the common case: TIMED commands almost always carry a nonzero
// v_max and/or omega) takes two `plan()` calls, i.e. ~2 loop cycles
// (~80ms), to become fully planned -- explicitly called out as acceptable
// in sprint.md ("a fresh command is ready ~2 cycles/80ms after enqueue").
// `tick()` never solves -- it is sample-only (`JerkTrajectory::sample()`),
// matching `App::Pilot::tick()`'s own motor-settle-block placement.
//
// -- Completion events --
// Ride Telemetry's existing depth-3 ack ring (telemetry.proto's AckStatus
// DONE/TRIVIAL/SUPERSEDED/FLUSHED/TIMEOUT/SOLVE_FAIL additions) rather
// than the orphaned `messages/event.h` -- see telemetry.proto's own doc
// comment for the full resolution of sprint.md's Open Question 3. This
// class holds its own small internal FIFO (popEvent()) so `motion/` never
// reaches into `app/`'s Telemetry directly (devices/app-layer boundary,
// src/firm/DESIGN.md Sec 2 dependency diagram) -- App::Pilot/RobotLoop
// drains it into the wire.
#pragma once

#include <cstdint>

#include "messages/planner.h"
#include "motion/cmd.h"
#include "motion/jerk_trajectory.h"

namespace Motion {

enum class State : uint8_t { kIdle, kRunning, kRampToRest, kStopping };

enum class CompletionStatus : uint8_t {
  kDone,
  kTrivial,
  kSuperseded,
  kFlushed,
  kTimeout,
  kSolveFail,
};

struct CompletionEvent {
  uint32_t id = 0;
  CompletionStatus status = CompletionStatus::kDone;
};

// EnqueueOutcome -- enqueue()'s own synchronous return value. This is
// deliberately NOT the same channel as CompletionEvent/popEvent(): the
// enqueue outcome answers "was this Move admitted" (acked against the
// CommandEnvelope's own corr_id, matching TWIST/CONFIG/STOP's existing
// convention) and is known immediately; a completion event answers "what
// happened to a PREVIOUSLY admitted command" (acked against that command's
// own Move.id) and can arrive many cycles later.
enum class EnqueueOutcome : uint8_t {
  kAccepted,      // activated immediately or appended to the ring
  kReplaced,      // replaced the ring tail or retargeted the active command
  kFull,          // ring already at kQueueDepth; plan untouched
  kTrivial,       // degenerate Move -- never queued
  kUnimplemented  // DISTANCE mode -- not implemented this ticket (005)
};

constexpr uint8_t kQueueDepth = 8;
constexpr uint8_t kEventRingDepth = 8;

class Executor {
 public:
  struct Twist {
    float v = 0.0f;      // [mm/s]
    float omega = 0.0f;  // [rad/s]
  };

  // configure -- stores both channels' own limits (forwarded to the two
  // owned JerkTrajectory instances' configure()) plus the decel/jerk pair
  // this class's own estimateStopDuration() scheduling heuristic needs.
  // Must be called before the first enqueue().
  void configure(const msg::PlannerConfig& config);

  // enqueue -- classify and admit one Cmd. See this class's own doc
  // comment for the degenerate/DISTANCE/TIMED/replace decision tree.
  EnqueueOutcome enqueue(const Cmd& cmd);

  // flush -- TWIST/STOP preemption (App::Pilot::flush()). Empties the ring
  // and clears any active command, pushing a kFlushed completion event for
  // each, and returns to kIdle. Does not touch Drive itself -- the caller
  // owns whatever twist Drive ends up staged with afterward.
  void flush();

  // plan -- at most one JerkTrajectory solve this call. See this class's
  // own "Solve budget" doc comment.
  void plan();

  // tick -- sample-only: advances the active command's own elapsed time by
  // dtMs, samples both channels, evaluates the RAMP_TO_REST deadline
  // trigger and RAMP_TO_REST completion, and returns the twist this
  // cycle's Drive::setTwist() call should stage ({0,0} while kIdle). Never
  // solves, never touches the bus.
  Twist tick(uint32_t dtMs);  // [ms]

  // popEvent -- drains one pending completion event, oldest first. Returns
  // false (out untouched) when none pending.
  bool popEvent(CompletionEvent* out);

  uint8_t queueDepth() const { return queueCount_; }
  uint32_t activeId() const { return active_.id; }
  State state() const { return state_; }

 private:
  // activate -- makes cmd the active command. retarget=false is a fresh
  // start-from-rest activation (JerkTrajectory::reset() on both channels);
  // retarget=true is a replace-while-active in-place retarget (channels
  // keep their own remembered last-sample seed -- see jerk_trajectory.h's
  // seeding contract -- so the new target is approached smoothly, never as
  // an instantaneous step). Either way, requests fresh solves for both
  // channels (serviced by the next one or two plan() calls).
  void activate(const Cmd& cmd, bool retarget);

  // activateNextOrIdle -- pops the ring's head (if any) and activates it
  // fresh-from-rest; otherwise clears the active command and returns to
  // kIdle. Called when the active command reaches its own DONE criterion.
  void activateNextOrIdle();

  void pushEvent(uint32_t id, CompletionStatus status);

  Cmd active_;
  bool activeValid_ = false;
  uint32_t activeElapsedMs_ = 0;  // [ms] since this active command's own activate()

  Cmd ring_[kQueueDepth]{};
  uint8_t queueCount_ = 0;

  State state_ = State::kIdle;

  JerkTrajectory linear_;
  JerkTrajectory rotational_;

  bool needLinearSolve_ = false;
  bool needRotationalSolve_ = false;
  float pendingLinearTarget_ = 0.0f;      // [mm/s]
  float pendingRotationalTarget_ = 0.0f;  // [rad/s]

  // Elapsed time since EACH channel's own last successful solve -- NOT
  // since activation. JerkTrajectory::sample()'s own contract ("elapsed
  // time since it was solved", jerk_trajectory.h) means the two channels
  // generally need DIFFERENT elapsed values: they are solved on different
  // plan() calls (at most one solve per cycle), and a mid-flight replace
  // re-solves from a fresh t=0 without resetting the OTHER channel's own
  // clock. Reset to 0 on that channel's own successful solve (plan()); NOT
  // the same thing as activeElapsedMs_ above (which tracks time since
  // ACTIVATION, for the TIMED deadline comparison).
  float linearElapsedS_ = 0.0f;      // [s]
  float rotationalElapsedS_ = 0.0f;  // [s]

  // Scheduling-only copies of PlannerConfig's own decel/jerk limits (the
  // estimateStopDuration() heuristic's inputs) -- JerkTrajectory keeps its
  // own copies privately for the real solve; this class needs its own to
  // decide WHEN to ask for one, per this file's own doc comment.
  float aDecelLinear_ = 0.0f;      // [mm/s^2]
  float jerkLinear_ = 0.0f;        // [mm/s^3]
  float aDecelRotational_ = 0.0f;  // [rad/s^2]
  float jerkRotational_ = 0.0f;    // [rad/s^3]
  float linearCeiling_ = 0.0f;     // [mm/s]
  float rotationalCeiling_ = 0.0f; // [rad/s]

  CompletionEvent events_[kEventRingDepth]{};
  uint8_t eventCount_ = 0;
};

}  // namespace Motion
