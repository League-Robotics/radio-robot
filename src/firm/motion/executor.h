// executor.h -- Motion::Executor: sequences Motion::Cmd arc commands into
// continuous motion. Owns the fixed ring queue (depth 8), the state
// machine (IDLE/RUNNING/RAMP_TO_REST/STOPPING), and per-command completion
// events; calls into Motion::JerkTrajectory for the actual solve, never
// does the solve math itself (jerk_trajectory.h's own boundary).
//
// 109-003 scope -- TIMED mode + replace only (see that ticket's own history
// below, kept for provenance). 109-005 scope -- DISTANCE mode: dominant-
// channel arc/pivot planning, the heading-reference/feedforward the arc
// ratio implies, dwell completion, and encoder-relative distance completion
// with a same-sign overshoot carry. Both modes share ONE state machine and
// ONE ring -- there is no separate "distance executor".
//
// -- Three Cmd modes (Executor::Mode below, decided once at activate()) --
//   kTimed -- Cmd::isTimed() (time > 0): both channels driven directly and
//     independently by Cmd::vMax/Cmd::omega (109-003's own scope, unchanged
//     by this ticket).
//   kPivot -- Cmd::isPivot() (distance == 0, not timed, not degenerate): the
//     ROTATIONAL channel is the only planned channel, solved directly to
//     `deltaHeading` (JerkTrajectory::solveToRest()) -- the linear channel is
//     never solved at all (JerkTrajectory::sample() on an un-calculate()'d
//     instance safely returns a zero State{}, so `v` is 0 throughout with no
//     special-casing needed here).
//   kArc -- distance != 0, not timed: the LINEAR channel is the dominant,
//     planned channel (solved to `effectiveDistance_`, see the overshoot-
//     carry note below); the rotational channel is never solved -- it is
//     SLAVED every tick to the linear channel's own sampled (position,
//     velocity) via the arc ratio `headingRatioPerMm_ = deltaHeading /
//     distance` (computed ONCE at activation from the Cmd's own, un-adjusted
//     distance -- the arc's curvature is a property of the REQUESTED
//     geometry, independent of exactly how far the overshoot carry nudges
//     the effective target): `thetaRef(t) = headingRatioPerMm_ *
//     linear.position(t)`, `omegaFf(t) = headingRatioPerMm_ *
//     linear.velocity(t)`. `deltaHeading == 0` (a plain straight leg) is
//     just the `headingRatioPerMm_ == 0` special case of the same formula --
//     no separate branch.
//
// -- Heading feedforward vs. the heading PD cascade --
// This class computes and exposes the feedforward half (`omegaFf`/
// `thetaRef` in Twist below) and the MEASURED-heading bookkeeping needed to
// close the loop (`thetaMeas`, the command-relative measured heading), but
// the PD CORRECTION TERM ITSELF (`heading_kp*(thetaRef-thetaMeas) +
// heading_kd*(omegaDes-omegaMeas)`) is computed by App::Pilot::tick(), not
// here -- sprint.md's own SUC-002 flow assigns that arithmetic to Pilot
// explicitly ("Each cycle, Pilot::tick() computes omega_cmd = omega_ff +
// heading_kp*(...)"). This keeps every heading GAIN (`heading_kp`/
// `heading_kd`) and every MEASUREMENT source (App::HeadingSource) entirely
// out of this leaf -- Executor stays a pure planner or msg::PlannerConfig
// and its own remembered command-relative state, never touching
// Devices::Otos/NezhaMotor or a sensor-fusion policy. Twist.omega therefore
// means TWO different things depending on mode: for kTimed and for a
// non-heading-bearing kArc leg (`deltaHeading == 0`), it is the FINAL
// commanded rate; for a heading-bearing kArc/kPivot command with
// `headingActive` true, it is `omegaFf` ONLY -- Pilot adds the PD term on
// top before calling Drive::setTwist(). This is a deliberate overload
// (documented here, not a bug) -- see Twist's own field comments.
//
// -- Terminal-decel PD gate --
// `headingActive` is ALSO false once the command has ALREADY satisfied the
// dwell gate's own tolerance/rate test (`headingDwellTol_`/
// `headingDwellRate_` -- the SAME test `tick()`'s completion logic uses),
// even for a heading-bearing command still technically "active" (not yet
// dwell-held long enough to complete, or not terminal so no dwell needed at
// all) -- "gated off during terminal decel" (sprint.md/ticket 005's own
// semantics item 3), read as an ERROR-based condition ("already landed,
// stop nudging it") rather than a fixed time-before-planned-completion
// window. A time-based window was this ticket's OWN first implementation
// and was caught by this ticket's own sim system test
// (test_heading_source.py): it disabled the PD correction during the final
// portion of the PLANNED trajectory's own duration regardless of the REAL
// plant's measured error at that point, so a real (non-ideal, laggy) plant
// that was still meaningfully off target when the window opened had its
// correction authority pulled right when it was needed most, latching a
// several-degree overshoot the PD was never given the chance to close. The
// error-based gate closes exactly the failure mode
// `.clasi/knowledge/d-drive-terminal-instability.md` documents (a commanded
// REVERSAL right at an ALREADY-GOOD landing) without also disabling
// correction while genuinely still far from the target.
//
// -- Distance completion + same-sign overshoot carry --
// "Encoder-relative travel" (ticket 005's own wording) means App::Pilot
// accumulates App::Odometry::lastDistance() every tick while a command is
// active and passes the running total into THIS class's own tick() call
// (`measuredDistanceDelta` -- Executor holds the accumulator
// (`measuredPathSinceActivation_`), not Pilot, so completion stays this
// class's own decision, matching the file's boundary comment: "calls into
// JerkTrajectory for the actual solve, never does the solve math itself" --
// Executor still gets to decide WHEN a command is done, from measured
// inputs Pilot merely samples and hands over). A kArc command (distance !=
// 0) completes its DISTANCE half once `|measuredPathSinceActivation_| >=
// |effectiveDistance_|`; the signed remainder (`measuredPathSinceActivation_
// - effectiveDistance_`) becomes `pendingOvershoot_`, consumed (added into
// `effectiveDistance_ = cmd.distance - pendingOvershoot_`) by the VERY NEXT
// activation IFF that next command is itself a same-sign kArc command --
// any other next command (kTimed, kPivot, opposite-sign kArc) silently
// drops the pending carry rather than applying it somewhere it doesn't
// belong. This is single-command bookkeeping only, NOT the full boundary-
// velocity carry (ticket 006's own scope) -- there is no attempt here to
// avoid decelerating to rest at the shared boundary, only to not silently
// lose a few mm of over/under-travel at a queue boundary.
//
// -- Dwell completion (heading-bearing commands) --
// A REST-TERMINATED heading-bearing command (this is the LAST command in
// the queue -- `queueCount_ == 0` at the moment its own distance/pivot
// criterion is met) must additionally hold `|deltaHeading - thetaMeas| <
// heading_dwell_tol` AND `|thetaRate| < heading_dwell_rate`
// (msg::PlannerConfig, ticket 005's own new fields) for `arrive_dwell`
// seconds (REUSED from the existing terminal-completion dwell field --
// ticket 005's own semantics item 4 numeric match: 150ms is both the
// existing arrive_dwell default AND the ticket's own dwell-hold spec) before
// completing DONE; a `STOP_TIME` backstop (`stopTimeBackstopMs()`, a
// generous multiple of the dominant channel's own solved duration) forces
// completion regardless, so a persistent oscillation or a measurement fault
// can never wedge the executor open forever. A CHAINED (non-terminal --
// `queueCount_ > 0`) heading-bearing command skips the hold-timer/rate gate
// entirely and completes on the tolerance test alone (no dwell) --
// "chained... use encoder/OTOS-accurate handoff without a dwell" (ticket
// 005's own semantics item 4).
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
  kAccepted,  // activated immediately or appended to the ring
  kReplaced,  // replaced the ring tail or retargeted the active command
  kFull,      // ring already at kQueueDepth; plan untouched
  kTrivial,   // degenerate Move -- never queued
};

constexpr uint8_t kQueueDepth = 8;
constexpr uint8_t kEventRingDepth = 8;

// kDeadTime -- [ms] the divergence-replan dead-time projection ticket 006's
// own checkDivergence() is documented to use, projecting "where should the
// plan already be" forward past a measurement's own transport lag. Re-
// derived at the 40ms cycle (109-005) -- see app/DESIGN.md's own
// "kDeadTime" Open-Questions entry for the full derivation: NOT hand-picked
// by scaling the old 120ms/20ms-tick constant onto the new cycle (explicitly
// disallowed by this ticket's own semantics), but also NOT a fresh bench
// characterization (the USB deploy path was confirmed broken this session
// -- one `mbdeploy probe` attempt, per hardware-bench-testing.md's own
// escalation path). Set to 130ms -- the midpoint of sprint 100's own
// ALREADY bench-measured `motor_lag` figure (120-140ms,
// architecture-update.md), a real-time physical actuation-transport delay
// independent of cycle period, not a tick-count artifact.
//
// STILL NOT LIVE, by deliberate choice, not oversight: ticket 006's own
// implementation tried wiring this into checkDivergence() as a
// `peek(elapsed + kDeadTime)` lead and found it produces FALSE-POSITIVE
// divergence triggers against these sub-second pivot/arc trajectories in
// the sim system tests (109-005's own scenarios 8/9/11 in
// `motion_executor_harness.cpp` regressed) -- 130ms is a large fraction of
// a typical pivot's own total duration, so "where the plan will be 130ms
// from now" is not a fair stand-in for "where the plan already is" without
// a matching measured-transport-lag model on the OTHER side of the
// comparison (the sim's own measured signal today has no real transport
// lag to project past at all). `checkDivergence()` compares against the
// CURRENT elapsed sample instead (correct for a zero-lag measured signal);
// this constant stays declared, with its derivation preserved here, for
// the real dead-time wiring once a genuine bench characterization exists
// (USB deploy confirmed broken this session) to validate the projection
// against.
constexpr uint32_t kDeadTime = 130;  // [ms]

class Executor {
 public:
  struct Twist {
    float v = 0.0f;      // [mm/s] linear velocity to command this cycle
    // omega -- see this file's own "Heading feedforward vs. the heading PD
    // cascade" comment for the mode-dependent meaning: FINAL commanded rate
    // unless headingActive is true, in which case this is omegaFf ONLY and
    // the caller (App::Pilot) must add the PD term.
    float omega = 0.0f;  // [rad/s]

    // headingActive -- true iff the caller should compute and add the
    // heading PD term this cycle (a heading-bearing kArc/kPivot command,
    // NOT in its terminal-decel window). false for kTimed, for a non-
    // heading-bearing kArc leg, and during terminal decel.
    bool headingActive = false;
    // withinTolerance -- this tick's own dwell-tolerance test result
    // (|thetaErrLead| < heading_dwell_tol), exported for App::Pilot's
    // minimum-command floor (2026-07-18): the floor drives the terminal
    // approach only while OUTSIDE tolerance and disengages inside it, so
    // the plant coasts to rest in the band instead of bang-banging
    // through it. Meaningful only while headingActive.
    bool withinTolerance = false;

    // thetaRef/thetaMeas -- both RELATIVE to this command's own activation
    // (its own theta==0 origin), same units/frame, meaningful only when
    // headingActive (or, for thetaMeas, whenever a heading-bearing command
    // is active at all, active-decel-window included -- Pilot's own
    // finite-difference rate estimate needs a continuous thetaMeas
    // sequence, not one that goes stale the instant headingActive flips
    // off). thetaRef is the arc/pivot's own progressive desired heading
    // (headingRatioPerMm_ * linear.position(t) for kArc, rotational.
    // position(t) for kPivot); thetaMeas is the CONTINUOUS (unwrapped)
    // relative heading accumulated since activation (109-009 fix -- see
    // `unwrappedThetaRel_`'s own doc comment for why a single wrapAngle()
    // diff against a fixed baseline is wrong for a |deltaHeading| > 180deg
    // command), where measuredHeadingAbs is this tick()'s own
    // caller-supplied App::HeadingSource reading.
    float thetaRef = 0.0f;   // [rad]
    float thetaMeas = 0.0f;  // [rad]

    // thetaMeasLead -- 109-010 locus 1: thetaMeas projected forward by
    // App::HeadingSource's own measurement-age lead (HeadingSource::
    // headingLead(), fed in via tick()'s own measuredHeadingLeadAbs
    // parameter). App::Pilot's PD cascade uses THIS for its error term
    // (thetaRef - thetaMeasLead), not the raw thetaMeas above -- thetaMeas
    // itself stays exactly as it was (feeding this class's own dwell/
    // divergence bookkeeping, unchanged, un-led). Meaningful only when
    // headingActive (or, like thetaMeas, while a heading-bearing command is
    // active at all) -- 0 otherwise.
    float thetaMeasLead = 0.0f;  // [rad]

    // omegaDes -- the heading PD law's own "omega_des" term (see this
    // file's own header comment for the full formula) -- for a heading-
    // bearing command this equals omegaFf (the SAME feedforward rate
    // driving `omega` above), exposed under its own name for readability at
    // the Pilot call site, which needs it paired with a separately-computed
    // omegaMeas (a measured-heading finite difference Pilot itself keeps,
    // since it spans TWO ticks and Executor's own tick() call is stateless
    // from Pilot's point of view). 0 when !headingActive.
    float omegaDes = 0.0f;  // [rad/s]
  };

  // configure -- stores both channels' own limits (forwarded to the two
  // owned JerkTrajectory instances' configure()) plus the decel/jerk pair
  // this class's own estimateStopDuration() scheduling heuristic needs, and
  // (109-005) the heading-dwell gate's own tolerance/rate/hold-time fields.
  // Must be called before the first enqueue().
  void configure(const msg::PlannerConfig& config);

  // enqueue -- classify and admit one Cmd. See this class's own doc
  // comment for the degenerate/kTimed/kArc/kPivot/replace decision tree.
  EnqueueOutcome enqueue(const Cmd& cmd);

  // flush -- TWIST/STOP preemption (App::Pilot::flush()). Empties the ring
  // and clears any active command, pushing a kFlushed completion event for
  // each, and returns to kIdle. Does not touch Drive itself -- the caller
  // owns whatever twist Drive ends up staged with afterward.
  void flush();

  // plan -- at most one JerkTrajectory solve this call. See this class's
  // own "Solve budget" doc comment (kept from 109-003, unchanged in shape:
  // still exactly one solveToVelocity()/solveToRest() call per plan(),
  // just dispatched to the right one for this command's own Mode).
  void plan();

  // tick -- sample-only: advances the active command's own elapsed time by
  // dtMs, samples the planned channel(s), evaluates completion (TIMED
  // deadline/RAMP_TO_REST, or 109-005's DISTANCE/dwell criteria), and
  // returns the twist this cycle's Drive::setTwist() call should stage
  // ({0,0} while kIdle). Never solves, never touches the bus.
  //
  // measuredDistanceDelta -- App::Odometry::lastDistance() THIS cycle
  // ([mm], encoder-relative, signed) -- accumulated internally into
  // measuredPathSinceActivation_ for the DISTANCE-completion criterion.
  // measuredHeadingAbs -- App::HeadingSource::heading() THIS cycle ([rad],
  // absolute) -- rebaselined internally to this command's own activation
  // instant to produce Twist::thetaMeas. measuredHeadingLeadAbs (109-010) --
  // App::HeadingSource::headingLead() THIS cycle ([rad], absolute,
  // measurement-age-projected) -- rebaselined the SAME way to produce
  // Twist::thetaMeasLead, a SEPARATE quantity from thetaMeas (see that
  // field's own doc comment). All three are harmless to pass even when the
  // active command doesn't use them (kTimed ignores all three) -- defaulted
  // so 109-003's own kTimed-only test callers (which never needed any of
  // them) keep compiling unchanged.
  Twist tick(uint32_t dtMs, float measuredDistanceDelta = 0.0f, float measuredHeadingAbs = 0.0f,
             float measuredHeadingLeadAbs = 0.0f);  // [ms] [mm] [rad] [rad]

  // popEvent -- drains one pending completion event, oldest first. Returns
  // false (out untouched) when none pending.
  bool popEvent(CompletionEvent* out);

  uint8_t queueDepth() const { return queueCount_; }
  uint32_t activeId() const { return active_.id; }
  State state() const { return state_; }

 private:
  // Mode -- which of the three shapes (file header) the ACTIVE command is.
  // Decided once, in activate(), from the Cmd's own isTimed()/isPivot().
  enum class Mode : uint8_t { kTimed, kArc, kPivot };

  // activate -- makes cmd the active command. retarget=false is a fresh
  // start-from-rest activation (JerkTrajectory::reset() on both channels);
  // retarget=true is a replace-while-active in-place retarget (channels
  // keep their own remembered last-sample seed -- see jerk_trajectory.h's
  // seeding contract -- so the new target is approached smoothly, never as
  // an instantaneous step). Either way, requests fresh solve(s) for
  // whichever channel(s) this Cmd's own Mode needs (serviced by the next
  // one or two plan() calls) and resets every piece of per-activation
  // bookkeeping (elapsed clocks, the measured-progress accumulator, the
  // heading baseline, the dwell hold timer).
  void activate(const Cmd& cmd, bool retarget);

  // activateNextOrIdle -- pops the ring's head (if any) and activates it;
  // otherwise clears the active command and returns to kIdle. Called when
  // the active command reaches its own DONE criterion.
  //
  // 109-006: velocity-continuous handoff (trigger (d)). Rather than an
  // unconditional fresh-from-rest reset of BOTH channels, this seeds the
  // JUST-COMPLETED command's own dominant channel (linear_ for kArc,
  // rotational_ for kPivot) from THIS tick's own last sample
  // (completionLinearVelocity_/completionLinearAcceleration_ or the
  // rotational twin -- set every kArc/kPivot tick(), see that method's own
  // comment) before calling activate(next, retarget=true) -- position
  // resets to 0 (the new command's own frame), velocity/acceleration carry
  // through unchanged. Harmless when the completing command's own exit
  // velocity was 0 (a rest-terminated or sign-reversal-forced completion):
  // its own last sample is already at (or within epsilon of) rest, so
  // seeding from it reads the same as a hard reset(). The non-dominant
  // channel (and a kTimed completion, which skips this seeding entirely)
  // still gets a full reset() -- only the ONE channel actually carrying a
  // boundary velocity is ever seeded nonzero.
  void activateNextOrIdle();

  // completeActive -- shared tail for every completion path (TIMED
  // deadline/RAMP_TO_REST rest, DISTANCE/dwell criteria, solve failure):
  // stages pendingOvershoot_ (kArc only -- see this file's own "Distance
  // completion" comment; a no-op for kTimed/kPivot), pushes the completion
  // event, and calls activateNextOrIdle() -- EXCEPT for kSolveFail (109-006):
  // a solve failure flushes the REST of the ring too (each own kFlushed --
  // continuing to the next queued command on the same, evidently broken,
  // channel configuration is not obviously safer than stopping outright)
  // and enters emergencyStopping_ (both channels solveToVelocity(0), see
  // that field's own comment) instead of calling activateNextOrIdle()
  // immediately -- activateNextOrIdle() itself only runs once tick() has
  // observed both channels actually at rest, so Drive is never left
  // holding a stale nonzero twist just because state() dropped to kIdle
  // before either channel had a chance to decelerate.
  void completeActive(CompletionStatus status);

  // computeExitVelocity -- ticket 006's own exitSpeed(active, next)
  // formula (sprint.md/the sprint issue's own "Boundary velocity" section,
  // verbatim): 0 when there is no queued successor, the successor is
  // TIMED (boundary-velocity carry is a DISTANCE-mode-only concept -- a
  // TIMED successor is handled by 109-003's own in-place
  // solveToVelocity()/replace path instead), or the active/successor pair
  // mismatches on pivot-ness (an arc chaining into a pivot or vice versa --
  // "pivot on either side" forces a full decel-to-rest at the boundary,
  // since a pivot's own dominant channel is rotational, not linear, so
  // there is no shared channel to carry a velocity through) or reverses
  // sign (a genuine direction change must decelerate through zero, never
  // carry a signed velocity across a reversal). Otherwise: the domain
  // (linear for an active/next kArc pair, rotational for an active/next
  // kPivot pair -- "pivot->pivot chains carry rotational velocity through
  // the SAME rule in the rotational domain") ceiling-mins the active and
  // next commands' own effective vmax, then additionally clamps by
  // reachableEntrySpeed(|next's own linear distance or angular delta|) --
  // the fastest speed this channel could enter a segment of that length at
  // and still be able to decelerate to rest by its own end, given this
  // channel's own aDecel/jerk. One-command lookahead only (never chases a
  // second successor beyond ring_[0]) -- reads active_/mode_/ring_[0]/
  // queueCount_, does not mutate anything.
  float computeExitVelocity() const;

  // maybeRetargetActiveForSuccessorChange -- replan triggers (a)/(b)-tail:
  // recomputes computeExitVelocity() (the active's immediate successor,
  // ring_[0], may have just changed -- an append to a previously-empty
  // ring, or a tail replace when ring_[0] IS the tail) and, if the new
  // value differs from the currently-planned exitVelocity_ by more than
  // the domain's own threshold (>1mm/s linear, >0.02rad/s rotational),
  // requests a fresh in-place solveToState() re-solve of the active's own
  // dominant channel toward the SAME target position at the NEW exit
  // velocity (seeded from that channel's own remembered last sample --
  // never a measured observation, matching JerkTrajectory's own seeding
  // contract). A change at or below threshold still updates exitVelocity_
  // (kept accurate for the NEXT comparison) but does not spend a plan()
  // solve on an imperceptible adjustment. Safe to call after ANY ring_/
  // queueCount_ mutation regardless of whether it actually changed ring_[0]
  // -- if it did not, computeExitVelocity() returns the same value and
  // this is a no-op.
  void maybeRetargetActiveForSuccessorChange();

  // checkDivergence -- replan trigger (c), called once per kArc/kPivot
  // tick() (never for kTimed -- that mode has no position-control target
  // to diverge from). Compares the MEASURED channel state against the
  // dominant channel's own kDeadTime-projected planned position (see that
  // constant's own doc comment) and, past a threshold, sets a pending-
  // solve flag/parameter members for plan() to service next -- this
  // method itself never calls into JerkTrajectory (tick() "never solves",
  // this file's own header comment; every actual retarget()/reanchor()
  // call happens inside plan(), see this file's own "Solve budget" note
  // carried from 109-003).
  //   - kArc, linear domain: |measuredPathSinceActivation_ minus the
  //     projected planned position| >= 40mm (and at least
  //     kDivergenceReanchorMinIntervalMs since the last reanchor, either
  //     channel) -> pendingLinearReanchor_ (the ONE sanctioned measured-
  //     velocity seed, via a finite difference of this tick's own
  //     measuredDistanceDelta/dt -- reanchor()'s own documented exception,
  //     acceleration forced to 0 by reanchor() itself); >= 5mm (below the
  //     reanchor threshold) -> pendingLinearRetarget_ (a position-target
  //     correction only -- still seeded from the channel's own remembered
  //     velocity/acceleration, never measured, matching retarget()'s own
  //     contract).
  //   - kPivot, rotational domain: >= 0.3rad -> pendingRotationalReanchor_
  //     only -- no separate small-threshold retarget tier for heading (the
  //     PD cascade in App::Pilot already continuously corrects small
  //     heading drift; only a GROSS divergence during a pivot warrants
  //     replanning the trajectory itself).
  // A retarget() re-baselines its own channel's position frame to 0 --
  // linearFrameOffset_/rotationalFrameOffset_ (this file's own field
  // comments) track the cumulative rebase so thetaRef/omegaFf (both
  // derived from the dominant channel's own sampled position) stay correct
  // in the command's own since-activation frame across any number of
  // rebases; reanchor() does NOT rebase (its position argument is supplied
  // directly in the channel's own current frame), so no offset update
  // follows a reanchor.
  //
  // plannedPositionSinceActivation is the SAME frame-offset-adjusted
  // dominant-channel position tick() already computed this cycle for
  // Twist::thetaRef (kArc: linearFrameOffset_ + the linear channel's own
  // sampled position; kPivot: rotationalFrameOffset_ + the rotational
  // channel's own sampled position) -- compared at the CURRENT elapsed
  // time, not a kDeadTime-projected one; see kDeadTime's own doc comment
  // for why the dead-time lead is declared but not yet wired into this
  // comparison (a naive elapsed+kDeadTime projection produced false-
  // positive divergence triggers against these sub-second pivot/arc
  // trajectories during this ticket's own implementation).
  void checkDivergence(float dtS, float measuredDistanceDelta, float thetaMeasRel, float thetaRate,
                        float plannedPositionSinceActivation);

  // resolveFromRest -- plan()'s recovery for a position-solve that failed on
  // a stale carried state (see its own definition comment, executor.cpp).
  // Resets `chan` to rest and re-solves to `posTarget` at rest; returns true
  // and zeroes *elapsed on success.
  bool resolveFromRest(JerkTrajectory& chan, float* elapsed, float posTarget, float ceiling);

  // stopTimeBackstopMs -- 109-005's own STOP_TIME backstop for a dwell-
  // gated heading command: a generous multiple of the dominant channel's
  // own solved duration, so a persistent oscillation or a stuck measurement
  // can never wedge the executor open forever. See this file's own "Dwell
  // completion" comment.
  uint32_t stopTimeBackstopMs() const;  // [ms]

  void pushEvent(uint32_t id, CompletionStatus status);

  Cmd active_;
  bool activeValid_ = false;
  uint32_t activeElapsedMs_ = 0;  // [ms] since this active command's own activate()
  Mode mode_ = Mode::kTimed;

  Cmd ring_[kQueueDepth]{};
  uint8_t queueCount_ = 0;

  State state_ = State::kIdle;

  JerkTrajectory linear_;
  JerkTrajectory rotational_;

  bool needLinearSolve_ = false;
  bool needRotationalSolve_ = false;
  float pendingLinearTarget_ = 0.0f;      // [mm/s] kTimed's own solveToVelocity() target
  float pendingRotationalTarget_ = 0.0f;  // [rad/s] kTimed's own solveToVelocity() target
  float pendingLinearVMax_ = 0.0f;        // [mm/s] kArc's own solveToRest() per-call ceiling (Cmd::vMax)

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

  // -- 109-005: DISTANCE-mode (kArc/kPivot) bookkeeping --
  float headingRatioPerMm_ = 0.0f;  // [rad/mm] deltaHeading/distance, kArc only, set once at activate()
  float effectiveDistance_ = 0.0f;  // [mm] cmd.distance adjusted by a carried-in same-sign overshoot
  float measuredPathSinceActivation_ = 0.0f;  // [mm] signed, App::Odometry::lastDistance() accumulated
  float pendingOvershoot_ = 0.0f;   // [mm] signed carry -- see this file's own "Distance completion" comment

  bool headingBaselineSet_ = false;
  // unwrappedThetaRel_/lastMeasuredHeadingAbs_ (109-009 fix): thetaMeasRel used to be a single
  // wrapAngle(measuredHeadingAbs - headingBaselineAbs) against a fixed activation-time baseline --
  // correct only while the total rotation since activation stays within (-pi, pi]. App::HeadingSource's
  // OTOS reading itself wraps at
  // +-180deg (the real chip's own convention), so a commanded |deltaHeading| > 180deg (TOUR_2's
  // own "RT -21700"/"RT 21500" legs, -217deg/+215deg) crossed that wrap mid-turn: measuredHeadingAbs
  // itself jumped by a full 2*pi right at the wrap boundary, and the single-formula diff aliased to
  // the WRONG relative angle, corrupting thetaErr for the rest of the command (and, once handed off,
  // left the executor in a state where the next queued command's own ack never arrived -- the sim
  // tour-closure gate's leg-6 timeout). Fixed by accumulating the CONTINUOUS relative angle
  // incrementally every tick() -- wrapAngle(thisReading - lastReading) is always small (bounded by
  // one cycle's own worst-case rotation rate, nowhere near +-180deg) and is added onto a running
  // unwrapped total, which can legitimately exceed +-180deg. lastMeasuredHeadingAbs_ is re-seeded
  // (to measuredHeadingAbs, with unwrappedThetaRel_ zeroed) on the SAME first-tick condition
  // headingBaselineSet_ already gated the old single-shot baseline on.
  float unwrappedThetaRel_ = 0.0f;       // [rad] CONTINUOUS relative heading since activation -- this
                                         // is the actual thetaMeasRel used everywhere below now.
  float lastMeasuredHeadingAbs_ = 0.0f;  // [rad] previous tick()'s raw (wrapped) measuredHeadingAbs
  float prevThetaMeasRel_ = 0.0f;    // [rad] previous tick()'s thetaMeas -- this class's own dwell-rate estimate
  uint32_t dwellHeldMs_ = 0;         // [ms] how long the dwell gate has held continuously
  float dwellRateFilt_ = 0.0f;       // [rad/s] 109-009 fix: exponentially-smoothed thetaRate used
                                      // ONLY by the dwell gate's own rate test (headingDwellRate_) --
                                      // see executor.cpp's own dwell-completion comment for why a raw
                                      // one-sample finite-difference derivative is unusable under the
                                      // realistic sim OTOS/encoder noise profile. The PD/completion's
                                      // OTHER uses of the instantaneous thetaRate are unaffected.

  float headingDwellTol_ = 0.0f;   // [rad] msg::PlannerConfig.heading_dwell_tol
  float headingDwellRate_ = 0.0f;  // [rad/s] msg::PlannerConfig.heading_dwell_rate
  float headingDwellHoldS_ = 0.0f; // [s] msg::PlannerConfig.arrive_dwell (REUSED, see file header)

  // -- 109-010: lead-compensation loci 2/3, see planner.proto's own
  // plan_lead/terminal_lead doc comments and this file's tick()/plan()
  // implementation comments for where each is applied. --
  float planLeadS_ = 0.0f;      // [s] msg::PlannerConfig.plan_lead (locus 2)
  float terminalLeadS_ = 0.0f;  // [s] msg::PlannerConfig.terminal_lead (locus 3)

  // -- 109-006: boundary-velocity carry + divergence replan --

  // exitVelocity_ -- the target-velocity argument fed to the ACTIVE
  // command's own dominant channel's solveToState() call (kArc: linear_,
  // kPivot: rotational_) -- computeExitVelocity()'s own current value.
  // Recomputed at activate() and by maybeRetargetActiveForSuccessorChange()
  // whenever the active's immediate successor (ring_[0]) changes; see
  // those methods' own comments.
  float exitVelocity_ = 0.0f;  // [mm/s] or [rad/s] signed, domain per mode_

  // linearFrameOffset_/rotationalFrameOffset_ -- the cumulative amount
  // added to the dominant channel's own sampled `position` to recover
  // "position since this command's own activation" (thetaRef/omegaFf's
  // own frame) after any number of divergence retarget() rebases (each of
  // which re-baselines that channel's OWN internal position frame to 0 --
  // see checkDivergence()'s own comment). 0 until the first retarget();
  // reset to 0 in activate() (a fresh command starts its own fresh frame).
  float linearFrameOffset_ = 0.0f;      // [mm]
  float rotationalFrameOffset_ = 0.0f;  // [rad]

  // pendingLinearReanchor_/pendingLinearRetarget_/pendingRotationalReanchor_
  // -- set by checkDivergence() (tick()), serviced by plan() (the ONLY
  // place that ever calls JerkTrajectory::retarget()/reanchor() -- tick()
  // itself never solves). No small-threshold rotational retarget tier --
  // see checkDivergence()'s own comment.
  bool pendingLinearReanchor_ = false;
  // pendingLinearRetarget_ -- as of the 2026-07-18 "plan once, finish on
  // the spot" restructure this is NO LONGER a mid-flight divergence
  // correction (that 5mm tier caused terminal reversal ringing -- see
  // checkDivergence()'s own comment). It is now set ONLY by tick()'s
  // terminal top-up: the profile ran out, the channel is at rest, and the
  // MEASURED distance landed short of target beyond the settle epsilon --
  // plan() then solves the (always forward, from-rest) remainder.
  bool pendingLinearRetarget_ = false;

  // msSinceLastReanchor_ -- time since the last reanchor() on EITHER
  // channel (a single shared timer -- only one of linear_/rotational_ is
  // ever the active dominant channel at a time, so there is never a need
  // to distinguish which channel it last gated). Reset to
  // kDivergenceReanchorMinIntervalMs (this file's own anonymous-namespace
  // constant, executor.cpp) at activate() so a fresh command's own first
  // possible reanchor is never blocked by a PREVIOUS command's timer.
  uint32_t msSinceLastReanchor_ = 0;  // [ms]

  // lastMeasuredVelocity_/lastThetaMeasRel_/lastThetaRate_ -- this tick's
  // own measured-state snapshot, cached so plan() (which runs AFTER tick()
  // within the same cycle, per app/DESIGN.md's own cycle-placement table)
  // can read them when servicing a pendingLinearReanchor_/
  // pendingRotationalReanchor_ request -- reanchor()'s own caller-supplied
  // position/velocity seed (the one sanctioned measured-state exception)
  // without plan() needing its own measured-signal parameters (plan()
  // takes none, matching its existing 109-003 signature).
  float lastMeasuredVelocity_ = 0.0f;  // [mm/s] measuredDistanceDelta/dt, this tick's own estimate

  // completionLinear{Velocity,Acceleration}_/completionRotational{Velocity,
  // Acceleration}_ -- this tick's own dominant-channel sample, refreshed
  // every kArc/kPivot tick() (unconditionally, cheap) so that IF this
  // tick's own completion test fires, activateNextOrIdle()'s velocity-
  // continuous handoff (this file's own doc comment) has a same-cycle,
  // exact seed to hand the next command's own dominant channel -- never
  // stale by even one cycle.
  float completionLinearVelocity_ = 0.0f;          // [mm/s]
  float completionLinearAcceleration_ = 0.0f;      // [mm/s^2]
  float completionRotationalVelocity_ = 0.0f;      // [rad/s]
  float completionRotationalAcceleration_ = 0.0f;  // [rad/s^2]

  // emergencyStopping_ -- 109-006's own solve-failure safety net
  // (completeActive()'s own kSolveFail branch): true while both channels
  // are being driven to solveToVelocity(0) after a solve failure, checked
  // first (ahead of mode_/pendingLinear*_ dispatch) by BOTH plan() (always
  // solveToVelocity(0), never the normal mode_-dependent solve) and tick()
  // (a dedicated sample-both/check-rest/activateNextOrIdle() branch at the
  // very top, bypassing the normal kTimed/kArc/kPivot dispatch and its own
  // distance/dwell completion tests entirely) -- prevents Drive from being
  // left holding a stale nonzero twist just because a solve failed mid-
  // cruise (state()!=kIdle is the ONLY thing that keeps App::Pilot::tick()
  // calling Drive::setTwist() at all).
  bool emergencyStopping_ = false;

  CompletionEvent events_[kEventRingDepth]{};
  uint8_t eventCount_ = 0;
};

}  // namespace Motion
