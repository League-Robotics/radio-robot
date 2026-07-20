// executor.cpp -- Motion::Executor implementation. See executor.h's file
// header for the module's boundary, the three Cmd modes (kTimed/kArc/
// kPivot), the heading-feedforward/PD-cascade split with App::Pilot, the
// terminal-decel PD gate, and the distance/dwell completion criteria.
#include "motion/executor.h"

#include <algorithm>
#include <cmath>

namespace Motion {

namespace {

// estimateStopDuration -- see executor.h's own doc comment: an analytic
// v1 approximation of how long a solveToVelocity(0, ...) decel from
// `velocity` would take, given this channel's own configured decel/jerk
// limits. Trapezoidal term (|v|/aDecel) plus one jerk-ramp term
// (aDecel/jerk) approximates the extra time an S-curve's jerk-limited
// accel/decel ramps add over a pure trapezoid -- a scheduling heuristic,
// not the exact solve (Ruckig's own solveToVelocity() is the exact one).
float estimateStopDuration(float velocity, float aDecel, float jerk) {  // -> [s]
  float v = std::fabs(velocity);
  // A channel already at (approximately) rest needs no decel time at all --
  // without this guard, a channel that never moves (e.g. the rotational
  // channel of a pure-linear TIMED command, omega==0 throughout) would
  // still report a spurious jerk-ramp-only stop duration (aDecel/jerk) even
  // at v==0, which could exceed a short TIMED deadline and fire
  // RAMP_TO_REST before the OTHER (actually moving) channel ever ramps up
  // at all -- exactly the bug this guard fixes (109-003 own bench/sim
  // debugging: a rotational aDecel/jerk pair alone produced a 250ms
  // "stop-needed" estimate at v==0, starving a 300ms TIMED command's
  // entire ramp).
  if (aDecel <= 0.0f || v <= 0.0f) return 0.0f;
  float t = v / aDecel;
  if (jerk > 0.0f) t += aDecel / jerk;
  return t;
}

// Rest epsilons -- "close enough to zero to call this channel stopped" for
// the RAMP_TO_REST completion gate. v1 constants (no bench-tuning input
// yet, unlike the sprint-098-era heading-loop gates ticket 005 restores).
constexpr float kLinearRestEpsilon = 2.0f;        // [mm/s]
constexpr float kRotationalRestEpsilon = 0.05f;   // [rad/s] (~2.9 deg/s)

// kStopTimeBackstopFactor/kStopTimeBackstopMarginS -- see executor.h's own
// "Dwell completion" comment: stopTimeBackstopMs() = dominant channel's own
// solved duration * kStopTimeBackstopFactor + kStopTimeBackstopMarginS. v1,
// not bench-tuned -- generous on purpose (this is a last-resort backstop,
// not the normal completion path).
constexpr float kStopTimeBackstopFactor = 2.0f;
constexpr float kStopTimeBackstopMarginS = 6.0f;  // [s]

// 109-006: exit-speed-change trigger (a)/(b)-tail thresholds -- verbatim
// per the sprint issue's own "Replan triggers" section ("exit speed changes
// >1 mm/s" / rotational domain's own analogous >0.02rad/s).
constexpr float kExitVelocityLinearThreshold = 1.0f;      // [mm/s]
constexpr float kExitVelocityRotationalThreshold = 0.02f; // [rad/s]

// 109-006 divergence-trigger (c) thresholds, gross-fault tier ONLY as of
// the 2026-07-18 "plan once, finish on the spot" restructure: the old 5mm
// mid-flight linear RETARGET tier (and its 3-tick streak guard) is GONE --
// see checkDivergence()'s own comment for the terminal-reversal ringing it
// caused. What remains is the unambiguous reanchor tier: genuine
// slip/stall, never ordinary tracking lag.
constexpr float kDivergenceReanchorLinearMm = 40.0f;        // [mm]
constexpr uint32_t kDivergenceReanchorMinIntervalMs = 60;   // [ms]

constexpr float kPi = 3.14159265358979323846f;

// wrapAngle -- normalize to (-pi, pi], the same convention Devices::Otos's
// own HEADING register already uses (otos.h's own doc comment) -- so a
// wrapped App::HeadingSource reading fed in here needs no further
// normalization before this function runs.
float wrapAngle(float angle) {  // [rad] -> [rad]
  while (angle > kPi) angle -= 2.0f * kPi;
  while (angle <= -kPi) angle += 2.0f * kPi;
  return angle;
}

// reachableEntrySpeed -- ticket 006's own boundary-velocity formula
// (verbatim, sprint issue's "Boundary velocity" section): the fastest
// speed a channel could enter a `d`-length decel-to-rest segment at and
// still be able to reach rest by the segment's own end, given this
// channel's own aDecel/jerk. jerk<=0 (the existing "off -- trapezoid"
// sentinel, mirrored from JerkTrajectory's own mapJerkSentinel()) collapses
// to the pure trapezoid form (no jerk-ramp term); aDecel<=0 (an
// unconfigured/degenerate channel) has no meaningful decel bound at all,
// so returns 0 rather than a divide-by-zero -- the caller's own
// min(vmaxEff(active), vmaxEff(next), reachableEntrySpeed(...)) then just
// forces exitVelocity_ to 0, the same safe "decelerate to rest" outcome an
// unconfigured channel should produce anyway.
float reachableEntrySpeed(float d, float aDecel, float jerk) {  // [mm] or [rad], [.../s^2], [.../s^3] -> [.../s]
  d = std::fabs(d);
  if (aDecel <= 0.0f) return 0.0f;
  if (jerk <= 0.0f) return std::sqrt(2.0f * aDecel * d);
  float k = (aDecel * aDecel) / (2.0f * jerk);
  return -k + std::sqrt(k * k + 2.0f * aDecel * d);
}

}  // namespace

void Executor::configure(const msg::PlannerConfig& config) {
  linear_.configure(config, /*isRotational=*/false);
  rotational_.configure(config, /*isRotational=*/true);
  aDecelLinear_ = config.a_decel;
  jerkLinear_ = config.j_max;
  aDecelRotational_ = config.yaw_acc_max;
  jerkRotational_ = config.yaw_jerk_max;
  linearCeiling_ = config.v_body_max;
  rotationalCeiling_ = config.yaw_rate_max;

  headingDwellTol_ = config.heading_dwell_tol;
  headingDwellHoldS_ = config.arrive_dwell;
  distanceTol_ = config.distance_tol;  // [mm] 112-004: unified completion rule's own linear tolerance
}

void Executor::pushEvent(uint32_t id, CompletionStatus status) {
  CompletionEvent entry;
  entry.id = id;
  entry.status = status;

  if (eventCount_ < kEventRingDepth) {
    events_[eventCount_++] = entry;
    return;
  }
  // Ring full -- evict the oldest, matching App::Telemetry's own ack-ring
  // eviction policy (ring_[] stays chronological, oldest-first).
  for (uint8_t i = 1; i < kEventRingDepth; ++i) events_[i - 1] = events_[i];
  events_[kEventRingDepth - 1] = entry;
}

bool Executor::popEvent(CompletionEvent* out) {
  if (eventCount_ == 0) return false;
  *out = events_[0];
  for (uint8_t i = 1; i < eventCount_; ++i) events_[i - 1] = events_[i];
  --eventCount_;
  return true;
}

void Executor::activate(const Cmd& cmd, bool retarget) {
  active_ = cmd;
  activeValid_ = true;
  activeElapsedMs_ = 0;
  linearElapsedS_ = 0.0f;
  rotationalElapsedS_ = 0.0f;

  measuredPathSinceActivation_ = 0.0f;
  headingBaselineSet_ = false;
  unwrappedThetaRel_ = 0.0f;
  lastMeasuredHeadingAbs_ = 0.0f;
  prevThetaMeasRel_ = 0.0f;
  dwellHeldMs_ = 0;
  headingRatioPerMm_ = 0.0f;
  effectiveDistance_ = cmd.distance;

  // 109-006: a fresh command starts its own fresh divergence/frame
  // bookkeeping -- see each field's own executor.h doc comment.
  linearFrameOffset_ = 0.0f;
  rotationalFrameOffset_ = 0.0f;
  pendingLinearReanchor_ = false;
  msSinceLastReanchor_ = kDivergenceReanchorMinIntervalMs;
  emergencyStopping_ = false;

  if (!retarget) {
    linear_.reset();
    rotational_.reset();
  }

  if (cmd.isTimed()) {
    mode_ = Mode::kTimed;
    pendingLinearTarget_ = cmd.vMax;
    pendingRotationalTarget_ = cmd.omega;
    needLinearSolve_ = true;
    needRotationalSolve_ = true;
  } else if (cmd.isPivot()) {
    mode_ = Mode::kPivot;
    needLinearSolve_ = false;
    needRotationalSolve_ = true;
  } else {
    // kArc: distance != 0, not timed. Dominant channel is linear.
    // 112-004: effectiveDistance_ is already cmd.distance (set unconditionally
    // above) -- there is no overshoot carry to adjust it with anymore.
    mode_ = Mode::kArc;
    headingRatioPerMm_ = cmd.deltaHeading / cmd.distance;

    pendingLinearVMax_ = cmd.vMax;
    needLinearSolve_ = true;
    needRotationalSolve_ = false;
  }

  // 109-006: one-command-lookahead boundary velocity -- recomputed on
  // EVERY activation (ring_[0], if any, is this new active command's own
  // immediate successor at this point -- see computeExitVelocity()'s own
  // doc comment). A no-op (0) for kTimed.
  exitVelocity_ = computeExitVelocity();

  state_ = State::kRunning;
}

void Executor::activateNextOrIdle() {
  if (queueCount_ > 0) {
    Cmd next = ring_[0];
    for (uint8_t i = 1; i < queueCount_; ++i) ring_[i - 1] = ring_[i];
    --queueCount_;

    // 109-006: velocity-continuous handoff (trigger (d)) -- see this
    // method's own executor.h doc comment. Reset both channels first
    // (matching the pre-006 fresh-start default), then re-seed ONLY the
    // just-completed command's own dominant channel from this tick's own
    // last sample -- the non-dominant channel (never live for that mode)
    // stays a clean reset() so a LATER command that switches modes never
    // inherits stale state from an unrelated earlier command.
    linear_.reset();
    rotational_.reset();
    if (mode_ == Mode::kArc) {
      linear_.seedCurrent(0.0f, completionLinearVelocity_, completionLinearAcceleration_);
    } else if (mode_ == Mode::kPivot) {
      rotational_.seedCurrent(0.0f, completionRotationalVelocity_, completionRotationalAcceleration_);
    }
    activate(next, /*retarget=*/true);
    return;
  }
  active_ = Cmd{};
  activeValid_ = false;
  state_ = State::kIdle;
  needLinearSolve_ = false;
  needRotationalSolve_ = false;
}

void Executor::completeActive(CompletionStatus status) {
  // 112-004: no longer stages a same-sign overshoot carry here (deleted --
  // see this file's own "Distance completion" comment).
  pushEvent(active_.id, status);

  if (status == CompletionStatus::kSolveFail) {
    // 109-006 edge case: a solve failure means Ruckig itself rejected this
    // channel's own limits/target, not a transient measurement blip --
    // continuing on to the next queued command against the same, evidently
    // broken, configuration is not obviously safer than stopping outright.
    // Flush the rest of the ring (each own kFlushed -- this active command's
    // own kSolveFail event, just pushed above, stays the one distinct
    // terminal event for it) and drive both channels to rest instead of
    // calling activateNextOrIdle() immediately -- see emergencyStopping_'s
    // own doc comment for why an immediate kIdle transition here would be
    // unsafe (Drive would be left holding its last commanded nonzero
    // twist). activateNextOrIdle() itself runs once tick()'s own dedicated
    // emergencyStopping_ branch observes both channels actually at rest.
    for (uint8_t i = 0; i < queueCount_; ++i) {
      pushEvent(ring_[i].id, CompletionStatus::kFlushed);
    }
    queueCount_ = 0;
    emergencyStopping_ = true;
    state_ = State::kRampToRest;
    pendingLinearReanchor_ = false;
    needLinearSolve_ = true;
    needRotationalSolve_ = true;
    return;
  }

  activateNextOrIdle();
}

uint32_t Executor::stopTimeBackstopMs() const {
  float dominantDurationS = (mode_ == Mode::kPivot) ? rotational_.duration() : linear_.duration();
  return static_cast<uint32_t>(
      (dominantDurationS * kStopTimeBackstopFactor + kStopTimeBackstopMarginS) * 1000.0f);
}

float Executor::computeExitVelocity() const {
  if (mode_ != Mode::kArc && mode_ != Mode::kPivot) return 0.0f;
  if (queueCount_ == 0) return 0.0f;  // no successor -- decelerate to rest

  const Cmd& next = ring_[0];
  // Boundary-velocity carry only chains a DISTANCE-mode successor -- a
  // queued TIMED command is 109-003's own in-place solveToVelocity()/
  // replace territory, not this ticket's.
  if (next.isTimed()) return 0.0f;

  bool activeIsPivot = (mode_ == Mode::kPivot);
  bool nextIsPivot = next.isPivot();
  // "pivot on either side" -- an arc chaining into a pivot (or vice versa)
  // has no shared dominant channel to carry a velocity through.
  if (activeIsPivot != nextIsPivot) return 0.0f;

  if (activeIsPivot) {
    // Pivot->pivot: same rule, rotational domain.
    if (active_.deltaHeading == 0.0f || next.deltaHeading == 0.0f) return 0.0f;
    bool sameSign = (active_.deltaHeading > 0.0f) == (next.deltaHeading > 0.0f);
    if (!sameSign) return 0.0f;  // sign reversal -- decelerate through zero
    float ve = std::min(rotationalCeiling_,
                         reachableEntrySpeed(next.deltaHeading, aDecelRotational_, jerkRotational_));
    return (active_.deltaHeading > 0.0f) ? ve : -ve;
  }

  // Arc->arc (straight legs included -- deltaHeading==0 is just the
  // headingRatioPerMm_==0 special case, this file header's own note).
  if (active_.distance == 0.0f || next.distance == 0.0f) return 0.0f;
  bool sameSign = (active_.distance > 0.0f) == (next.distance > 0.0f);
  if (!sameSign) return 0.0f;  // sign reversal -- decelerate through zero
  float vmaxActive = (active_.vMax != 0.0f) ? std::min(std::fabs(active_.vMax), linearCeiling_) : linearCeiling_;
  float vmaxNext = (next.vMax != 0.0f) ? std::min(std::fabs(next.vMax), linearCeiling_) : linearCeiling_;
  float ve = std::min(std::min(vmaxActive, vmaxNext),
                       reachableEntrySpeed(next.distance, aDecelLinear_, jerkLinear_));
  return (active_.distance > 0.0f) ? ve : -ve;
}

void Executor::maybeRetargetActiveForSuccessorChange() {
  if (!activeValid_ || (mode_ != Mode::kArc && mode_ != Mode::kPivot)) return;

  float newExit = computeExitVelocity();
  float threshold =
      (mode_ == Mode::kPivot) ? kExitVelocityRotationalThreshold : kExitVelocityLinearThreshold;
  bool needsResolve = std::fabs(newExit - exitVelocity_) > threshold;
  exitVelocity_ = newExit;
  if (!needsResolve) return;

  if (mode_ == Mode::kArc) {
    needLinearSolve_ = true;
  } else {
    needRotationalSolve_ = true;
  }
}

void Executor::checkDivergence(float dtS, float measuredDistanceDelta, float thetaMeasRel,
                                float thetaRate, float plannedPositionSinceActivation) {
  if (mode_ != Mode::kArc && mode_ != Mode::kPivot) return;

  lastMeasuredVelocity_ = (dtS > 0.0f) ? measuredDistanceDelta / dtS : lastMeasuredVelocity_;
  (void)thetaRate;
  msSinceLastReanchor_ += static_cast<uint32_t>(dtS * 1000.0f);

  // plannedPositionSinceActivation is the SAME frame-offset-adjusted
  // dominant-channel position tick() already computed for thetaRef/
  // linearPos this cycle (elapsed, no dead-time projection -- see this
  // method's own executor.h doc comment and kDeadTime's own doc comment
  // for why the dead-time lead is declared but NOT wired into this
  // comparison yet: a naive elapsed+kDeadTime projection, tried during
  // this ticket's own implementation, produced false-positive divergence
  // triggers against a sub-second pivot/arc trajectory in the sim system
  // tests -- kDeadTime (130ms) is a meaningful FRACTION of a typical
  // pivot's own total duration, so "where the plan will be 130ms from
  // now" is not a fair stand-in for "where the plan already is" on these
  // short commands without a real measured-transport-lag model to match
  // it against. Comparing against the CURRENT elapsed sample instead is
  // correct today (the sim's own measured signal has no real transport
  // lag either) and safe to revisit once a genuine bench dead-time
  // characterization exists (USB deploy confirmed broken this session).
  if (mode_ == Mode::kArc) {
    float err = measuredPathSinceActivation_ - plannedPositionSinceActivation;
    float absErr = std::fabs(err);

    if (absErr >= kDivergenceReanchorLinearMm && msSinceLastReanchor_ >= kDivergenceReanchorMinIntervalMs) {
      // 40mm is unambiguous -- a real drivetrain doesn't lag its own
      // commanded profile by 4cm during ordinary tracking, so this tier
      // acts on the very first sample past threshold (still rate-limited
      // by msSinceLastReanchor_ itself).
      pendingLinearReanchor_ = true;
      needLinearSolve_ = true;
      msSinceLastReanchor_ = 0;
    }
    // NO small-threshold mid-flight retarget tier (stakeholder 2026-07-18,
    // "plan the motion over the whole distance and finish on the spot"):
    // the old 5mm/3-tick streak tier re-solved the linear channel against
    // ordinary velocity-loop tracking lag -- and a re-solve mid-DECEL, from
    // nonzero velocity to a now-tiny-or-negative remaining distance, is
    // time-optimally an overshoot-then-REVERSE, which then lagged and
    // diverged again: the terminal +-100mm/s command ringing observed on
    // the sim wheel-speed trace (2026-07-18), with every re-solve also
    // resetting the trajectory clock so `trajDone` kept un-completing.
    // The plan is now solved ONCE and trusted; measured distance decides
    // COMPLETION (crossing / settle-epsilon) plus a forward-only top-up
    // from rest when the plant lands short (tick()'s terminal logic), and
    // the 40mm reanchor above stays as the gross-fault (genuine slip/
    // stall) recovery -- ordinary tracking lag never reaches it.
    return;
  }

  // kPivot -- NO mid-flight divergence correction at all (stakeholder
  // 2026-07-18, "plan once, finish on the spot", rotational half): the old
  // 0.3rad reanchor tier sat BELOW ordinary tracking lag at cruise (a
  // 4rad/s pivot with ~0.15s actuation lag runs ~0.6rad behind its plan),
  // so it fired every 60ms through any fast pivot, each reanchor
  // re-solving from measured state -- and a re-solve near the target from
  // full rate is time-optimally overshoot-then-REVERSE. Confirmed by
  // direct experiment (sim, 360deg pivot): above the threshold, a
  // decaying full-amplitude sign-flip limit cycle ending in the STOP_TIME
  // backstop (kTimeout); below it, a clean completion. The rotational
  // channel needs no correction tier: the heading PD (App::Pilot) is its
  // CONTINUOUS closer, the dwell gate completes on MEASURED heading, and
  // the STOP_TIME backstop bounds everything else.
  (void)thetaMeasRel;
  (void)plannedPositionSinceActivation;
}

EnqueueOutcome Executor::enqueue(const Cmd& cmd) {
  if (cmd.isDegenerate()) return EnqueueOutcome::kTrivial;

  if (cmd.replace) {
    if (queueCount_ > 0) {
      pushEvent(ring_[queueCount_ - 1].id, CompletionStatus::kSuperseded);
      ring_[queueCount_ - 1] = cmd;
      // 109-006 trigger (b)-tail: as (a) -- ring_[0] may itself be the
      // slot just replaced (queueCount_==1), so the active's own immediate
      // successor may have changed.
      maybeRetargetActiveForSuccessorChange();
      return EnqueueOutcome::kReplaced;
    }
    if (state_ != State::kIdle && activeValid_) {
      pushEvent(active_.id, CompletionStatus::kSuperseded);
      activate(cmd, /*retarget=*/true);
      return EnqueueOutcome::kReplaced;
    }
    // Nothing queued and nothing active -- replace has nothing to replace;
    // falls through to the fresh-enqueue path below.
  }

  // 109-006: a mid-decel enqueue while RAMP_TO_REST (empty queue, a TIMED
  // command coasting to rest with nothing behind it) is a moving-state
  // replan, not a "wait for full rest first" append -- retarget=true keeps
  // whatever velocity/acceleration seed the decelerating command still has
  // (activate() itself dispatches on the NEW cmd's own kind, same as any
  // other retarget=true activation).
  if ((state_ == State::kIdle || state_ == State::kRampToRest) && queueCount_ == 0) {
    activate(cmd, /*retarget=*/state_ == State::kRampToRest);
    return EnqueueOutcome::kAccepted;
  }

  if (queueCount_ >= kQueueDepth) return EnqueueOutcome::kFull;
  ring_[queueCount_++] = cmd;
  // 109-006 trigger (a): this enqueue may have just made `cmd` the
  // active's own immediate successor (an append to a previously-empty
  // ring, ring_[0] == cmd now) -- a no-op if it did not (appending behind
  // an already-nonempty ring never changes ring_[0]).
  maybeRetargetActiveForSuccessorChange();
  return EnqueueOutcome::kAccepted;
}

void Executor::flush() {
  for (uint8_t i = 0; i < queueCount_; ++i) {
    pushEvent(ring_[i].id, CompletionStatus::kFlushed);
  }
  queueCount_ = 0;

  if (state_ != State::kIdle && activeValid_) {
    pushEvent(active_.id, CompletionStatus::kFlushed);
  }
  active_ = Cmd{};
  activeValid_ = false;
  state_ = State::kIdle;
  needLinearSolve_ = false;
  needRotationalSolve_ = false;

  // 109-006: a flush abandons any in-flight boundary-velocity/divergence
  // bookkeeping too -- nothing left to carry a velocity or a pending
  // reanchor INTO once the ring and the active command are both gone.
  exitVelocity_ = 0.0f;
  linearFrameOffset_ = 0.0f;
  rotationalFrameOffset_ = 0.0f;
  pendingLinearReanchor_ = false;
  emergencyStopping_ = false;
}

void Executor::plan() {
  // At most ONE solve per call -- linear takes priority when both are
  // pending (arbitrary but fixed ordering; a fresh TIMED command with both
  // v_max and omega nonzero needs exactly two plan() calls to be fully
  // planned, per this file's own "Solve budget" doc comment). kArc/kPivot
  // each only ever request ONE of the two, so this ordering never affects
  // them. 109-006 priority within a channel's own branch: emergencyStopping_
  // (solve-failure safety net) first, then a pending divergence reanchor/
  // retarget (tick()'s own checkDivergence(), serviced here -- tick()
  // itself never solves), then the mode_-dependent normal path (unchanged
  // from 109-003/109-005, except kArc/kPivot's own position-control target
  // velocity is now exitVelocity_ rather than always 0 -- this ticket's own
  // boundary-velocity carry).
  if (needLinearSolve_) {
    needLinearSolve_ = false;

    if (emergencyStopping_) {
      if (linear_.solveToVelocity(0.0f, linearCeiling_)) linearElapsedS_ = 0.0f;
      else completeActive(CompletionStatus::kSolveFail);
      return;
    }

    // A divergence-triggered reanchor()/retarget() failing is NOT treated
    // as a fatal kSolveFail -- unlike a fresh/normal solve (below), which
    // means this command genuinely cannot be planned at all, a failed
    // divergence correction just means THIS PARTICULAR correction attempt
    // was infeasible (e.g. a measured velocity/position momentarily far
    // outside anything reachable -- a noise spike, or, in the sim system
    // tests, a deliberately unrealistic scripted "measured" signal that
    // outraces the channel's own configured limits). JerkTrajectory's own
    // solvePositionControl() only ever commits a solve into the held
    // trajectory on success (jerk_trajectory.cpp's own "temp-solve"
    // discipline) -- a failed reanchor()/retarget() leaves the PREVIOUS,
    // still-valid trajectory completely untouched, so silently declining
    // and letting checkDivergence() re-evaluate next tick is safe and
    // strictly better than tearing down the whole active command (and
    // flushing the rest of the queue) over a single bad correction
    // attempt.
    if (pendingLinearReanchor_) {
      pendingLinearReanchor_ = false;
      float internalPosition = measuredPathSinceActivation_ - linearFrameOffset_;
      if (linear_.reanchor(internalPosition, lastMeasuredVelocity_)) linearElapsedS_ = 0.0f;
      return;
    }

    bool ok;
    float linCeiling = linearCeiling_;
    float linPosTarget = effectiveDistance_;
    if (mode_ == Mode::kTimed) {
      ok = linear_.solveToVelocity(pendingLinearTarget_, linearCeiling_);
    } else {
      // kArc -- position-control solve to the effective distance (112-004:
      // always exactly cmd.distance, no overshoot adjustment -- see this
      // file's own "Distance completion" comment), ceilinged by the Cmd's
      // own requested vMax, carrying exitVelocity_ through the boundary (0
      // when there is no compatible successor -- this file's own
      // computeExitVelocity()). 112-004: the terminal straight-lead padding
      // that used to lengthen this solve target is DELETED -- the unified
      // completion rule's own |sErr| < distance_tol tolerance test replaces
      // the lead-then-crossing mechanism the padding existed to set up (see
      // this file's own "Unified completion" comment).
      linCeiling = (pendingLinearVMax_ != 0.0f)
                       ? std::min(std::fabs(pendingLinearVMax_), linearCeiling_)
                       : linearCeiling_;
      ok = linear_.solveToState(linPosTarget, exitVelocity_, linCeiling);
    }
    if (ok) {
      linearElapsedS_ = 0.0f;  // this channel's own clock restarts at its own solve
    } else if (mode_ != Mode::kTimed &&
               resolveFromRest(linear_, &linearElapsedS_, linPosTarget, linCeiling)) {
      // recovered from a stale-carried-state infeasibility -- see resolveFromRest()
    } else {
      completeActive(CompletionStatus::kSolveFail);
    }
    return;
  }
  if (needRotationalSolve_) {
    needRotationalSolve_ = false;

    if (emergencyStopping_) {
      if (rotational_.solveToVelocity(0.0f, rotationalCeiling_)) rotationalElapsedS_ = 0.0f;
      else completeActive(CompletionStatus::kSolveFail);
      return;
    }

    bool ok;
    if (mode_ == Mode::kTimed) {
      ok = rotational_.solveToVelocity(pendingRotationalTarget_, rotationalCeiling_);
    } else {
      // kPivot -- position-control solve directly to deltaHeading, carrying
      // exitVelocity_ through the boundary (pivot->pivot chains, same rule
      // in the rotational domain -- computeExitVelocity()'s own comment).
      ok = rotational_.solveToState(active_.deltaHeading, exitVelocity_, rotationalCeiling_);
    }
    if (ok) {
      rotationalElapsedS_ = 0.0f;
    } else if (mode_ != Mode::kTimed &&
               resolveFromRest(rotational_, &rotationalElapsedS_, active_.deltaHeading,
                               rotationalCeiling_)) {
      // recovered from a stale-carried-state infeasibility -- see resolveFromRest()
    } else {
      completeActive(CompletionStatus::kSolveFail);
    }
  }
}

// resolveFromRest -- recover a failed position-control solve by resetting
// the channel to rest and re-solving to `posTarget` at rest (exit velocity
// 0). Root cause it addresses (2026-07-18): a fresh command enqueued while
// the executor is still RAMP_TO_REST activates with retarget=true and seeds
// the solve from the channel's DECELERATING internal state; that carried
// (position, velocity, acceleration) can momentarily be infeasible for the
// new target, and Ruckig returns failure -- observed as a periodic
// SOLVE_FAIL across back-to-back pivots (every ~6th, with the pre-fail
// duration creeping up as the stale state accumulated). A reachable target
// is ALWAYS solvable from rest, so a reset-and-retry turns that transient
// infeasibility into a clean from-rest replan. kSolveFail (the caller's
// final else) then means genuinely unreachable even from rest -- a
// degenerate config, the only thing it should mean. The velocity
// discontinuity the reset introduces happens ONLY on the rare failure, never
// the nominal smooth-replan path (which keeps its carried velocity).
bool Executor::resolveFromRest(JerkTrajectory& chan, float* elapsed, float posTarget,
                               float ceiling) {
  chan.reset();
  if (!chan.solveToState(posTarget, 0.0f, ceiling)) return false;
  *elapsed = 0.0f;
  return true;
}

Executor::Twist Executor::tick(uint32_t dtMs, float measuredDistanceDelta,
                                float measuredHeadingAbs, float measuredHeadingLeadAbs) {
  Twist out;
  if (state_ == State::kIdle) return out;

  if (emergencyStopping_) {
    // 109-006 solve-failure safety net (completeActive()'s own kSolveFail
    // branch) -- sample-only, both channels, bypassing the normal kTimed/
    // kArc/kPivot dispatch and its own distance/dwell completion tests
    // entirely: this state exists ONLY to get both channels to rest before
    // handing off (activateNextOrIdle() pushes no event itself -- the ONE
    // kSolveFail event for the failed command was already pushed by
    // completeActive() before entering this state).
    activeElapsedMs_ += dtMs;
    linearElapsedS_ += static_cast<float>(dtMs) / 1000.0f;
    rotationalElapsedS_ += static_cast<float>(dtMs) / 1000.0f;
    JerkTrajectory::State linSample = linear_.sample(linearElapsedS_);
    JerkTrajectory::State rotSample = rotational_.sample(rotationalElapsedS_);
    out.v = linSample.velocity;
    out.omega = rotSample.velocity;
    bool atRest = std::fabs(linSample.velocity) < kLinearRestEpsilon &&
                  std::fabs(rotSample.velocity) < kRotationalRestEpsilon && !needLinearSolve_ &&
                  !needRotationalSolve_;
    if (atRest) {
      emergencyStopping_ = false;
      activateNextOrIdle();
    }
    return out;
  }

  activeElapsedMs_ += dtMs;
  float dtS = static_cast<float>(dtMs) / 1000.0f;  // [s]
  linearElapsedS_ += dtS;
  rotationalElapsedS_ += dtS;
  measuredPathSinceActivation_ += measuredDistanceDelta;

  if (!headingBaselineSet_) {
    lastMeasuredHeadingAbs_ = measuredHeadingAbs;
    unwrappedThetaRel_ = 0.0f;
    headingBaselineSet_ = true;
  }
  // Continuous (unwrapped) relative heading since activation -- see
  // `unwrappedThetaRel_`'s own doc comment (executor.h) for why a single
  // wrapAngle(measuredHeadingAbs - headingBaselineAbs_) is wrong once the
  // total rotation exceeds +-180deg. Each cycle's own step is always small
  // (bounded by one tick's worst-case rotation rate), so wrapAngle() on
  // JUST the step is safe even though the accumulated total is not.
  unwrappedThetaRel_ += wrapAngle(measuredHeadingAbs - lastMeasuredHeadingAbs_);
  lastMeasuredHeadingAbs_ = measuredHeadingAbs;
  float thetaMeasRel = unwrappedThetaRel_;

  // 109-010 locus 1: thetaMeasLeadRel is thetaMeasRel plus the SAME small
  // (measuredHeadingLeadAbs - measuredHeadingAbs) offset App::HeadingSource's
  // own headingLead() vs. heading() difference represents THIS cycle --
  // wrapAngle() is safe here (unlike the accumulator above) because this
  // offset is bounded by one cycle's own worst-case rotation rate times a
  // sub-second lead, never anywhere near +-180deg, so no separate unwrapped
  // accumulation is needed for it the way unwrappedThetaRel_ needs one for
  // the raw signal across a whole multi-turn command.
  float thetaMeasLeadRel = thetaMeasRel + wrapAngle(measuredHeadingLeadAbs - measuredHeadingAbs);

  if (mode_ == Mode::kTimed) {
    JerkTrajectory::State linSample = linear_.sample(linearElapsedS_);
    JerkTrajectory::State rotSample = rotational_.sample(rotationalElapsedS_);
    out.v = linSample.velocity;
    out.omega = rotSample.velocity;

    if (state_ == State::kRunning) {
      float remainingMs = active_.time - static_cast<float>(activeElapsedMs_);
      float stopLinMs = estimateStopDuration(linSample.velocity, aDecelLinear_, jerkLinear_) * 1000.0f;
      float stopRotMs =
          estimateStopDuration(rotSample.velocity, aDecelRotational_, jerkRotational_) * 1000.0f;
      float stopNeededMs = (stopLinMs > stopRotMs) ? stopLinMs : stopRotMs;

      if (remainingMs <= stopNeededMs) {
        pendingLinearTarget_ = 0.0f;
        pendingRotationalTarget_ = 0.0f;
        needLinearSolve_ = true;
        needRotationalSolve_ = true;
        state_ = State::kRampToRest;
      }
    }

    if (state_ == State::kRampToRest) {
      bool atRest = std::fabs(linSample.velocity) < kLinearRestEpsilon &&
                    std::fabs(rotSample.velocity) < kRotationalRestEpsilon && !needLinearSolve_ &&
                    !needRotationalSolve_;
      if (atRest) completeActive(CompletionStatus::kDone);
    }

    prevThetaMeasRel_ = thetaMeasRel;
    return out;
  }

  // -- kArc / kPivot (109-005) --
  JerkTrajectory::State linSample = linear_.sample(linearElapsedS_);
  JerkTrajectory::State rotSample = rotational_.sample(rotationalElapsedS_);

  bool headingContent = (active_.deltaHeading != 0.0f);
  float thetaRef = 0.0f;
  float omegaFf = 0.0f;

  // 109-006: this tick's own dominant-channel sample, cached for
  // activateNextOrIdle()'s own velocity-continuous handoff -- see that
  // method's and these fields' own executor.h doc comments. Position
  // reads add back linearFrameOffset_/rotationalFrameOffset_ (a divergence
  // retarget() may have rebased the channel's own internal frame to 0
  // one or more times since activation -- checkDivergence()'s own
  // comment) to stay in this command's own since-activation frame.
  completionLinearVelocity_ = linSample.velocity;
  completionLinearAcceleration_ = linSample.acceleration;
  completionRotationalVelocity_ = rotSample.velocity;
  completionRotationalAcceleration_ = rotSample.acceleration;

  // plannedPositionSinceActivation -- the dominant channel's own sampled
  // position, corrected for any divergence-retarget frame rebase(s) so far
  // (linearFrameOffset_/rotationalFrameOffset_) -- this command's own
  // since-activation frame. Feeds BOTH thetaRef (below) and
  // checkDivergence()'s own comparison (this file's own doc comments).
  float plannedPositionSinceActivation =
      (mode_ == Mode::kArc) ? (linearFrameOffset_ + linSample.position)
                             : (rotationalFrameOffset_ + rotSample.position);

  // 112-001: 109-010 locus 2 (the peek(elapsed + plan_lead) wheel-velocity
  // reference, plus the pivot-only kPivotOvershootLeadSlope extra lead) is
  // DELETED -- F2's jerk-warp bug: peeking at `elapsed + lead` evaluates
  // the reference at `2t` during the ramp-in, doubling commanded
  // acceleration and quadrupling commanded jerk right at Move activation.
  // out.v/omegaFf are now the same-instant sample() result already
  // computed above (linSample/rotSample) -- honest sampling, no peek()
  // ahead. thetaRef/plannedPositionSinceActivation were never touched by
  // the deleted lead (they always used the current elapsed sample) and are
  // unaffected by this deletion. See motion/DESIGN.md sec 2c for the
  // updated locus-2 write-up.
  if (mode_ == Mode::kArc) {
    out.v = linSample.velocity;
    out.aRef = linSample.acceleration;  // 112-002
    thetaRef = headingRatioPerMm_ * plannedPositionSinceActivation;
    omegaFf = headingRatioPerMm_ * linSample.velocity;
    out.alphaRef = headingRatioPerMm_ * linSample.acceleration;  // 112-002
    // 112-003: the linear channel's own since-activation reference/
    // measured pair, for App::Pilot's bounded position-feedback trim --
    // see Twist::sRef/sMeas's own doc comment. Left at Twist{}'s 0/0
    // default for kPivot/kTimed (the else branch below, and the kTimed
    // early return above, neither touch these two fields).
    out.sRef = plannedPositionSinceActivation;
    out.sMeas = measuredPathSinceActivation_;
  } else {
    out.v = 0.0f;
    out.aRef = 0.0f;  // 112-002: kPivot has no linear channel
    thetaRef = plannedPositionSinceActivation;
    omegaFf = rotSample.velocity;
    out.alphaRef = rotSample.acceleration;  // 112-002
  }

  // -- 112-004: unified completion rule (see this file's own "Unified
  // completion" comment) -- replaces the old distanceDone/top-up/dwell-EMA
  // patch stack entirely. sErr is the linear channel's own error against
  // the TARGET (effectiveDistance_, now always == cmd.distance), mirroring
  // thetaErr's own "target minus measured" shape one channel over -- 0 for
  // kPivot (no linear channel, mode_ != kArc guards it out below, matching
  // Twist::sRef/sMeas's own 0/0-for-kPivot pattern).
  float sErr = (mode_ == Mode::kArc) ? (effectiveDistance_ - measuredPathSinceActivation_) : 0.0f;

  // sOk -- the unified rule's own linear tolerance test, computed here
  // (ahead of the out.* assignment block below) so it can double as
  // Twist::withinDistanceTolerance -- App::Pilot's own terminal-decel gate
  // for the linear trim, mirroring headingActive/withinTolerance's own
  // shape exactly (see this file's own "Unified completion" comment and
  // pilot.cpp's own trim-gating comment). Trivially true for kPivot (no
  // linear channel, mirrors sErr's own 0-for-kPivot value).
  bool sOk = (mode_ != Mode::kArc) || (std::fabs(sErr) < distanceTol_);

  float thetaErr = active_.deltaHeading - thetaMeasRel;
  float thetaRate = (dtS > 0.0f) ? (thetaMeasRel - prevThetaMeasRel_) / dtS : 0.0f;

  // 112-004: withinTol now reads the raw thetaErr -- the predicted-state
  // `thetaErrLead`/`terminal_lead` (109-010 locus 3) stand-in this used to
  // read is deleted (see this file's own "Unified completion" comment).
  // thetaErr itself stays UNLED and keeps feeding checkDivergence() (via
  // thetaMeasRel directly, not thetaErr) and the crossedTarget sign-flip
  // test below -- unaffected by this deletion.
  bool withinTol = std::fabs(thetaErr) < headingDwellTol_;

  // thetaOk -- the unified rule's own angular error test, trivially
  // satisfied for a non-heading-bearing command (deltaHeading == 0) --
  // SUC-005's own Main Flow states the straight's own completion rule as
  // `|s_err| < distance_tol` alone, with no theta_err term at all.
  bool thetaOk = !headingContent || withinTol;

  // profileElapsed -- "t >= duration + margin" (sprint.md's own unified-rule
  // wording), margin folded to 0: the AND'd sOk/thetaOk tolerance tests
  // already prevent premature completion mid-cruise (a fast-moving channel
  // essentially never re-enters a tight tolerance band around the target
  // before its own profile ends), so a separate nonzero margin buys nothing
  // this implementation's own testing found a need for -- see this ticket's
  // completion notes. `dominantDurationS > 0.0f` guards the pre-first-solve
  // window exactly the way the deleted top-up trigger used to (a freshly
  // activated command's own channel has no committed plan yet, duration 0,
  // which would otherwise read as "profile already elapsed" on the very
  // first tick and short-circuit completion before any solve has run).
  float dominantDurationS = (mode_ == Mode::kPivot) ? rotational_.duration() : linear_.duration();
  float dominantElapsedS = (mode_ == Mode::kPivot) ? rotationalElapsedS_ : linearElapsedS_;
  bool profileElapsed = dominantDurationS > 0.0f && dominantElapsedS >= dominantDurationS;

  // 109-006 trigger (c): detect-only here (tick() never solves) -- sets
  // pendingLinearReanchor_ for plan() to service next. See checkDivergence()'s
  // own doc comment. UNTOUCHED by 112-004 -- the 40mm gross-divergence
  // reanchor tier is a DIFFERENT mechanism from the deleted terminal patch
  // stack (see this file's own "Distance completion" comment).
  checkDivergence(dtS, measuredDistanceDelta, thetaMeasRel, thetaRate, plannedPositionSinceActivation);

  // terminalDecel -- gate the heading PD off once the command has ALREADY
  // reached the dwell gate's own tolerance test (executor.h's own
  // "Terminal-decel PD gate" comment): an ERROR-based test, not a fixed
  // time-before-planned-completion window -- a time-based gate was this
  // ticket's own FIRST implementation and was caught, by this ticket's own
  // sim system test (test_heading_source.py), disabling the PD exactly
  // when a real (non-ideal, laggy) plant still had a large residual error
  // left to correct, latching a ~6deg overshoot the PD was never given the
  // chance to close. Gating on "already within tolerance" instead means
  // the PD keeps correcting for as long as it is actually needed, and
  // only steps back once further correction would just be chasing noise
  // around an already-good landing -- exactly the "no commanded reversal
  // NEAR TARGET" intent, read as a distance-to-target condition rather
  // than a time-to-planned-completion one. 112-004: the gate's own RATE
  // half is deleted along with dwellRateFilt_/headingDwellRate_ -- see this
  // file's own "Unified completion" comment.
  bool terminalDecel = withinTol;

  out.omega = omegaFf;
  out.thetaRef = thetaRef;
  out.thetaMeas = thetaMeasRel;
  out.thetaMeasLead = thetaMeasLeadRel;  // 109-010 locus 1, App::Pilot's PD error term
  out.headingActive = headingContent && !terminalDecel;
  out.withinTolerance = withinTol;       // diagnostic mirror -- see Twist::withinTolerance's own doc comment
  out.withinDistanceTolerance = sOk;     // App::Pilot's own linear-trim terminal-decel gate (see Twist's own doc comment)
  out.omegaDes = out.headingActive ? omegaFf : 0.0f;

  // carryingRotationalVelocity -- true iff THIS command's own planned exit
  // speed (`exitVelocity_`, set by computeExitVelocity()/
  // maybeRetargetActiveForSuccessorChange()) is nonzero, i.e. a genuine
  // same-sign chain (pivot->pivot, or a heading-bearing arc->arc) that is
  // DELIBERATELY still moving at handoff (the successor's own PD
  // immediately takes over the still-rotating channel, per SUC-003's
  // boundary-velocity carry). 109-009 fix, PRESERVED VERBATIM by 112-004
  // (this ticket's own guardrail): the dwell HOLD (below) is only
  // skippable for THIS case -- the original (pre-109-009) code skipped it
  // for every "chained, non-terminal" command (any command with ANY
  // successor queued, `queueCount_ > 0`), which wrongly included a pivot
  // chained into a plain DISTANCE leg (every TOUR_1/2 turn, via
  // run_tour()'s one-leg lookahead) or an opposite-sign/incompatible pivot
  // -- neither carries any velocity forward (`exitVelocity_` is exactly 0
  // for both, computeExitVelocity()'s own contract), so nothing downstream
  // corrects the residual angular momentum a bare tolerance-crossing
  // sample can still have. That bled into extra, uncorrected post-handoff
  // rotation the sim tour-closure gate (109-009) measured directly against
  // ground truth (observed up to ~3.5deg with an IDEAL/noiseless OTOS --
  // not sensor error, a real completion-gate bug). Requiring the FULL
  // dwell hold whenever there is no velocity to carry -- exactly the
  // terminal-command rule -- costs at most one `headingDwellHoldS_` window
  // (150ms) per turn and guarantees the heading has actually stopped
  // moving before handoff, the same way a real driver does not start
  // driving straight while still spinning. Gated on headingContent (a pure
  // kArc straight leg, deltaHeading==0, is never in this branch -- it has
  // no rotational channel to carry a velocity ON, even if its own LINEAR
  // exitVelocity_ happens to be nonzero from a same-sign arc->arc chain).
  bool carryingRotationalVelocity = headingContent && (exitVelocity_ != 0.0f);

  if (carryingRotationalVelocity) {
    // Carrying a rotational exit velocity into a compatible successor: no
    // HOLD needed (the successor's PD takes over the still-moving channel
    // immediately), but a bare magnitude-band ("withinTol") test can
    // straddle the entire tolerance window between two consecutive samples
    // at cruise rate (e.g. 4rad/s * 40ms ~= 9deg, far larger than
    // headingDwellTol_'s 0.5deg) and never land inside it -- not
    // theoretical, this is exactly what motion_executor_harness.cpp's own
    // Scenario 9 (chained pivot->pivot) hit once thetaMeasRel's own unwrap
    // was fixed to be continuous (109-009's own wrap fix, see
    // `unwrappedThetaRel_`'s doc comment) and could no longer rely on the
    // OLD wrap bug's incidental (and semantically meaningless) second
    // zero-crossing after a spurious 2*pi discontinuity. `crossedTarget`
    // (thetaErr's sign flipping since last cycle) catches the case a
    // sample stepped clean over the tolerance band -- there is no "settle"
    // to wait for, since the whole point of carrying velocity is to keep
    // moving into the successor. PRESERVED VERBATIM (112-004's own
    // guardrail): no STOP_TIME backstop in this branch, exactly as before
    // this ticket.
    float prevThetaErr = active_.deltaHeading - prevThetaMeasRel_;
    bool crossedTarget = (thetaErr <= 0.0f) != (prevThetaErr <= 0.0f);
    if (sOk && (withinTol || crossedTarget)) {
      completeActive(CompletionStatus::kDone);
    }
  } else {
    // Not carrying (terminal, or chained into a successor that does not
    // carry a boundary velocity through): the unified rule -- `profileElapsed
    // AND sOk AND thetaOk`, held continuously for headingDwellHoldS_
    // (arrive_dwell) via a plain hard-reset-on-any-miss counter (NOT the
    // deleted 109-009 leaky/decaying counter -- see executor.h's own
    // "Unified completion" comment for why a hard reset is safe again now
    // that the noisy RATE test that motivated the leaky counter is gone),
    // plus the SAME stopTimeBackstopMs() timeout backstop this branch
    // (both its former headingContent and non-heading incarnations) has
    // always had, so a persistent oscillation or a stuck measurement can
        // never wedge the executor open forever.
    if (profileElapsed && sOk && thetaOk) {
      dwellHeldMs_ += dtMs;
    } else {
      dwellHeldMs_ = 0;
    }
    uint32_t holdNeededMs = static_cast<uint32_t>(headingDwellHoldS_ * 1000.0f);
    if (profileElapsed && sOk && thetaOk && dwellHeldMs_ >= holdNeededMs) {
      completeActive(CompletionStatus::kDone);
    } else if (activeElapsedMs_ >= stopTimeBackstopMs()) {
      completeActive(CompletionStatus::kTimeout);
    }
  }

  prevThetaMeasRel_ = thetaMeasRel;
  return out;
}

}  // namespace Motion
