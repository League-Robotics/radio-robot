// executor.cpp -- Motion::Executor implementation. See executor.h's file
// header for the module's boundary, the three Cmd modes (kTimed/kArc/
// kPivot), the heading-feedforward/PD-cascade split with App::Pilot, the
// terminal-decel PD gate, and the distance/dwell completion criteria.
#include "motion/executor.h"

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
constexpr float kStopTimeBackstopMarginS = 1.0f;  // [s]

// 109-006: exit-speed-change trigger (a)/(b)-tail thresholds -- verbatim
// per the sprint issue's own "Replan triggers" section ("exit speed changes
// >1 mm/s" / rotational domain's own analogous >0.02rad/s).
constexpr float kExitVelocityLinearThreshold = 1.0f;      // [mm/s]
constexpr float kExitVelocityRotationalThreshold = 0.02f; // [rad/s]

// 109-006: divergence-trigger (c) thresholds -- verbatim per the sprint
// issue's own "Replan triggers" section ("old thresholds... 5mm retarget /
// 40mm reanchor linear, 0.3rad reanchor rotational, 60ms min interval").
constexpr float kDivergenceRetargetLinearMm = 5.0f;         // [mm]
constexpr float kDivergenceReanchorLinearMm = 40.0f;        // [mm]
constexpr float kDivergenceReanchorRotationalRad = 0.3f;    // [rad]
constexpr uint32_t kDivergenceReanchorMinIntervalMs = 60;   // [ms]

// kDivergenceRetargetStreakTicks -- 109-006's own anti-transient guard for
// the 5mm linear retarget tier (checkDivergence()'s own doc comment): the
// number of CONSECUTIVE ticks the linear channel must stay past the 5mm
// threshold before a retarget actually fires, distinguishing a momentary
// velocity-PID ramp-lag blip (self-resolving within a tick or two) from a
// genuinely sustained divergence. Not part of the sprint issue's own
// verbatim threshold table -- added during this ticket's own
// implementation after test_heading_source.py's ideal-plant coupled-arc
// scenario caught a real accuracy regression from reacting to single-
// sample transients (this file's own checkDivergence() doc comment).
constexpr uint8_t kDivergenceRetargetStreakTicks = 3;

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
  headingDwellRate_ = config.heading_dwell_rate;
  headingDwellHoldS_ = config.arrive_dwell;
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
  headingBaselineAbs_ = 0.0f;
  prevThetaMeasRel_ = 0.0f;
  dwellHeldMs_ = 0;
  headingRatioPerMm_ = 0.0f;
  effectiveDistance_ = cmd.distance;

  // 109-006: a fresh command starts its own fresh divergence/frame
  // bookkeeping -- see each field's own executor.h doc comment.
  linearFrameOffset_ = 0.0f;
  rotationalFrameOffset_ = 0.0f;
  pendingLinearReanchor_ = false;
  pendingLinearRetarget_ = false;
  pendingRotationalReanchor_ = false;
  linearRetargetStreak_ = 0;
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
    // A TIMED command carries no linear-distance progress/overshoot
    // semantics -- any carry from a PRECEDING kArc command is dropped here
    // (see this file header's "Distance completion" comment: only the
    // VERY NEXT activation can consume a pending carry, and only if it is
    // itself a same-sign kArc command).
    pendingOvershoot_ = 0.0f;
  } else if (cmd.isPivot()) {
    mode_ = Mode::kPivot;
    pendingOvershoot_ = 0.0f;  // a pivot produces/consumes no linear overshoot
    needLinearSolve_ = false;
    needRotationalSolve_ = true;
  } else {
    // kArc: distance != 0, not timed. Dominant channel is linear.
    mode_ = Mode::kArc;
    headingRatioPerMm_ = cmd.deltaHeading / cmd.distance;

    if (pendingOvershoot_ != 0.0f &&
        ((pendingOvershoot_ > 0.0f) == (cmd.distance > 0.0f))) {
      effectiveDistance_ = cmd.distance - pendingOvershoot_;
      // Never flip the direction of travel because of a carried-in
      // overshoot -- clamp to a small same-sign residual instead (a
      // documented v1 edge case: an overshoot larger than the successor's
      // own requested distance is rare and, when it happens, "drive a
      // token amount further in the same direction" is a safer failure
      // mode than silently reversing).
      bool sameSign = (effectiveDistance_ > 0.0f) == (cmd.distance > 0.0f);
      if (!sameSign || effectiveDistance_ == 0.0f) {
        effectiveDistance_ = (cmd.distance > 0.0f) ? 1.0f : -1.0f;
      }
    }
    pendingOvershoot_ = 0.0f;

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
  if (mode_ == Mode::kArc) {
    // Signed remainder carried into a same-sign successor -- see this
    // file's own "Distance completion" comment. Only meaningful on a real
    // DONE (a kTimeout/kSolveFail completion means the command never
    // reached its own distance criterion in the first place).
    pendingOvershoot_ = (status == CompletionStatus::kDone)
                            ? (measuredPathSinceActivation_ - effectiveDistance_)
                            : 0.0f;
  }
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
    pendingLinearRetarget_ = false;
    pendingRotationalReanchor_ = false;
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
  lastThetaMeasRel_ = thetaMeasRel;
  lastThetaRate_ = thetaRate;
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
      linearRetargetStreak_ = 0;
      pendingLinearReanchor_ = true;
      pendingLinearRetarget_ = false;
      needLinearSolve_ = true;
      msSinceLastReanchor_ = 0;
    } else if (absErr >= kDivergenceRetargetLinearMm) {
      // 5mm is NOT unambiguous -- ordinary velocity-PID tracking lag during
      // a command's own ramp-up/ramp-down routinely produces a brief few-mm
      // gap between the encoder and the Ruckig-planned position that
      // self-resolves within a tick or two as the wheel catches up to the
      // commanded profile (expected, not a fault). Reacting to a single
      // momentary sample here was this ticket's own first implementation
      // and was caught by test_heading_source.py's own ideal-plant
      // coupled-arc scenario: each transient ramp-lag blip "gave up
      // ground" (rebased the frame down to the momentarily-lagging
      // measured value) with nothing to claw it back, compounding into a
      // multi-degree heading undershoot by completion. Requiring the
      // divergence to PERSIST for kDivergenceRetargetStreakTicks
      // consecutive ticks (~kDivergenceRetargetStreakTicks*40ms) before
      // acting distinguishes a transient tracking lag (resolves within a
      // tick or two, streak resets) from a genuine, sustained divergence
      // (a real fault/slip) worth correcting.
      ++linearRetargetStreak_;
      if (linearRetargetStreak_ >= kDivergenceRetargetStreakTicks) {
        pendingLinearRetarget_ = true;
        needLinearSolve_ = true;
      }
    } else {
      linearRetargetStreak_ = 0;
    }
    return;
  }

  // kPivot -- reanchor-only, see this method's own executor.h doc comment.
  float err = thetaMeasRel - plannedPositionSinceActivation;
  if (std::fabs(err) >= kDivergenceReanchorRotationalRad &&
      msSinceLastReanchor_ >= kDivergenceReanchorMinIntervalMs) {
    pendingRotationalReanchor_ = true;
    needRotationalSolve_ = true;
    msSinceLastReanchor_ = 0;
  }
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
  pendingOvershoot_ = 0.0f;  // a flush abandons any in-flight carry too

  // 109-006: a flush abandons any in-flight boundary-velocity/divergence
  // bookkeeping too -- nothing left to carry a velocity or a pending
  // reanchor/retarget INTO once the ring and the active command are both
  // gone.
  exitVelocity_ = 0.0f;
  linearFrameOffset_ = 0.0f;
  rotationalFrameOffset_ = 0.0f;
  pendingLinearReanchor_ = false;
  pendingLinearRetarget_ = false;
  pendingRotationalReanchor_ = false;
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
    if (pendingLinearRetarget_) {
      pendingLinearRetarget_ = false;
      // newRemaining is computed relative to the MEASURED position (the
      // whole point of a position-target correction) -- retarget() itself
      // still seeds velocity/acceleration from the channel's OWN
      // remembered state (its own contract, jerk_trajectory.h), never
      // measured, so this stays within the "never seed a solve from a
      // measured observation" invariant for velocity/acceleration while
      // still correcting the POSITION target. Since newRemaining was
      // computed against measuredPathSinceActivation_, the new frame's own
      // origin (position 0, post-rebase) now REPRESENTS
      // measuredPathSinceActivation_ in the command's own since-activation
      // terms -- linearFrameOffset_ is therefore SET (not accumulated) to
      // measuredPathSinceActivation_, not the channel's own pre-rebase
      // position (which is exactly the divergent value this correction
      // exists to stop trusting for position bookkeeping). Getting this
      // backwards (bumping by the channel's own position instead) was
      // this ticket's own first implementation and was caught by
      // test_heading_source.py's own ideal-plant coupled-arc scenario:
      // it silently reintroduced the SAME divergence into thetaRef every
      // retarget, undershooting the commanded heading by more than the
      // dwell tolerance.
      float newRemaining = effectiveDistance_ - measuredPathSinceActivation_;
      if (linear_.retarget(newRemaining)) {
        linearFrameOffset_ = measuredPathSinceActivation_;
        linearElapsedS_ = 0.0f;
      }
      return;
    }

    bool ok;
    if (mode_ == Mode::kTimed) {
      ok = linear_.solveToVelocity(pendingLinearTarget_, linearCeiling_);
    } else {
      // kArc -- position-control solve to the (possibly overshoot-adjusted)
      // effective distance, ceilinged by the Cmd's own requested vMax,
      // carrying exitVelocity_ through the boundary (0 when there is no
      // compatible successor -- this file's own computeExitVelocity()).
      float ceiling = (pendingLinearVMax_ != 0.0f)
                          ? std::min(std::fabs(pendingLinearVMax_), linearCeiling_)
                          : linearCeiling_;
      ok = linear_.solveToState(effectiveDistance_, exitVelocity_, ceiling);
    }
    if (ok) {
      linearElapsedS_ = 0.0f;  // this channel's own clock restarts at its own solve
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

    // See the linear branch's own comment -- a failed divergence reanchor
    // is not fatal.
    if (pendingRotationalReanchor_) {
      pendingRotationalReanchor_ = false;
      // No small-threshold rotational retarget tier, hence no rotational
      // frame-offset bump here -- see checkDivergence()'s own comment.
      float internalPosition = lastThetaMeasRel_ - rotationalFrameOffset_;
      if (rotational_.reanchor(internalPosition, lastThetaRate_)) rotationalElapsedS_ = 0.0f;
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
    } else {
      completeActive(CompletionStatus::kSolveFail);
    }
  }
}

Executor::Twist Executor::tick(uint32_t dtMs, float measuredDistanceDelta,
                                float measuredHeadingAbs) {
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
    headingBaselineAbs_ = measuredHeadingAbs;
    headingBaselineSet_ = true;
  }
  float thetaMeasRel = wrapAngle(measuredHeadingAbs - headingBaselineAbs_);

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

  if (mode_ == Mode::kArc) {
    out.v = linSample.velocity;
    thetaRef = headingRatioPerMm_ * plannedPositionSinceActivation;
    omegaFf = headingRatioPerMm_ * linSample.velocity;
  } else {
    out.v = 0.0f;
    thetaRef = plannedPositionSinceActivation;
    omegaFf = rotSample.velocity;
  }

  // -- Completion + the terminal-decel PD gate (both need the SAME
  // measured-error test, computed once here) --
  bool distanceDone = (mode_ == Mode::kPivot) ||
                       (std::fabs(measuredPathSinceActivation_) >= std::fabs(effectiveDistance_));
  bool isTerminalCmd = (queueCount_ == 0);

  float thetaErr = active_.deltaHeading - thetaMeasRel;
  float thetaRate = (dtS > 0.0f) ? (thetaMeasRel - prevThetaMeasRel_) / dtS : 0.0f;
  bool withinTol = std::fabs(thetaErr) < headingDwellTol_;
  bool withinRate = std::fabs(thetaRate) < headingDwellRate_;

  // 109-006 trigger (c): detect-only here (tick() never solves) -- sets
  // pendingLinear{Reanchor,Retarget}_/pendingRotationalReanchor_ for
  // plan() to service next. See checkDivergence()'s own doc comment.
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
  // than a time-to-planned-completion one.
  bool terminalDecel = withinTol && withinRate;

  out.omega = omegaFf;
  out.thetaRef = thetaRef;
  out.thetaMeas = thetaMeasRel;
  out.headingActive = headingContent && !terminalDecel;
  out.omegaDes = out.headingActive ? omegaFf : 0.0f;

  if (headingContent) {
    if (isTerminalCmd) {
      if (distanceDone && withinTol && withinRate) {
        dwellHeldMs_ += dtMs;
      } else {
        dwellHeldMs_ = 0;
      }
      uint32_t holdNeededMs = static_cast<uint32_t>(headingDwellHoldS_ * 1000.0f);
      if (distanceDone && dwellHeldMs_ >= holdNeededMs) {
        completeActive(CompletionStatus::kDone);
      } else if (activeElapsedMs_ >= stopTimeBackstopMs()) {
        completeActive(CompletionStatus::kTimeout);
      }
    } else {
      // Chained, non-terminal: accurate handoff, no dwell hold required.
      if (distanceDone && withinTol) completeActive(CompletionStatus::kDone);
    }
  } else {
    // No heading content -- a plain kArc straight leg (deltaHeading == 0).
    bool trajDone = linearElapsedS_ >= linear_.duration();
    if (isTerminalCmd) {
      if (distanceDone && trajDone) completeActive(CompletionStatus::kDone);
    } else if (distanceDone) {
      completeActive(CompletionStatus::kDone);
    }
  }

  prevThetaMeasRel_ = thetaMeasRel;
  return out;
}

}  // namespace Motion
