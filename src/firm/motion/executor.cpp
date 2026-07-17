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

  state_ = State::kRunning;
}

void Executor::activateNextOrIdle() {
  if (queueCount_ > 0) {
    Cmd next = ring_[0];
    for (uint8_t i = 1; i < queueCount_; ++i) ring_[i - 1] = ring_[i];
    --queueCount_;
    activate(next, /*retarget=*/false);
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
  activateNextOrIdle();
}

uint32_t Executor::stopTimeBackstopMs() const {
  float dominantDurationS = (mode_ == Mode::kPivot) ? rotational_.duration() : linear_.duration();
  return static_cast<uint32_t>(
      (dominantDurationS * kStopTimeBackstopFactor + kStopTimeBackstopMarginS) * 1000.0f);
}

EnqueueOutcome Executor::enqueue(const Cmd& cmd) {
  if (cmd.isDegenerate()) return EnqueueOutcome::kTrivial;

  if (cmd.replace) {
    if (queueCount_ > 0) {
      pushEvent(ring_[queueCount_ - 1].id, CompletionStatus::kSuperseded);
      ring_[queueCount_ - 1] = cmd;
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

  if (state_ == State::kIdle && queueCount_ == 0) {
    activate(cmd, /*retarget=*/false);
    return EnqueueOutcome::kAccepted;
  }

  if (queueCount_ >= kQueueDepth) return EnqueueOutcome::kFull;
  ring_[queueCount_++] = cmd;
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
}

void Executor::plan() {
  // At most ONE solve per call -- linear takes priority when both are
  // pending (arbitrary but fixed ordering; a fresh TIMED command with both
  // v_max and omega nonzero needs exactly two plan() calls to be fully
  // planned, per this file's own "Solve budget" doc comment). kArc/kPivot
  // each only ever request ONE of the two, so this ordering never affects
  // them.
  if (needLinearSolve_) {
    needLinearSolve_ = false;
    bool ok;
    if (mode_ == Mode::kTimed) {
      ok = linear_.solveToVelocity(pendingLinearTarget_, linearCeiling_);
    } else {
      // kArc -- position-control solve to the (possibly overshoot-adjusted)
      // effective distance, ceilinged by the Cmd's own requested vMax.
      float ceiling = (pendingLinearVMax_ != 0.0f)
                          ? std::min(std::fabs(pendingLinearVMax_), linearCeiling_)
                          : linearCeiling_;
      ok = linear_.solveToRest(effectiveDistance_, ceiling);
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
    bool ok;
    if (mode_ == Mode::kTimed) {
      ok = rotational_.solveToVelocity(pendingRotationalTarget_, rotationalCeiling_);
    } else {
      // kPivot -- position-control solve directly to deltaHeading.
      ok = rotational_.solveToRest(active_.deltaHeading, rotationalCeiling_);
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

  if (mode_ == Mode::kArc) {
    out.v = linSample.velocity;
    thetaRef = headingRatioPerMm_ * linSample.position;
    omegaFf = headingRatioPerMm_ * linSample.velocity;
  } else {
    out.v = 0.0f;
    thetaRef = rotSample.position;
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
