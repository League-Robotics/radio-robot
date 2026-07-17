// executor.cpp -- Motion::Executor implementation. See executor.h's file
// header for the module's boundary, the 109-003 TIMED/DISTANCE scope
// split, and the deadline-driven RAMP_TO_REST design.
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
  if (!retarget) {
    linear_.reset();
    rotational_.reset();
  }
  pendingLinearTarget_ = cmd.vMax;
  pendingRotationalTarget_ = cmd.omega;
  needLinearSolve_ = true;
  needRotationalSolve_ = true;
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

EnqueueOutcome Executor::enqueue(const Cmd& cmd) {
  if (cmd.isDegenerate()) return EnqueueOutcome::kTrivial;
  if (!cmd.isTimed()) return EnqueueOutcome::kUnimplemented;  // DISTANCE mode -- ticket 005

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
}

void Executor::plan() {
  // At most ONE solve per call -- linear takes priority when both are
  // pending (arbitrary but fixed ordering; a fresh TIMED command with both
  // v_max and omega nonzero needs exactly two plan() calls to be fully
  // planned, per this file's own "Solve budget" doc comment).
  if (needLinearSolve_) {
    needLinearSolve_ = false;
    if (linear_.solveToVelocity(pendingLinearTarget_, linearCeiling_)) {
      linearElapsedS_ = 0.0f;  // this channel's own clock restarts at its own solve
    } else {
      pushEvent(active_.id, CompletionStatus::kSolveFail);
      activateNextOrIdle();
    }
    return;
  }
  if (needRotationalSolve_) {
    needRotationalSolve_ = false;
    if (rotational_.solveToVelocity(pendingRotationalTarget_, rotationalCeiling_)) {
      rotationalElapsedS_ = 0.0f;
    } else {
      pushEvent(active_.id, CompletionStatus::kSolveFail);
      activateNextOrIdle();
    }
  }
}

Executor::Twist Executor::tick(uint32_t dtMs) {
  Twist out;
  if (state_ == State::kIdle) return out;

  activeElapsedMs_ += dtMs;
  float dtS = static_cast<float>(dtMs) / 1000.0f;  // [s]
  linearElapsedS_ += dtS;
  rotationalElapsedS_ += dtS;

  JerkTrajectory::State linSample = linear_.sample(linearElapsedS_);
  JerkTrajectory::State rotSample = rotational_.sample(rotationalElapsedS_);
  out.v = linSample.velocity;
  out.omega = rotSample.velocity;

  if (state_ == State::kRunning && active_.isTimed()) {
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
    if (atRest) {
      pushEvent(active_.id, CompletionStatus::kDone);
      activateNextOrIdle();
    }
  }

  return out;
}

}  // namespace Motion
