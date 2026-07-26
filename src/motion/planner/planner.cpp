#include "planner.h"

#include <algorithm>
#include <cmath>

#include "profile.h"

namespace Motion {

namespace {

// Completion epsilons -- float-measurement noise floors, NOT motion-margin
// constants (the profile's terminal step lands exactly; these only absorb
// last-ulp rounding in the traveled-distance re-measurement).
constexpr float kDoneEpsilonLinear = 1e-3f;   // [mm]
constexpr float kDoneEpsilonAngular = 1e-5f;  // [rad]

float sign(float value) { return value < 0.0f ? -1.0f : 1.0f; }

// Signed one-axis ramp for Time/Wheels Moves: hold toward `cruise`, and
// once the remaining ticks are just enough to reach `boundary` at the
// decel ceiling, step toward it.
float timedRamp(float previous, float cruise, float boundary,
                float accelStep, float decelStep, float ticksLeft) {
  const float toBoundary = std::fabs(previous - boundary);
  const float stepsNeeded =
      decelStep > 0.0f ? std::ceil(toBoundary / decelStep) : 0.0f;
  if (ticksLeft <= stepsNeeded) {
    const float step = std::min(decelStep, toBoundary);
    return previous + (boundary > previous ? step : -step);
  }
  if (cruise > previous) return std::min(cruise, previous + accelStep);
  return std::max(cruise, previous - std::max(accelStep, decelStep));
}

}  // namespace

Planner::Planner(const PlannerLimits& limits) : limits_(limits) {
  left_.configure(limits_.velocityFilterWeight);
  right_.configure(limits_.velocityFilterWeight);
  pose_.configure(limits_.trackWidth);
}

bool Planner::move(const Move& next, bool replace) {
  // Shape validation: direction comes from the velocity sign, so the
  // profiled axis must actually be commanded; Wheels Moves are Time-bounded
  // only at v1 (sketch §3).
  const bool valid =
      next.threshold >= 0.0f &&
      ((next.velocityKind == Move::VelocityKind::Twist &&
        (next.kind != Move::Kind::Distance || next.v_x != 0.0f) &&
        (next.kind != Move::Kind::Angle || next.omega != 0.0f)) ||
       (next.velocityKind == Move::VelocityKind::Wheels &&
        next.kind == Move::Kind::Time));
  if (!valid) return false;

  if (replace) {
    pendingCount_ = 0;
    active_.occupied = false;  // the replacement activates next tick()
  }
  const int total = (active_.occupied ? 1 : 0) + pendingCount_;
  if (total >= kQueueDepth) return false;
  pending_[pendingCount_++] = next;
  return true;
}

void Planner::stop() {
  pendingCount_ = 0;
  active_.occupied = false;
  profileVelocity_ = 0.0f;
  cmdLeft_ = 0.0f;
  cmdRight_ = 0.0f;
}

TickResult Planner::tick(const RobotState& state) {
  TickResult result{};
  const uint32_t now = state.time.cycleStart;
  const float dt = limits_.controlPeriod * 0.001f;  // [s]
  lastTickTime_ = now;
  ticked_ = true;

  // SENSE -> ESTIMATE (sketch §4): filter on fresh samples only, ZOH
  // predict both wheels to a common epoch, integrate the pose from the
  // predicted positions, blend OTOS heading when fresh.
  left_.ingest(state.wheelLeft.position, state.wheelLeft.velocity,
               state.wheelLeft.sampleTime);
  right_.ingest(state.wheelRight.position, state.wheelRight.velocity,
                state.wheelRight.sampleTime);
  pose_.integrate(left_.positionAt(now), right_.positionAt(now));
  if (state.otos.present && limits_.headingOtosWeight > 0.0f &&
      now - state.otos.time <= limits_.otosStaleness) {
    pose_.blendHeading(state.otos.heading, limits_.headingOtosWeight);
  }

  if (!active_.occupied) activateNext(now);
  if (!active_.occupied) {
    drainToZero(dt);
    return result;
  }

  // DECIDE: completion first (against last tick's plan), then plan the
  // (possibly newly activated) Move's next interval.
  const Move& m = active_.move;
  const uint32_t elapsed = now - active_.activationTime;  // [ms]
  const float delay = limits_.actuationDelay * 0.001f;    // [s]
  const float bodyVelocity =
      0.5f * (left_.velocity() + right_.velocity());  // [mm/s]
  const float omegaBody =
      (right_.velocity() - left_.velocity()) / limits_.trackWidth;  // [rad/s]

  const bool timedOut = m.timeout > 0.0f &&
                        static_cast<float>(elapsed) >= m.timeout;
  bool done = timedOut;
  if (!done) {
    switch (m.kind) {
      case Move::Kind::Time:
        done = static_cast<float>(elapsed) >= m.threshold;
        break;
      case Move::Kind::Distance: {
        const float traveled = pose_.pathLength() +
                               std::fabs(bodyVelocity) * delay -
                               active_.baselinePath;
        const float remaining = m.threshold - traveled;
        // Carry boundary (>0): hand off the tick the crossing falls inside,
        // debiting the sub-tick residual to the next Move's cumulative
        // baseline. Final/orthogonal boundary (0): the terminal step has
        // already closed the sum -- complete at (float-noise) zero.
        done = remaining <= kDoneEpsilonLinear ||
               (activeBoundary_ > 0.0f &&
                remaining <= profileVelocity_ * dt + kDoneEpsilonLinear);
        break;
      }
      case Move::Kind::Angle: {
        const float dir = sign(m.omega);
        const float traveled =
            (pose_.heading() + omegaBody * delay - active_.baselineHeading) *
            dir;
        const float remaining = m.threshold - traveled;
        done = remaining <= kDoneEpsilonAngular ||
               (activeBoundary_ > 0.0f &&
                remaining <= profileVelocity_ * dt + kDoneEpsilonAngular);
        break;
      }
    }
  }

  if (done) {
    result.completed = true;
    result.moveId = m.id;
    result.timedOut = timedOut;
    // Cumulative-baseline carry (chain-exact accounting): a normally
    // completed Distance/Angle Move hands the next Move a baseline of
    // "where the boundary IS" (baseline + threshold), not "where we
    // happened to be when completion fired" -- sub-tick residual is
    // debited to the next Move so a chain leaks zero total error. A
    // timeout aborts the motion: no carry, next baselines re-anchor.
    carryValid_ = !timedOut;
    carryKind_ = m.kind;
    carryPath_ = active_.baselinePath + m.threshold;
    carryHeading_ =
        active_.baselineHeading + sign(m.omega) * m.threshold;
    active_.occupied = false;
    activateNext(now);
    if (!active_.occupied) {
      drainToZero(dt);
      return result;
    }
  }

  planActive(now, dt);
  return result;
}

void Planner::activateNext(uint32_t now) {
  carryValid_ = carryValid_ && pendingCount_ > 0;  // carry consumed below or dropped
  if (pendingCount_ == 0) {
    return;
  }
  const Move next = pending_[0];
  for (int i = 1; i < pendingCount_; ++i) pending_[i - 1] = pending_[i];
  --pendingCount_;

  active_.occupied = true;
  active_.move = next;
  active_.activationTime = now;
  active_.baselinePath = pose_.pathLength();
  active_.baselineHeading = pose_.heading();
  if (carryValid_) {
    if (carryKind_ == Move::Kind::Distance &&
        next.kind == Move::Kind::Distance) {
      active_.baselinePath = carryPath_;
    } else if (carryKind_ == Move::Kind::Angle &&
               next.kind == Move::Kind::Angle) {
      active_.baselineHeading = carryHeading_;
    }
  }
  carryValid_ = false;

  // Same-axis carry keeps the profile's ramp continuity; an axis change
  // starts the new axis's profile from rest (we landed at ~0 there).
  const Axis axis = axisOf(next);
  if (axis != lastAxis_) profileVelocity_ = 0.0f;
  lastAxis_ = axis;
  activeBoundary_ = 0.0f;
}

Planner::Axis Planner::axisOf(const Move& m) {
  if (m.velocityKind == Move::VelocityKind::Wheels) return Axis::Wheels;
  if (m.kind == Move::Kind::Angle) return Axis::Angular;
  return Axis::Linear;
}

float Planner::boundaryVelocity(float dt) const {
  if (pendingCount_ == 0) return 0.0f;
  const Move& m = active_.move;
  const Move& next = pending_[0];
  if (m.velocityKind != Move::VelocityKind::Twist ||
      next.velocityKind != Move::VelocityKind::Twist) {
    return 0.0f;
  }
  if (m.kind == Move::Kind::Distance || m.kind == Move::Kind::Time) {
    // Linear axis: carry into a same-direction Distance/Time successor.
    if ((next.kind == Move::Kind::Distance || next.kind == Move::Kind::Time) &&
        next.v_x != 0.0f && sign(next.v_x) == sign(m.v_x)) {
      const AxisLimits lin{limits_.vMax, limits_.aMax, limits_.aDecel};
      const float entryCap =
          next.kind == Move::Kind::Distance
              ? maxEntryVelocity(next.threshold, 0.0f, lin, dt)
              : limits_.vMax;
      return std::min(std::fabs(next.v_x), entryCap);
    }
    return 0.0f;
  }
  // Angular axis: carry into a same-direction Angle successor.
  if (m.kind == Move::Kind::Angle && next.kind == Move::Kind::Angle &&
      next.omega != 0.0f && sign(next.omega) == sign(m.omega)) {
    const AxisLimits ang{limits_.omegaMax, limits_.alphaMax,
                         limits_.alphaDecel};
    const float entryCap =
        maxEntryVelocity(next.threshold, 0.0f, ang, dt);
    return std::min(std::fabs(next.omega), entryCap);
  }
  return 0.0f;
}

void Planner::planActive(uint32_t now, float dt) {
  const Move& m = active_.move;
  const float delay = limits_.actuationDelay * 0.001f;  // [s]
  const float bodyVelocity = 0.5f * (left_.velocity() + right_.velocity());
  const float omegaBody =
      (right_.velocity() - left_.velocity()) / limits_.trackWidth;
  const float elapsed = static_cast<float>(now - active_.activationTime);
  const float period = limits_.controlPeriod;  // [ms]

  if (m.velocityKind == Move::VelocityKind::Wheels) {
    // Time-bounded per-wheel ramp (v1 scope; no wheels lookahead).
    const float ticksLeft = (m.threshold - elapsed) / period;
    const float accelStep = limits_.aMax * dt;
    const float decelStep = limits_.aDecel * dt;
    cmdLeft_ = timedRamp(cmdLeft_, m.vLeft, 0.0f, accelStep, decelStep,
                         ticksLeft);
    cmdRight_ = timedRamp(cmdRight_, m.vRight, 0.0f, accelStep, decelStep,
                          ticksLeft);
    activeBoundary_ = 0.0f;
    return;
  }

  switch (m.kind) {
    case Move::Kind::Distance: {
      const float dir = sign(m.v_x);
      const float traveled = pose_.pathLength() +
                             std::fabs(bodyVelocity) * delay -
                             active_.baselinePath;
      const float remaining = m.threshold - traveled;
      const AxisLimits lin{limits_.vMax, limits_.aMax, limits_.aDecel};
      activeBoundary_ = boundaryVelocity(dt);
      const ProfileResult step = profileStep(
          remaining, profileVelocity_, std::fabs(m.v_x), activeBoundary_,
          lin, dt);
      profileVelocity_ = step.velocity;
      active_.closingIssued = step.closing;
      const float v = dir * step.velocity;
      cmdLeft_ = v;
      cmdRight_ = v;
      break;
    }
    case Move::Kind::Angle: {
      const float dir = sign(m.omega);
      const float traveled =
          (pose_.heading() + omegaBody * delay - active_.baselineHeading) *
          dir;
      const float remaining = m.threshold - traveled;
      const AxisLimits ang{limits_.omegaMax, limits_.alphaMax,
                           limits_.alphaDecel};
      activeBoundary_ = boundaryVelocity(dt);
      const ProfileResult step = profileStep(
          remaining, profileVelocity_, std::fabs(m.omega), activeBoundary_,
          ang, dt);
      profileVelocity_ = step.velocity;
      active_.closingIssued = step.closing;
      const float omega = dir * step.velocity;
      const float halfTrack = 0.5f * limits_.trackWidth;
      cmdLeft_ = -omega * halfTrack;
      cmdRight_ = omega * halfTrack;
      break;
    }
    case Move::Kind::Time: {
      // Both twist axes ramp toward cruise; the linear axis may carry into
      // the next Move, the angular axis lands at zero.
      const float ticksLeft = (m.threshold - elapsed) / period;
      activeBoundary_ = boundaryVelocity(dt);
      const float vPrev = 0.5f * (cmdLeft_ + cmdRight_);
      const float omegaPrev = (cmdRight_ - cmdLeft_) / limits_.trackWidth;
      const float v = timedRamp(vPrev, m.v_x, sign(m.v_x) * activeBoundary_,
                                limits_.aMax * dt, limits_.aDecel * dt,
                                ticksLeft);
      const float omega =
          timedRamp(omegaPrev, m.omega, 0.0f, limits_.alphaMax * dt,
                    limits_.alphaDecel * dt, ticksLeft);
      profileVelocity_ = std::fabs(v);
      const float halfTrack = 0.5f * limits_.trackWidth;
      cmdLeft_ = v - omega * halfTrack;
      cmdRight_ = v + omega * halfTrack;
      break;
    }
  }
}

void Planner::drainToZero(float dt) {
  const float decelStep = limits_.aDecel * dt;
  auto toward = [&](float v) {
    if (v > decelStep) return v - decelStep;
    if (v < -decelStep) return v + decelStep;
    return 0.0f;
  };
  cmdLeft_ = toward(cmdLeft_);
  cmdRight_ = toward(cmdRight_);
  profileVelocity_ = std::max(0.0f, profileVelocity_ - decelStep);
}

void Planner::update(RobotState& state) const {
  state.wheelLeft.cmdVelocity = cmdLeft_;
  state.wheelRight.cmdVelocity = cmdRight_;
  state.command.moveActive = active_.occupied;
  state.command.activeMoveId = active_.occupied ? active_.move.id : 0;

  const float bodyVelocity = 0.5f * (left_.velocity() + right_.velocity());
  const float omegaBody =
      (right_.velocity() - left_.velocity()) / limits_.trackWidth;
  state.pose.x = pose_.x();
  state.pose.y = pose_.y();
  state.pose.heading = pose_.heading();
  state.pose.vx = bodyVelocity;
  state.pose.vy = 0.0f;
  state.pose.omega = omegaBody;

  state.estimate.wheelLeft = {left_.basisPosition(), left_.velocity(),
                              left_.basisTime(), left_.valid()};
  state.estimate.wheelRight = {right_.basisPosition(), right_.velocity(),
                               right_.basisTime(), right_.valid()};
  state.estimate.body = {pose_.x(),   pose_.y(), pose_.heading(),
                         bodyVelocity, 0.0f,     omegaBody,
                         lastTickTime_, ticked_};
}

}  // namespace Motion
