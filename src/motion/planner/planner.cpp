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

// Settle-confirm gates (PlannerLimits::requireSettle). Unlike the done
// epsilons above these ARE physical tolerances -- "close enough to the
// target, and stopped" for a real, lagging plant.
constexpr float kSettleEpsilonLinear = 1.0f;      // [mm]
constexpr float kSettleEpsilonAngular = 0.005f;   // [rad]
// Rest floors. The gate is |velocity| <= max(floor, one decel step): a
// body within one decel step of zero is PROVABLY at rest by the end of the
// next interval, which is what makes settle-complete coincide with
// profile-complete in a zero-error plant (the profiler's terminal step is
// itself capped at one decel step above the boundary). The constants are
// the noise floor below that, for an already-stopped real encoder.
constexpr float kSettleRestLinear = 5.0f;    // [mm/s]
constexpr float kSettleRestAngular = 0.05f;  // [rad/s]

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
  // The history too: after a stop there is no travel left to anticipate.
  cmdLeftPrevious_ = 0.0f;
  cmdRightPrevious_ = 0.0f;
}

// Age the staged command by one tick. Called from the two places that
// overwrite cmdLeft_/cmdRight_ -- exactly one of which runs per tick --
// and never before measure(), which reads both generations.
void Planner::rollCommandHistory() {
  cmdLeftPrevious_ = cmdLeft_;
  cmdRightPrevious_ = cmdRight_;
}

TickResult Planner::tick(const RobotState& state) {
  TickResult result{};
  const uint32_t now = state.time.cycleStart;
  const float dt = limits_.controlPeriod * 0.001f;  // [s]
  ticked_ = true;

  // SENSE -> ESTIMATE (sketch §4): filter on fresh samples only, integrate
  // the pose from the measured anchors, blend OTOS heading when fresh. The
  // ZOH predict-to-now happens in measure(), not here -- see below.
  left_.ingest(state.wheelLeft.position, state.wheelLeft.velocity,
               state.wheelLeft.sampleTime);
  right_.ingest(state.wheelRight.position, state.wheelRight.velocity,
                state.wheelRight.sampleTime);
  // Integrate the pose from the MEASURED anchors, never from the ZOH
  // extrapolation. pathLength accumulates |ds|, so any zero-mean jitter in
  // its input is rectified into a one-way drift: an extrapolated input
  // would make a standing robot's odometer keep climbing at roughly
  // |velocity noise| * period per tick. Anchors only move when a real
  // sample lands, so a standing robot's path length is exactly flat. The
  // extrapolation is still applied -- as an additive lookahead term in
  // measure(), where it informs the plan without accumulating (sketch §4:
  // "traveled distance is ALWAYS anchored to measured positions").
  pose_.integrate(left_.basisPosition(), right_.basisPosition());
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
  const Measurement measured = measure(now);

  const bool timedOut = m.timeout > 0.0f &&
                        static_cast<float>(elapsed) >= m.timeout;
  bool done = timedOut;
  if (!done && active_.settling) {
    done = true;  // profile-complete already fired; only the gate is pending
  } else if (!done) {
    switch (m.kind) {
      case Move::Kind::Time:
        done = static_cast<float>(elapsed) >= m.threshold;
        break;
      case Move::Kind::Distance:
        // Carry boundary (>0): hand off the tick the crossing falls inside,
        // debiting the sub-tick residual to the next Move's cumulative
        // baseline. Final/orthogonal boundary (0): the terminal step has
        // already closed the sum -- complete at (float-noise) zero.
        done = measured.plannedRemaining <= kDoneEpsilonLinear ||
               (activeBoundary_ > 0.0f &&
                measured.plannedRemaining <=
                    profileVelocity_ * dt + kDoneEpsilonLinear);
        break;
      case Move::Kind::Angle:
        done = measured.plannedRemaining <= kDoneEpsilonAngular ||
               (activeBoundary_ > 0.0f &&
                measured.plannedRemaining <=
                    profileVelocity_ * dt + kDoneEpsilonAngular);
        break;
    }
  }

  bool settled = false;
  if (done) {
    settled = settleReached(measured, dt);
    // Settle-confirm: hold the completion back until the body has actually
    // arrived and stopped. Only for a Distance/Angle Move landing at rest
    // -- a Time Move's stop condition IS the clock, and a Move handing off
    // at speed into a same-axis successor (activeBoundary_ > 0) is not
    // supposed to come to rest at all. A timeout aborts the motion and is
    // never deferred.
    const bool settleApplies =
        limits_.requireSettle && !timedOut && activeBoundary_ <= 0.0f &&
        (m.kind == Move::Kind::Distance || m.kind == Move::Kind::Angle) &&
        m.velocityKind == Move::VelocityKind::Twist;
    if (settleApplies && !settled) {
      if (!active_.settling) {
        active_.settling = true;
        active_.settleStart = now;
      }
      const float waited = static_cast<float>(now - active_.settleStart);
      // Past the window: complete anyway, reporting settled = false. This
      // is NOT a timeout -- the Move's own stop condition was met, only
      // the physical confirmation of it was not.
      done = waited >= limits_.settleWindow;
    }
  }

  if (done) {
    result.completed = true;
    result.moveId = m.id;
    result.timedOut = timedOut;
    result.settled = settled;
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

  // Re-measure: a same-tick hand-off above swapped the active Move, and
  // `remaining` is measured against ITS baseline and axis.
  planActive(now, dt, done ? measure(now) : measured);
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
  active_.closingIssued = false;
  active_.settling = false;
  active_.settleStart = now;
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

Planner::Measurement Planner::measure(uint32_t now) const {
  Measurement out;
  out.bodyVelocity = 0.5f * (left_.velocity() + right_.velocity());  // [mm/s]
  out.omega =
      (right_.velocity() - left_.velocity()) / limits_.trackWidth;  // [rad/s]
  if (!active_.occupied) return out;

  // Anticipation: how far the body still travels between the last measured
  // anchor and the instant this tick's command takes effect. Two adjacent
  // spans, each attributed to the command that actually drives it:
  //
  //   [anchorTime, now]  -- already elapsed, unobserved (the sample is
  //       older than the loop). With one cycle of staging latency the
  //       command in force over it was staged the tick BEFORE last.
  //   [now, now + delay] -- not yet elapsed; the command staged last tick
  //       is still in force until this tick's replaces it.
  //
  // Both spans use the COMMANDED velocity, not the measured one. Under the
  // velocity-tracking plant the whole profiler is built on they agree --
  // but the command is exact where the encoder's derivative is very noisy
  // (sketch §4), and this term is differenced for the angular axis, where
  // per-wheel noise does not cancel the way it does in the mean. If the
  // plant fails to track, the error is bounded by one sample interval and
  // fully corrected by the next anchor; it never accumulates, because the
  // pose itself is anchored to measured positions.
  const float delay = limits_.actuationDelay * 0.001f;  // [s]
  const float ageLeft =
      static_cast<float>(now - left_.basisTime()) * 0.001f;  // [s]
  const float ageRight =
      static_cast<float>(now - right_.basisTime()) * 0.001f;  // [s]
  const float elapsedLeft = delay > 0.0f ? cmdLeftPrevious_ : cmdLeft_;
  const float elapsedRight = delay > 0.0f ? cmdRightPrevious_ : cmdRight_;
  const float predictLeft =
      elapsedLeft * ageLeft + cmdLeft_ * delay;  // [mm]
  const float predictRight =
      elapsedRight * ageRight + cmdRight_ * delay;  // [mm]
  const float predictPath = 0.5f * (predictLeft + predictRight);       // [mm]
  const float predictHeading =
      (predictRight - predictLeft) / limits_.trackWidth;               // [rad]

  const Move& m = active_.move;
  switch (m.kind) {
    case Move::Kind::Time:
      break;  // a Time Move's residual is the clock, not a distance
    case Move::Kind::Distance: {
      const float dir = sign(m.v_x);
      out.anchoredRemaining =
          m.threshold - (pose_.pathLength() - active_.baselinePath);
      out.plannedRemaining = out.anchoredRemaining - dir * predictPath;
      break;
    }
    case Move::Kind::Angle: {
      const float dir = sign(m.omega);
      out.anchoredRemaining =
          m.threshold - (pose_.heading() - active_.baselineHeading) * dir;
      out.plannedRemaining = out.anchoredRemaining - dir * predictHeading;
      break;
    }
  }
  return out;
}

bool Planner::settleReached(const Measurement& measured, float dt) const {
  if (!active_.occupied) return false;
  const Move& m = active_.move;
  if (m.velocityKind != Move::VelocityKind::Twist) return false;
  switch (m.kind) {
    case Move::Kind::Time:
      return false;  // nothing physical to confirm
    case Move::Kind::Distance:
      return std::fabs(measured.anchoredRemaining) <= kSettleEpsilonLinear &&
             std::fabs(measured.bodyVelocity) <=
                 std::max(kSettleRestLinear, limits_.aDecel * dt);
    case Move::Kind::Angle:
      return std::fabs(measured.anchoredRemaining) <= kSettleEpsilonAngular &&
             std::fabs(measured.omega) <=
                 std::max(kSettleRestAngular, limits_.alphaDecel * dt);
  }
  return false;
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

void Planner::planActive(uint32_t now, float dt, const Measurement& measured) {
  rollCommandHistory();
  const Move& m = active_.move;
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
      const AxisLimits lin{limits_.vMax, limits_.aMax, limits_.aDecel};
      activeBoundary_ = boundaryVelocity(dt);
      const ProfileResult step = profileStep(
          measured.plannedRemaining, profileVelocity_, std::fabs(m.v_x),
          activeBoundary_, lin, dt);
      profileVelocity_ = step.velocity;
      active_.closingIssued = step.closing;
      const float v = dir * step.velocity;
      cmdLeft_ = v;
      cmdRight_ = v;
      applyHeadingHold();
      break;
    }
    case Move::Kind::Angle: {
      const float dir = sign(m.omega);
      const AxisLimits ang{limits_.omegaMax, limits_.alphaMax,
                           limits_.alphaDecel};
      activeBoundary_ = boundaryVelocity(dt);
      const ProfileResult step = profileStep(
          measured.plannedRemaining, profileVelocity_, std::fabs(m.omega),
          activeBoundary_, ang, dt);
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

void Planner::applyHeadingHold() {
  if (limits_.headingHoldGain <= 0.0f) return;
  // P on the uncommanded axis, back toward the Move's activation heading.
  const float error = active_.baselineHeading - pose_.heading();  // [rad]
  const float omegaCorrection = limits_.headingHoldGain * error;  // [rad/s]
  float differential = omegaCorrection * 0.5f * limits_.trackWidth;  // [mm/s]

  // Clamp the CORRECTION, never the profiled velocity: the faster wheel
  // must stay inside vMax, and the mean of the pair -- which is what the
  // odometry integrates as ds, and therefore what the distance accounting
  // depends on -- must come out exactly as profiled.
  const float profiled = 0.5f * (cmdLeft_ + cmdRight_);  // [mm/s]
  const float headroom = std::max(0.0f, limits_.vMax - std::fabs(profiled));
  differential = std::clamp(differential, -headroom, headroom);

  cmdLeft_ = profiled - differential;
  cmdRight_ = profiled + differential;
}

void Planner::drainToZero(float dt) {
  rollCommandHistory();
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
  // The body basis is stamped at the WHEEL anchors, not at the tick: the
  // pose is integrated from measured anchor positions, so it is an
  // estimate as of the older of the two anchors, and a consumer holding a
  // copied state must extrapolate from there -- which is exactly what the
  // basisTime + velocity pair is for. Stamping it `now` would claim a
  // freshness the pose does not have.
  const uint32_t basisTime =
      left_.basisTime() < right_.basisTime() ? left_.basisTime()
                                             : right_.basisTime();
  state.estimate.body = {pose_.x(),    pose_.y(), pose_.heading(),
                         bodyVelocity, 0.0f,      omegaBody,
                         basisTime,    ticked_ && left_.valid() &&
                                           right_.valid()};
}

}  // namespace Motion
