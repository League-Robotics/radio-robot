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
// (Settle-confirm arrival tolerances moved to PlannerLimits::
// settleEpsilonLinear/settleEpsilonAngular -- reachability depends on the
// robot's stiction-limited minimum creep step, a per-robot property.)
// Rest floors -- the settle gate's ONLY velocity criterion. (An earlier
// revision widened the gate to max(floor, one decel step), reasoning a
// commanded plant one step from zero is at rest next interval -- but a
// REAL plant with time constant tau COASTS ~v*tau past the target after
// the command reaches zero: at alphaDecel*dt = 0.25 rad/s that admitted
// +0.9 deg of post-settle coast per turn. The floors are sized so the
// worst coast is within the arrival epsilons.)
// (Rest floors moved to PlannerLimits::settleRestVelocity/settleRestOmega
// -- a per-robot noise property, not a universal constant.)

float sign(float value) { return value < 0.0f ? -1.0f : 1.0f; }

// Signed, clamped age between two robot-clock stamps [ms] -> [s]. The
// loop stamps cycleStart at the TOP of the cycle but collects encoder
// samples several ms LATER in the schedule, so a fresh sample's time is
// legitimately AHEAD of `now` -- unsigned subtraction would wrap to a
// ~50-day age and corrupt every prediction (surfaced the moment the sim
// mirrored the real schedule, 2026-07-26).
float ageSeconds(uint32_t now, uint32_t basis) {
  return std::max(
      0.0f, static_cast<float>(static_cast<int32_t>(now - basis)) * 0.001f);
}

// Signed one-axis ramp for Time/Wheels Moves: hold toward `cruise`, and
// once the remaining ticks are just enough to reach `boundary` at the
// decel ceiling, step toward it.
float timedRamp(float previous, float cruise, float boundary,
                float accelStep, float decelStep, float ticksLeft) {
  const float toBoundary = std::fabs(previous - boundary);
  const float stepsNeeded =
      decelStep > 0.0f ? std::ceil(toBoundary / decelStep) : 0.0f;
  // STRICT comparison: at ticksLeft == stepsNeeded the ramp holds cruise
  // one more tick and still lands within the Move (an elapsed-time Move's
  // landing is the clock, not a distance) -- anticipating one tick early
  // staged a zero-command frame BEFORE the completion ack could ride out,
  // which chain observers correctly read as a hand-off dip.
  if (ticksLeft < stepsNeeded) {
    const float step = std::min(decelStep, toBoundary);
    return previous + (boundary > previous ? step : -step);
  }
  if (cruise > previous) return std::min(cruise, previous + accelStep);
  return std::max(cruise, previous - std::max(accelStep, decelStep));
}

// Direction of the profiled axis for measurement/carry accounting. A
// Twist Move carries it on the commanded twist's sign; a Wheels Move has
// v_x == omega == 0, so its direction lives structurally on the pair --
// the mean for the linear axis, the differential for the angular axis.
// (Without this, a Wheels Move with a Distance/Angle stop measured a
// frozen remaining == threshold and could only ever end on its timeout.)
float linearDirection(const Move& m) {
  if (m.velocityKind == Move::VelocityKind::Wheels) {
    return sign(0.5f * (m.vLeft + m.vRight));
  }
  return sign(m.v_x);
}

float angularDirection(const Move& m) {
  if (m.velocityKind == Move::VelocityKind::Wheels) {
    return sign(m.vRight - m.vLeft);
  }
  return sign(m.omega);
}

}  // namespace

Planner::Planner(const PlannerLimits& limits) : limits_(limits) {
  left_.configure(limits_.velocityFilterWeight);
  right_.configure(limits_.velocityFilterWeight);
  pose_.configure(limits_.trackWidth);
  const PidGains gains{limits_.velKff, limits_.velKp, limits_.velKi,
                       limits_.velIMax, limits_.velKaff, limits_.velIAccelGate};
  pidLeft_.configure(gains);
  pidRight_.configure(gains);
}

// M4 duty output stage: this tick's staged velocity targets vs the
// filtered measured wheel velocities -> per-wheel duty. Runs on every
// tick() exit path so the duty outputs always mirror the velocity
// outputs; inert (0) at the default all-zero gains.
//
// Rest damping: an exactly-zero target with the wheel already near rest
// is a hard stop, not a trim problem -- the integral is reset (and held
// frozen by the braking gate below) so it can never reverse-creep the
// landed pose while unwinding through zero (measured on the duty plant:
// ~1 deg of back-rotation after a settled turn). The PROPORTIONAL term
// stays engaged: duty = -kp*measured is self-terminating (it crosses
// zero exactly when the measured velocity does, so it cannot push the
// wheel backward), drains the coast tail ~(1 + gain*kp)x faster than a
// dead-duty coast, and its residual noise response is far below the
// motor deadband. While the wheel is still moving fast the full PID
// stays engaged and actively brakes.
void Planner::stageDuty(float dt) {
  constexpr float kRestClampVelocity = 30.0f;  // [mm/s]
  const auto stage = [&](WheelPid& pid, float cmd, float cmdPrevious,
                         float measured, float measuredPrevious, float& duty) {
    if (cmd == 0.0f && std::fabs(measured) <= kRestClampVelocity) {
      pid.reset();
      // Below the per-robot measured-noise floor the wheel is
      // indistinguishable from stopped -- output EXACTLY zero so the
      // motor write path can actually go quiet (Drive's raw pass stays
      // silent at zero, and NezhaMotor::reconfigure()'s at-rest gate
      // demands a zero applied duty). P-damping a value that is pure
      // sensor noise would jitter the duty word forever for nothing.
      if (std::fabs(measured) <= limits_.settleRestVelocity) {
        duty = 0.0f;
        return;
      }
    }
    // The commanded accel behind this tick's target -- feeds the PID's
    // acceleration feedforward (PidGains::kaff). Forced to ZERO during
    // the settle phase: the creep is quasi-static, and running the accel
    // feedforward + filter-lag compensation on its per-tick command
    // jitter was measured to drive a +-0.25 rad/s hunt that kept the
    // rest gate from ever confirming.
    const float targetAccel =
        active_.settling ? 0.0f : (cmd - cmdPrevious) / dt;
    // Braking to rest (zero target, wheel still above the rest clamp) is
    // a pure transient: integrating its huge error winds the clamp full
    // (measured: -0.3 duty in 3 ticks on a STOP from cruise) and then
    // RELEASES as a reversal once the wheel stops. kp+kff brake; the
    // integral sits this one out.
    const bool brakingToRest = cmd == 0.0f;
    // Actuation-lead compensation while braking to rest: this tick's duty
    // reaches the wheels one actuation delay from now, and a hard-braking
    // wheel is measurably slower by then -- braking against the stale
    // measurement lands a full-strength duty on an already-stopped wheel
    // and rings it through zero (measured: a stop from cruise flipped to
    // -19 mm/s the tick after the wheel first read ~0). Lead the braking
    // measurement by the PREVIOUS interval's measured accel (leading with
    // this tick's own step would blind kp -- same rationale as the
    // command-side history), clamped at zero so braking can never be
    // computed against a predicted sign flip.
    float measurementBasis = measured;
    if (brakingToRest) {
      const float measuredAccel = (measured - measuredPrevious) / dt;
      measurementBasis =
          measured + measuredAccel * limits_.actuationDelay * 0.001f;
      if (measured > 0.0f) {
        measurementBasis = std::max(measurementBasis, 0.0f);
      } else if (measured < 0.0f) {
        measurementBasis = std::min(measurementBasis, 0.0f);
      }
    }
    // Filter-lag compensation: the EMA velocity filter has a group delay
    // of ~dt*(1-w)/w, so during a ramp the filtered feedback reads
    // a*lag LOW -- kp (and the ramp-rate integral) then chase a phantom
    // error that scales with accel. Predict the measurement forward by
    // the filter's own delay (derived from the configured weight, no new
    // tunable); exact zero at steady state.
    const float w = std::max(limits_.velocityFilterWeight, 0.05f);
    const float filterLag = dt * (1.0f - w) / w;  // [s]
    duty = pid.compute(cmd, targetAccel,
                       measurementBasis + targetAccel * filterLag, dt,
                       brakingToRest);
    // Stiction breakaway kick -- STUCK WHEELS ONLY: the settle creep
    // commands velocities whose PID duty computes below the gearbox
    // breakaway, so a wheel AT REST silently ignores them (measured: the
    // creep stalling dead while ~0.02-duty commands evaporated). When the
    // wheel is effectively at rest during settling and the creep wants
    // motion, kick at the breakaway duty IN THE COMMANDED DIRECTION for
    // this tick; the plant's ~230 ms time constant makes one 40 ms kick
    // reach only ~0.5 mm, so the step stays inside the arrival epsilons.
    // Deliberately NOT applied while the wheel is still moving: kinetic
    // friction is below breakaway and the plain PID handles a rolling
    // wheel -- flooring every near-zero duty instead bang-banged the
    // whole landing at +-floor as the raw duty's sign flickered
    // (measured: a second full-speed push past the boundary and paired
    // opposite-sign pulses, walking the turn to +10 deg residual).
    if (active_.settling && cmd != 0.0f &&
        std::fabs(measured) <= limits_.settleRestVelocity &&
        std::fabs(duty) < limits_.dutyFloor) {
      duty = std::copysign(limits_.dutyFloor, cmd);
    }
  };
  stage(pidLeft_, cmdLeft_, cmdLeftPrevious_, left_.velocity(),
        measuredLeftPrevious_, dutyLeft_);
  stage(pidRight_, cmdRight_, cmdRightPrevious_, right_.velocity(),
        measuredRightPrevious_, dutyRight_);
  measuredLeftPrevious_ = left_.velocity();
  measuredRightPrevious_ = right_.velocity();
}

void Planner::applyVelGains(float kff, float kp, float ki, float iMax) {
  limits_.velKff = kff;
  limits_.velKp = kp;
  limits_.velKi = ki;
  limits_.velIMax = iMax;
  const PidGains gains{kff, kp, ki, iMax, limits_.velKaff,
                       limits_.velIAccelGate};
  pidLeft_.configure(gains);
  pidRight_.configure(gains);
}

void Planner::applyShaperLimits(float aMax, float aDecel, float alphaMax,
                                float alphaDecel, float jerkMax,
                                float yawJerkMax) {
  limits_.aMax = aMax;
  limits_.aDecel = aDecel;
  limits_.alphaMax = alphaMax;
  limits_.alphaDecel = alphaDecel;
  limits_.jerkMax = jerkMax;
  limits_.yawJerkMax = yawJerkMax;
  shaperConfigured_ = true;
}

bool Planner::move(const Move& next, bool replace) {
  // Shape validation: direction comes from the velocity sign, so the
  // profiled axis must actually be commanded; Wheels Moves are Time-bounded
  // only at v1 (sketch §3).
  //
  // v_y is carried but not actuated on a differential drivetrain. A Move
  // whose ONLY commanded linear velocity is sideways therefore asks for a
  // motion this robot cannot make, and is rejected here rather than
  // accepted and silently driven as nothing -- the Distance/Angle guards
  // below test v_x/omega, which a pure-v_y Move leaves at zero.
  const bool valid =
      next.threshold >= 0.0f &&
      ((next.velocityKind == Move::VelocityKind::Twist &&
        (next.kind != Move::Kind::Distance || next.v_x != 0.0f) &&
        (next.kind != Move::Kind::Angle || next.omega != 0.0f)) ||
       (next.velocityKind == Move::VelocityKind::Wheels));
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
  profileAccel_ = 0.0f;
  cmdLeft_ = 0.0f;
  cmdRight_ = 0.0f;
  // The history too: after a stop there is no travel left to anticipate.
  cmdLeftPrevious_ = 0.0f;
  cmdRightPrevious_ = 0.0f;
  pidLeft_.reset();
  pidRight_.reset();
  dutyLeft_ = 0.0f;
  dutyRight_ = 0.0f;
}

// Age the staged command by one tick. Called from the two places that
// overwrite cmdLeft_/cmdRight_ -- exactly one of which runs per tick --
// and never before measure(), which reads both generations.
void Planner::rollCommandHistory() {
  cmdLeftPrevious_ = cmdLeft_;
  cmdRightPrevious_ = cmdRight_;
}

TickResult Planner::tick(const Types::RobotState& state) {
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
      static_cast<int32_t>(now - state.otos.sampleTime) <=
          static_cast<int32_t>(limits_.otosStaleness)) {
    pose_.blendHeading(state.otos.heading, limits_.headingOtosWeight);
  }

  if (!active_.occupied) activateNext(now);
  if (!active_.occupied) {
    drainToZero(dt);
    stageDuty(dt);
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
        // Complete on the tick whose interval CONTAINS the expiry -- the
        // same tick the ramp's taper lands at the boundary. When the
        // threshold falls mid-interval, waiting for elapsed >= threshold
        // adds a whole extra tick commanding zero while still active: one
        // more pre-ack zero-target cycle than the loop schedule's single
        // ack-visibility lag, which chain observers correctly read as a
        // hand-off gap. Exact-multiple thresholds still complete exactly
        // at the threshold, and every Move gets at least one planned tick.
        done = elapsed > 0 && static_cast<float>(elapsed) +
                                      limits_.controlPeriod >
                                  m.threshold;
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
    // completed Twist Distance/Angle Move hands the next Move BOTH
    // cumulative baselines -- "where the boundary IS" on its own axis
    // (baseline + threshold) and its own UNCHANGED baseline on the other
    // axis (a straight leg intends zero heading change; a pivot intends
    // zero path change). Carrying the full ledger across KINDS is what
    // closes a mixed leg/turn tour: each turn targets the cumulative
    // n*90deg and each leg's heading-hold pulls back to the carried
    // square heading, so per-landing residuals cancel instead of
    // accumulating (measured on the bench tour: +17 deg over 8 moves
    // with same-kind-only carry, every leg/turn boundary re-anchoring
    // to wherever the pose drifted). A timeout aborts the motion, and a
    // Time/Wheels Move has no single-axis intent -- no carry, next
    // baselines re-anchor to the pose.
    carryValid_ = !timedOut &&
                  m.velocityKind == Move::VelocityKind::Twist &&
                  (m.kind == Move::Kind::Distance ||
                   m.kind == Move::Kind::Angle);
    if (m.kind == Move::Kind::Distance) {
      carryPath_ = active_.baselinePath + linearDirection(m) * m.threshold;
      carryHeading_ = active_.baselineHeading;
    } else {
      carryPath_ = active_.baselinePath;
      carryHeading_ =
          active_.baselineHeading + angularDirection(m) * m.threshold;
    }
    active_.occupied = false;
    activateNext(now);
    if (!active_.occupied) {
      drainToZero(dt);
      stageDuty(dt);
      return result;
    }
  }

  // Re-measure: a same-tick hand-off above swapped the active Move, and
  // `remaining` is measured against ITS baseline and axis.
  planActive(now, dt, done ? measure(now) : measured);
  stageDuty(dt);
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
  active_.baselinePath =
      0.5f * (left_.basisPosition() + right_.basisPosition());
  active_.baselineHeading = pose_.heading();
  if (carryValid_ && next.velocityKind == Move::VelocityKind::Twist &&
      (next.kind == Move::Kind::Distance || next.kind == Move::Kind::Angle)) {
    // Full-ledger adoption (see the completion-side comment): both axes'
    // cumulative baselines, regardless of the predecessor's kind.
    active_.baselinePath = carryPath_;
    active_.baselineHeading = carryHeading_;
  }
  carryValid_ = false;

  // Same-axis carry keeps the profile's ramp continuity; an axis change
  // starts the new axis's profile from rest (we landed at ~0 there).
  const Axis axis = axisOf(next);
  if (axis != lastAxis_) {
    profileVelocity_ = 0.0f;
    profileAccel_ = 0.0f;
  }
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
  const float ageLeft = ageSeconds(now, left_.basisTime());    // [s]
  const float ageRight = ageSeconds(now, right_.basisTime());  // [s]
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
      // Traveled distance is the SIGNED mean-wheel displacement along the
      // Move's direction -- a telescoping measure (final minus baseline
      // anchor positions), immune to noise/quantum rectification. The
      // former pathLength() measure accumulated |ds| per cycle, so the
      // encoder quantum's flicker during a slow settle-creep INFLATED it
      // ~0.07 mm per jitter cycle and completed Moves measurably short.
      const float dir = linearDirection(m);
      const float meanPosition =
          0.5f * (left_.basisPosition() + right_.basisPosition());
      out.anchoredRemaining =
          m.threshold - dir * (meanPosition - active_.baselinePath);
      out.plannedRemaining = out.anchoredRemaining - dir * predictPath;
      break;
    }
    case Move::Kind::Angle: {
      const float dir = angularDirection(m);
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
      return std::fabs(measured.anchoredRemaining) <=
                 limits_.settleEpsilonLinear &&
             std::fabs(measured.bodyVelocity) <= limits_.settleRestVelocity;
    case Move::Kind::Angle:
      return std::fabs(measured.anchoredRemaining) <=
                 limits_.settleEpsilonAngular &&
             std::fabs(measured.omega) <= limits_.settleRestOmega;
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

  // M1 terminal-settle: once the profile has closed its sum, the final
  // approach is a CLOSED-LOOP, BIDIRECTIONAL creep on the MEASURED
  // residual -- the profile itself is positive-frame and legally lands
  // with up to one decel step of residual speed, which a lagging plant
  // coasts PAST the target (measured: +3.3 deg past a turn, from where a
  // forward-only profile can never return). The creep is a plain P law
  // with tight caps: it walks the residual to the arrival epsilon from
  // EITHER side, decelerating as it converges, so the rest gate and the
  // arrival gate become satisfiable together.
  if (active_.settling &&
      (m.kind == Move::Kind::Distance || m.kind == Move::Kind::Angle)) {
    // Gain sized for convergence well inside a ~2 s window from the
    // worst hand-off residual; safe from hunting now that the accel
    // feedforward and filter-lag compensation are disabled during
    // settling (they were the churn source, not the gain).
    constexpr float kCreepGain = 2.5f;         // [1/s]
    constexpr float kCreepMaxLinear = 35.0f;   // [mm/s]
    constexpr float kCreepMaxAngular = 0.35f;  // [rad/s]
    if (m.kind == Move::Kind::Distance) {
      const float dir = sign(m.v_x);
      const float previous = 0.5f * (cmdLeftPrevious_ + cmdRightPrevious_);
      float v = dir * std::clamp(kCreepGain * measured.anchoredRemaining,
                                 -kCreepMaxLinear, kCreepMaxLinear);
      // The creep obeys the axis accel/decel limits like any other
      // commanded motion -- a P law is not a license for command steps.
      v = std::clamp(v, previous - limits_.aDecel * dt,
                     previous + limits_.aMax * dt);
      profileVelocity_ = std::fabs(v);
      profileAccel_ = 0.0f;
      cmdLeft_ = v;
      cmdRight_ = v;
      applyHeadingHold();
    } else {
      const float dir = sign(m.omega);
      const float previousOmega =
          (cmdRightPrevious_ - cmdLeftPrevious_) / limits_.trackWidth;
      float omega =
          dir * std::clamp(kCreepGain * measured.anchoredRemaining,
                           -kCreepMaxAngular, kCreepMaxAngular);
      omega = std::clamp(omega, previousOmega - limits_.alphaDecel * dt,
                         previousOmega + limits_.alphaMax * dt);
      profileVelocity_ = std::fabs(omega);
      profileAccel_ = 0.0f;
      const float halfTrack = 0.5f * limits_.trackWidth;
      cmdLeft_ = -omega * halfTrack;
      cmdRight_ = omega * halfTrack;
    }
    return;
  }

  if (m.velocityKind == Move::VelocityKind::Wheels) {
    // Per-wheel ramp toward the commanded pair. Time-bounded Moves taper
    // to rest on the clock; Distance/Angle-bounded Wheels Moves (the wire
    // protocol's other arms) ramp and HOLD -- their completion is the
    // standard measured-threshold test in tick(), and the post-completion
    // drain ramps down (the pre-integration MoveQueue semantics: wheels
    // Moves are direct wheel commands, not profiled landings).
    const float ticksLeft =
        m.kind == Move::Kind::Time ? (m.threshold - elapsed) / period
                                   : 1.0e9f;
    const float accelStep = limits_.aMax * dt;
    const float decelStep = limits_.aDecel * dt;
    const float vCap = limits_.vMax;  // [mm/s] wheel-space ceiling
    cmdLeft_ = timedRamp(cmdLeft_, std::clamp(m.vLeft, -vCap, vCap), 0.0f,
                         accelStep, decelStep, ticksLeft);
    cmdRight_ = timedRamp(cmdRight_, std::clamp(m.vRight, -vCap, vCap), 0.0f,
                          accelStep, decelStep, ticksLeft);
    activeBoundary_ = 0.0f;
    return;
  }

  switch (m.kind) {
    case Move::Kind::Distance: {
      const float dir = sign(m.v_x);
      const AxisLimits lin{limits_.vMax, limits_.aMax, limits_.aDecel,
                           limits_.jerkMax};
      activeBoundary_ = boundaryVelocity(dt);
      // Plan from the max of the last command and the MEASURED speed: on
      // a lagging plant the body genuinely runs faster than the command
      // during decel (lag ~a*tau), and braking feasibility must hold for
      // the TRUE state or the landing starts too late (measured as gross
      // turn overshoot the moment the sim mirrored the real schedule).
      const ProfileResult step = profileStep(
          measured.plannedRemaining, profileVelocity_, std::fabs(m.v_x),
          activeBoundary_, lin, dt, profileAccel_);
      profileAccel_ = (step.velocity - profileVelocity_) / dt;
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
                           limits_.alphaDecel, limits_.yawJerkMax};
      activeBoundary_ = boundaryVelocity(dt);
      const ProfileResult step = profileStep(
          measured.plannedRemaining, profileVelocity_, std::fabs(m.omega),
          activeBoundary_, ang, dt, profileAccel_);
      profileAccel_ = (step.velocity - profileVelocity_) / dt;
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
      const float v = timedRamp(vPrev,
                                std::clamp(m.v_x, -limits_.vMax, limits_.vMax),
                                sign(m.v_x) * activeBoundary_,
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

void Planner::update(Types::RobotState& state) const {
  state.wheelLeft.cmdVelocity = cmdLeft_;
  state.wheelRight.cmdVelocity = cmdRight_;
  // The real (124) Command section carries mode + the commanded twist, not
  // a move id -- completion/ack identity rides TickResult, never the state.
  state.command.moveActive = active_.occupied;
  state.command.mode =
      active_.occupied ? Types::Mode::Velocity : Types::Mode::Idle;
  state.command.v_x = 0.5f * (cmdLeft_ + cmdRight_);
  state.command.omega = (cmdRight_ - cmdLeft_) / limits_.trackWidth;

  const float bodyVelocity = 0.5f * (left_.velocity() + right_.velocity());
  const float omegaBody =
      (right_.velocity() - left_.velocity()) / limits_.trackWidth;
  state.pose.x = pose_.x();
  state.pose.y = pose_.y();
  state.pose.heading = pose_.heading();
  state.pose.v_x = bodyVelocity;
  state.pose.v_y = 0.0f;
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
