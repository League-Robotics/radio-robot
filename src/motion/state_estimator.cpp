// state_estimator.cpp -- Motion::StateEstimator implementation. See
// state_estimator.h's file header for the module's boundary and rationale.
#include "motion/state_estimator.h"

#include <cmath>

namespace Motion {

StateEstimator::StateEstimator(FusionWeights weights) : weights_(weights) {}

void StateEstimator::update(const Input& input, uint32_t now) {  // [ms]
  // Wheel peers -- always refreshed straight from this cycle's already-
  // staged encoder reading (position, velocity, its own sample time). Each
  // wheel is its own independent peer -- no cross-wheel dependency here.
  wheelLeft_.distance = input.wheelLeft.position;
  wheelLeft_.velocity = input.wheelLeft.velocity;
  wheelLeft_.basisTime = input.wheelLeft.sampleTime;
  wheelLeft_.valid = true;

  wheelRight_.distance = input.wheelRight.position;
  wheelRight_.velocity = input.wheelRight.velocity;
  wheelRight_.basisTime = input.wheelRight.sampleTime;
  wheelRight_.valid = true;

  // Body peer -- x/y/v_x/v_y always come straight from Motion::Odometry's
  // own dead-reckoned pose/twist (never OTOS-blended this sprint -- see
  // BodyEstimate's own doc comment). heading/omega start from the SAME
  // encoder-derived values, then blend toward a fresh OTOS reading via the
  // v1 complementary weight.
  float heading = input.pose.heading;
  float omega = input.pose.omega;

  // Eligible to blend this cycle iff the frame's own per-cycle freshness
  // bit is set (input.otos.present -- "this cycle's burst actually
  // refreshed the cached pose", App::applyOtosSample()'s own doc comment)
  // AND the reading's own age is within the live staleness window.
  // `now >= input.otos.sampleTime` guards the unsigned subtract below --
  // both are the same [ms] robot-clock domain by construction
  // (input.otos.sampleTime is stamped by the SAME cycle's
  // applyOtosSample() call whenever `present` is true), so this should
  // hold whenever `present` does.
  bool otosFresh = input.otos.present && (now >= input.otos.sampleTime) &&
                    ((now - input.otos.sampleTime) <= weights_.staleness);
  if (otosFresh) {
    // Innovations are computed whenever a fresh OTOS reading is blended --
    // even at weight 0.0 (diagnostic/validation only at that weight; the
    // residual itself never feeds back into the estimate at v1). Computed
    // against the PRE-blend (pure encoder-derived) heading/omega, matching
    // "OTOS-vs-predicted" -- the prediction being compared against is this
    // cycle's own encoder-only estimate, before any OTOS influence.
    innovations_.heading = input.otos.heading - heading;
    innovations_.omega = input.otos.omega - omega;
    innovations_.valid = true;

    heading = heading + weights_.headingOtos * (input.otos.heading - heading);
    omega = omega + weights_.omegaOtos * (input.otos.omega - omega);
  }

  body_.x = input.pose.x;
  body_.y = input.pose.y;
  body_.heading = heading;
  body_.v_x = input.pose.v_x;
  body_.v_y = input.pose.v_y;
  body_.omega = omega;
  body_.basisTime = now;
  body_.valid = true;
}

WheelEstimate StateEstimator::wheelAt(Wheel wheel, uint32_t t) const {  // [ms]
  const WheelEstimate& basis = (wheel == Wheel::Left) ? wheelLeft_ : wheelRight_;

  WheelEstimate out = basis;
  if (!basis.valid) return out;

  // Age math: one integer subtract cast to seconds, no 64-bit divides per
  // query (mirrors Motion::StopCondition's own "convert once" precedent).
  // Precondition (this method's own doc comment): t is at or after basis.
  uint32_t ageMs = t - basis.basisTime;
  float age = static_cast<float>(ageMs) / 1000.0f;  // [s]

  out.distance = basis.distance + basis.velocity * age;
  // velocity/basisTime/valid already carried over from `out = basis` above
  // (velocity held constant under ZOH; basisTime stays the ORIGINAL basis
  // reading's timestamp -- see this method's own doc comment).
  return out;
}

BodyEstimate StateEstimator::bodyAt(uint32_t t) const {  // [ms]
  BodyEstimate out = body_;
  if (!body_.valid) return out;

  uint32_t ageMs = t - body_.basisTime;
  float age = static_cast<float>(ageMs) / 1000.0f;  // [s]

  // Project the held-constant body-frame (v_x, v_y) into world frame using
  // the BASIS heading (first-order approximation, valid for the small ages
  // this sprint's every-cycle basis refresh produces -- see this class's
  // own file header).
  float cosH = cosf(body_.heading);
  float sinH = sinf(body_.heading);
  out.x = body_.x + (body_.v_x * cosH - body_.v_y * sinH) * age;
  out.y = body_.y + (body_.v_x * sinH + body_.v_y * cosH) * age;
  out.heading = body_.heading + body_.omega * age;
  // v_x/v_y/omega/basisTime/valid already carried over from `out = body_`
  // above (all held constant under ZOH; basisTime stays the ORIGINAL basis
  // reading's timestamp -- see this method's own doc comment).
  return out;
}

BodyEstimate StateEstimator::whereAmI(uint32_t now) const { return bodyAt(now); }  // [ms]

WheelEstimate StateEstimator::wheelNow(Wheel wheel) const {
  return (wheel == Wheel::Left) ? wheelLeft_ : wheelRight_;
}

void StateEstimator::reset(float x, float y, float heading) {  // [mm] [mm] [rad]
  body_.x = x;
  body_.y = y;
  body_.heading = heading;
  // v_x/v_y/omega/basisTime/valid deliberately untouched -- see this
  // method's own doc comment (state_estimator.h).
}

}  // namespace Motion
