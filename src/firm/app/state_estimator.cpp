// state_estimator.cpp -- App::StateEstimator implementation. See
// state_estimator.h's file header for the module's boundary and rationale.
#include "app/state_estimator.h"

#include <cmath>

namespace App {

StateEstimator::StateEstimator(FusionWeights weights) : weights_(weights) {}

void StateEstimator::update(const Telemetry::Frame& frame, uint32_t now) {  // [ms]
  // Wheel peers -- always refreshed straight from this cycle's already-
  // staged EncoderReading (position, velocity, its own collect time). Each
  // wheel is its own independent peer -- no cross-wheel dependency here.
  wheelLeft_.distance = frame.encLeft.position;
  wheelLeft_.velocity = frame.encLeft.velocity;
  wheelLeft_.basisTime = frame.encLeft.time;
  wheelLeft_.valid = true;

  wheelRight_.distance = frame.encRight.position;
  wheelRight_.velocity = frame.encRight.velocity;
  wheelRight_.basisTime = frame.encRight.time;
  wheelRight_.valid = true;

  // Body peer -- x/y/v_x/v_y always come straight from Odometry's own
  // dead-reckoned frame.pose/frame.twist (never OTOS-blended this sprint --
  // see BodyEstimate's own doc comment). heading/omega start from the SAME
  // encoder-derived values, then blend toward a fresh OTOS reading via the
  // v1 complementary weight.
  float heading = frame.pose.h;
  float omega = frame.twist.omega;

  // Eligible to blend this cycle iff the frame's own per-cycle freshness
  // bit is set (frame.otosPresent -- "this cycle's burst actually
  // refreshed the cached pose", odometry.h's own applyOtosSample() doc
  // comment) AND the reading's own age is within the live staleness
  // window. `now >= frame.otos.time` guards the unsigned subtract below --
  // both are the same [ms] robot-clock domain by construction (frame.otos
  // is stamped by the SAME cycle's applyOtosSample() call whenever
  // otosPresent is true), so this should hold whenever otosPresent does.
  bool otosFresh = frame.otosPresent && (now >= frame.otos.time) &&
                    ((now - frame.otos.time) <= weights_.staleness);
  if (otosFresh) {
    // Innovations are computed whenever a fresh OTOS reading is blended --
    // even at weight 0.0 (diagnostic/validation only at that weight; the
    // residual itself never feeds back into the estimate at v1). Computed
    // against the PRE-blend (pure encoder-derived) heading/omega, matching
    // "OTOS-vs-predicted" -- the prediction being compared against is this
    // cycle's own encoder-only estimate, before any OTOS influence.
    innovations_.heading = frame.otos.heading - heading;
    innovations_.omega = frame.otos.omega - omega;
    innovations_.valid = true;

    heading = heading + weights_.headingOtos * (frame.otos.heading - heading);
    omega = omega + weights_.omegaOtos * (frame.otos.omega - omega);
  }

  body_.x = frame.pose.x;
  body_.y = frame.pose.y;
  body_.heading = heading;
  body_.v_x = frame.twist.v_x;
  body_.v_y = frame.twist.v_y;
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

}  // namespace App
