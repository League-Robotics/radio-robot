#include "wheel_plant.h"

#include <cmath>

namespace TestSim {

WheelPlant::WheelPlant(float dutyVelMax, float tau)
    : dutyVelMax_(dutyVelMax), tau_(tau) {}

void WheelPlant::step(float appliedDuty, float dt) {
  // Exact discretization of the continuous first-order lag
  // dv/dt = (target - v) / tau_ over one step of length dt -- see
  // wheel_plant.h's own comment for the derivation.
  float target = dutyVelMax_ * appliedDuty;
  float alpha = 1.0f - std::exp(-dt / tau_);
  velocity_ += (target - velocity_) * alpha;
  position_ += velocity_ * dt;
}

float WheelPlant::reportedPosition() {
  // Fault-knob precedence: freeze wins outright (an explicitly frozen
  // reading is never itself subject to dropout-driven staleness -- there is
  // nothing "fresher" to fall back to while frozen). Otherwise the dropout
  // accumulator decides fresh-vs-held for this call. Neither knob ever
  // touches position_/velocity_ -- step()'s own integration is untouched.
  float reportPosition;
  if (freezePosition_) {
    reportPosition = frozenPosition_;
  } else if (dropoutRate_ > 0.0f) {
    dropoutAccum_ += dropoutRate_;
    if (dropoutAccum_ >= 1.0f) {
      dropoutAccum_ -= 1.0f;
      reportPosition = lastReportedPosition_;   // hold: stale-not-fresh
    } else {
      reportPosition = position_;
    }
  } else if (encoderJitter_ && std::fabs(velocity_) < kRestVelocityThreshold) {
    // At rest: dither by one wire LSB, flipping sign only once every
    // kDitherPeriod calls (held steady in between) -- see kDitherPeriod's
    // own comment in wheel_plant.h for why every-call alternation (the
    // 108-011 original) is wrong. position_ itself (plant truth) is never
    // touched.
    reportPosition = position_ + (ditherPhase_ ? kDitherLsb : -kDitherLsb);
    if (++ditherCounter_ >= kDitherPeriod) {
      ditherCounter_ = 0;
      ditherPhase_ = !ditherPhase_;
    }
  } else {
    // Moving again -- next rest period starts its dither cycle fresh.
    ditherCounter_ = 0;
    reportPosition = position_;
  }
  // Scale error applies next, uniformly across every branch above (fresh,
  // held/dropout, frozen, or dithered) -- a fractional over/under-report
  // bias on whatever value was otherwise going to be reported, never
  // touching position_/velocity_ themselves. 0.0 is a genuine no-op.
  reportPosition *= (1.0f + scaleErr_);

  // Slip events (109-007): the accumulator advances on EVERY call (mirrors
  // dropoutAccum_'s own unconditional-advance design) and, once it crosses
  // 1.0, injects a PERMANENT signed offset -- a real slip event, once it
  // happens, never un-happens on its own. 0.0 rate is a genuine no-op
  // (slipOffset_ stays 0 forever). Applied before quantization -- see
  // setTickQuantization()'s own header comment for the ordering rationale.
  if (slipRate_ > 0.0f) {
    slipAccum_ += slipRate_;
    if (slipAccum_ >= 1.0f) {
      slipAccum_ -= 1.0f;
      slipOffset_ += slipMagnitude_;
    }
  }
  reportPosition += slipOffset_;

  // Tick quantization (109-007) -- applied LAST: a real encoder's own
  // finite count resolution is the final step between "whatever physical/
  // fault-biased value the chip would otherwise report" and the actual
  // wire reading. 0.0 is a genuine no-op.
  if (tickSize_ > 0.0f) {
    reportPosition = std::round(reportPosition / tickSize_) * tickSize_;
  }

  lastReportedPosition_ = reportPosition;
  return reportPosition;
}

void WheelPlant::freezePosition(bool freeze) {
  if (freeze && !freezePosition_) {
    frozenPosition_ = position_;   // capture only on the rising edge
  }
  freezePosition_ = freeze;
}

void WheelPlant::setDropoutRate(float fraction) {
  dropoutRate_ = fraction;
  dropoutAccum_ = 0.0f;   // fresh phase -- a rate change never inherits a stale one
}

void WheelPlant::setSlip(float rate, float magnitude) {
  slipRate_ = rate;
  slipMagnitude_ = magnitude;
  slipAccum_ = 0.0f;   // fresh phase -- a rate/magnitude change never inherits a stale one
}

}  // namespace TestSim
