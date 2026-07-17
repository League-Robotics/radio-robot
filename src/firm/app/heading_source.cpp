// heading_source.cpp -- App::HeadingSource implementation. See
// heading_source.h for the module's boundary, policy, and cadence.
#include "app/heading_source.h"

namespace App {

HeadingSource::HeadingSource(Devices::Otos& otos, Devices::NezhaMotor& left,
                             Devices::NezhaMotor& right, float trackWidth)
    : otos_(otos), left_(left), right_(right), trackWidth_(trackWidth) {}

void HeadingSource::configure(const msg::PlannerConfig& config) {
  mode_ = config.heading_source;
  // A forced mode takes effect immediately (not just on the next sample())
  // so a caller that configure()s mid-session (a future live-tuning path)
  // doesn't leave the OLD source active for one extra cycle.
  if (mode_ == msg::HeadingSourceMode::HEADING_SOURCE_FORCE_OTOS) {
    usingOtos_ = true;
  } else if (mode_ == msg::HeadingSourceMode::HEADING_SOURCE_FORCE_ENCODER) {
    usingOtos_ = false;
  }
}

bool HeadingSource::otosUsable() const {
  return otos_.present() && otos_.connected() && otos_.poseFresh();
}

float HeadingSource::encoderHeading() const {
  return (right_.position() - left_.position()) / trackWidth_;
}

void HeadingSource::sample() {
  fellBackEdge_ = false;
  recoveredEdge_ = false;

  if (mode_ == msg::HeadingSourceMode::HEADING_SOURCE_FORCE_OTOS) {
    usingOtos_ = true;
    return;
  }
  if (mode_ == msg::HeadingSourceMode::HEADING_SOURCE_FORCE_ENCODER) {
    usingOtos_ = false;
    return;
  }

  // AUTO policy (file header): otosUsable() true -> immediate re-promotion
  // (no hysteresis on the recovery side); otosUsable() false for
  // kFallbackStaleCycles CONSECUTIVE sample() calls -> demote.
  if (otosUsable()) {
    staleCount_ = 0;
    if (!usingOtos_) {
      usingOtos_ = true;
      recoveredEdge_ = true;
    }
    return;
  }

  if (usingOtos_) {
    ++staleCount_;
    if (staleCount_ >= kFallbackStaleCycles) {
      usingOtos_ = false;
      staleCount_ = 0;
      fellBackEdge_ = true;
    }
  }
  // Already on encoder and still stale -- nothing changes; staleCount_ is
  // only meaningful while usingOtos_ is true (the countdown to a demotion),
  // so it is deliberately not incremented once already demoted.
}

float HeadingSource::heading() const {
  return usingOtos_ ? otos_.pose().heading : encoderHeading();
}

}  // namespace App
