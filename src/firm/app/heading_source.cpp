// heading_source.cpp -- App::HeadingSource implementation. See
// heading_source.h for the module's boundary, policy, and cadence.
#include "app/heading_source.h"

namespace App {

HeadingSource::HeadingSource(Devices::Otos& otos, Devices::NezhaMotor& left,
                             Devices::NezhaMotor& right, float trackWidth)
    : otos_(otos), left_(left), right_(right), trackWidth_(trackWidth) {}

void HeadingSource::configure(const msg::PlannerConfig& config) {
  mode_ = config.heading_source;
  headingLeadBias_ = config.heading_lead_bias;  // [s] 109-010 locus 1
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

void HeadingSource::sample(uint64_t nowUs) {
  fellBackEdge_ = false;
  recoveredEdge_ = false;

  // 109-010 measurement-age tracker: independent of the AUTO/FORCE policy
  // below -- tracks "how stale is OTOS's own cached pose right now",
  // regardless of whether this class is currently trusting it. See
  // heading_source.h's own "measurement-age projection" doc comment for why
  // this is `nowUs - otos_.lastReadUs()` (the REAL cycle-ordering gap), not
  // a poseFresh()-gated cycle counter (that tracker measured zero effect --
  // it only sees Otos's own internal read-skip, not the dominant cross-
  // cycle "Pilot reads before this cycle's own OTOS refresh" ordering gap).
  // Clamped to >= 0 -- a misused/defaulted nowUs (e.g. a pre-109-010 test
  // caller that never passes one) must never produce a negative-wrapping
  // uint64_t subtraction turned into a huge bogus lead.
  uint64_t lastReadUs = otos_.lastReadUs();
  ageS_ = (nowUs > lastReadUs) ? static_cast<float>(nowUs - lastReadUs) / 1.0e6f : 0.0f;

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

float HeadingSource::headingLead() const {
  // 109-010 locus 1: see heading_source.h's own "measurement-age
  // projection" doc comment. Encoder fallback has no analogous cross-cycle
  // read-then-consume ordering gap -- collapses to heading() unchanged.
  if (!usingOtos_) return heading();
  return otos_.pose().heading + otos_.pose().omega * (ageS_ + headingLeadBias_);
}

}  // namespace App
