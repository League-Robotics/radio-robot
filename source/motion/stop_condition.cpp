// stop_condition.cpp -- Motion::evaluateStopCondition implementation. See
// stop_condition.h for the class-level design notes.
#include "motion/stop_condition.h"

#include <math.h>

namespace Motion {

namespace {

// wrapAngle -- wrap x into (-pi, pi]. Same atan2f(sinf, cosf) identity
// source_old/control/StopCondition.cpp's wrap_angle() and
// source/subsystems/pose_estimator.cpp's wrapPi() both use.
float wrapAngle(float x) { return atan2f(sinf(x), cosf(x)); }

}  // namespace

StopEvalResult evaluateStopCondition(const msg::StopCondition& cond,
                                     const MotionBaseline& base, uint32_t now,
                                     const msg::MotorState& leftObs,
                                     const msg::MotorState& rightObs,
                                     const msg::PoseEstimate& fusedPose) {
  switch (cond.kind) {
    case msg::StopKind::STOP_NONE:
      return StopEvalResult::NOT_FIRED;

    case msg::StopKind::STOP_TIME: {
      // `a` = threshold, ms. Signed delta guards against uint32 underflow
      // when now is momentarily less than t0 (matches the ported source's
      // watchdog-safe pattern).
      int32_t elapsed = static_cast<int32_t>(now - base.t0);
      return (elapsed >= static_cast<int32_t>(cond.a)) ? StopEvalResult::FIRED
                                                        : StopEvalResult::NOT_FIRED;
    }

    case msg::StopKind::STOP_DISTANCE: {
      // `a` = distance threshold, mm. Needs both wheels' encoder position
      // this tick -- if either is absent, report NOT_FIRED rather than
      // diffing against a fabricated zero baseline.
      if (!leftObs.position.has || !rightObs.position.has) {
        return StopEvalResult::NOT_FIRED;
      }
      float encAvg = (leftObs.position.val + rightObs.position.val) * 0.5f;
      float raw = encAvg - base.enc0;
      // Signed/direction-aware (072-002 concept): project the undirected raw
      // delta onto the commanded direction via base.vSign so travel in the
      // WRONG direction never satisfies `>= a`. vSign == 0.0 (no commanded
      // direction latched) falls back to the undirected |raw| magnitude.
      float signedTraveled = (base.vSign != 0.0f) ? (raw * base.vSign) : fabsf(raw);
      return (signedTraveled >= cond.a) ? StopEvalResult::FIRED : StopEvalResult::NOT_FIRED;
    }

    case msg::StopKind::STOP_HEADING: {
      // `a` = target heading delta from baseline, rad; `b` = eps, rad.
      float currentDelta = wrapAngle(fusedPose.pose.h - base.heading0);
      float error = wrapAngle(currentDelta - cond.a);
      return (fabsf(error) < cond.b) ? StopEvalResult::FIRED : StopEvalResult::NOT_FIRED;
    }

    case msg::StopKind::STOP_POSITION: {
      // `ax` = target X, mm; `a` = target Y, mm; `b` = arrival radius, mm.
      float dx = fusedPose.pose.x - cond.ax;
      float dy = fusedPose.pose.y - cond.a;
      float dist2 = dx * dx + dy * dy;
      return (dist2 < (cond.b * cond.b)) ? StopEvalResult::FIRED : StopEvalResult::NOT_FIRED;
    }

    case msg::StopKind::STOP_ROTATION: {
      // `a` = target per-wheel arc, mm. The encoder DIFFERENTIAL
      // (right - left) tracks rotation for a spin (the sum, used by
      // STOP_DISTANCE, stays ~0). Per-wheel arc = |delta diff| / 2.
      if (!leftObs.position.has || !rightObs.position.has) {
        return StopEvalResult::NOT_FIRED;
      }
      float diff = (rightObs.position.val - leftObs.position.val) - base.encDiff0;
      float signedDiff = (base.omegaSign != 0.0f) ? (diff * base.omegaSign) : fabsf(diff);
      return ((signedDiff * 0.5f) >= cond.a) ? StopEvalResult::FIRED : StopEvalResult::NOT_FIRED;
    }

    case msg::StopKind::STOP_SENSOR:
    case msg::StopKind::STOP_COLOR:
    case msg::StopKind::STOP_LINE_ANY:
      // No line/color sensor Hal leaf exists yet (architecture-update.md
      // Decision 4) -- recognized, not silently never-firing.
      return StopEvalResult::UNSUPPORTED;
  }

  return StopEvalResult::NOT_FIRED;  // unreachable; silences -Wreturn-type
}

}  // namespace Motion
