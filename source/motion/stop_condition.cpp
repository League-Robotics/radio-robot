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

// distanceProgress -- STOP_DISTANCE's shared geometry: the signed distance
// traveled (mm) since baseline, projected onto the commanded direction
// (base.vSign), matching evaluateStopCondition()'s own comment on why. False
// (leaving *signedTraveled untouched) when either wheel's encoder position
// observation is absent this tick -- callers must not fabricate a phantom
// zero-baseline delta from that. Shared by evaluateStopCondition() and
// remainingToStop() (086-003, architecture-update.md (086) Decision 2) so
// this geometry is computed in exactly one place.
bool distanceProgress(const MotionBaseline& base, const msg::MotorState& leftObs,
                      const msg::MotorState& rightObs, float* signedTraveled) {
  if (!leftObs.position.has || !rightObs.position.has) return false;
  float encAvg = (leftObs.position.val + rightObs.position.val) * 0.5f;
  float raw = encAvg - base.enc0;
  *signedTraveled = (base.vSign != 0.0f) ? (raw * base.vSign) : fabsf(raw);
  return true;
}

// rotationProgress -- STOP_ROTATION's shared geometry: the signed per-wheel
// arc (mm) traveled since baseline, from the encoder DIFFERENTIAL (right -
// left, which tracks rotation for a spin -- the sum, used by
// distanceProgress(), stays ~0), projected onto the commanded turn direction
// (base.omegaSign). False (leaving *signedArc untouched) when either wheel's
// encoder position observation is absent this tick. Shared the same way
// distanceProgress() is (see above).
bool rotationProgress(const MotionBaseline& base, const msg::MotorState& leftObs,
                      const msg::MotorState& rightObs, float* signedArc) {
  if (!leftObs.position.has || !rightObs.position.has) return false;
  float diff = (rightObs.position.val - leftObs.position.val) - base.encDiff0;
  float signedDiff = (base.omegaSign != 0.0f) ? (diff * base.omegaSign) : fabsf(diff);
  *signedArc = signedDiff * 0.5f;
  return true;
}

// headingError -- STOP_HEADING's shared geometry: the wrapped difference
// (rad) between how far the fused heading has actually rotated since
// baseline and the goal's target rotation `target` (cond.a) -- shrinks to
// (near) zero as the turn approaches its target, regardless of turn
// direction. Shared the same way distanceProgress()/rotationProgress() are.
float headingError(const MotionBaseline& base, const msg::PoseEstimate& fusedPose, float target) {
  float currentDelta = wrapAngle(fusedPose.pose.h - base.heading0);
  return wrapAngle(currentDelta - target);
}

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
      float signedTraveled = 0.0f;
      if (!distanceProgress(base, leftObs, rightObs, &signedTraveled)) {
        return StopEvalResult::NOT_FIRED;
      }
      return (signedTraveled >= cond.a) ? StopEvalResult::FIRED : StopEvalResult::NOT_FIRED;
    }

    case msg::StopKind::STOP_HEADING: {
      // `a` = target heading delta from baseline, rad; `b` = eps, rad.
      float error = headingError(base, fusedPose, cond.a);
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
      float signedArc = 0.0f;
      if (!rotationProgress(base, leftObs, rightObs, &signedArc)) {
        return StopEvalResult::NOT_FIRED;
      }
      return (signedArc >= cond.a) ? StopEvalResult::FIRED : StopEvalResult::NOT_FIRED;
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

StopEvalResult remainingToStop(const msg::StopCondition& cond, const MotionBaseline& base,
                               const msg::MotorState& leftObs, const msg::MotorState& rightObs,
                               const msg::PoseEstimate& fusedPose, float* remaining) {
  switch (cond.kind) {
    case msg::StopKind::STOP_DISTANCE: {
      float signedTraveled = 0.0f;
      if (!distanceProgress(base, leftObs, rightObs, &signedTraveled)) {
        // No encoder reading yet this tick -- conservatively report the
        // full distance as still remaining (see stop_condition.h's doc
        // comment) rather than an uninitialized/phantom value.
        *remaining = fabsf(cond.a);
        return StopEvalResult::NOT_FIRED;
      }
      float rem = cond.a - signedTraveled;
      *remaining = (rem > 0.0f) ? rem : 0.0f;
      return (signedTraveled >= cond.a) ? StopEvalResult::FIRED : StopEvalResult::NOT_FIRED;
    }

    case msg::StopKind::STOP_ROTATION: {
      float signedArc = 0.0f;
      if (!rotationProgress(base, leftObs, rightObs, &signedArc)) {
        *remaining = fabsf(cond.a);
        return StopEvalResult::NOT_FIRED;
      }
      float rem = cond.a - signedArc;
      *remaining = (rem > 0.0f) ? rem : 0.0f;
      return (signedArc >= cond.a) ? StopEvalResult::FIRED : StopEvalResult::NOT_FIRED;
    }

    case msg::StopKind::STOP_HEADING: {
      float error = headingError(base, fusedPose, cond.a);
      *remaining = fabsf(error);
      return (fabsf(error) < cond.b) ? StopEvalResult::FIRED : StopEvalResult::NOT_FIRED;
    }

    default:
      // STOP_TIME/STOP_POSITION/STOP_SENSOR/STOP_COLOR/STOP_LINE_ANY/
      // STOP_NONE -- no "remaining distance/angle" concept applies (see
      // stop_condition.h's doc comment on why STOP_POSITION is excluded).
      return StopEvalResult::UNSUPPORTED;
  }
}

}  // namespace Motion
