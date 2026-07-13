// arc_math.cpp -- Drive:: arc_math implementation. See arc_math.h for the
// class-level design notes (curvature convention, the closed-form
// derivation, the exact-projection contract).
#include "drive/arc_math.h"

#include <math.h>

namespace Drive {

namespace {

// kEpsilonKappa -- below this magnitude, 1/kappa's straight-line limit is
// used instead of the circular-arc closed form, to avoid the 1/kappa
// singularity. 1e-6 [1/mm] corresponds to a ~1 km radius -- any real
// segment at that radius is indistinguishable from straight at this
// robot's scale.
constexpr float kEpsilonKappa = 1.0e-6f;

}  // namespace

float wrapAngle(float x) { return atan2f(sinf(x), cosf(x)); }

Pose poseAlongArc(const Pose& anchor, float kappa, float s) {
  Pose result;
  if (fabsf(kappa) < kEpsilonKappa) {
    // Straight-line limit: dx/ds = cos(h), dy/ds = sin(h), dh/ds = 0.
    result.x = anchor.x + s * cosf(anchor.h);
    result.y = anchor.y + s * sinf(anchor.h);
    result.h = wrapAngle(anchor.h);
    return result;
  }
  const float h = anchor.h + kappa * s;
  result.x = anchor.x + (sinf(h) - sinf(anchor.h)) / kappa;
  result.y = anchor.y - (cosf(h) - cosf(anchor.h)) / kappa;
  result.h = wrapAngle(h);
  return result;
}

Pose composeArc(const Pose& start, float kappa, float arcLength) {
  // Same closed form as poseAlongArc() -- see arc_math.h's doc comment on
  // why these are two named entry points over one shared implementation.
  return poseAlongArc(start, kappa, arcLength);
}

ArcError projectOntoPose(const Pose& reference, const Pose& measured) {
  const float dx = measured.x - reference.x;
  const float dy = measured.y - reference.y;
  const float cosTheta = cosf(reference.h);
  const float sinTheta = sinf(reference.h);

  ArcError error;
  error.eAlong = dx * cosTheta + dy * sinTheta;
  error.eCross = -dx * sinTheta + dy * cosTheta;
  error.eTheta = wrapAngle(measured.h - reference.h);
  return error;
}

ArcError projectOntoArc(const Pose& anchor, float kappa, float s, const Pose& measured) {
  const Pose reference = poseAlongArc(anchor, kappa, s);
  return projectOntoPose(reference, measured);
}

}  // namespace Drive
