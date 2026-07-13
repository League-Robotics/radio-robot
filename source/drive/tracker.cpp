// tracker.cpp -- Drive:: tracker implementation. See tracker.h for the
// class-level design notes (cascade order, P-only rationale, the
// TrackerOutput/TrackRecord field correspondence).
#include "drive/tracker.h"

#include <math.h>

#include "drive/arc_math.h"

namespace Drive {

namespace {

// clampSymmetric -- clamp `value` to [-limit, limit]; `limit` is a
// non-negative magnitude (the issue's own "clamp(x, +/-bound)" notation).
// Sets *saturated true iff `value` was actually outside the band -- the
// trimSaturated contract ("reported true exactly when a trim was
// clamped").
float clampSymmetric(float value, float limit, bool* saturated) {
  if (value > limit) {
    *saturated = true;
    return limit;
  }
  if (value < -limit) {
    *saturated = true;
    return -limit;
  }
  return value;
}

// inverse -- BodyKinematics::inverse()'s scalar contract, hand-ported
// (source/drive/'s "copy, don't include" rule -- see arc_math.h's own
// class comment for the precedent; source/kinematics/body_kinematics.{h,
// cpp}, outside this directory, stays untouched until the cleanup ticket):
// vL = v - omega*(b/2), vR = v + omega*(b/2).
void inverse(float v, float omega, float trackwidth, float* left, float* right) {
  const float halfTrack = trackwidth * 0.5f;
  *left = v - omega * halfTrack;
  *right = v + omega * halfTrack;
}

// saturate -- BodyKinematics::saturate()'s curvature-preserving contract,
// hand-ported: when max(|left|, |right|) exceeds `ceiling`, BOTH wheels
// scale by the SAME factor so the faster wheel sits exactly at the
// ceiling -- the wheel-speed ratio (and therefore arc curvature) is
// preserved; pass-through otherwise. `ceiling` is limits.vWheelMax
// directly here -- Drivetrain::plan() (ticket 100-003) already folds trim
// headroom into the reference profile's own ceiling before this cascade
// ever runs (its own doc comment: "wheels cannot saturate, and trims keep
// authority at ceiling"), so this step is this cascade's own structural
// safety net, not a normal-operation participant.
void saturate(float left, float right, float ceiling, float* leftOut, float* rightOut) {
  const float absLeft = fabsf(left);
  const float absRight = fabsf(right);
  const float maxAbs = (absLeft > absRight) ? absLeft : absRight;
  if (maxAbs > ceiling && maxAbs > 0.0f) {
    const float scale = ceiling / maxAbs;
    *leftOut = scale * left;
    *rightOut = scale * right;
  } else {
    *leftOut = left;
    *rightOut = right;
  }
}

}  // namespace

TrackerOutput track(const RefState& ref, const BodyState& measured, const Limits& limits,
                     float trackwidth) {
  TrackerOutput out;

  // 1. Exact arc-frame error projection -- against RefState's OWN
  // closed-form pose (x, y, theta), never re-derived from anchor/kappa
  // here (that already happened once, inside referenceAt()). Reuses
  // arc_math's projectOntoPose() -- the same tangent/normal SE(2)
  // rotation projectOntoArc() itself is built on (arc_math.cpp).
  //
  // out.eAlong/eCross/eTheta report arc_math's own, native (measured -
  // reference) convention UNCHANGED -- these are the diagnostic fields
  // TrackRecord (motion_plan.h) copies verbatim. The trim law below uses
  // the NEGATED values (trimEAlong/trimECross/trimETheta = reference -
  // measured) -- see tracker.h's class comment ("Reconciled sign
  // convention") for why: the issue's own control-law section states
  // "errors reference−measured", the opposite of arc_math's convention,
  // and only that sign produces a stable, convergent loop (matches
  // sprint 098's hardware-proven heading loop exactly).
  const Pose referencePose{ref.x, ref.y, ref.theta};
  const ArcError error = projectOntoPose(referencePose, measured.pose);
  out.eAlong = error.eAlong;
  out.eCross = error.eCross;
  out.eTheta = error.eTheta;

  const float trimEAlong = -out.eAlong;
  const float trimECross = -out.eCross;
  const float trimETheta = -out.eTheta;

  const bool pivotMode = fabsf(ref.v) < limits.minSpeed;

  // 2. P-only Kanayama trims (issue's "errors reference-measured"
  // convention -- trimEAlong/trimECross/trimETheta above). Arc mode: both
  // trims clamped to +/-trimVMax/+/-trimOmegaMax. Pivot mode: v_cmd
  // forced to a LITERAL 0.0f (not merely near-zero) and omega's trim is
  // left UNCLAMPED -- matching sprint 098's proven heading loop
  // (tracker.h's class comment; types.h's Limits.trimOmegaMax doc comment
  // records why the issue's table's pivot-specific 2.0 rad/s cap is not
  // wired to a clamp here).
  if (pivotMode) {
    out.vTrim = 0.0f;
    out.vCmd = 0.0f;
    out.omegaTrim = limits.trackKTheta * trimETheta;
    out.omegaCmd = ref.omega + out.omegaTrim;
    out.trimSaturated = false;
  } else {
    bool vSaturated = false;
    bool omegaSaturated = false;

    const float rawVTrim = limits.trackKS * trimEAlong;
    out.vTrim = clampSymmetric(rawVTrim, limits.trimVMax, &vSaturated);
    out.vCmd = ref.v + out.vTrim;

    const float rawOmegaTrim =
        limits.trackKTheta * trimETheta + limits.trackKCross * ref.v * trimECross;
    out.omegaTrim = clampSymmetric(rawOmegaTrim, limits.trimOmegaMax, &omegaSaturated);
    out.omegaCmd = ref.omega + out.omegaTrim;

    out.trimSaturated = vSaturated || omegaSaturated;
  }

  // 3. IK: body twist -> wheel speeds.
  float wheelLeft = 0.0f;
  float wheelRight = 0.0f;
  inverse(out.vCmd, out.omegaCmd, trackwidth, &wheelLeft, &wheelRight);

  // 4. Curvature-preserving saturation.
  saturate(wheelLeft, wheelRight, limits.vWheelMax, &wheelLeft, &wheelRight);

  // 5. One-sided forward-arc wheel clamp -- STRUCTURAL: on a forward arc
  // (not a pivot, ref.v > 0), neither wheel's commanded velocity is ever
  // negative, regardless of how far the trims saturated above. Pivots are
  // exempt (wheels legitimately counter-rotate); a backward arc (ref.v <
  // 0) is also exempt -- this clamp is one-sided by name and by contract,
  // never a symmetric +/- clamp.
  const bool forwardArc = !pivotMode && ref.v > 0.0f;
  if (forwardArc) {
    if (wheelLeft < 0.0f) wheelLeft = 0.0f;
    if (wheelRight < 0.0f) wheelRight = 0.0f;
  }

  out.command.left = wheelLeft;
  out.command.right = wheelRight;
  return out;
}

}  // namespace Drive
