// arc_math.h -- Drive:: constant-curvature arc geometry: composeArc,
// poseAlongArc, the exact circle projection (projectOntoArc), and
// wrapAngle. Every function here is pure (no I2C, no global state, no heap
// allocation) -- the same discipline kinematics/body_kinematics.h documents
// for the differential IK/saturation math tracker.{h,cpp} (ticket 100-004)
// hand-ports separately. wrapAngle is hand-ported verbatim from
// motion/stop_condition.cpp's own wrapAngle() (itself already a copy of
// source_old/control/StopCondition.cpp's wrap_angle() and
// subsystems/pose_estimator.cpp's wrapPi() -- the SAME atan2f(sinf, cosf)
// identity copied a fourth time here, per this directory's "copy, don't
// include" rule). composeArc/poseAlongArc/projectOntoArc have no existing
// equivalent elsewhere in this codebase to hand-port from; they are the
// closed-form constant-curvature primitives every other source/drive/
// module (referenceAt()'s reference table, the tracker's path-frame error,
// Drivetrain::advance()'s ChainTail composition) is built on, per
// architecture-update.md (100) M2 and the driving issue's "Supporting
// files" list.
//
// Curvature convention: kappa = deltaHeading / arcLength [1/mm], signed;
// CCW turn (heading increasing with s) is kappa > 0, matching Pose::h's own
// CCW+ convention. kappa == 0 (within kEpsilonKappa) is the straight-line
// limit, handled explicitly below (1/kappa would blow up). A PIVOT (arc
// length == 0, kappa formally undefined -- 0/0) is NOT a case these
// functions handle: pivot pose composition is trivial (position fixed,
// heading offset directly) and is the CALLER's job (Drivetrain/MotionPlan,
// tickets 100-003/005), which already knows it is in pivot mode from its
// own Goal (arcLength == 0) before it would ever need to call into this
// header.
#pragma once
#include "drive/types.h"

namespace Drive {

// wrapAngle -- wrap x into (-pi, pi]. Same atan2f(sinf, cosf) identity as
// motion/stop_condition.cpp's wrapAngle() (see class comment above).
float wrapAngle(float x);

// poseAlongArc -- closed-form pose at signed arc-length coordinate `s`
// along the constant-curvature arc anchored at `anchor`. This is the
// PRIMITIVE both composeArc() and projectOntoArc() below are built on, and
// the function referenceAt()'s per-tick reference table (ticket 100-003)
// samples repeatedly across [0, duration] as its own master-DOF trajectory
// advances -- one call per RefState.
//
// Derivation: constant-curvature unicycle kinematics dx/ds = cos(h(s)),
// dy/ds = sin(h(s)), dh/ds = kappa integrate in closed form to:
//   h(s) = anchor.h + kappa * s
//   x(s) = anchor.x + (sin(h(s)) - sin(anchor.h)) / kappa
//   y(s) = anchor.y - (cos(h(s)) - cos(anchor.h)) / kappa
// (the kappa == 0 straight-line limit of the same integral is
// x(s) = anchor.x + s*cos(anchor.h), y(s) = anchor.y + s*sin(anchor.h),
// h(s) = anchor.h, applied whenever |kappa| < kEpsilonKappa to avoid the
// 1/kappa singularity).
//
// @param anchor  world pose at s == 0
// @param kappa   [1/mm] signed curvature; CCW+ (see class comment)
// @param s       [mm] signed arc-length coordinate from anchor
// @return        world pose at s, heading wrapped to (-pi, pi]
Pose poseAlongArc(const Pose& anchor, float kappa, float s);

// composeArc -- the SAME closed form as poseAlongArc(), named separately
// for its distinct call sites: composing a WHOLE segment's frozen goal pose
// (Drivetrain::plan() freezing the world goal, per the driving issue's
// plan() doc comment: "the world goal pose is composed and frozen into the
// plan here") and ChainTail::advance()'s predicted-tail composition --
// both one-shot, full-arcLength calls, as opposed to poseAlongArc()'s
// repeated partial-`s` sampling.
//
// @param start      world pose at the start of the arc
// @param kappa      [1/mm] signed curvature (see poseAlongArc())
// @param arcLength  [mm] signed total arc length
// @return           world pose at the arc's end
Pose composeArc(const Pose& start, float kappa, float arcLength);

// ArcError -- one measured pose's path-frame error against a
// constant-curvature reference arc, evaluated EXACTLY at reference
// parameter `s` (architecture-update.md's Control-laws section: "errors
// reference-measured; exact arc projection" -- never linearized or
// Euler-integrated). The reference pose is poseAlongArc(anchor, kappa, s)
// itself, so the projection carries zero integration drift no matter how
// sharply the arc curves between the plan's start and `s`. eAlong/eCross
// are the measured pose's world-frame offset from that exact reference
// point, ROTATED into the reference's own tangent/normal frame -- an exact
// SE(2) transform, valid for any offset magnitude (not a small-angle
// approximation): eAlong = dx*cos(theta_ref) + dy*sin(theta_ref),
// eCross = -dx*sin(theta_ref) + dy*cos(theta_ref), where
// (dx, dy) = measured.(x, y) - reference.(x, y).
struct ArcError {
  float eAlong = 0.0f;  // [mm] + ahead of the reference point along its tangent
  float eCross = 0.0f;  // [mm] + LEFT of the reference point's tangent direction
  float eTheta = 0.0f;  // [rad] wrapAngle(measured.h - referenceHeading), in (-pi, pi]
};

// projectOntoArc -- the exact circle projection: composes the reference
// pose at `s` via poseAlongArc(anchor, kappa, s), then rotates `measured`'s
// world-frame offset from it into the tangent/normal frame (see ArcError's
// doc comment). Ticket 100-004's tracker cascade is the primary caller,
// evaluating this once per step() tick against MotionPlan::referenceAt()'s
// own sample.
//
// @param anchor    world pose at s == 0 (the same arc poseAlongArc() uses)
// @param kappa     [1/mm] signed curvature
// @param s         [mm] reference arc-length coordinate (already known --
//                  e.g. RefState.s from referenceAt(t); this function does
//                  NOT search for the nearest point on the arc)
// @param measured  the measured world pose to project
// @return          path-frame error at the exact reference point
ArcError projectOntoArc(const Pose& anchor, float kappa, float s, const Pose& measured);

}  // namespace Drive
