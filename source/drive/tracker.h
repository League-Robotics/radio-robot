// tracker.h -- Drive:: pure per-tick tracking control law: converts one
// tick's reference-vs-measured error into a wheel-velocity command.
// Responsibility 4 from architecture-update.md (100) Step 2/M4 -- "how a
// reference-vs-measured error becomes a wheel-velocity command" --
// deliberately separated from responsibility 5 (policy/terminal decision,
// ticket 100-005, policy.{h,cpp}/motion_plan.cpp): this file answers HOW,
// never WHEN or WHAT STATUS. No StepState, no Status decision anywhere in
// this file -- see motion_plan.h's own StepState/Status, both untouched
// here.
//
// Cascade (fixed order, per the driving issue's "Control laws and numbers"
// section and architecture-update.md M4): reference sample (already done
// by the CALLER -- MotionPlan::referenceAt(), ticket 100-003, before this
// cascade ever runs) -> exact arc-frame error projection
// (arc_math::projectOntoPose(), against RefState's own closed-form x/y/
// theta -- never a linearized approximation) -> P-only Kanayama trims
// (clamped in arc mode) -> IK (BodyKinematics-equivalent inverse(),
// hand-ported below -- source/drive/'s "copy, don't include" rule, same as
// arc_math.h's own precedent) -> curvature-preserving saturation
// (BodyKinematics::saturate()'s "scale both wheels by the same factor so
// the faster wheel sits exactly at the ceiling" contract, hand-ported) ->
// one-sided forward-arc wheel clamp -> wheel velocity setpoints.
//
// P-only, no derivative, no integral (architecture-update.md's "P-only
// outer loops" rule; the issue's explicit k_d = 0 -- not shipped --
// rationale: "encoder omega-hat is 0-80ms-stale staggered noise" -- the
// LEVEL-2 wheel velocity PID, unchanged and outside this directory, owns
// steady-state authority instead). This file carries NO derivative term
// and NO integral/accumulator field anywhere -- grep-verifiable
// (test_drive_tracker.py asserts this directly, mirroring
// test_drive_isolation.py's forbidden-token pattern).
//
// -- Reconciled sign convention (ticket 100-004 finding, load-bearing) --
// The issue's own "Control laws and numbers" section headlines the trim
// law "errors reference−measured" -- the OPPOSITE sign of arc_math.h's
// already-committed, test-locked ArcError convention ("dx, dy =
// measured.(x, y) - reference.(x, y)", ticket 100-002). Applying the
// trim-law formulas literally to arc_math's (measured - reference) sign is
// NOT a stylistic difference: linearizing the closed loop around a
// straight reference (d(eTheta)/dt = omegaCmd, d(eCross)/dt =
// v_ref*eTheta) shows the pivot loop's own matrix has trace == +trackKTheta
// for ANY positive gain when fed arc_math's raw eTheta -- unconditionally
// unstable, independent of k_c's sign. Sprint 098's actual, hardware-
// proven heading loop (motion/segment_executor.cpp: `omega =
// desired.velocity + heading_kp * (desired.position - thetaMeasured) +
// ...`) computes its proportional term on (reference - measured), the
// SAME sign this file now uses -- confirming the issue's "errors
// reference−measured" phrase, not the literal e_theta/e_along/e_cross
// symbol names taken at arc_math's own sign, is the intended contract.
// track()'s implementation therefore negates arc_math's projectOntoPose()
// result BEFORE applying the trim law (tracker.cpp), while
// TrackerOutput's own eAlong/eCross/eTheta fields below still report
// arc_math's native (measured - reference) convention UNCHANGED -- the
// same values TrackRecord (motion_plan.h) already documents as "exact arc
// projection" and that ticket 100-002's own tests lock in. Only the
// INTERNAL trim-law inputs are sign-flipped; nothing about the arc_math
// public contract changed.
#pragma once

#include "drive/motion_plan.h"  // RefState
#include "drive/types.h"        // BodyState, Limits, WheelVelocities

namespace Drive {

// TrackerOutput -- the cascade's full result: the final wheel-velocity
// command (StepOutput.command's own eventual source, once ticket 100-005
// wires MotionPlan::step() through this cascade) plus every diagnostic
// intermediate motion_plan.h's TrackRecord needs. Field names below match
// TrackRecord's own eAlong/eCross/eTheta/vTrim/omegaTrim/vCmd/omegaCmd/
// trimSaturated exactly, so ticket 100-005 can copy them straight across;
// `command` carries what TrackRecord flattens into wheelLeft/wheelRight.
struct TrackerOutput {
  float eAlong = 0.0f;  // [mm] exact arc-frame along-track error (measured
                         // - reference, tangent-frame)
  float eCross = 0.0f;  // [mm] exact arc-frame cross-track error (measured
                         // - reference, left-normal-frame)
  float eTheta = 0.0f;  // [rad] heading error, wrapped to (-pi, pi]

  float vTrim = 0.0f;      // [mm/s] post-clamp along-track trim; a literal
                            // 0.0f in pivot mode (no along-track trim is
                            // computed there at all -- vCmd is forced
                            // directly, not via a zero trim)
  float omegaTrim = 0.0f;  // [rad/s] arc mode: post-clamp heading+cross
                            // trim. Pivot mode: trackKTheta * eTheta,
                            // UNCLAMPED (matches sprint 098's proven
                            // heading loop -- see tracker.cpp)

  float vCmd = 0.0f;      // [mm/s] body speed command (v_ref + vTrim);
                           // LITERAL 0.0f in pivot mode, not merely near-zero
  float omegaCmd = 0.0f;  // [rad/s] body yaw-rate command (omega_ref + omegaTrim)

  WheelVelocities command;  // [mm/s] final setpoints, post IK / curvature-
                             // preserving-saturate / one-sided-forward-arc-clamp

  bool trimSaturated = false;  // true iff vTrim or omegaTrim was clamped
                                // this tick. Always false in pivot mode --
                                // pivot's omegaTrim is never clamped
                                // (tracker.cpp), and pivot's vTrim isn't a
                                // clamp result at all.
};

// track -- the fixed cascade described in the class comment above. Pure:
// no state, no I/O, no heap allocation, same call every tick produces the
// same result for the same inputs.
//
// @param ref         the ALREADY-sampled reference state
//                     (MotionPlan::referenceAt(t), ticket 100-003) -- this
//                     function never re-derives a reference pose from
//                     anchor/kappa/s itself; it projects directly against
//                     ref's own x/y/theta.
// @param measured    the caller's pose estimate this tick. Only
//                     `measured.pose` is read -- `measured.twist` is
//                     deliberately UNUSED: no derivative/kd term (class
//                     comment above).
// @param limits      gains/ceilings snapshot: trackKS/trackKTheta/
//                     trackKCross/minSpeed (ticket 100-004's own
//                     additions to types.h's Limits) plus trimVMax/
//                     trimOmegaMax/vWheelMax (ticket 100-003's additions,
//                     reused here) -- see types.h's Limits doc comments.
// @param trackwidth  [mm] geometry, for the IK step.
// @return            the full cascade result (see TrackerOutput).
TrackerOutput track(const RefState& ref, const BodyState& measured, const Limits& limits,
                     float trackwidth);

}  // namespace Drive
