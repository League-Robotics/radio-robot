// drivetrain.h -- Drive::Drivetrain: the motion-control subsystem's single
// entry point. Holds ONLY immutable configuration (limits, gains, geometry)
// fixed at construction; every method is a pure function of its arguments.
// The caller owns the pose estimate, the clock, the plan object, and the
// StepState value -- this class remembers nothing between calls.
//
// Transcribed from the driving issue's own drivetrain.h sketch
// (architecture-update.md (100) M3; clasi/issues/motion-stack-v2-...md
// "source/drive/drivetrain.h" section) -- verbatim; the .cpp implementation
// detail this header does not specify (admit()'s exact check formulas,
// plan()'s pivot-branch ceiling fold, replan()'s exact re-seed method,
// planVelocity()'s two-channel solve) is documented at each site in
// drivetrain.cpp, not here.
#pragma once
#include <cstdint>

#include "drive/types.h"        // Pose, Twist, WheelState, WheelVelocities, Limits
#include "drive/motion_plan.h"

namespace Drive {

// Goal -- the desired outcome, relative to the start pose: one constant-
// curvature arc primitive. kappa = deltaHeading / arcLength.
//   straight: deltaHeading = 0     pivot: arcLength = 0 (exitSpeed must be 0)
struct Goal {
  float arcLength = 0.0f;     // [mm] signed path length; 0 = pivot in place
  float deltaHeading = 0.0f;  // [rad] total heading change, CCW+
  float exitSpeed = 0.0f;     // [mm/s] boundary velocity at segment end; 0 = stop
};

struct PlanRequest {
  Goal goal;
  Pose start;                 // [mm][mm][rad] world anchor (caller's estimate at start)
  float entrySpeed = 0.0f;    // [mm/s] chain-inherited (reference-continuous)
  float entryAccel = 0.0f;    // [mm/s^2]
};

enum class Verdict : uint8_t {
  OK, EXIT_UNREACHABLE, JOINT_STEP_TOO_LARGE, JOINT_SIGN_REVERSAL,
  PIVOT_NONZERO_EXIT, RADIUS_TOO_TIGHT, CEILING_INFEASIBLE, SOLVE_FAILED,
};

struct PlanResult {
  Verdict verdict = Verdict::SOLVE_FAILED;
  MotionPlan plan;            // valid iff verdict == OK
};

// ChainTail -- predicted chain state for queue-time admission; a pure value
// the CALLER carries (the adapter keeps it on the blackboard).
struct ChainTail {
  Pose pose;                  // predicted world pose at chain tail
  float exitSpeed = 0.0f;     // [mm/s]
  float kappa = 0.0f;         // [1/mm]
};

class Drivetrain {
 public:
  Drivetrain(const Limits& limits, float trackwidth);   // [mm] config, immutable

  // admit -- pure feasibility check for queueing `goal` after `tail`:
  // exit reachable within length; joint wheel-speed step v*|dKappa|*W/2
  // within cap; NO per-wheel sign reversal at nonzero joint speed; inner-
  // wheel floor for arcs entered at speed (R >= ~100mm); pivot => exit 0.
  Verdict admit(const Goal& goal, const ChainTail& tail) const;
  ChainTail advance(const Goal& goal, const ChainTail& tail) const;  // compose predicted tail

  // plan -- pure: solve ONE master jerk-limited profile (path length for
  // arcs, heading for pivots), target velocity = exitSpeed, under the
  // trim-headroom-folded ceiling
  //   v_eff = min(vBodyMax, omegaMax/|k|, (vWheelMax - headroom)/(1+|k|W/2)),
  // headroom = trimVMax + trimOmegaMax*W/2 -- wheels cannot saturate, and
  // trims keep authority at ceiling. The world goal pose is composed and
  // frozen into the plan here (replans re-aim at it; drift cannot compound).
  PlanResult plan(const PlanRequest& request) const;

  // replan -- pure: re-TIME the same anchored path from the measured state
  // (project pose onto the arc -- closed form -- re-solve master from
  // (s_meas, v_meas) to the SAME goal and exitSpeed). Never new geometry;
  // cross-track convergence stays the tracker's job. Solve failure returns
  // verdict != OK and the CALLER keeps the old plan (expected outcome for
  // asks reachable only by reversing).
  PlanResult replan(const MotionPlan& plan, const BodyState& measured,
                    float elapsed) const;  // [s]

  // planVelocity -- MOVER teleop: velocity-mode plan toward (v, omega) with
  // a deadman duration; same MotionPlan/step interface, no pose goal.
  PlanResult planVelocity(const Twist& target, float deadman,   // [ms]
                          const BodyState& current) const;

 private:
  Limits limits_;      // immutable after construction
  float trackwidth_;   // [mm]
};

}  // namespace Drive
