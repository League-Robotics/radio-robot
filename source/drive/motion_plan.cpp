// motion_plan.cpp -- Drive::MotionPlan implementation. See motion_plan.h
// for the class-level design notes (the private-field elaboration beyond
// the driving issue's own sketch, the step() stub's exact shape).
#include "drive/motion_plan.h"

#include "drive/arc_math.h"

namespace Drive {

MotionPlan::MotionPlan(MasterProfile profile, MasterProfile omegaProfile, float kappa,
                        float masterTarget, const Pose& anchor, const Pose& goal, float exitSpeed,
                        float effectiveCeiling, float duration, const Limits& limits,
                        float trackwidth, bool isPivot, bool isVelocityMode)
    : profile_(profile),
      omegaProfile_(omegaProfile),
      kappa_(kappa),
      masterTarget_(masterTarget),
      anchor_(anchor),
      goal_(goal),
      exitSpeed_(exitSpeed),
      effectiveCeiling_(effectiveCeiling),
      duration_(duration),
      limits_(limits),
      trackwidth_(trackwidth),
      isPivot_(isPivot),
      isVelocityMode_(isVelocityMode),
      valid_(true) {}

float MotionPlan::duration() const { return duration_; }

float MotionPlan::kappa() const { return kappa_; }

Pose MotionPlan::anchor() const { return anchor_; }

Pose MotionPlan::goal() const { return goal_; }

float MotionPlan::exitSpeed() const { return exitSpeed_; }

float MotionPlan::effectiveCeiling() const { return effectiveCeiling_; }

bool MotionPlan::isPivot() const { return isPivot_; }

bool MotionPlan::isVelocityMode() const { return isVelocityMode_; }

RefState MotionPlan::referenceAt(float elapsed) const {
  RefState ref;
  if (!valid_) return ref;  // default/empty plan -- all-zero RefState

  if (isVelocityMode_) {
    // Velocity-mode: two independently-solved channels, no pose reference
    // (motion_plan.h's own doc comment on why -- MOVER's (v, omega) is not
    // a single constant-curvature arc). peek() is const/pure -- NEVER
    // sample(), which would mutate the profile's own remembered seed and
    // break this method's "same plan + same t => same RefState, always"
    // contract.
    const MasterProfile::State vState = profile_.peek(elapsed);
    const MasterProfile::State omegaState = omegaProfile_.peek(elapsed);
    ref.s = vState.position;   // informational only -- not a pose coordinate
    ref.v = vState.velocity;
    ref.a = vState.acceleration;
    ref.omega = omegaState.velocity;
    ref.alpha = omegaState.acceleration;
    ref.x = anchor_.x;
    ref.y = anchor_.y;
    ref.theta = anchor_.h;     // no pose is composed for velocity mode --
                                // x/y/theta hold the anchor for the whole
                                // plan (see goal_ == anchor_ above)
    return ref;
  }

  const MasterProfile::State state = profile_.peek(elapsed);
  ref.s = state.position;

  if (isPivot_) {
    // RefState.v's own doc: "0 during pivot" -- the master DOF IS heading
    // here, so state.velocity/acceleration are omega/alpha directly, never
    // a path-length rate.
    ref.v = 0.0f;
    ref.a = 0.0f;
    ref.omega = state.velocity;
    ref.alpha = state.acceleration;
    ref.theta = wrapAngle(anchor_.h + state.position);
    ref.x = anchor_.x;
    ref.y = anchor_.y;
  } else {
    ref.v = state.velocity;
    ref.a = state.acceleration;
    // omega = kappa * v, ALWAYS derived -- never independently solved
    // (the issue's "one trajectory per segment" rule).
    ref.omega = kappa_ * state.velocity;
    ref.alpha = kappa_ * state.acceleration;
    const Pose pose = poseAlongArc(anchor_, kappa_, state.position);
    ref.x = pose.x;
    ref.y = pose.y;
    ref.theta = pose.h;
  }
  return ref;
}

StepOutput MotionPlan::step(const StepInput& in, StepState* state) const {
  // TICKET 100-003 STUB -- see motion_plan.h's own doc comment on step()
  // for the exact contract this placeholder honors (harmless, no crash,
  // *state untouched). Ticket 005 replaces this entire body with the
  // tracker + policy + terminal machine composition.
  (void)state;
  StepOutput out;
  out.status = Status::RUNNING;
  out.record.in = in;
  out.record.status = Status::RUNNING;
  return out;
}

}  // namespace Drive
