// drivetrain.cpp -- Drive::Drivetrain implementation. See drivetrain.h for
// the class-level design notes and each public method's own doc comment
// (transcribed from the driving issue's sketch). This file documents the
// implementation detail the sketch itself leaves open -- see each function
// below.
#include "drive/drivetrain.h"

#include <math.h>

#include <algorithm>

#include "drive/arc_math.h"

namespace Drive {

namespace {

// -- Shared thresholds --
// kEpsilonKappa matches arc_math.cpp's own straight-line-limit threshold
// (1e-6 [1/mm], a ~1km radius) -- reused here so admit()/plan() agree with
// arc_math's own notion of "effectively straight" everywhere kappa is
// tested against zero.
constexpr float kEpsilonKappa = 1.0e-6f;      // [1/mm]
// A Goal is a pivot iff |arcLength| is below this threshold (0.1um -- far
// tighter than any real segment, just enough to absorb float roundoff
// around a literal 0.0f).
constexpr float kEpsilonArcLength = 1.0e-4f;  // [mm]
// "Nonzero" threshold for exitSpeed/jointSpeed checks (PIVOT_NONZERO_EXIT,
// the joint-step/sign-reversal/radius-floor checks below).
constexpr float kEpsilonSpeed = 1.0e-3f;      // [mm/s]
// admit()'s EXIT_UNREACHABLE kinematic check is a coarse, conservative
// queue-time estimate (plan()'s real Ruckig solve is the exact answer) --
// this additive slack on the squared-speed comparison absorbs float
// roundoff without hiding a genuine several-mm/s infeasibility.
constexpr float kEpsilonReachSq = 25.0f;      // [(mm/s)^2] ~= 5 mm/s slack
// The issue's own "~100mm" inner-wheel floor radius for an arc entered at
// speed (RADIUS_TOO_TIGHT) -- a geometric constant of the check itself,
// not a PlannerConfig field (the issue's own "~" marks it as approximate,
// unlike the wire-tunable gains in Limits).
constexpr float kRadiusFloorMm = 100.0f;      // [mm]

bool isPivotGoal(const Goal& goal) { return fabsf(goal.arcLength) < kEpsilonArcLength; }

// goalKappa -- ONLY valid for a non-pivot goal (isPivotGoal() already
// false, so arcLength is safely bounded away from zero). A pivot's
// curvature is formally undefined (arc_math.h's own note: arc length == 0
// is 0/0) -- never called with a pivot Goal.
float goalKappa(const Goal& goal) { return goal.deltaHeading / goal.arcLength; }

}  // namespace

Drivetrain::Drivetrain(const Limits& limits, float trackwidth)
    : limits_(limits), trackwidth_(trackwidth) {}

// admit() -- the issue's own doc comment lists four checks; each is
// implemented as its own block below, in the order that lets later checks
// assume earlier ones already passed (a pivot short-circuits before the
// curvature-transition checks, which are meaningless for a pivot's
// undefined kappa).
Verdict Drivetrain::admit(const Goal& goal, const ChainTail& tail) const {
  const bool pivot = isPivotGoal(goal);

  // pivot => exit must be 0 (checked FIRST: a pivot with a nonzero exit
  // speed is a distinct, more specific rejection than the generic
  // reachability check below would produce).
  if (pivot && fabsf(goal.exitSpeed) > kEpsilonSpeed) {
    return Verdict::PIVOT_NONZERO_EXIT;
  }

  // Exit reachable within the arc length: a coarse kinematic v^2 bound,
  // |v_exit^2 - v_entry^2| <= 2*a*extent, using the accelerating bound
  // when |exit| > |entry| (speeding up) or the decelerating bound
  // otherwise -- the physically-binding direction for a queue-time
  // estimate (plan()'s Ruckig solve is the exact jerk-limited answer;
  // this is deliberately conservative, not exact).
  //
  // For a pivot, extent == 0 (arcLength, reused directly rather than
  // deltaHeading -- see drivetrain.h's own Goal doc: a pivot's own
  // "length" IS zero), which collapses the RHS to ~0 regardless of which
  // bound (accel/decel, and its unit domain -- rotational's rad/s^2
  // against tail.exitSpeed's mm/s) is picked: the check degenerates to
  // "tail.exitSpeed must be ~0", which is exactly the physical
  // requirement (a pivot must be entered from a full stop) -- so this ONE
  // formula also does double duty as the "pivot entered at speed" guard,
  // no separate case needed.
  {
    const ProfileLimits& profile = pivot ? limits_.rotational : limits_.linear;
    const float extent = fabsf(goal.arcLength);
    const bool speedingUp = fabsf(goal.exitSpeed) > fabsf(tail.exitSpeed);
    const float bound = speedingUp ? profile.accel : profile.decel;
    const float deltaVSq = goal.exitSpeed * goal.exitSpeed - tail.exitSpeed * tail.exitSpeed;
    if (fabsf(deltaVSq) > 2.0f * bound * extent + kEpsilonReachSq) {
      return Verdict::EXIT_UNREACHABLE;
    }
  }

  if (pivot) {
    // The remaining checks are all about a CURVATURE transition -- a
    // pivot's kappa is formally undefined, so none apply.
    return Verdict::OK;
  }

  const float newKappa = goalKappa(goal);
  const float jointSpeed = tail.exitSpeed;
  const float halfTrack = trackwidth_ * 0.5f;

  // The remaining three checks only matter at a nonzero joint speed (a
  // "flying" chain continuation) -- starting from rest, there is no
  // instantaneous wheel-speed jump and no curvature-entered-at-speed risk.
  if (fabsf(jointSpeed) > kEpsilonSpeed) {
    // Joint wheel-speed step: v*|dKappa|*W/2 within cap.
    const float dKappa = newKappa - tail.kappa;
    const float jointStep = fabsf(jointSpeed) * fabsf(dKappa) * halfTrack;
    if (jointStep > limits_.wheelStepMax) {
      return Verdict::JOINT_STEP_TOO_LARGE;
    }

    // NO per-wheel sign reversal at nonzero joint speed: differential IK's
    // own per-wheel factor is (1 -+ kappa*W/2) (left: minus, right: plus,
    // matching kappa's CCW+ convention -- a left/CCW turn slows the inner
    // LEFT wheel). Neither wheel's factor may cross zero between the OLD
    // curvature (tail.kappa) and the NEW one (newKappa) at this joint --
    // a sign flip means the wheel would have to reverse direction
    // mid-transition, which the issue's "no reversal write-trains,
    // structurally" rule forbids outright.
    const float leftFactorOld = 1.0f - tail.kappa * halfTrack;
    const float leftFactorNew = 1.0f - newKappa * halfTrack;
    const float rightFactorOld = 1.0f + tail.kappa * halfTrack;
    const float rightFactorNew = 1.0f + newKappa * halfTrack;
    const bool leftReverses = (leftFactorOld * leftFactorNew) < 0.0f;
    const bool rightReverses = (rightFactorOld * rightFactorNew) < 0.0f;
    if (leftReverses || rightReverses) {
      return Verdict::JOINT_SIGN_REVERSAL;
    }

    // Inner-wheel floor: an arc entered at speed must not be tighter than
    // the ~100mm radius floor -- inside that radius the inner wheel's
    // speed collapses toward zero (or would need to reverse) to hold the
    // curvature at this joint speed.
    if (fabsf(newKappa) > kEpsilonKappa) {
      const float radius = 1.0f / fabsf(newKappa);
      if (radius < kRadiusFloorMm) {
        return Verdict::RADIUS_TOO_TIGHT;
      }
    }
  }

  return Verdict::OK;
}

ChainTail Drivetrain::advance(const Goal& goal, const ChainTail& tail) const {
  ChainTail next;
  next.exitSpeed = goal.exitSpeed;

  if (isPivotGoal(goal)) {
    next.pose = tail.pose;
    next.pose.h = wrapAngle(tail.pose.h + goal.deltaHeading);
    // kappa is formally undefined after a pivot; 0.0f is a safe
    // placeholder -- any SUBSEQUENT admit() call's curvature-transition
    // checks are skipped anyway whenever jointSpeed == 0 (see admit()'s
    // own comment above), and a pivot's own exitSpeed is always 0 by
    // construction (PIVOT_NONZERO_EXIT), so this value is never actually
    // read as a "real" curvature by any caller.
    next.kappa = 0.0f;
  } else {
    const float kappa = goalKappa(goal);
    next.pose = composeArc(tail.pose, kappa, goal.arcLength);
    next.kappa = kappa;
  }
  return next;
}

PlanResult Drivetrain::plan(const PlanRequest& request) const {
  PlanResult result;
  const Goal& goal = request.goal;
  const bool pivot = isPivotGoal(goal);

  // A pivot Goal with nonzero exitSpeed is rejected here too -- plan() is
  // independently callable (not required to go through admit() first),
  // and must never silently clamp it to zero.
  if (pivot && fabsf(goal.exitSpeed) > kEpsilonSpeed) {
    result.verdict = Verdict::PIVOT_NONZERO_EXIT;
    return result;
  }

  const float kappa = pivot ? 0.0f : goalKappa(goal);

  // headroom = trimVMax + trimOmegaMax*W/2 -- the trim law's own worst-
  // case authority budget (drivetrain.h's own plan() doc comment,
  // verbatim formula).
  const float headroom = limits_.trimVMax + limits_.trimOmegaMax * trackwidth_ * 0.5f;
  const float wheelBudget = limits_.vWheelMax - headroom;

  // v_eff = min(vBodyMax, omegaMax/|k|, (vWheelMax - headroom)/(1+|k|W/2))
  // -- the sketch's own formula, verbatim, for an arc/straight (kappa may
  // be 0). PIVOT BRANCH (not specified by the sketch's literal formula,
  // which is written in the linear [mm/s] domain): the master DOF is
  // heading, so the analogous fold is in the rotational [rad/s] domain --
  // omega_eff = min(omegaMax, (vWheelMax - headroom)/(W/2)), the pivot's
  // own wheel-budget term (both wheels move at |omega|*W/2, opposite
  // signs, during a pivot -- no "1 +" term, since there is no forward
  // component to add to). This is the one deliberate extension beyond the
  // header sketch this ticket makes (documented in completion notes).
  float ceiling = 0.0f;
  if (pivot) {
    ceiling = std::min(limits_.rotational.velocity, wheelBudget / (trackwidth_ * 0.5f));
  } else {
    ceiling = limits_.linear.velocity;  // vBodyMax
    if (fabsf(kappa) > kEpsilonKappa) {
      ceiling = std::min(ceiling, limits_.rotational.velocity / fabsf(kappa));  // omegaMax/|k|
    }
    const float denom = 1.0f + fabsf(kappa) * trackwidth_ * 0.5f;
    ceiling = std::min(ceiling, wheelBudget / denom);
  }

  if (ceiling <= 0.0f) {
    result.verdict = Verdict::CEILING_INFEASIBLE;
    return result;
  }

  MasterProfile profile;
  profile.configure(pivot ? limits_.rotational : limits_.linear);
  profile.seedCurrent(0.0f, request.entrySpeed, request.entryAccel);

  const float target = pivot ? goal.deltaHeading : goal.arcLength;
  if (!profile.solveToExit(target, goal.exitSpeed, ceiling)) {
    result.verdict = Verdict::SOLVE_FAILED;
    return result;
  }

  // Compose and FREEZE the world goal pose here -- never re-derived on
  // replan (drivetrain.h's own plan() doc comment: "drift cannot
  // compound").
  const Pose goalPose = pivot
      ? Pose{request.start.x, request.start.y, wrapAngle(request.start.h + goal.deltaHeading)}
      : composeArc(request.start, kappa, goal.arcLength);

  // ruckig::Ruckig<1> (inside MasterProfile) holds a const member, so
  // MotionPlan is copy-CONSTRUCTIBLE but not assignABLE (see
  // master_profile.h's own otg_ field) -- built and returned via
  // aggregate list-init (guaranteed prvalue copy elision, C++17) rather
  // than `result.plan = MotionPlan(...)`, which would not compile.
  MasterProfile unusedOmegaProfile;  // segment plans use only ONE channel
  return PlanResult{Verdict::OK,
                     MotionPlan(profile, unusedOmegaProfile, kappa, target, request.start,
                                goalPose, goal.exitSpeed, ceiling, profile.duration(), limits_,
                                trackwidth_, pivot, /*isVelocityMode=*/false)};
}

PlanResult Drivetrain::replan(const MotionPlan& plan, const BodyState& measured,
                               float elapsed) const {
  PlanResult result;

  // Velocity-mode plans have no anchored path to re-time -- MOVER's own
  // "replan" is simply a fresh planVelocity() call (latest-wins,
  // SUC-010), never this method. Report SOLVE_FAILED (verdict != OK,
  // per this method's own contract) rather than silently doing nothing.
  if (plan.isVelocityMode()) {
    result.verdict = Verdict::SOLVE_FAILED;
    return result;
  }

  const bool pivot = plan.isPivot();
  const Pose anchor = plan.anchor();
  const float kappa = plan.kappa();

  // Sample the reference at `elapsed` (closed-form, pure) to get s_ref --
  // the starting point for the exact projection below. referenceAt()
  // itself never mutates plan (it is const), matching this method's own
  // purity.
  const RefState ref = plan.referenceAt(elapsed);

  // Project the measured pose onto the arc EXACTLY at s_ref (closed
  // form -- never a search, per arc_math.h's own projectOntoArc() doc),
  // then correct s_ref by the measured along-track offset to get s_meas.
  // For a pivot there is no arc to project onto (kappa undefined); the
  // equivalent correction is the wrapped heading error against the
  // reference's own (already-wrapped) theta.
  float sMeas = 0.0f;
  float vMeas = 0.0f;
  if (pivot) {
    const float eTheta = wrapAngle(measured.pose.h - ref.theta);
    sMeas = ref.s + eTheta;
    vMeas = measured.twist.omega;
  } else {
    const ArcError err = projectOntoArc(anchor, kappa, ref.s, measured.pose);
    sMeas = ref.s + err.eAlong;
    vMeas = measured.twist.v_x;
  }

  // Re-solve the master profile from (s_meas, v_meas) to the SAME goal
  // and exitSpeed -- never new geometry (master_profile.h's own seeding
  // contract: seedCurrent() with the measured state, then solveToExit()
  // again). Seed acceleration is 0.0f: BodyState carries no measured
  // acceleration, and a fresh solve starting from a neutral (rather than
  // stale-predicted) acceleration is the more conservative choice.
  MasterProfile profile;
  profile.configure(pivot ? limits_.rotational : limits_.linear);
  profile.seedCurrent(sMeas, vMeas, 0.0f);

  if (!profile.solveToExit(plan.masterTarget_, plan.exitSpeed(), plan.effectiveCeiling())) {
    // Solve failure IS the expected outcome for an ask reachable only by
    // reversing (the directional no-reversal band, master_profile.h's own
    // class comment) -- the caller keeps the OLD plan on this result.
    result.verdict = Verdict::SOLVE_FAILED;
    return result;
  }

  // Same guaranteed-copy-elision aggregate-init pattern as plan() above
  // (MotionPlan is not assignABLE -- see that comment).
  MasterProfile unusedOmegaProfile;
  return PlanResult{Verdict::OK,
                     MotionPlan(profile, unusedOmegaProfile, kappa, plan.masterTarget_, anchor,
                                plan.goal(), plan.exitSpeed(), plan.effectiveCeiling(),
                                profile.duration(), plan.limits_, plan.trackwidth_, pivot,
                                /*isVelocityMode=*/false)};
}

PlanResult Drivetrain::planVelocity(const Twist& target, float deadman,
                                     const BodyState& current) const {
  PlanResult result;

  // Ceiling fold, specialized for velocity mode: this ticket folds only
  // the LINEAR channel's own ceiling (vBodyMax, vWheelMax - headroom) --
  // the SAME straight-line (kappa == 0) case as plan()'s own formula. The
  // joint v+omega wheel-speed coupling (both wheels must respect vWheelMax
  // simultaneously for an ARBITRARY (v, omega) pair, not just a constant-
  // curvature one) is the tracker cascade's own job at each tick (ticket
  // 004) -- SUC-003's v_eff-fold invariant test targets admit()/plan()'s
  // segment plans specifically, not planVelocity(); this is a deliberate,
  // documented scope simplification, not an oversight.
  const float headroom = limits_.trimVMax + limits_.trimOmegaMax * trackwidth_ * 0.5f;
  const float vCeiling = std::min(limits_.linear.velocity, limits_.vWheelMax - headroom);
  const float omegaCeiling = limits_.rotational.velocity;

  if (vCeiling <= 0.0f) {
    result.verdict = Verdict::CEILING_INFEASIBLE;
    return result;
  }

  // Two independently-solved, open-ended channels (solveToVelocity, not
  // solveToExit -- no target position, matches jerk_trajectory.h's own
  // cruise-ramp/teleop precedent carried forward verbatim by
  // master_profile.h).
  MasterProfile vProfile;
  vProfile.configure(limits_.linear);
  vProfile.seedCurrent(0.0f, current.twist.v_x, 0.0f);
  const bool vOk = vProfile.solveToVelocity(target.v_x, vCeiling);

  MasterProfile omegaProfile;
  omegaProfile.configure(limits_.rotational);
  omegaProfile.seedCurrent(0.0f, current.twist.omega, 0.0f);
  const bool omegaOk = omegaProfile.solveToVelocity(target.omega, omegaCeiling);

  if (!vOk || !omegaOk) {
    result.verdict = Verdict::SOLVE_FAILED;
    return result;
  }

  // deadman is tagged [ms] in the sketch's own drivetrain.h comment, while
  // MotionPlan::duration() is [s] throughout (motion_plan.h's own RefState/
  // duration() doc comments) -- converted here, the one unit reconciliation
  // this ticket's .cpp needed the sketch did not spell out explicitly.
  const float duration = deadman * 0.001f;

  // No pose goal (drivetrain.h's own planVelocity() doc comment) --
  // anchor_ == goal_ == current.pose signals "no displacement target";
  // kappa/masterTarget are not meaningful for a velocity-mode plan. Same
  // guaranteed-copy-elision aggregate-init pattern as plan()/replan()
  // above (MotionPlan is not assignABLE -- see plan()'s own comment).
  return PlanResult{Verdict::OK,
                     MotionPlan(vProfile, omegaProfile, /*kappa=*/0.0f, /*masterTarget=*/0.0f,
                                current.pose, current.pose, target.v_x, vCeiling, duration,
                                limits_, trackwidth_, /*isPivot=*/false,
                                /*isVelocityMode=*/true)};
}

}  // namespace Drive
