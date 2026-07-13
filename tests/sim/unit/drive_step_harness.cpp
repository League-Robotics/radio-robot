// drive_step_harness.cpp -- off-hardware acceptance harness for ticket
// 100-005 (SUC-005/SUC-006/SUC-007): exercises the FULL Drive::MotionPlan::
// step() composition (reference sample -> tracker cascade -> policy
// evaluation, ticket 100-005's own real body replacing the 100-003 stub)
// against REAL solved Drive::Drivetrain::plan() plans and a first-order
// plant stub, mirroring drive_tracker_harness.cpp's stepPlant()/PlantState
// closed-loop pattern -- hand-rolled assertions, no gtest/pytest-native C++
// framework, run via test_drive_step.py.
//
// drive_policy_harness.cpp already exercises Drive::evaluate()'s individual
// branch MECHANICS directly against synthetic TrackerOutput values, with
// exact numeric control. This harness instead proves the REAL, composed
// system: a genuine solved MotionPlan, the genuine tracker cascade, and a
// closed-loop plant -- the things only an end-to-end run can show.
//
// Scenarios (the ticket's own (a)-(h), at the integration level):
//  (1) closed-loop stop-segment convergence: lands within tolerance, dwells,
//      snaps to a literal 0.0f, and NEVER commands a negative wheel speed
//      anywhere in SETTLING (the dedicated no-reversal regression).
//  (2) an overshot approach (measured already past the frozen goal at
//      t >= T_plan) completes at a literal 0.0f, never negative.
//  (3) a plant frozen at its start pose the whole time (pathological,
//      non-convergent) reaches an EXPLICIT abort (ABORT_TIMEOUT or, via
//      repeated replan()s, ABORT_REPLAN_LIMIT) -- never an infinite/silent
//      SETTLING.
//  (4) two chained segments (a flying handoff into a stop) show no large
//      wheel-velocity discontinuity at the handoff boundary.
//  (5) a handoff attempted outside its envelope emits REPLAN_DUE, never a
//      silent DONE_HANDOFF.
//  (6) pose-fix steps via step(): <=30mm absorbed (no replan), >30mm
//      bypasses sustain (immediate REPLAN_DUE).
//  (7) a poseStep injected while the terminal dwell is counting does not
//      reset/extend the dwell.
//  (8) StepState round-trip determinism: the SAME (plan, input, state)
//      fed to step() twice produces byte-identical StepOutput and
//      resulting StepState.
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

#include "drive/arc_math.h"
#include "drive/drivetrain.h"
#include "drive/motion_plan.h"
#include "drive/types.h"

namespace {

// --- Hand-rolled assertion plumbing (mirrors drive_tracker_harness.cpp) ---

int g_failureCount = 0;
std::string g_scenarioName;

void beginScenario(const std::string& name) {
  g_scenarioName = name;
  std::printf("--- %s\n", name.c_str());
}

void fail(const std::string& what) {
  ++g_failureCount;
  std::printf("  FAIL [%s]: %s\n", g_scenarioName.c_str(), what.c_str());
}

void checkTrue(bool condition, const std::string& what) {
  if (!condition) fail(what + " -- expected true, got false");
}

void checkFalse(bool condition, const std::string& what) {
  if (condition) fail(what + " -- expected false, got true");
}

void checkNear(double actual, double expected, double tol, const std::string& what) {
  if (std::fabs(actual - expected) > tol) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected ~%g (tol %g), got %g", what.c_str(), expected,
                  tol, actual);
    fail(buf);
  }
}

void checkExactly(double actual, double expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected EXACTLY %g, got %g", what.c_str(), expected,
                  actual);
    fail(buf);
  }
}

void checkVerdict(Drive::Verdict actual, Drive::Verdict expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected verdict %d, got %d", what.c_str(),
                  static_cast<int>(expected), static_cast<int>(actual));
    fail(buf);
  }
}

template <typename T>
void checkEnum(T actual, T expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %d, got %d", what.c_str(),
                  static_cast<int>(expected), static_cast<int>(actual));
    fail(buf);
  }
}

// --- Shared fixtures ---

constexpr float kTrackwidth = 128.0f;  // [mm]

Drive::Limits makeLimits() {
  Drive::Limits limits;
  limits.linear.velocity = 400.0f;
  limits.linear.accel = 800.0f;
  limits.linear.decel = 800.0f;
  limits.linear.jerk = 0.0f;
  limits.rotational.velocity = 3.0f;
  limits.rotational.accel = 15.0f;
  limits.rotational.decel = 15.0f;
  limits.rotational.jerk = 0.0f;
  limits.vWheelMax = 600.0f;
  limits.trimVMax = 120.0f;
  limits.trimOmegaMax = 1.0f;
  limits.wheelStepMax = 200.0f;
  limits.trackKS = 2.0f;
  limits.trackKTheta = 6.0f;
  limits.trackKCross = 1.5e-5f;
  limits.minSpeed = 20.0f;
  return limits;
}

// injectOffset -- same technique as drive_tracker_harness.cpp/drive_plan_
// harness.cpp: build a Pose exactly (along, cross, dTheta) away from
// `reference` in the reference's own tangent/normal frame.
Drive::Pose injectOffset(const Drive::Pose& reference, float along, float cross, float dTheta) {
  const float cosT = std::cos(reference.h);
  const float sinT = std::sin(reference.h);
  Drive::Pose measured;
  measured.x = reference.x + along * cosT - cross * sinT;
  measured.y = reference.y + along * sinT + cross * cosT;
  measured.h = reference.h + dTheta;
  return measured;
}

// --- First-order plant stub (same shape as drive_tracker_harness.cpp's
// PlantState/stepPlant -- a ticket-scoped stand-in, superseded once ticket
// 100-006's real plant model lands, per that ticket's own note). ---

struct PlantState {
  Drive::Pose pose;
  float v = 0.0f;
  float omega = 0.0f;
};

void stepPlant(PlantState* plant, float wheelLeft, float wheelRight, float trackwidth, float dt,
               float lagAlpha) {
  const float vCmd = (wheelLeft + wheelRight) * 0.5f;
  const float omegaCmd = (wheelRight - wheelLeft) / trackwidth;

  plant->v += lagAlpha * (vCmd - plant->v);
  plant->omega += lagAlpha * (omegaCmd - plant->omega);

  plant->pose.x += plant->v * std::cos(plant->pose.h) * dt;
  plant->pose.y += plant->v * std::sin(plant->pose.h) * dt;
  plant->pose.h = Drive::wrapAngle(plant->pose.h + plant->omega * dt);
}

// --- (1) closed-loop stop-segment convergence: lands, dwells, snaps, never negative ---

void scenarioClosedLoopStopSegmentConvergesDwellsSnaps() {
  beginScenario("closed-loop stop segment: converges, dwells, snaps to literal 0.0f, never negative");

  const Drive::Limits limits = makeLimits();
  Drive::Drivetrain dt(limits, kTrackwidth);

  Drive::PlanRequest req;
  req.goal.arcLength = 500.0f;
  req.goal.deltaHeading = 0.0f;
  req.goal.exitSpeed = 0.0f;
  req.start = Drive::Pose{0.0f, 0.0f, 0.0f};

  Drive::PlanResult planResult = dt.plan(req);
  checkVerdict(planResult.verdict, Drive::Verdict::OK, "straight 500mm stop-segment plan succeeds");
  if (planResult.verdict != Drive::Verdict::OK) return;

  PlantState plant;
  plant.pose = req.start;
  Drive::StepState state;
  const float dtStep = 0.01f;
  const float lagAlpha = 0.3f;

  bool sawDone = false;
  Drive::StepOutput lastOut;
  float t = 0.0f;
  long negativeCommandTicks = 0;
  long settlingTicks = 0;

  for (int i = 0; i < 200000 && !sawDone; ++i, t += dtStep) {
    Drive::StepInput in;
    in.t = t;
    in.measured.pose = plant.pose;
    in.measured.twist.v_x = plant.v;
    in.measured.twist.omega = plant.omega;

    Drive::StepOutput out = planResult.plan.step(in, &state);
    lastOut = out;

    if (out.status == Drive::Status::SETTLING) {
      ++settlingTicks;
      if (out.command.left < 0.0f || out.command.right < 0.0f) ++negativeCommandTicks;
    }
    checkTrue(out.status != Drive::Status::ABORT_TIMEOUT && out.status != Drive::Status::ABORT_REPLAN_LIMIT,
              "a well-tracking plant on a plain stop segment never aborts");

    if (out.status == Drive::Status::DONE_STOP) {
      sawDone = true;
      break;
    }

    stepPlant(&plant, out.command.left, out.command.right, kTrackwidth, dtStep, lagAlpha);
  }

  checkTrue(sawDone, "the closed loop reaches DONE_STOP");
  checkExactly(lastOut.command.left, 0.0, "DONE_STOP: left wheel setpoint snaps to a literal 0.0f");
  checkExactly(lastOut.command.right, 0.0, "DONE_STOP: right wheel setpoint snaps to a literal 0.0f");
  checkTrue(settlingTicks > 0, "the run genuinely passed through SETTLING (not an instant landing)");
  checkExactly(static_cast<double>(negativeCommandTicks), 0.0,
               "NO reversal write-train: zero negative wheel commands across every SETTLING tick");

  const float finalAlong = std::fabs(plant.pose.x - req.goal.arcLength);
  checkTrue(finalAlong < 25.0f, "the plant lands within a reasonable distance of the 500mm goal");
}

// --- (2) overshoot completes at a literal 0.0f, never negative ---

void scenarioOvershootCompletesAtZeroNeverNegative() {
  beginScenario("terminal overshoot (measured past the frozen goal): 0.0f command, never negative, still completes");

  const Drive::Limits limits = makeLimits();
  Drive::Drivetrain dt(limits, kTrackwidth);

  Drive::PlanRequest req;
  req.goal.arcLength = 400.0f;
  req.goal.deltaHeading = 0.0f;
  req.goal.exitSpeed = 0.0f;
  req.start = Drive::Pose{0.0f, 0.0f, 0.0f};

  Drive::PlanResult planResult = dt.plan(req);
  checkVerdict(planResult.verdict, Drive::Verdict::OK, "straight 400mm stop-segment plan succeeds");
  if (planResult.verdict != Drive::Verdict::OK) return;

  const Drive::Pose goalPose = planResult.plan.goal();
  const float duration = planResult.plan.duration();

  Drive::StepState state;
  Drive::BodyState measured;
  measured.pose = injectOffset(goalPose, 20.0f /* 20mm PAST the goal */, 0.0f, 0.0f);
  measured.twist.v_x = 0.0f;
  measured.twist.omega = 0.0f;

  bool sawDone = false;
  long negativeCommandTicks = 0;
  Drive::Status finalStatus = Drive::Status::SETTLING;

  for (float t = duration; t < duration + 0.5f; t += 0.01f) {
    Drive::StepInput in;
    in.t = t;
    in.measured = measured;
    Drive::StepOutput out = planResult.plan.step(in, &state);
    finalStatus = out.status;
    if (out.command.left < 0.0f || out.command.right < 0.0f) ++negativeCommandTicks;
    if (out.status == Drive::Status::DONE_STOP) {
      sawDone = true;
      checkExactly(out.command.left, 0.0, "overshoot completion: left wheel is a literal 0.0f");
      checkExactly(out.command.right, 0.0, "overshoot completion: right wheel is a literal 0.0f");
      break;
    }
  }

  checkExactly(static_cast<double>(negativeCommandTicks), 0.0,
               "an overshoot NEVER produces a negative (backward-correcting) wheel command");
  checkTrue(sawDone, "an overshot-but-stationary plant still reaches DONE_STOP once velocity holds");
  (void)finalStatus;
}

// --- (3) a plant frozen at its start pose reaches an EXPLICIT abort ---

void scenarioFrozenPlantReachesExplicitAbort() {
  beginScenario("a plant frozen at its start pose reaches an EXPLICIT abort -- never an infinite/silent SETTLING");

  const Drive::Limits limits = makeLimits();
  Drive::Drivetrain dt(limits, kTrackwidth);

  Drive::PlanRequest req;
  req.goal.arcLength = 500.0f;
  req.goal.deltaHeading = 0.0f;
  req.goal.exitSpeed = 0.0f;
  req.start = Drive::Pose{0.0f, 0.0f, 0.0f};

  Drive::PlanResult planResult = dt.plan(req);
  checkVerdict(planResult.verdict, Drive::Verdict::OK, "straight 500mm stop-segment plan succeeds");
  if (planResult.verdict != Drive::Verdict::OK) return;

  // MotionPlan is copy-CONSTRUCTIBLE but not copy-ASSIGNABLE (Ruckig<1>'s
  // own const member deletes assignment) -- hold the "currently active"
  // plan behind a unique_ptr, replaced wholesale via reset() on each
  // replan() (construction, never assignment).
  auto plan = std::make_unique<Drive::MotionPlan>(planResult.plan);
  Drive::StepState state;

  const Drive::BodyState measured{req.start, Drive::Twist{}};  // frozen forever at the anchor
  float planStart = 0.0f;
  float wallClock = 0.0f;
  const float dtStep = 0.02f;
  bool sawExplicitAbort = false;

  for (int i = 0; i < 10000 && !sawExplicitAbort; ++i, wallClock += dtStep) {
    const float elapsed = wallClock - planStart;
    Drive::StepInput in;
    in.t = elapsed;
    in.measured = measured;
    Drive::StepOutput out = plan->step(in, &state);

    checkTrue(out.status != Drive::Status::DONE_STOP && out.status != Drive::Status::DONE_HANDOFF,
              "a plant frozen at the start pose the whole time must NEVER report a DONE_* completion");

    if (out.status == Drive::Status::REPLAN_DUE) {
      Drive::PlanResult replanned = dt.replan(*plan, measured, elapsed);
      if (replanned.verdict == Drive::Verdict::OK) {
        plan = std::make_unique<Drive::MotionPlan>(replanned.plan);
        planStart = wallClock;  // the re-timed plan's own t=0 is "now"
      } else {
        sawExplicitAbort = true;  // replan() itself failing is also an explicit resolution
      }
    } else if (out.status == Drive::Status::ABORT_TIMEOUT ||
               out.status == Drive::Status::ABORT_REPLAN_LIMIT) {
      sawExplicitAbort = true;
    }
  }

  checkTrue(sawExplicitAbort,
            "a persistently-frozen plant eventually reaches an EXPLICIT abort/resolution, "
            "never an infinite silent loop");
}

// --- (4) chained handoff: no large wheel-velocity discontinuity ---

void scenarioChainedHandoffNoVelocityDiscontinuity() {
  beginScenario("chained segments: no large wheel-velocity discontinuity across the handoff boundary");

  const Drive::Limits limits = makeLimits();
  Drive::Drivetrain dt(limits, kTrackwidth);
  const float vExit = 150.0f;

  Drive::PlanRequest req1;
  req1.goal.arcLength = 400.0f;
  req1.goal.deltaHeading = 0.0f;
  req1.goal.exitSpeed = vExit;
  req1.start = Drive::Pose{0.0f, 0.0f, 0.0f};

  Drive::PlanResult r1 = dt.plan(req1);
  checkVerdict(r1.verdict, Drive::Verdict::OK, "segment 1 (flying, vExit=150) plan succeeds");
  if (r1.verdict != Drive::Verdict::OK) return;

  PlantState plant;
  plant.pose = req1.start;
  Drive::StepState state1;
  const float dtStep = 0.01f;
  const float lagAlpha = 0.4f;  // fast-tracking plant -- reaches the handoff envelope cleanly

  bool handoff = false;
  Drive::WheelVelocities lastCommand;
  float t = 0.0f;

  for (int i = 0; i < 200000 && !handoff; ++i, t += dtStep) {
    Drive::StepInput in;
    in.t = t;
    in.measured.pose = plant.pose;
    in.measured.twist.v_x = plant.v;
    in.measured.twist.omega = plant.omega;
    Drive::StepOutput out = r1.plan.step(in, &state1);
    lastCommand = out.command;

    if (out.status == Drive::Status::DONE_HANDOFF) {
      handoff = true;
      break;
    }
    checkTrue(out.status == Drive::Status::RUNNING,
              "segment 1 stays RUNNING (or reaches DONE_HANDOFF) with a well-tracking plant, never aborts");
    stepPlant(&plant, out.command.left, out.command.right, kTrackwidth, dtStep, lagAlpha);
  }
  checkTrue(handoff, "segment 1 reaches DONE_HANDOFF with a fast-tracking plant");
  if (!handoff) return;

  // Seeding contract (policy.cpp's own documented contract): the NEXT plan
  // seeds entrySpeed = THIS segment's own Goal::exitSpeed (the reference's
  // boundary velocity), from the anchor at segment 1's frozen goal pose.
  Drive::PlanRequest req2;
  req2.goal.arcLength = 400.0f;
  req2.goal.deltaHeading = 0.0f;
  req2.goal.exitSpeed = 0.0f;  // segment 2 stops
  req2.start = r1.plan.goal();
  req2.entrySpeed = vExit;
  req2.entryAccel = 0.0f;

  Drive::PlanResult r2 = dt.plan(req2);
  checkVerdict(r2.verdict, Drive::Verdict::OK, "segment 2 (seeded at vExit) plan succeeds");
  if (r2.verdict != Drive::Verdict::OK) return;

  const Drive::RefState ref2At0 = r2.plan.referenceAt(0.0f);
  checkNear(ref2At0.v, vExit, 1.0, "segment 2's reference starts at segment 1's own exit speed (C1 continuity)");

  // Compare segment 1's LAST commanded wheel speed against segment 2's
  // FIRST, computed from the SAME carried-over plant state (no teleport).
  Drive::StepState state2;
  Drive::StepInput in2;
  in2.t = 0.0f;
  in2.measured.pose = plant.pose;
  in2.measured.twist.v_x = plant.v;
  in2.measured.twist.omega = plant.omega;
  Drive::StepOutput out2 = r2.plan.step(in2, &state2);

  checkNear(out2.command.left, lastCommand.left, 40.0,
            "no large wheel-velocity discontinuity at the handoff boundary (left)");
  checkNear(out2.command.right, lastCommand.right, 40.0,
            "no large wheel-velocity discontinuity at the handoff boundary (right)");
}

// --- (5) handoff outside envelope -> REPLAN_DUE, never a silent DONE_HANDOFF ---

void scenarioHandoffOutsideEnvelopeNeverSilentDoneHandoff() {
  beginScenario("handoff outside its envelope emits REPLAN_DUE, never a silent DONE_HANDOFF");

  const Drive::Limits limits = makeLimits();
  Drive::Drivetrain dt(limits, kTrackwidth);

  Drive::PlanRequest req;
  req.goal.arcLength = 400.0f;
  req.goal.deltaHeading = 0.0f;
  req.goal.exitSpeed = 150.0f;
  req.start = Drive::Pose{0.0f, 0.0f, 0.0f};

  Drive::PlanResult r = dt.plan(req);
  checkVerdict(r.verdict, Drive::Verdict::OK, "flying segment plan succeeds");
  if (r.verdict != Drive::Verdict::OK) return;

  const float duration = r.plan.duration();
  const Drive::RefState refEnd = r.plan.referenceAt(duration);
  const Drive::Pose refEndPose{refEnd.x, refEnd.y, refEnd.theta};

  Drive::StepState state;
  Drive::BodyState measured;
  measured.pose = injectOffset(refEndPose, 0.0f, 60.0f /* 60mm cross -- outside the 30mm handoff tol */, 0.0f);
  measured.twist.v_x = refEnd.v;
  measured.twist.omega = 0.0f;

  Drive::StepInput in;
  in.t = duration;
  in.measured = measured;
  Drive::StepOutput out = r.plan.step(in, &state);

  checkFalse(out.status == Drive::Status::DONE_HANDOFF,
             "an out-of-envelope handoff NEVER silently reports DONE_HANDOFF");
  checkEnum(out.status, Drive::Status::REPLAN_DUE, "an out-of-envelope handoff emits REPLAN_DUE instead");
}

// --- (6) pose-fix via step(): small absorbed, large bypasses sustain ---

void scenarioPoseFixViaStep() {
  beginScenario("pose-fix via step(): <=30mm absorbed (no replan), >30mm bypasses sustain (immediate REPLAN_DUE)");

  const Drive::Limits limits = makeLimits();
  Drive::Drivetrain dt(limits, kTrackwidth);

  Drive::PlanRequest req;
  req.goal.arcLength = 500.0f;
  req.goal.deltaHeading = 0.0f;
  req.goal.exitSpeed = 0.0f;
  req.start = Drive::Pose{0.0f, 0.0f, 0.0f};

  Drive::PlanResult r = dt.plan(req);
  checkVerdict(r.verdict, Drive::Verdict::OK, "plan succeeds");
  if (r.verdict != Drive::Verdict::OK) return;

  const float t = r.plan.duration() * 0.3f;
  const Drive::RefState ref = r.plan.referenceAt(t);

  {
    Drive::StepState state;
    Drive::StepInput in;
    in.t = t;
    in.measured.pose = Drive::Pose{ref.x, ref.y, ref.theta};  // otherwise-perfect tracking
    in.measured.twist.v_x = ref.v;
    in.poseStep = 15.0f;  // <= 30mm
    Drive::StepOutput out = r.plan.step(in, &state);
    checkEnum(out.status, Drive::Status::RUNNING, "a small (<=30mm) poseStep never itself triggers a replan");
  }

  {
    Drive::StepState state;
    Drive::StepInput in;
    in.t = t;
    in.measured.pose = Drive::Pose{ref.x, ref.y, ref.theta};
    in.measured.twist.v_x = ref.v;
    in.poseStep = 45.0f;  // > 30mm
    Drive::StepOutput out = r.plan.step(in, &state);
    checkEnum(out.status, Drive::Status::REPLAN_DUE,
              "a large (>30mm) poseStep triggers REPLAN_DUE on the same tick, bypassing sustain");
  }
}

// --- (7) poseStep during the terminal dwell does not reset/extend it ---

void scenarioPoseFixDuringDwellDoesNotResetViaStep() {
  beginScenario("a poseStep during the terminal dwell (via step()) does not reset/extend the dwell");

  const Drive::Limits limits = makeLimits();
  Drive::Drivetrain dt(limits, kTrackwidth);

  Drive::PlanRequest req;
  req.goal.arcLength = 300.0f;
  req.goal.deltaHeading = 0.0f;
  req.goal.exitSpeed = 0.0f;
  req.start = Drive::Pose{0.0f, 0.0f, 0.0f};

  Drive::PlanResult r = dt.plan(req);
  checkVerdict(r.verdict, Drive::Verdict::OK, "plan succeeds");
  if (r.verdict != Drive::Verdict::OK) return;

  const Drive::Pose goalPose = r.plan.goal();
  const float duration = r.plan.duration();

  Drive::StepState state;
  Drive::BodyState measured;
  measured.pose = goalPose;  // exactly on target, at rest
  measured.twist.v_x = 0.0f;

  Drive::StepInput firstIn;
  firstIn.t = duration;
  firstIn.measured = measured;
  Drive::StepOutput firstOut = r.plan.step(firstIn, &state);
  checkEnum(firstOut.status, Drive::Status::SETTLING, "first tick at t==duration, on target: SETTLING, dwell starts");
  const float dwellAfterFirst = state.dwellStart;
  checkTrue(dwellAfterFirst >= 0.0f, "dwell has started");

  Drive::StepInput poseFixIn;
  poseFixIn.t = duration + 0.02f;
  poseFixIn.measured = measured;
  poseFixIn.poseStep = 50.0f;  // large -- would bypass sustain if NOT mid-dwell
  Drive::StepOutput poseFixOut = r.plan.step(poseFixIn, &state);

  checkEnum(poseFixOut.status, Drive::Status::SETTLING, "a poseStep mid-dwell does not trigger REPLAN_DUE");
  checkNear(state.dwellStart, dwellAfterFirst, 1e-9, "a poseStep mid-dwell does not reset/extend dwellStart");
}

// --- (8) StepState round-trip determinism ---

void scenarioStepStateRoundTrip() {
  beginScenario("StepState round-trips: same (plan, input, state) fed to step() twice matches exactly");

  const Drive::Limits limits = makeLimits();
  Drive::Drivetrain dt(limits, kTrackwidth);

  Drive::PlanRequest req;
  req.goal.arcLength = 500.0f;
  req.goal.deltaHeading = 0.003f * 500.0f;
  req.goal.exitSpeed = 0.0f;
  req.start = Drive::Pose{0.0f, 0.0f, 0.0f};

  Drive::PlanResult r = dt.plan(req);
  checkVerdict(r.verdict, Drive::Verdict::OK, "curved plan succeeds");
  if (r.verdict != Drive::Verdict::OK) return;

  const float t = r.plan.duration() * 0.5f;
  const Drive::RefState ref = r.plan.referenceAt(t);

  Drive::StepInput in;
  in.t = t;
  in.measured.pose = injectOffset(Drive::Pose{ref.x, ref.y, ref.theta}, 8.0f, -3.0f, 0.01f);
  in.measured.twist.v_x = ref.v - 5.0f;
  in.measured.twist.omega = ref.omega;

  Drive::StepState stateA;
  stateA.sustainStart = 0.2f;
  Drive::StepState stateB = stateA;  // identical copy

  Drive::StepOutput outA = r.plan.step(in, &stateA);
  Drive::StepOutput outB = r.plan.step(in, &stateB);

  checkEnum(outA.status, outB.status, "same plan+input+state -> same Status");
  checkExactly(outA.command.left, outB.command.left, "byte-identical left wheel command");
  checkExactly(outA.command.right, outB.command.right, "byte-identical right wheel command");
  checkExactly(outA.record.eAlong, outB.record.eAlong, "byte-identical record.eAlong");
  checkExactly(outA.record.eCross, outB.record.eCross, "byte-identical record.eCross");
  checkExactly(stateA.dwellStart, stateB.dwellStart, "resulting StepState.dwellStart identical");
  checkExactly(stateA.sustainStart, stateB.sustainStart, "resulting StepState.sustainStart identical");
  checkExactly(stateA.lastReplan, stateB.lastReplan, "resulting StepState.lastReplan identical");
  checkExactly(stateA.replanCount, stateB.replanCount, "resulting StepState.replanCount identical");
  checkExactly(stateA.settling ? 1.0 : 0.0, stateB.settling ? 1.0 : 0.0, "resulting StepState.settling identical");
}

}  // namespace

int main() {
  scenarioClosedLoopStopSegmentConvergesDwellsSnaps();
  scenarioOvershootCompletesAtZeroNeverNegative();
  scenarioFrozenPlantReachesExplicitAbort();
  scenarioChainedHandoffNoVelocityDiscontinuity();
  scenarioHandoffOutsideEnvelopeNeverSilentDoneHandoff();
  scenarioPoseFixViaStep();
  scenarioPoseFixDuringDwellDoesNotResetViaStep();
  scenarioStepStateRoundTrip();

  if (g_failureCount == 0) {
    std::printf("OK: all Drive:: step() closed-loop scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the Drive:: step() closed-loop scenarios\n", g_failureCount);
  return 1;
}
