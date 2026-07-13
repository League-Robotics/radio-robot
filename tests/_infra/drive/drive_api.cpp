// drive_api.cpp -- extern "C" C ABI over Drive::Drivetrain/Drive::MotionPlan
// (source/drive/), the tier-0 host test instrument for the sprint-100 v2
// motion stack (architecture-update.md (100); the driving issue's "Testing:
// the four-tier ladder" section -- tier 0 is Python-over-ctypes directly
// against source/drive/, no plant, no adapter, no hardware; the cheapest and
// most complete tier). Mirrors tests/_infra/sim/sim_api.cpp's proven
// ctypes-ABI shape (opaque handle + extern "C" functions, loaded from Python
// via ctypes) as closely as the two subsystems' different surfaces allow --
// see this file's own "struct-passing convention" below for the one
// deliberate departure (sim_api.cpp passes scalars/flat float arrays only;
// this file's structs are richer, so ticket 100-006's own acceptance
// criteria calls for ctypes.Structure passing instead).
//
// -- Build --
//   just build-drive
// (cmake -S tests/_infra/drive -B tests/_infra/drive/build && cmake --build
// tests/_infra/drive/build --parallel -- tests/_infra/drive/CMakeLists.txt,
// mirroring tests/_infra/sim/CMakeLists.txt's shape). source/drive/ compiles
// STANDALONE (ticket 100-002's own isolation boundary, SUC-008, enforced by
// tests/sim/unit/test_drive_isolation.py): this build needs only the six
// source/drive/*.cpp files, the vendored Ruckig sources
// (libraries/ruckig/src/*.cpp), and this file -- no messages/, no hal/, no
// subsystems/, no HOST_BUILD/ROBOT_DEV_BUILD defines, unlike
// tests/_infra/sim/'s much larger firmware_host build.
//
// -- Run the tier-0 suite --
//   uv run python -m pytest tests/sim/drive/ -q
// (tests/sim/drive/conftest.py's `build_drive_lib` session fixture runs
// `just build-drive` once per pytest session, mirroring tests/sim/
// conftest.py's own `build_lib` fixture for the sim domain.)
//
// -- Struct-passing convention --
// Every struct below is a flat, standard-layout aggregate whose fields are
// ALL either `float` or `int32_t` -- never `bool`/`uint8_t` (both are
// widened to int32_t here purely to keep every field's size/alignment
// identical and trivially predictable across the ctypes boundary;
// Drive::StepState::replanCount is uint8_t on the C++ side, widened to
// int32_t here for the same reason). Each struct is mirrored field-for-field
// by an identical ctypes.Structure in drive.py (SAME declared order, SAME
// types -- ctypes lays a Structure out with the platform's natural
// alignment, matching this file's own struct layout exactly since every
// field here is 4 bytes and naturally aligned already, so no explicit
// _pack_/#pragma pack is needed on either side).
//
// This file is the ONLY place that bridges Drive:: to the outside world (the
// ticket's own isolation rule: "the ABI ... is the ONLY place that bridges
// Drive:: to the outside; it may include source/drive/ headers"). Every
// function below hand-converts between one of these C structs and the
// matching Drive:: C++ type field-by-field -- nothing reinterpret_casts
// across the boundary, so a future Drive:: field reorder/rename cannot
// silently corrupt the ABI (a compile error here catches it instead).
//
// -- Opaque handles --
// drive_create()'s returned void* is a Drive::Drivetrain* -- one immutable
// config, exactly like sim_api.cpp's SimHandle: construct once, reuse for
// many plan()/admit()/replan()/planVelocity() calls; drive_destroy() frees
// it.
//
// The void** outPlan argument of drive_plan()/drive_replan()/
// drive_plan_velocity() is written a HEAP-ALLOCATED Drive::MotionPlan*
// (nullptr on any non-OK verdict). Drive::MotionPlan is copy-CONSTRUCTIBLE
// but NOT copy-ASSIGNABLE (master_profile.h's own ruckig::Ruckig<1> const
// member -- documented at drivetrain.cpp's own plan()/replan() comments), so
// this file heap-copies PlanResult::plan via `new Drive::MotionPlan(result.
// plan)`, the same pattern tests/sim/unit/drive_step_harness.cpp's own
// std::unique_ptr<Drive::MotionPlan> already established for the identical
// reason. The Python side owns this handle's lifetime and must call
// drive_plan_destroy() exactly once per successful plan()/replan()/
// planVelocity() call (drive.py's `Plan` class wraps this in a context
// manager, mirroring firmware.py's `Sim` class's own `close()`/context-
// manager shape).
#include <cstdint>

#include "drive/drivetrain.h"
#include "drive/motion_plan.h"
#include "drive/types.h"

extern "C" {

// ---------------------------------------------------------------------------
// C ABI structs -- field-for-field mirrors of source/drive/types.h's and
// motion_plan.h's/drivetrain.h's plain value types. See file header's
// "Struct-passing convention" above.
// ---------------------------------------------------------------------------

struct DrvProfileLimits {
  float velocity, accel, decel, jerk;
};

struct DrvLimits {
  DrvProfileLimits linear;
  DrvProfileLimits rotational;
  float vWheelMax, trimVMax, trimOmegaMax, wheelStepMax;
  float trackKS, trackKTheta, trackKCross, minSpeed;
};

struct DrvPose {
  float x, y, h;
};

struct DrvTwist {
  float vX, vY, omega;
};

struct DrvGoal {
  float arcLength, deltaHeading, exitSpeed;
};

struct DrvPlanRequest {
  DrvGoal goal;
  DrvPose start;
  float entrySpeed, entryAccel;
};

struct DrvWheelVelocities {
  float left, right;
};

struct DrvWheelState {
  float position, velocity;
  int32_t positionValid, velocityValid;
};

struct DrvBodyState {
  DrvPose pose;
  DrvTwist twist;
};

struct DrvRefState {
  float s, v, a, theta, omega, alpha, x, y;
};

struct DrvStepState {
  float dwellStart, sustainStart, lastReplan;
  int32_t replanCount;
  int32_t settling;
};

struct DrvStepInput {
  float t;
  DrvBodyState measured;
  DrvWheelState left;
  DrvWheelState right;
  float poseStep, poseStepTheta;
};

struct DrvTrackRecord {
  DrvStepInput in;
  DrvRefState ref;
  float eAlong, eCross, eTheta;
  float vTrim, omegaTrim;
  float vCmd, omegaCmd;
  float wheelLeft, wheelRight;
  int32_t trimSaturated;
  int32_t status;
};

struct DrvStepOutput {
  DrvWheelVelocities command;
  int32_t status;
  DrvTrackRecord record;
};

struct DrvChainTail {
  DrvPose pose;
  float exitSpeed, kappa;
};

}  // extern "C"

// ---------------------------------------------------------------------------
// Conversion helpers -- hand-written, field-by-field, both directions. See
// file header's "Struct-passing convention": nothing here reinterpret_casts
// across the boundary.
// ---------------------------------------------------------------------------
namespace {

Drive::ProfileLimits toDrive(const DrvProfileLimits& c) {
  Drive::ProfileLimits d;
  d.velocity = c.velocity;
  d.accel = c.accel;
  d.decel = c.decel;
  d.jerk = c.jerk;
  return d;
}

Drive::Limits toDrive(const DrvLimits& c) {
  Drive::Limits d;
  d.linear = toDrive(c.linear);
  d.rotational = toDrive(c.rotational);
  d.vWheelMax = c.vWheelMax;
  d.trimVMax = c.trimVMax;
  d.trimOmegaMax = c.trimOmegaMax;
  d.wheelStepMax = c.wheelStepMax;
  d.trackKS = c.trackKS;
  d.trackKTheta = c.trackKTheta;
  d.trackKCross = c.trackKCross;
  d.minSpeed = c.minSpeed;
  return d;
}

Drive::Pose toDrive(const DrvPose& c) { return Drive::Pose{c.x, c.y, c.h}; }
DrvPose fromDrive(const Drive::Pose& d) { return DrvPose{d.x, d.y, d.h}; }

Drive::Twist toDrive(const DrvTwist& c) { return Drive::Twist{c.vX, c.vY, c.omega}; }
DrvTwist fromDrive(const Drive::Twist& d) { return DrvTwist{d.v_x, d.v_y, d.omega}; }

Drive::Goal toDrive(const DrvGoal& c) {
  Drive::Goal d;
  d.arcLength = c.arcLength;
  d.deltaHeading = c.deltaHeading;
  d.exitSpeed = c.exitSpeed;
  return d;
}

Drive::PlanRequest toDrive(const DrvPlanRequest& c) {
  Drive::PlanRequest d;
  d.goal = toDrive(c.goal);
  d.start = toDrive(c.start);
  d.entrySpeed = c.entrySpeed;
  d.entryAccel = c.entryAccel;
  return d;
}

DrvWheelVelocities fromDrive(const Drive::WheelVelocities& d) {
  return DrvWheelVelocities{d.left, d.right};
}

Drive::WheelState toDrive(const DrvWheelState& c) {
  Drive::WheelState d;
  d.position = c.position;
  d.velocity = c.velocity;
  d.positionValid = c.positionValid != 0;
  d.velocityValid = c.velocityValid != 0;
  return d;
}

DrvWheelState fromDrive(const Drive::WheelState& d) {
  DrvWheelState c;
  c.position = d.position;
  c.velocity = d.velocity;
  c.positionValid = d.positionValid ? 1 : 0;
  c.velocityValid = d.velocityValid ? 1 : 0;
  return c;
}

Drive::BodyState toDrive(const DrvBodyState& c) {
  Drive::BodyState d;
  d.pose = toDrive(c.pose);
  d.twist = toDrive(c.twist);
  return d;
}

DrvBodyState fromDrive(const Drive::BodyState& d) {
  DrvBodyState c;
  c.pose = fromDrive(d.pose);
  c.twist = fromDrive(d.twist);
  return c;
}

DrvRefState fromDrive(const Drive::RefState& d) {
  DrvRefState c;
  c.s = d.s;
  c.v = d.v;
  c.a = d.a;
  c.theta = d.theta;
  c.omega = d.omega;
  c.alpha = d.alpha;
  c.x = d.x;
  c.y = d.y;
  return c;
}

Drive::StepState toDrive(const DrvStepState& c) {
  Drive::StepState d;
  d.dwellStart = c.dwellStart;
  d.sustainStart = c.sustainStart;
  d.lastReplan = c.lastReplan;
  d.replanCount = static_cast<uint8_t>(c.replanCount);
  d.settling = c.settling != 0;
  return d;
}

DrvStepState fromDrive(const Drive::StepState& d) {
  DrvStepState c;
  c.dwellStart = d.dwellStart;
  c.sustainStart = d.sustainStart;
  c.lastReplan = d.lastReplan;
  c.replanCount = static_cast<int32_t>(d.replanCount);
  c.settling = d.settling ? 1 : 0;
  return c;
}

Drive::StepInput toDrive(const DrvStepInput& c) {
  Drive::StepInput d;
  d.t = c.t;
  d.measured = toDrive(c.measured);
  d.left = toDrive(c.left);
  d.right = toDrive(c.right);
  d.poseStep = c.poseStep;
  d.poseStepTheta = c.poseStepTheta;
  return d;
}

DrvStepInput fromDrive(const Drive::StepInput& d) {
  DrvStepInput c;
  c.t = d.t;
  c.measured = fromDrive(d.measured);
  c.left = fromDrive(d.left);
  c.right = fromDrive(d.right);
  c.poseStep = d.poseStep;
  c.poseStepTheta = d.poseStepTheta;
  return c;
}

DrvTrackRecord fromDrive(const Drive::TrackRecord& d) {
  DrvTrackRecord c;
  c.in = fromDrive(d.in);
  c.ref = fromDrive(d.ref);
  c.eAlong = d.eAlong;
  c.eCross = d.eCross;
  c.eTheta = d.eTheta;
  c.vTrim = d.vTrim;
  c.omegaTrim = d.omegaTrim;
  c.vCmd = d.vCmd;
  c.omegaCmd = d.omegaCmd;
  c.wheelLeft = d.wheelLeft;
  c.wheelRight = d.wheelRight;
  c.trimSaturated = d.trimSaturated ? 1 : 0;
  c.status = static_cast<int32_t>(d.status);
  return c;
}

DrvStepOutput fromDrive(const Drive::StepOutput& d) {
  DrvStepOutput c;
  c.command = fromDrive(d.command);
  c.status = static_cast<int32_t>(d.status);
  c.record = fromDrive(d.record);
  return c;
}

Drive::ChainTail toDrive(const DrvChainTail& c) {
  Drive::ChainTail d;
  d.pose = toDrive(c.pose);
  d.exitSpeed = c.exitSpeed;
  d.kappa = c.kappa;
  return d;
}

DrvChainTail fromDrive(const Drive::ChainTail& d) {
  DrvChainTail c;
  c.pose = fromDrive(d.pose);
  c.exitSpeed = d.exitSpeed;
  c.kappa = d.kappa;
  return c;
}

}  // namespace

// ---------------------------------------------------------------------------
// extern "C" ABI functions.
// ---------------------------------------------------------------------------
extern "C" {

// ---- Lifecycle: Drivetrain ----

void* drive_create(const DrvLimits* limits, float trackwidth) {
  return new Drive::Drivetrain(toDrive(*limits), trackwidth);
}

void drive_destroy(void* h) { delete static_cast<Drive::Drivetrain*>(h); }

// ---- Admission ----

int drive_admit(void* h, const DrvGoal* goal, const DrvChainTail* tail) {
  auto* dt = static_cast<Drive::Drivetrain*>(h);
  return static_cast<int>(dt->admit(toDrive(*goal), toDrive(*tail)));
}

void drive_advance(void* h, const DrvGoal* goal, const DrvChainTail* tail, DrvChainTail* out) {
  auto* dt = static_cast<Drive::Drivetrain*>(h);
  *out = fromDrive(dt->advance(toDrive(*goal), toDrive(*tail)));
}

// ---- Planning: each writes *outPlan a heap-allocated Drive::MotionPlan*
// (nullptr on any non-OK verdict) -- see file header's "Opaque handles". ----

int drive_plan(void* h, const DrvPlanRequest* request, void** outPlan) {
  auto* dt = static_cast<Drive::Drivetrain*>(h);
  Drive::PlanResult result = dt->plan(toDrive(*request));
  *outPlan = (result.verdict == Drive::Verdict::OK) ? new Drive::MotionPlan(result.plan) : nullptr;
  return static_cast<int>(result.verdict);
}

int drive_replan(void* h, void* plan, const DrvBodyState* measured, float elapsed, void** outPlan) {
  auto* dt = static_cast<Drive::Drivetrain*>(h);
  auto* p = static_cast<Drive::MotionPlan*>(plan);
  Drive::PlanResult result = dt->replan(*p, toDrive(*measured), elapsed);
  *outPlan = (result.verdict == Drive::Verdict::OK) ? new Drive::MotionPlan(result.plan) : nullptr;
  return static_cast<int>(result.verdict);
}

int drive_plan_velocity(void* h, const DrvTwist* target, float deadman, const DrvBodyState* current,
                         void** outPlan) {
  auto* dt = static_cast<Drive::Drivetrain*>(h);
  Drive::PlanResult result = dt->planVelocity(toDrive(*target), deadman, toDrive(*current));
  *outPlan = (result.verdict == Drive::Verdict::OK) ? new Drive::MotionPlan(result.plan) : nullptr;
  return static_cast<int>(result.verdict);
}

void drive_plan_destroy(void* plan) { delete static_cast<Drive::MotionPlan*>(plan); }

// ---- Plan queries (pure; MotionPlan()'s own default-constructed-plan
// safe-zero contract is preserved for a null `plan` too, defensively). ----

float drive_plan_duration(void* plan) {
  return plan ? static_cast<Drive::MotionPlan*>(plan)->duration() : 0.0f;
}

float drive_plan_kappa(void* plan) {
  return plan ? static_cast<Drive::MotionPlan*>(plan)->kappa() : 0.0f;
}

void drive_plan_anchor(void* plan, DrvPose* out) {
  *out = plan ? fromDrive(static_cast<Drive::MotionPlan*>(plan)->anchor()) : DrvPose{0.0f, 0.0f, 0.0f};
}

void drive_plan_goal(void* plan, DrvPose* out) {
  *out = plan ? fromDrive(static_cast<Drive::MotionPlan*>(plan)->goal()) : DrvPose{0.0f, 0.0f, 0.0f};
}

float drive_plan_exit_speed(void* plan) {
  return plan ? static_cast<Drive::MotionPlan*>(plan)->exitSpeed() : 0.0f;
}

float drive_plan_effective_ceiling(void* plan) {
  return plan ? static_cast<Drive::MotionPlan*>(plan)->effectiveCeiling() : 0.0f;
}

int drive_plan_is_pivot(void* plan) {
  return (plan && static_cast<Drive::MotionPlan*>(plan)->isPivot()) ? 1 : 0;
}

int drive_plan_is_velocity_mode(void* plan) {
  return (plan && static_cast<Drive::MotionPlan*>(plan)->isVelocityMode()) ? 1 : 0;
}

void drive_reference_at(void* plan, float elapsed, DrvRefState* out) {
  if (!plan) {
    *out = DrvRefState{};
    return;
  }
  *out = fromDrive(static_cast<Drive::MotionPlan*>(plan)->referenceAt(elapsed));
}

// ---- The step: `state` is passed IN/OUT (mirrors MotionPlan::step()'s own
// (const StepInput&, StepState*) contract exactly -- ALL mutation lands in
// *state, never in `plan`). ----

void drive_step(void* plan, const DrvStepInput* in, DrvStepState* state, DrvStepOutput* out) {
  if (!plan) {
    *out = DrvStepOutput{};
    return;
  }
  Drive::StepState nativeState = toDrive(*state);
  Drive::StepOutput result = static_cast<Drive::MotionPlan*>(plan)->step(toDrive(*in), &nativeState);
  *state = fromDrive(nativeState);
  *out = fromDrive(result);
}

}  // extern "C"
