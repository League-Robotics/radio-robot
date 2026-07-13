// motion_plan.h -- Drive::MotionPlan: one solved segment. IMMUTABLE after
// planning -- pure data plus const queries. step() is a const method: all
// mutable state lives in the caller-owned StepState value. Same plan +
// same input + same state => same output, always: any tick recorded on
// sim, bench, or field replays bit-for-bit offline.
//
// Transcribed from the driving issue's own motion_plan.h sketch
// (architecture-update.md (100) M3; clasi/issues/motion-stack-v2-...md
// "source/drive/motion_plan.h" section) -- RefState/StepState/StepInput/
// Status/TrackRecord/StepOutput and the public MotionPlan query surface
// are verbatim. MotionPlan's PRIVATE section is elaborated beyond the
// sketch's one-line "Immutable solve results: master trajectory
// polynomial (Ruckig), geometry (kappa, anchor, goal), limits snapshot.
// No mutable members." comment -- see the class's own private-section
// comment below for exactly what was added and why (ticket 100-003
// completion notes cover this in full: a second MasterProfile for
// velocity-mode's independently-solved omega channel, the master-DOF
// absolute target replan() needs to re-solve toward, and the Limits/
// trackwidth snapshot the sketch's own comment already calls for, ahead
// of ticket 004/005's step() actually reading it).
#pragma once
#include <cstdint>

#include "drive/master_profile.h"
#include "drive/types.h"

namespace Drive {

// RefState -- the reference trajectory at one instant: THE plottable
// artifact. referenceAt(t) sampled over [0, duration] is the "show me the
// plan before anything moves" table.
struct RefState {
  float s = 0.0f;       // [mm] or [rad] master-DOF position
  float v = 0.0f;       // [mm/s] body speed along path (0 during pivot)
  float a = 0.0f;       // [mm/s^2]
  float theta = 0.0f;   // [rad] reference heading (world)
  float omega = 0.0f;   // [rad/s] = kappa * v (arc) or master rate (pivot)
  float alpha = 0.0f;   // [rad/s^2]
  float x = 0.0f;       // [mm] reference world position (closed-form arc)
  float y = 0.0f;       // [mm]
};

// StepState -- ALL mutable state in the subsystem, owned by the CALLER as
// a transparent value. Everything else is pure. Five scalars: the policy
// timers are the subsystem's ONLY state (the wheel-PID integrators live at
// LEVEL 2, the HAL leaf, per the two-levels decision). See the plan's
// "Statelessness accounting".
struct StepState {
  float dwellStart = -1.0f;      // [s] terminal tolerance first held (<0 = not held)
  float sustainStart = -1.0f;    // [s] replan-envelope first exceeded (<0 = inside)
  float lastReplan = -1.0f;      // [s] rate-limit anchor
  uint8_t replanCount = 0;       // toward the N-max abort
  bool settling = false;         // terminal state machine entered
};

struct StepInput {
  float t = 0.0f;          // [s] elapsed since plan start (caller's clock)
  BodyState measured;      // caller-maintained pose estimate + body twist
  WheelState left, right;  // measured wheel position/velocity (+validity)
  float poseStep = 0.0f;   // [mm] magnitude of an external pose-fix step
  float poseStepTheta = 0.0f;  // [rad] applied since last step (0 = none)
};

enum class Status : uint8_t {
  RUNNING,          // tracking the reference
  SETTLING,         // stop segment past T_plan: banded one-sided walk-in
  REPLAN_DUE,       // caller should invoke Drivetrain::replan and swap plans
  DONE_STOP,        // completion gate held (pose+vel tolerance, dwelled)
  DONE_HANDOFF,     // vExit != 0: exhausted AND within handoff envelope
  ABORT_TIMEOUT,    // explicit failure -- never silent
  ABORT_REPLAN_LIMIT,
};

// TrackRecord -- one step's FULL introspection row, including everything
// needed to REPLAY the step offline (the inputs) and everything needed to
// diagnose it (the intermediates). This is the wire trace payload.
struct TrackRecord {
  StepInput in;                                  // replay: the exact inputs
  RefState ref;                                  // the sampled reference
  float eAlong = 0.0f, eCross = 0.0f, eTheta = 0.0f;  // [mm][mm][rad] exact arc projection
  float vTrim = 0.0f, omegaTrim = 0.0f;          // [mm/s][rad/s] post-clamp
  float vCmd = 0.0f, omegaCmd = 0.0f;            // [mm/s][rad/s] body command
  float wheelLeft = 0.0f, wheelRight = 0.0f;     // [mm/s] post-IK/saturate/clamp setpoints
  bool trimSaturated = false;
  Status status = Status::RUNNING;
};

struct StepOutput {
  WheelVelocities command;   // [mm/s] setpoints for the LEVEL-2 motor velocity PIDs
  Status status = Status::RUNNING;
  TrackRecord record;
};

class MotionPlan {
 public:
  // Default-constructed MotionPlan: the intentionally INVALID/empty plan
  // (PlanResult's own default state -- "valid iff verdict == OK"). Every
  // query below returns a safe zero/default on a plan constructed this
  // way; referenceAt() returns a zeroed RefState, duration() returns 0.
  MotionPlan() = default;

  // --- pure queries on the immutable solve ---
  float duration() const;        // [s]
  float kappa() const;           // [1/mm]
  Pose anchor() const;           // world pose at segment start
  Pose goal() const;             // world goal, frozen at plan time
  float exitSpeed() const;       // [mm/s]
  float effectiveCeiling() const; // [mm/s]|[rad/s] the folded v_eff -- dumpable
  bool isPivot() const;
  bool isVelocityMode() const;   // MOVER teleop plan
  RefState referenceAt(float elapsed) const;   // [s] closed-form, pure

  // --- the step: const on the plan; ALL mutation in *state ---
  // reference sample -> path-frame errors (exact circle projection) ->
  // P-trims (clamped; pivot mode: v == literal 0, heading-only) -> IK ->
  // curvature-preserving saturate -> one-sided wheel clamp (forward arcs)
  // -> wheel velocity setpoints. Terminal state machine per the settle
  // spec (LEVEL 2, the leaf PID, turns setpoints into duty outside).
  // Emits REPLAN_DUE (never replans itself); large pose-fix steps bypass
  // the sustain filter; small ones reset it.
  //
  // TICKET 100-003 STUB: this ticket produces the immutable, closed-form-
  // samplable plan only -- the tracker cascade (ticket 004) and the
  // policy/terminal machine + this method's real composition (ticket 005)
  // do not exist yet. This body is a harmless placeholder: it echoes `in`
  // into the returned TrackRecord (so a caller can see what it fed in),
  // reports Status::RUNNING, commands a literal-zero WheelVelocities, and
  // never touches *state. It is NOT an assert/abort -- drive_api.cpp
  // (ticket 006) and any exploratory host tooling may legitimately call
  // step() before 005 lands, and must get a well-defined, inert answer,
  // never a crash. Ticket 005 replaces this ENTIRE body; nothing here is
  // meant to survive that ticket.
  StepOutput step(const StepInput& in, StepState* state) const;

 private:
  // Drivetrain::plan()/replan()/planVelocity() are the ONLY producers of a
  // VALID MotionPlan -- befriended so they can use the private field
  // constructor below; every other caller gets a MotionPlan only by
  // copying/receiving one of THEIR return values (PlanResult::plan).
  friend class Drivetrain;

  // Full-field private constructor -- see the class comment above and the
  // member list below for what each field is and why it exists beyond the
  // sketch's own terse "Immutable solve results" comment.
  MotionPlan(MasterProfile profile, MasterProfile omegaProfile, float kappa, float masterTarget,
             const Pose& anchor, const Pose& goal, float exitSpeed, float effectiveCeiling,
             float duration, const Limits& limits, float trackwidth, bool isPivot,
             bool isVelocityMode);

  // Immutable solve results: master trajectory polynomial (Ruckig),
  // geometry (kappa, anchor, goal), limits snapshot -- exactly the
  // sketch's own private-section comment, elaborated:
  MasterProfile profile_;       // the master DOF (path-length for an arc,
                                 // heading for a pivot); ALSO the linear
                                 // (v) channel for a velocity-mode plan
  MasterProfile omegaProfile_;  // the SECOND, independently-solved angular
                                 // -rate channel -- used ONLY when
                                 // isVelocityMode_ is true (MOVER's own
                                 // (v, omega) pair is not a single
                                 // constant-curvature arc, so it needs two
                                 // profiles, not one -- see planVelocity()'s
                                 // own doc comment in drivetrain.h).
                                 // Default-constructed/unused otherwise.
  float kappa_ = 0.0f;          // [1/mm] segment curvature; 0.0f (not
                                 // meaningful) when isVelocityMode_
  float masterTarget_ = 0.0f;   // [mm] or [rad] the master DOF's absolute
                                 // target position this plan solved
                                 // toward (goal.arcLength for an arc,
                                 // goal.deltaHeading for a pivot) --
                                 // Drivetrain::replan()'s own re-solve
                                 // target (master_profile.h's seeding
                                 // contract: a re-solve reseeds the
                                 // CURRENT state and solves toward the
                                 // SAME target again). Not meaningful when
                                 // isVelocityMode_ (velocity-mode plans
                                 // are open-ended, no target position).
  Pose anchor_;                 // world pose at segment start (or
                                 // planVelocity()'s own `current.pose`)
  Pose goal_;                   // frozen world goal pose; == anchor_ for a
                                 // velocity-mode plan ("no pose goal", per
                                 // the driving issue's planVelocity() doc)
  float exitSpeed_ = 0.0f;      // [mm/s]
  float effectiveCeiling_ = 0.0f;  // [mm/s] or [rad/s] -- the folded
                                    // ceiling actually used for this
                                    // plan's solve (v_eff for an arc,
                                    // the analogous omega_eff for a
                                    // pivot, the linear-channel ceiling
                                    // for a velocity-mode plan)
  float duration_ = 0.0f;       // [s] T_plan -- profile_.duration() for a
                                 // segment plan; the caller-supplied
                                 // deadman (converted ms -> s) for a
                                 // velocity-mode plan, so the SAME
                                 // terminal machine (ticket 005) that
                                 // handles a stop segment's t >= T_plan
                                 // also handles MOVER's deadman elapsing,
                                 // per SUC-010's own "no separate watchdog
                                 // logic duplicated in the adapter" rule
  Limits limits_;                // gains/ceilings snapshot -- ticket
                                  // 004/005's tracker/policy read THIS
                                  // copy inside step(), since step() takes
                                  // no Limits parameter of its own (the
                                  // sketch's own "limits snapshot" note)
  float trackwidth_ = 0.0f;      // [mm] geometry snapshot -- same reason
  bool isPivot_ = false;
  bool isVelocityMode_ = false;
  bool valid_ = false;           // false for a default-constructed
                                  // (invalid/empty) plan; every query
                                  // above checks this first
};

}  // namespace Drive
