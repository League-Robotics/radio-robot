// planner_types.h -- the plain, motion-owned value types the Planner's
// public surface speaks: Move (what to do), PlannerLimits (construction
// config), TickResult (per-cycle completion event). No msg::*, no wire
// types -- the base's dispatch converts wire msg::Move -> Motion::Move at
// the boundary (motion-planner sketch §2). All structs are trivially
// copyable and C-layout so the ctypes harness mirrors them directly.
#pragma once

#include <cstdint>

namespace Motion {

// One bounded motion. Direction comes from the velocity sign; thresholds
// are positive magnitudes (protocol-v4 Move semantics, minus wire baggage).
struct Move {
  // Kind -- what ENDS this Move. Time/Distance/Angle are stop CONDITIONS
  // measured against the clock or odometry. `Stop` is the odd one out and
  // deliberately so: it is the PLANNED STOP queue entry
  // (command-ingestion-ring-buffered-comms-subsystem-routing-two-stops.md
  // §5, the wire's `STOP` verb) -- "come to a stop when you reach THIS
  // point in the queued sequence." It gets its own enumerator rather than a
  // bool on the side because "a Move that is a stop" and "a Move that stops
  // when X" are different in kind, and a bool would leave every switch here
  // silently doing the wrong thing for it.
  enum class Kind : uint8_t { Time, Distance, Angle, Stop };
  enum class VelocityKind : uint8_t { Twist, Wheels };

  uint32_t id = 0;
  Kind kind = Kind::Time;
  float threshold = 0.0f;  // [ms] Time / [mm] Distance / [rad] Angle; positive
  float timeout = 0.0f;    // [ms] safety backstop; 0 = none
  VelocityKind velocityKind = VelocityKind::Twist;
  float v_x = 0.0f;        // [mm/s] signed cruise, Twist
  // Carried so a holonomic drivetrain needs no type change, and because a
  // twist HAS three components (naming-and-style rule 2) -- but accepted
  // and ignored on this differential build, exactly as the wire's own
  // MoveTwist.v_y is. A Distance Move with v_x == 0 and v_y != 0 is a
  // pure-sideways motion this drivetrain cannot make, and move() rejects
  // it rather than silently driving nothing.
  float v_y = 0.0f;        // [mm/s] signed cruise, Twist; ignored on differential
  float omega = 0.0f;      // [rad/s] signed cruise, Twist
  float vLeft = 0.0f;      // [mm/s] signed cruise, Wheels
  float vRight = 0.0f;     // [mm/s] signed cruise, Wheels
  // What the CALLER asked for, before any upstream rewrite of `threshold`
  // (134-001). `threshold` is the ACTUATION-sized command the profiler
  // plans against; an ingestion-side corrector may legitimately size it to
  // something other than the request -- App::RobotLoop::handleMove()'s
  // rotation-calibration inversion does exactly that, turning a 90 deg
  // request into whatever command lands 90 deg on this plant. Both readers
  // are right and they want different numbers: the PROFILER wants the
  // command, the Planner's cumulative-baseline LEDGER wants the intent
  // ("how much heading was this Move meant to contribute to the chain?"),
  // and without a second field the ledger cannot recover it.
  //
  // <= 0 means UNSET, and the ledger then falls back to `threshold` -- the
  // same fail-open convention the rest of this surface uses for an
  // unconfigured <=0 bound ("contributes nothing", not "zero"). For every
  // direct caller with no ingestion-side rewrite in front of it (the
  // ctests, the sim harnesses, the ctypes bench harness) the command IS
  // the intent, so the fallback is the right answer, not a degraded one.
  // Note it is NOT the same answer 130-010's measured carry gave those
  // callers: an unset Move now projects an EXACT baseline+threshold
  // instead of whatever pose_.heading() read at the completion tick, which
  // typically ran ~1 deg behind true rest. That difference is the point of
  // this change, not a side effect of the default.
  // [ms] Time / [mm] Distance / [rad] Angle; positive. 0 = unset.
  float requestedThreshold = 0.0f;
  // APPEND NEW FIELDS TO THE END. src/tests/bench/planner_harness.py
  // mirrors this struct field-for-field over ctypes; capi.cpp's
  // plannerStructSizes() is the only guard, and it is size-only.
};

// PlannerLimits -- 130-009 (planner-honesty-pass-50ms-period-tick-state-
// machine-limits-reduction.md item 3) reshaped this struct from 34 flat
// fields to 18, grouped into four coherent sub-structs, breaking the
// append-only ctypes-mirror constraint ONCE, deliberately (a single,
// contained ABI break -- sprint 130's own Migration Concerns). The sprint's
// own planning text targeted 23 fields (keeping five more, the old
// velocity-domain trim gains trimKp/trimKi/trimIMax/trimKaff/trimMax under
// a `tracking` group) -- this ticket found, by direct evidence (grep
// against planner.cpp/planner.h), that those five ALSO have zero readers:
// 130-005 deleted Motion::WheelTrim (the only thing that ever read them)
// and left the fields themselves behind, orphaned exactly like the M4
// duty-stage fields ticket 007 orphaned. Per this ticket's own instruction
// to work from evidence rather than the target count, they are cut here
// too -- see the ticket's own completion note for the full justification.
// That leaves `tracking` holding just `headingHoldGain` (still genuinely
// read, planner.cpp's applyHeadingHold()).
//
// Cut (16, not 11): `velKff`/`velKp`/`velKi`/`velIMax`/`velKaff`/
// `velIAccelGate`/`dutyFloor` (the M4 duty stage, deleted outright by
// 130-007 -- WheelPid/stageDuty() had already deleted the code that
// consumed them, these were the trailing dead config fields);
// `headingOtosWeight`/`otosStaleness` (the OTOS heading-blend feature --
// a live code path in Planner::tick() as of this ticket, but out of
// SCOPE by explicit sprint decision, tracked forward to
// clasi/issues/later/estimator-v2-otos-fusion-sim-first.md -- the blend
// call site in tick() is deleted alongside these two fields);
// `requireSettle`/`settleWindow` (the settle-confirm DEFER path, dissolved
// by 130-008's `Settling`-state deletion -- `plannedStopWindow()` was the
// one other reader of `settleWindow` and now falls back to its built-in
// default allowance); `trimKp`/`trimKi`/`trimIMax`/`trimKaff`/`trimMax`
// (the planner-side velocity trim, superseded and left dead by 130-005's
// move of wheel actuation into App::Drive -- see above).
struct PlannerLimits {
  // Profile ceilings: the ideal-plant magnitude bounds the trapezoid/
  // S-curve profiler plans against, body-frame linear and angular.
  struct Ceilings {
    float vMax = 600.0f;      // [mm/s] linear velocity ceiling
    float aMax = 0.0f;        // [mm/s^2] linear accel-ramp ceiling
    float aDecel = 0.0f;      // [mm/s^2] linear decel-taper ceiling
    float omegaMax = 8.0f;    // [rad/s] angular velocity ceiling
    float alphaMax = 0.0f;    // [rad/s^2] angular accel-ramp ceiling
    float alphaDecel = 0.0f;  // [rad/s^2] angular decel-taper ceiling
    // Jerk ceilings (S-curve shaping): bound how fast the PROFILE's own
    // acceleration may change, and taper acceleration to zero exactly at
    // cruise (no instant accel corners -> no whiplash). 0 = disabled
    // (plain trapezoid, all pre-existing behavior unchanged).
    float jerkMax = 0.0f;     // [mm/s^3] linear axis
    float yawJerkMax = 0.0f;  // [rad/s^3] angular axis
  } ceilings;

  // Plant/loop facts: the physical wheelbase and the discrete timing every
  // exactness calculation assumes.
  struct Plant {
    float trackWidth = 100.0f;    // [mm] wheel separation
    float controlPeriod = 50.0f;  // [ms] the loop interval one tick() plans for
    float actuationDelay = 0.0f;  // [ms] command-staged-to-wheels latency compensated in planning
    float velocityFilterWeight = 1.0f;  // EMA weight on fresh encoder velocity; 1 = unfiltered
  } plant;

  // Arrival/landing: "close enough to the target, and at rest" for THIS
  // robot's fine-positioning resolution, plus the decel-authority leeway
  // the profile plans its brake-start against.
  struct Landing {
    // Settle-confirm arrival tolerances. On an ideal plant the defaults are
    // easily reachable; on a stiction plant whose minimum creep step is one
    // duty pulse per control period, an epsilon finer than that step is
    // unreachable.
    float settleEpsilonLinear = 1.0f;     // [mm]
    float settleEpsilonAngular = 0.005f;  // [rad]
    // Settle rest floors -- "at rest" thresholds on the FILTERED measured
    // velocity. A per-robot property: they must sit ABOVE the robot's own
    // filtered velocity-noise floor (else rest never confirms) and LOW
    // enough that the post-settle coast (~floor * plant tau) stays inside
    // the arrival epsilons.
    float settleRestVelocity = 5.0f;  // [mm/s]
    float settleRestOmega = 0.02f;    // [rad/s]
    // Decel LEEWAY: the fraction of the decel ceiling the profile plans its
    // brake-START against (profile.h AxisLimits::aDecelPlan). 0 or 1 means
    // "plan at full authority", the pre-existing behavior.
    //
    // Below 1 the profile commits to braking sooner and rides a gentler
    // ramp, holding the rest of the authority in reserve. That matters on a
    // plant with a real time constant: braking at the full ceiling, the
    // wheel cannot follow the command down, so the command reaches zero
    // while the wheel is still moving and the body COASTS ~v*tau past the
    // target (measured here as +6.5 deg of overshoot past the last turn of
    // a square tour). A gentler planned ramp is one the plant can actually
    // track, so it arrives at the boundary already slow.
    //
    // This is decel-AUTHORITY headroom, not command-authority headroom: the
    // profile still commands all the way down to the boundary, and the
    // terminal step and the per-step floor still use the FULL ceiling, so
    // the landing stays discrete-exact and an infeasible state still brakes
    // as hard as it can.
    float decelPlanFraction = 0.0f;  // [1] 0 or 1 = full authority

    // TERMINAL FINE-ALIGN (134-003, docs/bench-reports/motion-planning-lab-
    // 2026-08-04.md §5.2/§3). After a Twist Angle Move's profile has
    // landed, the Planner holds the Move open in MoveLifecycle::Aligning
    // and trims the residual against the cumulative-intent ledger with
    // bounded low-speed pivot nudges, re-settling between each. Measured
    // on `tovez` by the host-side graft this reproduces: planner square-tour
    // closure 25.8 -> 9.4 mm, ~2 s/corner, nothing else changed.
    //
    // alignTol is RADIANS. The report states the operating point in
    // DEGREES (1.0 deg); the conversion happens once, in the robot JSON
    // (0.017453 rad), and never again -- these are siblings of
    // settleEpsilonAngular [rad] and settleRestOmega [rad/s], not of
    // anything measured in degrees.
    //
    // NEITHER IS A TUNING KNOB TO REACH FOR. Both come from 333 individual
    // trim nudges: the low-speed corrective pivot is bimodal (26% deliver
    // <0.25 deg, no breakaway; the rest a median 1.72 deg), so a tolerance
    // under that ~1.8 deg quantum asks for authority the mechanism does not
    // have -- at 0.3 deg, corner convergence measured 93% -> 64%, cost
    // tripled, and some corners got WORSE. The road below ~8 mm of closure
    // is a finer terminal ACTUATOR (report §5.4), not a finer threshold.
    //
    // <= 0 on EITHER field is UNSET and disables the phase outright (the
    // same fail-open convention the rest of this struct uses for an
    // unconfigured bound): a Move then completes at its landing exactly as
    // it did before this feature existed, which is what every direct
    // caller that never sets these -- the ctests, the sim harnesses, the
    // ctypes bench harness -- gets by construction from these defaults.
    float alignTol = 0.0f;       // [rad] 0 = fine-align disabled
    // int32, not a narrower count type: it keeps Landing's uniform 4-byte
    // stride, so no padding appears between it and whatever is appended
    // next and capi.cpp's flat offset guard stays meaningful.
    int32_t alignMaxNudges = 0;  // 0 = fine-align disabled
  } landing;

  // Angular tracking correction on top of the profiled command.
  struct Tracking {
    // Heading hold on Distance Moves: P gain on the UNCOMMANDED angular
    // axis, driving heading back to the Move's activation baseline. The
    // correction is purely differential, so the profiled path length --
    // and therefore the distance exactness -- is untouched. 0 = off.
    float headingHoldGain = 0.0f;  // [1/s] rad/s of correction per rad of error
  } tracking;

  // APPEND NEW FIELDS TO THE END OF WHICHEVER SUB-STRUCT THEY BELONG IN
  // (or add a new sub-struct at the end of PlannerLimits itself).
  // src/tests/bench/planner_harness.py mirrors this struct field-for-field
  // over ctypes, sub-struct for sub-struct; inserting mid-struct silently
  // scrambles every field after the insertion point on the Python side, and
  // a same-size insertion passes a size-only guard. plannerLimitsOffsets()
  // checks per-field OFFSETS too, which is what catches it.
};

// Which regime the ACTIVE Move is in, folded from both wheels' StepPhase
// (profile.h) and latched so it can never run backwards: a Move
// accelerates, holds, then decelerates, and never accelerates again once
// it has begun braking. This is `Tracking`'s own sub-phase (MoveLifecycle,
// below) -- a sibling of nothing at the top level.
//
// `Settle` (the settle-confirm closed-loop terminal creep's own phase) is
// DELETED by 130-008 along with the defer path that gated it
// (PlannerLimits::requireSettle's doc comment) -- it was never actually
// assigned even before this ticket (grep-verified), so removing the
// enumerator changes no behavior.
enum class MovePhase : uint8_t {
  Idle,     // no active Move, or the queue is draining
  Accel,    // climbing toward cruise
  Hold,     // at cruise, command unchanged -- the only phase that integrates
  Decel,    // braking toward the landing boundary
};

// The Move lifecycle Planner::tick() dispatches over (130-008,
// planner-honesty-pass-50ms-period-tick-state-machine-limits-reduction.md
// item 2) -- the explicit top-level state that replaces the implicit one
// the previous `occupied`/`hasMoved`/`settling` boolean combination
// encoded. MovePhase (above) is `Tracking`'s own sub-phase, not a sibling
// of these. Full transition table and event definitions: Planner::tick()'s
// own doc comment in planner.cpp.
enum class MoveLifecycle : uint8_t {
  Idle,       // no active Move; the staged command is already zero
  Draining,   // no active Move; ramping the staged command toward zero
  Breakaway,  // active Move, arrival/stall detection not yet armed (has not
              // measurably left rest)
  Tracking,   // active Move driving its profile -- see MovePhase for the
              // Accel/Hold/Decel sub-phase
  Stopping,   // an active Kind::Stop entry ramping the body to rest
};

// The per-tick completion event, returned by tick() (never written into
// RobotState -- acks are protocol bookkeeping, not robot state). At most
// one Move can end per tick.
struct TickResult {
  bool completed = false;
  uint32_t moveId = 0;
  bool timedOut = false;
  // True when the Move was, at the moment it completed, both within the
  // arrival epsilon of its target and at rest. Always evaluated and
  // reported truthfully -- the settle-confirm DEFER path that used to hold
  // a completion back until this was true (`PlannerLimits::requireSettle`,
  // deleted by 130-008/130-009) is gone; `settled` is simply computed at
  // whichever tick the Move actually completes on.
  bool settled = false;
};

}  // namespace Motion
