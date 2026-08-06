// arc_solver.h -- Motion::ArcSolver (135-002): pure-geometry tangent-arc
// solve from a pose to a target point, a C++ port of the host's
// src/host/robot_radio/pathplan/solver.py `solveArcToPoint()`/
// `SolverLimits`/`ArcSolution`/`_clampOmegaStep()`.
//
// Given a current pose and a target point, there is exactly one circular
// arc through the target that is tangent to the current heading. This
// module computes that arc's curvature, length, and cruise velocity,
// applies a curvature slew clamp and a target-behind guard, and packages
// the result as an `ArcSolution` -- the parameters for a Distance-stopped
// body-frame Twist Move (`Motion::Move::Kind::Distance`,
// `Move::VelocityKind::Twist`; see `planner/planner_types.h`), never an
// Angle-stopped Move. `ArcSolution` has no field that could express an
// Angle stop condition, so the "never change axis while moving" design
// constraint holds by construction, not by a runtime check -- exactly
// `solver.py`'s own `ArcSolution` docstring, ported verbatim in spirit.
//
// PURE GEOMETRY ONLY (135 sprint.md, Architecture Overview #1): no
// navigation policy (that's Motion::Navigator, ticket 003), no Planner
// call, no wire awareness, no Types::RobotState dependency. `solve()` is a
// pure function -- same pose/target/limits/previousOmega always produces
// the same ArcSolution, no I/O, no held state. `previousOmega` is the one
// piece of state a caller-visible safety behavior needs (the curvature
// slew clamp, a property of a SEQUENCE of calls); the caller (Navigator)
// threads it through explicitly every cycle, exactly like `solver.py`'s
// own design -- this module remembers nothing between calls itself.
//
// Units: mm / rad throughout, matching this codebase's native firmware
// convention (Types::RobotState::Otos/Pose are already mm/rad) -- unlike
// `solver.py`, which carries a cm-to-mm `_POSITION_SCALE` boundary because
// its own `nav.pose.Pose` is centimetres. No such scale factor is needed
// here.
//
// This ticket does NOT touch Motion::Move/PlannerLimits and adds no new
// Move::Kind -- sprint 135's Architecture > Design Rationale, Decision 1:
// the Planner's discrete-exact-landing proof assumes a fixed left:right
// ratio captured once at Move activation, so a per-tick re-solved
// curvature INSIDE the Planner would make that hazard the steady state.
// The arc this module computes becomes, in ticket 003, an ORDINARY Move
// built from fields that already exist on Motion::Move.
#pragma once

namespace Motion {

// --- Curvature slew-clamp budget: derivation ------------------------------
//
// This bound exists for exactly one reason: to keep a `replace=true`
// transition from landing as an abrupt step while the firmware's own
// profiler cannot ramp it cleanly (src/motion/planner/planner.cpp
// reinterprets the carried profileVelocity_ under the NEW move's own
// axis/body-speed ratio on every replace -- a firmware defect tracked at
// clasi/issues/replace-rescales-carried-profile-velocity-by-new-shape.md).
// solver.py's own module docstring measured the hazard directly (127-001):
// replacing an in-flight 150 mm/s straight with a tight arc produced a
// single-tick commanded-wheel discontinuity of 433.3333 mm/s
// (CASE2_EDGE_B_DISCONTINUITY_MM_S). This solver cannot fix the firmware's
// carry-over bug itself -- instead it bounds how much the CURVATURE is
// allowed to change between successive solves, so the caller never asks
// for a step anywhere near that hazard figure, however the carry-over bug
// reinterprets whatever it is asked for.
//
// NOT an acceleration limiter -- the firmware's own profiler already
// enforces one, on every wheel command it receives, independent of
// anything this solver does (PlannerLimits::Ceilings::aMax, currently
// 300 mm/s^2, src/firm/config/boot_config.cpp's defaultPlannerGroup()). A
// slew bound set below that figure would fight the firmware's own limiter
// for no reason; this one is sized to stay comfortably above it, so aMax
// stays the binding constraint in normal operation and this bound is only
// ever felt on the specific replace discontinuity it exists to smooth.
//
// The budget is a deliberate fraction of the wheel's own physical
// acceleration authority -- solver.py's own docstring cites the plant
// model this is anchored to: kPlantGain ~= 1370 mm/s per duty,
// kPlantTau ~= 0.23 s, giving plantAccel = kPlantGain / kPlantTau ~= 5957
// mm/s^2 (what a wheel can PHYSICALLY do). kArcSolverSlewAccel below is
// ~42% of that -- comfortably above aMax while remaining well short of the
// plant's own physical ceiling. This FRACTION carries over unchanged from
// solver.py: it is a property of the PLANT, not of which loop spends it.
inline constexpr float kArcSolverSlewAccel = 2500.0f;  // [mm/s^2]

// [s] the re-solve cadence the budget above is spent ONCE PER SOLVE of.
// solver.py derives its own MAX_WHEEL_STEP against the HOST planner
// loop's cadence (planner.CYCLE_PERIOD = 0.1 s) -- but this port's own
// caller, Motion::Navigator (135-003), re-solves on the FIRMWARE's own
// 50 ms cycle (sprint 135 sprint.md Solution: "re-solving a tangent arc
// every 50 ms cycle"; App::RobotLoop::kCycle = 50 ms, ~20 Hz). Re-deriving
// against the loop that actually spends this budget -- rather than
// copying solver.py's numeric MAX_WHEEL_STEP verbatim -- is the whole
// point of solver.py's own docstring warning: "if planner.CYCLE_PERIOD
// ever changes, _SOLVE_PERIOD here must be updated to match or this
// budget silently drifts out of sync with the loop that actually spends
// it." Here, the loop is different from the start, so the derived figure
// is different from the start too.
inline constexpr float kArcSolverSolvePeriod = 0.05f;  // [s]

// [mm/s] per solve -- the derived default for NavigatorLimits::
// maxWheelStep below (2500.0 * 0.05 == 125.0, vs. solver.py's own
// 250.0 mm/s derived against its 0.1 s host cadence). Still comfortably
// under the measured 433.3333 mm/s Edge-B hazard figure above, and well
// above aMax's own 300 mm/s^2 * 0.05 s = 15 mm/s per-solve authority, so
// aMax remains the binding constraint in normal operation. Exposed here
// (not file-local in arc_solver.cpp) so a parity ctest can lock the
// derivation formula directly, the same way solver.py's own
// test_max_wheel_step_derivation imports and asserts against its public
// MAX_WHEEL_STEP constant.
inline constexpr float kArcSolverMaxWheelStepDefault =
    kArcSolverSlewAccel * kArcSolverSolvePeriod;  // == 125.0f

// A plain (x, y, heading) point -- ArcSolver's own minimal pose type.
// Deliberately NOT Types::RobotState::Otos/Pose (both carry velocity
// fields irrelevant here and would pull a firmware-blackboard dependency
// into a module that has none) and definitely not msg::Pose2D (a wire
// type, raw/scale packed) -- this keeps ArcSolver constructible and
// testable with hand-fed numbers alone, matching every other leaf in this
// tree (src/motion/DESIGN.md §3's dependency constraint).
struct Pose {
  float x = 0.0f;        // [mm]
  float y = 0.0f;        // [mm]
  float heading = 0.0f;  // [rad] CCW-positive, 0 = +x
};

// NavigatorLimits -- the plain, motion-owned config struct ArcSolver::solve()
// (this ticket) and Motion::Navigator (135-003) are both constructed with.
// Same convention as PlannerLimits (planner/planner_types.h): no wire
// types, trivially copyable, hand-fed-constructible. One configuration
// group (one robot-JSON block, one bake path -- configuration-discipline)
// even though solve() itself only reads the first block of fields below;
// this ticket defines the struct's SHAPE only -- wiring it to the robot
// JSON / bake path is ticket 004's job.
struct NavigatorLimits {
  // --- fields ArcSolver::solve() reads -----------------------------------

  // [mm] wheel separation -- differential-drive kinematics, converts the
  // curvature slew budget (maxWheelStep) into an omega-step budget.
  // <= 0 disables the slew clamp (no meaningful wheel-speed budget to
  // convert), the same fail-open convention PlannerLimits uses throughout.
  float trackWidth = 100.0f;

  // [mm/s] commanded forward body-frame cruise speed, far from the target.
  // See approachRadius/approachSpeed below for the near-target taper that
  // solve() itself applies on top of this.
  float speed = 150.0f;

  // [mm/s] curvature slew-clamp budget: the maximum allowed per-wheel
  // commanded-speed change between successive solves. See this file's own
  // "Curvature slew-clamp budget: derivation" comment above for the full
  // first-principles derivation -- ported from solver.py's module
  // docstring but re-anchored to THIS firmware's own 50 ms Navigator
  // re-solve cadence, not the host's 100 ms one (NOT a blind copy of
  // solver.py's own 250.0 default).
  float maxWheelStep = kArcSolverMaxWheelStepDefault;

  // [rad] ~pi/2 (90 deg): a target whose body-frame bearing magnitude
  // exceeds this has no finite-radius tangent arc -- triggers the
  // target-behind guard (ArcSolution::stop) rather than an extreme,
  // physically-unrealizable arc. Cutoff is exclusive (bearing == exactly
  // this value does NOT stop), matching solver.py's own `> limits.
  // behindAngle` comparison.
  float behindAngle = 1.5707963267948966f;

  // --- fields ArcSolver::solve() does NOT read; Motion::Navigator
  //     (135-003) reads them for its own state-machine policy. Carried on
  //     THIS struct, not a separate one, because all of NavigatorLimits is
  //     one configuration group even though only the block above feeds
  //     this ticket's own pure-geometry solve(). ----------------------

  // [rad] ~50 deg (goto_otos.py's own TURN_FIRST): bearings at or beyond
  // this stop the in-flight arc and pivot toward the target first, rather
  // than steering the error out with curvature alone (SUC-004). Not
  // consulted by solve() itself -- Navigator's own policy decides whether
  // and when to pivot; solve() only ever reports `bearing`.
  float turnFirstAngle = 0.8726646259971648f;

  // [mm] distance from the target within which solve()'s own cruise speed
  // tapers from `speed` down to `approachSpeed` -- tracking accuracy, NOT
  // feasibility (Motion::Planner's own shapeLimits() elsewhere already
  // handles feasibility; this is not a re-implementation of that). 0 =
  // taper disabled (fail-open, matching PlannerLimits' own <= 0
  // convention) -- every direct caller that never sets this (ctests, the
  // ctypes bench harness) gets the untapered constant `speed` exactly as
  // before this field existed.
  float approachRadius = 0.0f;

  // [mm/s] the floor speed the taper above linearly ramps down to,
  // reached at zero distance from the target. Ignored when approachRadius
  // <= 0.
  float approachSpeed = 0.0f;

  // [mm] fallback arrival tolerance a GotoTarget applies when its own
  // `tolerance` field arrives <= 0 (navigator.h's GotoTarget doc comment,
  // the fail-open convention every <=0 bound in this codebase uses) --
  // ported from pathplan.planner.TERMINATION_TOLERANCE (the host's own
  // provisional default) into config (135-004), matching this default's
  // own prior value exactly (was a compile-time kNavDefaultArrivalTolerance
  // constant, navigator.h; superseded here so every value the robot uses
  // comes from the file, configuration-discipline.md).
  float defaultArrivalTolerance = 100.0f;

  // Sign relating commanded Move::omega to true-world CCW (135-004,
  // "Landmine 4" -- see navigator.h's own NavigatorLimits::yawSign-area
  // comment for the
  // full derivation). +-1 only, dimensionless. +1.0 (default): no
  // correction -- Move::omega already agrees with the convention
  // Navigator's own world-frame bearing solve uses (every ctest/sim
  // plant's own implicit convention, since none of them can independently
  // model a real drivetrain's own commanded-omega-vs-world-CCW quirk).
  // -1.0: this robot's commanded Move::omega is measured OPPOSITE
  // true-world CCW (matches src/tests/bench/goto_otos.py's own
  // `YAW_SIGN`) -- set per-robot from a real bench pass (ticket 006),
  // never guessed. Not read by ArcSolver::solve() itself -- Navigator
  // applies it at its own pose-building and Move-construction boundary.
  float yawSign = 1.0f;
};

// One solve()'s result -- either an arc twist or a stop signal.
struct ArcSolution {
  // [mm/s] commanded forward body-frame speed. 0.0 when `stop`.
  float v_x = 0.0f;
  // [rad/s] commanded body-frame yaw rate, CCW-positive (Pose::heading's
  // own sign convention). 0.0 when `stop`.
  float omega = 0.0f;
  // [mm] the Distance stop condition -- ALWAYS Linear-axis, never Angle
  // (see this header's own top comment). 0.0 when `stop`.
  float arcLength = 0.0f;
  // True when the target-behind guard (or the zero-distance degenerate
  // case) fired. The caller should halt (estop()/plannedStop(), not drive
  // an arc) and is expected to turn in place toward `bearing` separately
  // -- safe because that axis change then happens from rest.
  bool stop = false;
  // [rad] signed body-frame bearing to the target (0 = straight ahead,
  // positive = left, matching the omega sign convention). Always the real
  // computed bearing when `stop` fired from the target-behind guard; 0.0
  // for the zero-distance degenerate case (no direction to turn toward)
  // and for an ordinary (non-stop) solution.
  float bearing = 0.0f;
};

namespace ArcSolver {

// Compute the single circular arc from `pose` to `target` that is tangent
// to `pose.heading`, curvature-slew-clamped against `previousOmega` and
// guarded against a target with no finite-radius tangent arc.
//
// `target` is a full Pose (x, y, heading) so a future holonomic
// drivetrain could reuse this signature, but THIS differential-drive
// solver IGNORES `target.heading` -- only `target.x`/`target.y` matter
// (terminal-heading honoring is an explicit sprint 135 non-goal, sprint.md
// Out of Scope).
ArcSolution solve(const Pose& pose, const Pose& target, const NavigatorLimits& limits,
                   float previousOmega);

}  // namespace ArcSolver

}  // namespace Motion
