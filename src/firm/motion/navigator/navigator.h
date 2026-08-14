// navigator.h -- Motion::Navigator (135-003): the navigation-POLICY module
// that drives one goto target to completion by repeatedly calling
// Motion::ArcSolver::solve() (135-002) and issuing internal
// Planner::move(next, replace=true)/plannedStop() calls into a REAL
// Motion::Planner it holds a reference to.
//
// Dependency boundary (sprint.md Architecture Overview #2, src/firm/motion/
// DESIGN.md §3): depends on Motion::Planner's public surface (move(),
// plannedStop(), estop(), active(), lifecycle(), tick()'s TickResult) and
// Types::RobotState (for OTOS pose) -- NO other src/firm dependency. Does
// NOT parse the wire (ticket 004) and does NOT solve arc geometry itself
// (delegates to ArcSolver, ticket 002).
//
// --- Ownership of Planner::tick()/update() -----------------------------
//
// Navigator::tick() calls planner_.tick(state) and planner_.update(state)
// ITSELF, once, internally -- it does not merely decide and let the
// caller drive the Planner separately. This is the "one owner" rule
// (sprint.md Architecture Overview #3) applied literally: whichever
// subsystem currently owns Drive (Drive teleop / a plain Planner Move /
// this Navigator) is the ONE thing that may call planner_.tick() in a
// given cycle -- two calls in one cycle would corrupt Planner-internal
// per-cycle accounting (stallTicks, cmdLeftPrevious_, ...). The caller
// (ticket 004's Core::RobotLoop::cycle()) is expected to call EITHER
// navigator_.tick(state) (a goto owns Drive) OR its own direct
// planner_.tick(state)/update(state) pair (a plain MOVE/WHEELS owns
// Drive) -- never both in the same cycle.
//
// Per-cycle causality inside tick(): planner_.tick(state) executes
// whatever was STAGED last cycle (a Move only activates on the Planner's
// OWN next tick() after move()/plannedStop() stages it -- planner.h's own
// doc comment) and returns this cycle's TickResult; Navigator then reads
// that TickResult plus the CURRENT state to decide what to stage for the
// Move that will activate NEXT cycle. This is "decide, then act next
// cycle" -- the same one-cycle bootstrap latency every other
// Planner::move() caller in this codebase already accepts.
//
// --- OTOS sign convention (settled 2026-08-13, superseding ticket 008) ---
//
// ONE convention, everywhere: ROS REP-103. Right-handed, x forward, y
// left, angles CCW-positive. state.otos.heading (the raw wire value),
// Core::Odometry's encoder-derived state.pose.heading, Move::omega,
// ArcSolution::omega, Pose::heading and the overhead camera are all in it.
// There is nothing to reconcile and this file performs NO sign flip when
// it builds its working Pose from state.otos -- see navigator.cpp's
// tick().
//
// This paragraph used to say the opposite: that the wire heading was "the
// hardware-mounted sign... the NEGATION of encoder-derived heading", and
// that this file therefore had to negate it the same way planner.cpp's
// applyOtosHeading() call did. The underlying MEASUREMENT was real (one
// rotation read +84.58deg optical against -82.45deg encoder) but the
// diagnosis was backwards. The OTOS was right and the ENCODERS were
// wrong: on tovez the firmware's "left" wheel was physically its RIGHT
// wheel, because gen_boot_config.py hardcoded LEFT_PORT=1/RIGHT_PORT=2
// while that robot is wired port 1 = right. Swapping the two labels
// negates omega = (vR - vL) / b, hence every encoder-derived heading,
// while leaving forward motion correct -- which is exactly why it hid for
// so long and got patched THREE times downstream (this file's negation,
// planner.cpp's, and sim_plant.cpp's kOtosHardwareMountSign) instead of
// once at the source.
//
// The source fix is per-robot `motors.left_port`/`right_port` in the
// robot JSON (gen_boot_config.py's _drive_ports()), moved TOGETHER with
// that robot's paired *_left/*_right calibrations. Verified on tovez
// against overhead-camera truth: one physical CCW rotation measured
// camera +79.1deg, OTOS raw +80.5deg, encoder odometry +79.9deg.
//
// `Kinematics::Differential` keeps the standard omega = (vR - vL) / b and
// must NOT be "fixed" to compensate -- src/tests/sim/unit/
// test_kinematics.py asserts that formula by name. If some angle looks
// backwards, the bug is a stale assumption somewhere, not a missing flip.
//
// With one convention there is no longer an encoder-frame/world-frame
// split to navigate: `pose` serves the arrival/frozen distance check
// (rotation-invariant anyway) AND the SUC-005 dead-reckoning fallback
// (`lastGoodPose_`/`lastGoodWheelPose_` differenced against `state.pose`,
// which is now the SAME convention, by construction) AND, via the
// now-identity `worldPose` below, the world-frame bearing-to-target
// computation.
#pragma once

#include <cstdint>

#include "arc_solver.h"
#include "planner.h"
#include "types/robot_state.h"

namespace Motion {

// --- Material-change replace-throttle (step 4 of the state machine) ----
//
// Ported from src/host/robot_radio/pathplan/planner.py's ReplaceThreshold
// (_shouldReplace()) -- SAME numeric defaults, because these are ordinary
// loop-pacing knobs (not a cycle-period-derived physical budget like
// arc_solver.h's kArcSolverMaxWheelStepDefault), so re-deriving them
// against this firmware's 50 ms cadence buys nothing solver.py's own
// values don't already give: omegaThreshold is sized above the curvature
// slew clamp's own per-solve step so a still-converging sequence keeps
// issuing while a stable solution stops; arcLengthThreshold absorbs
// ordinary per-cycle pose noise; refreshFraction is dimensionless and
// cadence-independent by construction. Exposed here (not file-local in
// navigator.cpp) so a ctest can assert against them directly, mirroring
// arc_solver.h's own kArcSolverSlewAccel convention.
inline constexpr float kNavOmegaReplaceThreshold = 0.05f;      // [rad/s]
inline constexpr float kNavArcLengthReplaceThreshold = 15.0f;  // [mm]
inline constexpr float kNavRefreshFraction = 0.5f;             // [1]

// --- Omega sign at the Navigator/Planner command boundary (135-004,
// "Landmine 4") -----------------------------------------------------------
//
// SUPERSEDED 2026-08-13, kept because the reasoning is instructive. This
// block used to assert that "commanded Move::omega is opposite true-world
// CCW ON REAL HARDWARE", citing every bench script's own `YAW_SIGN =
// -1.0`, and insisted it was "a SEPARATE fact" from the OTOS-mount sign
// above -- "both are real, both apply, and they compose by simple
// sequencing, not cancellation."
//
// They were not two facts. They were one: tovez's swapped drive ports
// (see this header's OTOS sign convention section). A robot whose "left"
// wheel is physically its right wheel has BOTH symptoms at once -- its
// encoder heading runs backwards AND its commanded omega turns it the
// wrong way -- because both come from the same omega = (vR - vL) / b with
// the labels transposed. Fixing the port binding retires both symptoms
// together, and tovez.json's `navigator.yaw_sign` is now +1.0.
//
// `Motion::ArcSolver::solve()` (135-002) is pure geometry: fed a pose, it
// returns an omega in THAT pose's own heading convention (arc_solver.h's
// own `ArcSolution::omega` doc: "CCW-positive, Pose::heading's own sign
// convention"). Now that `pose`, `worldPose`, `Move::omega` and the world
// are ALL one convention, that is the only convention in play.
//
// `NavigatorLimits::yawSign` (below) stays a CONFIGURABLE field, per
// configuration-discipline.md, and stays at its `+1.0` default -- which
// makes `worldPose.heading` equal `pose.heading` exactly and both of this
// file's `limits_.yawSign * ...` multiplications identities. It is now an
// escape hatch, not a correction anyone is expected to need: reach for it
// ONLY for a robot measured to turn opposite its commanded omega with its
// drive ports already correctly bound. A robot that needs `yawSign =
// -1.0` should be suspected of a mis-bound `motors.left_port`/
// `right_port` FIRST, because that is what the -1.0 here was really
// standing in for.
//
// NOTE the sim cannot referee this choice for you: SimPlant/IdealPlant
// both read `Move`-commanded wheel velocities directly as their own
// ground truth, so they are tautologically self-consistent with whatever
// convention Navigator uses. `Core::BootOverrides::navigatorYawSign`
// pins the sim harnesses to +1.0 for exactly that reason. Only a camera
// pass on real hardware can measure this.
//
// Internal bookkeeping that threads omega between successive solve()
// calls (`previousOmega_`, `lastIssuedOmega_`) stays in solve()'s own
// native (worldPose-relative) convention throughout, matching what
// `solve()` itself expects back as `previousOmega` -- only the
// `Move::omega` this file assigns for `planner_.move()` gets the flip.

// Provisional default arrival tolerance when a GotoTarget arrives with
// tolerance <= 0 (the fail-open convention every <=0 bound in this
// codebase uses) -- ported from pathplan.planner.TERMINATION_TOLERANCE,
// the host's own provisional default, pending the same future
// measurement that constant's own docstring cites. MOVED into
// NavigatorLimits::defaultArrivalTolerance (135-004, arc_solver.h) --
// every value the robot uses comes from the file, configuration-
// discipline.md; this compile-time constant is retired, not kept as a
// second source of the same default.

// Bounded window (SUC-005) a sustained state.otos.connected == false may
// run before Navigator gives up and aborts the goto. No existing
// NavigatorLimits/PlannerLimits field covers this (PlannerLimits' own
// otosStaleness field was cut outright, 130-009 -- see planner_types.h's
// own comment -- tracked forward to estimator-v2, not restored here);
// this is a Navigator-owned policy constant, not a physical plant
// property. 1 s bridges a momentary I2C hiccup (measured OTOS dropouts in
// this codebase are on the order of tens of ms) while keeping the
// dead-reckoned position error bounded to whatever the wheel odometry
// drifts by in that same second -- long enough to not be trigger-happy,
// short enough that the bounded fallback in SUC-005's own wording stays
// meaningfully bounded.
inline constexpr uint32_t kNavOtosDisconnectAbortWindow = 1000;  // [ms]

// Per-segment/pivot Move timeout (the ticket text's "per-segment safety
// backstop") -- ported from pathplan.planner._moveTimeoutFor()'s own
// derivation (ideal duration * a generous multiplier, plus a floor
// covering accel/decel ramps and staging latency, capped so a degenerate
// solve cannot request an enormous backstop). Loop-cadence-independent
// (a travel-time budget, not a per-cycle one), so reused verbatim rather
// than re-derived, the same way arc_solver.h reused behindAngle/
// turnFirstAngle verbatim while re-deriving maxWheelStep.
inline constexpr float kNavSegmentTimeoutMultiplier = 3.0f;
inline constexpr float kNavSegmentTimeoutFloor = 3000.0f;  // [ms]
inline constexpr float kNavSegmentTimeoutCap = 20000.0f;   // [ms]

// One goto request, already resolved to WORLD frame -- RobotLoop::
// handleGoto() (ticket 004) resolves a ROBOT-frame request to world once,
// at acceptance (sprint.md SUC-001 step 1), so Navigator itself never
// sees a frame tag or does frame math.
struct GotoTarget {
  // Wire-visible goto id. ALWAYS distinct, by construction, from the
  // internal segment Moves Navigator itself issues (Move::id == 0,
  // "internal segment" -- see tick()'s own Move-construction site):
  // ticket 004's TickResult routing depends on that distinction to keep
  // an internal segment from reaching the unconditional MOVE-completion
  // ack path (sprint.md "What Changed").
  uint32_t id = 0;
  float x = 0.0f;          // [mm] world frame
  float y = 0.0f;          // [mm] world frame
  // [mm] arrival tolerance. <= 0 -> NavigatorLimits::defaultArrivalTolerance
  // (fail-open, matching every other <=0 bound in this codebase).
  float tolerance = 0.0f;
  // [ms] overall goto safety backstop, independent of the per-segment
  // Move timeouts below it. 0 = none (fail-open, matching Move::timeout's
  // own "0 = none" convention).
  uint32_t timeout = 0;
  // [mm/s] cruise speed override for THIS goto only -- 135-004, wire
  // parity with envelope.proto's GoTo.speed ("cruise; 0 = config
  // default"). <= 0 -> NavigatorLimits::speed (the config default),
  // fail-open matching every other <=0 bound in this codebase. Applies
  // ONLY to the cruise arc's own commanded speed (ArcSolver::solve()'s
  // `speed`/the segment timeout derived from it) -- deliberately NOT
  // pivotOmega(), which this field's own wire doc names "cruise", a
  // linear-motion concept distinct from pivot rotation rate.
  float speed = 0.0f;
};

// Navigator's own per-cycle completion event -- TickResult's sibling, the
// same one-shot-per-ending-cycle shape (at most one true `completed` per
// goto, structurally: phase_ latches to Idle the same tick `completed`
// fires, and every earlier return in tick() guards on phase_ first).
struct NavResult {
  bool completed = false;  // true exactly the one tick this goto ends
  uint32_t id = 0;         // the GotoTarget::id that ended
  // true = Aborted (SUC-005 OTOS-disconnect timeout, or the goto's own
  // overall timeout); false = Done (arrived, at rest -- SUC-001).
  bool fault = false;
};

class Navigator {
 public:
  Navigator(const NavigatorLimits& limits, Planner& planner)
      : limits_(limits), planner_(planner) {}

  // Begin a new goto, replacing whatever goto (if any) was previously
  // active. No "who sent this" branch (sprint.md Decision 2): the caller
  // (ticket 004's RobotLoop) uses this SAME entry point both for a fresh
  // GO_TO acceptance and for a streamed EXTERNAL target update arriving
  // mid-goto -- INTERNAL mode is simply the degenerate case of the
  // Navigator re-solving against its own last-accepted target every
  // cycle.
  void start(const GotoTarget& target);

  // Abandon the active goto with NO completion ack (SUC-003 preemption --
  // a MOVE/WHEELS/ESTOP arriving mid-goto). Does not itself touch the
  // Planner: the caller is already about to issue its own MOVE/WHEELS/
  // estop() through the normal path, which supersedes whatever this
  // Navigator last issued via replace=true. This IS the "Drive ownership
  // released" signal ticket 004 coordinates against -- active() goes
  // false the instant this returns, exactly as it does the tick()
  // `completed` fires.
  void cancel();

  // True while this Navigator owns a goto -- false before start() and
  // again from the exact tick() call whose NavResult::completed is true
  // (or from cancel()). Ticket 004's own "release Drive ownership" signal:
  // once false, nothing further will be issued to the Planner, so
  // ownership may be handed elsewhere.
  bool active() const { return phase_ != Phase::Idle; }

  // Drive one 50 ms cycle. See this file's header comment for why this
  // call owns planner_.tick()/update() itself -- update() writes
  // cmdVelocity back into `state`, so this takes a mutable reference,
  // matching Planner::update()'s own signature.
  NavResult tick(Types::RobotState& state);

  // Observability (tests / telemetry derivation), same convention as
  // Motion::Planner's own active()/pendingCount()/etc. replaceCount()
  // counts only ordinary cruise re-issues (the material-change/half-arc
  // throttle decision) -- a pivot Move's own issue is a different kind of
  // event and is not counted here.
  uint32_t replaceCount() const { return replaceCount_; }
  uint32_t tickCount() const { return tickCount_; }

 private:
  enum class Phase : uint8_t { Idle, Active };
  enum class PivotPhase : uint8_t { None, StoppingForPivot, Pivoting };

  NavResult doneResult();
  NavResult abortResult();
  // `worldPose`: the TRUE-world-frame pose (see NavigatorLimits::yawSign's
  // own comment
  // above) -- NOT the encoder-sign `pose` local to tick(). Both functions
  // compute a bearing via bodyOffset() and need the same true-world
  // convention ArcSolver::solve() does.
  void beginPivotSequence(const Pose& worldPose);
  void issuePivotMove(const Pose& worldPose);
  float pivotOmega() const;                        // [rad/s]
  float segmentTimeout(float arcLength, float speed) const;  // [mm] [mm/s] -> [ms]
  float pivotTimeout(float bearingMagnitude) const;           // [rad] -> [ms]

  const NavigatorLimits& limits_;
  Planner& planner_;

  Phase phase_ = Phase::Idle;
  PivotPhase pivotPhase_ = PivotPhase::None;
  GotoTarget target_{};
  bool justStarted_ = false;
  uint32_t startCycleStart_ = 0;  // [ms]

  // OTOS connectivity / staleness bookkeeping (SUC-005).
  bool haveObservedOtosSample_ = false;
  uint32_t lastOtosSampleTime_ = 0;  // [ms]
  bool disconnected_ = false;
  uint32_t disconnectStartCycle_ = 0;  // [ms]
  bool haveGoodPose_ = false;
  Pose lastGoodPose_{};       // encoder-sign world pose, last connected cycle
  Pose lastGoodWheelPose_{};  // state.pose snapshot, same cycle

  // Arc-solve / replace-throttle state.
  float previousOmega_ = 0.0f;
  bool hasIssued_ = false;
  float lastIssuedOmega_ = 0.0f;
  float lastIssuedArcLength_ = 0.0f;
  Pose issuedAtPose_{};

  bool frozen_ = false;  // within arrival tolerance; no further re-solving

  uint32_t replaceCount_ = 0;
  uint32_t tickCount_ = 0;
};

}  // namespace Motion
