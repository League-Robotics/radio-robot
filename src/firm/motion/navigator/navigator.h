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
// --- OTOS sign convention (ticket 008's settled fix) --------------------
//
// state.otos.heading is the WIRE-level value, the hardware-mounted sign
// (ticket 008's Completion Notes: "the settled sign convention") -- the
// NEGATION of encoder-derived heading. Motion::Planner's own internal
// pose_ (estimation.h's PoseTracker) reconciles this at exactly one point,
// planner.cpp:513 (`pose_.applyOtosHeading(-state.otos.heading, ...)`),
// landing in the encoder-sign convention every other angular decision in
// this codebase (Move::omega, ArcSolution::omega, Pose::heading) already
// uses. Planner exposes NO pose accessor Navigator could read instead
// (planner.h's public surface has none), so Navigator independently
// negates the SAME raw wire value, the SAME way, in the ONE place this
// file builds its own working Pose from state.otos -- see navigator.cpp's
// tick(). This is not a second, competing sign flip; it is the same
// reconciliation ticket 008 already settled, applied a second time
// because there are structurally two independent readers of the one raw
// wire fact (Planner's pose_, and this file), not because the convention
// itself is in question.
//
// This reconciled `pose` (encoder-sign convention) is the right frame for
// exactly two things this file does: the arrival/frozen distance check
// (rotation-invariant -- either convention gives the same distance) and
// the SUC-005 dead-reckoning fallback (`lastGoodPose_`/`lastGoodWheelPose_`
// difference against `state.pose`, which IS encoder-sign convention, by
// construction -- Odometry's own accumulated heading). It is the WRONG
// frame for a WORLD-frame bearing-to-target computation -- see ticket
// 004's own "omega sign" comment on `NavigatorLimits::yawSign` below for
// why, and for
// `worldPose`, the second, separately-purposed pose this file builds for
// exactly that computation.
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
// Commanded Move::omega is opposite true-world CCW ON REAL HARDWARE --
// measured, and reconciled on the host TODAY by every bench script's own
// `YAW_SIGN = -1.0` (e.g. src/tests/bench/goto_otos.py:54, "commanded
// omega is opposite to world CCW (measured)"; matches
// src/firm/motion/planner/planner.cpp:509-512's own comment naming the RIGHT
// fix as "the body kinematics' omega sign", deliberately deferred --
// Option B, out of scope this sprint, ticket 008's own "Scope" section).
//
// This is a SEPARATE fact from ticket 008's OTOS-mount sign (the
// `pose`/`-state.otos.heading` reconciliation above): that one settles
// how the OTOS chip's own mounted orientation relates to encoder-derived
// heading; THIS one is about how the DRIVETRAIN's own commanded-omega
// convention relates to true-world CCW, independent of OTOS entirely (it
// is exactly as present on a robot with no OTOS at all -- goto_otos.py's
// sibling scripts measure it via camera, not OTOS). Both are real,
// both apply, and they compose by simple sequencing, not cancellation.
//
// `Motion::ArcSolver::solve()` (135-002) is pure geometry with no
// knowledge of either quirk: fed a pose, it returns an omega in THAT
// pose's own heading convention (arc_solver.h's own `ArcSolution::omega`
// doc: "CCW-positive, Pose::heading's own sign convention"). Whether that
// pose's heading should be RAW `state.otos.heading` (true-world) or the
// encoder-sign `pose` above depends entirely on whether THIS robot's
// drivetrain has the quirk -- a real, hardware-specific, physically-
// measured fact this file cannot derive or assume, and which no firmware
// simulator (IdealPlant/SimPlant alike) can expose either: both read
// `Move`-commanded wheel velocities directly as their own ground truth,
// so they are tautologically self-consistent with whichever convention
// Navigator uses, by construction, regardless of what a REAL robot's
// motors/encoders happen to do (verified directly: hardcoding this flip
// unconditionally reproduces testConvergesFromRestAndSettles's own
// failure against the ctest suite's IdealPlant -- there is no plant
// model that can validate this sign choice in sim, only a real bench
// pass, ticket 006, can).
//
// `NavigatorLimits::yawSign` (below) is therefore a CONFIGURABLE field,
// following configuration-discipline.md exactly the way a per-direction
// rotation-calibration gain already does elsewhere in this codebase: it
// defaults to `+1.0` (no flip), which makes `worldPose.heading` (below)
// equal `pose.heading` (encoder-sign) EXACTLY -- i.e. every existing
// ctest/sim scenario that does not set it (every one as of this ticket)
// is COMPLETELY UNCHANGED from ticket 003's original behavior, byte for
// byte. A robot whose bench pass (ticket 006) measures the quirk sets
// `yawSign = -1.0` in ITS OWN robot JSON `navigator` block (this ticket's
// own NavigatorLimits config group), matching goto_otos.py's measured
// `YAW_SIGN` for that same robot.
//
// `worldPose.heading = yawSign * pose.heading`: at `yawSign = +1` this is
// `pose.heading` (encoder-sign, ticket 003's original convention,
// unchanged). At `yawSign = -1` this is `-pose.heading` == raw, un-negated
// `state.otos.heading` (ticket 008 settled that this raw wire value
// tracks true-world CCW directly) -- exactly the pose goto_otos.py's own
// `solve_arc()` feeds its identical tangent-arc formula. `solve()`'s
// returned omega, and the pivot sub-machine's own bearing-sign omega, are
// therefore in THAT SAME convention `worldPose.heading` used -- multiplying
// by the SAME `yawSign`, at the ONE place each is assigned into a `Move`
// handed to `planner_.move()`, converts back to Move::omega's own wire
// convention (a no-op at `yawSign = +1`; goto_otos.py's own `YAW_SIGN`
// conversion at `yawSign = -1`). This is the "one constant, one comment,
// one flip" discipline ticket 008's own Completion Notes already applied
// to ITS sign fix (`kOtosHardwareMountSign`) -- change the ONE config
// value, not scattered call sites, if and when Option B (the deferred
// kinematics fix) ever lands and makes this field obsolete.
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
