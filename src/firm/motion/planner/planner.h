// planner.h -- Motion::Planner: the standalone motion planner (design of
// record: docs/design/motion-planner-sketch.md, stakeholder-reviewed
// 2026-07-25). Owns everything between "a Move arrived" and "here are the
// wheel velocity targets for the next control interval": the move queue and
// lookahead, the discrete-exact trapezoid profiler (profile.h), and the
// sense-side estimation (estimation.h).
//
// The two-method contract, enforced by constness in both directions:
//   tick(const Types::RobotState&)  -- DO THE WORK; provably cannot touch the
//                               blackboard. Returns the completion event.
//   update(Types::RobotState&) const -- SAVE tick()'s results into the state;
//                               provably computes nothing new. The ONLY
//                               Planner method that mutates Types::RobotState.
// Moves are handed in as they arrive via move(); activation happens on the
// next tick() (which is where current time/pose baselines live).
//
// tick()'s own Move lifecycle (130-008, planner-honesty-pass-50ms-period-
// tick-state-machine-limits-reduction.md item 2) is an explicit state
// machine, Motion::MoveLifecycle (planner_types.h): Idle -> Breakaway ->
// Tracking -> (Aligning) -> (Idle|Draining), with Stopping as the parallel
// path a Kind::Stop entry takes, and Draining as the transient path back to
// Idle once the queue runs dry. Replaces the previous implicit encoding as
// interacting `occupied`/`hasMoved`/`settling` booleans. MovePhase
// (Accel/Hold/Decel) is `Tracking`'s own sub-phase, not a sibling state.
// Full transition table, event definitions, and completion-priority order:
// Planner::tick()'s own doc comment in planner.cpp.
//
// Aligning (134-003) is the one state a Move can be in AFTER its profile
// has landed and BEFORE it is acked: a Twist Angle Move holds itself open
// there while it trims its residual against the cumulative-intent ledger
// with bounded low-speed pivot nudges, re-settling between each. "A Move
// is not done until it has landed" -- so `TickResult::completed` fires on
// the transition OUT of Aligning, never into it. The Move's own wall-clock
// `timeout` keeps running throughout and still outranks the trim, so a
// non-converging alignment cannot wedge the queue.
#pragma once

#include <cstdint>

#include "estimation.h"
#include "planner_types.h"
#include "profile.h"
#include "shape.h"
#include "types/robot_state.h"

namespace Motion {

class Planner {
 public:
  // 1 active + 4 pending, protocol v4's 5-deep queue.
  static constexpr int kQueueDepth = 5;

  explicit Planner(const PlannerLimits& limits);

  // Queue a Move. replace=true drops everything (active included) and the
  // new Move activates on the next tick(). Returns false when the queue is
  // full (caller acks ERR_FULL; queue provably unchanged).
  bool move(const Move& next, bool replace);

  // Enqueue a PLANNED stop (command-ingestion-...-two-stops.md §5, the
  // wire's `STOP` verb): an ordinary queue entry (Move::Kind::Stop) that
  // executes IN SEQUENCE behind whatever is already queued. On activation
  // it ramps the staged command down to zero at the decel ceiling and
  // completes once the body is at rest, acking `moveId` through the usual
  // TickResult path -- so a host can say "finish the two legs you already
  // have, then stop" and learn when that stop actually happened.
  //
  // NOT the panic stop: that is estop() below. Returns false when the queue
  // is full, exactly like move().
  bool plannedStop(uint32_t moveId);

  // Panic stop (§5): clear the active Move AND every pending entry
  // immediately and command zero. The discarded entries get NO completion
  // acks -- the host asked for a halt, not for a report that the things it
  // cancelled finished. Replaces the previous stop(), which had this
  // behavior under a name that now means the planned stop above.
  void estop();

  // See the file header. `state.time.cycleStart` is the clock.
  TickResult tick(const Types::RobotState& state);
  void update(Types::RobotState& state) const;

  // Observability (tests / telemetry derivation).
  bool active() const { return active_.occupied; }
  int pendingCount() const { return pendingCount_; }
  uint32_t activeMoveId() const { return active_.move.id; }  // valid iff active()
  float commandedLeft() const { return cmdLeft_; }    // [mm/s]
  float commandedRight() const { return cmdRight_; }  // [mm/s]

  // Which regime the active Move is in -- 130-005: no longer gates any
  // trim integrator here (Motion::WheelTrim/stageTrim() are deleted; the
  // wheel-speed controller's own fast-PID steady gate now lives in
  // Core::DifferentialDrive::fastPid(), keyed off cmdAccel, not MovePhase -- see
  // drive.h's own header) -- kept for what the bench charts still shade
  // behind the velocity traces and for Move-lifecycle observability.
  MovePhase phase() const {
    return active_.occupied ? active_.phase : MovePhase::Idle;
  }

  // The top-level Move lifecycle state (130-008) -- see this file's own
  // header comment and Planner::tick()'s doc comment in planner.cpp for
  // the full transition table. Observability (tests / telemetry
  // derivation), same footing as phase() above.
  MoveLifecycle lifecycle() const { return lifecycle_; }

  // Live-tuning entry points (the CONFIG wire arm / persisted tuning):
  // plain in-memory updates, never persisted here.
  //
  // 130-007: applyVelGains() (the M4 duty stage's own gains setter) is
  // deleted with the stage -- the `pid.*` CONFIG wire keys it used to
  // (silently) target were already repointed onto Core::DifferentialDrive's unified
  // wheel-speed controller by ticket 005
  // (wheel-speed-controller-moves-into-drive.md Phase 3); this deletion
  // just removes the dead destination those keys no longer reach.
  void applyShaperLimits(float aMax, float aDecel, float alphaMax,
                         float alphaDecel, float jerkMax, float yawJerkMax);
  const PlannerLimits& limits() const { return limits_; }

  // No Config::Robot-consuming configure() on THIS class (132-007, the-
  // configuration-object.md, sprint 132 "configuration discipline"):
  // src/firm/motion's own dependency rule (src/firm/motion/DESIGN.md §3) forbids
  // ANY Config::*/Core::*/Devices::* dependency in this tree, no
  // exception -- stricter than the (already strict) devices isolation
  // invariant this project also has. Core::configurePlanner(Motion::
  // Planner&, const Config::Robot&) (src/firm/app/boot_calibration.h)
  // is the Config::Robot-consuming entry point instead, calling
  // applyShaperLimits() above.

  // True once applyShaperLimits() has ever landed real profile ceilings
  // (boot config or a wire EstimatorConfigPatch). The loop publishes
  // kFlagFaultShapingDisabled -- the loud off-state for the silent-off
  // shaping config boundary (119-001) -- from this while a Move is active.
  bool shaperConfigured() const { return shaperConfigured_; }

 private:
  struct ActiveMove {
    bool occupied = false;
    Move move{};
    uint32_t activationTime = 0;   // [ms]
    float baselinePath = 0.0f;     // [mm] signed mean wheel position at activation
    float baselineHeading = 0.0f;  // [rad] heading() at activation
    bool closingIssued = false;    // last command was the exact terminal step

    // Stall backstop. Completion tests `plannedRemaining <= epsilon` on a
    // SIGNED residual, so a Move that OVERSHOOTS completes at once (the
    // residual goes negative) but one that lands SHORT by more than the
    // epsilon never completes on its own: with the wheels stopped the
    // in-flight prediction that would carry the residual negative is also
    // zero, so it is pinned and the Move runs out its full MOVE_TIMEOUT
    // (measured 2026-07-28: turns idling 27+ s with both wheels at rest and
    // the Move still flagged active). Widening the epsilon only moves the
    // cliff -- a weaker plant lands further short and hangs again, which is
    // exactly what happened after the 1 um -> 1 mm widening.
    //
    // So arrival is not the only way to be finished: a body that has come
    // to REST and is making no further progress is done, wherever it
    // stopped. These track that -- `stallRemaining` is the anchored
    // residual when the stall window opened, `stallTicks` how many
    // consecutive ticks it has held while at rest.
    float stallRemaining = 0.0f;   // [mm] or [rad], the Move's own axis
    uint32_t stallTicks = 0;
    // Gate: the backstop only arms once the body has actually left rest.
    // Without this, every Move is "at rest and making no progress" on its
    // own activation tick, and one that is merely slow to break away (the
    // accel ramp plus ~120-140 ms of actuation lag) would be completed
    // instead of driven -- skipping the leg outright, which is far worse
    // than the hang being fixed. "Moved, then stopped, and stayed stopped"
    // is the condition; "has not started yet" is not.
    //
    // 130-008: this used to be its own boolean (`hasMoved`), flipped once
    // and read directly. It is now the Planner-level `lifecycle_`'s own
    // Breakaway->Tracking transition (see tick()'s doc comment) --
    // `lifecycle_ != MoveLifecycle::Breakaway` is the exact replacement
    // test, and the settle-confirm `settling`/`settleStart` pair this used
    // to sit beside is deleted outright (dead: PlannerLimits::
    // requireSettle no longer drives anything -- see its own doc comment).

    // Per-wheel plan, captured once at activation (shape.h). The profiler
    // works in SHAPE space: one scalar lambda -- the dominant wheel's own
    // speed -- times the constant unit ratio.
    MoveShape shape{};
    AxisLimits wheelLimits{};  // shape-space ceilings, [mm/s] and [mm/s^2]
    // How much of the Move's OWN axis quantity (body path [mm] for a
    // Distance Move, heading [rad] for an Angle Move) advances per unit of
    // lambda. Everything outside planActive() -- the completion tests,
    // activeBoundary_, profileVelocity_ -- stays in axis units; this
    // factor is the only bridge, applied on the way in and
    // undone on the way out. A straight has axisPerLambda == 1 (lambda IS
    // the body speed) and a pivot has 2/trackWidth (lambda is the wheel
    // speed, omega = lambda/halfTrack), which is exactly the half-track
    // scaling the old per-Kind Angle case applied after profiling.
    float axisPerLambda = 1.0f;
    MovePhase phase = MovePhase::Idle;
    // Braking never un-begins WHILE it remains genuine: "a move accelerates
    // at the start and decelerates at the end, never the reverse" -- but
    // 131-006 makes this a conditional latch, not a pure one-way ratchet.
    // Set on a Decel/Closing step (planWheels()'s own doc comment has the
    // full mechanism); cleared -- not merely overridden for one tick, but
    // released so the fresh classification stands -- the instant a LATER
    // tick's own from-scratch profileStep() recomputation says Accel or
    // Hold, i.e. re-measurement shows the Move was never truly braking
    // (a transient plannedRemaining under-estimate tripped it, not a real
    // decision to land). See planWheels()'s own doc comment at this field's
    // read site for the full rationale and profile.cpp's "let
    // re-measurement recover" comment this now honors at the caller level
    // too.
    bool decelLatched = false;
  };

  enum class Axis : uint8_t { None, Linear, Angular, Wheels };
  static Axis axisOf(const Move& m);

  // The measured state the completion tests and the settle gate read,
  // computed once per tick from the freshly integrated estimate.
  struct Measurement {
    float bodyVelocity = 0.0f;  // [mm/s] filtered, signed
    float omega = 0.0f;         // [rad/s] filtered, signed
    // Distance-to-go on the active Move's own axis, in its own units
    // ([mm] Distance, [rad] Angle).
    //   plannedRemaining  -- extrapolated forward by the ZOH predict and
    //       the actuation delay: the residual at the instant this tick's
    //       command takes EFFECT, which is what the profiler must plan
    //       against and what profile-completion tests.
    //   anchoredRemaining -- the residual against the last MEASURED wheel
    //       positions, extrapolated by nothing. Noise-free by construction
    //       (it only moves when a real sample lands), which is what "have
    //       we actually ARRIVED?" has to ask.
    float plannedRemaining = 0.0f;
    float anchoredRemaining = 0.0f;
  };
  Measurement measure(uint32_t now) const;  // [ms]

  // Has the active Move physically arrived and come to rest? (See
  // PlannerLimits::requireSettle.) Evaluated on every completion so
  // TickResult::settled is truthful even with settle-confirm off.
  bool settleReached(const Measurement& measured, float dt) const;

  // Pop pending_[0] into active_, capturing baselines from the current
  // pose -- or from the completed predecessor's cumulative boundary
  // (baseline + threshold) on a same-measure chain, so sub-tick boundary
  // residual is debited to the successor and a chain leaks zero total
  // error. Resets the profile carry velocity unless the new Move
  // continues the same axis (same-axis carry, sketch §3 lookahead).
  void activateNext(uint32_t now);

  // The boundary speed to land the ACTIVE Move at, in SHAPE space (lambda
  // -- the dominant wheel's own speed): 0, or the entry speed the next
  // pending Move can accept. Compatibility is a ratio identity, so a
  // reversal, an axis change, or a planned stop all answer 0.
  float boundaryLambda(float dt) const;  // [s] -> [mm/s]

  // Compute the active Move's command for the next interval; stages
  // cmdLeft_/cmdRight_ and updates the profile carry state.
  void planActive(uint32_t now, float dt, const Measurement& measured);

  // The per-wheel profiler behind planActive()'s Distance/Angle cases:
  // profiles each wheel against its own remaining distance, then applies
  // the SAME feasible fraction to both (the ratio lock) so the commanded
  // left:right ratio -- and therefore the heading -- cannot drift. See the
  // long comment at its definition for why this keeps the discrete-exact
  // landing guarantee.
  void planWheels(float dt, const Measurement& measured);  // [s]

  // Add the heading-hold differential on top of an already-staged pair of
  // wheel commands (Distance Moves only; no-op at zero gain). Clamped so
  // the faster wheel never exceeds vMax, and differential by construction
  // so the profiled path length is unchanged.
  void applyHeadingHold();

  // No active Move: ramp the staged command down to zero within limits.
  void drainToZero(float dt);

  // Ramp the staged pair toward zero by one decel step, preserving the
  // commanded left:right ratio (see the definition). Shared by the drain
  // and the planned-stop entry.
  void rampCommandsToZero(float decelStep);  // [mm/s] per interval

  // The decel step the drain ramps at -- the active (or just-completed)
  // shape's own ceiling, so the profile's terminal step always fits inside
  // exactly one drain step.
  float drainDecel(float dt) const;  // [s] -> [mm/s] per interval

  // Bounded backstop for a Kind::Stop entry: the longest it may take to
  // reach rest before completing anyway. Derived, not a magic number --
  // the worst-case decel ramp from vMax plus one settle allowance -- so a
  // planned stop can never wedge the queue on a plant whose measured
  // velocity never quiets below the rest floors.
  float plannedStopWindow() const;  // [ms]

  // Age the staged command by one tick (see cmdLeftPrevious_).
  void rollCommandHistory();

  // ---- Terminal fine-align (134-003, MoveLifecycle::Aligning) -----------
  //
  // The per-corner heading trim, in firmware. Proven on `tovez` by the
  // host-side graft this reproduces: planner square-tour closure 25.8 ->
  // 9.4 mm at a 1.0 deg tolerance, ~2 s/corner, nothing else changed
  // (docs/bench-reports/motion-planning-lab-2026-08-04.md §5.2). Depends
  // on the cumulative-intent ledger (134-001) for an honest target and on
  // the speed floor no longer boosting planner-shaped commands (134-002)
  // for a deliverable nudge.

  // May the just-completed Move hold itself open to trim? Twist Angle
  // Moves only, completed normally (never on timeout or stall), never one
  // handing off at speed, and only with both config fields set.
  bool alignApplies(const Move& m, bool timedOut, bool stalled) const;

  // Latch the ledger target and open the nudge budget. Called ONCE, on the
  // tick the Move would otherwise have completed.
  void enterAligning(uint32_t now);  // [ms]

  // One Aligning tick. Returns true when the alignment is finished and the
  // Move may complete; false while it continues -- in which case this call
  // has already staged the tick's own command, exactly one staging call per
  // tick like every other path through tick().
  bool alignStep(uint32_t now, float dt, const Measurement& measured);

  // Re-arm active_ as a bounded low-speed corrective pivot of `residual`,
  // so the nudge reaches the wheels through the NORMAL planActive()/
  // planWheels() path rather than a private bypass.
  void startAlignNudge(uint32_t now, float residual);  // [ms] [rad]

  // The settling half of an Aligning tick: ramp the staged command out at
  // the shape's own decel ceiling and hold the boundary at zero.
  void alignSettleCommand(float dt);  // [s]

  // The corrective pivot's angular cruise -- a bounded fraction of the
  // configured angular ceiling (see the definition for the sizing).
  float alignNudgeOmega() const;  // [rad/s]

  // The bounded window one corrective pivot may run for before the phase
  // gives up on it and re-measures. Derived from the pivot itself; needed
  // because a measured 26% of nudges never break away at all.
  float alignNudgeWindow(float threshold, float omega) const;  // [rad] [rad/s] -> [ms]

  // The bounded window one settle phase may wait for rest before giving up
  // and completing the Move where it stands -- the same "a noisy plant
  // cannot wedge the queue here" backstop plannedStopWindow() provides for
  // a Kind::Stop, and the reason a Move with no `timeout` at all still
  // cannot hang in Aligning.
  float alignSettleWindow() const;  // [ms]

  PlannerLimits limits_;
  WheelChannel left_, right_;
  PoseTracker pose_;

  Move pending_[kQueueDepth]{};
  int pendingCount_ = 0;
  ActiveMove active_{};

  // The top-level Move lifecycle (130-008) -- see this file's own header
  // comment and Planner::tick()'s doc comment (planner.cpp) for the full
  // transition table. Reset on every activateNext() and on every tick
  // that leaves the planner with no active Move.
  MoveLifecycle lifecycle_ = MoveLifecycle::Idle;

  // Positive-frame previous command on the profiled axis (the carry the
  // next tick ramps from); per-wheel carries for Wheels Moves and drain.
  float profileVelocity_ = 0.0f;
  float profileAccel_ = 0.0f;  // [mm/s^2 or rad/s^2] last step's implied accel (jerk carry)
  float cmdLeft_ = 0.0f;   // [mm/s] staged last tick
  float cmdRight_ = 0.0f;  // [mm/s] staged last tick
  // Staged the tick before last -- the command actually driving the wheels
  // over the just-elapsed interval when there is one cycle of actuation
  // latency. measure()'s anticipation needs it; nothing else does.
  float cmdLeftPrevious_ = 0.0f;   // [mm/s]
  float cmdRightPrevious_ = 0.0f;  // [mm/s]
  Axis lastAxis_ = Axis::None;
  float activeBoundary_ = 0.0f;  // last planned boundary velocity, positive frame
  // The shape-space decel ceiling of the most recently active shaped Move,
  // kept so the post-completion drain (which runs with no active Move)
  // ramps at the same ceiling the profile landed under.
  float lastShapeDecel_ = 0.0f;  // [mm/s^2]

  // Terminal fine-align working state (134-003). Lives BESIDE active_, not
  // inside it, for one specific reason: a corrective nudge deliberately
  // REWRITES active_.move (threshold/omega/v_x) and active_.baselineHeading
  // -- that rewrite is what lets the nudge flow through the ordinary
  // planWheels() path instead of a bypass -- so the ledger target this
  // phase is aiming at has to be latched somewhere that rewrite cannot
  // reach.
  struct AlignState {
    float target = 0.0f;       // [rad] cumulative-intent heading to land on
    uint32_t nudgeStart = 0;   // [ms] when the current nudge's window opened
    uint32_t settleStart = 0;  // [ms] when the current settle phase opened
    float nudgeWindow = 0.0f;  // [ms] how long that nudge may run
    // |residual| the last nudge was fired at, so the next measurement can
    // tell "this nudge helped / did nothing / made it worse" apart.
    float residualBefore = -1.0f;  // [rad] magnitude; <0 = no nudge yet
    int32_t nudges = 0;        // corrective pivots spent so far
    uint32_t restTicks = 0;    // consecutive command-quiet ticks
    bool nudging = false;      // driving a nudge (else settling to measure)
  };
  AlignState align_{};

  // Cumulative-baseline carry staged between a completion and the next
  // activation (see activateNext()).
  bool carryValid_ = false;
  float carryPath_ = 0.0f;     // [mm]
  float carryHeading_ = 0.0f;  // [rad]

  // Idle-gap disturbance reference (134-006). The carry above used to be
  // dropped whenever the queue happened to be empty at a hand-off, which
  // used queue occupancy as a proxy for "nothing disturbed the robot" --
  // false for every caller that waits for a completion ack before
  // enqueuing the next Move. These two replace that proxy with the real
  // condition: the heading the planner was left at when it went idle, and
  // whether there is one to compare against. See activateNext().
  float idleHeading_ = 0.0f;   // [rad] pose heading when the queue went empty
  bool idleLatched_ = false;   // idleHeading_ holds an idle-transition reading

  bool ticked_ = false;  // tick() has run at least once
  bool shaperConfigured_ = false;  // applyShaperLimits() ever landed
};

}  // namespace Motion
