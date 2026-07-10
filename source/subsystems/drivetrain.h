// drivetrain.h -- Subsystems::Drivetrain: the two-wheel differential
// Drivetrain MOTION PLANNER (sprint 094 rewrite -- ticket 094-004,
// clasi/sprints/094-drivetrain-becomes-the-motion-planner-segment-executor-
// move-command/architecture-update.md Section 3, "Subsystems::Drivetrain
// (rewrite) -- the motion planner").
//
// Before this ticket, Drivetrain was a thin faceplate: it held no motor
// reference, received the full per-port observation array as a tick()
// argument, and HELD its output (a Hal::DrivetrainToHardwareCommand) for
// Rt::MainLoop to drain via hasCommand()/takeCommand() (sprint 079's
// held-output design, later 087-003's driveIn queue). That whole
// hold-and-route chain is gone: Drivetrain now HOLDS `Hardware& hardware_`,
// resolves its own bound wheel pair through it, and STAGES its output
// directly via `hardware_.motor(port).apply(cmd)` -- there is nothing left
// for the loop to route (main_loop.cpp's former routeOutputs() step is
// deleted, ticket 094-005).
//
// This ticket also folds in the sprint's motion-planning half: Drivetrain
// now owns one Motion::SegmentExecutor (source/motion/segment_executor.h --
// the lifted, near-verbatim Ruckig-based trajectory engine from
// Subsystems::Planner, ticket 094-001) plus an 8-slot Rt::WorkQueue<
// Motion::Segment, 8> ring. `MOVE`'s wire handler (094-006, not this
// ticket) posts parsed Motion::Segments to the BLACKBOARD's own
// `bb.segmentIn` queue (094-005); THIS class drains that queue into its
// own internal ring_ every tick() and executes the ring's head segment via
// executor_.
//
// --- The two command sources, and their precedence (architecture-update.md
//     Section 6, "Command precedence and the 'no hiccups' requirement") ---
//
// `driveIn` (Rt::WorkQueue<msg::DrivetrainCommand, 8>) is now the S/STOP
// ESCAPE-HATCH input to this Drivetrain ONLY -- there is no more Planner
// producer, no more routeOutputs() consumer (087-003's arbitration
// commentary describing a Planner producer is stale; see blackboard.h's own
// updated doc comment). `segmentIn` (Rt::WorkQueue<Motion::Segment, 8>) is
// `MOVE`'s fan-in. Every tick():
//   1. `driveIn` is drained ONE command per tick (FIFO pop, matching this
//      class's pre-094 drain cadence) and applied FIRST. A WHEELS or NEUTRAL
//      command PREEMPTS: it clears ring_ and switches this tick (and every
//      subsequent tick, until a fresh Motion::Segment reclaims it -- see
//      below) to DIRECT mode -- immediate, ungoverned-by-the-executor wheel
//      targets, exactly like today's `S`. TWIST is dispatched the same way
//      (kept for oneof-dispatch symmetry -- see commandedWheelTargets() --
//      though no live producer posts TWIST via driveIn this sprint).
//      NEUTRAL is special: if a segment is actively executing (SEGMENT mode
//      AND executor_.active()), NEUTRAL does NOT preempt to direct/instant-
//      zero -- it calls executor_.stop(now), which arms the executor's OWN
//      presolved graceful decel-to-zero (solveToVelocity(0, ...) from the
//      channel's current sampled state) and this Drivetrain stays in
//      SEGMENT mode, riding that decel down to a literal 0.0f twist over
//      subsequent ticks -- see segment_executor.h's own stop() doc comment.
//      Only when there is nothing in-flight to decelerate (DIRECT mode was
//      already active, or the executor was already idle) does NEUTRAL fall
//      through to an instant zero-velocity command, matching pre-094 `STOP`
//      behavior for the case that behavior applies to (a plain `S` then
//      `STOP`, with no segment ever in flight).
//   2. Otherwise (driveIn did not just preempt this tick) `segmentIn` is
//      drained IN FULL into ring_ (a WorkQueue post() that returns false --
//      ring_ already at its 8-slot cap -- silently drops the excess; this
//      should not occur in ordinary operation since segmentIn itself caps
//      at the same depth and is drained every tick). Queuing at least one
//      fresh segment switches (or keeps) this Drivetrain in SEGMENT mode --
//      a `MOVE` reclaims control from a stale DIRECT-mode `S`, deliberately
//      (a wire MOVE is an explicit "please plan this" request; it should
//      not be silently ignored just because an earlier `S` happened to be
//      the last escape-hatch command).
//
// In SEGMENT mode, once executor_ is idle() and ring_ is non-empty, the
// ring's head segment is popped and handed to executor_.start() -- "pop on
// completion, start next." The executor's per-tick body twist
// (msg::BodyTwist3, v_x for TRANSLATE / omega for PRE_PIVOT/TERMINAL_PIVOT,
// pose-free -- segment_executor.h) is converted to wheel targets via the
// SAME BodyKinematics::inverse() the TWIST arm always used, then governed by
// the SAME governRatio() (below, UNCHANGED math) before being staged.
//
// A freshly constructed Drivetrain starts in DIRECT mode with mode_ ==
// NEUTRAL (matches the pre-094 idle default: an explicit NEUTRAL command
// staged every tick until a real command arrives, not a spurious
// VELOCITY(0) -- see tick()'s own body).
//
// No PID lives here -- that stays entirely inside NezhaMotor. The only
// control law this class runs is the ratio (sync) governor: see
// governRatio() below, whose math this ticket does not change.
//
// --- Staging-only write path (architecture-update.md Section 5) ---
// Drivetrain computes leftCmd/rightCmd (msg::MotorCommand) and STAGES them:
// `hardware_.motor(port).apply(cmd)`. Hal::Motor::apply() only calls the
// leaf's primitive setters (setVelocity()/setNeutral()/...), which are
// themselves staging-only (Hal::NezhaMotor::setVelocity()/setDutyCycle()
// only set mode_/velocityTarget_/dutyTarget_ -- confirmed by this ticket's
// own host unit test, tests/sim/unit/nezha_staging_only_harness.cpp). The
// actual I2C write happens only inside NezhaMotor::tick()'s mode dispatch,
// itself called only from NezhaHardware::tick()'s COLLECT_DUE case
// (unchanged this sprint -- harmonized against the bare `main()` loop: the
// 094-003 `serviceBus()` rename in the design note was DROPPED, Hardware
// keeps the name `tick()`). So a setpoint staged THIS pass is flushed at
// Hardware::tick()'s own cadence -- the bare loop runs `hardware.tick(now)`
// BEFORE `drivetrain.tick(now, ...)` every pass (source/main.cpp,
// tests/_infra/sim/sim_api.cpp), so a setpoint staged this pass is flushed
// the FOLLOWING pass: identical one-pass latency to the pre-094
// `routeOutputs() -> bb.motorIn[] -> next-pass drain` chain. Timing is
// untouched.
//
// `hasCommand()`/`takeCommand()`/the held `Hal::DrivetrainToHardwareCommand`
// output are DELETED -- there is nothing left to route.
#pragma once

#include <stdint.h>

#include "messages/drivetrain.h"
#include "messages/motor.h"
#include "messages/planner.h"
#include "motion/segment.h"
#include "motion/segment_executor.h"
#include "runtime/queue.h"
#include "subsystems/hardware.h"

namespace Subsystems {

// The Drivetrain's bound wheel-motor pair, as 0-based Hardware motor
// indices -- read via ports() (`DEV DT PORTS` -> DrivetrainConfig.
// left_port/right_port, per sprint 079 decision 8; unchanged this ticket).
// (0-based motor indices, OOP refactor) msg::DrivetrainConfig.left_port/
// right_port are wire/serialized keys and stay 1-based (the brick label);
// configure() converts them to 0-based indices EXACTLY ONCE, the moment a
// DrivetrainConfig arrives -- see configure()'s own doc comment, the single
// conversion point for this Drivetrain. ports() returns those already-
// converted indices; every other member uses them directly, with no
// further `- 1`/`+ 1` anywhere in this class.
struct DrivetrainPorts {
  uint32_t left;   // 0-based Hardware motor index
  uint32_t right;  // 0-based Hardware motor index
};

class Drivetrain {
 public:
  // Stores hardware BY REFERENCE (never copied) -- the container this
  // Drivetrain resolves its bound wheel pair through every tick() (see the
  // class comment). Declaration-order note for both composition roots
  // (main.cpp / tests/_infra/sim/sim_api.cpp, ticket 094-005): `hardware`
  // must be constructed before `drivetrain`.
  explicit Drivetrain(Hardware& hardware);

  // --- Primitive setters -- the DIRECT (escape-hatch) arms' real
  // implementation. Unchanged shape from before this ticket (do not change
  // the TWIST/WHEELS/NEUTRAL dispatch shape) -- only how they are reached
  // (via tick()'s driveIn precedence, below) and what they compete with
  // (the segment ring) changed. ---

  // A twist is a directed body-frame velocity: v_x, v_y, omega (matches
  // msg::BodyTwist3). v_y is always ignored -- this Drivetrain is
  // differential-only (capabilities().holonomic is always false).
  void setTwist(float v_x, float v_y, float omega);   // [mm/s] [mm/s] [rad/s]

  // Direct per-wheel velocity targets -- bypasses kinematics AND the
  // segment executor entirely (the coupled bench rig's curve tests use
  // this on ports 3+4, same as before this ticket). Still passes through
  // the ratio governor.
  void setWheelTargets(float left, float right);      // [mm/s] signed wheel velocities

  void setNeutral(msg::Neutral mode);

  // --- Faceplate verbs. ---

  // configure() -- THE single conversion point (0-based motor indices, OOP
  // refactor) where this Drivetrain's bound wheel pair, carried on the wire
  // as msg::DrivetrainConfig.left_port/right_port (1-based brick labels,
  // unchanged -- a wire/serialized key), is converted to 0-based Hardware
  // motor indices EXACTLY ONCE: `boundLeft_ = config.left_port - 1;`
  // `boundRight_ = config.right_port - 1;`. Every other member of this
  // class (tick()/state()/ports()) uses boundLeft_/boundRight_ directly --
  // no further port math anywhere else in this Drivetrain.
  void configure(const msg::DrivetrainConfig& config);

  // configureMotion -- boot-only motion-limit defaults for the owned
  // Motion::SegmentExecutor (msg::PlannerConfig, reused as-is --
  // architecture-update.md Section 8's jerk-config knob). Forwards to
  // executor_.configure(). Per-segment MOVE overrides (094-006's
  // speedMax/accelMax/jerkMax/yawRateMax/yawAccelMax/yawJerkMax fields,
  // segment.h) take precedence per-solve when nonzero; this is only the
  // fallback a 0 field resolves to. No runtime SET/GET path calls this --
  // both composition roots (main.cpp / sim_api.cpp) call it exactly once,
  // at construction (094-005).
  void configureMotion(const msg::PlannerConfig& config);

  // Unpacks the oneof -> the setters above, THEN dispatches the standby
  // side-channel (see the class comment's "Authority arbitration" heritage,
  // unchanged this ticket). Called from tick() below, AFTER the
  // ring/segmentMode side effects tick() itself decides based on the same
  // command's control_kind (see tick()'s own doc comment) -- this method
  // has no ring/segmentMode side effects of its own; it is the shared
  // "unpack the oneof into mode_/targets" step both the escape hatch and
  // (indirectly, informationally) the graceful-stop path route through.
  void apply(const msg::DrivetrainCommand& command);

  // tick -- the mandatory per-pass control step (run AFTER hardware.tick()
  // -- 094-005). now: [ms]. segmentIn: the blackboard's `MOVE`-command
  // fan-in (drained into ring_ -- see the class comment). driveIn: the
  // S/STOP escape-hatch fan-in (drained FIRST, one command per tick, per
  // the class comment's precedence rules). Resolves this Drivetrain's OWN
  // bound wheel pair via `hardware_.motorState(i)`/`hardware_.motor(i)`
  // internally, using boundLeft_/boundRight_ (already-converted 0-based
  // indices -- see configure()'s own doc comment, the ONE place a `- 1`
  // exists in this class) -- Hardware::state()/motor() take a 0-based index
  // and do their own out-of-range clamping (each concrete owner's own doc
  // comment). The range assert below is kept as a defensive guard against a
  // misconfigured (out-of-range) bound index.
  void tick(uint32_t now,
            Rt::WorkQueue<Motion::Segment, 8>& segmentIn,
            Rt::WorkQueue<msg::DrivetrainCommand, 8>& driveIn);

  // Assembled from getters. enc_[]/vel_[] are sourced from
  // hardware_.motorState(port) -- MEASURED, not commanded (replaces the pre-094
  // "reports the pre-governor commanded target" behavior entirely: this is
  // a genuinely different, measured source, not a preserved one -- see
  // architecture-update.md Section 3's Drivetrain boundary and ticket
  // 094-004's own acceptance criteria).
  msg::DrivetrainState state() const;
  msg::DrivetrainCapabilities capabilities() const;

  // Records the two bound wheel motors' capabilities, needed only so
  // capabilities().onboard_position can report accurately without this
  // class holding a second, redundant Hal::Motor reference of its own
  // (hardware_ already gives it one). Unchanged from before this ticket.
  void setMotorCapabilities(const msg::MotorCapabilities& left,
                             const msg::MotorCapabilities& right);

  // --- Port binding + authority arbitration (unchanged from before this
  // ticket -- sprint 079's design, not touched by 094). ---

  DrivetrainPorts ports() const;
  bool active() const;
  void standby();

 private:
  // The DIRECT (escape-hatch) arm currently staged -- consulted only while
  // NOT in SEGMENT mode (see tick()'s dispatch). Shape unchanged from
  // before this ticket.
  enum class Mode : uint8_t { NEUTRAL, TWIST, WHEELS };

  // dispatchEscapeHatch -- tick()'s own helper: decides the ring-clearing/
  // segmentMode_/executor_.stop() side effects a driveIn command causes
  // (see the class comment's precedence rules) BEFORE calling apply() to
  // unpack the oneof into mode_/targets. Returns true if this command
  // PREEMPTS segmentIn's drain for this tick (WHEELS/TWIST/NEUTRAL -- an
  // actual arm was dispatched), false for POSE/NONE (no arm dispatched,
  // matching apply()'s own "no action" default case -- nothing to preempt
  // with).
  bool dispatchEscapeHatch(const msg::DrivetrainCommand& command, uint32_t now);

  // Drains ring_ completely (repeated take()s -- Rt::WorkQueue has no
  // clear()). Used when an escape-hatch command preempts an in-flight (or
  // merely queued) segment plan.
  void clearRing();

  // Computes this Drivetrain's currently-commanded DIRECT-mode wheel
  // velocity targets, BEFORE the ratio governor -- kinematics for TWIST, a
  // direct pass-through for WHEELS, zero for NEUTRAL. Unchanged shape from
  // before this ticket. Only consulted from tick() while NOT in SEGMENT
  // mode.
  void commandedWheelTargets(float* targetLeft, float* targetRight) const;

  // Ratio (sync) governor: if a wheel underachieves its target (bogged
  // down), lower the shared speed ceiling so left/right hold their
  // commanded ratio (curvature), instead of letting the healthy wheel run
  // away. UNCHANGED MATH this ticket -- applied uniformly to DIRECT-mode
  // targets (TWIST/WHEELS) and to SEGMENT-mode targets (the executor's body
  // twist, converted via BodyKinematics::inverse()) alike; only a literal
  // NEUTRAL instant-zero bypasses it, exactly as before this ticket.
  // DrivetrainConfig.sync_gain is the tuning knob (0 = fully independent).
  void governRatio(float* targetLeft, float* targetRight,
                    const msg::MotorState& leftObs,
                    const msg::MotorState& rightObs) const;

  Hardware& hardware_;

  // The lifted Ruckig-based trajectory engine (094-001) this Drivetrain now
  // owns. Pose-free, encoder-only -- see segment_executor.h.
  Motion::SegmentExecutor executor_;

  // The 8-slot segment ring "owned by the Drivetrain" the originating
  // issue calls for -- matches bb.segmentIn's own depth (094-005) so a
  // full blackboard drain never has to be rejected by this ring under
  // ordinary operation.
  Rt::WorkQueue<Motion::Segment, 8> ring_;

  // true: ring_/executor_ drives this tick's staged output (SEGMENT mode).
  // false: mode_/commandedWheelTargets() drives it (DIRECT/escape-hatch
  // mode). Starts false (DIRECT, mode_ == NEUTRAL) -- see the class
  // comment's "freshly constructed" note. Set true whenever segmentIn
  // drains at least one fresh segment into ring_ (a MOVE reclaims control);
  // set false by a WHEELS/TWIST escape-hatch command, or by a NEUTRAL that
  // finds nothing in-flight to gracefully decelerate. A NEUTRAL that DOES
  // find an in-flight segment leaves this true (see dispatchEscapeHatch()).
  bool segmentMode_ = false;

  msg::DrivetrainConfig config_ = {};
  Mode mode_ = Mode::NEUTRAL;

  // The bound wheel pair, as 0-based Hardware motor indices -- converted
  // from config_.left_port/right_port (1-based wire labels) EXACTLY ONCE,
  // in configure() (this class's single conversion point). ports() returns
  // these verbatim; tick()/state() resolve hardware_.motor(i)/state(i)
  // through these, never through config_.left_port/right_port directly.
  uint32_t boundLeft_ = 0;
  uint32_t boundRight_ = 1;

  // DIRECT/TWIST-arm state.
  float v_x_ = 0.0f;      // [mm/s]
  float v_y_ = 0.0f;      // [mm/s] always ignored -- see setTwist()
  float omega_ = 0.0f;    // [rad/s]

  // DIRECT/WHEELS-arm state.
  float wheelTargetLeft_ = 0.0f;    // [mm/s]
  float wheelTargetRight_ = 0.0f;   // [mm/s]

  // Last pass's post-governor commanded wheel velocities, surfaced via
  // state()/TLM `cmd=` (measured vel= vs the setpoint the velocity PID
  // chases). Written by tick(); read by the const state().
  float cmdVel_[2] = {0.0f, 0.0f};   // [mm/s]

  // Measured per-wheel acceleration, EMA-filtered in firmware (see
  // updateAccelEma()'s doc comment in drivetrain.cpp) -- surfaced via
  // state()/TLM `acc=`. Indexed [0]=bound left wheel, [1]=bound right.
  void updateAccelEma(uint32_t now, int wheel, const msg::MotorState& obs);
  float accelEma_[2] = {0.0f, 0.0f};        // [mm/s^2]
  float lastVelSample_[2] = {0.0f, 0.0f};   // [mm/s] last DISTINCT velocity sample
  uint32_t lastVelSampleMs_[2] = {0, 0};    // [ms]
  bool haveVelSample_[2] = {false, false};

  msg::Neutral neutralMode_ = msg::Neutral::BRAKE;

  msg::MotorCapabilities leftMotorCaps_ = {};
  msg::MotorCapabilities rightMotorCaps_ = {};

  // Authority state (sprint 079, unchanged shape). No live producer posts
  // an authority-steal (`standby=true` alone) this sprint -- kept for API
  // symmetry/a future revival, exactly as it already was pre-094.
  bool active_ = false;
};

}  // namespace Subsystems
