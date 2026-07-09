// segment_executor.h -- Motion::SegmentExecutor: executes one Motion::Segment
// (segment.h) as a chain of up to three single-channel Ruckig phases --
// PRE_PIVOT -> TRANSLATE -> TERMINAL_PIVOT -- emitting a body twist each
// tick(). Ticket 094-001 (architecture-update.md Section 3/4, "the
// SegmentExecutor" / "Executor lift plan").
//
// THE LIFT: this class carries the non-GOTO internals of Subsystems::Planner
// (source/subsystems/planner.{h,cpp}) forward near-verbatim -- two
// Motion::JerkTrajectory channels (linear_/rotational_), Motion::
// MotionBaseline capture, the divergence-triggered replan
// (maybeReplanTranslate()/maybeReplanPivot(), Planner's own
// maybeReplanDistance()/maybeReplanRotational()), the SAME compile-split
// dead-time (kOutputHops/kDeadTime, planner.cpp:150-156, ported byte-for-byte
// including the `#ifdef HOST_BUILD` sim-40ms/firmware-80ms split), and the
// presolved graceful decel-to-zero (armTranslateStopDecel()/
// armPivotStopDecel(), Planner's own armDistanceStopDecel()/
// armRotationalStopDecel()/armVelocityStopDecel()) -- including the literal-
// 0.0f snap on rotational convergence (planner.cpp:964-966) and its
// documented rationale (defeats Hal::MotorVelocityPid's zero-deadband
// residual reverse-spin).
//
// THE ONE GENUINELY NEW PIECE: the 3-phase PRE_PIVOT -> TRANSLATE ->
// TERMINAL_PIVOT sequencer a Segment needs that Planner's own goal kinds
// never did (DISTANCE/TURN/ROTATION were each a single phase). A
// differential drive satisfies an independent finalHeading by pivoting at
// the END, not a coupled arc -- see architecture-update.md's phase table:
//
//   | Phase          | Fires when              | Channel / solve                    | Stop (encoder-only)          |
//   |----------------|--------------------------|-------------------------------------|-------------------------------|
//   | PRE_PIVOT      | skip if |direction| ~= 0 | rotational solveToRest(direction)   | STOP_ROTATION, arc = |direction|*trackwidth/2 |
//   | TRANSLATE      | skip if |distance| ~= 0  | linear solveToRest(distance)         | STOP_DISTANCE at |distance|   |
//   | TERMINAL_PIVOT | skip if finalHeading~=direction | rotational solveToRest(finalHeading-direction) | STOP_ROTATION, arc = |delta|*trackwidth/2 |
//
// Each phase is a FRESH Motion::MotionBaseline + a fresh single-channel
// Ruckig solve (Decision 2's "Pattern A" -- solve-to-rest-at-a-known-target,
// the ONLY pattern a Segment needs: every phase's target is fully known when
// the phase starts, so stageVelocityGoal()'s "Pattern B" -- cruise/re-target
// -- has no equivalent here). Between phases there is no coasting: each
// phase's own position-control Ruckig solve decelerates to rest AT its own
// target as an intrinsic property of the whole-trajectory solve, so the next
// phase always starts from a clean, already-at-rest seed.
//
// POSE-FREE, exactly like Planner's DISTANCE/TURN/ROTATION goal kinds: stop
// conditions are encoder-only (STOP_DISTANCE/STOP_ROTATION) plus the
// STOP_TIME safety net -- NO STOP_HEADING/STOP_POSITION (those need the fused
// pose 093 removed with PoseEstimator). PRE_PIVOT/TERMINAL_PIVOT convert
// their angle target into a per-wheel-arc STOP_ROTATION threshold via
// trackwidth, mirroring handleRT's own `arc = |angle| * trackwidth/2`
// (source/commands/motion_commands.cpp).
//
// NO motor/blackboard/CODAL dependency -- exactly like Motion::JerkTrajectory
// "knows nothing about goal kinds" (jerk_trajectory.h's own class comment):
// this class takes encoder observations and a `now` timestamp as tick()
// arguments only, and returns a body twist. It has no Hal::Motor/
// Subsystems::Drivetrain/blackboard reference or pointer.
#pragma once

#include <stdint.h>

#include "messages/common.h"
#include "messages/motor.h"
#include "messages/planner.h"
#include "motion/jerk_trajectory.h"
#include "motion/motion_baseline.h"
#include "motion/segment.h"

namespace Motion {

class SegmentExecutor {
 public:
  // configure -- store the executor's default motion limits (msg::
  // PlannerConfig, reused as-is -- see architecture-update.md Section 8).
  // Per-segment speedMax/accelMax/jerkMax/yawRateMax/yawAccelMax/yawJerkMax
  // (segment.h) override these on a per-start() basis when nonzero; a 0
  // field falls back to whatever configure() last stored.
  void configure(const msg::PlannerConfig& config);

  // start -- stage a fresh segment: picks the first non-degenerate phase
  // (PRE_PIVOT -> TRANSLATE -> TERMINAL_PIVOT, skipping any phase whose own
  // target is ~0) and solves it. `now` is [ms] system time; `trackwidth` is
  // [mm], needed ONLY to convert PRE_PIVOT/TERMINAL_PIVOT's angle deltas into
  // a per-wheel-arc STOP_ROTATION threshold (mirrors handleRT's own `arc =
  // |angle| * trackwidth/2`, motion_commands.cpp:614). Takes no encoder
  // observations -- mirrors Planner::apply()'s own "no observations at stage
  // time" constraint (planner.h's class comment); the first phase's
  // Motion::MotionBaseline is captured on the first tick() call after
  // start(), which does receive observations. A fully degenerate segment
  // (distance~=0, direction~=0, finalHeading~=direction) leaves the executor
  // idle() immediately.
  void start(const Segment& segment, uint32_t now, float trackwidth);

  // stop -- force the graceful decel-to-zero immediately from the CURRENTLY
  // ACTIVE phase's own live channel state, abandoning any remaining phases
  // once it converges (mirrors the STOP wire verb's "clears the ring and
  // triggers the executor's graceful decel-to-zero" semantics,
  // architecture-update.md Section 6). No-op while idle().
  void stop(uint32_t now);

  // tick -- advance one pass: capture the active phase's baseline on its
  // first call, sample its channel, run the divergence-triggered replan
  // (while the phase's own stop has not fired), evaluate the phase's stop
  // conditions (encoder-only STOP_DISTANCE/STOP_ROTATION + the STOP_TIME
  // safety net), arm/track the presolved graceful decel-to-zero, and advance
  // to the next phase (or idle()) once that decel converges. `now`/
  // `encLeft`/`encRight` are THIS TICK's observations only -- never stored
  // beyond this call. Returns the commanded body twist (v_x for TRANSLATE,
  // omega for PRE_PIVOT/TERMINAL_PIVOT, the other component always exactly
  // 0 -- a Segment's phases are never simultaneously translating and
  // pivoting).
  msg::BodyTwist3 tick(uint32_t now, const msg::MotorState& encLeft,
                       const msg::MotorState& encRight);

  // active -- true while a phase (PRE_PIVOT/TRANSLATE/TERMINAL_PIVOT) or its
  // trailing graceful decel-to-zero is still running.
  bool active() const;
  bool idle() const { return !active(); }

  // converged -- true once the WHOLE segment -- every phase it needed, plus
  // each phase's own trailing graceful stop -- has settled to a literal-zero
  // twist. Equivalent to !active(), spelled out separately per this ticket's
  // own acceptance criteria ("a way to query whether the whole segment...
  // has converged").
  bool converged() const { return !active(); }

 private:
  enum class Phase : uint8_t { IDLE, PRE_PIVOT, TRANSLATE, TERMINAL_PIVOT };

  // effectiveLinearConfig/effectiveRotationalConfig -- fold a Segment's own
  // per-segment limit overrides (0 => fall back to config_) onto config_,
  // producing the msg::PlannerConfig each channel is configure()'d with for
  // this segment's whole lifetime (all phases on a given channel share one
  // segment-level limit set -- see segment.h's own field comments).
  msg::PlannerConfig effectiveLinearConfig(const Segment& segment) const;
  msg::PlannerConfig effectiveRotationalConfig(const Segment& segment) const;

  // beginPrePivot/beginTranslate/beginTerminalPivot -- (re)solve the named
  // phase's channel from rest to its own precomputed target (preRotateTarget_/
  // translateTarget_/terminalPivotTarget_, set once by start()), reset this
  // phase's stops_[]/stopsCount_ to its own built-in stop (STOP_ROTATION or
  // STOP_DISTANCE) plus a generous STOP_TIME safety net, and set phase_.
  void beginPrePivot(uint32_t now);
  void beginTranslate(uint32_t now);
  void beginTerminalPivot(uint32_t now);

  // advancePhase -- called once the active phase's trailing graceful decel
  // has converged (or forceStopArmed_ is set): moves to the next
  // non-degenerate phase (skipping any whose own target is ~0), or to
  // Phase::IDLE if none remain / forceStopArmed_ was set (stop()'s "abandon
  // any remaining phases" contract).
  void advancePhase(uint32_t now);

  // captureBaseline -- snapshot a Motion::MotionBaseline from this tick's
  // observations for the CURRENTLY active phase. Pose fields (heading0/
  // pose0X/pose0Y) are left at 0 -- dead, this executor is pose-free (see
  // class comment) -- only t0/enc0/encDiff0/vSign/omegaSign are meaningful.
  void captureBaseline(uint32_t now, const msg::MotorState& encLeft,
                       const msg::MotorState& encRight);

  // appendStop -- append one stop condition to the active phase's stops_[]
  // (bounded to the 4-slot cap, mirrors Planner::appendStop()).
  void appendStop(msg::StopKind kind, float a, float b = 0.0f, float ax = 0.0f);

  // maybeReplanTranslate/maybeReplanPivot -- the divergence-triggered replan
  // (architecture-update.md Decision 10, ported from Planner's own
  // maybeReplanDistance()/maybeReplanRotational()): while the active phase's
  // own stop has not fired, compares the channel's own remembered
  // plan-remaining against Motion::remainingToStop()'s MEASURED remaining;
  // past kDivergenceThreshold/kRotDivergenceThreshold, retarget()s (or, past
  // kGrossDivergenceThreshold/kRotGrossDivergenceThreshold, reanchor()s) the
  // channel -- guarded by the SAME three guards Planner's own methods
  // enforce (stop-not-fired via the caller, no-reverse-target, and a shared
  // kMinReplanInterval rate limit).
  void maybeReplanTranslate(uint32_t now, const msg::MotorState& encLeft,
                            const msg::MotorState& encRight);
  void maybeReplanPivot(uint32_t now, const msg::MotorState& encLeft,
                        const msg::MotorState& encRight);

  // armTranslateStopDecel/armPivotStopDecel -- the presolved graceful
  // decel-to-zero (ported from Planner's armDistanceStopDecel()/
  // armRotationalStopDecel()): called exactly once, the instant the active
  // phase's own stop condition fires (or stop() is called externally). If
  // the channel's own plan has already naturally converged to rest, this is
  // a no-op; otherwise re-solves a fresh velocity-control decel-to-rest
  // seeded from the channel's own current sampled state (never encLeft/
  // encRight -- Motion::JerkTrajectory's own seeding contract,
  // jerk_trajectory.h).
  void armTranslateStopDecel(uint32_t now);
  void armPivotStopDecel(uint32_t now);

  // linearElapsed/rotationalElapsed -- [s] elapsed time since the named
  // channel's most recent (re)solve. Ported from Planner's own identically-
  // named helpers.
  float linearElapsed(uint32_t now) const;
  float rotationalElapsed(uint32_t now) const;

  msg::PlannerConfig config_ = {};

  Motion::JerkTrajectory linear_;
  Motion::JerkTrajectory rotational_;

  uint32_t linearSolveMs_ = 0;      // [ms] absolute time of linear_'s most recent (re)solve
  uint32_t rotationalSolveMs_ = 0;  // [ms] absolute time of rotational_'s most recent (re)solve

  float linearTarget_ = 0.0f;       // [mm] linear_'s CURRENT target, its own frame
  float linearCeiling_ = 0.0f;      // [mm/s] per-call max_velocity most recently used
  float rotationalTarget_ = 0.0f;   // [rad] rotational_'s CURRENT target, its own frame
  float rotationalCeiling_ = 0.0f;  // [rad/s] per-call max_velocity most recently used
  // arcScale_ -- converts a pivot phase's STOP_ROTATION measured remaining
  // (a per-wheel ARC, mm) into rotational_'s own radian domain, mirroring
  // Planner's rotationalArcScale_. For THIS executor it is always exactly
  // trackwidth_/2 (arc is DEFINED as |targetAngle| * trackwidth/2 at phase
  // start -- unlike Planner's RT, which took an independently-supplied arc
  // threshold, a Segment's arc and angle target are the SAME relationship by
  // construction, so this never needs a per-phase recompute beyond
  // trackwidth_ itself). [mm/rad]
  float arcScale_ = 1.0f;

  // Divergence-replan rate limiting (Decision 10 guard 3) -- shared across
  // whichever single phase is active at a time, exactly like Planner's
  // lastReplanMs_.
  uint32_t lastReplanMs_ = 0;  // [ms] last divergence-triggered replan (or phase-start) time
  static constexpr float kDivergenceThreshold = 3.0f;        // [mm]
  static constexpr float kGrossDivergenceThreshold = 40.0f;  // [mm]
  static constexpr uint32_t kMinReplanInterval = 60;         // [ms] shared, linear+rotational
  static constexpr float kRotDivergenceThreshold = 0.03f;      // [rad]
  static constexpr float kRotGrossDivergenceThreshold = 0.3f;  // [rad]

  bool stopping_ = false;         // true during the trailing graceful decel-to-zero
  bool baselineCaptured_ = false;
  bool forceStopArmed_ = false;   // true once stop() is called -- skip remaining phases
  uint32_t softDeadline_ = 0;     // [ms] absolute deadline for the graceful decel-to-zero
  static constexpr uint32_t kSoftDeadlineMs = 3000;  // [ms] matches Planner::kSoftDeadlineMs

  msg::StopCondition stops_[4] = {};
  uint8_t stopsCount_ = 0;

  Motion::MotionBaseline baseline_ = {};

  Phase phase_ = Phase::IDLE;

  // Per-segment precomputed phase targets/gates (start()'s own bookkeeping --
  // segment.h's fields are relative deltas; these are the exact Ruckig
  // targets each phase solves to).
  float preRotateTarget_ = 0.0f;      // [rad] == segment.direction
  float translateTarget_ = 0.0f;      // [mm]  == segment.distance
  float terminalPivotTarget_ = 0.0f;  // [rad] == segment.finalHeading - segment.direction
  bool needPrePivot_ = false;
  bool needTranslate_ = false;
  bool needTerminalPivot_ = false;
  float trackwidth_ = 0.0f;  // [mm]
};

}  // namespace Motion
