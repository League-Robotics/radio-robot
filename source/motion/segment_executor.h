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
//   | Phase          | Fires when              | Channel / solve                    | Completion (encoder-only)    |
//   |----------------|--------------------------|-------------------------------------|-------------------------------|
//   | PRE_PIVOT      | skip if |direction| ~= 0 | rotational solveToRest(direction), heading PD-corrected (sprint 098 M3) | heading tolerance+dwell (M4) + STOP_TIME backstop |
//   | TRANSLATE      | skip if |distance| ~= 0  | linear solveToRest(distance)         | STOP_DISTANCE at |distance|   |
//   | TERMINAL_PIVOT | skip if finalHeading~=direction | rotational solveToRest(finalHeading-direction), heading PD-corrected (sprint 098 M3) | heading tolerance+dwell (M4) + STOP_TIME backstop |
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
// Sprint 098 (architecture-update.md M3/M4/M5) adds the outer heading PD
// cascade for PRE_PIVOT/TERMINAL_PIVOT ONLY (never TRANSLATE, never BLEND):
// their commanded omega is Kp/Kd-corrected against MEASURED (encoder-
// derived) heading/rate rather than the raw Ruckig-plan sample while the
// phase is actively converging, and their completion is a tolerance+dwell
// gate on that same measured heading/rate -- STOP_ROTATION is no longer
// appended to their stops_[] (STOP_TIME stays, as the independent
// stall/non-convergence backstop). TRANSLATE's STOP_DISTANCE and BLEND's
// STOP_ROTATION/STOP_DISTANCE pair are both unchanged by this.
//
// NO motor/blackboard/CODAL dependency -- exactly like Motion::JerkTrajectory
// "knows nothing about goal kinds" (jerk_trajectory.h's own class comment):
// this class takes encoder observations and a `now` timestamp as tick()
// arguments only, and returns a body twist. It has no Hal::Motor/
// Subsystems::Drivetrain/blackboard reference or pointer.
//
// Sprint 098 (M6, ticket 004, Stage 2, OPTIONAL): tick() gains a fourth,
// DEFAULTED `msg::PoseEstimate` parameter -- Decision 4's reused (not new)
// pose seam. Defaulted so every pre-004 caller (every OTHER caller in this
// codebase, and every one of this file's own pre-098-004 sim scenarios)
// compiles and behaves UNCHANGED, passing no fourth argument at all, which
// default-constructs an all-zero/`stamp.valid == false` `msg::
// PoseEstimate{}` -- bit-identical to Stage 1's own hardcoded empty one.
// Still POSE-FREE for X/Y and for
// PRE_PIVOT/TERMINAL_PIVOT's own completion/replan logic (STOP_DISTANCE/
// STOP_ROTATION/STOP_TIME, `maybeReplanPivot()`/`maybeReplanTranslate()`,
// unchanged) -- ONLY `measuredHeading()`'s PD-cascade input (M3) prefers the
// supplied pose's `pose.h`, and only while `pose.stamp.valid` (the caller's
// own combined valid/connected signal -- see `Subsystems::Drivetrain::
// tick()`'s doc comment for where that gets folded in), falling back
// tick-by-tick to the unmodified ticket-002 encoder path otherwise.
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
  //
  // `pose` (098-004/M6, Stage 2, optional, defaulted): this tick's OTOS-
  // sourced `msg::PoseEstimate`, consumed ONLY by PRE_PIVOT/TERMINAL_PIVOT's
  // `measuredHeading()` step, and ONLY while `pose.stamp.valid` -- see this
  // class's own file header and `measuredHeading()`'s doc comment. Never
  // stored beyond this call, same as `encLeft`/`encRight`.
  msg::BodyTwist3 tick(uint32_t now, const msg::MotorState& encLeft,
                       const msg::MotorState& encRight,
                       const msg::PoseEstimate& pose = msg::PoseEstimate{});

  // active -- true while a phase (PRE_PIVOT/TRANSLATE/TERMINAL_PIVOT/BLEND)
  // or its trailing graceful decel-to-zero is still running.
  bool active() const;
  bool idle() const { return !active(); }

  // offerNext/hasPending/streaming -- streaming merge support (OOP
  // 2026-07-09, realizing the 094 issue's original "decel-to-zero only when
  // the queue empties" semantic). While a STREAMING segment (segment.stream,
  // wire `MOVE ... s=1`) is executing, the Drivetrain pre-loads ONE pending
  // stream segment; on the executor's next tick it MERGES: remaining
  // distance/heading ACCUMULATE and both channels retarget() from their
  // current moving state (Phase::BLEND -- translate+pivot simultaneous, a
  // differential arc). Merging is what makes joystick micro-segment
  // streaming drivable: each plan is solved to-rest, so waiting for its stop
  // to fire chains from ~zero velocity, and a from-rest segment of duration
  // T covers only ~a*T^2/4 -- unchained/late-chained streams cap at a
  // crawl. The merged plan's own to-rest tail IS the graceful stop when the
  // stream runs dry. Merging is stream-only by design: discrete segments
  // would corrupt (fwd 300 + back 300 merges to net 0). offerNext returns
  // false while idle, already holding a pending, or force-stopping.
  bool offerNext(const Segment& segment);
  bool hasPending() const { return hasPending_; }
  bool streaming() const { return phase_ != Phase::IDLE && currentStream_; }

  // replaceStream (MOVER, OOP 2026-07-09) -- the deadman-velocity teleop
  // primitive: REPLACE whatever is executing with this segment, replanned
  // from the channels' CURRENT velocity (solveToVelocity's own seeding).
  // time > 0 (the teleop form): velocity control toward segment.v/.omega,
  // with a deadline of `time` ms -- if no further replacement arrives, the
  // executor decels gracefully to rest (the deadman). time == 0: a
  // position-mode replace (targets swapped in, retarget()ed from the moving
  // state). Works from IDLE (starts fresh) or mid-anything.
  void replaceStream(const Segment& segment, uint32_t now, float trackwidth);

  // remainingLinear -- plan-frame remaining translation [mm] (0 when idle).
  // The streaming teleop's flow-control signal: the host holds this near a
  // target (~0.4s of motion) so the plan's to-rest tail never bites
  // mid-stream (buffer too shallow = the 5Hz accelerate/brake pulsing).
  float remainingLinear(uint32_t now) const;   // [ms]

  // converged -- true once the WHOLE segment -- every phase it needed, plus
  // each phase's own trailing graceful stop -- has settled to a literal-zero
  // twist. Equivalent to !active(), spelled out separately per this ticket's
  // own acceptance criteria ("a way to query whether the whole segment...
  // has converged").
  bool converged() const { return !active(); }

 private:
  enum class Phase : uint8_t { IDLE, PRE_PIVOT, TRANSLATE, TERMINAL_PIVOT, BLEND };

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

  // beginStreamFresh -- start a STREAMING segment from idle: single-phase
  // BLEND, both channels solved from rest toward the segment's targets.
  void beginStreamFresh(uint32_t now);

  // buildBlendStops -- (re)build the BLEND stop set for the current merged
  // targets (encoder stops + STOP_TIME net).
  void buildBlendStops();

  // mergePending -- consume pending_ MID-plan: sample each channel's current
  // remaining, ACCUMULATE the pending segment's distance/heading onto it,
  // retarget() both channels from their moving state, rebase the baseline
  // from this tick's observations, and rebuild the stops for the merged
  // targets. No replans in BLEND (the next merge IS the correction). `pose`
  // is threaded through to captureBaseline()'s rebase call only -- BLEND
  // never consumes it otherwise (M3's heading PD cascade is PRE_PIVOT/
  // TERMINAL_PIVOT only, never BLEND -- see this file's own header).
  void mergePending(uint32_t now, const msg::MotorState& encLeft,
                    const msg::MotorState& encRight, const msg::PoseEstimate& pose);

  // advancePhase -- called once the active phase's trailing graceful decel
  // has converged (or forceStopArmed_ is set): moves to the next
  // non-degenerate phase (skipping any whose own target is ~0), or to
  // Phase::IDLE if none remain / forceStopArmed_ was set (stop()'s "abandon
  // any remaining phases" contract).
  void advancePhase(uint32_t now);

  // captureBaseline -- snapshot a Motion::MotionBaseline from this tick's
  // observations for the CURRENTLY active phase. heading0/pose0X/pose0Y stay
  // at 0 -- still dead, reserved for a future full EKF-fused pose (sprint
  // 099's scope), NOT this ticket. otosHeading0 (098-004/M6) IS captured --
  // `pose.pose.h` when `pose.stamp.valid` this tick, else 0.0f (unused in
  // that case) -- see motion_baseline.h's own field comment. t0/enc0/
  // encDiff0/vSign/omegaSign are unchanged from before this ticket.
  void captureBaseline(uint32_t now, const msg::MotorState& encLeft,
                       const msg::MotorState& encRight, const msg::PoseEstimate& pose);

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
  //
  // Sprint 098 (M5): maybeReplanPivot()'s sub-gross (kRotDivergenceThreshold,
  // retarget-only) branch is retired to a no-op -- the outer heading PD
  // cascade (M3, tick()'s PRE_PIVOT/TERMINAL_PIVOT branch) is now the
  // continuous corrector for nominal tracking lag; the gross-divergence
  // (kRotGrossDivergenceThreshold, reanchor) branch is UNCHANGED, staying
  // live as stall protection. Since STOP_ROTATION is no longer appended to
  // stops_[] for these two phases (M4), maybeReplanPivot() reconstructs the
  // same target-arc StopCondition locally instead of looking it up from
  // stops_[].
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

  // measuredHeading -- M3/M4's shared measured-heading/measured-rate
  // derivation for PRE_PIVOT/TERMINAL_PIVOT (architecture-update.md M3),
  // used by both the outer heading PD cascade and the tolerance/dwell
  // completion gate -- both need the identical quantities, every tick.
  //
  // thetaMeasured (098-004/M6, Stage 2): prefers the OTOS source -- wrapped
  // `pose.pose.h - baseline_.otosHeading0`, in the SAME signed
  // relative-to-phase-start frame as the encoder path below -- whenever
  // `pose.stamp.valid` this tick (the caller's own combined valid/connected
  // signal). Falls back, TICK-BY-TICK (never latched for the phase), to
  // ticket 002's unmodified encoder-differential derivation otherwise: the
  // encoder heading relative to the phase's OWN baseline (baseline_.
  // encDiff0), in the SAME signed frame as rotationalTarget_/rotational_'s
  // own sample() -- see the .cpp for the derivation note (algebraically
  // identical to rotationProgress()'s STOP_ROTATION geometry, motion/
  // stop_condition.cpp, just expressed in radians instead of per-wheel-arc
  // mm). That encoder fallback itself falls back to thetaDesired
  // (headingError == 0 that tick -- never fabricate a phantom delta from a
  // momentarily missing observation) when either wheel's position.has is
  // false.
  //
  // omegaMeasured is ALWAYS encoder-derived, unchanged from ticket 002 --
  // Stage 2 trusts OTOS heading only, never OTOS twist/rate (architecture-
  // update.md M6's own boundary) -- falling back to omegaDesired when
  // either wheel's velocity.has is false (mirrors maybeReplanPivot()'s
  // existing reanchor-seed fallback exactly).
  void measuredHeading(const msg::MotorState& encLeft, const msg::MotorState& encRight,
                       const msg::PoseEstimate& pose, float thetaDesired, float omegaDesired,
                       float* thetaMeasured, float* omegaMeasured) const;

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

  // Phase-anchored replan window (2026-07-09 multi-hump fix): replans are
  // permitted only until this absolute deadline, set ONCE per phase from the
  // ORIGINAL solve's duration (kReplanWindowFraction of it). Anchoring to
  // the phase -- not the latest re-solve -- is what terminates the cascade:
  // a per-plan window re-opens with every retarget/reanchor, so the tail of
  // each re-solve could keep spawning the next (the decaying-hump defect).
  uint32_t phaseReplanDeadline_ = 0;  // [ms] absolute
  // Divergence thresholds -- recalibrated 2026-07-11, per channel: each must
  // sit ABOVE its lag model's own noise floor (or the replan fires on model
  // error and shaves the plan) and BELOW the real divergence it exists to
  // catch. The model (measured == plan kDeadTime ago, exact via
  // JerkTrajectory::peek()) still carries a feedback-dependent residual --
  // kp acting on the tracking error makes the plant's true delay
  // ramp-dependent, worth ~0.03 rad (~2mm of wheel arc) of phantom
  // divergence during hard accel/decel at speed.
  //
  // ROTATION (0.10 rad): pivots CANNOT saturate the plant (the 6 rad/s yaw
  // ceiling commands ~384 mm/s wheels, under the sim plant's 400 and the
  // real motors' ~600 plateau), so the only sub-gross divergence source is
  // a stalled/bogged wheel -- which grows by ~omega*kDeadTime every
  // dead-time (0.25 rad per 40ms at the yaw ceiling) and clears 0.10 rad
  // within ~2 loop passes. The OLD 0.03 sat exactly on the phantom floor
  // and shrink-retargeted every high-speed pivot ~2 deg short per hit.
  // LINEAR (5mm): straight moves DO routinely saturate the plant --
  // v_body_max (1000) deliberately exceeds both plants' ceilings, and the
  // replan's extend-on-deficit IS the designed correction for the real
  // travel deficit saturation accrues (observed: a D 345 cruising at a
  // planned 465 mm/s on the 400 mm/s sim plant accrues ~12mm; at 10mm the
  // threshold outran the deficit inside the replan window and D landed
  // 12mm short). 5mm clears the ~2-3mm phantom floor with margin while
  // catching the deficit within ~75ms of saturation onset.
  static constexpr float kDivergenceThreshold = 5.0f;        // [mm]
  static constexpr float kGrossDivergenceThreshold = 40.0f;  // [mm]
  static constexpr uint32_t kMinReplanInterval = 60;         // [ms] shared, linear+rotational
  // ROTATION threshold raised 0.10 -> 0.22 (2026-07-11, second pass): at
  // the 6 rad/s yaw ceiling ONE loop pass of per-wheel sampling jitter is
  // ~0.15 rad of apparent divergence -- 0.10 sat below that floor, so long
  // ceiling-speed cruises (360 deg pivots; 90/180 barely dwell there)
  // accumulated shrink-retargets ~25-30 deg short while short pivots were
  // fine. A genuinely stalled wheel accrues ~0.15 rad EVERY pass, so 0.22
  // still trips within ~2 passes.
  static constexpr float kRotDivergenceThreshold = 0.22f;      // [rad]
  static constexpr float kRotGrossDivergenceThreshold = 0.3f;  // [rad]

  bool stopping_ = false;         // true during the trailing graceful decel-to-zero
  bool baselineCaptured_ = false;
  bool forceStopArmed_ = false;   // true once stop() is called -- skip remaining phases
  uint32_t softDeadline_ = 0;     // [ms] absolute deadline for the graceful decel-to-zero
  static constexpr uint32_t kSoftDeadlineMs = 3000;  // [ms] matches Planner::kSoftDeadlineMs

  // Heading tolerance/dwell completion gate (M4, sprint 098) --
  // PRE_PIVOT/TERMINAL_PIVOT only. headingDwellActive_ tracks whether
  // |rotationalTarget_ - thetaMeasured| < kHeadingTol && |omegaMeasured| <
  // kHeadingRateTol has held CONTINUOUSLY since headingDwellStartMs_; reset
  // to false the instant that AND goes false, and (re)initialized at phase
  // start (beginPrePivot()/beginTerminalPivot()) -- see segment_executor.cpp's
  // anonymous namespace for kHeadingTol/kHeadingRateTol/kHeadingDwellMs
  // themselves (file-local constexpr, architecture-update.md Decision 3).
  bool headingDwellActive_ = false;
  uint32_t headingDwellStartMs_ = 0;  // [ms] absolute; valid only while headingDwellActive_

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

  // Streaming merge slot -- see offerNext()'s doc comment.
  Segment pending_ = {};
  bool hasPending_ = false;
  bool currentStream_ = false;   // the segment in flight is a streaming one

  // Deadman-velocity mode (replaceStream with time > 0): velocity control
  // toward the segment's v/omega until velDeadline_, then graceful decel.
  bool velocityMode_ = false;
  uint32_t velDeadline_ = 0;   // [ms] absolute
};

}  // namespace Motion
