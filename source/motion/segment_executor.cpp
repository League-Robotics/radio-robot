// segment_executor.cpp -- Motion::SegmentExecutor implementation. See
// segment_executor.h for the class-level design notes (the lift from
// Subsystems::Planner + the new 3-phase sequencer).
#include "motion/segment_executor.h"

#include <math.h>

#include "motion/stop_condition.h"

namespace Motion {

namespace {

// kOutputHops/kAssumedPassPeriod/kDeadTime -- ported VERBATIM from
// source/subsystems/planner.cpp (ticket 087-009's fixed two-output-pass
// dead-time compensation, extended by 089-003/005's divergence replan). Same
// values, same #ifdef HOST_BUILD split -- this compile-time split is a
// PLANT-SPECIFIC physical quantity (real firmware measures ~120-140ms true
// command->motion delay, dominated by the Nezha flip-flop's own worst-case
// 2-port x 2-phase x 20ms = 80ms alignment; the sim leaf applies duty
// immediately, no flip-flop, no transport lag) -- see planner.cpp's own
// extensive comment for the full derivation. DO NOT retune independently of
// that comment; this is the actuation-latency mitigation the project memory
// warns must never regress (clasi/issues/
// motor-actuation-latency-flipflop-coupling.md).
#ifdef HOST_BUILD
// Sim: 1.5 output passes (30ms) -- RE-MEASURED 2026-07-11 after the sim
// boot velocity gains were calibrated to the plant (tests/_infra/sim/
// sim_api.cpp defaultMotorConfigSet(): kff = 1/kNominalMaxSpeed exact
// feed-forward, gentle kp/ki, velFiltAlpha = 1.0 honest measurement). The
// STRUCTURAL pipeline is 2 passes (one pass command->plant application +
// one pass plant->encoder reporting; encoder-vs-command-integral
// cross-correlation is ambiguous between 1 and 2 passes at 20ms sampling),
// but the EFFECTIVE lag the stop projection must model is smaller: the
// proportional term acts on the tracking error and claws back ~half a
// pass. Landed empirically between the structural bounds: at 2.0 the
// projected fire overestimates in-flight coverage and lands D 345 ~6mm
// short; at 1.0 pivots run ~3 deg long (phantom-extension replans); at
// 1.5, D 240/345/700 land within 1mm and pivots within 0.5 deg. The
// PREVIOUS gain set (stale kff = 0.0038 overdriving the feed-forward 52%,
// strong kp through a laggy 0.3 filter) made the plant LEAD its own
// command integral, which invalidated any calibration of this constant --
// gains first, then this.
constexpr float kOutputHops = 1.5f;
#else
// Real brick: RE-MEASURED 2026-07-11 at the bench (motion-onset delay,
// command-commit `now` vs first encoder movement `ts`, across 90/180/360
// pivots: 112-136ms; velocity onset 100-112ms) AFTER the velocity
// feed-forward was calibrated to the measured motor plateau (tovez.json
// vel_kff = 1/650 -- the old 0.001 made the plant so sluggish the shorter
// 80ms model happened to fit its EFFECTIVE lag). With the modeled dead
// time 40ms short of reality, maybeReplanPivot()'s expectation read every
// pivot as diverged and shrink-retargeted plans ~15-25 deg short
// (emitted integral 64 deg for a 90 deg ask). Same lesson as the sim
// constant above: this is a GAIN-DEPENDENT calibration -- re-measure it
// whenever the velocity gains change.
constexpr float kOutputHops = 6.0f;  // 120 ms
#endif
constexpr float kAssumedPassPeriod = 0.020f;  // [s] matches main.cpp's kPeriod
constexpr float kDeadTime = kOutputHops * kAssumedPassPeriod;  // [s]

// kAngleEps/kDistEps -- "~=0" thresholds for skipping a degenerate phase
// (architecture-update.md's phase table: "skip if |direction| ~= 0" etc.).
constexpr float kAngleEps = 1e-4f;  // [rad] ~ 0.0057 deg
constexpr float kDistEps = 0.5f;    // [mm]

// kReplanWindowFraction -- the divergence replan only acts during this
// leading fraction of the active plan. In the tail the plant's lag
// legitimately exceeds the modeled kDeadTime (decel + stiction, worst for
// counter-rotating pivots), so a tail comparison reads divergent and
// re-solves one last small trapezoid -- the residual-hump defect. The
// terminal path (dead-time-projected stop firing + exhaustion completion)
// owns the endgame instead.
constexpr float kReplanWindowFraction = 0.75f;

// kHeadingTol/kHeadingRateTol/kHeadingDwellMs -- M4's tolerance/dwell
// completion gate for PRE_PIVOT/TERMINAL_PIVOT (architecture-update.md
// Decision 3): a first-cut, code-edit-iterable set of constants, NOT
// PlannerConfig fields -- unlike heading_kp/heading_kd (plant-scale-
// dependent per-robot tunables), "how precise is done" is a control-design
// choice with nothing in this sprint's scope needing it to differ per robot
// (Decision 3's full rationale). kHeadingTol/kHeadingRateTol are the
// ~0.5deg/~1deg-per-s figures the issue itself suggests as a starting point;
// kHeadingDwellMs sits at the low end of the issue's 100-200ms suggested
// range. Like the kDivergenceThreshold family above, expect these to be
// recalibrated against real hardware (ticket 003) -- a compile-time
// constant, not a wire field, exactly like that family.
constexpr float kHeadingTol = 0.00873f;     // [rad] ~0.5 deg
constexpr float kHeadingRateTol = 0.0175f;  // [rad/s] ~1 deg/s
constexpr uint32_t kHeadingDwellMs = 150;   // [ms]

// wrapAngle -- wrap x into (-pi, pi]. Same atan2f(sinf, cosf) identity
// stop_condition.cpp's own file-local wrapAngle() and source/subsystems/
// pose_estimator.cpp's wrapPi() both use (098-004/M6: measuredHeading()'s
// OTOS path needs the SAME identity to recover a true incremental heading
// delta from the OTOS leaf's own wrapped absolute pose.h -- exact for any
// single-phase rotation under +-180 deg, which is this sprint's scope:
// PRE_PIVOT/TERMINAL_PIVOT are single in-place pivots, never multi-turn).
float wrapAngle(float x) { return atan2f(sinf(x), cosf(x)); }

}  // namespace

void SegmentExecutor::configure(const msg::PlannerConfig& config) { config_ = config; }

msg::PlannerConfig SegmentExecutor::effectiveLinearConfig(const Segment& segment) const {
  msg::PlannerConfig cfg = config_;
  if (segment.speedMax > 0.0f) cfg.v_body_max = segment.speedMax;
  if (segment.accelMax > 0.0f) {
    cfg.a_max = segment.accelMax;
    cfg.a_decel = segment.accelMax;
  }
  if (segment.jerkMax > 0.0f) cfg.j_max = segment.jerkMax;
  return cfg;
}

msg::PlannerConfig SegmentExecutor::effectiveRotationalConfig(const Segment& segment) const {
  msg::PlannerConfig cfg = config_;
  if (segment.yawRateMax > 0.0f) cfg.yaw_rate_max = segment.yawRateMax;
  if (segment.yawAccelMax > 0.0f) cfg.yaw_acc_max = segment.yawAccelMax;
  if (segment.yawJerkMax > 0.0f) cfg.yaw_jerk_max = segment.yawJerkMax;
  return cfg;
}

void SegmentExecutor::appendStop(msg::StopKind kind, float a, float b, float ax) {
  if (stopsCount_ >= 4) return;  // cap already full -- mirrors Planner::appendStop()
  msg::StopCondition c;
  c.kind = kind;
  c.a = a;
  c.b = b;
  c.ax = ax;
  stops_[stopsCount_++] = c;
}

void SegmentExecutor::start(const Segment& segment, uint32_t now, float trackwidth) {
  trackwidth_ = trackwidth;
  arcScale_ = trackwidth_ * 0.5f;  // [mm/rad] -- see header's own doc comment

  msg::PlannerConfig linCfg = effectiveLinearConfig(segment);
  msg::PlannerConfig rotCfg = effectiveRotationalConfig(segment);
  linear_.configure(linCfg, /*isRotational=*/false);
  rotational_.configure(rotCfg, /*isRotational=*/true);
  linearCeiling_ = linCfg.v_body_max;
  rotationalCeiling_ = rotCfg.yaw_rate_max;

  preRotateTarget_ = segment.direction;
  translateTarget_ = segment.distance;
  terminalPivotTarget_ = segment.finalHeading - segment.direction;

  needPrePivot_ = fabsf(preRotateTarget_) > kAngleEps;
  needTranslate_ = fabsf(translateTarget_) > kDistEps;
  needTerminalPivot_ = fabsf(terminalPivotTarget_) > kAngleEps;

  stopping_ = false;
  baselineCaptured_ = false;
  forceStopArmed_ = false;
  hasPending_ = false;   // a fresh start abandons any stale merge slot
  velocityMode_ = false;
  currentStream_ = segment.stream;

  if (currentStream_) {
    // Streaming segment from idle: single-phase BLEND, both channels from
    // rest. `direction` is folded into finalHeading (streamers send dir=0).
    translateTarget_ = segment.distance;
    terminalPivotTarget_ = segment.finalHeading;
    preRotateTarget_ = 0.0f;
    needPrePivot_ = false;
    needTranslate_ = fabsf(translateTarget_) > kDistEps;
    needTerminalPivot_ = fabsf(terminalPivotTarget_) > kAngleEps;
    if (!needTranslate_ && !needTerminalPivot_) {
      phase_ = Phase::IDLE;
      return;
    }
    beginStreamFresh(now);
    return;
  }

  if (needPrePivot_) {
    beginPrePivot(now);
  } else if (needTranslate_) {
    beginTranslate(now);
  } else if (needTerminalPivot_) {
    beginTerminalPivot(now);
  } else {
    phase_ = Phase::IDLE;  // fully degenerate segment -- nothing to do
  }
}

bool SegmentExecutor::offerNext(const Segment& segment) {
  if (phase_ == Phase::IDLE || hasPending_ || forceStopArmed_) return false;
  pending_ = segment;
  hasPending_ = true;
  return true;
}

void SegmentExecutor::replaceStream(const Segment& segment, uint32_t now,
                                    float trackwidth) {
  trackwidth_ = trackwidth;
  arcScale_ = trackwidth_ * 0.5f;
  hasPending_ = false;     // replace semantics: anything pending is superseded
  forceStopArmed_ = false;
  stopping_ = false;
  currentStream_ = true;

  msg::PlannerConfig linCfg = effectiveLinearConfig(segment);
  msg::PlannerConfig rotCfg = effectiveRotationalConfig(segment);
  linear_.configure(linCfg, /*isRotational=*/false);
  rotational_.configure(rotCfg, /*isRotational=*/true);
  linearCeiling_ = linCfg.v_body_max;
  rotationalCeiling_ = rotCfg.yaw_rate_max;

  if (segment.time > 0.0f) {
    // Deadman-velocity mode: velocity control toward the SIGNED targets,
    // seeded from each channel's CURRENT state (solveToVelocity's own
    // contract) -- this IS "replan from the current velocity". No encoder
    // stops; the deadline is the only terminal, and a replacement arriving
    // before it simply re-solves (the deadman keeps getting re-primed).
    velocityMode_ = true;
    float vCeil = fabsf(segment.v) > 1.0f ? fabsf(segment.v) : 1.0f;
    float wCeil = fabsf(segment.omega) > 1e-3f ? fabsf(segment.omega) : 1e-3f;
    linear_.solveToVelocity(segment.v, vCeil);
    rotational_.solveToVelocity(segment.omega, wCeil);
    linearSolveMs_ = now;
    rotationalSolveMs_ = now;
    velDeadline_ = now + static_cast<uint32_t>(segment.time);
    stopsCount_ = 0;   // no encoder stops in velocity mode
    linearTarget_ = 0.0f;   // meaningless in velocity mode (remainingLinear -> 0-ish)
    rotationalTarget_ = 0.0f;
  } else {
    // Position-mode replace: swap in the new targets (NOT accumulated --
    // that is mergePending()'s job) and retarget() from the moving state.
    velocityMode_ = false;
    translateTarget_ = segment.distance;
    terminalPivotTarget_ = segment.finalHeading;
    preRotateTarget_ = 0.0f;
    needPrePivot_ = false;
    needTranslate_ = fabsf(translateTarget_) > kDistEps;
    needTerminalPivot_ = fabsf(terminalPivotTarget_) > kAngleEps;
    // UNCONDITIONAL else branches (2026-07-10 UB fix, mirrors
    // beginStreamFresh()/mergePending()): this leaves phase_ == BLEND below,
    // which samples both channels every pass, so neither may be left
    // un-calculate()'d regardless of its need flag.
    if (linear_.duration() > 0.0f) {
      linear_.retarget(translateTarget_);
    } else {
      linear_.reset();
      linear_.solveToRest(translateTarget_, linearCeiling_);
    }
    linearTarget_ = translateTarget_;
    linearSolveMs_ = now;
    if (rotational_.duration() > 0.0f) {
      rotational_.retarget(terminalPivotTarget_);
    } else {
      rotational_.reset();
      rotational_.solveToRest(terminalPivotTarget_, rotationalCeiling_);
    }
    rotationalTarget_ = terminalPivotTarget_;
    rotationalSolveMs_ = now;
    buildBlendStops();
  }

  lastReplanMs_ = now;
  phaseReplanDeadline_ = now;   // BLEND: divergence replans disabled
  phase_ = Phase::BLEND;
  baselineCaptured_ = false;    // next tick's obs rebase the encoder stops
}

// buildBlendStops -- (re)build the BLEND phase's stop set for the current
// translateTarget_/terminalPivotTarget_: the segment's own encoder stops +
// the STOP_TIME net.
void SegmentExecutor::buildBlendStops() {
  stopsCount_ = 0;
  if (needTranslate_) {
    appendStop(msg::StopKind::STOP_DISTANCE, fabsf(translateTarget_));
  }
  if (needTerminalPivot_) {
    appendStop(msg::StopKind::STOP_ROTATION, fabsf(terminalPivotTarget_) * arcScale_);
  }
  float speedMag = (fabsf(linearCeiling_) < 1.0f) ? 1.0f : fabsf(linearCeiling_);
  float omegaMag = (fabsf(rotationalCeiling_) < 1e-3f) ? 1e-3f : fabsf(rotationalCeiling_);
  float nominal = fabsf(translateTarget_) / speedMag * 1000.0f;         // [ms]
  float nominalRot = fabsf(terminalPivotTarget_) / omegaMag * 1000.0f;  // [ms]
  if (nominalRot > nominal) nominal = nominalRot;
  appendStop(msg::StopKind::STOP_TIME, nominal * 2.0f + 2000.0f);
}

void SegmentExecutor::beginStreamFresh(uint32_t now) {
  linear_.reset();
  rotational_.reset();
  // UNCONDITIONAL for both channels (2026-07-10 UB fix): tick()'s BLEND
  // branch samples BOTH linear_ AND rotational_ every pass -- translate and
  // pivot run simultaneously in BLEND -- so a channel whose need flag is
  // false here must still hold a real calculate()'d Ruckig trajectory
  // before phase_ becomes BLEND below, or sampling it is undefined behavior
  // (jerk_trajectory.h's calculated_ doc). translateTarget_/
  // terminalPivotTarget_ are already ~0 in the not-needed case (that IS
  // what "not needed" means -- see kDistEps/kAngleEps in start()), so this
  // just solves a trivial "stay at rest" trajectory for that channel; not a
  // behavior change for the needed-channel case, which solved exactly this
  // way before.
  linear_.solveToRest(translateTarget_, linearCeiling_);
  rotational_.solveToRest(terminalPivotTarget_, rotationalCeiling_);
  linearTarget_ = translateTarget_;
  rotationalTarget_ = terminalPivotTarget_;
  linearSolveMs_ = now;
  rotationalSolveMs_ = now;

  buildBlendStops();
  lastReplanMs_ = now;
  phaseReplanDeadline_ = now;   // BLEND: divergence replans disabled
  phase_ = Phase::BLEND;
}

void SegmentExecutor::mergePending(uint32_t now, const msg::MotorState& encLeft,
                                   const msg::MotorState& encRight,
                                   const msg::PoseEstimate& pose) {
  Segment seg = pending_;
  hasPending_ = false;

  // Each channel's remaining, in its own (post-retarget-rebased) frame,
  // sampled from the LIVE plan -- then the pending segment's contribution
  // ACCUMULATES onto it. This is the whole trick: the plan keeps moving and
  // its rest-point keeps being pushed ahead; the to-rest tail only ever
  // plays out when the stream stops feeding it.
  float linRemaining = linearTarget_ - linear_.sample(linearElapsed(now)).position;
  float rotRemaining = rotationalTarget_ - rotational_.sample(rotationalElapsed(now)).position;
  translateTarget_ = linRemaining + seg.distance;
  terminalPivotTarget_ = rotRemaining + seg.finalHeading;
  preRotateTarget_ = 0.0f;
  needPrePivot_ = false;
  needTranslate_ = fabsf(translateTarget_) > kDistEps;
  needTerminalPivot_ = fabsf(terminalPivotTarget_) > kAngleEps;

  msg::PlannerConfig linCfg = effectiveLinearConfig(seg);
  msg::PlannerConfig rotCfg = effectiveRotationalConfig(seg);
  linear_.configure(linCfg, /*isRotational=*/false);
  rotational_.configure(rotCfg, /*isRotational=*/true);
  linearCeiling_ = linCfg.v_body_max;
  rotationalCeiling_ = rotCfg.yaw_rate_max;

  // retarget() continues from the channel's CURRENT velocity/accel
  // (jerk_trajectory.h's chaining contract) -- including retarget(0) to
  // smoothly shed a channel the merged stream no longer drives. A channel
  // with no plan yet this life (duration 0) solves fresh from rest --
  // UNCONDITIONALLY (2026-07-10 UB fix, mirrors beginStreamFresh()): BLEND
  // samples both channels every pass, so a channel must never be left
  // un-calculate()'d here regardless of its need flag; translateTarget_/
  // terminalPivotTarget_ are already ~0 in the not-needed case.
  if (linear_.duration() > 0.0f) {
    linear_.retarget(translateTarget_);
  } else {
    linear_.reset();
    linear_.solveToRest(translateTarget_, linearCeiling_);
  }
  linearTarget_ = translateTarget_;
  linearSolveMs_ = now;

  if (rotational_.duration() > 0.0f) {
    rotational_.retarget(terminalPivotTarget_);
  } else {
    rotational_.reset();
    rotational_.solveToRest(terminalPivotTarget_, rotationalCeiling_);
  }
  rotationalTarget_ = terminalPivotTarget_;
  rotationalSolveMs_ = now;

  buildBlendStops();
  stopping_ = false;
  currentStream_ = true;
  lastReplanMs_ = now;
  phaseReplanDeadline_ = now;   // BLEND: divergence replans disabled
  phase_ = Phase::BLEND;
  captureBaseline(now, encLeft, encRight, pose);
  baselineCaptured_ = true;
}

void SegmentExecutor::beginPrePivot(uint32_t now) {
  rotational_.reset();
  rotational_.solveToRest(preRotateTarget_, rotationalCeiling_);
  rotationalTarget_ = preRotateTarget_;
  rotationalSolveMs_ = now;
  lastReplanMs_ = now;
  phaseReplanDeadline_ = now + static_cast<uint32_t>(
      kReplanWindowFraction * rotational_.duration() * 1000.0f);

  stopsCount_ = 0;
  // STOP_ROTATION is no longer appended here (M4, architecture-update.md):
  // the tolerance+dwell completion gate in tick() now owns PRE_PIVOT's
  // completion. STOP_TIME stays, unchanged, as the independent stall/
  // non-convergence backstop.
  float omegaMag = fabsf(rotationalCeiling_);
  float nominal = (omegaMag > 1e-3f) ? (fabsf(preRotateTarget_) / omegaMag) * 1000.0f : 0.0f;  // [ms]
  appendStop(msg::StopKind::STOP_TIME, nominal * 2.0f + 2000.0f);

  headingDwellActive_ = false;  // M4: (re)initialize the dwell timer at phase start
  phase_ = Phase::PRE_PIVOT;
}

void SegmentExecutor::beginTranslate(uint32_t now) {
  linear_.reset();
  linear_.solveToRest(translateTarget_, linearCeiling_);
  linearTarget_ = translateTarget_;
  linearSolveMs_ = now;
  lastReplanMs_ = now;
  phaseReplanDeadline_ = now + static_cast<uint32_t>(
      kReplanWindowFraction * linear_.duration() * 1000.0f);

  stopsCount_ = 0;
  float mag = fabsf(translateTarget_);
  appendStop(msg::StopKind::STOP_DISTANCE, mag);
  float speedMag = fabsf(linearCeiling_);
  if (speedMag < 1.0f) speedMag = 1.0f;
  float nominal = (mag / speedMag) * 1000.0f;  // [ms]
  appendStop(msg::StopKind::STOP_TIME, nominal * 2.0f + 2000.0f);

  phase_ = Phase::TRANSLATE;
}

void SegmentExecutor::beginTerminalPivot(uint32_t now) {
  rotational_.reset();
  rotational_.solveToRest(terminalPivotTarget_, rotationalCeiling_);
  rotationalTarget_ = terminalPivotTarget_;
  rotationalSolveMs_ = now;
  lastReplanMs_ = now;
  phaseReplanDeadline_ = now + static_cast<uint32_t>(
      kReplanWindowFraction * rotational_.duration() * 1000.0f);

  stopsCount_ = 0;
  // STOP_ROTATION is no longer appended here (M4, architecture-update.md):
  // the tolerance+dwell completion gate in tick() now owns TERMINAL_PIVOT's
  // completion. STOP_TIME stays, unchanged, as the independent stall/
  // non-convergence backstop.
  float omegaMag = fabsf(rotationalCeiling_);
  float nominal =
      (omegaMag > 1e-3f) ? (fabsf(terminalPivotTarget_) / omegaMag) * 1000.0f : 0.0f;  // [ms]
  appendStop(msg::StopKind::STOP_TIME, nominal * 2.0f + 2000.0f);

  headingDwellActive_ = false;  // M4: (re)initialize the dwell timer at phase start
  phase_ = Phase::TERMINAL_PIVOT;
}

void SegmentExecutor::advancePhase(uint32_t now) {
  stopping_ = false;
  if (forceStopArmed_) {
    forceStopArmed_ = false;
    phase_ = Phase::IDLE;
    return;
  }
  switch (phase_) {
    case Phase::PRE_PIVOT:
      if (needTranslate_) {
        beginTranslate(now);
      } else if (needTerminalPivot_) {
        beginTerminalPivot(now);
      } else {
        phase_ = Phase::IDLE;
      }
      return;
    case Phase::TRANSLATE:
      if (needTerminalPivot_) {
        beginTerminalPivot(now);
      } else {
        phase_ = Phase::IDLE;
      }
      return;
    case Phase::TERMINAL_PIVOT:
    case Phase::BLEND:
    case Phase::IDLE:
    default:
      phase_ = Phase::IDLE;
      hasPending_ = false;   // defensive: never strand a chain slot at idle
      return;
  }
}

void SegmentExecutor::stop(uint32_t now) {
  if (phase_ == Phase::IDLE) return;
  hasPending_ = false;   // STOP abandons the chain slot along with the phases
  forceStopArmed_ = true;
  if (!stopping_) {
    stopping_ = true;
    softDeadline_ = now + kSoftDeadlineMs;
    if (phase_ == Phase::TRANSLATE) {
      armTranslateStopDecel(now);
    } else if (phase_ == Phase::BLEND) {
      armTranslateStopDecel(now);
      armPivotStopDecel(now);
    } else {
      armPivotStopDecel(now);
    }
  }
}

void SegmentExecutor::captureBaseline(uint32_t now, const msg::MotorState& encLeft,
                                      const msg::MotorState& encRight,
                                      const msg::PoseEstimate& pose) {
  baseline_.t0 = now;
  baseline_.enc0 = 0.0f;
  baseline_.encDiff0 = 0.0f;
  if (encLeft.position.has && encRight.position.has) {
    baseline_.enc0 = (encLeft.position.val + encRight.position.val) * 0.5f;
    baseline_.encDiff0 = encRight.position.val - encLeft.position.val;
  }
  // heading0/pose0X/pose0Y are still dead fields, left at their default 0 --
  // reserved for a future full EKF-fused pose (sprint 099's scope), not this
  // ticket (see class comment). otosHeading0 (098-004/M6, Stage 2) IS
  // captured here -- the OTOS leaf's own pose.h when this tick's supplied
  // PoseEstimate is valid/connected, else 0.0f (unused/meaningless in that
  // case, mirroring encDiff0's own "captured every phase start regardless,
  // only meaningful when consumed" convention).
  baseline_.otosHeading0 = pose.stamp.valid ? pose.pose.h : 0.0f;
  if (phase_ == Phase::TRANSLATE) {
    baseline_.vSign =
        (translateTarget_ > 0.0f) ? 1.0f : (translateTarget_ < 0.0f ? -1.0f : 0.0f);
    baseline_.omegaSign = 0.0f;
  } else if (phase_ == Phase::BLEND) {
    baseline_.vSign =
        (translateTarget_ > 0.0f) ? 1.0f : (translateTarget_ < 0.0f ? -1.0f : 0.0f);
    baseline_.omegaSign = (terminalPivotTarget_ > 0.0f)
                              ? 1.0f
                              : (terminalPivotTarget_ < 0.0f ? -1.0f : 0.0f);
  } else {
    float target = (phase_ == Phase::PRE_PIVOT) ? preRotateTarget_ : terminalPivotTarget_;
    baseline_.omegaSign = (target > 0.0f) ? 1.0f : (target < 0.0f ? -1.0f : 0.0f);
    baseline_.vSign = 0.0f;
  }
}

float SegmentExecutor::linearElapsed(uint32_t now) const {
  return static_cast<float>(static_cast<int32_t>(now - linearSolveMs_)) * 0.001f;
}

float SegmentExecutor::rotationalElapsed(uint32_t now) const {
  return static_cast<float>(static_cast<int32_t>(now - rotationalSolveMs_)) * 0.001f;
}

// measuredHeading -- see segment_executor.h's doc comment for the contract.
// Derivation of the ENCODER path's sign convention: rotationProgress()'s own
// STOP_ROTATION geometry (motion/stop_condition.cpp) computes signedArc =
// ((encRight - encLeft) - baseline_.encDiff0) * omegaSign * 0.5, and that
// condition FIRES (signedArc >= cond.a, cond.a == |rotationalTarget_| *
// arcScale_ == |rotationalTarget_| * trackwidth_/2) exactly when
// ((encRight - encLeft) - baseline_.encDiff0) / trackwidth_ == rotationalTarget_
// -- for BOTH signs of omegaSign (the omegaSign/fabsf factors cancel
// algebraically at that boundary). So thetaMeasured below, computed WITHOUT
// any omegaSign multiplication, lands in exactly the same signed frame as
// rotationalTarget_ and rotational_'s own sample().position (0 at phase
// start, growing toward rotationalTarget_) -- the two are directly
// comparable, and a positive headingError (thetaDesired - thetaMeasured)
// always means "measured is behind the plan in the commanded direction",
// never the reverse. Do not reintroduce an omegaSign factor here.
//
// 098-004/M6 (Stage 2, optional): the OTOS path is gated STRICTLY on
// `pose.stamp.valid` (the caller's own combined valid/connected signal --
// Subsystems::Drivetrain::tick() folds odometer()->connected() into it
// before this is ever reached) -- when false (the default-constructed
// `msg::PoseEstimate{}` every pre-004 caller still passes), this function is
// BIT-IDENTICAL to ticket 002's own implementation. wrapAngle(pose.pose.h -
// baseline_.otosHeading0) recovers a true incremental heading delta from the
// OTOS leaf's own wrapped absolute pose.h the SAME way stop_condition.cpp's
// headingError() does for a fused pose -- lands in the SAME relative-to-
// phase-start signed frame the encoder path establishes above.
void SegmentExecutor::measuredHeading(const msg::MotorState& encLeft,
                                      const msg::MotorState& encRight,
                                      const msg::PoseEstimate& pose, float thetaDesired,
                                      float omegaDesired, float* thetaMeasured,
                                      float* omegaMeasured) const {
  if (pose.stamp.valid) {
    *thetaMeasured = wrapAngle(pose.pose.h - baseline_.otosHeading0);
  } else {
    *thetaMeasured = thetaDesired;  // fallback: no correction this tick if position is momentarily absent
    if (encLeft.position.has && encRight.position.has && trackwidth_ > 1e-3f) {
      *thetaMeasured =
          ((encRight.position.val - encLeft.position.val) - baseline_.encDiff0) / trackwidth_;
    }
  }
  // omegaMeasured stays encoder-derived ALWAYS -- Stage 2 trusts OTOS
  // heading only, never OTOS twist/rate (architecture-update.md M6's own
  // boundary) -- ticket 002's unmodified path.
  *omegaMeasured = omegaDesired;  // fallback -- mirrors maybeReplanPivot()'s reanchor seed exactly
  if (encLeft.velocity.has && encRight.velocity.has && trackwidth_ > 1e-3f) {
    *omegaMeasured = (encRight.velocity.val - encLeft.velocity.val) / trackwidth_;
  }
}

void SegmentExecutor::maybeReplanTranslate(uint32_t now, const msg::MotorState& encLeft,
                                           const msg::MotorState& encRight) {
  // Replan is a MID-plan divergence correction only -- permitted only inside
  // the PHASE-anchored window (kReplanWindowFraction of the phase's ORIGINAL
  // solve, set in begin*()). Anchoring to the phase, not the current plan,
  // terminates the cascade: each retarget/reanchor would otherwise re-open a
  // fresh window whose own tail reads divergent and re-solves the next
  // decaying trapezoid (the multi-hump defect). Past the deadline, the
  // terminal path (dead-time-projected stop firing + exhaustion completion
  // in tick()) owns the endgame; terminal accuracy is calibration work.
  if (static_cast<int32_t>(now - phaseReplanDeadline_) >= 0) return;
  if (linearElapsed(now) >= linear_.duration()) return;  // current plan exhausted
  if (static_cast<int32_t>(now - lastReplanMs_) < static_cast<int32_t>(kMinReplanInterval)) {
    return;
  }

  const msg::StopCondition* distCond = nullptr;
  for (uint8_t i = 0; i < stopsCount_; ++i) {
    if (stops_[i].kind == msg::StopKind::STOP_DISTANCE) {
      distCond = &stops_[i];
      break;
    }
  }
  if (distCond == nullptr) return;  // defensive -- beginTranslate() always appends one

  float vSign = baseline_.vSign;
  if (vSign == 0.0f) return;  // no meaningful direction to replan in

  float measuredRemainingMag = 0.0f;
  Motion::StopEvalResult r = Motion::remainingToStop(*distCond, baseline_, encLeft, encRight,
                                                     msg::PoseEstimate{}, &measuredRemainingMag);
  if (r != Motion::StopEvalResult::NOT_FIRED) return;  // FIRED: the caller's own stop-eval
                                                       // loop owns completion, not a replan

  float linElapsed = linearElapsed(now);
  Motion::JerkTrajectory::State state = linear_.sample(linElapsed);

  // Model the actuation dead-time INTO the comparison: the plant
  // legitimately lags the plan by kDeadTime, so the measured remaining is
  // EXPECTED to read what the plan's remaining WAS kDeadTime ago. Only
  // divergence BEYOND that modeled lag is real (093's "delay in the plan",
  // applied to the replan trigger) -- without this, the tail of every solve
  // reads as divergent and re-solves a fresh decaying trapezoid: the
  // multi-hump terminal defect. The expectation is taken EXACTLY from the
  // plan's own profile (peek() at elapsed - kDeadTime) rather than the old
  // linear planRemaining + v*kDeadTime approximation, whose missing
  // second-order term read as phantom divergence during hard decels.
  float lagElapsed = linElapsed - kDeadTime;
  if (lagElapsed < 0.0f) lagElapsed = 0.0f;
  Motion::JerkTrajectory::State lagged = linear_.peek(lagElapsed);
  float expectedRemainingMag = fabsf(linearTarget_ - lagged.position);
  float divergence = fabsf(expectedRemainingMag - measuredRemainingMag);
  if (divergence < kDivergenceThreshold) return;  // within tolerance -- no replan

  // Pipeline-in-flight travel, exact from the plan: command distance
  // already emitted that the plant hasn't shown yet.
  float pipelineMag = fabsf(state.position - lagged.position);
  float projectedRemainingMag = measuredRemainingMag - pipelineMag;
  if (projectedRemainingMag <= 0.0f) return;  // never solve backward

  // Same failed-solve discipline as maybeReplanPivot() below: rate-limit
  // regardless, but never touch the plan's target or timeline unless the
  // re-solve actually succeeded (a failure would otherwise restart the old
  // plan's elapsed clock and replay it mid-flight).
  lastReplanMs_ = now;
  if (divergence >= kGrossDivergenceThreshold) {
    float measuredPositionSigned = linearTarget_ - vSign * measuredRemainingMag;
    float measuredVelocitySigned = 0.0f;
    if (encLeft.velocity.has && encRight.velocity.has) {
      measuredVelocitySigned = (encLeft.velocity.val + encRight.velocity.val) * 0.5f;
    }
    if (!linear_.reanchor(measuredPositionSigned, measuredVelocitySigned)) {
      return;
    }
  } else {
    // EXTEND-ONLY -- see maybeReplanPivot()'s matching guard: arrival is
    // owned by the encoder stop; a mid-flight shrink only encodes
    // measurement bias (per-wheel sampling stagger).
    Motion::JerkTrajectory::State curLin = linear_.peek(linElapsed);
    float planRemainingNowMag = fabsf(linearTarget_ - curLin.position);
    if (projectedRemainingMag <= planRemainingNowMag) {
      return;
    }
    float newRemainingSigned = vSign * projectedRemainingMag;
    if (!linear_.retarget(newRemainingSigned)) {
      return;
    }
    linearTarget_ = newRemainingSigned;
  }
  linearSolveMs_ = now;
}

void SegmentExecutor::maybeReplanPivot(uint32_t now, const msg::MotorState& encLeft,
                                       const msg::MotorState& encRight) {
  // Mid-plan divergence correction only, inside the PHASE-anchored window --
  // see maybeReplanTranslate()'s matching gate for the full cascade
  // rationale (worst for pivots: counter-rotating wheels + stiction).
  if (static_cast<int32_t>(now - phaseReplanDeadline_) >= 0) return;
  if (rotationalElapsed(now) >= rotational_.duration()) return;  // current plan exhausted
  if (static_cast<int32_t>(now - lastReplanMs_) < static_cast<int32_t>(kMinReplanInterval)) {
    return;
  }

  // M5 (architecture-update.md): STOP_ROTATION is no longer appended to
  // stops_[] for PRE_PIVOT/TERMINAL_PIVOT (M4's tolerance+dwell gate
  // replaced its completion role) -- this function's own remaining/
  // expected geometry still needs the SAME target arc that used to be
  // looked up from stops_[], reconstructed locally instead, exactly
  // matching what beginPrePivot()/beginTerminalPivot() used to append
  // (fabsf(rotationalTarget_) * arcScale_).
  msg::StopCondition rotCond;
  rotCond.kind = msg::StopKind::STOP_ROTATION;
  rotCond.a = fabsf(rotationalTarget_) * arcScale_;

  float omegaSign = baseline_.omegaSign;
  if (omegaSign == 0.0f) return;  // no meaningful direction to replan in

  float measuredRemainingNative = 0.0f;
  Motion::StopEvalResult r = Motion::remainingToStop(
      rotCond, baseline_, encLeft, encRight, msg::PoseEstimate{}, &measuredRemainingNative);
  if (r != Motion::StopEvalResult::NOT_FIRED) return;  // FIRED: caller's own stop-eval loop owns
                                                       // completion, not a replan

  float measuredRemainingRad = measuredRemainingNative / arcScale_;

  float rotElapsed = rotationalElapsed(now);
  Motion::JerkTrajectory::State state = rotational_.sample(rotElapsed);

  // Dead-time lag modeled EXACTLY from the plan itself (see
  // maybeReplanTranslate()'s matching block for the full rationale): the
  // measured encoders show where the plant was kDeadTime ago, and the plan
  // knows precisely where IT was kDeadTime ago -- peek() there instead of
  // the old linear planRemaining + v*kDeadTime approximation, whose missing
  // second-order term (a*kDeadTime^2/2) read as phantom divergence during
  // every hard decel and shrink-retargeted high-speed pivots ~2 deg short.
  float lagElapsed = rotElapsed - kDeadTime;
  if (lagElapsed < 0.0f) lagElapsed = 0.0f;
  Motion::JerkTrajectory::State lagged = rotational_.peek(lagElapsed);
  float expectedRemainingRad = fabsf(rotationalTarget_ - lagged.position);
  float divergence = fabsf(expectedRemainingRad - measuredRemainingRad);
  if (divergence < kRotDivergenceThreshold) return;  // within tolerance -- no replan

  // Pipeline-in-flight travel, also exact from the plan: the command
  // distance already emitted that the plant hasn't shown yet.
  float pipelineRad = fabsf(state.position - lagged.position);
  float projectedRemainingRad = measuredRemainingRad - pipelineRad;
  if (projectedRemainingRad <= 0.0f) return;  // never solve backward

  // Rate-limit further attempts whether or not the solve below succeeds --
  // a persistently infeasible ask (see the failure note below) would
  // otherwise re-attempt every pass.
  lastReplanMs_ = now;

  // A FAILED solve must leave the in-flight plan completely untouched --
  // trajectory, target, AND timeline. JerkTrajectory's temp-solve keeps the
  // trajectory intact on failure, but resetting rotationalSolveMs_ here
  // without a new solve would RESTART the old plan's elapsed clock and
  // replay it from t=0 mid-flight (observed: a 90 deg pivot re-emitting the
  // full 90 deg profile on top of its first 77 deg). Failures are now an
  // expected outcome, not an anomaly: the directional velocity band
  // (jerk_trajectory.cpp min_velocity) correctly refuses any replan ask
  // that could only be met by reversing -- e.g. a retarget whose remaining
  // is shorter than the current speed can stop in.
  if (divergence >= kRotGrossDivergenceThreshold) {
    float measuredPositionSigned = rotationalTarget_ - omegaSign * measuredRemainingRad;
    // Seed the re-solve with the MEASURED angular rate -- (vR - vL)/track --
    // falling back to the plan's own sampled rate when either wheel velocity
    // is absent this tick. The original 0.0f seed ("no reliable measured
    // angular rate") told Ruckig the robot was AT REST while its wheels were
    // doing ~300 mm/s: the re-solve then planned from rest -- commanded
    // velocity CLIFFS to zero mid-pivot, the robot visibly stalls ~0.25s,
    // then a second full acceleration bell follows (the mid-move stall
    // observed on the bench 2026-07-11: endpoint acceptable, trajectory
    // garbage). A quantized/lagged measured rate is enormously closer to
    // the truth than a fabricated zero.
    float measuredOmega = state.velocity;   // plan-sampled fallback
    if (encLeft.velocity.has && encRight.velocity.has && trackwidth_ > 1e-3f) {
      measuredOmega = (encRight.velocity.val - encLeft.velocity.val) / trackwidth_;
    }
    if (!rotational_.reanchor(measuredPositionSigned, measuredOmega)) {
      return;
    }
  } else {
    // M5 (architecture-update.md): sub-gross EXTEND-only retirement,
    // PRE_PIVOT/TERMINAL_PIVOT specifically -- retired to a no-op. The
    // outer heading PD cascade (M3, tick()'s pivot branch) is now the
    // continuous corrector for nominal tracking lag; leaving this branch
    // live would re-solve the Ruckig plan out from under the PD loop's own
    // correction (a double-correction hazard). The gross-divergence branch
    // above is UNCHANGED and stays live -- a genuinely stalled/bogged wheel
    // is not something omega's gain alone can fix if the wheel-level
    // PID/motor cannot achieve it, so re-anchoring the plan to reality
    // stays as a safety measure.
    return;
  }
  rotationalSolveMs_ = now;
}

void SegmentExecutor::armTranslateStopDecel(uint32_t now) {
  if (linearElapsed(now) >= linear_.duration()) return;  // already converged -- no-op
  linear_.solveToVelocity(0.0f, linearCeiling_);
  linearSolveMs_ = now;
}

void SegmentExecutor::armPivotStopDecel(uint32_t now) {
  if (rotationalElapsed(now) >= rotational_.duration()) return;  // already converged -- no-op
  rotational_.solveToVelocity(0.0f, rotationalCeiling_);
  rotationalSolveMs_ = now;
}

msg::BodyTwist3 SegmentExecutor::tick(uint32_t now, const msg::MotorState& encLeft,
                                      const msg::MotorState& encRight,
                                      const msg::PoseEstimate& pose) {
  msg::BodyTwist3 twist{};
  if (phase_ == Phase::IDLE) return twist;  // never started, or fully converged

  if (!baselineCaptured_) {
    captureBaseline(now, encLeft, encRight, pose);
    baselineCaptured_ = true;
  }

  // Streaming merge: consume the pending segment IMMEDIATELY (mid-plan) --
  // remaining targets accumulate and the channels retarget() from their
  // current moving state. Merging must not wait for the stop to fire: each
  // plan is solved to-rest, so a fire-time chain would re-launch from ~zero
  // velocity every segment and the stream would crawl.
  if (!stopping_ && hasPending_ && currentStream_ && !velocityMode_) {
    mergePending(now, encLeft, encRight, pose);
  }

  float v = 0.0f;
  float omega = 0.0f;
  bool rotElapsedPastDuration = false;
  // M3/M4: PRE_PIVOT/TERMINAL_PIVOT's measured heading/rate this tick --
  // populated by the pivot branch below, read again further down by the
  // tolerance/dwell completion gate (both need the identical quantities).
  // Unused (left at 0) for TRANSLATE/BLEND.
  float thetaMeasured = 0.0f;
  float omegaMeasured = 0.0f;

  if (phase_ == Phase::TRANSLATE) {
    if (!stopping_) maybeReplanTranslate(now, encLeft, encRight);
    v = linear_.sample(linearElapsed(now)).velocity;
  } else if (phase_ == Phase::BLEND) {
    // BLEND (chained streaming segment): both channels run simultaneously --
    // on a differential that is an arc. No divergence replans here
    // (phaseReplanDeadline_ == chain instant): micro-segments arrive faster
    // than a replan could help; the next chain IS the correction.
    v = linear_.sample(linearElapsed(now)).velocity;
    float rotElapsed = rotationalElapsed(now);
    omega = rotational_.sample(rotElapsed).velocity;
    rotElapsedPastDuration = rotElapsed >= rotational_.duration();
    if (stopping_) {
      // Literal-0.0f snaps once each channel's stop-decel has fully played
      // out -- same PID zero-deadband rationale as the pivot branch below.
      if (rotElapsedPastDuration) omega = 0.0f;
      if (linearElapsed(now) >= linear_.duration()) v = 0.0f;
    }
  } else {
    // PRE_PIVOT / TERMINAL_PIVOT.
    if (!stopping_) maybeReplanPivot(now, encLeft, encRight);
    float rotElapsed = rotationalElapsed(now);
    Motion::JerkTrajectory::State desired = rotational_.sample(rotElapsed);
    rotElapsedPastDuration = rotElapsed >= rotational_.duration();
    if (stopping_) {
      // Riding the terminal graceful decel's own tail, UNCORRECTED -- the
      // outer heading PD cascade (M3, below) is gated to !stopping_, the
      // same terminal-reversal safety boundary maybeReplanPivot()'s own
      // `if (!stopping_)` guard above already respects: once the graceful
      // decel-to-zero is armed there is no longer a moving target to track
      // against, and a PD term nulling residual error here is exactly the
      // "small terminal correction that could ask for a brief reversal"
      // architecture-update.md's Risks section warns against.
      omega = desired.velocity;
    } else {
      // M3: outer heading PD cascade (architecture-update.md) -- corrects
      // the Ruckig plan's own desired rate against MEASURED heading/rate
      // before it reaches the (unchanged) inner wheel-velocity loop,
      // replacing the raw plan-velocity passthrough this branch used to
      // emit unconditionally. Kp=Kd=0 (an unmigrated robot's PlannerConfig)
      // degenerates this to exactly that old open-loop passthrough.
      measuredHeading(encLeft, encRight, pose, desired.position, desired.velocity, &thetaMeasured,
                      &omegaMeasured);
      omega = desired.velocity + config_.heading_kp * (desired.position - thetaMeasured) +
              config_.heading_kd * (desired.velocity - omegaMeasured);
    }
    // Snap to a LITERAL 0.0f once the STOP-TRIGGERED decel-to-zero has fully
    // converged (only while stopping_) -- ported VERBATIM rationale from
    // Subsystems::Planner::tick() (planner.cpp:964-966): Ruckig's own
    // past-duration "hold at final state" is not guaranteed bit-exact the
    // way Motion::VelocityRamp's linear `cur + (tgt - cur)` approach was, and
    // a ~1e-15-scale residual defeats Hal::MotorVelocityPid's zero-deadband
    // (spAbs <= minDuty) -- the integrator-freeze fix never engages for a
    // target that never reaches a literal 0.0f, producing a sustained,
    // slowly-decaying reverse-spin residual. NOT applied to the ongoing
    // (not-yet-stopping_) position-control solve, where the PD cascade (and,
    // for stall protection, the divergence replan) is supposed to keep
    // correcting against a lagging real plant -- forcing a hard 0 there
    // would fight it.
    if (stopping_ && rotElapsedPastDuration) omega = 0.0f;
  }

  if (stopping_) {
    bool linConv = linearElapsed(now) >= linear_.duration();
    bool converged = (phase_ == Phase::TRANSLATE)
                         ? linConv
                         : (phase_ == Phase::BLEND ? (linConv && rotElapsedPastDuration)
                                                    : rotElapsedPastDuration);
    int32_t dtDeadline = static_cast<int32_t>(now - softDeadline_);
    if (converged || dtDeadline >= 0) {
      advancePhase(now);
      if (phase_ != Phase::IDLE) {
        // The next phase's fresh solve starts from rest (v/omega == 0 at
        // elapsed == 0) -- capture its baseline immediately from THIS tick's
        // own observations (already in hand) rather than deferring an extra
        // idle tick, since -- unlike Planner::apply(), which has no
        // observations at all -- this transition happens from inside tick(),
        // which already received encLeft/encRight this call.
        captureBaseline(now, encLeft, encRight, pose);
        baselineCaptured_ = true;
      }
    }
  } else if (phase_ == Phase::BLEND && velocityMode_) {
    // Deadman-velocity mode: the deadline is the ONLY terminal. A
    // replaceStream() arriving before it re-solves and pushes it out; past
    // it, decel gracefully to rest (the deadman firing).
    if (static_cast<int32_t>(now - velDeadline_) >= 0) {
      stopping_ = true;
      softDeadline_ = now + kSoftDeadlineMs;
      armTranslateStopDecel(now);
      armPivotStopDecel(now);
    }
  } else {
    bool fired = false;
    // promptHalt: only a NON-position stop (STOP_TIME safety net, a sensor
    // stop) demands a freshly-armed decel ramp -- it can fire mid-cruise
    // with the plan still carrying arbitrary travel. A fired POSITION stop
    // (STOP_DISTANCE/STOP_ROTATION, raw or dead-time-projected) or plan
    // exhaustion instead RIDES THE PLAN'S OWN TAIL: every position-mode
    // plan is a Ruckig to-rest solve whose velocity and acceleration reach
    // 0 SIMULTANEOUSLY at the target -- that tail is already the optimal
    // graceful no-reverse stop. Re-arming solveToVelocity(0) from
    // mid-decel (velocity small, acceleration strongly negative) asks the
    // velocity interface for "v=0 as fast as possible", which it reaches
    // EARLY with acceleration still negative -- so the profile passes
    // through zero and comes back: a commanded ~1-2 deg terminal REVERSAL
    // at every pivot end (and the streamed-drain reverse-dip; on hardware,
    // reversal write-trains are the known encoder-wedge trigger). Riding
    // the existing profile is the library-intended stop for this case.
    bool promptHalt = false;
    // Dead-time-projected firing for the encoder stops (the 093 spike's
    // "delay in the plan", applied to the terminal edge): the plant lags the
    // plan by kDeadTime, so a raw-threshold stop is systematically reached
    // ~plannedSpeed*kDeadTime SHORT of target -- the old behavior was to let
    // the plan exhaust undershot and have the divergence replan launch a
    // fresh (decaying) trapezoid, over and over: the multi-hump pivot
    // defect. Instead, fire once the MEASURED remaining is within what the
    // plant covers during the actuation lag (remaining <= plannedSpeed *
    // kDeadTime) -- the in-flight motion carries it home.
    for (uint8_t i = 0; i < stopsCount_ && !fired; ++i) {
      const msg::StopCondition& cond = stops_[i];
      if (cond.kind == msg::StopKind::STOP_DISTANCE ||
          cond.kind == msg::StopKind::STOP_ROTATION) {
        float remainingNative = 0.0f;   // [mm] (per-wheel arc mm for ROTATION)
        Motion::StopEvalResult r = Motion::remainingToStop(
            cond, baseline_, encLeft, encRight, msg::PoseEstimate{}, &remainingNative);
        if (r == Motion::StopEvalResult::FIRED) {
          fired = true;
        } else if (r == Motion::StopEvalResult::NOT_FIRED) {
          float plannedSpeedNative =   // [mm/s] plan speed in the stop's own native units
              (cond.kind == msg::StopKind::STOP_DISTANCE)
                  ? fabsf(linear_.sample(linearElapsed(now)).velocity)
                  : fabsf(rotational_.sample(rotationalElapsed(now)).velocity) * arcScale_;
          if (remainingNative <= plannedSpeedNative * kDeadTime) fired = true;
        }
        // UNSUPPORTED falls through as NOT_FIRED (defensive; these two kinds
        // are always supported by remainingToStop()).
      } else if (Motion::evaluateStopCondition(cond, baseline_, now, encLeft, encRight,
                                               msg::PoseEstimate{}) ==
                 Motion::StopEvalResult::FIRED) {
        fired = true;
        promptHalt = true;
      }
    }
    // Plan exhaustion also completes the phase: if the profile has fully
    // played out (plus one dead-time of grace for the plant to finish
    // arriving) and the encoder stop STILL hasn't fired, the residual is
    // terminal undershoot -- the exact thing the old code hunted after with
    // fresh decaying re-solves (and, failing that, sat out the STOP_TIME
    // net, stalling multi-segment sequences ~2.5s per pivot). Accept it and
    // complete; residual accuracy is calibration work, not something a
    // second trapezoid can fix.
    if (!fired) {
      if (phase_ == Phase::BLEND) {
        bool linDone = linear_.duration() <= 0.0f ||
                       linearElapsed(now) >= linear_.duration() + kDeadTime;
        bool rotDone = rotational_.duration() <= 0.0f ||
                       rotationalElapsed(now) >= rotational_.duration() + kDeadTime;
        fired = linDone && rotDone;
      } else if (phase_ == Phase::TRANSLATE) {
        float elapsed = linearElapsed(now);
        float duration = linear_.duration();
        if (duration > 0.0f && elapsed >= duration + kDeadTime) fired = true;
      } else {
        // PRE_PIVOT / TERMINAL_PIVOT (M4): tolerance+dwell completion gate
        // -- replaces the dead-time-projected STOP_ROTATION firing + plan-
        // exhaustion fallback this branch used for these two phases before
        // this ticket (architecture-update.md M4). STOP_TIME (evaluated
        // above, in the stops_[] loop) remains the ONLY other completion
        // path for these two phases -- the independent stall/non-
        // convergence backstop this gate does not replace. thetaMeasured/
        // omegaMeasured were computed by the pivot v/omega branch above,
        // earlier this SAME tick() call.
        bool withinTol = fabsf(rotationalTarget_ - thetaMeasured) < kHeadingTol &&
                         fabsf(omegaMeasured) < kHeadingRateTol;
        if (withinTol) {
          if (!headingDwellActive_) {
            headingDwellActive_ = true;
            headingDwellStartMs_ = now;
          }
          if (static_cast<int32_t>(now - headingDwellStartMs_) >=
              static_cast<int32_t>(kHeadingDwellMs)) {
            fired = true;
          }
        } else {
          headingDwellActive_ = false;
        }
      }
    }
    if (fired) {
      // (Streaming note: a pending stream segment never reaches this point --
      // it merges at the TOP of tick(), mid-plan. By fire time the queue was
      // dry, so the graceful stop below IS the stream's graceful stop.)
      stopping_ = true;
      softDeadline_ = now + kSoftDeadlineMs;
      if (promptHalt) {
        // Sensor/time stop: can fire mid-cruise with arbitrary plan travel
        // left -- a freshly-armed velocity-mode decel ramp is the right
        // (and Ruckig-documented) stop for this genuinely preemptive case.
        if (phase_ == Phase::TRANSLATE) {
          armTranslateStopDecel(now);
        } else if (phase_ == Phase::BLEND) {
          armTranslateStopDecel(now);
          armPivotStopDecel(now);
        } else {
          armPivotStopDecel(now);
        }
      }
      // else: position stop / exhaustion -- ride the plan's own to-rest
      // tail (see promptHalt's declaration comment). The stopping_
      // convergence below keys on the SAME (unchanged) plan durations.
    }
  }

  twist.v_x = v;
  twist.omega = omega;
  return twist;
}

bool SegmentExecutor::active() const { return phase_ != Phase::IDLE; }

float SegmentExecutor::remainingLinear(uint32_t now) const {
  if (phase_ == Phase::IDLE || velocityMode_) return 0.0f;   // no position target
  // Plan-frame remaining translation. Const-cast: JerkTrajectory::sample()
  // is logically const (pure evaluation at an elapsed time) but not marked
  // so; this accessor must stay const for Drivetrain::state().
  Motion::JerkTrajectory& lin = const_cast<Motion::JerkTrajectory&>(linear_);
  return fabsf(linearTarget_ - lin.sample(linearElapsed(now)).position);
}

}  // namespace Motion
