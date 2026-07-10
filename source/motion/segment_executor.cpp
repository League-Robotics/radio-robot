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
constexpr float kOutputHops = 2.0f;  // sim: no flip-flop, no transport lag
#else
constexpr float kOutputHops = 4.0f;  // real brick: measured ~80 ms flip-flop actuation dead-time
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
  if (needTranslate_) linear_.solveToRest(translateTarget_, linearCeiling_);
  if (needTerminalPivot_) rotational_.solveToRest(terminalPivotTarget_, rotationalCeiling_);
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
                                   const msg::MotorState& encRight) {
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
  // with no plan yet this life (duration 0) solves fresh from rest.
  if (linear_.duration() > 0.0f) {
    linear_.retarget(translateTarget_);
  } else if (needTranslate_) {
    linear_.reset();
    linear_.solveToRest(translateTarget_, linearCeiling_);
  }
  linearTarget_ = translateTarget_;
  linearSolveMs_ = now;

  if (rotational_.duration() > 0.0f) {
    rotational_.retarget(terminalPivotTarget_);
  } else if (needTerminalPivot_) {
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
  captureBaseline(now, encLeft, encRight);
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
  float arc = fabsf(preRotateTarget_) * arcScale_;  // [mm]
  appendStop(msg::StopKind::STOP_ROTATION, arc);
  float omegaMag = fabsf(rotationalCeiling_);
  float nominal = (omegaMag > 1e-3f) ? (fabsf(preRotateTarget_) / omegaMag) * 1000.0f : 0.0f;  // [ms]
  appendStop(msg::StopKind::STOP_TIME, nominal * 2.0f + 2000.0f);

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
  float arc = fabsf(terminalPivotTarget_) * arcScale_;  // [mm]
  appendStop(msg::StopKind::STOP_ROTATION, arc);
  float omegaMag = fabsf(rotationalCeiling_);
  float nominal =
      (omegaMag > 1e-3f) ? (fabsf(terminalPivotTarget_) / omegaMag) * 1000.0f : 0.0f;  // [ms]
  appendStop(msg::StopKind::STOP_TIME, nominal * 2.0f + 2000.0f);

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
                                      const msg::MotorState& encRight) {
  baseline_.t0 = now;
  baseline_.enc0 = 0.0f;
  baseline_.encDiff0 = 0.0f;
  if (encLeft.position.has && encRight.position.has) {
    baseline_.enc0 = (encLeft.position.val + encRight.position.val) * 0.5f;
    baseline_.encDiff0 = encRight.position.val - encLeft.position.val;
  }
  // Pose-free: heading0/pose0X/pose0Y are dead fields, left at their default
  // 0 (see class comment) -- only enc0/encDiff0/vSign/omegaSign matter here.
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

  Motion::JerkTrajectory::State state = linear_.sample(linearElapsed(now));
  float planRemainingMag = fabsf(linearTarget_ - state.position);

  // Model the actuation dead-time INTO the comparison: the plant
  // legitimately lags the plan by kDeadTime, so the measured remaining is
  // EXPECTED to read planRemaining + planSpeed*kDeadTime. Only divergence
  // BEYOND that modeled lag is real (093's "delay in the plan", applied to
  // the replan trigger) -- without this, the tail of every solve reads as
  // divergent and re-solves a fresh decaying trapezoid: the multi-hump
  // terminal defect.
  float expectedRemainingMag =
      planRemainingMag + fabsf(state.velocity) * kDeadTime;
  float divergence = fabsf(expectedRemainingMag - measuredRemainingMag);
  if (divergence < kDivergenceThreshold) return;  // within tolerance -- no replan

  float planSpeedMag = fabsf(state.velocity);
  float projectedRemainingMag = measuredRemainingMag - planSpeedMag * kDeadTime;
  if (projectedRemainingMag <= 0.0f) return;  // never solve backward

  if (divergence >= kGrossDivergenceThreshold) {
    float measuredPositionSigned = linearTarget_ - vSign * measuredRemainingMag;
    float measuredVelocitySigned = 0.0f;
    if (encLeft.velocity.has && encRight.velocity.has) {
      measuredVelocitySigned = (encLeft.velocity.val + encRight.velocity.val) * 0.5f;
    }
    linear_.reanchor(measuredPositionSigned, measuredVelocitySigned);
  } else {
    float newRemainingSigned = vSign * projectedRemainingMag;
    linear_.retarget(newRemainingSigned);
    linearTarget_ = newRemainingSigned;
  }
  linearSolveMs_ = now;
  lastReplanMs_ = now;
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

  const msg::StopCondition* rotCond = nullptr;
  for (uint8_t i = 0; i < stopsCount_; ++i) {
    if (stops_[i].kind == msg::StopKind::STOP_ROTATION) {
      rotCond = &stops_[i];
      break;
    }
  }
  if (rotCond == nullptr) return;  // defensive -- beginPrePivot()/beginTerminalPivot() always
                                    // append one

  float omegaSign = baseline_.omegaSign;
  if (omegaSign == 0.0f) return;  // no meaningful direction to replan in

  float measuredRemainingNative = 0.0f;
  Motion::StopEvalResult r = Motion::remainingToStop(
      *rotCond, baseline_, encLeft, encRight, msg::PoseEstimate{}, &measuredRemainingNative);
  if (r != Motion::StopEvalResult::NOT_FIRED) return;  // FIRED: caller's own stop-eval loop owns
                                                       // completion, not a replan

  float measuredRemainingRad = measuredRemainingNative / arcScale_;

  Motion::JerkTrajectory::State state = rotational_.sample(rotationalElapsed(now));
  float planRemainingRad = fabsf(rotationalTarget_ - state.position);

  // Dead-time lag modeled into the comparison -- see maybeReplanTranslate()'s
  // matching block for the full rationale.
  float expectedRemainingRad =
      planRemainingRad + fabsf(state.velocity) * kDeadTime;
  float divergence = fabsf(expectedRemainingRad - measuredRemainingRad);
  if (divergence < kRotDivergenceThreshold) return;  // within tolerance -- no replan

  float planSpeedMagRad = fabsf(state.velocity);
  float projectedRemainingRad = measuredRemainingRad - planSpeedMagRad * kDeadTime;
  if (projectedRemainingRad <= 0.0f) return;  // never solve backward

  if (divergence >= kRotGrossDivergenceThreshold) {
    float measuredPositionSigned = rotationalTarget_ - omegaSign * measuredRemainingRad;
    rotational_.reanchor(measuredPositionSigned, 0.0f);  // no reliable measured angular rate
  } else {
    float newRemainingSigned = omegaSign * projectedRemainingRad;
    rotational_.retarget(newRemainingSigned);
    rotationalTarget_ = newRemainingSigned;
  }
  rotationalSolveMs_ = now;
  lastReplanMs_ = now;
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
                                      const msg::MotorState& encRight) {
  msg::BodyTwist3 twist{};
  if (phase_ == Phase::IDLE) return twist;  // never started, or fully converged

  if (!baselineCaptured_) {
    captureBaseline(now, encLeft, encRight);
    baselineCaptured_ = true;
  }

  // Streaming merge: consume the pending segment IMMEDIATELY (mid-plan) --
  // remaining targets accumulate and the channels retarget() from their
  // current moving state. Merging must not wait for the stop to fire: each
  // plan is solved to-rest, so a fire-time chain would re-launch from ~zero
  // velocity every segment and the stream would crawl.
  if (!stopping_ && hasPending_ && currentStream_) {
    mergePending(now, encLeft, encRight);
  }

  float v = 0.0f;
  float omega = 0.0f;
  bool rotElapsedPastDuration = false;

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
    if (!stopping_) maybeReplanPivot(now, encLeft, encRight);
    float rotElapsed = rotationalElapsed(now);
    omega = rotational_.sample(rotElapsed).velocity;
    rotElapsedPastDuration = rotElapsed >= rotational_.duration();
    // Snap to a LITERAL 0.0f once the STOP-TRIGGERED decel-to-zero has fully
    // converged (only while stopping_) -- ported VERBATIM rationale from
    // Subsystems::Planner::tick() (planner.cpp:964-966): Ruckig's own
    // past-duration "hold at final state" is not guaranteed bit-exact the
    // way Motion::VelocityRamp's linear `cur + (tgt - cur)` approach was, and
    // a ~1e-15-scale residual defeats Hal::MotorVelocityPid's zero-deadband
    // (spAbs <= minDuty) -- the integrator-freeze fix never engages for a
    // target that never reaches a literal 0.0f, producing a sustained,
    // slowly-decaying reverse-spin residual. NOT applied to the ongoing
    // (not-yet-stopping_) position-control solve, where the divergence
    // replan is supposed to keep re-extending the plan against a lagging
    // real plant -- forcing a hard 0 there would fight it.
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
        captureBaseline(now, encLeft, encRight);
        baselineCaptured_ = true;
      }
    }
  } else {
    bool fired = false;
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
      } else {
        float elapsed = (phase_ == Phase::TRANSLATE) ? linearElapsed(now)
                                                      : rotationalElapsed(now);
        float duration = (phase_ == Phase::TRANSLATE) ? linear_.duration()
                                                       : rotational_.duration();
        if (duration > 0.0f && elapsed >= duration + kDeadTime) fired = true;
      }
    }
    if (fired) {
      // (Streaming note: a pending stream segment never reaches this point --
      // it merges at the TOP of tick(), mid-plan. By fire time the queue was
      // dry, so the decel below IS the stream's graceful stop.)
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

  twist.v_x = v;
  twist.omega = omega;
  return twist;
}

bool SegmentExecutor::active() const { return phase_ != Phase::IDLE; }

float SegmentExecutor::remainingLinear(uint32_t now) const {
  if (phase_ == Phase::IDLE) return 0.0f;
  // Plan-frame remaining translation. Const-cast: JerkTrajectory::sample()
  // is logically const (pure evaluation at an elapsed time) but not marked
  // so; this accessor must stay const for Drivetrain::state().
  Motion::JerkTrajectory& lin = const_cast<Motion::JerkTrajectory&>(linear_);
  return fabsf(linearTarget_ - lin.sample(linearElapsed(now)).position);
}

}  // namespace Motion
