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

void SegmentExecutor::beginPrePivot(uint32_t now) {
  rotational_.reset();
  rotational_.solveToRest(preRotateTarget_, rotationalCeiling_);
  rotationalTarget_ = preRotateTarget_;
  rotationalSolveMs_ = now;
  lastReplanMs_ = now;

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
    case Phase::IDLE:
    default:
      phase_ = Phase::IDLE;
      return;
  }
}

void SegmentExecutor::stop(uint32_t now) {
  if (phase_ == Phase::IDLE) return;
  forceStopArmed_ = true;
  if (!stopping_) {
    stopping_ = true;
    softDeadline_ = now + kSoftDeadlineMs;
    if (phase_ == Phase::TRANSLATE) {
      armTranslateStopDecel(now);
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

  float divergence = fabsf(planRemainingMag - measuredRemainingMag);
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

  float divergence = fabsf(planRemainingRad - measuredRemainingRad);
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

  float v = 0.0f;
  float omega = 0.0f;
  bool rotElapsedPastDuration = false;

  if (phase_ == Phase::TRANSLATE) {
    if (!stopping_) maybeReplanTranslate(now, encLeft, encRight);
    v = linear_.sample(linearElapsed(now)).velocity;
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
    bool converged = (phase_ == Phase::TRANSLATE) ? (linearElapsed(now) >= linear_.duration())
                                                   : rotElapsedPastDuration;
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
    for (uint8_t i = 0; i < stopsCount_; ++i) {
      Motion::StopEvalResult r = Motion::evaluateStopCondition(stops_[i], baseline_, now, encLeft,
                                                               encRight, msg::PoseEstimate{});
      if (r == Motion::StopEvalResult::FIRED) {
        fired = true;
        break;
      }
      // UNSUPPORTED is treated identically to NOT_FIRED (mirrors Planner's
      // own stop-eval loop; this executor only ever appends STOP_ROTATION/
      // STOP_DISTANCE/STOP_TIME, which are always SUPPORTED, but the
      // defensive handling costs nothing).
    }
    if (fired) {
      stopping_ = true;
      softDeadline_ = now + kSoftDeadlineMs;
      if (phase_ == Phase::TRANSLATE) {
        armTranslateStopDecel(now);
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

}  // namespace Motion
