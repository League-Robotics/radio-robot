// pose_estimator.cpp — Subsystems::PoseEstimator implementation. See
// pose_estimator.h for the class-level design notes.
#include "subsystems/pose_estimator.h"

#include <math.h>

namespace Subsystems {

float PoseEstimator::sentinelOr(float configured, float fallback) {
  return (configured == 0.0f) ? fallback : configured;
}

float PoseEstimator::wrapPi(float theta) {
  return atan2f(sinf(theta), cosf(theta));
}

float PoseEstimator::effectiveSlip(float rawSlip) {
  if (rawSlip <= 0.0f) return 1.0f;
  if (rawSlip < 0.5f) return 0.5f;
  if (rawSlip > 1.0f) return 1.0f;
  return rawSlip;
}

void PoseEstimator::configure(const msg::DrivetrainConfig& config) {
  config_ = config;   // verbatim round-trip copy (087-004) -- see config()
  trackwidth_ = config.trackwidth;
  rotationalSlip_ = config.rotational_slip;

  // Zero-as-unset sentinel (see the class comment and each kDefault*
  // constant's provenance note in pose_estimator.h): proto3 floats default
  // to 0.0f, which SET/GET code cannot distinguish from "the stakeholder
  // explicitly configured zero" -- exactly 0.0f means "never configured" for
  // these four EKF noise fields specifically (mirrors effectiveSlip()'s own
  // zero-as-unset treatment of rotational_slip, immediately above). A
  // non-zero configured value always passes through unchanged.
  float qXy = sentinelOr(config.ekf_q_xy, kDefaultQXy);
  float qTheta = sentinelOr(config.ekf_q_theta, kDefaultQTheta);
  float rOtosXy = sentinelOr(config.ekf_r_otos_xy, kDefaultROtosXy);
  float rOtosTheta = sentinelOr(config.ekf_r_otos_theta, kDefaultROtosTheta);

  ekf_.init(qXy, qTheta, rOtosXy, rOtosTheta);
}

void PoseEstimator::tick(uint32_t now, const msg::MotorState& leftObs,
                          const msg::MotorState& rightObs,
                          const msg::PoseEstimate* otosObs,
                          Rt::WorkQueue<Rt::PoseResetCommand, 4>& poseResetIn,
                          Rt::Mailbox<msg::SetPose>& otosSetPoseOut) {
  // 099-004: lastPoseStep_ reflects only the IMMEDIATELY-prior tick()'s
  // correction -- reset to {0, 0} at the very top of every call, before the
  // drain below has a chance to (re)populate it.
  lastPoseStep_ = msg::PoseStep();

  // 087-004: drain poseResetIn completely, FIFO, BEFORE anything else --
  // even before the "no observation this pass" early return below, so a
  // queued SI/ZERO reset is never skipped just because this pass's encoder
  // observations happen to be momentarily absent. Pure routing: neither
  // setPose() nor resetEncoderBaseline()'s own internals (the phantom-jump-
  // avoidance mechanism) change here.
  while (!poseResetIn.empty()) {
    Rt::PoseResetCommand cmd = poseResetIn.take();
    switch (cmd.kind) {
      case Rt::PoseResetCommand::kSetPose: {
        // 099-004: capture fusedPose() before/after setPose() so
        // lastPoseStep_ reports the magnitude of THIS correction, then post
        // the re-anchored fusedPose() to otosSetPoseOut -- MainLoop drains
        // it into hardware_.odometer()->applySetPose(...) the same way it
        // always has (architecture-update.md D1/D8).
        msg::PoseEstimate before = fusedPose();
        setPose(cmd.pose);
        msg::PoseEstimate after = fusedPose();

        float dx = after.pose.x - before.pose.x;
        float dy = after.pose.y - before.pose.y;
        lastPoseStep_.pos = sqrtf(dx * dx + dy * dy);
        lastPoseStep_.theta = fabsf(wrapPi(after.pose.h - before.pose.h));

        msg::SetPose fixedPose;
        fixedPose.x = after.pose.x;
        fixedPose.y = after.pose.y;
        fixedPose.h = after.pose.h;
        otosSetPoseOut.post(fixedPose);
        break;
      }
      case Rt::PoseResetCommand::kResetBaseline:
        resetEncoderBaseline();
        break;
    }
  }

  // Encoder delta requires BOTH wheels' position observation this tick. If
  // either is absent, skip this tick's update entirely -- no encoder-
  // accumulator advance, no EKF predict, no stale-data corruption. Leave
  // the previous-encoder baseline and last-tick timestamp untouched so the
  // next valid tick's delta/dt span exactly the gap left by this one.
  if (!leftObs.position.has || !rightObs.position.has) {
    return;
  }

  float left = leftObs.position.val;
  float right = rightObs.position.val;

  // dt for the EKF predict step -- signed cast avoids uint32 underflow on
  // rollover (see the watchdog-uint32-underflow project finding: never
  // plain-subtract two uint32 ms stamps without a signed cast). Zero on the
  // very first valid tick (no prior timestamp yet).
  float dt = haveLastTick_ ? static_cast<int32_t>(now - lastTick_) * 0.001f
                           : 0.0f;

  // 084-007 (SUC-006): apply a pending resetEncoderBaseline() request only
  // on a GENUINELY time-advancing tick (dt > 0) -- see that method's own
  // doc comment (pose_estimator.h) for why a dt == 0 tick (this same
  // command's own dispatch pass, or any further synchronous command
  // dispatched before the next real tick) must NOT consume this one-shot
  // guard: the staged hardware encoder reset (Hal::Motor::resetPosition())
  // may not have landed yet, so left/right here could still be the STALE
  // pre-reset reading.
  if (encBaselineResetPending_ && dt > 0.0f) {
    haveEncBaseline_ = false;
    encBaselineResetPending_ = false;
  }

  // Encoder delta against the previous-encoder baseline. Zero on the very
  // first valid tick (no prior baseline yet) -- see haveEncBaseline_'s doc
  // comment in the header for why this is a "no motion yet" default rather
  // than a diff against an arbitrary 0.0f.
  float dL = haveEncBaseline_ ? (left - prevEncLeft_) : 0.0f;
  float dR = haveEncBaseline_ ? (right - prevEncRight_) : 0.0f;
  prevEncLeft_ = left;
  prevEncRight_ = right;
  haveEncBaseline_ = true;

  // Midpoint (exact-arc) integration -- matches the ported Odometry
  // source's predict() math (source_old/control/Odometry.cpp):
  //   dCenter = (dL + dR) / 2
  //   dTheta  = ((dR - dL) / trackwidth) * effectiveSlip(rotationalSlip)
  float dCenter = (dL + dR) * 0.5f;
  float slip = effectiveSlip(rotationalSlip_);
  float dTheta = ((dR - dL) / trackwidth_) * slip;

  // Encoder-only dead-reckoning accumulate -- this is encoderPose()'s
  // entire backing state; the EKF never writes here.
  float encThetaMid = encTheta_ + dTheta * 0.5f;
  encX_ += dCenter * cosf(encThetaMid);
  encY_ += dCenter * sinf(encThetaMid);
  encTheta_ = wrapPi(encTheta_ + dTheta);

  // EKF predict -- runs every tick unconditionally (dead-reckoning always
  // advances, whether or not an odometer is present). thetaBefore is the
  // EKF's OWN previous heading, read before predict() mutates it -- per
  // ekf_tiny.h's documented caller contract. Note this is deliberately NOT
  // encTheta_: the encoder-only accumulator and the EKF's belief start
  // identical and take identical (dCenter, dTheta) inputs every tick, but
  // are two independent pieces of state so that an EKF correction (below)
  // can make them diverge without touching the pure dead-reckoning value.
  float thetaBeforeEkf = ekf_.theta();
  ekf_.predict(dCenter, dTheta, thetaBeforeEkf, dt);

  // EKF correct -- only when a fresh odometer reading is present.
  if (otosObs != nullptr && otosObs->stamp.valid) {
    ekf_.updatePosition(otosObs->pose.x, otosObs->pose.y);
    ekf_.updateHeading(otosObs->pose.h);
  }

  lastTick_ = now;
  haveLastTick_ = true;
}

msg::PoseEstimate PoseEstimator::encoderPose() const {
  msg::PoseEstimate result;
  result.pose.x = encX_;
  result.pose.y = encY_;
  result.pose.h = encTheta_;
  // twist left at its zero default -- see the header's doc comment.
  result.stamp.valid = haveLastTick_;
  result.stamp.last_upd = lastTick_;
  return result;
}

msg::PoseEstimate PoseEstimator::fusedPose() const {
  msg::PoseEstimate result;
  result.pose.x = ekf_.x();
  result.pose.y = ekf_.y();
  result.pose.h = ekf_.theta();
  // twist left at its zero default -- see the header's doc comment.
  result.stamp.valid = haveLastTick_;
  result.stamp.last_upd = lastTick_;
  return result;
}

void PoseEstimator::setPose(const msg::SetPose& pose) {
  // pose.h arrives already in radians (the caller -- SI's handler -- did
  // the wire cdeg->rad conversion) -- see this method's header doc comment.
  float theta = wrapPi(pose.h);
  encX_ = pose.x;
  encY_ = pose.y;
  encTheta_ = theta;
  // EkfTiny::setPose() (082-001) overwrites state with a sane diagonal
  // P-prior instead of zeroing P -- see ekf_tiny.h's own doc comment.
  ekf_.setPose(pose.x, pose.y, theta);
}

void PoseEstimator::resetEncoderBaseline() {
  // Deferred to the next genuinely time-advancing tick() -- see this
  // method's own doc comment (pose_estimator.h) and tick()'s matching
  // dt > 0 gate above.
  encBaselineResetPending_ = true;
}

}  // namespace Subsystems
