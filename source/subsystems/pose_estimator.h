// pose_estimator.h — Subsystems::PoseEstimator: encoder dead-reckoning +
// OTOS (EkfTiny) fusion, a Subsystems-tier peer of Subsystems::Drivetrain.
//
// Deliberately NOT folded into Drivetrain (architecture-update.md Decision 1,
// sprint 082, ticket 002): control-law tuning (Drivetrain's ratio governor)
// and sensor-fusion-noise tuning (this class's EKF) change for different
// reasons — a cohesion decision, not an oversight of the fact that
// msg::DrivetrainState/DrivetrainConfig already scaffold pose/EKF fields.
//
// Owns one EkfTiny (sprint 082, ticket 001 — source/estimation/ekf_tiny.h)
// plus its own encoder-only dead-reckoning accumulator (arc-segment
// integration, ported in concept from source_old/control/Odometry.cpp's
// encoder half — see pose_estimator.cpp's tick() for exactly which lines
// correspond). Exposes two independent readings:
//   - encoderPose() — pure dead-reckoning from wheel encoder deltas alone.
//     The EKF never writes here, ever.
//   - fusedPose() — the EKF's belief: predicted every tick from the same
//     encoder deltas, corrected by the odometer's reading when one is
//     present and fresh (stamp.valid).
//
// Like Drivetrain, PoseEstimator holds NO Hal::Motor/Hal::Odometer
// reference or pointer: tick() takes this tick's observations as arguments
// only (msg::MotorState for each wheel, a nullable msg::PoseEstimate for the
// odometer) — see drivetrain.h's class comment for the same discipline.
//
// Uses only msg:: pose types (source/messages/common.h) — never the
// parallel, unit-suffixed Pose2D/BodyTwist3 family that used to live at
// source/kinematics/pose2d.h; that file was deleted pre-082 (commit
// f5fd7dde) and must not be recreated.
#pragma once

#include <stdint.h>

#include "estimation/ekf_tiny.h"
#include "messages/drivetrain.h"
#include "messages/motor.h"

namespace Subsystems {

class PoseEstimator {
 public:
  // configure — reads trackwidth, rotational_slip, and the four EKF noise
  // fields (ekf_q_xy, ekf_q_theta, ekf_r_otos_xy, ekf_r_otos_theta) from the
  // SAME msg::DrivetrainConfig type Drivetrain::configure() already takes
  // (no new config message, no proto change).
  //
  // Zero-as-unset sentinel (mirrors the ported Odometry source's
  // effectiveSlip() pattern — see pose_estimator.cpp): a noise field arriving
  // as exactly 0.0f (the proto zero-default, meaning "never configured") is
  // substituted with a small, hardcoded, documented fallback before being
  // passed to EkfTiny::init(). A non-zero configured value passes through
  // unchanged.
  //
  // Calls EkfTiny::init() — per that method's own doc comment, this is
  // BOOT-ONLY (also resets EKF state/covariance to zero); matches
  // Drivetrain::configure()'s own no-nuance direct-copy-in precedent.
  void configure(const msg::DrivetrainConfig& config);

  // tick — advance both readings by one control-loop tick.
  //   now      — [ms] robot system clock; used only for stamping outputs and
  //              computing dt for the EKF (no clock is read internally).
  //   leftObs/rightObs — this tick's sampled MotorState for the two wheels
  //              this estimator tracks (the SAME per-wheel observation shape
  //              Drivetrain::tick() already takes) — arguments only, never
  //              stored, never read from a Motor reference.
  //   otosObs  — this tick's odometer reading, or nullptr if none is
  //              available. Only consumed when non-null AND
  //              otosObs->stamp.valid is true.
  //
  // Sequencing (see pose_estimator.cpp for the full rationale):
  //   1. If leftObs.position or rightObs.position lacks .has, this tick's
  //      update is skipped entirely — no encoder-accumulator advance, no EKF
  //      predict, no stale-data corruption. The previous-encoder baseline
  //      and last-tick timestamp are left untouched so the next valid tick's
  //      delta/dt span exactly the gap.
  //   2. Otherwise: compute the encoder delta, midpoint-arc-integrate it into
  //      the encoder-only accumulator (encoderPose()'s backing state).
  //   3. EkfTiny::predict() runs unconditionally (dead-reckoning always
  //      advances, whether or not an odometer is present).
  //   4. EkfTiny::updatePosition()/updateHeading() run ONLY when otosObs is
  //      non-null and fresh (stamp.valid).
  void tick(uint32_t now, const msg::MotorState& leftObs,
            const msg::MotorState& rightObs,
            const msg::PoseEstimate* otosObs);

  // encoderPose — pure dead-reckoning pose (x, y, heading) from wheel
  // encoder deltas only. The EKF never writes here, ever. twist is left at
  // its zero default — this ticket computes no encoder-rate velocity (out
  // of scope; see ekf_tiny.h's file header on why velocity states were
  // dropped from EkfTiny entirely).
  msg::PoseEstimate encoderPose() const;

  // fusedPose — the EKF's current belief (x, y, heading), advanced by
  // predict() every tick and corrected by updatePosition()/updateHeading()
  // whenever a fresh odometer reading was present. twist left at its zero
  // default (same rationale as encoderPose()).
  msg::PoseEstimate fusedPose() const;

 private:
  // sentinelOr — zero-as-unset substitution: returns fallback when
  // configured is exactly 0.0f, otherwise returns configured unchanged.
  // Mirrors the ported Odometry source's effectiveSlip() pattern (see
  // source_old/control/Odometry.h) applied to the four EKF noise fields
  // instead of rotational_slip.
  static float sentinelOr(float configured, float fallback);

  // Wrap heading to (-pi, pi] using the atan2f identity — same identity
  // EkfTiny itself uses (ekf_tiny.cpp's own wrapPi()), kept as an
  // independent copy here since the encoder-only accumulator never calls
  // into EkfTiny at all.
  static float wrapPi(float theta);

  // effectiveSlip — migration-safe rotationalSlip clamp, ported verbatim
  // (semantics, not textually — this is a private static method rather than
  // a free inline function) from source_old/control/Odometry.h's own
  // effectiveSlip(): 0 or negative -> 1.0 (no correction; legacy
  // config-safe), (0, 0.5) -> 0.5 (clamp floor), [0.5, 1.0] -> pass-through,
  // > 1.0 -> 1.0 (clamp ceiling).
  static float effectiveSlip(float rawSlip);

  EkfTiny ekf_;

  // Kinematics config, set by configure() and read by tick(). Defaults
  // (128mm / 0-unset) mirror the ported Odometry source's own defaults and
  // are never actually exercised in production (the wiring layer always
  // calls configure() before the first tick()); they exist only so a
  // construction-time tick() call would not divide by zero.
  float trackwidth_ = 128.0f;      // [mm]
  float rotationalSlip_ = 0.0f;    // 0 = unset -> effectiveSlip() returns 1.0

  // Previous-encoder baseline — intermediate compute state for this tick's
  // delta, analogous to the ported Odometry source's _prevEncL/_prevEncR.
  // haveEncBaseline_ guards the very first valid tick (no prior reading to
  // diff against yet): that tick's delta is treated as zero motion instead
  // of diffing against an arbitrary uninitialized 0.0f baseline, which would
  // otherwise fabricate a phantom jump whenever encoder positions do not
  // themselves start at exactly zero.
  bool haveEncBaseline_ = false;
  float prevEncLeft_ = 0.0f;    // [mm]
  float prevEncRight_ = 0.0f;   // [mm]

  // Encoder-only dead-reckoning accumulator (this class's own state — the
  // EKF never writes here). Backs encoderPose().
  float encX_ = 0.0f;       // [mm]
  float encY_ = 0.0f;       // [mm]
  float encTheta_ = 0.0f;   // [rad]

  // dt tracking for the EKF's predict() timestep. haveLastTick_ guards the
  // very first valid tick (no prior timestamp to diff against yet).
  bool haveLastTick_ = false;
  uint32_t lastTick_ = 0;   // [ms] timestamp of the last valid tick

  // EKF noise-fallback constants for configure()'s zero-as-unset sentinel.
  // Provenance: source_old/robot/DefaultConfig.cpp lines 57-68
  // (p.ekfQxy/ekfQtheta/ekfROtosXy/ekfROtosTheta) — the pre-082 firmware's
  // own production defaults for the (dropped, 5-state) EKFTiny predecessor.
  // Reused here as a reasonable, documented starting point for this trimmed
  // 3-state filter, NOT a value re-tuned for it (per the ticket's own
  // "reasonable starting point, not a tuned value" instruction).
  static constexpr float kDefaultQXy = 800.0f;          // [mm^2]
  static constexpr float kDefaultQTheta = 4.0f;         // [rad^2]
  static constexpr float kDefaultROtosXy = 50.0f;       // [mm^2]
  static constexpr float kDefaultROtosTheta = 0.01f;    // [rad^2] ~(5.7 deg)^2
};

}  // namespace Subsystems
