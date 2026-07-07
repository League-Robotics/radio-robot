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
#include "runtime/commands.h"
#include "runtime/queue.h"

namespace Subsystems {

class PoseEstimator {
 public:
  // configure — reads trackwidth, rotational_slip, and the four EKF noise
  // fields (ekf_q_xy, ekf_q_theta, ekf_r_otos_xy, ekf_r_otos_theta) from the
  // SAME msg::DrivetrainConfig type Drivetrain::configure() already takes
  // (no new config message, no proto change). The full incoming config is
  // ALSO stored verbatim (config_) so config() (below) can round-trip it —
  // mirrors Subsystems::Drivetrain's own config_ member (087-004).
  //
  // Zero-as-unset sentinel (mirrors the ported Odometry source's
  // effectiveSlip() pattern — see pose_estimator.cpp): a noise field arriving
  // as exactly 0.0f (the proto zero-default, meaning "never configured") is
  // substituted with a small, hardcoded, documented fallback before being
  // passed to EkfTiny::init(). A non-zero configured value passes through
  // unchanged. This substitution affects only the EKF's own init() call —
  // config()'s returned value is the RAW config as given to configure(),
  // unmodified.
  //
  // Calls EkfTiny::init() — per that method's own doc comment, this is
  // BOOT-ONLY (also resets EKF state/covariance to zero); matches
  // Drivetrain::configure()'s own no-nuance direct-copy-in precedent.
  void configure(const msg::DrivetrainConfig& config);

  // config — the current config, as last passed to configure(), verbatim
  // (087-004, architecture-update-r1.md Step 3: "PoseEstimator (existing,
  // gains config()/reset-queue drain)"). Kills the config-shadow this
  // sprint's design removes elsewhere — a caller (the Configurator, ticket
  // 005) can read back what was configured without a separate cache.
  msg::DrivetrainConfig config() const { return config_; }

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
  //   poseResetIn — (087-004) the blackboard-sourced target-drained reset
  //              queue (Rt::WorkQueue<Rt::PoseResetCommand,4>, source/
  //              runtime/commands.h — Decision 7: SI/ZERO resets stay
  //              owned by THIS class rather than externally applied by the
  //              Configurator). Drained FIFO, ALL entries, every tick() call
  //              — BEFORE the sequencing below, so a queued reset is never
  //              skipped just because this pass's observations happen to be
  //              absent (see step 1). kSetPose dispatches to the existing
  //              setPose(); kResetBaseline dispatches to the existing
  //              resetEncoderBaseline() — neither method's own internals
  //              change; this is pure routing. An empty poseResetIn is a
  //              no-op, matching today's behavior when no SI/ZERO command is
  //              in flight this pass (the wire-level routing of SI/ZERO INTO
  //              this queue is ticket 006's job, out of this ticket's scope
  //              — today's handlers still call setPose()/
  //              resetEncoderBaseline() directly).
  //
  // Sequencing (see pose_estimator.cpp for the full rationale):
  //   0. Drain poseResetIn completely (see above).
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
            const msg::PoseEstimate* otosObs,
            Rt::WorkQueue<Rt::PoseResetCommand, 4>& poseResetIn);

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

  // setPose -- 084-007 (SUC-006): re-anchor BOTH encoderPose() and
  // fusedPose() to the given world pose (pose.x, pose.y, pose.h). `SI`'s
  // handler (source/commands/pose_commands.cpp) is this method's one wire
  // caller -- it converts SI's wire centi-degrees to radians BEFORE calling
  // this method, matching every other pose field's existing radians
  // convention (pose.h here is already in radians, like fusedPose().pose.h/
  // encoderPose().pose.h). Deliberately does NOT touch haveEncBaseline_/
  // prevEncLeft_/prevEncRight_/haveLastTick_: SI re-anchors the BELIEVED
  // pose only -- it never rezeroes the encoders themselves (that is
  // ZERO enc's/resetEncoderBaseline()'s job, immediately below) -- so
  // encoder-delta tracking continues uninterrupted from wherever the wheels
  // actually are. Wraps pose.h through wrapPi() before storing, matching
  // encTheta_'s own always-wrapped invariant (tick()'s own wrapPi() call).
  void setPose(const msg::SetPose& pose);

  // resetEncoderBaseline -- 084-007 (SUC-006): `ZERO enc`'s own effect on
  // this class (source/commands/pose_commands.cpp's handleZero(), called
  // in the SAME wire dispatch that also stages the bound pair's hardware
  // encoder zero via Hal::Motor::resetPosition()). Does NOT touch encX_/
  // encY_/encTheta_ or the EKF's own state -- the believed pose itself is
  // untouched; only the encoder-delta bookkeeping is (eventually) resynced.
  //
  // Deferred, not immediate: Hal::Motor::resetPosition() is itself STAGED
  // ("zero encoder (staged, not immediate)" -- hal/capability/motor.h) --
  // its actual hardware effect lands only at the top of the leaf's next
  // tick(), which is not necessarily THIS pass's tick() (NezhaHardware's
  // per-port I2C round-robin may take several passes to reach the affected
  // port; the sim harness's dt=0 synchronous-command replay -- see
  // tests/_infra/sim/sim_api.cpp's own Decision 4 doc comment -- makes THIS
  // pass's tick() a guaranteed no-op for the encoder read). If this method
  // cleared haveEncBaseline_ synchronously, the very next tick() call --
  // which may still observe the STALE, not-yet-zeroed encoder reading --
  // would immediately consume the one-shot guard and re-baseline against
  // that stale value, so the LATER tick() where the reading actually snaps
  // to zero would then diff the fresh zero against the stale baseline,
  // fabricating exactly the large phantom jump this method exists to
  // prevent (empirically confirmed against the sim harness while
  // implementing this ticket).
  //
  // Instead, this method only arms encBaselineResetPending_. tick() applies
  // the actual haveEncBaseline_/prevEncLeft_/prevEncRight_ reset (and clears
  // the pending flag) on the FIRST subsequent tick() whose dt is genuinely
  // > 0 -- i.e., the first tick that reflects real elapsed time, which is
  // exactly the first tick() call any staged hardware effect (including the
  // paired resetPosition() calls) has had a chance to actually land by. A
  // dt == 0 tick() (this same pass, or any further synchronous command
  // dispatched before the next real tick) leaves the pending flag armed and
  // falls through to ordinary processing unaffected (a zero encoder delta
  // regardless, since the reading has not changed yet).
  void resetEncoderBaseline();

  // trackwidth -- the SAME configured trackwidth used internally by tick()'s
  // dead-reckoning kinematics (configure()'s config.trackwidth). Small,
  // read-only addition (082, ticket 004): commands/telemetry_commands.cpp's
  // TLM `twist=` field is a pure kinematic transform of the two DIRECTLY-read
  // wheel velocities (BodyKinematics::forward(velLeft, velRight, trackwidth,
  // ...)) -- never Drivetrain::state(), never EKF velocity-channel state
  // (architecture-update.md Decision 7) -- and needs the same trackwidth this
  // class already holds, rather than a second, independently-configured
  // copy. Mirrors Hal::PhysicsWorld::trackwidth()'s existing pure-getter
  // precedent (source/hal/sim/physics_world.h).
  float trackwidth() const { return trackwidth_; }

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

  // The full config as last passed to configure(), stored verbatim (087-004)
  // — backs config()'s round-trip, mirrors Subsystems::Drivetrain's own
  // config_ member. NOT used internally by tick() (trackwidth_/
  // rotationalSlip_ below remain the fields tick()'s own math reads,
  // unchanged from before this ticket).
  msg::DrivetrainConfig config_ = {};

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

  // encBaselineResetPending_ -- 084-007 (SUC-006): armed by
  // resetEncoderBaseline(), consumed by tick() on the first subsequent call
  // whose dt is genuinely > 0 (see resetEncoderBaseline()'s own doc comment
  // for why this must be deferred rather than applied synchronously).
  bool encBaselineResetPending_ = false;

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
