// main_loop.cpp -- Rt::MainLoop: see main_loop.h for the class-level
// contract. Sprint 094 ticket 094-005 reorders tick() to `hardware_.
// tick(now)` -> `drivetrain_.tick(now, bb.segmentIn, bb.driveIn)` -> commit,
// and deletes routeOutputs() -- Subsystems::Drivetrain (094-004) now stages
// its own wheel writes directly through hardware_'s motor refs, so there is
// nothing left to route. Sprint 099 ticket 099-004 adds a PoseEstimator
// pass step (encoder-only this ticket -- see architecture-update.md D1's
// pass pseudocode; OTOS fusion is ticket 099-007's "one-token flip").
#include "runtime/main_loop.h"

#include "kinematics/body_kinematics.h"

namespace Rt {

MainLoop::MainLoop(Subsystems::Hardware& hardware, Subsystems::Drivetrain& drivetrain,
                    Subsystems::PoseEstimator& poseEstimator)
    : hardware_(hardware), drivetrain_(drivetrain), poseEstimator_(poseEstimator) {}

void MainLoop::commit(Blackboard& bb, uint32_t now, bool otosFusable,
                       const msg::PoseEstimate& otosSample) {
  // === COMMIT (clock edge): copy each subsystem cell into bb -> x[k+1]. ===
  bb.motors = hardware_.motorStates();
  bb.drivetrain = drivetrain_.state();

  bb.encoderPose = poseEstimator_.encoderPose();
  bb.fusedPose = poseEstimator_.fusedPose();
  bb.poseStepped = poseEstimator_.lastPoseStep();

  // bb.otos/bb.otosValid/bb.otosConnected (099-002/099-007): otosSample and
  // otosFusable are THIS pass's already-read values (tick()'s single
  // fusableThisPass()/pose() call, threaded in as arguments rather than
  // re-read here) -- committed every pass regardless of whether
  // PoseEstimator actually fused it (that gate also folds in
  // otosSample.stamp.valid; bb.otosValid reflects fusableThisPass() alone,
  // matching architecture-update.md D1's pseudocode exactly). connected()
  // is deliberately the LIVE, re-evaluated-every-tick() flag here (unlike
  // NezhaHardware::tick()'s own scheduling gate, which needed the permanent
  // present() instead -- see architecture-update-r1.md Decision 2):
  // bb.otosConnected is a per-pass telemetry/diagnostic read, not a
  // scheduling decision, so it should track the chip's actual live bus
  // health, not just "was one ever detected."
  bb.otos = otosSample;
  bb.otosValid = otosFusable;
  bb.otosConnected = hardware_.odometer()->connected();

  // bodyState (099-004, architecture-update.md Addition 2): pose from the
  // SAME bb.fusedPose just committed above; twist via BodyKinematics::
  // forward() on the bound pair's DIRECTLY-read wheel velocities (bb.motors[]
  // was just refreshed above, from the SAME hardware_.tick() pass tick()'s
  // own leftObs/rightObs reads came from -- reading it back here rather than
  // threading leftObs/rightObs through commit()'s own signature) and
  // poseEstimator_.trackwidth() (the ONE trackwidth source -- never a
  // second, independently-configured copy; mirrors tlm_frame.cpp's own
  // twist= derivation exactly). Differential-only this sprint: v_y stays 0.
  uint32_t leftIdx = bb.drivetrainConfig.left_port - 1;
  uint32_t rightIdx = bb.drivetrainConfig.right_port - 1;
  const msg::MotorState& left = bb.motors[leftIdx];
  const msg::MotorState& right = bb.motors[rightIdx];
  float velLeft = left.velocity.has ? left.velocity.val : 0.0f;
  float velRight = right.velocity.has ? right.velocity.val : 0.0f;

  bb.bodyState.pose = bb.fusedPose.pose;
  BodyKinematics::forward(velLeft, velRight, poseEstimator_.trackwidth(),
                           bb.bodyState.twist.v_x, bb.bodyState.twist.omega);
  bb.bodyState.twist.v_y = 0.0f;
  bb.bodyState.stamp = bb.fusedPose.stamp;

  bb.loopNow = now;   // commit stamp for TLM now= (cmd='s true time)
}

void MainLoop::tick(Blackboard& bb, uint32_t now) {
  // === MANDATORY: control. ===
  //
  // hardware_.tick() stays FIRST (its pre-094 position): it flushes
  // whatever Drivetrain STAGED onto the motor refs last pass (via
  // hardware_.motor(port).apply(), inside Drivetrain::tick() below) and
  // collects fresh encoders -- so a setpoint staged THIS pass is flushed
  // the FOLLOWING pass, identical one-pass latency to the pre-094
  // `routeOutputs() -> bb.motorIn[] -> next-pass drain` chain (the
  // load-bearing sequencing decision -- architecture-update.md Section 5,
  // "Loop order"). drivetrain_.tick() then reads FRESH encoders via
  // hardware_.motorState(), runs its own SegmentExecutor/escape-hatch dispatch,
  // and stages THIS pass's setpoints (flushed next pass by the step
  // above).
  hardware_.tick(now);
  drivetrain_.tick(now, bb.segmentIn, bb.replaceIn, bb.driveIn);

  // === PoseEstimator (099-004/099-007): read the bound pair's FRESH
  // MotorState (post hardware_.tick()/drivetrain_.tick() above -- mirrors
  // tlm_frame.cpp's own left_port/right_port -> 0-based-index conversion
  // pattern exactly) and tick PoseEstimator, now WITH OTOS fusion (099-007,
  // "the one-token flip" -- architecture-update.md D1's pseudocode,
  // implemented verbatim below). ===
  uint32_t leftIdx = bb.drivetrainConfig.left_port - 1;
  uint32_t rightIdx = bb.drivetrainConfig.right_port - 1;
  msg::MotorState leftObs = hardware_.motorState(leftIdx);
  msg::MotorState rightObs = hardware_.motorState(rightIdx);

  // fusableThisPass() (099-007): a ONE-SHOT, read-and-clear signal --
  // hal/capability/odometer.h's own doc comment names THIS call site as its
  // one sanctioned caller and warns against a second call anywhere else in
  // the same pass (it would wrongly consume the signal). Called EXACTLY
  // ONCE per pass, right here. otosSample is read unconditionally (a cheap
  // getter, unlike fusableThisPass()) so commit() below can still publish
  // bb.otos regardless of fusability.
  bool fusable = hardware_.odometer()->fusableThisPass();
  msg::PoseEstimate otosSample = hardware_.odometer()->pose();

  // otosArg: only pass a non-null observation into PoseEstimator::tick()
  // when it is both fusable (no odometer reset landed THIS pass) and fresh
  // (stamp.valid) -- matching pose_estimator.cpp's own consumption gate
  // (`otosObs->stamp.valid`) exactly, so this check is never redundantly
  // re-done inside PoseEstimator itself.
  const msg::PoseEstimate* otosArg =
      (fusable && otosSample.stamp.valid) ? &otosSample : nullptr;

  poseEstimator_.tick(now, leftObs, rightObs, otosArg, bb.poseResetIn, bb.otosSetPoseIn,
                       bb.poseFixIn);

  // A queued SI-equivalent re-anchor (BinaryChannel::handlePose(), 099-004)
  // posts its freshly re-anchored fusedPose() here -- drain it into the
  // ODOMETER's own setPose(), the existing, already-implemented
  // Odometer::applySetPose() primitive (its own doc comment already names
  // this exact call site as "ported verbatim from main_loop.cpp's former
  // inline otosSetPoseIn drain").
  if (!bb.otosSetPoseIn.empty()) {
    hardware_.odometer()->applySetPose(bb.otosSetPoseIn.take());
  }

  // === COMMIT (clock edge): x[k] -> x[k+1]. Nothing left to route --
  // Drivetrain already staged its own wheel writes above. ===
  commit(bb, now, fusable, otosSample);
}

}  // namespace Rt
