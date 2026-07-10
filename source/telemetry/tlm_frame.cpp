// tlm_frame.cpp -- Telemetry::tick()/buildTelemetryMessage(). See
// tlm_frame.h for the full design rationale and the wire-key exclusion
// note. The text formatter this file used to also carry (buildTlmFrame(),
// plus its modeChar()/appendField()/kAngleScale helpers) was deleted by
// 097-008 (architecture-update-r2.md Decision 9, pure-binary firmware) --
// see git history for that prior code.
#include "telemetry/tlm_frame.h"

#include "kinematics/body_kinematics.h"

namespace Telemetry {

TlmFrameInput tick(uint32_t now, const Rt::Blackboard& bb) {
  // enc=/vel= read bb.motors[]'s primitive fields DIRECTLY for the
  // Drivetrain's bound pair -- never bb.drivetrain's vel_[] (commanded
  // targets, a different semantic). THE conversion boundary (0-based motor
  // indices, OOP refactor): bb.drivetrainConfig.left_port/right_port are
  // wire/serialized 1-based labels -- converted to 0-based Hardware motor
  // indices here, once.
  uint32_t leftIdx = bb.drivetrainConfig.left_port - 1;
  uint32_t rightIdx = bb.drivetrainConfig.right_port - 1;
  const msg::MotorState& left = bb.motors[leftIdx];
  const msg::MotorState& right = bb.motors[rightIdx];

  float velLeft = left.velocity.has ? left.velocity.val : 0.0f;
  float velRight = right.velocity.has ? right.velocity.val : 0.0f;

  TlmFrameInput in;
  in.now = now;
  // driveMode -- 084-005/096-003: bb.planner.mode is the SOLE source
  // (architecture-update.md (084) Decision 6), copied verbatim -- see
  // tlm_frame.h's own doc comment on TlmFrameInput.driveMode (097-008: now
  // the only mode-carrying field on this struct).
  in.driveMode = bb.planner.mode;
  // seq= -- READ ONLY. The shared periodic-emission counter (bb.telemetrySeq)
  // is advanced by the caller, not here -- see this function's own doc
  // comment in tlm_frame.h.
  in.seq = bb.telemetrySeq;

  in.hasEnc = true;
  in.encLeft = left.position.has ? left.position.val : 0.0f;
  in.encRight = right.position.has ? right.position.val : 0.0f;

  in.hasVel = true;
  in.velLeft = velLeft;
  in.velRight = velRight;

  // cmd= -- the drivetrain's commanded per-wheel velocity (vel_[0]=left,
  // vel_[1]=right; drivetrain.cpp state()), i.e. the velocity PID's setpoint.
  // Present whenever the drivetrain reports its two wheel targets (vel_count
  // >= 2); omitted (like every optional field) when it does not. Read from
  // bb.drivetrain here deliberately -- unlike vel=/enc= (measured, from
  // bb.motors[]), cmd= IS the commanded-target semantic bb.drivetrain owns.
  if (bb.drivetrain.vel_count_val() >= 2) {
    in.hasCmdVel = true;
    in.cmdVelLeft = bb.drivetrain.vel()[0];
    in.cmdVelRight = bb.drivetrain.vel()[1];
  }

  // pose=/encpose= read bb's two independent pose readings -- never
  // bb.drivetrain either.
  in.hasPose = true;
  in.pose = bb.fusedPose.pose;

  in.hasEncPose = true;
  in.encPose = bb.encoderPose.pose;

  // otos= -- the raw sampled odometer pose, OMITTED (not zero-filled) when
  // no odometer device exists at all (bb.otosPresent, a boot-time snapshot).
  if (bb.otosPresent) {
    in.hasOtos = true;
    in.otos = bb.otos.pose;
    in.otosConnected = bb.otosConnected;
  }

  // twist= -- a pure kinematic transform (BodyKinematics::forward()) of the
  // SAME directly-read wheel velocities vel= uses, plus the SAME trackwidth
  // PoseEstimator::configure() was given (bb.drivetrainConfig.trackwidth).
  // Directly-measured/derived, never bb.drivetrain, never EKF
  // velocity-channel state.
  in.hasTwist = true;
  BodyKinematics::forward(velLeft, velRight, bb.drivetrainConfig.trackwidth,
                           in.twist.v_x, in.twist.omega);
  in.twist.v_y = 0.0f;   // differential-only this sprint -- see drivetrain.h

  // Bench-diagnostic fields (096-003) -- TRANSCRIBED EXACTLY from
  // handleTlm()'s own computation (motion_commands.cpp), never re-derived.
  // acc= is the firmware-EMA measured acceleration; active= is motion in
  // progress (dt.busy, NOT the authority flag -- see handleTlm()'s own
  // comment on why); conn=/glitch=/ts= read bb.motors[0]/bb.motors[1]
  // DIRECTLY -- the SAME hardcoded bound-pair indices handleTlm() itself
  // uses, deliberately NOT the leftIdx/rightIdx bb.drivetrainConfig-derived
  // indices enc=/vel= use above (handleTlm() never made that
  // generalization, and this transcribes its computation exactly).
  const msg::DrivetrainState& dt = bb.drivetrain;
  in.accLeft = dt.acc_count_val() >= 1 ? dt.acc()[0] : 0.0f;
  in.accRight = dt.acc_count_val() >= 2 ? dt.acc()[1] : 0.0f;
  in.active = dt.busy;
  in.connLeft = bb.motors[0].connected;
  in.connRight = bb.motors[1].connected;
  in.glitchLeft = bb.motors[0].enc_glitch_count.has ? bb.motors[0].enc_glitch_count.val : 0;
  in.glitchRight = bb.motors[1].enc_glitch_count.has ? bb.motors[1].enc_glitch_count.val : 0;
  in.tsLeft = bb.motors[0].sampled_at.has ? bb.motors[0].sampled_at.val : 0;
  in.tsRight = bb.motors[1].sampled_at.has ? bb.motors[1].sampled_at.val : 0;

  return in;
}

void buildTelemetryMessage(msg::Telemetry& out, const TlmFrameInput& in) {
  // Pure, stateless: always start from a fresh POD -- never assume the
  // caller pre-cleared `out` (the SAME "same inputs always produce the same
  // outputs" contract the deleted text formatter, buildTlmFrame(), also
  // held -- 097-008).
  out = msg::Telemetry();

  out.now = in.now;
  out.mode = in.driveMode;   // the RAW enum -- see TlmFrameInput.driveMode's own doc comment
  out.seq = in.seq;

  out.has_enc = in.hasEnc;
  out.enc_left = in.encLeft;
  out.enc_right = in.encRight;

  out.has_vel = in.hasVel;
  out.vel_left = in.velLeft;
  out.vel_right = in.velRight;

  out.has_cmd_vel = in.hasCmdVel;
  out.cmd_vel_left = in.cmdVelLeft;
  out.cmd_vel_right = in.cmdVelRight;

  out.has_pose = in.hasPose;
  out.pose = in.pose;

  // encpose/hasEncPose intentionally NOT copied -- msg::Telemetry has no
  // corresponding field (096-001's trim, Decision 6) -- see this
  // function's own doc comment in tlm_frame.h.

  out.has_otos = in.hasOtos;
  out.otos = in.otos;
  out.otos_connected = in.otosConnected;

  out.has_twist = in.hasTwist;
  out.twist = in.twist;

  // Bench-diagnostic fields -- unconditionally copied, no `has_*` flag on
  // either side (mirrors the now-deleted text handleTlm()'s own reply,
  // which never omitted them -- see motion_commands.cpp git history,
  // 097-008).
  out.acc_left = in.accLeft;
  out.acc_right = in.accRight;
  out.active = in.active;
  out.conn_left = in.connLeft;
  out.conn_right = in.connRight;
  out.glitch_left = in.glitchLeft;
  out.glitch_right = in.glitchRight;
  out.ts_left = in.tsLeft;
  out.ts_right = in.tsRight;
}

}  // namespace Telemetry
