// tlm_frame.cpp -- Telemetry::tick()/buildTlmFrame(). See tlm_frame.h for
// the full design rationale and the wire-key exclusion note.
#include "telemetry/tlm_frame.h"

#include <cstdarg>
#include <cstddef>
#include <cstdio>
#include <cstring>

#include "kinematics/body_kinematics.h"

namespace Telemetry {

namespace {

// kAngleScale -- 18000/pi ~= 5729.5779513, converting radians to
// centidegrees. Same constant source_old/robot/RobotTelemetry.cpp's
// buildTlmFrame() used for pose=/encpose=/otos=.
constexpr float kAngleScale = 5729.5779513f;   // [cdeg/rad]

// modeChar -- 084-005: maps msg::DriveMode to TLM's single-character `mode=`
// wire value, per docs/protocol-v2.md §8's I/S/T/D/G vocabulary and
// architecture-update.md (084) Decision 6. Moved here from
// commands/telemetry_commands.cpp by 087-008, alongside the rest of
// Telemetry::tick()'s field-sourcing logic.
char modeChar(msg::DriveMode mode) {
  switch (mode) {
    case msg::DriveMode::IDLE: return 'I';
    case msg::DriveMode::STREAMING: return 'S';
    case msg::DriveMode::TIMED: return 'T';
    case msg::DriveMode::DISTANCE: return 'D';
    case msg::DriveMode::GO_TO: return 'G';
    case msg::DriveMode::VELOCITY:
    default:
      return 'I';
  }
}

// appendField -- format one field into buf+pos (size rem), advancing
// pos/rem only on a fully-committed (non-truncated) write. Once `truncated`
// is set (this write, or an earlier one, didn't fit), every subsequent call
// is a no-op: this is what the incremental-snprintf idiom in
// source_old/robot/RobotTelemetry.cpp's buildTlmFrame() got subtly wrong for
// a buffer too small even for the MANDATORY t=/mode=/seq= prefix -- without
// this guard, a later field's write would land at the SAME buf+pos the
// (also-truncated) earlier field just wrote, silently replacing it. vsnprintf
// itself always NUL-terminates within the given size (as long as rem > 0,
// which holds here: rem only ever shrinks via the successful branch, which
// leaves it >= 1), so buf is always a valid, safely-terminated C string on
// return, truncated or not.
void appendField(char* buf, int& pos, int& rem, bool& truncated, const char* fmt, ...) {
  if (truncated) return;

  va_list ap;
  va_start(ap, fmt);
  int n = std::vsnprintf(buf + pos, static_cast<size_t>(rem), fmt, ap);
  va_end(ap);

  if (n > 0 && n < rem) {
    pos += n;
    rem -= n;
  } else {
    truncated = true;
  }
}

}  // namespace

TlmFrameInput tick(uint32_t now, const Rt::Blackboard& bb) {
  // enc=/vel= read bb.motor[]'s primitive fields DIRECTLY for the
  // Drivetrain's bound pair -- never bb.drivetrain's vel_[] (commanded
  // targets, a different semantic).
  uint32_t leftPort = bb.drivetrainConfig.left_port;
  uint32_t rightPort = bb.drivetrainConfig.right_port;
  const msg::MotorState& left = bb.motor[leftPort - 1];
  const msg::MotorState& right = bb.motor[rightPort - 1];

  float velLeft = left.velocity.has ? left.velocity.val : 0.0f;
  float velRight = right.velocity.has ? right.velocity.val : 0.0f;

  TlmFrameInput in;
  in.now = now;
  // mode= -- 084-005: bb.planner.mode is the SOLE source (architecture-
  // update.md (084) Decision 6).
  in.mode = modeChar(bb.planner.mode);
  // seq= -- READ ONLY. The shared STREAM/SNAP counter (bb.telemetrySeq) is
  // advanced by the caller, not here -- see this function's own doc
  // comment in tlm_frame.h.
  in.seq = bb.telemetrySeq;

  in.hasEnc = true;
  in.encLeft = left.position.has ? left.position.val : 0.0f;
  in.encRight = right.position.has ? right.position.val : 0.0f;

  in.hasVel = true;
  in.velLeft = velLeft;
  in.velRight = velRight;

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

  return in;
}

int buildTlmFrame(char* buf, int len, const TlmFrameInput& in) {
  if (buf == nullptr || len <= 0) return 0;

  int pos = 0;
  int rem = len;
  bool truncated = false;

  appendField(buf, pos, rem, truncated, "TLM t=%lu mode=%c seq=%u",
              static_cast<unsigned long>(in.now), in.mode,
              static_cast<unsigned>(in.seq));

  if (in.hasEnc) {
    appendField(buf, pos, rem, truncated, " enc=%d,%d",
                static_cast<int>(in.encLeft), static_cast<int>(in.encRight));
  }

  if (in.hasVel) {
    appendField(buf, pos, rem, truncated, " vel=%d,%d",
                static_cast<int>(in.velLeft), static_cast<int>(in.velRight));
  }

  if (in.hasPose) {
    appendField(buf, pos, rem, truncated, " pose=%d,%d,%d",
                static_cast<int>(in.pose.x), static_cast<int>(in.pose.y),
                static_cast<int>(in.pose.h * kAngleScale));
  }

  if (in.hasEncPose) {
    appendField(buf, pos, rem, truncated, " encpose=%d,%d,%d",
                static_cast<int>(in.encPose.x), static_cast<int>(in.encPose.y),
                static_cast<int>(in.encPose.h * kAngleScale));
  }

  if (in.hasOtos) {
    appendField(buf, pos, rem, truncated, " otos=%d,%d,%d",
                static_cast<int>(in.otos.x), static_cast<int>(in.otos.y),
                static_cast<int>(in.otos.h * kAngleScale));
  }

  if (in.hasTwist) {
    appendField(buf, pos, rem, truncated, " twist=%d,%d",
                static_cast<int>(in.twist.v_x),
                static_cast<int>(in.twist.omega * 1000.0f));
  }

  // The return value reflects the buffer's ACTUAL string length (via
  // strlen(), not the internal `pos` bookkeeping above): when nothing was
  // truncated the two agree exactly, but a too-small buffer's final
  // (truncated) appendField() call still leaves a valid, shorter
  // NUL-terminated string in buf that `pos` alone would under-report.
  return static_cast<int>(std::strlen(buf));
}

}  // namespace Telemetry
