#pragma once

#include <stdint.h>

#include "messages/common.h"

// ---------------------------------------------------------------------------
// tlm_frame.h -- Telemetry::buildTlmFrame(): a pure, stateless TLM
// frame-formatting function (sprint 082, ticket 004). Ported in CONCEPT
// (field set, per-field omission, integer scaling) from
// source_old/robot/RobotTelemetry.cpp's Robot::buildTlmFrame(), trimmed to
// this dev-bench tree's minimal fixed field set (architecture-update.md (082)
// Decision 5 -- no `STREAM fields=<csv>` subscription bitmask, so there is no
// per-field gating bit to read here; every field's presence is instead an
// explicit `has*` flag on TlmFrameInput, set by the CALLER
// (commands/telemetry_commands.cpp), never by this file).
//
// No I/O, no state: buildTlmFrame() reads only its `in` argument and writes
// only into the caller-supplied buffer -- the SAME TlmFrameInput always
// produces the SAME wire line. This is what makes it independently
// unit-testable (tests/sim/unit/tlm_frame_harness.cpp) with no DevLoop /
// Hardware / Drivetrain / PoseEstimator dependency at all -- plain scalar
// and msg:: struct inputs only.
//
// Wire contract (docs/protocol-v2.md §8; see that section's "minimal
// subset" note added by this ticket): the field tokens (`t=`, `mode=`,
// `seq=`, `enc=`, `vel=`, `pose=`, `encpose=`, `otos=`, `twist=`) are wire
// keys, excluded from the no-units-in-identifiers convention
// (.claude/rules/coding-standards.md, "Wire/serialized identifiers are
// excluded") -- they are never renamed here even though some read like
// unit-suffixed names.
// ---------------------------------------------------------------------------

namespace Telemetry {

// TlmFrameInput -- one frame's worth of already-sampled state. `now`/`mode`/
// `seq` are always present (the wire format's mandatory prefix); every other
// field carries its own `has*` flag so a caller (or a unit test) can omit
// any subset independently -- there is no coupling between one field's
// presence and another's. Mirrors the project's own msg::Opt<T>
// nullable-field convention (messages/common.h) rather than inventing a
// parallel optional type.
struct TlmFrameInput {
  uint32_t now = 0;   // [ms] t= -- always present
  char mode = 'I';    // 'I' or 'S' -- mode= -- always present
  uint16_t seq = 0;   // seq= -- always present

  // enc=<l>,<r> -- per-wheel accumulated encoder distance.
  bool hasEnc = false;
  float encLeft = 0.0f;    // [mm]
  float encRight = 0.0f;   // [mm]

  // vel=<l>,<r> -- per-wheel actual velocity.
  bool hasVel = false;
  float velLeft = 0.0f;    // [mm/s] signed
  float velRight = 0.0f;   // [mm/s] signed

  // pose=<x>,<y>,<h> -- fused world pose (EKF belief).
  bool hasPose = false;
  msg::Pose2D pose = {};   // x,y [mm]; h [rad] -- converted to centidegrees at format time

  // encpose=<x>,<y>,<h> -- encoder-only dead-reckoned world pose.
  bool hasEncPose = false;
  msg::Pose2D encPose = {};

  // otos=<x>,<y>,<h> -- raw sampled odometer pose. The caller omits this
  // (hasOtos = false) rather than zero-filling it when no odometer is
  // present -- see architecture-update.md (082) Decision 7's
  // omission-vs-zero-fill rule.
  bool hasOtos = false;
  msg::Pose2D otos = {};

  // twist=<v>,<omega> -- directly-measured/derived body twist (never EKF
  // velocity-channel state -- see commands/telemetry_commands.cpp for how
  // the caller derives this from directly-read wheel velocities).
  bool hasTwist = false;
  msg::BodyTwist3 twist = {};   // v_x [mm/s]; omega [rad/s] -- converted to mrad/s at format time
};

// buildTlmFrame -- format `in` into one NUL-terminated "TLM ..." wire line in
// buf[0..len-1]. Writes at most len-1 characters plus the terminating NUL
// (never overruns buf). Returns the number of characters written, excluding
// the NUL -- the same convention snprintf()/this codebase's other
// frame-builders use (see commands/dev_commands.cpp's emitMotorState()).
int buildTlmFrame(char* buf, int len, const TlmFrameInput& in);

}  // namespace Telemetry
