#pragma once

#include <stdint.h>

#include "messages/common.h"
#include "runtime/blackboard.h"

// ---------------------------------------------------------------------------
// tlm_frame.h -- Telemetry's own TLM-frame internals (sprint 082 ticket 004;
// re-pointed at the committed blackboard by sprint 087 ticket 008):
//
//   Telemetry::tick()        -- reads every field the frame emits directly
//                                from the committed Rt::Blackboard snapshot
//                                bb (x[k+1]) and returns a populated
//                                TlmFrameInput. Holds no Subsystems::*
//                                reference (Rt::Blackboard itself holds
//                                none -- SUC-006).
//   Telemetry::buildTlmFrame() -- a pure, stateless formatter: TlmFrameInput
//                                -> one wire line. Unchanged since 082-004.
//
// Ported in CONCEPT (field set, per-field omission, integer scaling) from
// source_old/robot/RobotTelemetry.cpp's Robot::buildTlmFrame(), trimmed to
// this dev-bench tree's minimal fixed field set (architecture-update.md (082)
// Decision 5 -- no `STREAM fields=<csv>` subscription bitmask, so there is no
// per-field gating bit to read here; every field's presence is instead an
// explicit `has*` flag on TlmFrameInput).
//
// Both functions are pure: tick() reads only `bb` (never mutates it -- in
// particular it does NOT advance bb.telemetrySeq; that shared STREAM/SNAP
// counter is the CALLER's bookkeeping -- see commands/telemetry_commands.cpp's
// telemetryEmit()) and buildTlmFrame() reads only its `in` argument and
// writes only into the caller-supplied buffer. The SAME inputs always
// produce the SAME outputs. This is what makes both independently
// unit-testable (tests/sim/unit/tlm_frame_harness.cpp) -- buildTlmFrame()
// with plain scalar/msg:: struct inputs, tick() with a bare, non-live
// Rt::Blackboard -- with no DevLoop/Hardware/Drivetrain/PoseEstimator/
// Planner/CommandRouter dependency at all (087-008, SUC-002).
//
// Wire contract (docs/protocol-v2.md §8; see that section's "minimal
// subset" note added by 082-004): the field tokens (`t=`, `mode=`,
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

// tick -- Telemetry's own frame-assembly step (087-008): reads every field
// the TLM frame emits directly from the committed Rt::Blackboard snapshot
// `bb` (x[k+1]) and returns the populated TlmFrameInput, ready for
// buildTlmFrame(). Field sourcing (mirrors the pre-087-008 Decision 7 rules,
// now enforced here instead of in commands/telemetry_commands.cpp):
//   enc=/vel=  -- bb.motors[]'s position/velocity DIRECTLY for the
//                 Drivetrain's bound pair (bb.drivetrainConfig.left_port/
//                 right_port). NEVER bb.drivetrain's vel_[] (commanded
//                 targets, a different semantic).
//   pose=/encpose= -- bb.fusedPose/bb.encoderPose.
//   otos=      -- bb.otos, OMITTED (not zero-filled) when bb.otosPresent is
//                 false.
//   twist=     -- BodyKinematics::forward() applied to the SAME directly-read
//                 wheel velocities vel= uses, plus bb.drivetrainConfig's
//                 trackwidth (the SAME value PoseEstimator::configure() was
//                 given -- both share msg::DrivetrainConfig) -- a pure
//                 kinematic transform, never bb.drivetrain, never EKF
//                 velocity-channel state.
//   mode=      -- bb.planner.mode (msg::DriveMode), mapped to a single wire
//                 character -- I/S/T/D/G, per docs/protocol-v2.md §8.
//   seq=       -- bb.telemetrySeq, READ only -- tick() never increments it;
//                 the caller (telemetryEmit()) advances the shared
//                 STREAM/SNAP counter itself, immediately after capturing
//                 this call's return value.
TlmFrameInput tick(uint32_t now, const Rt::Blackboard& bb);

// buildTlmFrame -- format `in` into one NUL-terminated "TLM ..." wire line in
// buf[0..len-1]. Writes at most len-1 characters plus the terminating NUL
// (never overruns buf). Returns the number of characters written, excluding
// the NUL -- the same convention snprintf()/this codebase's other
// frame-builders use (see commands/dev_commands.cpp's emitMotorState()).
int buildTlmFrame(char* buf, int len, const TlmFrameInput& in);

}  // namespace Telemetry
