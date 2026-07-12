#pragma once

#include <stdint.h>

#include "messages/common.h"
#include "messages/telemetry.h"
#include "runtime/blackboard.h"

// ---------------------------------------------------------------------------
// tlm_frame.h -- Telemetry's own TLM-frame internals (sprint 082 ticket 004;
// re-pointed at the committed blackboard by sprint 087 ticket 008; extended
// with the bench-diagnostic fields and a binary formatter by 096-003,
// architecture-update.md (096) M3/Decision 6):
//
//   Telemetry::tick()        -- reads every field the frame emits directly
//                                from the committed Rt::Blackboard snapshot
//                                bb (x[k+1]) and returns a populated
//                                TlmFrameInput. Holds no Subsystems::*
//                                reference (Rt::Blackboard itself holds
//                                none -- SUC-006).
//   Telemetry::buildTlmFrame() -- DELETED (097-008, architecture-update-r2.md
//                                Decision 9, pure-binary firmware): the text
//                                "TLM t=... mode=..." line formatter this
//                                function used to pair with. Its only
//                                callers (STREAM/SNAP's text handlers,
//                                commands/telemetry_commands.cpp) were
//                                deleted the same ticket -- see that file's
//                                own header comment.
//   Telemetry::buildTelemetryMessage() -- (096-003) the sole remaining
//                                formatter: TlmFrameInput -> a populated
//                                msg::Telemetry POD. Used by the binary
//                                `stream` path (commands/
//                                telemetry_commands.cpp's tickTelemetry(),
//                                unconditionally since 097-008 -- there is no
//                                more text path to branch against).
//
// Ported in CONCEPT (field set, per-field omission, integer scaling) from
// source_old/robot/RobotTelemetry.cpp's Robot::buildTlmFrame(), trimmed to
// this dev-bench tree's minimal fixed field set (architecture-update.md (082)
// Decision 5 -- no `STREAM fields=<csv>` subscription bitmask, so there is no
// per-field gating bit to read here; every field's presence is instead an
// explicit `has*` flag on TlmFrameInput).
//
// Both functions are pure: tick() reads only `bb` (never mutates it -- in
// particular it does NOT advance bb.telemetrySeq; that shared counter is the
// CALLER's bookkeeping -- see commands/telemetry_commands.cpp's
// telemetryEmitBinary()) and buildTelemetryMessage() reads only its `in`
// argument and writes only into the caller-supplied `out`. The SAME inputs
// always produce the SAME outputs. This is what makes both independently
// unit-testable (tests/sim/unit/tlm_frame_harness.cpp) -- buildTelemetryMessage()
// with plain scalar/msg:: struct inputs, tick() with a bare, non-live
// Rt::Blackboard -- with no DevLoop/Hardware/Drivetrain/PoseEstimator/
// Planner/CommandRouter dependency at all (087-008, SUC-002).
//
// Historical wire contract (docs/protocol-v2.md §8; see that section's
// "minimal subset" note added by 082-004): the now-deleted text formatter's
// field tokens (`t=`, `mode=`, `seq=`, `enc=`, `vel=`, `pose=`, `encpose=`,
// `otos=`, `otosconn=` (092-002), `twist=`) were wire keys, excluded from
// the no-units-in-identifiers convention (.claude/rules/coding-standards.md,
// "Wire/serialized identifiers are excluded"). Kept here as a historical
// note now that buildTlmFrame() itself is gone (097-008) -- the live wire
// contract is protos/telemetry.proto's own field names.
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
  // driveMode (096-003) -- the RAW msg::DriveMode, sourced directly from
  // bb.planner.mode (tick()'s own assignment), for buildTelemetryMessage()'s
  // exclusive use. Until 097-008 this coexisted with a second, lossy
  // char-mapped `mode` field the now-deleted text formatter used (IDLE and
  // VELOCITY both mapped to 'I', a mapping that could not be reversed) --
  // that field and its modeChar() helper are gone; driveMode is now the
  // sole mode-carrying field on this struct.
  msg::DriveMode driveMode = msg::DriveMode::IDLE;
  uint16_t seq = 0;   // seq= -- always present

  // enc=<l>,<r> -- per-wheel accumulated encoder distance.
  bool hasEnc = false;
  float encLeft = 0.0f;    // [mm]
  float encRight = 0.0f;   // [mm]

  // vel=<l>,<r> -- per-wheel actual (measured) velocity.
  bool hasVel = false;
  float velLeft = 0.0f;    // [mm/s] signed
  float velRight = 0.0f;   // [mm/s] signed

  // cmd=<l>,<r> -- per-wheel COMMANDED velocity: the velocity PID's own
  // setpoint (bb.drivetrain.vel_[], == each motor's velocityTarget_, set
  // directly with no slew on the setpoint -- nezha_motor.cpp). Distinct from
  // vel= (measured): plotting cmd= against vel= exposes the velocity loop's
  // tracking error and terminal overshoot, which vel= alone cannot show.
  bool hasCmdVel = false;
  float cmdVelLeft = 0.0f;    // [mm/s] signed
  float cmdVelRight = 0.0f;   // [mm/s] signed

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

  // otosconn=0|1 (092-002) -- a SEPARATE token, sharing hasOtos's own
  // omission gate: Hal::Odometer::connected() this pass (bb.otosConnected)
  // -- "does a real device exist and answer at all," distinct from
  // otosValid (fusableThisPass()'s reset-tracking flag, not itself
  // surfaced over the wire) and from otos='s own per-pass freshness
  // (which this trimmed EkfTiny/telemetry surface does not expose
  // either). Added as a diagnostic for the frozen-fused-pose
  // investigation (clasi/issues/poseestimator-fused-pose-frozen-on-
  // hardware.md) -- see ticket 092-002's completion notes for why: no
  // existing wire verb told a bench session whether Hal::OtosOdometer
  // had ever detected a chip. A separate token (not a 4th otos= value)
  // so the existing, host-parsed otos= 3-tuple shape never changes.
  bool otosConnected = false;

  // twist=<v>,<omega> -- directly-measured/derived body twist (never EKF
  // velocity-channel state -- see commands/telemetry_commands.cpp for how
  // the caller derives this from directly-read wheel velocities).
  bool hasTwist = false;
  msg::BodyTwist3 twist = {};   // v_x [mm/s]; omega [rad/s] -- converted to mrad/s at format time

  // --- Bench-diagnostic fields (096-003; transcribed, not re-derived, from
  // handleTlm()'s own computation, source/commands/motion_commands.cpp --
  // see that function's own doc comment for the full field-by-field
  // rationale). Unconditionally present, no `has*` flag: handleTlm()'s own
  // text reply never omits them, and buildTlmFrame() never reads them at
  // all (they exist on TlmFrameInput solely for buildTelemetryMessage()'s
  // use) -- see this struct's own file header for why the text formatter's
  // output stays byte-identical despite this extension.
  float accLeft = 0.0f;    // [mm/s^2] bb.drivetrain.acc()[0], EMA-filtered
  float accRight = 0.0f;   // [mm/s^2] bb.drivetrain.acc()[1], EMA-filtered
  bool active = false;     // bb.drivetrain.busy -- motion in progress
  bool connLeft = false;   // bb.motors[0].connected
  bool connRight = false;  // bb.motors[1].connected
  uint32_t glitchLeft = 0;    // bb.motors[0].enc_glitch_count
  uint32_t glitchRight = 0;   // bb.motors[1].enc_glitch_count
  uint32_t tsLeft = 0;    // [ms] bb.motors[0].sampled_at
  uint32_t tsRight = 0;   // [ms] bb.motors[1].sampled_at
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
//   otosconn=  -- (092-002) bb.otosConnected, gated on the SAME
//                 bb.otosPresent condition as otos= (never emitted alone).
//   twist=     -- BodyKinematics::forward() applied to the SAME directly-read
//                 wheel velocities vel= uses, plus bb.drivetrainConfig's
//                 trackwidth (the SAME value PoseEstimator::configure() was
//                 given -- both share msg::DrivetrainConfig) -- a pure
//                 kinematic transform, never bb.drivetrain, never EKF
//                 velocity-channel state.
//   driveMode  -- bb.planner.mode (msg::DriveMode), copied verbatim (097-008:
//                 the only mode-carrying field now -- the deleted text
//                 formatter's lossy char mapping is gone) for
//                 buildTelemetryMessage()'s use.
//   seq=       -- bb.telemetrySeq, READ only -- tick() never increments it;
//                 the caller (telemetryEmitBinary()) advances the shared
//                 counter itself, immediately after capturing this call's
//                 return value.
//   acc=/active=/conn=/glitch=/ts= (096-003) -- transcribed EXACTLY from
//                 handleTlm()'s own computation (motion_commands.cpp):
//                 bb.drivetrain.acc()/.busy for acc=/active=; bb.motors[0]/
//                 bb.motors[1] (the SAME hardcoded bound-pair indices
//                 handleTlm() itself uses -- NOT the leftIdx/rightIdx
//                 bb.drivetrainConfig-derived indices enc=/vel= use above)
//                 for conn=/glitch=/ts=.
TlmFrameInput tick(uint32_t now, const Rt::Blackboard& bb);

// buildTelemetryMessage -- (096-003) populate `out` (a generated
// msg::Telemetry POD) from `in`, writing typed struct fields. Pure and
// stateless: reads only `in`, writes only `out` (always starting from a
// fresh msg::Telemetry{}, never assuming the caller pre-cleared it).
// `encpose`/`hasEncPose` have no counterpart on msg::Telemetry (096-001's
// trim, architecture-update.md (096) Decision 6) and are not copied; every
// other TlmFrameInput field, plus the five bench-diagnostic field groups,
// maps 1:1 onto msg::Telemetry's own field/`has_*`-flag pairs
// (protos/telemetry.proto). This is now the ONLY TlmFrameInput formatter --
// its text-formatter sibling, buildTlmFrame(), was deleted by 097-008
// (architecture-update-r2.md Decision 9, pure-binary firmware).
void buildTelemetryMessage(msg::Telemetry& out, const TlmFrameInput& in);

}  // namespace Telemetry
