// binary_channel.cpp -- see binary_channel.h for the module contract.
//
// Dearmor (`*B<base64>` -> raw bytes, WireRuntime::base64Decode) -> decode
// (msg::wire::decode -> CommandEnvelope, ticket 005) -> oneof-arm dispatch
// -> Blackboard post or inline reply -> encode+armor the ReplyEnvelope
// (msg::wire::encode -> WireRuntime::base64Encode) -> replyFn. Every exit
// path replies exactly once, armored the same way -- no bare text ever
// escapes this file (a binary client only ever sees `*B<base64>` lines
// back, matching what it sent).
//
// Layout: handle() is dearmor + decode + a thin one-call-per-arm dispatch
// switch. Each oneof arm's behavior lives in its own handle<Arm>() helper
// below (the CONFIG arm's four patch kinds likewise in
// handleConfig<Kind>()), mirroring the text plane's handler-per-verb
// layout (text_channel.cpp, and the now-deleted text SET/GET's
// config_commands.cpp before 097-007). Every helper replies exactly once,
// through the sendReply funnel.
//
// 097-011 briefly also hosted tickTelemetry() + file-local
// telemetryEmitBinary(), relocated verbatim from telemetry_commands.cpp; a
// later cleanup moved both again, this time to
// source/telemetry/telemetry_tick.{h,cpp} -- see binary_channel.h's own
// header comment.
#include "commands/binary_channel.h"

#include <cstring>

#include "commands/text_channel.h"
#include "messages/wire.h"
#include "messages/wire_runtime.h"
#include "motion/segment.h"
#include "runtime/command_router.h"
#include "types/clock.h"

namespace BinaryChannel {

namespace {

Rt::Blackboard& bb(void* routerCtx) {
  return static_cast<Rt::CommandRouter*>(routerCtx)->blackboard();
}

// kMaxEnvelopeBytes -- the larger of the two generated per-direction
// budgets (both 168B as of this ticket; wire.h's own static_asserts keep
// either one from silently exceeding the 186B envelope cap on a future
// schema change) -- one raw-byte scratch buffer, reused sequentially for
// the incoming decode and the outgoing encode within a single handle()
// call (never overlapping).
constexpr size_t kMaxEnvelopeBytes =
    (msg::wire::kCommandEnvelopeMaxEncodedSize > msg::wire::kReplyEnvelopeMaxEncodedSize)
        ? msg::wire::kCommandEnvelopeMaxEncodedSize
        : msg::wire::kReplyEnvelopeMaxEncodedSize;

// kArmoredBufSize -- "*B" (2) + base64(kMaxEnvelopeBytes) (ceil(168/3)*4 =
// 224) + NUL (1) = 227, rounded up with headroom; matches
// Subsystems::CommunicatorToCommandProcessorCommand::line's own 256-byte
// budget (wire_command.h) so an armored reply always fits the SAME
// transport a request arrived on.
constexpr size_t kArmoredBufSize = 256;

// toSegment -- Decision 2's one-directional, field-by-field copy from the
// decoded wire message into the SegmentExecutor's own internal
// representation. Every field is already in Motion::Segment's native units
// (mm, rad, mm/s, ...) -- protos/motion.proto's own header comment: the
// binary plane parses real floats natively, so (unlike handleMove's/
// handleMover's own wire-cdeg -> rad conversion) no unit conversion happens
// here, only the name mapping.
Motion::Segment toSegment(const msg::MotionSegment& src) {
  Motion::Segment seg;
  seg.distance = src.distance;
  seg.direction = src.direction;
  seg.finalHeading = src.final_heading;
  seg.speedMax = src.speed_max;
  seg.accelMax = src.accel_max;
  seg.jerkMax = src.jerk_max;
  seg.yawRateMax = src.yaw_rate_max;
  seg.yawAccelMax = src.yaw_accel_max;
  seg.yawJerkMax = src.yaw_jerk_max;
  seg.time = src.time;
  seg.v = src.v;
  seg.omega = src.omega;
  seg.stream = src.stream;
  return seg;
}

// sendReply -- encode+armor+send one ReplyEnvelope. The one exit path
// every handler below funnels through, so "always reply exactly once,
// always armored" is enforced structurally rather than repeated at every
// call site.
void sendReply(const msg::ReplyEnvelope& reply, ReplyFn replyFn, void* replyCtx) {
  uint8_t rawBuf[kMaxEnvelopeBytes];
  const uint16_t n = msg::wire::encode(reply, rawBuf, static_cast<uint16_t>(sizeof(rawBuf)));
  if (n == 0) {
    // Unreachable in practice: kMaxEnvelopeBytes is sized from the SAME
    // generated kCommandEnvelopeMaxEncodedSize/kReplyEnvelopeMaxEncodedSize
    // constants encode() itself is budgeted against (wire.h's own
    // static_asserts), so every ReplyEnvelope this file ever builds fits.
    // No reply is sent rather than emitting a malformed/truncated line.
    return;
  }

  char armored[kArmoredBufSize];
  armored[0] = '*';
  armored[1] = 'B';
  size_t b64Len = 0;
  if (!WireRuntime::base64Encode(rawBuf, n, armored + 2, sizeof(armored) - 3, &b64Len)) {
    return;  // same unreachable-in-practice sizing argument as above
  }
  armored[2 + b64Len] = '\0';
  replyFn(armored, replyCtx);
}

// sendError/sendAck -- `corrId` is always the TRIGGERING CommandEnvelope's
// own corr_id (0 when none could be recovered, e.g. a dearmor failure that
// never reached decode() at all) -- envelope.proto's own doc comment:
// "corr_id is echoed back on ReplyEnvelope so a pipelined client can
// correlate replies out of order." Every reply this file sends threads it
// through; there is no reply path that omits it.
void sendError(msg::ErrCode code, uint16_t field, uint32_t corrId, ReplyFn replyFn, void* replyCtx) {
  msg::ReplyEnvelope reply;
  reply.corr_id = corrId;
  reply.body_kind = msg::ReplyEnvelope::BodyKind::ERR;
  reply.body.err.code = code;
  reply.body.err.field = field;
  sendReply(reply, replyFn, replyCtx);
}

// sendAck -- the shared success reply for drive/segment/replace/stop/
// config/stream: q/rem computed exactly the way handleMove()'s/
// handleMover()'s own text acks compute them (git history,
// motion_commands.cpp -- both deleted 097-006) --
// bb.segmentIn's undrained depth plus the Drivetrain's own committed
// ring+executing depth, and the live plan's remaining translation. t stays
// 0 (Ack.t is PING's own field -- see envelope.proto's doc comment).
void sendAck(Rt::Blackboard& b, uint32_t corrId, ReplyFn replyFn, void* replyCtx) {
  msg::ReplyEnvelope reply;
  reply.corr_id = corrId;
  reply.body_kind = msg::ReplyEnvelope::BodyKind::OK;
  reply.body.ok.q = b.segmentIn.size() + b.drivetrain.queue;
  reply.body.ok.rem = b.drivetrain.rem;
  reply.body.ok.t = 0;
  sendReply(reply, replyFn, replyCtx);
}

// --- Per-arm handlers, one per CommandEnvelope oneof arm -----------------

// handleDrive -- no translation needed: the decoded arm is already a
// msg::DrivetrainCommand, posted straight through, mirroring handleS()'s
// own post (git history, motion_commands.cpp -- deleted 097-006).
void handleDrive(const msg::DrivetrainCommand& cmd, Rt::Blackboard& b, uint32_t corrId,
                 ReplyFn replyFn, void* replyCtx) {
  b.driveIn.post(cmd);
  sendAck(b, corrId, replyFn, replyCtx);
}

// handleSegment -- queue one segment. The ERR_FULL reply mirrors
// handleMove()'s own `ERR full` text behavior -- not field-specific (0),
// the same way handleMove's ERR carries no detail token for this case.
void handleSegment(const msg::MotionSegment& src, Rt::Blackboard& b, uint32_t corrId,
                   ReplyFn replyFn, void* replyCtx) {
  if (!b.segmentIn.post(toSegment(src))) {
    sendError(msg::ErrCode::ERR_FULL, 0, corrId, replyFn, replyCtx);
    return;
  }
  sendAck(b, corrId, replyFn, replyCtx);
}

// handleReplace -- Mailbox<Motion::Segment>::post() always succeeds
// (latest-wins), mirroring handleMover()'s own unchecked post.
void handleReplace(const msg::MotionSegment& src, Rt::Blackboard& b, uint32_t corrId,
                   ReplyFn replyFn, void* replyCtx) {
  b.replaceIn.post(toSegment(src));
  sendAck(b, corrId, replyFn, replyCtx);
}

// handleStop -- byte-identical to the text handleStop()'s own construction
// (commands/text_channel.cpp) -- Decision 3: NOT derived from any
// caller-supplied field (Stop{} has none).
void handleStop(Rt::Blackboard& b, uint32_t corrId, ReplyFn replyFn, void* replyCtx) {
  msg::DrivetrainCommand cmd;
  cmd.setNeutral(msg::Neutral::BRAKE);
  b.driveIn.post(cmd);
  sendAck(b, corrId, replyFn, replyCtx);
}

// handlePing -- robot-clock timestamp for clock-sync parity with text
// PING's own `OK pong t=<ms>` reply (Types::systemClockNow(), matching
// the text handlePing() exactly -- commands/text_channel.cpp). No
// Blackboard post.
void handlePing(uint32_t corrId, ReplyFn replyFn, void* replyCtx) {
  msg::ReplyEnvelope reply;
  reply.corr_id = corrId;
  reply.body_kind = msg::ReplyEnvelope::BodyKind::OK;
  reply.body.ok.q = 0;
  reply.body.ok.rem = 0.0f;
  reply.body.ok.t = Types::systemClockNow();
  sendReply(reply, replyFn, replyCtx);
}

// handleEcho -- echoes the payload back verbatim, mirroring the text
// handleEcho()'s behavior (reassemble and echo payload). No Blackboard
// post.
void handleEcho(const msg::Echo& echo, uint32_t corrId, ReplyFn replyFn, void* replyCtx) {
  msg::ReplyEnvelope reply;
  reply.corr_id = corrId;
  reply.body_kind = msg::ReplyEnvelope::BodyKind::ECHO;
  reply.body.echo.payload_count = echo.payload_count;
  std::memcpy(reply.body.echo.payload_, echo.payload_, echo.payload_count);
  sendReply(reply, replyFn, replyCtx);
}

// handleId -- sources model/name/serial/fw/proto from the SAME
// deviceIdentity() helper handleId()/formatDeviceAnnouncement() already use
// (commands/text_channel.{h,cpp}) -- never a second #ifdef HOST_BUILD
// branch. No Blackboard post.
//
// Reused verbatim for the `hello`/`ver` request arms too (stakeholder-
// directed 6-verb minimal command surface, 2026-07-10; see this file's
// handle() dispatch switch below): DeviceId already carries every field
// both HELLO's announcement and VER's fw/proto content need (Decision 4's
// reasoning extended), so hello/ver/id all reply the identical
// ReplyEnvelope{id: DeviceId{...}} -- the CLIENT (rogo/the proxy) decides
// how to RENDER that shared payload back into HELLO's `DEVICE:...` banner,
// ID's own `ID model=...` line, or VER's `OK ver fw=... proto=...` line;
// none of that rendering is a firmware concern.
void handleId(uint32_t corrId, ReplyFn replyFn, void* replyCtx) {
  const char* name;
  uint32_t serial;
  deviceIdentity(&name, &serial);

  msg::ReplyEnvelope reply;
  reply.corr_id = corrId;
  reply.body_kind = msg::ReplyEnvelope::BodyKind::ID;
  std::strncpy(reply.body.id.model, "NEZHA2", sizeof(reply.body.id.model) - 1);
  std::strncpy(reply.body.id.name, name, sizeof(reply.body.id.name) - 1);
  reply.body.id.serial = serial;
  std::strncpy(reply.body.id.fw_version, FIRMWARE_VERSION, sizeof(reply.body.id.fw_version) - 1);
  reply.body.id.proto_version = static_cast<uint32_t>(PROTO_VERSION);
  sendReply(reply, replyFn, replyCtx);
}

// handleHelp -- HELP's binary reply: HelpText{text}, sourced from the SAME
// Rt::CommandRouter::listVerbs() the text HELP handler reads
// (text_channel.cpp's own handleHelp()) -- never a second, separately-
// maintained verb list, and never out of sync with what textCommands()
// actually registers. `routerCtx` is the SAME opaque
// handlerCtx-cast-to-Rt::CommandRouter* this whole file's dispatch already
// threads through (Decision 1) -- reached here via `handle()`'s own
// `routerCtx` parameter, not a second pointer this file stores.
void handleHelp(uint32_t corrId, void* routerCtx, ReplyFn replyFn, void* replyCtx) {
  msg::ReplyEnvelope reply;
  reply.corr_id = corrId;
  reply.body_kind = msg::ReplyEnvelope::BodyKind::HELPTEXT;
  static_cast<Rt::CommandRouter*>(routerCtx)
      ->listVerbs(reply.body.helptext.text, sizeof(reply.body.helptext.text));
  sendReply(reply, replyFn, replyCtx);
}

// --- CONFIG patch handlers (096-004, Decision 3) --------------------------
// Hand-translate the ONE populated Patch's present (Opt<T>.has) fields into
// a freshly-built Rt::ConfigDelta{target, mask, value} -- one "if (has) {
// field = val; mask |= bitOf(...); }" per field, mirroring the now-deleted
// text SET handler's own applyConfigKey() (config_commands.cpp, removed
// 097-007) per-key assignment shape exactly. No strcmp dispatch (the
// oneof's own patch_kind discriminant replaces it) and no hand parsing/
// range checks (the generated decoder's min/max/abs_max/req validation
// already ran during decode()).

void handleConfigDrivetrain(const msg::DrivetrainConfigPatch& p, Rt::Blackboard& b,
                            uint32_t corrId, ReplyFn replyFn, void* replyCtx) {
  Rt::ConfigDelta delta;
  delta.target = Rt::ConfigDelta::kDrivetrain;
  if (p.trackwidth.has) {
    delta.drivetrain.trackwidth = p.trackwidth.val;
    delta.mask |= Rt::bitOf(Rt::DrivetrainConfigField::kTrackwidth);
  }
  if (p.rotational_slip.has) {
    delta.drivetrain.rotational_slip = p.rotational_slip.val;
    delta.mask |= Rt::bitOf(Rt::DrivetrainConfigField::kRotationalSlip);
  }
  if (p.ekf_q_xy.has) {
    delta.drivetrain.ekf_q_xy = p.ekf_q_xy.val;
    delta.mask |= Rt::bitOf(Rt::DrivetrainConfigField::kEkfQXy);
  }
  if (p.ekf_q_theta.has) {
    delta.drivetrain.ekf_q_theta = p.ekf_q_theta.val;
    delta.mask |= Rt::bitOf(Rt::DrivetrainConfigField::kEkfQTheta);
  }
  if (p.ekf_r_otos_xy.has) {
    delta.drivetrain.ekf_r_otos_xy = p.ekf_r_otos_xy.val;
    delta.mask |= Rt::bitOf(Rt::DrivetrainConfigField::kEkfROtosXy);
  }
  if (p.ekf_r_otos_theta.has) {
    delta.drivetrain.ekf_r_otos_theta = p.ekf_r_otos_theta.val;
    delta.mask |= Rt::bitOf(Rt::DrivetrainConfigField::kEkfROtosTheta);
  }
  // 099-008: ekf_r_fix_xy/ekf_r_fix_theta -- mirrors the two ekf_r_otos_*
  // blocks immediately above exactly.
  if (p.ekf_r_fix_xy.has) {
    delta.drivetrain.ekf_r_fix_xy = p.ekf_r_fix_xy.val;
    delta.mask |= Rt::bitOf(Rt::DrivetrainConfigField::kEkfRFixXy);
  }
  if (p.ekf_r_fix_theta.has) {
    delta.drivetrain.ekf_r_fix_theta = p.ekf_r_fix_theta.val;
    delta.mask |= Rt::bitOf(Rt::DrivetrainConfigField::kEkfRFixTheta);
  }
  if (delta.mask != 0 && !b.configIn.post(delta)) {
    sendError(msg::ErrCode::ERR_FULL, 0, corrId, replyFn, replyCtx);
    return;
  }
  sendAck(b, corrId, replyFn, replyCtx);
}

// handleConfigMotor -- Decision 5: `side` disambiguates travel_calib ONLY;
// any present Gains field (kp/ki/kff/i_max/kaw) applies to BOTH bound
// motors unconditionally -- two separate ConfigDelta posts, one per bound
// index -- mirroring applyConfigKey()'s exact both-sides pid.* behavior.
// Never a per-side Gains split.
void handleConfigMotor(const msg::MotorConfigPatch& p, Rt::Blackboard& b, uint32_t corrId,
                       ReplyFn replyFn, void* replyCtx) {
  // Same conversion boundary the now-deleted text handlers used
  // (config_commands.cpp's handleSet()/handleGet(), removed 097-007):
  // bb.drivetrainConfig.left_port/right_port are wire/serialized 1-based
  // labels, converted to 0-based Hardware motor indices here, once.
  uint32_t leftIdx = b.drivetrainConfig.left_port - 1;
  uint32_t rightIdx = b.drivetrainConfig.right_port - 1;

  if (p.travel_calib.has) {
    Rt::ConfigDelta delta;
    delta.target = Rt::ConfigDelta::kMotor;
    delta.port = (p.side == msg::BoundMotorSide::RIGHT) ? rightIdx : leftIdx;
    delta.motor.travel_calib = p.travel_calib.val;
    delta.mask = Rt::bitOf(Rt::MotorConfigField::kTravelCalib);
    if (!b.configIn.post(delta)) {
      sendError(msg::ErrCode::ERR_FULL, 0, corrId, replyFn, replyCtx);
      return;
    }
  }

  uint64_t gainsMask = 0;
  msg::MotorConfig gains;
  if (p.kp.has) {
    gains.vel_gains.kp = p.kp.val;
    gainsMask |= Rt::bitOf(Rt::MotorConfigField::kVelGainsKp);
  }
  if (p.ki.has) {
    gains.vel_gains.ki = p.ki.val;
    gainsMask |= Rt::bitOf(Rt::MotorConfigField::kVelGainsKi);
  }
  if (p.kff.has) {
    gains.vel_gains.kff = p.kff.val;
    gainsMask |= Rt::bitOf(Rt::MotorConfigField::kVelGainsKff);
  }
  if (p.i_max.has) {
    gains.vel_gains.i_max = p.i_max.val;
    gainsMask |= Rt::bitOf(Rt::MotorConfigField::kVelGainsIMax);
  }
  if (p.kaw.has) {
    gains.vel_gains.kaw = p.kaw.val;
    gainsMask |= Rt::bitOf(Rt::MotorConfigField::kVelGainsKaw);
  }
  if (gainsMask != 0) {
    Rt::ConfigDelta deltaLeft;
    deltaLeft.target = Rt::ConfigDelta::kMotor;
    deltaLeft.port = leftIdx;
    deltaLeft.motor = gains;
    deltaLeft.mask = gainsMask;
    Rt::ConfigDelta deltaRight;
    deltaRight.target = Rt::ConfigDelta::kMotor;
    deltaRight.port = rightIdx;
    deltaRight.motor = gains;
    deltaRight.mask = gainsMask;
    if (!b.configIn.post(deltaLeft) || !b.configIn.post(deltaRight)) {
      sendError(msg::ErrCode::ERR_FULL, 0, corrId, replyFn, replyCtx);
      return;
    }
  }
  sendAck(b, corrId, replyFn, replyCtx);
}

void handleConfigPlanner(const msg::PlannerConfigPatch& p, Rt::Blackboard& b, uint32_t corrId,
                         ReplyFn replyFn, void* replyCtx) {
  Rt::ConfigDelta delta;
  delta.target = Rt::ConfigDelta::kPlanner;
  if (p.min_speed.has) {
    delta.planner.min_speed = p.min_speed.val;
    delta.mask |= Rt::bitOf(Rt::PlannerConfigField::kMinSpeed);
  }
  // 098-005: heading_kp/heading_kd -- the outer heading-loop PD gains
  // (098-001/098-002) -- mirror min_speed's own has/val -> planner field +
  // mask-bit shape exactly. No legacy text key to mirror (config_commands.cpp
  // was already deleted, 097-007, before these two fields existed) -- see
  // protos/config.proto's PlannerConfigPatch comment.
  if (p.heading_kp.has) {
    delta.planner.heading_kp = p.heading_kp.val;
    delta.mask |= Rt::bitOf(Rt::PlannerConfigField::kHeadingKp);
  }
  if (p.heading_kd.has) {
    delta.planner.heading_kd = p.heading_kd.val;
    delta.mask |= Rt::bitOf(Rt::PlannerConfigField::kHeadingKd);
  }
  if (delta.mask != 0 && !b.configIn.post(delta)) {
    sendError(msg::ErrCode::ERR_FULL, 0, corrId, replyFn, replyCtx);
    return;
  }
  sendAck(b, corrId, replyFn, replyCtx);
}

// handleConfigWatchdog -- Open Question 4 (096): sTimeout is NOT one of the
// Configurator's four fold targets -- posts straight to
// bb.streamWatchdogWindowIn (bb.streamWatchdogWindow's producer mailbox --
// the StreamingDriveWatchdog class this comment used to name as its
// consumer was itself already-dead code and was deleted outright, 097-006;
// see text_channel.h's own Section 1 file header (formerly
// motion_commands.h's). Nothing currently drains this
// mailbox into a live watchdog), never bb.configIn, mirroring the
// now-deleted text handler's own sTimeout special-case and file-header note
// that sTimeout is "the one key that is NOT one of the Configurator's four
// targets" (config_commands.cpp/.h, removed 097-007).
void handleConfigWatchdog(uint32_t window,  // [ms]
                          Rt::Blackboard& b, uint32_t corrId, ReplyFn replyFn,
                          void* replyCtx) {
  b.streamWatchdogWindowIn.post(window);
  sendAck(b, corrId, replyFn, replyCtx);
}

// handleConfig -- patch_kind dispatch for the CONFIG arm; the oneof's own
// discriminant replaces the text plane's strcmp key dispatch.
void handleConfig(const msg::ConfigDelta& patch, Rt::Blackboard& b, uint32_t corrId,
                  ReplyFn replyFn, void* replyCtx) {
  switch (patch.patch_kind) {
    case msg::ConfigDelta::PatchKind::DRIVETRAIN:
      handleConfigDrivetrain(patch.patch.drivetrain, b, corrId, replyFn, replyCtx);
      break;
    case msg::ConfigDelta::PatchKind::MOTOR:
      handleConfigMotor(patch.patch.motor, b, corrId, replyFn, replyCtx);
      break;
    case msg::ConfigDelta::PatchKind::PLANNER:
      handleConfigPlanner(patch.patch.planner, b, corrId, replyFn, replyCtx);
      break;
    case msg::ConfigDelta::PatchKind::WATCHDOG:
      handleConfigWatchdog(patch.patch.watchdog, b, corrId, replyFn, replyCtx);
      break;
    case msg::ConfigDelta::PatchKind::NONE:
    default:
      // No Patch populated at all -- not field-specific beyond "which arm"
      // (field 6, CommandEnvelope.cmd.config's own field number, mirroring
      // this file's declared-only-arm error convention).
      sendError(msg::ErrCode::ERR_UNKNOWN, 6, corrId, replyFn, replyCtx);
      break;
  }
}

// handleGet -- config snapshot read-back (096-004). ConfigGet.target is
// `optional` + `(req)=true` (ticket 001) -- the generated decoder already
// rejected an envelope missing it (ERR_BADARG field=1) before dispatch
// ever reached here, so get.target.has is guaranteed true.
void handleGet(const msg::ConfigGet& get, Rt::Blackboard& b, uint32_t corrId,
               ReplyFn replyFn, void* replyCtx) {
  const msg::ConfigTarget target = get.target.val;
  // Same conversion boundary the now-deleted text handler's handleGet()
  // used (config_commands.cpp, removed 097-007): bb.drivetrainConfig.
  // left_port/right_port are wire/serialized 1-based labels, converted to
  // 0-based Hardware motor indices here, once.
  uint32_t leftIdx = b.drivetrainConfig.left_port - 1;
  uint32_t rightIdx = b.drivetrainConfig.right_port - 1;

  msg::ConfigSnapshot cfg;
  cfg.target = target;

  switch (target) {
    case msg::ConfigTarget::CONFIG_DRIVETRAIN: {
      cfg.patch_kind = msg::ConfigSnapshot::PatchKind::DRIVETRAIN;
      msg::DrivetrainConfigPatch& p = cfg.patch.drivetrain;
      p.trackwidth = {true, b.drivetrainConfig.trackwidth};
      p.rotational_slip = {true, b.drivetrainConfig.rotational_slip};
      p.ekf_q_xy = {true, b.drivetrainConfig.ekf_q_xy};
      p.ekf_q_theta = {true, b.drivetrainConfig.ekf_q_theta};
      p.ekf_r_otos_xy = {true, b.drivetrainConfig.ekf_r_otos_xy};
      p.ekf_r_otos_theta = {true, b.drivetrainConfig.ekf_r_otos_theta};
      // 099-008: mirrors the two ekf_r_otos_* read-backs immediately above.
      p.ekf_r_fix_xy = {true, b.drivetrainConfig.ekf_r_fix_xy};
      p.ekf_r_fix_theta = {true, b.drivetrainConfig.ekf_r_fix_theta};
      break;
    }
    case msg::ConfigTarget::CONFIG_MOTOR_LEFT:
    case msg::ConfigTarget::CONFIG_MOTOR_RIGHT: {
      bool isLeft = (target == msg::ConfigTarget::CONFIG_MOTOR_LEFT);
      const msg::MotorConfig& m = b.motorConfig[isLeft ? leftIdx : rightIdx];
      cfg.patch_kind = msg::ConfigSnapshot::PatchKind::MOTOR;
      msg::MotorConfigPatch& p = cfg.patch.motor;
      p.side = isLeft ? msg::BoundMotorSide::LEFT : msg::BoundMotorSide::RIGHT;
      p.travel_calib = {true, m.travel_calib};
      p.kp = {true, m.vel_gains.kp};
      p.ki = {true, m.vel_gains.ki};
      p.kff = {true, m.vel_gains.kff};
      p.i_max = {true, m.vel_gains.i_max};
      p.kaw = {true, m.vel_gains.kaw};
      break;
    }
    case msg::ConfigTarget::CONFIG_PLANNER: {
      cfg.patch_kind = msg::ConfigSnapshot::PatchKind::PLANNER;
      cfg.patch.planner.min_speed = {true, b.plannerConfig.min_speed};
      // 098-005: heading_kp/heading_kd -- mirrors min_speed's own
      // always-has=true read-back exactly (a GET snapshot is a full read of
      // the target's current state, not a partial patch -- see this
      // function's own doc comment above). Without this, host protocol.py's
      // _PLANNER_KEYS-driven get_config("headingKp"/"headingKd") would
      // silently read back a permanent 0.0 regardless of the live value.
      cfg.patch.planner.heading_kp = {true, b.plannerConfig.heading_kp};
      cfg.patch.planner.heading_kd = {true, b.plannerConfig.heading_kd};
      break;
    }
    case msg::ConfigTarget::CONFIG_WATCHDOG: {
      cfg.patch_kind = msg::ConfigSnapshot::PatchKind::WATCHDOG;
      cfg.patch.watchdog = b.streamWatchdogWindow;
      break;
    }
    default:
      // Not one of the 5 known ConfigTarget enumerators -- the generated
      // decoder has no enum-range validation to catch this (no
      // (min)/(max)/(abs_max) on an enum field), so it is caught here.
      // field 1 = ConfigGet.target's own field number.
      sendError(msg::ErrCode::ERR_UNKNOWN, 1, corrId, replyFn, replyCtx);
      return;
  }

  msg::ReplyEnvelope reply;
  reply.corr_id = corrId;
  reply.body_kind = msg::ReplyEnvelope::BodyKind::CFG;
  reply.body.cfg = cfg;
  sendReply(reply, replyFn, replyCtx);
}

// handlePose -- 099-004/099-008: CommandEnvelope.cmd.pose_fix's dispatch
// (D5-D8, architecture-update.md). `reset`/`zero_encoders` reuse
// PoseEstimator's existing, already-tested setPose()/resetEncoderBaseline()
// dispatch through the UNCHANGED bb.poseResetIn queue -- no new PoseEstimator
// code path for SI/ZERO (both may be set in one message; both branches run).
// Neither flag set is a genuine timestamped delayed camera fix -- 099-008
// makes this branch live, posting Rt::PoseFixCommand{fix.x, fix.y, fix.h,
// fix.t} to bb.poseFixIn (a latest-wins Mailbox, D7 -- post() always
// succeeds, so there is no ERR_FULL path for this branch, unlike
// poseResetIn's bounded WorkQueue below). Subsystems::PoseEstimator::tick()
// (its own 7th parameter) drains it, interpolates/composes against its
// private pose-history ring, and applies an UNGATED EkfTiny correction (D5
// -- the camera is authoritative; ticket 006's OTOS-only innovation gate
// never applies here). Replaces the ERR_UNIMPLEMENTED stub ticket 004 left
// in place for this branch.
void handlePose(const msg::PoseFix& fix, Rt::Blackboard& b, uint32_t corrId,
                ReplyFn replyFn, void* replyCtx) {
  if (!fix.reset && !fix.zero_encoders) {
    Rt::PoseFixCommand cmd;
    cmd.x = fix.x;
    cmd.y = fix.y;
    cmd.h = fix.h;
    cmd.t = fix.t;
    b.poseFixIn.post(cmd);
    sendAck(b, corrId, replyFn, replyCtx);
    return;
  }

  bool posted = true;
  if (fix.reset) {
    Rt::PoseResetCommand cmd;
    cmd.kind = Rt::PoseResetCommand::kSetPose;
    cmd.pose.x = fix.x;
    cmd.pose.y = fix.y;
    cmd.pose.h = fix.h;
    posted = b.poseResetIn.post(cmd) && posted;
  }
  if (fix.zero_encoders) {
    Rt::PoseResetCommand cmd;
    cmd.kind = Rt::PoseResetCommand::kResetBaseline;
    posted = b.poseResetIn.post(cmd) && posted;
  }
  if (!posted) {
    sendError(msg::ErrCode::ERR_FULL, 0, corrId, replyFn, replyCtx);
    return;
  }
  sendAck(b, corrId, replyFn, replyCtx);
}

// handleStream -- 096-005: mirrors the text handleStream()'s own
// state-setting exactly (git history, telemetry_commands.cpp -- the text
// STREAM/SNAP handlers were deleted outright by 097-008) -- minus the text
// ArgSchema/ArgList parsing layer, which the generated decoder's own
// (min)/(max) validation already replaced (StreamControl.period is
// wire-bounded [0, 60000], same range as kStreamArgs). kStreamFloorMs was
// duplicated here rather than shared: the now-deleted text handler kept it
// TU-local (unnamed namespace), and this file already hand-mirrors
// handleStream()'s state-setting rather than reaching across TUs for it
// (same pattern as toSegment()'s own field-by-field copy).
void handleStream(const msg::StreamControl& sc, Rt::Blackboard& b, void* routerCtx,
                  uint32_t corrId, ReplyFn replyFn, void* replyCtx) {
  constexpr uint32_t kStreamFloorMs = 20;  // [ms] docs/protocol-v2.md §8
  b.telemetryPeriod = (sc.period == 0) ? 0
                       : (sc.period < kStreamFloorMs ? kStreamFloorMs : sc.period);
  // Channel binding (docs/protocol-v2.md §8): the SAME routerCtx idiom
  // every other handler in this file uses to reach the Blackboard
  // (Decision 1) -- resolved from the CommandRouter currently dispatching
  // this envelope (currentChannel()), never a captured ReplyFn/void* pair,
  // mirroring handleStream()'s own comment on why. Rebound unconditionally,
  // even for period 0 (disabling still records "this channel asked last").
  b.telemetryChannel = static_cast<Rt::CommandRouter*>(routerCtx)->currentChannel();
  b.telemetryBinary = sc.binary;
  // Deliberately NOT reproducing handleStream()'s old same-reply
  // "immediate first frame" concatenation (Open Question 5, ticket 002's
  // own note) -- the first frame arrives one pass later via
  // tickTelemetry()'s normal !telemetryHasLastEmit trigger, uniformly for
  // text and binary. sendAck mirrors drive/segment/replace/stop's own ack
  // shape; stream does not get a bespoke reply shape.
  sendAck(b, corrId, replyFn, replyCtx);
}

}  // namespace

void handle(const char* line, ReplyFn replyFn, void* replyCtx, void* routerCtx) {
  // --- Dearmor: strip "*B", trim trailing whitespace, base64-decode. ---
  // Callers only ever reach here with line[0] == '*' (CommandProcessor::
  // process()'s own branch) -- line[1] != 'B' is still a real possibility
  // (a malformed/future-armor line) and must be rejected cleanly, not
  // assumed away.
  if (line[0] != '*' || line[1] != 'B') {
    sendError(msg::ErrCode::ERR_DECODE, 0, 0, replyFn, replyCtx);
    return;
  }

  const char* b64 = line + 2;
  size_t b64Len = std::strlen(b64);
  while (b64Len > 0 && (b64[b64Len - 1] == '\r' || b64[b64Len - 1] == '\n' ||
                        b64[b64Len - 1] == ' ' || b64[b64Len - 1] == '\t')) {
    --b64Len;
  }

  uint8_t rawBuf[kMaxEnvelopeBytes];
  size_t rawLen = 0;
  if (!WireRuntime::base64Decode(b64, b64Len, rawBuf, sizeof(rawBuf), &rawLen)) {
    sendError(msg::ErrCode::ERR_DECODE, 0, 0, replyFn, replyCtx);
    return;
  }

  // --- Decode: walk the generated field table, validating bounds inline. ---
  msg::CommandEnvelope env;
  const msg::wire::Result r = msg::wire::decode(env, rawBuf, static_cast<uint16_t>(rawLen));
  if (!r.ok) {
    // env.corr_id may or may not have been populated before the failing
    // field, depending on wire order -- best effort (0 if never reached),
    // matching every real protobuf encoder's field-ascending emission
    // order in practice (corr_id is field 1).
    sendError(r.code, r.field, env.corr_id, replyFn, replyCtx);
    return;
  }

  // --- Dispatch: one call per oneof arm; every helper replies exactly
  //     once. The Blackboard is reached through the SAME opaque-handlerCtx
  //     idiom every text command family already uses (Decision 1). ---
  Rt::Blackboard& b = bb(routerCtx);

  switch (env.cmd_kind) {
    case msg::CommandEnvelope::CmdKind::DRIVE:
      handleDrive(env.cmd.drive, b, env.corr_id, replyFn, replyCtx);
      break;
    case msg::CommandEnvelope::CmdKind::SEGMENT:
      handleSegment(env.cmd.segment, b, env.corr_id, replyFn, replyCtx);
      break;
    case msg::CommandEnvelope::CmdKind::REPLACE:
      handleReplace(env.cmd.replace, b, env.corr_id, replyFn, replyCtx);
      break;
    case msg::CommandEnvelope::CmdKind::STOP:
      handleStop(b, env.corr_id, replyFn, replyCtx);
      break;
    case msg::CommandEnvelope::CmdKind::PING:
      handlePing(env.corr_id, replyFn, replyCtx);
      break;
    case msg::CommandEnvelope::CmdKind::ECHO:
      handleEcho(env.cmd.echo, env.corr_id, replyFn, replyCtx);
      break;
    case msg::CommandEnvelope::CmdKind::ID:
    case msg::CommandEnvelope::CmdKind::HELLO:
    case msg::CommandEnvelope::CmdKind::VER:
      // hello/ver (stakeholder-directed 6-verb minimal command surface,
      // 2026-07-10) reuse handleId()'s identical DeviceId reply -- see that
      // function's own doc comment above.
      handleId(env.corr_id, replyFn, replyCtx);
      break;
    case msg::CommandEnvelope::CmdKind::HELP:
      handleHelp(env.corr_id, routerCtx, replyFn, replyCtx);
      break;
    case msg::CommandEnvelope::CmdKind::CONFIG:
      handleConfig(env.cmd.config, b, env.corr_id, replyFn, replyCtx);
      break;
    case msg::CommandEnvelope::CmdKind::POSE_FIX:
      handlePose(env.cmd.pose_fix, b, env.corr_id, replyFn, replyCtx);
      break;
    case msg::CommandEnvelope::CmdKind::OTOS:
      sendError(msg::ErrCode::ERR_UNIMPLEMENTED, 8, env.corr_id, replyFn, replyCtx);
      break;
    case msg::CommandEnvelope::CmdKind::GET:
      handleGet(env.cmd.get, b, env.corr_id, replyFn, replyCtx);
      break;
    case msg::CommandEnvelope::CmdKind::STREAM:
      handleStream(env.cmd.stream, b, routerCtx, env.corr_id, replyFn, replyCtx);
      break;
    case msg::CommandEnvelope::CmdKind::NONE:
    default:
      // No oneof arm set at all (an envelope with only corr_id, or a
      // reserved/unknown field number decode() correctly skipped) -- not
      // field-specific.
      sendError(msg::ErrCode::ERR_UNKNOWN, 0, env.corr_id, replyFn, replyCtx);
      break;
  }
}

}  // namespace BinaryChannel
