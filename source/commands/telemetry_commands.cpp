// telemetry_commands.cpp -- STREAM/SNAP command handlers + telemetryEmit().
// See telemetry_commands.h for the full vocabulary, the field-sourcing
// rules (Decision 7), and the ROBOT_DEV_BUILD gating rationale.
#include "commands/telemetry_commands.h"

#if ROBOT_DEV_BUILD

#include "commands/command_processor.h"
#include "kinematics/body_kinematics.h"
#include "telemetry/tlm_frame.h"
#include "types/clock.h"

namespace {

Rt::Blackboard& bb(void* handlerCtx) { return static_cast<Rt::CommandRouter*>(handlerCtx)->blackboard(); }

// ---------------------------------------------------------------------------
// STREAM <ms> -- pure fixed-shape `<verb> <int>` command, so it uses
// ArgSchema (same mixed hand-rolled/schema approach dev_commands.h's Open
// Question 3 documents for DEV WD <window>). The 20ms floor is enforced by
// handleStream() below, not the schema -- STREAM 10 must be ACCEPTED and
// clamped to 20, not rejected as out-of-range.
// ---------------------------------------------------------------------------
const ArgDef kStreamArgs[] = {
    { "period", ArgKind::INT, true, 0, 60000 },
};
const ArgSchema kStreamSchema = { kStreamArgs, 1, 1, false, nullptr };

constexpr uint32_t kStreamFloorMs = 20;   // [ms] docs/protocol-v2.md §8's documented minimum

void handleStream(const ArgList& args, const char* corrId,
                   ReplyFn replyFn, void* replyCtx, void* handlerCtx) {
  Rt::CommandRouter* router = static_cast<Rt::CommandRouter*>(handlerCtx);
  Rt::Blackboard& b = router->blackboard();
  uint32_t requested = static_cast<uint32_t>(args.args[0].ival);
  uint32_t period = (requested == 0) ? 0
                    : (requested < kStreamFloorMs ? kStreamFloorMs : requested);

  b.telemetryPeriod = period;
  // Channel binding (docs/protocol-v2.md §8): the periodic-emission reply
  // channel is whichever channel issued the most recently accepted STREAM
  // command -- rebound unconditionally, even for STREAM 0 (disabling still
  // records "this channel asked last"). Resolved via CommandRouter's
  // currentChannel() (the statement CommandRouter::route() is currently
  // dispatching), never a captured ReplyFn/void* pair -- see
  // telemetry_commands.h's file header.
  b.telemetryChannel = router->currentChannel();

  char rbuf[48];
  CommandProcessor::replyOKf(rbuf, sizeof(rbuf), "stream", corrId, replyFn, replyCtx,
                             "period=%u", static_cast<unsigned>(period));

  // Immediate first frame (docs/protocol-v2.md §8 / dev_loop.h's own doc
  // comment: "the very first pass after a channel issues STREAM emits
  // immediately", generalized to "or enough time has elapsed since the
  // last emission") -- concatenated into THIS SAME dispatch's own reply
  // (replyFn/replyCtx), exactly mirroring the pre-087 loop's periodic-
  // emission step, which captured this handler's OWN replyFn/replyCtx and
  // so happened to land in the SAME reply whenever it fired same-pass.
  // 087-006: bb.telemetryChannel/the loop's own resolveTelemetryReply()
  // cannot reproduce that same-reply concatenation (a Channel enum, not a
  // captured ReplyFn/void* pair -- see telemetry_commands.h's file header),
  // so this handler performs the SAME-PASS immediate emission itself,
  // directly on its own dispatch reply sink, and updates
  // bb.telemetryLastEmitMs/bb.telemetryHasLastEmit so the loop's own later
  // per-pass check (dev_loop.cpp) does not double-emit this same pass.
  uint32_t now = Types::systemClockNow();
  if (period > 0 && (!b.telemetryHasLastEmit || (now - b.telemetryLastEmitMs) >= period)) {
    telemetryEmit(b, now, replyFn, replyCtx);
    b.telemetryLastEmitMs = now;
    b.telemetryHasLastEmit = true;
  }
}

// ---------------------------------------------------------------------------
// SNAP -- no arguments, no schema (parseFn = nullptr), mirroring how
// PING/VER register (system_commands.cpp). Replies on its OWN dispatch
// replyFn/replyCtx (the channel SNAP itself arrived on) -- NOT
// bb.telemetryChannel (the STREAM-bound channel); only bb.telemetrySeq is
// shared between the two verbs -- see telemetry_commands.h's header comment.
// ---------------------------------------------------------------------------
void handleSnap(const ArgList& /*args*/, const char* /*corrId*/,
                ReplyFn replyFn, void* replyCtx, void* handlerCtx) {
  Rt::Blackboard& b = bb(handlerCtx);
  uint32_t now = Types::systemClockNow();
  telemetryEmit(b, now, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// modeChar -- 084-005: maps msg::DriveMode to TLM's single-character `mode=`
// wire value, per docs/protocol-v2.md §8's I/S/T/D/G vocabulary and
// architecture-update.md (084) Decision 6.
// ---------------------------------------------------------------------------
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

}  // namespace

void telemetryEmit(Rt::Blackboard& b, uint32_t now, ReplyFn replyFn, void* replyCtx) {
  // A channel that never bound a replyFn must not dereference a null
  // function pointer. bb.telemetrySeq is left untouched -- no frame was
  // emitted.
  if (replyFn == nullptr) return;

  // --- Field sourcing (Decision 7) -- see telemetry_commands.h's header
  // comment for the full rule table. ---
  uint32_t leftPort = b.drivetrainConfig.left_port;
  uint32_t rightPort = b.drivetrainConfig.right_port;
  const msg::MotorState& left = b.motor[leftPort - 1];
  const msg::MotorState& right = b.motor[rightPort - 1];

  // enc=/vel= read bb.motor[]'s primitive fields DIRECTLY -- never
  // bb.drivetrain's vel_[] (commanded targets, a different semantic).
  float velLeft = left.velocity.has ? left.velocity.val : 0.0f;
  float velRight = right.velocity.has ? right.velocity.val : 0.0f;

  Telemetry::TlmFrameInput in;
  in.now = now;
  // mode= -- 084-005: bb.planner.mode is the SOLE source (architecture-
  // update.md (084) Decision 6; see this file's header comment).
  in.mode = modeChar(b.planner.mode);
  in.seq = b.telemetrySeq++;   // shared by every STREAM-driven frame AND SNAP

  in.hasEnc = true;
  in.encLeft = left.position.has ? left.position.val : 0.0f;
  in.encRight = right.position.has ? right.position.val : 0.0f;

  in.hasVel = true;
  in.velLeft = velLeft;
  in.velRight = velRight;

  // pose=/encpose= read bb's two independent pose readings -- never
  // bb.drivetrain either.
  in.hasPose = true;
  in.pose = b.fusedPose.pose;

  in.hasEncPose = true;
  in.encPose = b.encoderPose.pose;

  // otos= -- the raw sampled odometer pose, OMITTED (not zero-filled) when
  // no odometer device exists at all (bb.otosPresent, a boot-time snapshot --
  // see blackboard.h's file header).
  if (b.otosPresent) {
    in.hasOtos = true;
    in.otos = b.otos.pose;
  }

  // twist= -- a pure kinematic transform (BodyKinematics::forward()) of the
  // SAME directly-read wheel velocities vel= uses, plus the SAME trackwidth
  // PoseEstimator::configure() was given (bb.drivetrainConfig.trackwidth --
  // both share msg::DrivetrainConfig, tickets 087-004/005). Directly-
  // measured/derived, never bb.drivetrain, never EKF velocity-channel state.
  in.hasTwist = true;
  BodyKinematics::forward(velLeft, velRight, b.drivetrainConfig.trackwidth,
                           in.twist.v_x, in.twist.omega);
  in.twist.v_y = 0.0f;   // differential-only this sprint -- see drivetrain.h

  char buf[300];
  Telemetry::buildTlmFrame(buf, sizeof(buf), in);
  replyFn(buf, replyCtx);
}

std::vector<CommandDescriptor> telemetryCommands(Rt::CommandRouter& router) {
  std::vector<CommandDescriptor> cmds;
  cmds.push_back(makeSchemaCmd("STREAM", &kStreamSchema, handleStream, &router,
                               "badarg", ForceReply::NONE, CMD_NONE));
  cmds.push_back(makeCmd("SNAP", nullptr, handleSnap, &router,
                         "badarg", ForceReply::NONE, CMD_NONE));
  return cmds;
}

#endif  // ROBOT_DEV_BUILD
