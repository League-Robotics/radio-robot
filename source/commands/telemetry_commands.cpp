// telemetry_commands.cpp -- STREAM/SNAP command handlers + telemetryEmit().
// See telemetry_commands.h for the full vocabulary, the field-sourcing
// rules (Decision 7), and the ROBOT_DEV_BUILD gating rationale (matches
// dev_commands.h/.cpp's own gating -- this family depends on
// Subsystems::Hardware/Drivetrain/PoseEstimator, all DEV-tier).
#include "commands/telemetry_commands.h"

#if ROBOT_DEV_BUILD

#include "commands/command_processor.h"
#include "kinematics/body_kinematics.h"
#include "telemetry/tlm_frame.h"
#include "types/clock.h"

namespace {

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
  TelemetryState& state = *static_cast<TelemetryState*>(handlerCtx);
  uint32_t requested = static_cast<uint32_t>(args.args[0].ival);
  uint32_t period = (requested == 0) ? 0
                    : (requested < kStreamFloorMs ? kStreamFloorMs : requested);

  state.periodMs = period;
  // Channel binding (docs/protocol-v2.md §8): the periodic-emission reply
  // channel is whichever channel issued the most recently accepted STREAM
  // command -- rebound unconditionally, even for STREAM 0 (disabling still
  // records "this channel asked last").
  state.replyFn = replyFn;
  state.replyCtx = replyCtx;

  char rbuf[48];
  CommandProcessor::replyOKf(rbuf, sizeof(rbuf), "stream", corrId, replyFn, replyCtx,
                             "period=%u", static_cast<unsigned>(period));
}

// ---------------------------------------------------------------------------
// SNAP -- no arguments, no schema (parseFn = nullptr), mirroring how
// PING/VER register (system_commands.cpp). Replies on its OWN dispatch
// replyFn/replyCtx (the channel SNAP itself arrived on) -- NOT
// state.replyFn/state.replyCtx (the STREAM-bound channel); only state.seq is
// shared between the two verbs -- see telemetry_commands.h's header comment.
// ---------------------------------------------------------------------------
void handleSnap(const ArgList& /*args*/, const char* /*corrId*/,
                ReplyFn replyFn, void* replyCtx, void* handlerCtx) {
  TelemetryState& state = *static_cast<TelemetryState*>(handlerCtx);
  uint32_t now = Types::systemClockNow();
  telemetryEmit(state, now, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// modeChar -- 084-005: maps Subsystems::Planner::state().mode (msg::
// DriveMode) to TLM's single-character `mode=` wire value, per
// docs/protocol-v2.md §8's I/S/T/D/G vocabulary and architecture-update.md
// (084) Decision 6. IDLE/STREAMING/TIMED/DISTANCE/GO_TO each map to their
// own documented character directly -- Decision 6's self-terminating-vs-
// open-ended collapse (which is what lets STREAMING/TIMED each be shared by
// more than one wire verb) happens entirely inside planner.cpp's
// velocityShapedMode(), before this function ever sees the value; this
// switch has no knowledge of, and does not need, which verb produced a
// given mode. `VELOCITY` is no longer emitted by planner.cpp at all (see
// velocityShapedMode()'s own doc comment) -- kept here only as a defensive
// fallback (mapped to 'I', matching the "no active command" default) rather
// than assuming that invariant holds forever.
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

void telemetryEmit(TelemetryState& state, uint32_t now, ReplyFn replyFn, void* replyCtx) {
  // Mirrors dev_loop.cpp's own null-fn guard on the watchdog-fire EVT reply:
  // a channel that never bound a replyFn must not dereference a null
  // function pointer. state.seq is left untouched -- no frame was emitted.
  if (replyFn == nullptr) return;

  // --- Field sourcing (Decision 7) -- see telemetry_commands.h's header
  // comment for the full rule table. ---
  Subsystems::DrivetrainPorts p = state.drivetrain->ports();
  Hal::Motor& left = state.hardware->motor(p.left);
  Hal::Motor& right = state.hardware->motor(p.right);

  // enc=/vel= read Hal::Motor's primitive getters DIRECTLY -- never
  // Drivetrain::state()'s vel_[] (commanded targets, a different semantic).
  // This file does not include or reference Drivetrain::state() at all.
  float velLeft = left.velocity();
  float velRight = right.velocity();

  Telemetry::TlmFrameInput in;
  in.now = now;
  // mode= -- 084-005: Subsystems::Planner::state().mode is the SOLE source
  // (architecture-update.md (084) Decision 6; see this file's header
  // comment) -- no longer drivetrain.active() (082's minimal I/S-only
  // precedent). Reading state() fresh here (never cached) also satisfies
  // the ticket's polling requirement: mode= reflects Planner's true current
  // mode independent of whether any EVT has been drained.
  in.mode = modeChar(state.planner->state().mode);
  in.seq = state.seq++;   // shared by every STREAM-driven frame AND SNAP

  in.hasEnc = true;
  in.encLeft = left.position();
  in.encRight = right.position();

  in.hasVel = true;
  in.velLeft = velLeft;
  in.velRight = velRight;

  // pose=/encpose= read PoseEstimator's two independent readings (ticket
  // 002/003) -- never Drivetrain::state() either.
  in.hasPose = true;
  in.pose = state.poseEstimator->fusedPose().pose;

  in.hasEncPose = true;
  in.encPose = state.poseEstimator->encoderPose().pose;

  // otos= -- the raw sampled odometer pose (ticket 003's Hardware::odometer()
  // seam), OMITTED (not zero-filled) when no odometer is present.
  Hal::Odometer* odometer = state.hardware->odometer();
  if (odometer != nullptr) {
    in.hasOtos = true;
    in.otos = odometer->pose().pose;
  }

  // twist= -- a pure kinematic transform (BodyKinematics::forward()) of the
  // SAME directly-read wheel velocities vel= uses, plus PoseEstimator's own
  // configured trackwidth. Directly-measured/derived, never Drivetrain::
  // state(), never EKF velocity-channel state (EkfTiny implements none --
  // see estimation/ekf_tiny.h's file header).
  in.hasTwist = true;
  BodyKinematics::forward(velLeft, velRight, state.poseEstimator->trackwidth(),
                           in.twist.v_x, in.twist.omega);
  in.twist.v_y = 0.0f;   // differential-only this sprint -- see drivetrain.h

  char buf[300];
  Telemetry::buildTlmFrame(buf, sizeof(buf), in);
  replyFn(buf, replyCtx);
}

std::vector<CommandDescriptor> telemetryCommands(TelemetryState& state) {
  std::vector<CommandDescriptor> cmds;
  cmds.push_back(makeSchemaCmd("STREAM", &kStreamSchema, handleStream, &state,
                               "badarg", ForceReply::NONE, CMD_NONE));
  cmds.push_back(makeCmd("SNAP", nullptr, handleSnap, &state,
                         "badarg", ForceReply::NONE, CMD_NONE));
  return cmds;
}

#endif  // ROBOT_DEV_BUILD
