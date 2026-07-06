// pose_commands.cpp -- SI/ZERO handlers. See pose_commands.h for the
// file-level design notes (Decision 1's SI-bypasses-Drivetrain rationale,
// the synchronous/config-plane shape).
#include "commands/pose_commands.h"

#if ROBOT_DEV_BUILD

#include <cstdio>
#include <cstring>

#include "commands/arg_parse.h"
#include "commands/command_processor.h"
#include "messages/drivetrain.h"
#include "messages/odometer.h"

namespace {

// kCdegToRad -- centidegrees -> radians. Same conversion factor motion_
// commands.cpp's own kCdegToRad uses (duplicated here per this codebase's
// existing per-file convention -- there is no shared conversion-constant
// header).
constexpr float kCdegToRad = 3.14159265f / 18000.0f;

// ---------------------------------------------------------------------------
// SI -- SI <x> <y> <h> (mm, mm, cdeg). 3 mandatory INTs, ranged=false (plain
// atoi, no range check) -- ported wire shape from source_old/commands/
// SystemCommands.cpp's siSchema/handleSI, confirmed against
// host/robot_radio/testgui/operations.py's on_sync_pose()/
// build_setpose_command() convention (the one host caller this wire shape
// must match).
// ---------------------------------------------------------------------------
const ArgDef kSiDefs[3] = {
    {"x", ArgKind::INT, false, 0, 0},
    {"y", ArgKind::INT, false, 0, 0},
    {"h", ArgKind::INT, false, 0, 0},
};
const ArgSchema kSiSchema = {kSiDefs, 3, 3, false, nullptr};

// ---------------------------------------------------------------------------
// handleSI -- calls Subsystems::PoseEstimator::setPose() DIRECTLY (Decision
// 1 -- see pose_commands.h's file header); never routes through
// Drivetrain::apply()'s POSE arm. Does not itself cancel an in-flight
// Planner command (architecture-update.md (084) Open Question 4) -- a
// G/TURN in progress keeps pursuing its goal against the newly-anchored
// fused pose on its very next tick, which may produce a visible course
// correction rather than a smooth continuation -- observed, not
// specifically engineered, behavior.
//
// 084-008 gap closure: ALSO re-anchors the active Hal::Odometer (if any)
// via apply()'s SET_POSE arm, in the SAME wire dispatch -- mirrors
// source_old's own two-call handleSI (PoseEstimator reset + `hal.otos().
// setWorldPose()`). Before this, SI re-anchored PoseEstimator alone
// (ticket 007's own documented caveat -- see test_pose_commands.py's module
// docstring): the very next devLoopTick() pass fused a fresh, un-reanchored
// OTOS reading into the EKF, pulling the fused `pose=` back toward the
// odometer's own (unrelated) frame. Re-anchoring both in the same dispatch
// means that next fusion pass reads an odometer sample that ALREADY agrees
// with the just-set anchor, so the EKF update's residual is zero and
// `pose=` reads back exactly `x`,`y`,`h` too, not just `encpose=`.
// hardware.odometer() is nullptr on Subsystems::NezhaHardware -- a no-op
// there, unchanged from ticket 007's behavior on that build.
// ---------------------------------------------------------------------------
void handleSI(const ArgList& args, const char* corrId, ReplyFn replyFn, void* replyCtx,
              void* handlerCtx) {
  PoseCommandState& state = *static_cast<PoseCommandState*>(handlerCtx);
  int32_t x = args.args[0].ival;  // [mm]
  int32_t y = args.args[1].ival;  // [mm]
  int32_t h = args.args[2].ival;  // [cdeg]

  msg::SetPose pose;
  pose.x = static_cast<float>(x);
  pose.y = static_cast<float>(y);
  pose.h = static_cast<float>(h) * kCdegToRad;  // [rad] -- PoseEstimator's internal convention
  state.poseEstimator->setPose(pose);

  Hal::Odometer* odometer = state.hardware->odometer();
  if (odometer != nullptr) {
    msg::Pose2D otosPose;
    otosPose.x = pose.x;
    otosPose.y = pose.y;
    otosPose.h = pose.h;
    msg::OdometerCommand otosCmd;
    otosCmd.setSetPose(otosPose);
    odometer->apply(otosCmd);
  }

  char body[48];
  snprintf(body, sizeof(body), "x=%d y=%d h=%d", static_cast<int>(x), static_cast<int>(y),
           static_cast<int>(h));
  char rbuf[80];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "setpose", body, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// ZERO -- ZERO enc [#id] -> OK zero enc [#id] (docs/protocol-v2.md section
// 10's "### ZERO"). This ticket (084-007) implements the `enc` sub-verb
// only -- its own acceptance criteria and architecture-update.md (084) name
// no other sub-verb (`pose`/`T`/`D`, present in the pre-rebuild source_old
// grammar, are out of this ticket's scope); any statement without the
// literal `enc` token is `ERR badarg`, the same "at least one of enc/pose"
// shape source_old's own parseZero enforced, narrowed to the one sub-verb
// this ticket implements.
// ---------------------------------------------------------------------------
ParseResult parseZero(const char* const* tokens, int ntokens, const KVPair* /*kvs*/,
                       int /*nkv*/) {
  ParseResult r;
  bool hasEnc = false;
  for (int i = 0; i < ntokens; ++i) {
    if (strcmp(tokens[i], "enc") == 0) hasEnc = true;
  }
  if (!hasEnc) {
    r.ok = false;
    r.err.code = "badarg";
    r.err.detail = nullptr;
    return r;
  }
  r.ok = true;
  r.args.count = 1;
  argStr(r.args.args[0], "enc");
  r.args.suppliedCount = r.args.count;
  return r;
}

// ---------------------------------------------------------------------------
// handleZero -- resets the bound pair's encoders (Hal::Motor::
// resetPosition(), the same primitive DEV M <n> RESET already stages -- see
// this file's header comment) AND PoseEstimator's encoder-baseline
// accumulator, in the SAME call, so the next tick's delta is computed
// against the freshly-zeroed encoders -- no phantom jump (084-007's own
// acceptance criterion; pose_estimator.h's resetEncoderBaseline() doc
// comment has the full hazard description).
// ---------------------------------------------------------------------------
void handleZero(const ArgList& /*args*/, const char* corrId, ReplyFn replyFn, void* replyCtx,
                 void* handlerCtx) {
  PoseCommandState& state = *static_cast<PoseCommandState*>(handlerCtx);

  Subsystems::DrivetrainPorts ports = state.drivetrain->ports();
  state.hardware->motor(ports.left).resetPosition();
  state.hardware->motor(ports.right).resetPosition();
  state.poseEstimator->resetEncoderBaseline();

  char rbuf[48];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "zero", "enc", corrId, replyFn, replyCtx);
}

}  // namespace

std::vector<CommandDescriptor> poseCommands(PoseCommandState& state) {
  std::vector<CommandDescriptor> cmds;
  cmds.push_back(
      makeSchemaCmd("SI", &kSiSchema, handleSI, &state, "badarg", ForceReply::NONE, CMD_NONE));
  cmds.push_back(makeCmd("ZERO", parseZero, handleZero, &state, "badarg", ForceReply::NONE,
                          CMD_ACCESS_HARDWARE));
  return cmds;
}

#endif  // ROBOT_DEV_BUILD
