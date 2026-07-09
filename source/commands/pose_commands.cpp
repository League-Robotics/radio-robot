// pose_commands.cpp -- SI/ZERO handlers. See pose_commands.h for the
// file-level design notes (Decision 7's router-half fan-out, 087-006's
// pointerless reshape).
#include "commands/pose_commands.h"


#include <cstdio>
#include <cstring>

#include "commands/arg_parse.h"
#include "commands/command_processor.h"
#include "messages/drivetrain.h"

namespace {

Rt::Blackboard& bb(void* handlerCtx) { return static_cast<Rt::CommandRouter*>(handlerCtx)->blackboard(); }

// kCdegToRad -- centidegrees -> radians. Duplicated here per this codebase's
// existing per-file convention (there is no shared conversion-constant
// header).
constexpr float kCdegToRad = 3.14159265f / 18000.0f;

// ---------------------------------------------------------------------------
// SI -- SI <x> <y> <h> (mm, mm, cdeg). 3 mandatory INTs, ranged=false (plain
// atoi, no range check) -- unaffected by this rewrite (pure parsing).
// ---------------------------------------------------------------------------
const ArgDef kSiDefs[3] = {
    {"x", ArgKind::INT, false, 0, 0},
    {"y", ArgKind::INT, false, 0, 0},
    {"h", ArgKind::INT, false, 0, 0},
};
const ArgSchema kSiSchema = {kSiDefs, 3, 3, false, nullptr};

// ---------------------------------------------------------------------------
// handleSI -- posts Rt::PoseResetCommand{kSetPose} to bb.poseResetIn
// (Decision 1 unchanged: never routes through Drivetrain -- Drivetrain holds
// no PoseEstimator reference by design and must not gain one just for this)
// AND the same pose to bb.otosSetPoseIn, in the SAME wire dispatch, so the
// very next fusion pass reads an odometer sample that already agrees with
// the freshly-set anchor (mirrors the pre-087 two-call handleSI() -- see
// pose_commands.h's file header). Both posts are no-ops on their respective
// drain sides when nothing is actually present to consume them (PoseEstimator
// always exists; the odometer post is simply never applied when
// hardware.odometer() is null on the loop's drain side).
// ---------------------------------------------------------------------------
void handleSI(const ArgList& args, const char* corrId, ReplyFn replyFn, void* replyCtx,
              void* handlerCtx) {
  Rt::Blackboard& b = bb(handlerCtx);
  int32_t x = args.args[0].ival;  // [mm]
  int32_t y = args.args[1].ival;  // [mm]
  int32_t h = args.args[2].ival;  // [cdeg]

  msg::SetPose pose;
  pose.x = static_cast<float>(x);
  pose.y = static_cast<float>(y);
  pose.h = static_cast<float>(h) * kCdegToRad;  // [rad] -- PoseEstimator's internal convention

  Rt::PoseResetCommand reset;
  reset.kind = Rt::PoseResetCommand::kSetPose;
  reset.pose = pose;
  b.poseResetIn.post(reset);

  b.otosSetPoseIn.post(pose);

  char body[48];
  snprintf(body, sizeof(body), "x=%d y=%d h=%d", static_cast<int>(x), static_cast<int>(y),
           static_cast<int>(h));
  char rbuf[80];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "setpose", body, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// ZERO -- ZERO enc [#id] -> OK zero enc [#id] (docs/protocol-v2.md section
// 10's "### ZERO"). This ticket implements the `enc` sub-verb only. Unaffected
// parsing -- see the original file header note (kept for historical
// context, no longer duplicated verbatim here since the design is
// unchanged).
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
// handleZero -- posts Rt::PoseResetCommand{kResetBaseline} to bb.poseResetIn.
// (093/094 teardown) No longer also sets bb.motorResetIn[left-1]/[right-1] --
// that per-port reset flag is gone along with the rest of Rt::Blackboard's
// motor/hardware inbound queues (blackboard.h's file header); Hardware no
// longer receives commands of any kind through the Blackboard.
// ---------------------------------------------------------------------------
void handleZero(const ArgList& /*args*/, const char* corrId, ReplyFn replyFn, void* replyCtx,
                 void* handlerCtx) {
  Rt::Blackboard& b = bb(handlerCtx);

  Rt::PoseResetCommand reset;
  reset.kind = Rt::PoseResetCommand::kResetBaseline;
  b.poseResetIn.post(reset);

  char rbuf[48];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "zero", "enc", corrId, replyFn, replyCtx);
}

}  // namespace

std::vector<CommandDescriptor> poseCommands(Rt::CommandRouter& router) {
  std::vector<CommandDescriptor> cmds;
  cmds.push_back(
      makeSchemaCmd("SI", &kSiSchema, handleSI, &router, "badarg", ForceReply::NONE, CMD_NONE));
  cmds.push_back(makeCmd("ZERO", parseZero, handleZero, &router, "badarg", ForceReply::NONE,
                          CMD_ACCESS_HARDWARE));
  return cmds;
}

