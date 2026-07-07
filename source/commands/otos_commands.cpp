// otos_commands.cpp -- OI/OZ/OR/OP/OV/OL/OA handlers. See otos_commands.h
// for the file-level design notes (bb.otosPresent's boot-snapshot rationale,
// OP's CMD_NONE read/write distinction, OL/OA's Configurator-routed
// read-modify-write).
#include "commands/otos_commands.h"

#if ROBOT_DEV_BUILD

#include <cstdint>
#include <cstdio>

#include "commands/command_processor.h"
#include "messages/odometer.h"

namespace {

Rt::Blackboard& bb(void* handlerCtx) { return static_cast<Rt::CommandRouter*>(handlerCtx)->blackboard(); }

// kCdegToRad/kRadToCdeg -- centidegrees<->radians, duplicated here per this
// codebase's existing per-file convention. kRadToCdeg matches
// telemetry/tlm_frame.cpp's own kAngleScale (18000/pi) bit-for-bit.
constexpr float kCdegToRad = 3.14159265f / 18000.0f;
constexpr float kRadToCdeg = 5729.5779513f;

// ---------------------------------------------------------------------------
// otosReady -- shared nodev guard: bb.otosPresent is a boot-time snapshot of
// "does any Hal::Odometer exist at all" (see otos_commands.h's file header)
// -- emits "ERR nodev <verb>" when false.
// ---------------------------------------------------------------------------
bool otosReady(const Rt::Blackboard& b, const char* verb, const char* corrId, ReplyFn replyFn,
               void* replyCtx) {
  if (!b.otosPresent) {
    char rbuf[48];
    CommandProcessor::replyErr(rbuf, sizeof(rbuf), "nodev", verb, corrId, replyFn, replyCtx);
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
// OI -- OI [#id] -> OK oi [#id] | ERR nodev oi [#id]. Posts an INIT action to
// bb.otosCommandIn -- the loop drains it against hardware.odometer()
// directly (Hal::Odometer::apply()'s INIT arm).
// ---------------------------------------------------------------------------
void handleOI(const ArgList& /*args*/, const char* corrId, ReplyFn replyFn, void* replyCtx,
              void* handlerCtx) {
  Rt::Blackboard& b = bb(handlerCtx);
  if (!otosReady(b, "oi", corrId, replyFn, replyCtx)) return;

  msg::OdometerCommand cmd;
  cmd.setInit(true);
  b.otosCommandIn.post(cmd);

  char rbuf[32];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "oi", nullptr, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// OZ -- OZ [#id] -> OK oz [#id] | ERR nodev oz [#id]. Posts a ZERO action.
// ---------------------------------------------------------------------------
void handleOZ(const ArgList& /*args*/, const char* corrId, ReplyFn replyFn, void* replyCtx,
              void* handlerCtx) {
  Rt::Blackboard& b = bb(handlerCtx);
  if (!otosReady(b, "oz", corrId, replyFn, replyCtx)) return;

  msg::OdometerCommand cmd;
  cmd.setZero(true);
  b.otosCommandIn.post(cmd);

  char rbuf[32];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "oz", nullptr, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// OR -- OR [#id] -> OK or [#id] | ERR nodev or [#id]. Posts a RESET_TRACKING
// action.
// ---------------------------------------------------------------------------
void handleOR(const ArgList& /*args*/, const char* corrId, ReplyFn replyFn, void* replyCtx,
              void* handlerCtx) {
  Rt::Blackboard& b = bb(handlerCtx);
  if (!otosReady(b, "or", corrId, replyFn, replyCtx)) return;

  msg::OdometerCommand cmd;
  cmd.setResetTracking(true);
  b.otosCommandIn.post(cmd);

  char rbuf[32];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "or", nullptr, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// OP -- OP [#id] -> OK pos x=<x> y=<y> h=<h> [#id] | ERR nodev op [#id].
// Reads bb.otos directly -- a committed state-cell read, not a queue post --
// see otos_commands.h's file header for why this is CMD_NONE.
// ---------------------------------------------------------------------------
void handleOP(const ArgList& /*args*/, const char* corrId, ReplyFn replyFn, void* replyCtx,
              void* handlerCtx) {
  Rt::Blackboard& b = bb(handlerCtx);
  char rbuf[64];
  if (!b.otosPresent) {
    CommandProcessor::replyErr(rbuf, sizeof(rbuf), "nodev", "op", corrId, replyFn, replyCtx);
    return;
  }

  int x = static_cast<int>(b.otos.pose.x);   // [mm]
  int y = static_cast<int>(b.otos.pose.y);   // [mm]
  int h = static_cast<int>(b.otos.pose.h * kRadToCdeg);   // [cdeg]

  char body[48];
  snprintf(body, sizeof(body), "x=%d y=%d h=%d", x, y, h);
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "pos", body, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// OV -- OV <x> <y> <h> [#id] -> OK setpos x=<x> y=<y> h=<h> [#id] | ERR
// nodev ov [#id]. Posts a SET_POSE action to bb.otosCommandIn. x, y: mm; h:
// cdeg (docs/protocol-v2.md §11).
// ---------------------------------------------------------------------------
const ArgDef kOvDefs[3] = {
    {"x", ArgKind::INT, false, 0, 0},
    {"y", ArgKind::INT, false, 0, 0},
    {"h", ArgKind::INT, false, 0, 0},
};
const ArgSchema kOvSchema = {kOvDefs, 3, 3, false, nullptr};

void handleOV(const ArgList& args, const char* corrId, ReplyFn replyFn, void* replyCtx,
              void* handlerCtx) {
  Rt::Blackboard& b = bb(handlerCtx);
  if (!otosReady(b, "ov", corrId, replyFn, replyCtx)) return;

  int32_t x = args.args[0].ival;   // [mm]
  int32_t y = args.args[1].ival;   // [mm]
  int32_t h = args.args[2].ival;   // [cdeg]

  msg::Pose2D pose;
  pose.x = static_cast<float>(x);
  pose.y = static_cast<float>(y);
  pose.h = static_cast<float>(h) * kCdegToRad;   // [rad]

  msg::OdometerCommand cmd;
  cmd.setSetPose(pose);
  b.otosCommandIn.post(cmd);

  char body[48];
  snprintf(body, sizeof(body), "x=%d y=%d h=%d", static_cast<int>(x), static_cast<int>(y),
           static_cast<int>(h));
  char rbuf[80];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "setpos", body, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// OL -- OL <val> [#id] (set) | OL [#id] (read) -> OK linear scalar=<val>
// [#id] | ERR nodev ol [#id]. Config-plane (read-modify-write persistent
// register), unlike the four one-shot verbs above: reads bb.odometerConfig
// (the Configurator's published cell) and, on a set, posts a field-masked
// Rt::ConfigDelta (kOdometer) to bb.configIn -- mirrors
// config_commands.h's SET pattern. The reply echoes the CANDIDATE value
// directly (the just-parsed value on a set, else the CURRENT published
// value) -- never a bb read-back, so no post-then-read-back race.
// ---------------------------------------------------------------------------
const ArgDef kOlDefs[1] = {
    {"scalar", ArgKind::INT, false, 0, 0},
};
const ArgSchema kOlSchema = {kOlDefs, 1, 0, false, nullptr};

void handleOL(const ArgList& args, const char* corrId, ReplyFn replyFn, void* replyCtx,
              void* handlerCtx) {
  Rt::Blackboard& b = bb(handlerCtx);
  if (!otosReady(b, "ol", corrId, replyFn, replyCtx)) return;

  float value = b.odometerConfig.linear_scalar;
  if (args.suppliedCount >= 1) {
    // int8_t cast preserves the wire-documented register width (silent
    // truncation), matching the pre-087 handleOL/setLinearScalar exactly.
    value = static_cast<float>(static_cast<int8_t>(args.args[0].ival));

    Rt::ConfigDelta delta;
    delta.target = Rt::ConfigDelta::kOdometer;
    delta.mask = Rt::bitOf(Rt::OdometerConfigField::kLinearScalar);
    delta.odometer.linear_scalar = value;
    b.configIn.post(delta);
  }

  char body[24];
  snprintf(body, sizeof(body), "scalar=%d", static_cast<int>(value));
  char rbuf[48];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "linear", body, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// OA -- OA <val> [#id] (set) | OA [#id] (read) -> OK angular scalar=<val>
// [#id] | ERR nodev oa [#id]. Same config-plane shape as OL, above.
// ---------------------------------------------------------------------------
const ArgDef kOaDefs[1] = {
    {"scalar", ArgKind::INT, false, 0, 0},
};
const ArgSchema kOaSchema = {kOaDefs, 1, 0, false, nullptr};

void handleOA(const ArgList& args, const char* corrId, ReplyFn replyFn, void* replyCtx,
              void* handlerCtx) {
  Rt::Blackboard& b = bb(handlerCtx);
  if (!otosReady(b, "oa", corrId, replyFn, replyCtx)) return;

  float value = b.odometerConfig.angular_scalar;
  if (args.suppliedCount >= 1) {
    value = static_cast<float>(static_cast<int8_t>(args.args[0].ival));

    Rt::ConfigDelta delta;
    delta.target = Rt::ConfigDelta::kOdometer;
    delta.mask = Rt::bitOf(Rt::OdometerConfigField::kAngularScalar);
    delta.odometer.angular_scalar = value;
    b.configIn.post(delta);
  }

  char body[24];
  snprintf(body, sizeof(body), "scalar=%d", static_cast<int>(value));
  char rbuf[48];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "angular", body, corrId, replyFn, replyCtx);
}

}  // namespace

std::vector<CommandDescriptor> otosCommands(Rt::CommandRouter& router) {
  std::vector<CommandDescriptor> cmds;
  cmds.push_back(
      makeCmd("OI", nullptr, handleOI, &router, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE));
  cmds.push_back(
      makeCmd("OZ", nullptr, handleOZ, &router, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE));
  cmds.push_back(
      makeCmd("OR", nullptr, handleOR, &router, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE));
  cmds.push_back(makeCmd("OP", nullptr, handleOP, &router, "badarg", ForceReply::NONE, CMD_NONE));
  cmds.push_back(makeSchemaCmd("OV", &kOvSchema, handleOV, &router, "badarg", ForceReply::NONE,
                                CMD_ACCESS_HARDWARE));
  cmds.push_back(makeSchemaCmd("OL", &kOlSchema, handleOL, &router, "badarg", ForceReply::NONE,
                                CMD_ACCESS_HARDWARE));
  cmds.push_back(makeSchemaCmd("OA", &kOaSchema, handleOA, &router, "badarg", ForceReply::NONE,
                                CMD_ACCESS_HARDWARE));
  return cmds;
}

#endif  // ROBOT_DEV_BUILD
