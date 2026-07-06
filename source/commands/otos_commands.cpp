// otos_commands.cpp -- OI/OZ/OR/OP/OV/OL/OA handlers. See otos_commands.h
// for the file-level design notes (live odometer() resolution, OP's
// CMD_NONE read/write distinction, OL/OA's shadow pattern).
#include "commands/otos_commands.h"

#if ROBOT_DEV_BUILD

#include <cstdint>
#include <cstdio>

#include "commands/command_processor.h"
#include "hal/capability/odometer.h"

namespace {

// kCdegToRad/kRadToCdeg -- centidegrees<->radians, duplicated here per this
// codebase's existing per-file convention (see pose_commands.cpp's own
// kCdegToRad comment -- there is no shared conversion-constant header).
// kRadToCdeg matches telemetry/tlm_frame.cpp's own kAngleScale (18000/pi)
// bit-for-bit, so OP's readback never drifts against TLM's pose=/otos=
// fields for the same underlying radians value. docs/protocol-v2.md §11
// documents OP's/OV's h in centi-degrees (an intentional new-tree
// convention -- source_old's own OtosCommands.cpp used milliradians
// instead; this file follows §11, the already-approved wire contract, not
// source_old's unit choice).
constexpr float kCdegToRad = 3.14159265f / 18000.0f;
constexpr float kRadToCdeg = 5729.5779513f;

// ---------------------------------------------------------------------------
// otosReady -- shared nodev guard, ported from source_old/commands/
// OtosCommands.cpp's own otosReady(): resolves hardware.odometer() LIVE
// (this file's own header comment) and emits "ERR nodev <verb>" when null.
// ---------------------------------------------------------------------------
bool otosReady(OtosCommandState& state, Hal::Odometer** out, const char* verb,
               const char* corrId, ReplyFn replyFn, void* replyCtx) {
  Hal::Odometer* odometer = state.hardware->odometer();
  if (odometer == nullptr) {
    char rbuf[48];
    CommandProcessor::replyErr(rbuf, sizeof(rbuf), "nodev", verb, corrId, replyFn, replyCtx);
    return false;
  }
  *out = odometer;
  return true;
}

// ---------------------------------------------------------------------------
// OI -- OI [#id] -> OK oi [#id] | ERR nodev oi [#id]. Re-initialises OTOS
// signal processing (Hal::Odometer::apply()'s INIT arm).
// ---------------------------------------------------------------------------
void handleOI(const ArgList& /*args*/, const char* corrId, ReplyFn replyFn, void* replyCtx,
              void* handlerCtx) {
  OtosCommandState& state = *static_cast<OtosCommandState*>(handlerCtx);
  Hal::Odometer* odometer;
  if (!otosReady(state, &odometer, "oi", corrId, replyFn, replyCtx)) return;

  msg::OdometerCommand cmd;
  cmd.setInit(true);
  odometer->apply(cmd);

  char rbuf[32];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "oi", nullptr, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// OZ -- OZ [#id] -> OK oz [#id] | ERR nodev oz [#id]. Zeroes the OTOS
// world-frame position to the current location (Hal::Odometer::apply()'s
// ZERO arm).
// ---------------------------------------------------------------------------
void handleOZ(const ArgList& /*args*/, const char* corrId, ReplyFn replyFn, void* replyCtx,
              void* handlerCtx) {
  OtosCommandState& state = *static_cast<OtosCommandState*>(handlerCtx);
  Hal::Odometer* odometer;
  if (!otosReady(state, &odometer, "oz", corrId, replyFn, replyCtx)) return;

  msg::OdometerCommand cmd;
  cmd.setZero(true);
  odometer->apply(cmd);

  char rbuf[32];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "oz", nullptr, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// OR -- OR [#id] -> OK or [#id] | ERR nodev or [#id]. Resets OTOS Kalman
// filter / tracking state (Hal::Odometer::apply()'s RESET_TRACKING arm).
// ---------------------------------------------------------------------------
void handleOR(const ArgList& /*args*/, const char* corrId, ReplyFn replyFn, void* replyCtx,
              void* handlerCtx) {
  OtosCommandState& state = *static_cast<OtosCommandState*>(handlerCtx);
  Hal::Odometer* odometer;
  if (!otosReady(state, &odometer, "or", corrId, replyFn, replyCtx)) return;

  msg::OdometerCommand cmd;
  cmd.setResetTracking(true);
  odometer->apply(cmd);

  char rbuf[32];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "or", nullptr, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// OP -- OP [#id] -> OK pos x=<x> y=<y> h=<h> [#id] | ERR nodev op [#id].
// Reads Hal::Odometer::pose() directly -- a cheap accessor, not tick() --
// see this file's header comment for why this is CMD_NONE, not
// CMD_ACCESS_HARDWARE.
// ---------------------------------------------------------------------------
void handleOP(const ArgList& /*args*/, const char* corrId, ReplyFn replyFn, void* replyCtx,
              void* handlerCtx) {
  OtosCommandState& state = *static_cast<OtosCommandState*>(handlerCtx);
  Hal::Odometer* odometer = state.hardware->odometer();
  char rbuf[64];
  if (odometer == nullptr) {
    CommandProcessor::replyErr(rbuf, sizeof(rbuf), "nodev", "op", corrId, replyFn, replyCtx);
    return;
  }

  msg::PoseEstimate est = odometer->pose();
  int x = static_cast<int>(est.pose.x);   // [mm]
  int y = static_cast<int>(est.pose.y);   // [mm]
  int h = static_cast<int>(est.pose.h * kRadToCdeg);   // [cdeg]

  char body[48];
  snprintf(body, sizeof(body), "x=%d y=%d h=%d", x, y, h);
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "pos", body, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// OV -- OV <x> <y> <h> [#id] -> OK setpos x=<x> y=<y> h=<h> [#id] | ERR
// nodev ov [#id]. Sets the OTOS world-frame position (Hal::Odometer::
// apply()'s SET_POSE arm). x, y: mm; h: cdeg (docs/protocol-v2.md §11) --
// three mandatory INTs, no range check, matching SI's own kSiSchema shape
// (pose_commands.cpp) and source_old/commands/OtosCommands.cpp's ovSchema.
// ---------------------------------------------------------------------------
const ArgDef kOvDefs[3] = {
    {"x", ArgKind::INT, false, 0, 0},
    {"y", ArgKind::INT, false, 0, 0},
    {"h", ArgKind::INT, false, 0, 0},
};
const ArgSchema kOvSchema = {kOvDefs, 3, 3, false, nullptr};

void handleOV(const ArgList& args, const char* corrId, ReplyFn replyFn, void* replyCtx,
              void* handlerCtx) {
  OtosCommandState& state = *static_cast<OtosCommandState*>(handlerCtx);
  Hal::Odometer* odometer;
  if (!otosReady(state, &odometer, "ov", corrId, replyFn, replyCtx)) return;

  int32_t x = args.args[0].ival;   // [mm]
  int32_t y = args.args[1].ival;   // [mm]
  int32_t h = args.args[2].ival;   // [cdeg]

  msg::Pose2D pose;
  pose.x = static_cast<float>(x);
  pose.y = static_cast<float>(y);
  pose.h = static_cast<float>(h) * kCdegToRad;   // [rad]

  msg::OdometerCommand cmd;
  cmd.setSetPose(pose);
  odometer->apply(cmd);

  char body[48];
  snprintf(body, sizeof(body), "x=%d y=%d h=%d", static_cast<int>(x), static_cast<int>(y),
           static_cast<int>(h));
  char rbuf[80];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "setpos", body, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// OL -- OL <val> [#id] (set) | OL [#id] (read) -> OK linear scalar=<val>
// [#id] | ERR nodev ol [#id]. Gets or sets the OTOS linear scalar
// calibration register (int8_t, docs/protocol-v2.md §11) -- store-and-echo
// only against Hal::SimOdometer this sprint (Decision 5's Consequences: no
// physical effect). Optional int8 scalar; 0 or 1 INT token; no range check
// -- matches source_old/commands/OtosCommands.cpp's olSchema.
// ---------------------------------------------------------------------------
const ArgDef kOlDefs[1] = {
    {"scalar", ArgKind::INT, false, 0, 0},
};
const ArgSchema kOlSchema = {kOlDefs, 1, 0, false, nullptr};

void handleOL(const ArgList& args, const char* corrId, ReplyFn replyFn, void* replyCtx,
              void* handlerCtx) {
  OtosCommandState& state = *static_cast<OtosCommandState*>(handlerCtx);
  Hal::Odometer* odometer;
  if (!otosReady(state, &odometer, "ol", corrId, replyFn, replyCtx)) return;

  if (args.suppliedCount >= 1) {
    // int8_t cast preserves the wire-documented register width (silent
    // truncation), matching source_old's handleOL/setLinearScalar exactly.
    state.configShadow.linear_scalar =
        static_cast<float>(static_cast<int8_t>(args.args[0].ival));
    odometer->configure(state.configShadow);
  }

  char body[24];
  snprintf(body, sizeof(body), "scalar=%d", static_cast<int>(state.configShadow.linear_scalar));
  char rbuf[48];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "linear", body, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// OA -- OA <val> [#id] (set) | OA [#id] (read) -> OK angular scalar=<val>
// [#id] | ERR nodev oa [#id]. Gets or sets the OTOS angular scalar
// calibration register (int8_t) -- same store-and-echo shape as OL, above.
// ---------------------------------------------------------------------------
const ArgDef kOaDefs[1] = {
    {"scalar", ArgKind::INT, false, 0, 0},
};
const ArgSchema kOaSchema = {kOaDefs, 1, 0, false, nullptr};

void handleOA(const ArgList& args, const char* corrId, ReplyFn replyFn, void* replyCtx,
              void* handlerCtx) {
  OtosCommandState& state = *static_cast<OtosCommandState*>(handlerCtx);
  Hal::Odometer* odometer;
  if (!otosReady(state, &odometer, "oa", corrId, replyFn, replyCtx)) return;

  if (args.suppliedCount >= 1) {
    state.configShadow.angular_scalar =
        static_cast<float>(static_cast<int8_t>(args.args[0].ival));
    odometer->configure(state.configShadow);
  }

  char body[24];
  snprintf(body, sizeof(body), "scalar=%d", static_cast<int>(state.configShadow.angular_scalar));
  char rbuf[48];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "angular", body, corrId, replyFn, replyCtx);
}

}  // namespace

std::vector<CommandDescriptor> otosCommands(OtosCommandState& state) {
  std::vector<CommandDescriptor> cmds;
  cmds.push_back(
      makeCmd("OI", nullptr, handleOI, &state, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE));
  cmds.push_back(
      makeCmd("OZ", nullptr, handleOZ, &state, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE));
  cmds.push_back(
      makeCmd("OR", nullptr, handleOR, &state, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE));
  cmds.push_back(makeCmd("OP", nullptr, handleOP, &state, "badarg", ForceReply::NONE, CMD_NONE));
  cmds.push_back(makeSchemaCmd("OV", &kOvSchema, handleOV, &state, "badarg", ForceReply::NONE,
                                CMD_ACCESS_HARDWARE));
  cmds.push_back(makeSchemaCmd("OL", &kOlSchema, handleOL, &state, "badarg", ForceReply::NONE,
                                CMD_ACCESS_HARDWARE));
  cmds.push_back(makeSchemaCmd("OA", &kOaSchema, handleOA, &state, "badarg", ForceReply::NONE,
                                CMD_ACCESS_HARDWARE));
  return cmds;
}

#endif  // ROBOT_DEV_BUILD
