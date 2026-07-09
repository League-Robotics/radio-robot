// motion_commands.cpp -- S/T/D/R/TURN/RT/G/STOP handlers + stop= clause
// grammar. See motion_commands.h for the file-level design notes.
//
// Grammar (parseS/parseT/parseD/mc_packStopKVs/mc_parseStopTokenInto) is
// ported from source_old/commands/MotionCommands.cpp -- the WIRE SHAPE only,
// unaffected by this rewrite. Every handler BODY posts a Rt::MotionCommand
// to bb.motionIn instead of calling Subsystems::Planner::apply()/tick()
// through the (sprint-079-deleted) CommandQueue or the (087-006-deleted)
// MotionLoopState outbox.
#include "commands/motion_commands.h"


#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <math.h>

#include "commands/arg_parse.h"
#include "commands/command_processor.h"
#include "kinematics/body_kinematics.h"
#include "messages/drivetrain.h"

namespace {

Rt::Blackboard& bb(void* handlerCtx) { return static_cast<Rt::CommandRouter*>(handlerCtx)->blackboard(); }

// kMaxStopConds -- docs/protocol-v2.md §10's "Up to 4 stop= clauses are
// accepted per command"; matches msg::PlannerCommand::stops_[4]'s capacity
// and Subsystems::Planner::copyCallerStops()'s own cap (planner.cpp).
constexpr uint8_t kMaxStopConds = 4;

// ---------------------------------------------------------------------------
// copyCorrId -- copy the wire correlation id (may be "", never nullptr, per
// HandlerFn's contract) into a msg::PlannerCommand's corr_id[64], bounded and
// NUL-terminated. Subsystems::Planner::apply() copies this verbatim into any
// completion Event it later queues (planner.cpp's stageGoal()).
// ---------------------------------------------------------------------------
void copyCorrId(msg::PlannerCommand& cmd, const char* corrId) {
  int i = 0;
  if (corrId) {
    for (; corrId[i] != '\0' && i < static_cast<int>(sizeof(cmd.corr_id)) - 1; ++i) {
      cmd.corr_id[i] = corrId[i];
    }
  }
  cmd.corr_id[i] = '\0';
}

// ---------------------------------------------------------------------------
// parseStopClauseValue -- parse the value portion of one "stop=<kind>:<args>"
// token (the string AFTER "stop=") into a msg::StopCondition. Unaffected by
// this rewrite -- pure parsing, no state.
// ---------------------------------------------------------------------------
bool parseStopClauseValue(const char* value, msg::StopCondition& out) {
  char buf[48];
  int vlen = 0;
  for (const char* p = value; *p != '\0' && vlen < static_cast<int>(sizeof(buf)) - 1; ++p, ++vlen) {
    buf[vlen] = *p;
  }
  buf[vlen] = '\0';

  char* colon1 = strchr(buf, ':');
  if (!colon1) return false;
  *colon1 = '\0';
  const char* kind = buf;
  const char* rest = colon1 + 1;

  if (strcmp(kind, "t") == 0) {
    out = msg::StopCondition();
    out.kind = msg::StopKind::STOP_TIME;
    out.a = static_cast<float>(atof(rest));   // [ms]
    return true;
  }

  if (strcmp(kind, "d") == 0) {
    out = msg::StopCondition();
    out.kind = msg::StopKind::STOP_DISTANCE;
    out.a = static_cast<float>(atof(rest));   // [mm]
    return true;
  }

  if (strcmp(kind, "rot") == 0) {
    out = msg::StopCondition();
    out.kind = msg::StopKind::STOP_ROTATION;
    out.a = static_cast<float>(atof(rest));   // [mm] per-wheel arc
    return true;
  }

  if (strcmp(kind, "heading") == 0) {
    char* colon2 = strchr(const_cast<char*>(rest), ':');
    if (!colon2) return false;
    *colon2 = '\0';
    const char* headingStr = rest;
    const char* epsStr = colon2 + 1;
    constexpr float kCdegToRad = 3.14159265f / (100.0f * 180.0f);
    out = msg::StopCondition();
    out.kind = msg::StopKind::STOP_HEADING;
    out.a = static_cast<float>(atof(headingStr)) * kCdegToRad;
    out.b = static_cast<float>(atof(epsStr)) * kCdegToRad;
    return true;
  }

  // "sensor"/"color"/"line" (and any other unrecognized kind prefix): no
  // sensor Hal leaf exists yet -- recognized-but-unsupported and genuinely
  // malformed both land here; the caller rejects either with ERR badarg.
  return false;
}

// ---------------------------------------------------------------------------
// collectStopClauses -- scan args.args[startIdx..count-1] (packed by
// packStopKVs below) for "stop=<value>"/"sensor=<value>" STR tokens, parsing
// each into `out[]`. Unaffected by this rewrite.
// ---------------------------------------------------------------------------
bool collectStopClauses(const ArgList& args, int startIdx, msg::StopCondition* out,
                        uint8_t& countOut) {
  countOut = 0;
  for (int i = startIdx; i < args.count; ++i) {
    if (args.args[i].type != ArgType::STR) continue;
    const char* s = args.args[i].sval;

    const char* value = nullptr;
    bool isSensorAlias = false;
    if (strncmp(s, "stop=", 5) == 0) {
      value = s + 5;
    } else if (strncmp(s, "sensor=", 7) == 0) {
      isSensorAlias = true;
    } else {
      continue;
    }

    if (isSensorAlias) return false;   // always a SENSOR-kind clause -- unsupported

    if (countOut >= kMaxStopConds) continue;   // cap; extra clauses dropped, not an error

    msg::StopCondition cond;
    if (!parseStopClauseValue(value, cond)) return false;
    out[countOut++] = cond;
  }
  return true;
}

// ---------------------------------------------------------------------------
// packStopKVs -- scan kvs for "stop"/"sensor" keys; pack each as a STR arg
// "stop=<value>"/"sensor=<value>" into out.args[*idxInOut..]. Unaffected by
// this rewrite.
// ---------------------------------------------------------------------------
void packStopKVs(const KVPair* kvs, int nkv, ArgList& out, int& idxInOut) {
  for (int i = 0; i < nkv; ++i) {
    if (idxInOut >= MAX_ARGS) break;
    if (!kvs[i].key || !kvs[i].value) continue;

    bool isStop = (strcmp(kvs[i].key, "stop") == 0);
    bool isSensor = (strcmp(kvs[i].key, "sensor") == 0);
    if (!isStop && !isSensor) continue;

    Argument& a = out.args[idxInOut];
    a.type = ArgType::STR;
    a.ival = 0;
    const char* prefix = isStop ? "stop=" : "sensor=";
    int j = 0;
    const char* src = prefix;
    while (*src && j < static_cast<int>(sizeof(a.sval)) - 1) a.sval[j++] = *src++;
    src = kvs[i].value;
    while (*src && j < static_cast<int>(sizeof(a.sval)) - 1) a.sval[j++] = *src++;
    a.sval[j] = '\0';
    out.count = ++idxInOut;
  }
}

// ---------------------------------------------------------------------------
// replyStopBadarg -- shared ERR badarg reply for a stop= clause that failed
// to parse. Unaffected by this rewrite.
// ---------------------------------------------------------------------------
void replyStopBadarg(const char* corrId, ReplyFn replyFn, void* replyCtx) {
  char rbuf[48];
  CommandProcessor::replyErr(rbuf, sizeof(rbuf), "badarg", "stop", corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// parseS -- S <l> <r>. No stop=/sensor= support: 093-001 removed stop-
// condition evaluation from handleS's path entirely (the Planner that used
// to evaluate stop clauses is no longer wired to this verb), so a stop=/
// sensor= kv token is rejected outright as `badarg` rather than silently
// accepted and ignored -- an ignored wire argument the caller believes will
// be honored is confusing (093-001 ticket decision).
// ---------------------------------------------------------------------------
ParseResult parseS(const char* const* tokens, int ntokens, const KVPair* kvs, int nkv) {
  ParseResult res;
  if (ntokens < 2) {
    res.ok = false; res.err.code = nullptr; res.err.detail = nullptr; return res;
  }
  int l = atoi(tokens[0]);
  int r = atoi(tokens[1]);
  if (l < -1000 || l > 1000) {
    res.ok = false; res.err.code = "range"; res.err.detail = "l"; return res;
  }
  if (r < -1000 || r > 1000) {
    res.ok = false; res.err.code = "range"; res.err.detail = "r"; return res;
  }
  if (kvFind(kvs, nkv, "stop") != nullptr) {
    res.ok = false; res.err.code = "badarg"; res.err.detail = "stop"; return res;
  }
  if (kvFind(kvs, nkv, "sensor") != nullptr) {
    res.ok = false; res.err.code = "badarg"; res.err.detail = "sensor"; return res;
  }
  res.ok = true;
  res.args.count = 2;
  argInt(res.args.args[0], l);
  argInt(res.args.args[1], r);
  res.args.suppliedCount = res.args.count;
  return res;
}

// ---------------------------------------------------------------------------
// parseT -- T <l> <r> <ms> [stop=...] [sensor=...]
// ---------------------------------------------------------------------------
ParseResult parseT(const char* const* tokens, int ntokens, const KVPair* kvs, int nkv) {
  ParseResult res;
  if (ntokens < 3) {
    res.ok = false; res.err.code = nullptr; res.err.detail = nullptr; return res;
  }
  int l = atoi(tokens[0]);
  int r = atoi(tokens[1]);
  int ms = atoi(tokens[2]);
  if (l < -1000 || l > 1000) {
    res.ok = false; res.err.code = "range"; res.err.detail = "l"; return res;
  }
  if (r < -1000 || r > 1000) {
    res.ok = false; res.err.code = "range"; res.err.detail = "r"; return res;
  }
  if (ms < 1 || ms > 30000) {
    res.ok = false; res.err.code = "range"; res.err.detail = "ms"; return res;
  }
  res.ok = true;
  res.args.count = 3;
  argInt(res.args.args[0], l);
  argInt(res.args.args[1], r);
  argInt(res.args.args[2], ms);
  int idx = 3;
  packStopKVs(kvs, nkv, res.args, idx);
  res.args.suppliedCount = res.args.count;
  return res;
}

// ---------------------------------------------------------------------------
// parseD -- D <l> <r> <mm> [stop=...] [sensor=...]
// ---------------------------------------------------------------------------
ParseResult parseD(const char* const* tokens, int ntokens, const KVPair* kvs, int nkv) {
  ParseResult res;
  if (ntokens < 3) {
    res.ok = false; res.err.code = nullptr; res.err.detail = nullptr; return res;
  }
  int l = atoi(tokens[0]);
  int r = atoi(tokens[1]);
  int mm = atoi(tokens[2]);
  if (l < -1000 || l > 1000) {
    res.ok = false; res.err.code = "range"; res.err.detail = "l"; return res;
  }
  if (r < -1000 || r > 1000) {
    res.ok = false; res.err.code = "range"; res.err.detail = "r"; return res;
  }
  if (mm < 1 || mm > 10000) {
    res.ok = false; res.err.code = "range"; res.err.detail = "mm"; return res;
  }
  res.ok = true;
  res.args.count = 3;
  argInt(res.args.args[0], l);
  argInt(res.args.args[1], r);
  argInt(res.args.args[2], mm);
  int idx = 3;
  packStopKVs(kvs, nkv, res.args, idx);
  res.args.suppliedCount = res.args.count;
  return res;
}

// ---------------------------------------------------------------------------
// parseR -- R <speed> <radius> [stop=...] [sensor=...]
// ---------------------------------------------------------------------------
ParseResult parseR(const char* const* tokens, int ntokens, const KVPair* kvs, int nkv) {
  ParseResult res;
  if (ntokens < 2) {
    res.ok = false; res.err.code = nullptr; res.err.detail = nullptr; return res;
  }
  int speed = atoi(tokens[0]);
  int radius = atoi(tokens[1]);
  if (speed < -1000 || speed > 1000) {
    res.ok = false; res.err.code = "range"; res.err.detail = "speed"; return res;
  }
  if (radius < -10000 || radius > 10000) {
    res.ok = false; res.err.code = "range"; res.err.detail = "radius"; return res;
  }
  res.ok = true;
  res.args.count = 2;
  argInt(res.args.args[0], speed);
  argInt(res.args.args[1], radius);
  int idx = 2;
  packStopKVs(kvs, nkv, res.args, idx);
  res.args.suppliedCount = res.args.count;
  return res;
}

// ---------------------------------------------------------------------------
// parseTURN -- TURN <heading> [eps=<cdeg>] [stop=...] [sensor=...]
// ---------------------------------------------------------------------------
ParseResult parseTURN(const char* const* tokens, int ntokens, const KVPair* kvs, int nkv) {
  ParseResult res;
  if (ntokens < 1) {
    res.ok = false; res.err.code = "badarg"; res.err.detail = nullptr; return res;
  }
  int heading = atoi(tokens[0]);
  if (heading < -18000 || heading > 18000) {
    res.ok = false; res.err.code = "range"; res.err.detail = "heading"; return res;
  }
  // Optional eps=<cdeg>; default 300.
  int eps = 300;
  const KVPair* epsKv = kvFind(kvs, nkv, "eps");
  if (epsKv) {
    eps = atoi(epsKv->value);
    if (eps < 10 || eps > 1800) {
      res.ok = false; res.err.code = "range"; res.err.detail = "eps"; return res;
    }
  }
  res.ok = true;
  res.args.count = 2;
  argInt(res.args.args[0], heading);
  argInt(res.args.args[1], eps);
  int idx = 2;
  packStopKVs(kvs, nkv, res.args, idx);
  res.args.suppliedCount = res.args.count;
  return res;
}

// ---------------------------------------------------------------------------
// parseRT -- RT <relAngle> [stop=...] [sensor=...]
// ---------------------------------------------------------------------------
ParseResult parseRT(const char* const* tokens, int ntokens, const KVPair* kvs, int nkv) {
  ParseResult res;
  if (ntokens < 1) {
    res.ok = false; res.err.code = nullptr; res.err.detail = nullptr; return res;
  }
  int relAngle = atoi(tokens[0]);
  if (relAngle < -180000 || relAngle > 180000) {
    res.ok = false; res.err.code = "range"; res.err.detail = "relAngle"; return res;
  }
  res.ok = true;
  res.args.count = 1;
  argInt(res.args.args[0], relAngle);
  int idx = 1;
  packStopKVs(kvs, nkv, res.args, idx);
  res.args.suppliedCount = res.args.count;
  return res;
}

// ---------------------------------------------------------------------------
// handleS -- 093-001: direct wheel drive, no kinematics/ramp/stop-condition
// closure. Builds a msg::WheelTargets straight from the parsed l/r ints and
// posts a msg::DrivetrainCommand{WHEELS} to bb.driveIn, mirroring DEV DT
// WHEELS's own construction idiom exactly (dev_commands.cpp's
// DtMode::WHEELS case) -- Subsystems::Drivetrain::apply() maps WHEELS to
// setWheelTargets(left, right) directly (drivetrain.cpp), bypassing the
// (now-unwired) Planner entirely. Reply stays `OK drive l=.. r=..`.
// ---------------------------------------------------------------------------
void handleS(const ArgList& args, const char* corrId, ReplyFn replyFn, void* replyCtx,
            void* handlerCtx) {
  Rt::Blackboard& b = bb(handlerCtx);
  int l = args.args[0].ival;
  int r = args.args[1].ival;

  msg::WheelTargets wt;
  wt.w_[0].speed.has = true; wt.w_[0].speed.val = static_cast<float>(l);
  wt.w_[1].speed.has = true; wt.w_[1].speed.val = static_cast<float>(r);
  wt.w_count = 2;
  msg::DrivetrainCommand cmd;
  cmd.setWheels(wt);
  b.driveIn.post(cmd);

  char body[32];
  snprintf(body, sizeof(body), "l=%d r=%d", l, r);
  char rbuf[64];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "drive", body, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// handleT -- bounded-time drive: converts l/r to (v, omega), posts a TIMED
// goal.
// ---------------------------------------------------------------------------
void handleT(const ArgList& args, const char* corrId, ReplyFn replyFn, void* replyCtx,
            void* handlerCtx) {
  Rt::Blackboard& b = bb(handlerCtx);
  int l = args.args[0].ival;
  int r = args.args[1].ival;
  int ms = args.args[2].ival;

  msg::StopCondition stops[kMaxStopConds];
  uint8_t stopsCount = 0;
  if (!collectStopClauses(args, 3, stops, stopsCount)) {
    replyStopBadarg(corrId, replyFn, replyCtx);
    return;
  }

  float v = 0.0f, omega = 0.0f;
  BodyKinematics::forward(static_cast<float>(l), static_cast<float>(r), b.drivetrainConfig.trackwidth,
                          v, omega);

  msg::PlannerCommand cmd;
  msg::TimedGoal goal;
  goal.v_x = v;
  goal.omega = omega;
  goal.duration = static_cast<uint32_t>(ms);
  cmd.setTimed(goal);
  for (uint8_t i = 0; i < stopsCount; ++i) cmd.stops_[i] = stops[i];
  cmd.stops_count = stopsCount;
  copyCorrId(cmd, corrId);

  Rt::MotionCommand mc;
  mc.command = cmd;   // verb left empty -- T stages its own DriveMode::TIMED
  b.motionIn.post(mc);

  char body[48];
  snprintf(body, sizeof(body), "l=%d r=%d ms=%d", l, r, ms);
  char rbuf[80];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "drive", body, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// handleD -- bounded-distance drive: msg::DistanceGoal carries only a scalar
// speed/distance pair, no omega -- a straight-line-only goal.
// ---------------------------------------------------------------------------
void handleD(const ArgList& args, const char* corrId, ReplyFn replyFn, void* replyCtx,
            void* handlerCtx) {
  Rt::Blackboard& b = bb(handlerCtx);
  int l = args.args[0].ival;
  int r = args.args[1].ival;
  int mm = args.args[2].ival;

  msg::StopCondition stops[kMaxStopConds];
  uint8_t stopsCount = 0;
  if (!collectStopClauses(args, 3, stops, stopsCount)) {
    replyStopBadarg(corrId, replyFn, replyCtx);
    return;
  }

  float v = 0.0f, omega = 0.0f;
  BodyKinematics::forward(static_cast<float>(l), static_cast<float>(r), b.drivetrainConfig.trackwidth,
                          v, omega);
  (void)omega;   // straight-line only this ticket -- see the doc comment above

  float direction = (v < 0.0f) ? -1.0f : 1.0f;

  msg::PlannerCommand cmd;
  msg::DistanceGoal goal;
  goal.speed = fabsf(v);
  goal.distance = direction * static_cast<float>(mm);
  cmd.setDistance(goal);
  for (uint8_t i = 0; i < stopsCount; ++i) cmd.stops_[i] = stops[i];
  cmd.stops_count = stopsCount;
  copyCorrId(cmd, corrId);

  Rt::MotionCommand mc;
  mc.command = cmd;   // verb left empty -- see runtime/commands.h's field doc comment
  b.motionIn.post(mc);

  char body[48];
  snprintf(body, sizeof(body), "l=%d r=%d mm=%d", l, r, mm);
  char rbuf[80];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "drive", body, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// kCdegToRad -- centidegrees -> radians, shared by handleTURN/handleRT below.
// ---------------------------------------------------------------------------
constexpr float kCdegToRad = 3.14159265f / 18000.0f;

// kTurnOmega/kRotationOmega -- fixed spin-in-place rates for TURN/RT.
constexpr float kTurnOmega = 1.2217f;      // [rad/s] ~70 deg/s
constexpr float kRotationOmega = 1.7453f;  // [rad/s] ~100 deg/s

// wrapAngle -- wrap x into (-pi, pi].
float wrapAngle(float x) { return atan2f(sinf(x), cosf(x)); }

// ---------------------------------------------------------------------------
// handleR -- open-loop constant-curvature arc: omega = speed/radius. Posts a
// VELOCITY goal exactly like a bare S -- runs until an explicit STOP or a
// stop= clause fires. Sets Rt::MotionCommand::verb="R" -- shared with
// TURN/RT (planner.cpp's velocityShapedMode()), disambiguated for the
// loop's "EVT done <verb>" text and the sTimeout exclusion gate.
// ---------------------------------------------------------------------------
void handleR(const ArgList& args, const char* corrId, ReplyFn replyFn, void* replyCtx,
            void* handlerCtx) {
  Rt::Blackboard& b = bb(handlerCtx);
  int speed = args.args[0].ival;
  int radius = args.args[1].ival;

  msg::StopCondition stops[kMaxStopConds];
  uint8_t stopsCount = 0;
  if (!collectStopClauses(args, 2, stops, stopsCount)) {
    replyStopBadarg(corrId, replyFn, replyCtx);
    return;
  }

  // omega = speed/radius (kappa = 1/radius); 0 when radius == 0. Positive
  // radius -> positive omega -> CCW (left) arc.
  float omega = (radius != 0) ? (static_cast<float>(speed) / static_cast<float>(radius)) : 0.0f;

  msg::PlannerCommand cmd;
  msg::VelocityGoal goal;
  goal.v_x = static_cast<float>(speed);
  goal.v_y = 0.0f;
  goal.omega = omega;
  cmd.setVelocity(goal);
  for (uint8_t i = 0; i < stopsCount; ++i) cmd.stops_[i] = stops[i];
  cmd.stops_count = stopsCount;
  copyCorrId(cmd, corrId);
  // 090-004: threaded through to Planner's own persisted verb_ (stageCommon())
  // so a completed goal's msg::Event can self-describe its "done R" wire
  // name -- mirrors mc.verb below exactly (motion_commands.h's field doc
  // comment).
  snprintf(cmd.verb, sizeof(cmd.verb), "R");

  Rt::MotionCommand mc;
  mc.command = cmd;
  snprintf(mc.verb, sizeof(mc.verb), "R");
  b.motionIn.post(mc);

  char body[48];
  snprintf(body, sizeof(body), "speed=%d radius=%d", speed, radius);
  char rbuf[80];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "arc", body, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// handleTURN -- absolute-heading turn-in-place, closed-loop against
// bb.fusedPose.pose.h (the SAME reading Planner::tick() would receive):
// reads the current fused heading, computes the shortest-path signed delta
// to the absolute target, and posts a fixed-rate spin in that direction plus
// a HEADING stop at the resolved delta.
// ---------------------------------------------------------------------------
void handleTURN(const ArgList& args, const char* corrId, ReplyFn replyFn, void* replyCtx,
                void* handlerCtx) {
  Rt::Blackboard& b = bb(handlerCtx);
  int heading = args.args[0].ival;   // [cdeg] absolute target heading
  int eps = args.args[1].ival;       // [cdeg]

  // Reserve 1 of kMaxStopConds's 4 slots for the built-in HEADING stop; up
  // to kMaxStopConds - 1 caller stop= clauses are accepted.
  msg::StopCondition userStops[kMaxStopConds];
  uint8_t userCount = 0;
  if (!collectStopClauses(args, 2, userStops, userCount)) {
    replyStopBadarg(corrId, replyFn, replyCtx);
    return;
  }
  if (userCount > kMaxStopConds - 1) userCount = kMaxStopConds - 1;

  float currentHeading = b.fusedPose.pose.h;   // [rad]
  float diff = static_cast<float>(heading) * kCdegToRad - currentHeading;
  float delta = wrapAngle(diff);   // [rad] shortest-path signed delta, (-pi, pi]
  float omega = (delta >= 0.0f) ? kTurnOmega : -kTurnOmega;

  msg::PlannerCommand cmd;
  msg::TurnGoal goal;
  // heading: informational only -- planner.cpp's TURN case reads only
  // goal.turn.speed (the already-signed rate); see planner.h's class comment.
  goal.heading = static_cast<float>(heading) * kCdegToRad;
  goal.speed = omega;
  cmd.setTurn(goal);

  msg::StopCondition headingStop;
  headingStop.kind = msg::StopKind::STOP_HEADING;
  headingStop.a = delta;
  headingStop.b = static_cast<float>(eps) * kCdegToRad;

  uint8_t total = 0;
  cmd.stops_[total++] = headingStop;
  for (uint8_t i = 0; i < userCount; ++i) cmd.stops_[total++] = userStops[i];
  cmd.stops_count = total;
  copyCorrId(cmd, corrId);
  // 090-004: see handleR()'s own comment on why this mirrors mc.verb below.
  snprintf(cmd.verb, sizeof(cmd.verb), "TURN");

  Rt::MotionCommand mc;
  mc.command = cmd;
  snprintf(mc.verb, sizeof(mc.verb), "TURN");
  b.motionIn.post(mc);

  char body[48];
  snprintf(body, sizeof(body), "heading=%d eps=%d", heading, eps);
  char rbuf[80];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "turn", body, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// handleRT -- relative turn-in-place, closed-loop against the per-wheel
// encoder arc (a ROTATION stop condition), reading bb.drivetrainConfig.
// trackwidth for the per-wheel arc computation.
// ---------------------------------------------------------------------------
void handleRT(const ArgList& args, const char* corrId, ReplyFn replyFn, void* replyCtx,
              void* handlerCtx) {
  Rt::Blackboard& b = bb(handlerCtx);
  int relAngle = args.args[0].ival;   // [cdeg]

  // Reserve 1 of kMaxStopConds's 4 slots for the built-in ROTATION stop; up
  // to kMaxStopConds - 1 caller stop= clauses are accepted.
  msg::StopCondition userStops[kMaxStopConds];
  uint8_t userCount = 0;
  if (!collectStopClauses(args, 1, userStops, userCount)) {
    replyStopBadarg(corrId, replyFn, replyCtx);
    return;
  }
  if (userCount > kMaxStopConds - 1) userCount = kMaxStopConds - 1;

  float trackwidth = b.drivetrainConfig.trackwidth;   // [mm]
  // Per-wheel arc = |relAngle| (rad) * (trackwidth/2) -- the ideal
  // spin-in-place geometry, no slip correction.
  float arc = fabsf(static_cast<float>(relAngle)) * kCdegToRad * (trackwidth * 0.5f);   // [mm]
  float omega = (relAngle >= 0) ? kRotationOmega : -kRotationOmega;   // + => CCW (left)

  msg::PlannerCommand cmd;
  msg::RotationGoal goal;
  // angle: informational only -- planner.cpp's ROTATION case reads only
  // goal.rotation.speed (the already-signed rate); see planner.h's class comment.
  goal.angle = static_cast<float>(relAngle) * kCdegToRad;
  goal.speed = omega;
  cmd.setRotation(goal);

  msg::StopCondition rotStop;
  rotStop.kind = msg::StopKind::STOP_ROTATION;
  rotStop.a = arc;

  uint8_t total = 0;
  cmd.stops_[total++] = rotStop;
  for (uint8_t i = 0; i < userCount; ++i) cmd.stops_[total++] = userStops[i];
  cmd.stops_count = total;
  copyCorrId(cmd, corrId);
  // 090-004: see handleR()'s own comment on why this mirrors mc.verb below.
  snprintf(cmd.verb, sizeof(cmd.verb), "RT");

  Rt::MotionCommand mc;
  mc.command = cmd;
  snprintf(mc.verb, sizeof(mc.verb), "RT");
  b.motionIn.post(mc);

  char body[32];
  snprintf(body, sizeof(body), "rot=%d", relAngle);
  char rbuf[64];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "rt", body, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// parseG -- G <x> <y> <speed>. No stop=/sensor= support.
// ---------------------------------------------------------------------------
ParseResult parseG(const char* const* tokens, int ntokens, const KVPair* kvs, int nkv) {
  ParseResult res;
  (void)kvs;
  (void)nkv;
  if (ntokens < 3) {
    res.ok = false; res.err.code = nullptr; res.err.detail = nullptr; return res;
  }
  int x = atoi(tokens[0]);
  int y = atoi(tokens[1]);
  int speed = atoi(tokens[2]);
  if (x < -10000 || x > 10000) {
    res.ok = false; res.err.code = "range"; res.err.detail = "x"; return res;
  }
  if (y < -10000 || y > 10000) {
    res.ok = false; res.err.code = "range"; res.err.detail = "y"; return res;
  }
  if (speed < 1 || speed > 1000) {
    res.ok = false; res.err.code = "range"; res.err.detail = "speed"; return res;
  }
  res.ok = true;
  res.args.count = 3;
  argInt(res.args.args[0], x);
  argInt(res.args.args[1], y);
  argInt(res.args.args[2], speed);
  res.args.suppliedCount = res.args.count;
  return res;
}

// ---------------------------------------------------------------------------
// handleG -- relative-XY go-to: posts a GOTO_GOAL goal. Subsystems::Planner
// owns the entire PRE_ROTATE/PURSUE state machine internally -- this handler
// only builds the msg::GotoGoal and posts it, mirroring handleD's shape.
// ---------------------------------------------------------------------------
void handleG(const ArgList& args, const char* corrId, ReplyFn replyFn, void* replyCtx,
            void* handlerCtx) {
  Rt::Blackboard& b = bb(handlerCtx);
  int x = args.args[0].ival;
  int y = args.args[1].ival;
  int speed = args.args[2].ival;

  msg::PlannerCommand cmd;
  msg::GotoGoal goal;
  goal.x = static_cast<float>(x);
  goal.y = static_cast<float>(y);
  goal.speed = static_cast<float>(speed);
  cmd.setGotoGoal(goal);
  copyCorrId(cmd, corrId);

  Rt::MotionCommand mc;
  mc.command = cmd;   // verb left empty -- see runtime/commands.h's field doc comment
  b.motionIn.post(mc);

  char body[48];
  snprintf(body, sizeof(body), "x=%d y=%d speed=%d", x, y, speed);
  char rbuf[80];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "goto", body, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// handleStop -- 093-001 (fixed): STOP posts a NEUTRAL msg::DrivetrainCommand
// straight to bb.driveIn, built inline WITHOUT the standby side-channel --
// deliberately NOT dev_commands.h's buildDrivetrainStop() helper, which sets
// {NEUTRAL, standby=true}. That shape was found to be a correctness bug: in
// Rt::MainLoop::routeOutputs(), the computed NEUTRAL wheel command is only
// posted to bb.motorIn[] when drivetrain_.active() is true, and
// Subsystems::Drivetrain::apply() processes standby=true AFTER the NEUTRAL
// arm, immediately flipping active_ back to false in the same apply() call --
// so the neutral command was silently dropped and the wheels kept spinning
// at their last commanded speed. Leaving standby unset keeps the drivetrain
// active, so routeOutputs() passes the neutral through to bb.motorIn[] and
// Hal::Hardware::tick() actually neutralizes both motors. In this four-verb
// loop there is no authority-steal producer for the standby gate to protect
// against (DEV M et al. are unregistered), and a subsequent `S` re-activates
// via setWheelTargets() regardless, so an active-neutral STOP is correct and
// simplest. buildDrivetrainStop() itself is left unchanged -- other/parked
// callers (DEV STOP, DEV DT STOP, the loop's watchdog-fire path) still rely
// on its standby=true shape. No EVT. Reply stays `OK stop`.
// ---------------------------------------------------------------------------
void handleStop(const ArgList& /*args*/, const char* corrId, ReplyFn replyFn, void* replyCtx,
                void* handlerCtx) {
  Rt::Blackboard& b = bb(handlerCtx);

  msg::DrivetrainCommand cmd;
  cmd.setNeutral(msg::Neutral::BRAKE);
  b.driveIn.post(cmd);

  char rbuf[32];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "stop", nullptr, corrId, replyFn, replyCtx);
}

// handleQlen -- sprint 093 debug: report current Blackboard queue occupancy so
// a bench operator can SEE a routed command land on its target queue while the
// control loop is disabled (nothing drains these, so a posted command
// accumulates instead of being consumed). Mailbox cells report 0/1
// (latest-wins); WorkQueues report size().
void handleQlen(const ArgList& /*args*/, const char* corrId, ReplyFn replyFn, void* replyCtx,
                void* handlerCtx) {
  Rt::Blackboard& b = bb(handlerCtx);
  char body[192];
  snprintf(body, sizeof(body),
           "cmd=%u drive=%d motion=%d cfg=%u pose=%u m1=%d m2=%d m3=%d m4=%d",
           static_cast<unsigned>(b.commandsIn.size()),
           b.driveIn.empty() ? 0 : 1,
           b.motionIn.empty() ? 0 : 1,
           static_cast<unsigned>(b.configIn.size()),
           static_cast<unsigned>(b.poseResetIn.size()),
           b.motorIn[0].empty() ? 0 : 1, b.motorIn[1].empty() ? 0 : 1,
           b.motorIn[2].empty() ? 0 : 1, b.motorIn[3].empty() ? 0 : 1);
  char rbuf[240];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "qlen", body, corrId, replyFn, replyCtx);
}

}  // namespace

// 093-001: pruned to the sprint's four live verbs' motion half (S/STOP) --
// the "four live verbs" decision (issue simplify-the-main-loop-strip-it-to-
// bare-wheel-driving.md) applies literally to this table, not just to
// buildTable()'s family-level selection. T/D/R/TURN/RT/G's parse/handle
// functions above are left source-unchanged and simply uncalled here --
// same "unregistered not deleted" treatment as the other command families
// (architecture-update.md Step 5/Migration Concerns).
std::vector<CommandDescriptor> motionCommands(Rt::CommandRouter& router) {
  std::vector<CommandDescriptor> cmds;
  cmds.push_back(makeCmd("S", parseS, handleS, &router, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE));
  cmds.push_back(makeCmd("STOP", nullptr, handleStop, &router, "badarg", ForceReply::NONE,
                         CMD_ACCESS_HARDWARE));
  // 093 debug: QLEN -- read-only Blackboard queue-occupancy probe (handlerCtx
  // = &router so it can reach the blackboard via bb()).
  cmds.push_back(makeCmd("QLEN", nullptr, handleQlen, &router, "badarg"));
  return cmds;
}

