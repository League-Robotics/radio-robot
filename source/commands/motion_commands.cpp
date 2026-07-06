// motion_commands.cpp -- S/T/D/STOP handlers + stop= clause grammar. See
// motion_commands.h for the file-level design notes.
//
// Grammar (parseS/parseT/parseD, the "kind:args" stop-clause split, and the
// stop=/sensor= kv-packing helper) is ported from source_old/commands/
// MotionCommands.cpp's parseS/parseT/parseD/mc_packStopKVs/
// mc_parseStopTokenInto -- the WIRE SHAPE only. Every handler body is new:
// it stages a msg::PlannerCommand into MotionLoopState's outbox instead of
// calling Superstructure::requestGoal()/Planner::beginX() through the
// (sprint-079-deleted) CommandQueue.
#include "commands/motion_commands.h"

#if ROBOT_DEV_BUILD

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <math.h>

#include "commands/arg_parse.h"
#include "commands/command_processor.h"
#include "kinematics/body_kinematics.h"
#include "types/clock.h"

namespace {

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
// token (the string AFTER "stop=") into a msg::StopCondition. Ported concept
// from source_old/commands/MotionCommands.cpp's mc_parseStopTokenInto(),
// scoped to the five kinds architecture-update.md (084) Decision 4 keeps this
// sprint -- t/d/heading/rot fully implemented; sensor/color/line are
// recognized (their kind prefix matches) but return false here, same as any
// genuinely malformed clause -- the caller (collectStopClauses) turns EITHER
// outcome into one `ERR badarg`, never a silent drop (Decision 4's
// Consequences; docs/protocol-v2.md §10's stop= clause table note this
// ticket adds).
//
// `heading:<cdeg>:<eps_cdeg>` -- unlike source_old's absolute-heading
// reading, motion/stop_condition.cpp's STOP_HEADING (ticket 001) compares
// against the DELTA from the goal's OWN starting heading (its `a` field is
// "target heading delta from baseline, rad" -- see evaluateStopCondition()'s
// own doc comment), so the wire value here is parsed as that same relative
// delta, converted cdeg -> rad.
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
// each into `out[]`. Returns false (out/countOut left in a partial,
// meaningless state) the instant ANY clause fails to parse, including the
// always-rejected "sensor=" back-compat alias (docs/protocol-v2.md §10:
// "sensor=<ch>:<op>:<thr> is accepted as a back-compat alias for
// stop=sensor:<ch>:<op>:<thr>" -- still a SENSOR-kind clause, still
// unsupported) -- callers must validate BEFORE staging any goal, matching
// source_old/commands/MotionCommands.cpp's handleT/handleD precedent of
// validating sensor= clauses before ever replying OK.
//
// Clauses beyond kMaxStopConds are silently dropped (not an error) -- matches
// Subsystems::Planner::copyCallerStops()'s own cap, which would clamp them
// right back down anyway.
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
// "stop=<value>"/"sensor=<value>" into out.args[*idxInOut..]. Ported from
// source_old/commands/MotionCommands.cpp's mc_packStopKVs().
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
// to parse (malformed, or a recognized-but-unsupported sensor/color/line
// kind) -- see collectStopClauses()'s own doc comment.
// ---------------------------------------------------------------------------
void replyStopBadarg(const char* corrId, ReplyFn replyFn, void* replyCtx) {
  char rbuf[48];
  CommandProcessor::replyErr(rbuf, sizeof(rbuf), "badarg", "stop", corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// parseS -- S <l> <r> [stop=...] [sensor=...]
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
  res.ok = true;
  res.args.count = 2;
  argInt(res.args.args[0], l);
  argInt(res.args.args[1], r);
  int idx = 2;
  packStopKVs(kvs, nkv, res.args, idx);
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
// handleS -- streams a body twist (converted from wheel speeds l/r via
// BodyKinematics::forward(), the same conversion telemetry_commands.cpp's
// twist= field already performs) into a STREAM goal, and feeds sTimeout --
// the ONE handler that does so (see motion_commands.h's class comment).
// ---------------------------------------------------------------------------
void handleS(const ArgList& args, const char* corrId, ReplyFn replyFn, void* replyCtx,
            void* handlerCtx) {
  MotionLoopState& state = *static_cast<MotionLoopState*>(handlerCtx);
  int l = args.args[0].ival;
  int r = args.args[1].ival;

  msg::StopCondition stops[kMaxStopConds];
  uint8_t stopsCount = 0;
  if (!collectStopClauses(args, 2, stops, stopsCount)) {
    replyStopBadarg(corrId, replyFn, replyCtx);
    return;
  }

  float v = 0.0f, omega = 0.0f;
  BodyKinematics::forward(static_cast<float>(l), static_cast<float>(r), state.poseEstimator->trackwidth(),
                          v, omega);

  msg::PlannerCommand cmd;
  msg::StreamGoal goal;
  goal.v_x = v;
  goal.v_y = 0.0f;
  goal.omega = omega;
  cmd.setStream(goal);
  for (uint8_t i = 0; i < stopsCount; ++i) cmd.stops_[i] = stops[i];
  cmd.stops_count = stopsCount;
  copyCorrId(cmd, corrId);

  state.command = cmd;
  state.hasCommand = true;

  // sTimeout: fed HERE, exactly the way DEV DT VW feeds SerialSilenceWatchdog
  // today (i.e. the arriving-command's own dispatch beat) -- but only S
  // feeds this one (see motion_commands.h). `now` for the feed is whatever
  // dev_loop.cpp's drain step's `now` will be on this SAME pass (the drain
  // runs later in this identical devLoopTick() call) -- feeding it here at
  // parse time rather than there keeps the "S's handler feeds it" contract
  // literal and independent of the outbox-drain step's own placement.
  state.sTimeout.feed(Types::systemClockNow());

  char body[32];
  snprintf(body, sizeof(body), "l=%d r=%d", l, r);
  char rbuf[64];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "drive", body, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// handleT -- bounded-time drive: converts l/r to (v, omega), stages a TIMED
// goal (Planner's own apply() adds the implicit STOP_TIME(duration) --
// planner.cpp's TIMED case).
// ---------------------------------------------------------------------------
void handleT(const ArgList& args, const char* corrId, ReplyFn replyFn, void* replyCtx,
            void* handlerCtx) {
  MotionLoopState& state = *static_cast<MotionLoopState*>(handlerCtx);
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
  BodyKinematics::forward(static_cast<float>(l), static_cast<float>(r), state.poseEstimator->trackwidth(),
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

  state.command = cmd;
  state.hasCommand = true;

  char body[48];
  snprintf(body, sizeof(body), "l=%d r=%d ms=%d", l, r, ms);
  char rbuf[80];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "drive", body, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// handleD -- bounded-distance drive: msg::DistanceGoal (ticket 001's
// Planner) carries only a scalar `speed`/`distance` pair, no omega -- a
// straight-line-only goal (matching Planner's own GOTO_GOAL placeholder
// precedent; see planner.h's class comment). l/r are converted via
// BodyKinematics::forward() the same as S/T so an l==r symmetric D drives
// correctly (this ticket's acceptance bar); an l!=r D drives straight at
// the average forward speed with no turning component -- an arced D would
// need a DistanceGoal schema change, out of this ticket's scope (planner.{h,cpp}
// are not in its files-to-modify list).
// ---------------------------------------------------------------------------
void handleD(const ArgList& args, const char* corrId, ReplyFn replyFn, void* replyCtx,
            void* handlerCtx) {
  MotionLoopState& state = *static_cast<MotionLoopState*>(handlerCtx);
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
  BodyKinematics::forward(static_cast<float>(l), static_cast<float>(r), state.poseEstimator->trackwidth(),
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

  state.command = cmd;
  state.hasCommand = true;

  char body[48];
  snprintf(body, sizeof(body), "l=%d r=%d mm=%d", l, r, mm);
  char rbuf[80];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "drive", body, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// handleStop -- STOP: immediate halt, no EVT (docs/protocol-v2.md §10:
// "Stops motors immediately... No EVT is emitted"). Stages goal_kind=STOP;
// Subsystems::Planner::apply()'s STOP case resets the ramp and clears the
// active goal synchronously, with no held Event queued.
// ---------------------------------------------------------------------------
void handleStop(const ArgList& /*args*/, const char* corrId, ReplyFn replyFn, void* replyCtx,
                void* handlerCtx) {
  MotionLoopState& state = *static_cast<MotionLoopState*>(handlerCtx);

  msg::PlannerCommand cmd;
  cmd.setStop(true);
  copyCorrId(cmd, corrId);

  state.command = cmd;
  state.hasCommand = true;

  char rbuf[32];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "stop", nullptr, corrId, replyFn, replyCtx);
}

}  // namespace

std::vector<CommandDescriptor> motionCommands(MotionLoopState& state) {
  std::vector<CommandDescriptor> cmds;
  cmds.push_back(makeCmd("S", parseS, handleS, &state, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE));
  cmds.push_back(makeCmd("T", parseT, handleT, &state, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE));
  cmds.push_back(makeCmd("D", parseD, handleD, &state, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE));
  cmds.push_back(makeCmd("STOP", nullptr, handleStop, &state, "badarg", ForceReply::NONE,
                         CMD_ACCESS_HARDWARE));
  return cmds;
}

#endif  // ROBOT_DEV_BUILD
