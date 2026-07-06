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
  // Optional eps=<cdeg>; default 300 (docs/protocol-v2.md section 10's
  // documented default once this ticket's section lands).
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
//
// relAngle range +-180000 cdeg (+-1800 degrees, up to 5 full turns) --
// ported verbatim from source_old/commands/MotionCommands.cpp's rtSchema;
// unlike TURN's absolute-heading target (naturally bounded to +-180 degrees
// of travel either way), a RELATIVE turn has no such natural bound.
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
  // Clear activeVelocityVerb -- 084-005: S stages its own DriveMode::
  // STREAMING unconditionally, but a bare R also reports STREAMING now
  // (planner.cpp's velocityShapedMode(), Decision 6); a stale R/TURN/RT
  // value here must not leak into THIS goal's own "EVT done" text -- see
  // motion_commands.h's field doc comment.
  state.activeVelocityVerb[0] = '\0';

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
  // Clear activeVelocityVerb -- 084-005: T stages its own DriveMode::TIMED
  // unconditionally, but a stop=-bearing R/TURN/RT also report TIMED now
  // (planner.cpp's velocityShapedMode(), Decision 6); a stale R/TURN/RT
  // value here must not leak into THIS goal's own "EVT done" text -- see
  // motion_commands.h's field doc comment.
  state.activeVelocityVerb[0] = '\0';

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
  // Clear activeVelocityVerb -- 084-005: D's DriveMode::DISTANCE is not
  // itself shared with R/TURN/RT, but clearing here anyway keeps the
  // invariant uniform (non-empty iff the active goal is R/TURN/RT) rather
  // than relying on DISTANCE's own non-collision as an implicit assumption
  // -- see motion_commands.h's field doc comment.
  state.activeVelocityVerb[0] = '\0';

  char body[48];
  snprintf(body, sizeof(body), "l=%d r=%d mm=%d", l, r, mm);
  char rbuf[80];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "drive", body, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// kCdegToRad -- centidegrees -> radians, shared by handleTURN/handleRT below.
// Same conversion factor as parseStopClauseValue()'s own inline heading:eps
// conversion above (duplicated at this scope since C++ has no clean way to
// share a function-local constant across sibling functions).
// ---------------------------------------------------------------------------
constexpr float kCdegToRad = 3.14159265f / 18000.0f;

// kTurnOmega/kRotationOmega -- fixed spin-in-place rates for TURN/RT.
// Subsystems::Planner exposes no live PlannerConfig getter this ticket (its
// config is write-only via configure() -- see planner.h), so the wire
// handler cannot read yaw_rate_max the way source_old/control/
// PlannerBegin.cpp's beginTurn() read _cfg.yawRateMax. Both constants are
// fixed wire-layer values, well under defaultPlannerConfig()'s 6.0 rad/s
// yaw_rate_max (source/main.cpp) so Motion::VelocityRamp's own clamp never
// kicks in -- deterministic, sim-testable convergence. kTurnOmega mirrors
// source_old/robot/DefaultConfig.cpp's yawRateMax default (70 deg/s);
// kRotationOmega matches source_old's own RT-specific kRtRate constant
// exactly (MotionCommand.cpp's beginRotation, 100 deg/s) -- that ported
// source deliberately used its own fixed rate rather than yawRateMax, and
// this port preserves that same distinction.
constexpr float kTurnOmega = 1.2217f;      // [rad/s] ~70 deg/s
constexpr float kRotationOmega = 1.7453f;  // [rad/s] ~100 deg/s

// wrapAngle -- wrap x into (-pi, pi]. Same atan2f(sinf, cosf) identity
// Motion::evaluateStopCondition's own (private) wrapAngle() uses
// (motion/stop_condition.cpp) -- duplicated here since that helper has no
// external linkage, mirroring source/subsystems/pose_estimator.h's own
// documented precedent for this exact kind of independent-copy duplication.
float wrapAngle(float x) { return atan2f(sinf(x), cosf(x)); }

// ---------------------------------------------------------------------------
// handleR -- open-loop constant-curvature arc: omega = speed/radius (kappa =
// 1/radius; 0 when radius == 0), v = speed. Stages a VELOCITY goal exactly
// like a bare S -- runs until an explicit STOP or a stop= clause fires (no
// stop of its own -- ticket 084-003's acceptance: "none for R").
// ---------------------------------------------------------------------------
void handleR(const ArgList& args, const char* corrId, ReplyFn replyFn, void* replyCtx,
            void* handlerCtx) {
  MotionLoopState& state = *static_cast<MotionLoopState*>(handlerCtx);
  int speed = args.args[0].ival;
  int radius = args.args[1].ival;

  msg::StopCondition stops[kMaxStopConds];
  uint8_t stopsCount = 0;
  if (!collectStopClauses(args, 2, stops, stopsCount)) {
    replyStopBadarg(corrId, replyFn, replyCtx);
    return;
  }

  // omega = speed/radius (kappa = 1/radius); 0 when radius == 0. Positive
  // radius -> positive omega -> CCW (left) arc -- matches source_old/
  // commands/MotionCommands.cpp's handleR sign convention exactly.
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

  state.command = cmd;
  state.hasCommand = true;
  // R stages msg::DriveMode::VELOCITY -- the SAME DriveMode TURN/RT also
  // stage (planner.cpp's apply()) -- see MotionLoopState::activeVelocityVerb's
  // own doc comment (motion_commands.h) for why this field exists.
  snprintf(state.activeVelocityVerb, sizeof(state.activeVelocityVerb), "R");

  char body[48];
  snprintf(body, sizeof(body), "speed=%d radius=%d", speed, radius);
  char rbuf[80];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "arc", body, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// handleTURN -- absolute-heading turn-in-place, closed-loop against
// Planner::tick()'s fusedPose heading argument (this + RT are the FIRST real
// consumers of that already-threaded argument -- see subsystems/planner.h's
// class comment on TurnGoal.speed being an ALREADY-SIGNED rate the caller
// resolves). Ported concept from source_old/control/PlannerBegin.cpp's
// beginTurn(): reads the current fused heading, computes the shortest-path
// signed delta to the absolute target, and stages a fixed-rate spin in that
// direction plus a HEADING stop at the resolved delta -- the "matching stop
// condition" ticket 084-003's own architecture-update.md Decision text
// calls for (singular: no secondary runaway-timeout stop is added here,
// matching the acceptance criterion's "built-in stop (... heading for
// TURN)" wording exactly).
// ---------------------------------------------------------------------------
void handleTURN(const ArgList& args, const char* corrId, ReplyFn replyFn, void* replyCtx,
                void* handlerCtx) {
  MotionLoopState& state = *static_cast<MotionLoopState*>(handlerCtx);
  int heading = args.args[0].ival;   // [cdeg] absolute target heading
  int eps = args.args[1].ival;       // [cdeg]

  // Reserve 1 of kMaxStopConds's 4 slots for the built-in HEADING stop; up
  // to kMaxStopConds - 1 caller stop= clauses are accepted (extras silently
  // dropped, matching collectStopClauses()'s own cap discipline above).
  msg::StopCondition userStops[kMaxStopConds];
  uint8_t userCount = 0;
  if (!collectStopClauses(args, 2, userStops, userCount)) {
    replyStopBadarg(corrId, replyFn, replyCtx);
    return;
  }
  if (userCount > kMaxStopConds - 1) userCount = kMaxStopConds - 1;

  float currentHeading = state.poseEstimator->fusedPose().pose.h;   // [rad]
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

  state.command = cmd;
  state.hasCommand = true;
  snprintf(state.activeVelocityVerb, sizeof(state.activeVelocityVerb), "TURN");

  char body[48];
  snprintf(body, sizeof(body), "heading=%d eps=%d", heading, eps);
  char rbuf[80];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "turn", body, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// handleRT -- relative turn-in-place, closed-loop against the per-wheel
// encoder arc (a ROTATION stop condition -- ticket 001's Motion::
// evaluateStopCondition's existing kind), not against fused heading. Ported
// concept from source_old/control/PlannerBegin.cpp's beginRotation(), minus
// its rotational-slip/coast-anticipation refinement: Subsystems::
// PoseEstimator exposes no rotationalSlip getter (only trackwidth()), and
// coast-anticipation is not part of this ticket's acceptance bar -- the sim
// test measures and documents the resulting plant tolerance instead (ticket
// 084-003's own testing note: "RT/TURN accuracy may show small under/over-
// rotation -- set test tolerances to the plant's actual behavior").
// ---------------------------------------------------------------------------
void handleRT(const ArgList& args, const char* corrId, ReplyFn replyFn, void* replyCtx,
              void* handlerCtx) {
  MotionLoopState& state = *static_cast<MotionLoopState*>(handlerCtx);
  int relAngle = args.args[0].ival;   // [cdeg]

  // Reserve 1 of kMaxStopConds's 4 slots for the built-in ROTATION stop; up
  // to kMaxStopConds - 1 caller stop= clauses are accepted (extras silently
  // dropped, matching collectStopClauses()'s own cap discipline above).
  msg::StopCondition userStops[kMaxStopConds];
  uint8_t userCount = 0;
  if (!collectStopClauses(args, 1, userStops, userCount)) {
    replyStopBadarg(corrId, replyFn, replyCtx);
    return;
  }
  if (userCount > kMaxStopConds - 1) userCount = kMaxStopConds - 1;

  float trackwidth = state.poseEstimator->trackwidth();   // [mm]
  // Per-wheel arc = |relAngle| (rad) * (trackwidth/2) -- the ideal
  // spin-in-place geometry, no slip correction (see doc comment above).
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

  state.command = cmd;
  state.hasCommand = true;
  snprintf(state.activeVelocityVerb, sizeof(state.activeVelocityVerb), "RT");

  char body[32];
  snprintf(body, sizeof(body), "rot=%d", relAngle);
  char rbuf[64];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "rt", body, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// parseG -- G <x> <y> <speed>. No stop=/sensor= support -- unlike every
// other verb in this file, docs/protocol-v2.md section 10's G contract
// defines no stop= clause for G at all (this ticket's acceptance criterion:
// "G accepts no stop= clauses beyond what docs/protocol-v2.md already
// documents for it") -- no packStopKVs()/collectStopClauses() call here.
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
// handleG -- relative-XY go-to: stages a GOTO_GOAL goal. Subsystems::Planner
// owns the entire PRE_ROTATE/PURSUE state machine internally (planner.cpp,
// ticket 084-004) -- this handler only builds the msg::GotoGoal and stages
// it, mirroring handleD's shape.
// ---------------------------------------------------------------------------
void handleG(const ArgList& args, const char* corrId, ReplyFn replyFn, void* replyCtx,
            void* handlerCtx) {
  MotionLoopState& state = *static_cast<MotionLoopState*>(handlerCtx);
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

  state.command = cmd;
  state.hasCommand = true;
  // Clear activeVelocityVerb -- 084-005: G's DriveMode::GO_TO is not itself
  // shared with R/TURN/RT, but clearing here anyway keeps the invariant
  // uniform -- see motion_commands.h's field doc comment (and D's handler
  // above for the identical reasoning).
  state.activeVelocityVerb[0] = '\0';

  char body[48];
  snprintf(body, sizeof(body), "x=%d y=%d speed=%d", x, y, speed);
  char rbuf[80];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "goto", body, corrId, replyFn, replyCtx);
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
  cmds.push_back(makeCmd("R", parseR, handleR, &state, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE));
  cmds.push_back(
      makeCmd("TURN", parseTURN, handleTURN, &state, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE));
  cmds.push_back(makeCmd("RT", parseRT, handleRT, &state, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE));
  cmds.push_back(makeCmd("G", parseG, handleG, &state, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE));
  cmds.push_back(makeCmd("STOP", nullptr, handleStop, &state, "badarg", ForceReply::NONE,
                         CMD_ACCESS_HARDWARE));
  return cmds;
}

#endif  // ROBOT_DEV_BUILD
