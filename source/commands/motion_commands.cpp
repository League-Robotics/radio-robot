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
#include "motion/segment.h"

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
// Per-segment motion-limit override bounds (094-006) -- generous ceilings
// above the boot Motion::SegmentExecutor defaults (source/main.cpp's/
// tests/_infra/sim/sim_api.cpp's defaultMotionConfig(): a_max=800 mm/s^2,
// v_body_max=1000 mm/s, yaw_rate_max=6 rad/s (~34400 cdeg/s),
// yaw_acc_max=20 rad/s^2 (~114600 cdeg/s^2), j_max=5000 mm/s^3,
// yaw_jerk_max=100 rad/s^3 (~572960 cdeg/s^3)), NOT the executor's own hard
// physical limit -- these only reject an obviously-malformed override, the
// same "sanity ceiling, not a physics model" role D's mm/T's ms/TURN's eps
// bounds already play above.
// ---------------------------------------------------------------------------
constexpr float kMoveMaxSpeedMax = 3000.0f;         // [mm/s]
constexpr float kMoveMaxAccelMax = 6000.0f;         // [mm/s^2]
constexpr float kMoveMaxJerkMax = 60000.0f;         // [mm/s^3]
constexpr float kMoveMaxYawRateMaxCdeg = 72000.0f;      // [cdeg/s]    (~720 deg/s)
constexpr float kMoveMaxYawAccelMaxCdeg = 500000.0f;    // [cdeg/s^2]  (~5000 deg/s^2)
constexpr float kMoveMaxYawJerkMaxCdeg = 2000000.0f;    // [cdeg/s^3]  (~20000 deg/s^3)

// ---------------------------------------------------------------------------
// parseMove -- MOVE <distance_mm> <direction_cdeg> <finalHeading_cdeg>
//              [v=<mm/s>] [a=<mm/s^2>] [j=<mm/s^3>]
//              [w=<cdeg/s>] [wa=<cdeg/s^2>] [wj=<cdeg/s^3>]
//
// Packs 9 args in a FIXED order: [0]=distance(INT mm), [1]=direction(INT
// cdeg), [2]=finalHeading(INT cdeg), [3..8]=v/a/j/w/wa/wj (FLOAT, still WIRE
// units -- handleMove does the cdeg->rad conversion for direction/
// finalHeading/w/wa/wj, matching handleTURN's/handleRT's own split of
// "parse in wire units, convert in the handler" via the shared kCdegToRad
// constant declared below, near handleD).
//
// distance/direction/finalHeading are all SIGNED and may be 0 (Motion::
// Segment's own pose-free geometry -- architecture-update.md Section 5:
// distance==0 is a pure in-place turn, direction==finalHeading==0 is a
// plain straight). direction/finalHeading use RT's own wider relative-angle
// bound (+-180000 cdeg, i.e. +-1800 degrees) rather than TURN's absolute-
// heading +-18000 bound -- both fields here are RELATIVE deltas, the same
// idiom RT's relAngle already uses, not an absolute heading.
//
// v=/a=/j=/w=/wa=/wj= are optional per-segment overrides of the executor's
// boot config; an absent kv defaults to 0.0f via kvFloat -- exactly
// Motion::Segment's own 0-sentinel ("fall back to the executor's configured
// default"), so no separate "was this key supplied" bookkeeping is needed
// here the way parseTURN's `eps` needed one.
// ---------------------------------------------------------------------------
ParseResult parseMove(const char* const* tokens, int ntokens, const KVPair* kvs, int nkv) {
  ParseResult res;
  if (ntokens < 3) {
    res.ok = false; res.err.code = nullptr; res.err.detail = nullptr; return res;
  }
  int distance = atoi(tokens[0]);
  int direction = atoi(tokens[1]);
  int finalHeading = atoi(tokens[2]);

  if (distance < -10000 || distance > 10000) {
    res.ok = false; res.err.code = "range"; res.err.detail = "distance"; return res;
  }
  if (direction < -180000 || direction > 180000) {
    res.ok = false; res.err.code = "range"; res.err.detail = "direction"; return res;
  }
  if (finalHeading < -180000 || finalHeading > 180000) {
    res.ok = false; res.err.code = "range"; res.err.detail = "finalHeading"; return res;
  }

  float v = kvFloat(kvs, nkv, "v", 0.0f);
  float a = kvFloat(kvs, nkv, "a", 0.0f);
  float j = kvFloat(kvs, nkv, "j", 0.0f);
  float w = kvFloat(kvs, nkv, "w", 0.0f);
  float wa = kvFloat(kvs, nkv, "wa", 0.0f);
  float wj = kvFloat(kvs, nkv, "wj", 0.0f);
  float s = kvFloat(kvs, nkv, "s", 0.0f);   // s=1 -> STREAMING segment (merge-chain)

  if (v < 0.0f || v > kMoveMaxSpeedMax) {
    res.ok = false; res.err.code = "range"; res.err.detail = "v"; return res;
  }
  if (a < 0.0f || a > kMoveMaxAccelMax) {
    res.ok = false; res.err.code = "range"; res.err.detail = "a"; return res;
  }
  if (j < 0.0f || j > kMoveMaxJerkMax) {
    res.ok = false; res.err.code = "range"; res.err.detail = "j"; return res;
  }
  if (w < 0.0f || w > kMoveMaxYawRateMaxCdeg) {
    res.ok = false; res.err.code = "range"; res.err.detail = "w"; return res;
  }
  if (wa < 0.0f || wa > kMoveMaxYawAccelMaxCdeg) {
    res.ok = false; res.err.code = "range"; res.err.detail = "wa"; return res;
  }
  if (wj < 0.0f || wj > kMoveMaxYawJerkMaxCdeg) {
    res.ok = false; res.err.code = "range"; res.err.detail = "wj"; return res;
  }

  res.ok = true;
  res.args.count = 10;
  argInt(res.args.args[0], distance);
  argInt(res.args.args[1], direction);
  argInt(res.args.args[2], finalHeading);
  argFloat(res.args.args[3], v);
  argFloat(res.args.args[4], a);
  argFloat(res.args.args[5], j);
  argFloat(res.args.args[6], w);
  argFloat(res.args.args[7], wa);
  argFloat(res.args.args[8], wj);
  argFloat(res.args.args[9], s);
  // Only the 3 required positional tokens count toward suppliedCount (see
  // the trailing doc comment below).
  res.args.suppliedCount = 3;
  return res;
}

// ---------------------------------------------------------------------------
// parseMover -- MOVER <distance_mm> <direction_cdeg> <finalHeading_cdeg>
//               [t=<ms>] [v=<mm/s SIGNED>] [w=<cdeg/s SIGNED>]
//               [a=][j=][wa=][wj=]
// The REPLACE-semantics segment (deadman-velocity teleop): t > 0 makes it
// TIME-bounded (velocity control toward signed v/w for t ms, then graceful
// stop unless replaced); t == 0 is a position-mode replace. distance and t
// both nonzero is an ERROR (a segment is bounded by one or the other, never
// both). v/w are SIGNED here (they carry direction in time mode), unlike
// MOVE's unsigned ceilings.
// ---------------------------------------------------------------------------
ParseResult parseMover(const char* const* tokens, int ntokens, const KVPair* kvs, int nkv) {
  ParseResult res;
  if (ntokens < 3) {
    res.ok = false; res.err.code = nullptr; res.err.detail = nullptr; return res;
  }
  int distance = atoi(tokens[0]);
  int direction = atoi(tokens[1]);
  int finalHeading = atoi(tokens[2]);
  if (distance < -10000 || distance > 10000) {
    res.ok = false; res.err.code = "range"; res.err.detail = "distance"; return res;
  }
  if (direction < -180000 || direction > 180000) {
    res.ok = false; res.err.code = "range"; res.err.detail = "direction"; return res;
  }
  if (finalHeading < -180000 || finalHeading > 180000) {
    res.ok = false; res.err.code = "range"; res.err.detail = "finalHeading"; return res;
  }

  float t = kvFloat(kvs, nkv, "t", 0.0f);    // [ms] deadman window
  float v = kvFloat(kvs, nkv, "v", 0.0f);    // [mm/s] SIGNED
  float a = kvFloat(kvs, nkv, "a", 0.0f);
  float j = kvFloat(kvs, nkv, "j", 0.0f);
  float w = kvFloat(kvs, nkv, "w", 0.0f);    // [cdeg/s] SIGNED
  float wa = kvFloat(kvs, nkv, "wa", 0.0f);
  float wj = kvFloat(kvs, nkv, "wj", 0.0f);

  if (t < 0.0f || t > 5000.0f) {
    res.ok = false; res.err.code = "range"; res.err.detail = "t"; return res;
  }
  if (t > 0.0f && distance != 0) {
    // Time-bounded OR distance-bounded, never both.
    res.ok = false; res.err.code = "badarg"; res.err.detail = "t+distance"; return res;
  }
  if (v < -kMoveMaxSpeedMax || v > kMoveMaxSpeedMax) {
    res.ok = false; res.err.code = "range"; res.err.detail = "v"; return res;
  }
  if (w < -kMoveMaxYawRateMaxCdeg || w > kMoveMaxYawRateMaxCdeg) {
    res.ok = false; res.err.code = "range"; res.err.detail = "w"; return res;
  }
  if (a < 0.0f || a > kMoveMaxAccelMax) {
    res.ok = false; res.err.code = "range"; res.err.detail = "a"; return res;
  }
  if (j < 0.0f || j > kMoveMaxJerkMax) {
    res.ok = false; res.err.code = "range"; res.err.detail = "j"; return res;
  }
  if (wa < 0.0f || wa > kMoveMaxYawAccelMaxCdeg) {
    res.ok = false; res.err.code = "range"; res.err.detail = "wa"; return res;
  }
  if (wj < 0.0f || wj > kMoveMaxYawJerkMaxCdeg) {
    res.ok = false; res.err.code = "range"; res.err.detail = "wj"; return res;
  }

  res.ok = true;
  res.args.count = 10;
  argInt(res.args.args[0], distance);
  argInt(res.args.args[1], direction);
  argInt(res.args.args[2], finalHeading);
  argFloat(res.args.args[3], t);
  argFloat(res.args.args[4], v);
  argFloat(res.args.args[5], a);
  argFloat(res.args.args[6], j);
  argFloat(res.args.args[7], w);
  argFloat(res.args.args[8], wa);
  argFloat(res.args.args[9], wj);
  // Only the 3 required positional tokens count toward suppliedCount -- the
  // 6 kv overrides' "was this supplied" question is already answered by
  // their own 0-sentinel value (see this function's own doc comment), the
  // same reason parseD/parseT/parseR never widen suppliedCount for their
  // packed stop=/sensor= tail either.
  res.args.suppliedCount = 3;
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

  // T -> one straight Motion::Segment: drive the distance the commanded wheel
  // speeds would cover in `ms` (motion planning, not path planning). Ruckig
  // profiles it; the segment self-terminates at that distance. Posted to
  // bb.segmentIn exactly like MOVE (handleMove) -- the Planner path is parked.
  float v = 0.0f, omega = 0.0f;
  BodyKinematics::forward(static_cast<float>(l), static_cast<float>(r),
                          b.drivetrainConfig.trackwidth, v, omega);
  (void)omega;   // straight only -- T maps to a distance-bounded segment
  Motion::Segment seg;
  seg.distance = v * (static_cast<float>(ms) / 1000.0f);   // [mm] signed by v
  // speedMax left 0 -> executor's default profile (proven no-reverse-creep);
  // the commanded l/r only sizes/signs the distance, not the speed cap (a low
  // per-segment speedMax induces a small terminal decel overshoot).
  b.segmentIn.post(seg);

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

  // D -> one straight Motion::Segment of `mm`, signed by the drive direction.
  // Ruckig profiles it (trapezoid/S-curve); the segment self-terminates at
  // the commanded distance. Posted to bb.segmentIn like MOVE -- Planner parked.
  float v = 0.0f, omega = 0.0f;
  BodyKinematics::forward(static_cast<float>(l), static_cast<float>(r),
                          b.drivetrainConfig.trackwidth, v, omega);
  (void)omega;   // straight-line only
  float sign = (v < 0.0f) ? -1.0f : 1.0f;
  Motion::Segment seg;
  seg.distance = sign * static_cast<float>(mm);   // [mm]
  // speedMax left 0 -> executor's default profile (proven no-reverse-creep);
  // the commanded l/r only signs the distance, not the speed cap.
  b.segmentIn.post(seg);

  char body[48];
  snprintf(body, sizeof(body), "l=%d r=%d mm=%d", l, r, mm);
  char rbuf[80];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "drive", body, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// kCdegToRad -- centidegrees -> radians, shared by handleTURN/handleRT below.
// ---------------------------------------------------------------------------
constexpr float kCdegToRad = 3.14159265f / 18000.0f;

// kTurnOmega -- fixed spin-in-place rate for the (unregistered) absolute TURN
// handler. RT no longer uses a fixed rate: it builds a Motion::Segment and the
// SegmentExecutor's Ruckig pivot owns the rate.
constexpr float kTurnOmega = 1.2217f;      // [rad/s] ~70 deg/s

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

  // RT -> one pure in-place turn Motion::Segment: distance 0, finalHeading =
  // relAngle (relative, CCW+). The SegmentExecutor's TERMINAL_PIVOT phase
  // profiles the rotation with Ruckig and self-terminates on the encoder
  // ROTATION stop it derives from trackwidth. Posted to bb.segmentIn like
  // MOVE -- the Planner path is parked.
  Motion::Segment seg;
  seg.finalHeading = static_cast<float>(relAngle) * kCdegToRad;   // [rad] relative
  b.segmentIn.post(seg);

  char body[32];
  snprintf(body, sizeof(body), "rot=%d", relAngle);
  char rbuf[64];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "rt", body, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// handleMove -- 094-006: the sprint's one new wire verb. Converts
// direction_cdeg/finalHeading_cdeg/w=/wa=/wj= from wire centidegrees to
// radians via the shared kCdegToRad constant (declared above, near
// handleD), builds a Motion::Segment 1:1 from parseMove's packed args, and
// posts it to bb.segmentIn -- Subsystems::Drivetrain::tick() drains that
// queue into its own internal ring_ every pass (drivetrain.h's class
// comment) and executes the ring's head segment via its owned
// Motion::SegmentExecutor. No kinematics/goal/stop-condition machinery
// here at all -- unlike every other handler above, this one does not touch
// bb.motionIn/msg::PlannerCommand (that whole path stays Planner-only,
// parked -- 094-002).
// ---------------------------------------------------------------------------
void handleMove(const ArgList& args, const char* corrId, ReplyFn replyFn, void* replyCtx,
                void* handlerCtx) {
  Rt::Blackboard& b = bb(handlerCtx);
  int distance = args.args[0].ival;       // [mm]
  int direction = args.args[1].ival;      // [cdeg]
  int finalHeading = args.args[2].ival;   // [cdeg]
  float v = args.args[3].fval;    // [mm/s]
  float a = args.args[4].fval;    // [mm/s^2]
  float j = args.args[5].fval;    // [mm/s^3]
  float w = args.args[6].fval;    // [cdeg/s]
  float wa = args.args[7].fval;   // [cdeg/s^2]
  float wj = args.args[8].fval;   // [cdeg/s^3]

  Motion::Segment seg;
  seg.distance = static_cast<float>(distance);
  seg.direction = static_cast<float>(direction) * kCdegToRad;
  seg.finalHeading = static_cast<float>(finalHeading) * kCdegToRad;
  seg.speedMax = v;
  seg.accelMax = a;
  seg.jerkMax = j;
  seg.yawRateMax = w * kCdegToRad;
  seg.yawAccelMax = wa * kCdegToRad;
  seg.yawJerkMax = wj * kCdegToRad;
  seg.stream = args.args[9].fval > 0.5f;   // s=1 -> merge-chain into the live plan

  // Streaming teleop flow control (OOP 2026-07-09): a full segmentIn now
  // replies `ERR full` instead of dropping silently -- a streamer treats it
  // as "back off hard". Every accepted MOVE's ack carries q=<depth>: the
  // segments still in segmentIn (undrained this pass) plus the Drivetrain's
  // committed ring+executing depth (bb.drivetrain.queue, one pass stale --
  // fine for rate control). The client aims to hold q ~= 3.
  if (!b.segmentIn.post(seg)) {
    char ebuf[48];
    CommandProcessor::replyErr(ebuf, sizeof(ebuf), "full", nullptr, corrId, replyFn, replyCtx);
    return;
  }

  unsigned q = b.segmentIn.size() + b.drivetrain.queue;
  // rem= -- remaining translation in the live plan [mm] (one pass stale):
  // the streaming client's buffer-depth feedback. Held near ~0.4s of motion
  // so the plan's to-rest tail never starts mid-stream.
  char body[96];
  snprintf(body, sizeof(body), "dist=%d dir=%d fh=%d q=%u rem=%d",
           distance, direction, finalHeading, q,
           static_cast<int>(b.drivetrain.rem));
  char rbuf[128];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "move", body, corrId, replyFn, replyCtx);
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
// handleMover -- the REPLACE-semantics segment (deadman-velocity teleop,
// OOP 2026-07-09). Posts to bb.replaceIn -- a latest-wins Mailbox, so two
// MOVERs in one pass leave only the newest (replace semantics on the wire
// itself). The Drivetrain drains it ahead of segmentIn, clears the ring,
// and the executor replans from its CURRENT velocity
// (SegmentExecutor::replaceStream). t > 0: velocity control toward the
// SIGNED v/w for t ms, then a graceful stop unless replaced first -- the
// joystick deadman. t == 0: position-mode replace.
// ---------------------------------------------------------------------------
void handleMover(const ArgList& args, const char* corrId, ReplyFn replyFn, void* replyCtx,
                 void* handlerCtx) {
  Rt::Blackboard& b = bb(handlerCtx);
  int distance = args.args[0].ival;       // [mm]
  int direction = args.args[1].ival;      // [cdeg]
  int finalHeading = args.args[2].ival;   // [cdeg]
  float t = args.args[3].fval;    // [ms]
  float v = args.args[4].fval;    // [mm/s] SIGNED
  float a = args.args[5].fval;    // [mm/s^2]
  float j = args.args[6].fval;    // [mm/s^3]
  float w = args.args[7].fval;    // [cdeg/s] SIGNED
  float wa = args.args[8].fval;   // [cdeg/s^2]
  float wj = args.args[9].fval;   // [cdeg/s^3]

  Motion::Segment seg;
  seg.stream = true;
  seg.time = t;                                         // [ms]
  seg.distance = static_cast<float>(distance);
  seg.direction = static_cast<float>(direction) * kCdegToRad;
  seg.finalHeading = static_cast<float>(finalHeading) * kCdegToRad;
  seg.v = v;                                            // [mm/s] signed
  seg.omega = w * kCdegToRad;                           // [rad/s] signed
  seg.speedMax = fabsf(v);                              // ceiling = |target|
  seg.yawRateMax = fabsf(w) * kCdegToRad;
  seg.accelMax = a;
  seg.jerkMax = j;
  seg.yawAccelMax = wa * kCdegToRad;
  seg.yawJerkMax = wj * kCdegToRad;

  b.replaceIn.post(seg);

  unsigned q = b.segmentIn.size() + b.drivetrain.queue;
  char body[80];
  snprintf(body, sizeof(body), "t=%d v=%d w=%d q=%u",
           static_cast<int>(t), static_cast<int>(v), static_cast<int>(w), q);
  char rbuf[112];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "mover", body, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// handleStop -- 093-001 (fixed), physical behavior updated by 094-004/006:
// STOP posts a NEUTRAL msg::DrivetrainCommand straight to bb.driveIn, built
// inline WITHOUT the standby side-channel -- deliberately NOT
// dev_commands.h's buildDrivetrainStop() helper, which sets {NEUTRAL,
// standby=true}. That shape was found to be a correctness bug (093-001):
// the pre-094 routeOutputs() step posted the computed NEUTRAL wheel command
// to bb.motorIn[] only when drivetrain_.active() was true, and
// Subsystems::Drivetrain::apply() processed standby=true AFTER the NEUTRAL
// arm, immediately flipping active_ back to false in the same apply() call
// -- so the neutral command was silently dropped and the wheels kept
// spinning at their last commanded speed. Leaving standby unset keeps the
// drivetrain active, so the neutral reaches the motors. That
// routeOutputs()/bb.motorIn[] plumbing is itself gone now (094-005 --
// Drivetrain stages its own wheel writes directly through hardware_'s
// motor refs), but the underlying reason to leave standby unset (a
// subsequent `S` re-activates via setWheelTargets() regardless, and there
// is still no authority-steal producer in this trimmed table for the
// standby gate to protect against) is unchanged, so this handler's own
// NEUTRAL construction is unchanged.
//
// PHYSICAL EFFECT changed with 094-004's Drivetrain rewrite, though: this
// same NEUTRAL command no longer means "instant brake" in every case.
// Subsystems::Drivetrain::dispatchEscapeHatch() (drivetrain.cpp) inspects
// whether a Motion::Segment is actively executing (SEGMENT mode AND
// executor_.active()) when a NEUTRAL arrives -- if so, it arms the owned
// Motion::SegmentExecutor's OWN presolved graceful decel-to-zero
// (executor_.stop(now)) instead of zeroing the wheels instantly, and this
// Drivetrain keeps riding that decel down to a literal 0.0f twist over
// subsequent ticks (architecture-update.md Section 6, "STOP triggers the
// graceful decel-to-zero" -- the communicator issue's own fix request).
// Only when there is nothing in-flight to decelerate (a plain `S` then
// `STOP`, no segment ever queued, or the executor was already idle) does
// STOP fall straight through to the pre-094 instant-neutral behavior --
// see test_bare_loop_commands.py's own DIRECT-mode STOP test, still green
// unchanged, and test_bare_loop_move_and_tlm.py's SEGMENT-mode STOP test
// (094-006) for the graceful path, both exercised over this SAME handler.
// This handler itself needed no code change for that behavior switch --
// entirely a Drivetrain-level (094-004) decision on the SAME NEUTRAL
// command shape this handler already built. No EVT. Reply stays `OK stop`
// unchanged, even though the physical effect it describes changed.
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

// ---------------------------------------------------------------------------
// handleTlm -- 094-006: one-shot, SNAP-style synchronous read of
// bb.drivetrain (itself sourced from Subsystems::Drivetrain::state(),
// populated every pass by Rt::MainLoop::commit()/main.cpp's own commit
// line -- MEASURED per-wheel encoder position/velocity, not a commanded
// target, since 094-004's rewrite of Drivetrain::state()). Replies through
// the command's own ReplyFn/ctx, exactly like PING/STOP above -- no
// blackboard post (the simplest handler in this file), no EVT, no periodic
// timer, no loop-output queue: architecture-update.md Section 7 Decision 2
// explicitly rules out reviving the pre-093 STREAM/SNAP drain seam for a
// single one-shot producer. Reply shape is this ticket's own choice --
// `OK tlm ...`, wrapped like every other verb in this trimmed table,
// deliberately NOT the pre-093 SNAP verb's unwrapped raw `TLM t=...` line
// (docs/protocol-v2.md section 8) -- see this ticket's completion notes for
// the docs/protocol-v2.md reconciliation this implies (that update itself
// is deferred, per architecture-update.md Step 7 Open Question 1).
// `active=` reports msg::DrivetrainState.active, which 094-006 also widened
// (drivetrain.cpp's state()) to OR in the owned Motion::SegmentExecutor's
// own active/idle status alongside the pre-079 authority flag -- see that
// method's own doc comment for why the authority flag alone would report
// `active=0` throughout a MOVE-only session.
// ---------------------------------------------------------------------------
void handleTlm(const ArgList& /*args*/, const char* corrId, ReplyFn replyFn, void* replyCtx,
              void* handlerCtx) {
  Rt::Blackboard& b = bb(handlerCtx);
  const msg::DrivetrainState& dt = b.drivetrain;

  float encL = dt.enc_count_val() >= 1 ? dt.enc()[0] : 0.0f;
  float encR = dt.enc_count_val() >= 2 ? dt.enc()[1] : 0.0f;
  float velL = dt.vel_count_val() >= 1 ? dt.vel()[0] : 0.0f;
  float velR = dt.vel_count_val() >= 2 ? dt.vel()[1] : 0.0f;
  float cmdL = dt.cmd_count_val() >= 1 ? dt.cmd()[0] : 0.0f;
  float cmdR = dt.cmd_count_val() >= 2 ? dt.cmd()[1] : 0.0f;
  float accL = dt.acc_count_val() >= 1 ? dt.acc()[0] : 0.0f;
  float accR = dt.acc_count_val() >= 2 ? dt.acc()[1] : 0.0f;

  // cmd= is the post-governor commanded wheel velocity (the setpoint the
  // velocity PID chases) vs measured vel=; acc= is the firmware-EMA measured
  // acceleration (raw host-side d(vel)/dt is quantization noise). conn=
  // surfaces per-drive-motor I2C health (NezhaMotor::connected(), via
  // bb.motors[] which main.cpp commits each pass) -- conn=0,0 with
  // everything else ACKing = the Nezha brick is off the bus. glitch= is the
  // cumulative count of encoder samples rejected by the leaf's source-side
  // plausibility gate (corrupted reads; a rising count = bus noise). Drive
  // pair = bound indices 0/1 -> motors[0]/[1].
  unsigned glitchL = b.motors[0].enc_glitch_count.has ? b.motors[0].enc_glitch_count.val : 0;
  unsigned glitchR = b.motors[1].enc_glitch_count.has ? b.motors[1].enc_glitch_count.val : 0;
  // ts= -- each wheel's OWN sample instant (firmware loop clock): the
  // flip-flop samples the two motors on different ~40-80ms slots, so a host
  // plotting both at poll-receive time renders an aliasing staircase; these
  // stamps let it place every reading at its true time (2026-07-09
  // smooth-telemetry fix). enc=/vel= gain 0.1 resolution for the same
  // reason -- integer truncation was adding artificial texture.
  unsigned tsL = b.motors[0].sampled_at.has ? b.motors[0].sampled_at.val : 0;
  unsigned tsR = b.motors[1].sampled_at.has ? b.motors[1].sampled_at.val : 0;
  // Tenths rendered with integer math: the firmware's newlib-nano snprintf
  // has no float support linked (%f silently emits NOTHING -- verified on
  // the bench: `enc=, vel=,`), and pulling in _printf_float costs flash.
  auto formatTenths = [](char* out, size_t n, float v) {
    long t = lroundf(v * 10.0f);
    const char* sign = (t < 0) ? "-" : "";
    if (t < 0) t = -t;
    snprintf(out, n, "%s%ld.%ld", sign, t / 10, t % 10);
  };
  char encLs[16], encRs[16], velLs[16], velRs[16];
  formatTenths(encLs, sizeof(encLs), encL);
  formatTenths(encRs, sizeof(encRs), encR);
  formatTenths(velLs, sizeof(velLs), velL);
  formatTenths(velRs, sizeof(velRs), velR);
  char body[240];
  snprintf(body, sizeof(body),
           "enc=%s,%s vel=%s,%s cmd=%d,%d acc=%d,%d active=%d conn=%d,%d glitch=%u,%u ts=%u,%u now=%u",
           encLs, encRs, velLs, velRs,
           static_cast<int>(cmdL), static_cast<int>(cmdR),
           static_cast<int>(accL), static_cast<int>(accR),
           // active= reports BUSY (motion in progress), not the authority
           // flag -- setNeutral() sets the authority flag TRUE (holding
           // neutral IS governing), so dt.active latches 1 after the first
           // STOP and can never mean "idle". See DrivetrainState.busy.
           dt.busy ? 1 : 0,
           b.motors[0].connected ? 1 : 0, b.motors[1].connected ? 1 : 0,
           glitchL, glitchR, tsL, tsR, b.loopNow);
  char rbuf[272];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "tlm", body, corrId, replyFn, replyCtx);
}

// handleQlen -- sprint 093 debug: report current Blackboard queue occupancy so
// a bench operator can SEE a routed command land on its target queue while the
// control loop is disabled (nothing drains these, so a posted command
// accumulates instead of being consumed). Mailbox cells report 0/1
// (latest-wins); WorkQueues report size(). (093/094 teardown) The m1..m4
// motorIn[] fields are gone -- those per-port queues no longer exist on
// Rt::Blackboard (blackboard.h's file header).
void handleQlen(const ArgList& /*args*/, const char* corrId, ReplyFn replyFn, void* replyCtx,
                void* handlerCtx) {
  Rt::Blackboard& b = bb(handlerCtx);
  char body[192];
  snprintf(body, sizeof(body),
           "cmd=%u drive=%u motion=%d cfg=%u pose=%u",
           static_cast<unsigned>(b.commandsIn.size()),
           static_cast<unsigned>(b.driveIn.size()),
           b.motionIn.empty() ? 0 : 1,
           static_cast<unsigned>(b.configIn.size()),
           static_cast<unsigned>(b.poseResetIn.size()));
  char rbuf[240];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "qlen", body, corrId, replyFn, replyCtx);
}

}  // namespace

// 093-001: pruned to the sprint's live verbs' motion half (S/STOP) -- the
// "four live verbs" decision (issue simplify-the-main-loop-strip-it-to-
// bare-wheel-driving.md) applies literally to this table, not just to
// buildTable()'s family-level selection. T/D/R/TURN/RT/G's parse/handle
// functions above are left source-unchanged and simply uncalled here --
// same "unregistered not deleted" treatment as the other command families
// (architecture-update.md Step 5/Migration Concerns). 094-006 adds the
// sprint's one new verb, MOVE (parseMove/handleMove above), plus the
// minimal pull-based TLM (handleTlm above, nullptr parseFn -- TLM takes no
// args, the same `nullptr`-parseFn precedent STOP/QLEN already use).
std::vector<CommandDescriptor> motionCommands(Rt::CommandRouter& router) {
  std::vector<CommandDescriptor> cmds;
  cmds.push_back(makeCmd("S", parseS, handleS, &router, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE));
  cmds.push_back(makeCmd("STOP", nullptr, handleStop, &router, "badarg", ForceReply::NONE,
                         CMD_ACCESS_HARDWARE));
  // Post-094 (OOP): D/T/RT each re-parse into ONE Motion::Segment posted to
  // bb.segmentIn, exactly like MOVE -- the Drivetrain's SegmentExecutor
  // profiles them with Ruckig. D = straight `mm`; T = straight over `ms`
  // (distance = v*t); RT = pure relative turn. (R arc + absolute TURN stay
  // unregistered: an arc needs a sweep angle and absolute TURN needs the
  // parked pose estimator -- neither is a single pose-free segment.)
  cmds.push_back(makeCmd("D", parseD, handleD, &router, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE));
  cmds.push_back(makeCmd("T", parseT, handleT, &router, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE));
  cmds.push_back(makeCmd("RT", parseRT, handleRT, &router, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE));
  // 094-006: MOVE -- parses into a Motion::Segment, posts to bb.segmentIn.
  // CMD_ACCESS_HARDWARE like S/STOP -- it (eventually) drives the motors.
  cmds.push_back(makeCmd("MOVE", parseMove, handleMove, &router, "badarg", ForceReply::NONE,
                         CMD_ACCESS_HARDWARE));
  // MOVER (OOP): the REPLACE-semantics / deadman-velocity segment (teleop).
  cmds.push_back(makeCmd("MOVER", parseMover, handleMover, &router, "badarg", ForceReply::NONE,
                         CMD_ACCESS_HARDWARE));
  // 094-006: TLM -- one-shot synchronous read of bb.drivetrain; no
  // CMD_ACCESS_HARDWARE flag (mirrors QLEN below -- it reads the ALREADY-
  // committed blackboard cell, not hardware directly, at dispatch time).
  cmds.push_back(makeCmd("TLM", nullptr, handleTlm, &router, "badarg"));
  // 093 debug: QLEN -- read-only Blackboard queue-occupancy probe (handlerCtx
  // = &router so it can reach the blackboard via bb()).
  cmds.push_back(makeCmd("QLEN", nullptr, handleQlen, &router, "badarg"));
  return cmds;
}

