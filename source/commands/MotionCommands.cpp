// MotionCommands.cpp — motion command parsing, conversion, and reply
// formatting.
//
// Dependency direction: app/ → control/.  All CommandProcessor reply calls
// live here.
//
// D11 suppression rule (sprint 026-002):
//   Each converter handler (S, T, D, G, R, TURN, RT) calls replyOK ONCE
//   and then pushes a VW ParsedCommand onto the queue.  When handleVW is
//   later dispatched from the queue for a converter push, it MUST NOT call
//   replyOK again — the converter already replied.
//   Only the open-ended VW branch (no stop params) in handleVW emits replyOK.
//   All stop-param branches in handleVW call begin*() and return WITHOUT
//   calling replyOK.
//
// Migration (051-006): bespoke parse functions (parseS, parseT, parseD,
// parseG, parseR, parseRT, parseX, parseNoArgs) replaced with static ArgSchema
// structs + makeSchemaCmd registrations.  parseTURN, parseVW, parse_VW are
// retained with bodies rewritten using argInt / argStr / kvFind helpers.
// setIntArg, packSensorArg, vwScanKV, vwHasKey local helpers deleted; their
// call sites now use argInt (from ArgParse.h) and inline KV scanning (for
// handleVW's STR-args-based KV lookup).

#include "MotionCommands.h"
#include "superstructure/Planner.h"
#include "Superstructure.h"
#include "Robot.h"
#include "CommandProcessor.h"
#include "CommandQueue.h"
#include "BodyKinematics.h"
#include "StopCondition.h"
#include "CommandTypes.h"
#include "ArgParse.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

// ---------------------------------------------------------------------------
// Forward declarations for static handler/parser functions used by
// getMotionCommands() (defined later in this file).
// ---------------------------------------------------------------------------
static ParseResult parseVW(const char* const* tokens, int ntokens,
                           const KVPair* kvs, int nkv);
static void handleVW(const ArgList& args, const char* corrId,
                     ReplyFn replyFn, void* replyCtx, void* handlerCtx);

// ---------------------------------------------------------------------------
// mc_parseSensorToken — parse "sensor=<ch>:<op>:<thr>" into channel, cmp,
// threshold.
//
// Returns true on success; false on any parse/lookup failure.
// ---------------------------------------------------------------------------

// Channel lookup table shared by mc_parseSensorToken and mc_parseStopToken.
// 0-3: line[0..3]; 4-7: colorR/G/B/C; 8-11: analogIn[0..3].
struct SensorChannel { const char* name; uint8_t idx; };
static const SensorChannel kSensorChannels[] = {
    { "line0",    0 }, { "line1",    1 }, { "line2",    2 }, { "line3",    3 },
    { "colorR",   4 }, { "colorG",   5 }, { "colorB",   6 }, { "colorC",   7 },
    { "analogIn0", 8 }, { "analogIn1", 9 }, { "analogIn2", 10 }, { "analogIn3", 11 },
};
static const int kSensorChannelCount = 12;

// Parse a <op> string ("ge" or "le") into a Cmp enum.
// Returns true on success, false on unknown op.
static bool mc_parseCmp(const char* op_str, StopCondition::Cmp& cmp_out)
{
    if (strcmp(op_str, "ge") == 0) { cmp_out = StopCondition::Cmp::GE; return true; }
    if (strcmp(op_str, "le") == 0) { cmp_out = StopCondition::Cmp::LE; return true; }
    return false;
}

static bool mc_parseSensorToken(const char* value,
                                uint8_t& ch_out, float& thr_out,
                                StopCondition::Cmp& cmp_out)
{
    char buf[32];
    int vlen = 0;
    for (const char* p = value; *p && vlen < (int)sizeof(buf) - 1; ++p, ++vlen)
        buf[vlen] = *p;
    buf[vlen] = '\0';

    char* colon1 = strchr(buf, ':');
    if (!colon1) return false;
    *colon1 = '\0';
    const char* ch_name = buf;
    const char* rest    = colon1 + 1;

    char* colon2 = strchr(const_cast<char*>(rest), ':');
    if (!colon2) return false;
    *colon2 = '\0';
    const char* op_str  = rest;
    const char* thr_str = colon2 + 1;

    uint8_t ch = 0;
    bool found = false;
    for (int i = 0; i < kSensorChannelCount; ++i) {
        if (strcmp(ch_name, kSensorChannels[i].name) == 0) {
            ch    = kSensorChannels[i].idx;
            found = true;
            break;
        }
    }
    if (!found) return false;

    StopCondition::Cmp cmp;
    if (!mc_parseCmp(op_str, cmp)) return false;

    int thr = atoi(thr_str);
    ch_out  = ch;
    thr_out = (float)thr;
    cmp_out = cmp;
    return true;
}

// ---------------------------------------------------------------------------
// mc_parseStopTokenInto — parse the value portion of a "stop=<kind>:<args>"
// token into a StopCondition struct (no MotionCommand required).
//
// @param value  String after "stop=" (e.g. "d:300", "line:ge:512",
//               "sensor:line0:ge:512", "color:120:0.5:0.4:0.1",
//               "heading:4500:300", "rot:250", "t:1000").
// @param out    StopCondition to populate on success.
// @return       true when a valid stop condition was parsed; false on error.
//
// Dispatch on prefix before the first ':':
//   t:<ms>                   → makeTimeStop(ms)
//   d:<mm>                   → makeDistanceStop(mm)
//   line:<ge|le>:<thr>       → makeLineAnyStop(thr, cmp)
//   sensor:<ch>:<ge|le>:<thr>→ makeSensorStop(ch, thr, cmp)
//   color:<h>:<s>:<v>:<dist> → makeColorStop(h, s, v, dist)
//   heading:<cdeg>:<eps_cdeg>→ makeHeadingStop(rad, eps_rad)
//   rot:<arc_mm>             → makeRotationStop(arc_mm)
// ---------------------------------------------------------------------------
static bool mc_parseStopTokenInto(const char* value, StopCondition& out)
{
    char buf[64];
    int vlen = 0;
    for (const char* p = value; *p && vlen < (int)sizeof(buf) - 1; ++p, ++vlen)
        buf[vlen] = *p;
    buf[vlen] = '\0';

    // Split on the first ':' to get the kind prefix.
    char* colon1 = strchr(buf, ':');
    if (!colon1) return false;
    *colon1 = '\0';
    const char* kind = buf;
    const char* rest = colon1 + 1;

    if (strcmp(kind, "t") == 0) {
        float ms = (float)atof(rest);
        out = makeTimeStop(ms);
        return true;
    }

    if (strcmp(kind, "d") == 0) {
        float mm = (float)atof(rest);
        out = makeDistanceStop(mm);
        return true;
    }

    if (strcmp(kind, "rot") == 0) {
        float arc = (float)atof(rest);
        out = makeRotationStop(arc);
        return true;
    }

    if (strcmp(kind, "line") == 0) {
        // line:<ge|le>:<thr>
        char* colon2 = strchr(const_cast<char*>(rest), ':');
        if (!colon2) return false;
        *colon2 = '\0';
        const char* op_str  = rest;
        const char* thr_str = colon2 + 1;
        StopCondition::Cmp cmp;
        if (!mc_parseCmp(op_str, cmp)) return false;
        float thr = (float)atof(thr_str);
        out = makeLineAnyStop(thr, cmp);
        return true;
    }

    if (strcmp(kind, "sensor") == 0) {
        // sensor:<ch>:<ge|le>:<thr>
        uint8_t ch; float thr; StopCondition::Cmp cmp;
        if (!mc_parseSensorToken(rest, ch, thr, cmp)) return false;
        out = makeSensorStop(ch, thr, cmp);
        return true;
    }

    if (strcmp(kind, "color") == 0) {
        // color:<h>:<s>:<v>:<dist>
        char* p2 = strchr(const_cast<char*>(rest), ':');
        if (!p2) return false;
        *p2 = '\0';
        float h = (float)atof(rest);
        const char* r2 = p2 + 1;

        char* p3 = strchr(const_cast<char*>(r2), ':');
        if (!p3) return false;
        *p3 = '\0';
        float s = (float)atof(r2);
        const char* r3 = p3 + 1;

        char* p4 = strchr(const_cast<char*>(r3), ':');
        if (!p4) return false;
        *p4 = '\0';
        float v = (float)atof(r3);
        const char* r4 = p4 + 1;
        float dist = (float)atof(r4);

        out = makeColorStop(h, s, v, dist);
        return true;
    }

    if (strcmp(kind, "heading") == 0) {
        // heading:<cdeg>:<eps_cdeg>
        char* colon2 = strchr(const_cast<char*>(rest), ':');
        if (!colon2) return false;
        *colon2 = '\0';
        const char* eps_str = colon2 + 1;
        float cdeg     = (float)atof(rest);
        float eps_cdeg = (float)atof(eps_str);
        const float kCdegToRad = 3.14159265f / (100.0f * 180.0f);
        float headingRad = cdeg     * kCdegToRad;
        float epsRad     = eps_cdeg * kCdegToRad;
        out = makeHeadingStop(headingRad, epsRad);
        return true;
    }

    return false;  // unknown kind
}

// ---------------------------------------------------------------------------
// mc_parseStopToken — parse a "stop=<kind>:<args>" token and add the result
// to a MotionCommand via mc.addStop().  Wraps mc_parseStopTokenInto.
//
// @param value  String after "stop=".
// @param mc     MotionCommand to call addStop() on when successful.
// @return       true when a valid stop condition was parsed and added.
// ---------------------------------------------------------------------------
static bool mc_parseStopToken(const char* value, MotionCommand& mc)
{
    StopCondition cond;
    if (!mc_parseStopTokenInto(value, cond)) return false;
    return mc.addStop(cond);
}

// ---------------------------------------------------------------------------
// mc_applyStopClauses — scan STR args for "stop=<value>" and "sensor=<value>"
// tokens and apply each as a StopCondition via mc_parseStopToken.
//
// Iterates args.args[startIdx..args.count-1].  For each STR entry:
//   - prefix "stop="   → calls mc_parseStopToken(sval+5, mc)
//   - prefix "sensor=" → treats as stop=sensor:<value> for back-compat
//
// Stops early if kMaxStopConds is reached.
// ---------------------------------------------------------------------------
static void mc_applyStopClauses(const ArgList& args, int startIdx,
                                MotionCommand& mc)
{
    for (int i = startIdx; i < args.count; ++i) {
        if (args.args[i].type != ArgType::STR) continue;
        const char* s = args.args[i].sval;

        if (strncmp(s, "stop=", 5) == 0) {
            mc_parseStopToken(s + 5, mc);
        } else if (strncmp(s, "sensor=", 7) == 0) {
            // Back-compat: "sensor=<ch>:<op>:<thr>" → treated as sensor stop.
            uint8_t ch; float thr; StopCondition::Cmp cmp;
            if (mc_parseSensorToken(s + 7, ch, thr, cmp)) {
                mc.addStop(makeSensorStop(ch, thr, cmp));
            }
        }
    }
}

// ---------------------------------------------------------------------------
// mc_packStopKVs — scan kvs for "stop" and "sensor" entries; pack each as a
// STR arg "stop=<value>" or "sensor=<value>" into out.args[*idxInOut..].
//
// On return, *idxInOut is advanced past the last packed arg.
// Stops when out.count reaches MAX_ARGS.
// ---------------------------------------------------------------------------
static void mc_packStopKVs(const KVPair* kvs, int nkv,
                            ArgList& out, int& idxInOut)
{
    for (int i = 0; i < nkv; ++i) {
        if (idxInOut >= MAX_ARGS) break;
        if (!kvs[i].key || !kvs[i].value) continue;

        bool isStop   = (strcmp(kvs[i].key, "stop") == 0);
        bool isSensor = (strcmp(kvs[i].key, "sensor") == 0);
        if (!isStop && !isSensor) continue;

        Argument& a = out.args[idxInOut];
        a.type = ArgType::STR;
        a.ival = 0;
        a.fval = 0.0f;
        const char* prefix = isStop ? "stop=" : "sensor=";
        int j = 0;
        const char* src = prefix;
        while (*src && j < (int)(sizeof(a.sval) - 1))
            a.sval[j++] = *src++;
        src = kvs[i].value;
        while (*src && j < (int)(sizeof(a.sval) - 1))
            a.sval[j++] = *src++;
        a.sval[j] = '\0';
        out.count = ++idxInOut;
    }
}

// ---------------------------------------------------------------------------
// pushVW — build a VW ParsedCommand and push_front it onto the queue.
//
// args must already contain args[0].ival=v (mm/s), args[1].ival=omega (mrad/s),
// and any stop params packed in args[2..] as ArgType::STR "key=value" strings.
//
// Returns false if the queue is null (caller falls back to direct begin*() call).
// ---------------------------------------------------------------------------
static bool pushVW(MotionCtx* ctx, const ArgList& args,
                   const char* corrId, ReplyFn replyFn, void* replyCtx)
{
    if (ctx->queue == nullptr) return false;
    ParsedCommand pc;
    pc.desc    = &ctx->vwDesc;
    pc.args    = args;
    pc.replyFn = replyFn;
    pc.replyCtx = replyCtx;
    int cidLen = 0;
    while (corrId && corrId[cidLen] != '\0' && cidLen < (int)(sizeof(pc.corrId) - 1)) {
        pc.corrId[cidLen] = corrId[cidLen];
        ++cidLen;
    }
    pc.corrId[cidLen] = '\0';
    return ctx->queue->push_front(pc);
}

// ── Helper: pack a "key=value" STR arg ─────────────────────────────────────
static int packKVArg(ArgList& out, int idx, const char* key, int ival)
{
    out.args[idx].type = ArgType::STR;
    out.args[idx].ival = 0;
    out.args[idx].fval = 0.0f;
    snprintf(out.args[idx].sval, sizeof(out.args[idx].sval), "%s=%d", key, ival);
    return idx + 1;
}

// ── Inline KV helpers for handleVW (scan STR args[2..] for "key=value") ────
//
// These replace the deleted vwHasKey / vwScanKV local helpers.  They scan
// args.args[2..args.count-1] for STR entries whose prefix matches "key=",
// mirroring the original logic exactly.

static bool argsHasKey(const ArgList& args, const char* key)
{
    int keyLen = 0;
    while (key[keyLen]) ++keyLen;
    for (int i = 2; i < args.count; ++i) {
        if (args.args[i].type != ArgType::STR) continue;
        const char* s = args.args[i].sval;
        int j = 0;
        while (j < keyLen && s[j] == key[j]) ++j;
        if (j == keyLen && s[j] == '=') return true;
    }
    return false;
}

static int argsScanKV(const ArgList& args, const char* key, int defVal)
{
    int keyLen = 0;
    while (key[keyLen]) ++keyLen;
    for (int i = 2; i < args.count; ++i) {
        if (args.args[i].type != ArgType::STR) continue;
        const char* s = args.args[i].sval;
        int j = 0;
        while (j < keyLen && s[j] == key[j]) ++j;
        if (j == keyLen && s[j] == '=') {
            return atoi(s + j + 1);
        }
    }
    return defVal;
}

// ---------------------------------------------------------------------------
// Argument schemas — declarative replacements for bespoke parse functions.
// ---------------------------------------------------------------------------

// G <x> <y> <speed> — 3 mandatory ranged INTs.
static const ArgDef gDefs[3] = {
    { "x",     ArgKind::INT, true, -10000, 10000 },
    { "y",     ArgKind::INT, true, -10000, 10000 },
    { "speed", ArgKind::INT, true,      1,  1000 },
};
static const ArgSchema gSchema = { gDefs, 3, 3, false, nullptr };

// RT <deg> — 1 mandatory ranged INT.
static const ArgDef rtDefs[1] = {
    { "deg", ArgKind::INT, true, -180000, 180000 },
};
static const ArgSchema rtSchema = { rtDefs, 1, 1, false, nullptr };

// X [soft] — 0 or 1 optional token; variadic so "soft" token is captured as STR.
static const ArgSchema xSchema = { nullptr, 0, 0, true, nullptr };

// ---------------------------------------------------------------------------
// parseS — S <l> <r> [stop=...]
//
// Parses 2 ranged INTs and collects any "stop=" / "sensor=" KV pairs
// (back-compat for "sensor=") from kvs into trailing STR args.
// ---------------------------------------------------------------------------
static ParseResult parseS(const char* const* tokens, int ntokens,
                          const KVPair* kvs, int nkv)
{
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
    mc_packStopKVs(kvs, nkv, res.args, idx);
    return res;
}

// ---------------------------------------------------------------------------
// parseT — T <l> <r> <ms> [stop=...] [sensor=...]
//
// Parses 3 ranged INTs and collects any "stop=" / "sensor=" KV pairs
// from kvs into trailing STR args (with full prefix, e.g. "stop=d:300",
// "sensor=line0:ge:512").  Replaces the schema-based tSchema + packKv="sensor".
// ---------------------------------------------------------------------------
static ParseResult parseT(const char* const* tokens, int ntokens,
                          const KVPair* kvs, int nkv)
{
    ParseResult res;
    if (ntokens < 3) {
        res.ok = false; res.err.code = nullptr; res.err.detail = nullptr; return res;
    }
    int l  = atoi(tokens[0]);
    int r  = atoi(tokens[1]);
    int ms = atoi(tokens[2]);
    if (l  < -1000 || l  > 1000)  {
        res.ok = false; res.err.code = "range"; res.err.detail = "l";  return res;
    }
    if (r  < -1000 || r  > 1000)  {
        res.ok = false; res.err.code = "range"; res.err.detail = "r";  return res;
    }
    if (ms < 1     || ms > 30000) {
        res.ok = false; res.err.code = "range"; res.err.detail = "ms"; return res;
    }
    res.ok = true;
    res.args.count = 3;
    argInt(res.args.args[0], l);
    argInt(res.args.args[1], r);
    argInt(res.args.args[2], ms);
    int idx = 3;
    mc_packStopKVs(kvs, nkv, res.args, idx);
    return res;
}

// ---------------------------------------------------------------------------
// parseD — D <l> <r> <mm> [stop=...] [sensor=...]
//
// Parses 3 ranged INTs and collects any "stop=" / "sensor=" KV pairs
// from kvs into trailing STR args.  Replaces the schema-based dSchema.
// ---------------------------------------------------------------------------
static ParseResult parseD(const char* const* tokens, int ntokens,
                          const KVPair* kvs, int nkv)
{
    ParseResult res;
    if (ntokens < 3) {
        res.ok = false; res.err.code = nullptr; res.err.detail = nullptr; return res;
    }
    int l  = atoi(tokens[0]);
    int r  = atoi(tokens[1]);
    int mm = atoi(tokens[2]);
    if (l  < -1000 || l  > 1000)  {
        res.ok = false; res.err.code = "range"; res.err.detail = "l";  return res;
    }
    if (r  < -1000 || r  > 1000)  {
        res.ok = false; res.err.code = "range"; res.err.detail = "r";  return res;
    }
    if (mm < 1     || mm > 10000) {
        res.ok = false; res.err.code = "range"; res.err.detail = "mm"; return res;
    }
    res.ok = true;
    res.args.count = 3;
    argInt(res.args.args[0], l);
    argInt(res.args.args[1], r);
    argInt(res.args.args[2], mm);
    int idx = 3;
    mc_packStopKVs(kvs, nkv, res.args, idx);
    return res;
}

// ---------------------------------------------------------------------------
// parseR — R <speed> <radius> [stop=...]
//
// Parses 2 ranged INTs and collects any "stop=" / "sensor=" KV pairs
// from kvs into trailing STR args.  Replaces the schema-based rSchema.
// ---------------------------------------------------------------------------
static ParseResult parseR(const char* const* tokens, int ntokens,
                          const KVPair* kvs, int nkv)
{
    ParseResult res;
    if (ntokens < 2) {
        res.ok = false; res.err.code = nullptr; res.err.detail = nullptr; return res;
    }
    int speed  = atoi(tokens[0]);
    int radius = atoi(tokens[1]);
    if (speed  < -1000 || speed  > 1000)  {
        res.ok = false; res.err.code = "range"; res.err.detail = "speed";  return res;
    }
    if (radius < -10000 || radius > 10000) {
        res.ok = false; res.err.code = "range"; res.err.detail = "radius"; return res;
    }
    res.ok = true;
    res.args.count = 2;
    argInt(res.args.args[0], speed);
    argInt(res.args.args[1], radius);
    int idx = 2;
    mc_packStopKVs(kvs, nkv, res.args, idx);
    return res;
}

// ── S ────────────────────────────────────────────────────────────────────────

// handleS — velocity-goal (streamSeed) path (migrated 053-003).
//
// Computes (v, ω) from (l, r) via BodyKinematics::forward(), builds a
// GoalRequest with goal=VELOCITY and streamSeed=true (seeds BVC immediately,
// no trapezoid ramp), and calls requestGoal().  Any stop= / sensor= clauses
// packed by parseS into args[2..] are copied into gr.stops[] so they fire.
//
// Phase 1 deferral resolved: stop= clauses on S now attach to the active
// MotionCommand and fire normally.
//
// D11 note: S replies OK here in the handler (same as before), then calls
// requestGoal.  requestGoal internally calls beginVelocity which does NOT
// emit its own reply — the handler's replyOK covers the command.
static void handleS(const ArgList& args, const char* corrId,
                    ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    MotionCtx* ctx = static_cast<MotionCtx*>(handlerCtx);
    int l = args.args[0].ival;
    int r = args.args[1].ival;

    // Compute body twist via forward kinematics.
    float v_mms, omega_rads;
    BodyKinematics::forward((float)l, (float)r, ctx->robot->config.trackwidthMm,
                            v_mms, omega_rads);

    uint32_t now = ctx->robot->systemTime();

    // Build GoalRequest for VELOCITY with streamSeed=true (immediate seed, no ramp).
    GoalRequest gr{};
    gr.goal       = Goal::VELOCITY;
    gr.robot      = ctx->robot;
    gr.now_ms     = now;
    gr.replyFn    = replyFn;
    gr.replyCtx   = replyCtx;
    gr.corrId     = corrId;
    gr.v_mms      = v_mms;
    gr.omega_rads = omega_rads;
    gr.streamSeed = true;
    gr.doneLabel  = "EVT done S";

    // Pack any stop= / sensor= clauses from args[2..] (parsed and forwarded by
    // parseS via mc_packStopKVs).  These are STR args with full "stop=<value>"
    // or "sensor=<value>" prefixes.  Use mc_parseStopTokenInto to populate
    // gr.stops[] directly without needing an active MotionCommand.
    for (int i = 2; i < args.count && gr.nStops < 4; ++i) {
        if (args.args[i].type != ArgType::STR) continue;
        const char* s = args.args[i].sval;
        StopCondition cond;
        bool ok = false;
        if (strncmp(s, "stop=", 5) == 0) {
            ok = mc_parseStopTokenInto(s + 5, cond);
        } else if (strncmp(s, "sensor=", 7) == 0) {
            // Back-compat: "sensor=<ch>:<op>:<thr>" → SENSOR stop.
            uint8_t ch; float thr; StopCondition::Cmp cmp;
            if (mc_parseSensorToken(s + 7, ch, thr, cmp)) {
                cond = makeSensorStop(ch, thr, cmp);
                ok   = true;
            }
        }
        if (ok) gr.stops[gr.nStops++] = cond;
    }

    ctx->superstructure->requestGoal(gr);

    char body[32];
    snprintf(body, sizeof(body), "l=%d r=%d", l, r);
    char rbuf[64];
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "drive", body, corrId, replyFn, replyCtx);
}

// ── T ────────────────────────────────────────────────────────────────────────

// handleT — direct requestGoal(VELOCITY) with a TIME stop (053-004).
//
// Computes (v, ω) from (l, r) via forward kinematics, builds a GoalRequest
// with goal=VELOCITY, stops[0]=makeTimeStop(ms), doneLabel="EVT done T", and
// calls requestGoal directly.  Eliminates the stringify/inverse round-trip that
// previously packed "t=<ms>" into VW args and re-parsed them in handleVW.
//
// D11 note: replyOK is called BEFORE requestGoal (converter already replied;
// the queue-drain hop is eliminated).
//
// Falls back to direct beginTimed() when queue is null (sim path preserved).
static void handleT(const ArgList& args, const char* corrId,
                    ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    MotionCtx* ctx = static_cast<MotionCtx*>(handlerCtx);
    int l  = args.args[0].ival;
    int r  = args.args[1].ival;
    int ms = args.args[2].ival;

    if (ctx->queue != nullptr) {
        // Validate sensor= back-compat clauses BEFORE replying OK (N16 fix, 030-009).
        for (int i = 3; i < args.count; ++i) {
            if (args.args[i].type != ArgType::STR) continue;
            const char* s = args.args[i].sval;
            if (strncmp(s, "sensor=", 7) == 0) {
                uint8_t ch; float thr; StopCondition::Cmp cmp;
                if (!mc_parseSensorToken(s + 7, ch, thr, cmp)) {
                    char rbuf[80];
                    CommandProcessor::replyErr(rbuf, sizeof(rbuf), "badarg", "sensor",
                                               corrId, replyFn, replyCtx);
                    return;
                }
            }
        }

        // Compute body twist via forward kinematics (no integer truncation).
        float v_mms, omega_rads;
        BodyKinematics::forward((float)l, (float)r, ctx->robot->config.trackwidthMm,
                                v_mms, omega_rads);

        uint32_t now = ctx->robot->systemTime();

        // Build GoalRequest for VELOCITY + TIME stop — no pushVW, no KV packing.
        GoalRequest gr{};
        gr.goal       = Goal::VELOCITY;
        gr.robot      = ctx->robot;
        gr.now_ms     = now;
        gr.replyFn    = replyFn;
        gr.replyCtx   = replyCtx;
        gr.corrId     = corrId;
        gr.v_mms      = v_mms;
        gr.omega_rads = omega_rads;
        gr.doneLabel  = "EVT done T";
        gr.streamSeed = false;
        gr.stops[gr.nStops++] = makeTimeStop((float)ms);

        // Pack any additional stop= / sensor= clauses from args[3..].
        for (int i = 3; i < args.count && gr.nStops < MotionCommand::kMaxStopConds; ++i) {
            if (args.args[i].type != ArgType::STR) continue;
            const char* s = args.args[i].sval;
            StopCondition cond;
            if (strncmp(s, "stop=", 5) == 0 && mc_parseStopTokenInto(s + 5, cond)) {
                gr.stops[gr.nStops++] = cond;
            } else if (strncmp(s, "sensor=", 7) == 0) {
                uint8_t ch; float thr; StopCondition::Cmp cmp;
                if (mc_parseSensorToken(s + 7, ch, thr, cmp))
                    gr.stops[gr.nStops++] = makeSensorStop(ch, thr, cmp);
            }
        }

        // D11: reply before requestGoal (converter already replied;
        // handleVW is no longer called for T).
        char body[48];
        snprintf(body, sizeof(body), "l=%d r=%d ms=%d", l, r, ms);
        char rbuf[80];
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "drive", body, corrId, replyFn, replyCtx);
        ctx->superstructure->requestGoal(gr);
    } else {
        // Queue not available: fall back to direct beginTimed().
        ctx->mc->beginTimed((float)l, (float)r, (uint32_t)ms,
                            ctx->robot->systemTime(),
                            ctx->robot->state.desired,
                            replyFn, replyCtx, corrId);
        // Apply any stop= / sensor= clauses packed by parseT at args[3..].
        mc_applyStopClauses(args, 3, ctx->mc->activeCmd());
        char body[48];
        snprintf(body, sizeof(body), "l=%d r=%d ms=%d", l, r, ms);
        char rbuf[80];
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "drive", body, corrId, replyFn, replyCtx);
    }
}

// ── D ────────────────────────────────────────────────────────────────────────

// handleD — direct requestGoal(DISTANCE) with a DISTANCE stop (053-004).
//
// Builds a GoalRequest with goal=DISTANCE (preserving the atomic encoder reset
// via robot->distanceDrive in Superstructure::requestGoal), leftMms/rightMms
// as integer wheel speeds, targetMm=mm, and doneLabel="EVT done D".
// Eliminates the stringify/inverse round-trip. gr.stops[] carries only
// wire-supplied stop=/sensor= clauses (065-001 / CR-01) — the primary
// DISTANCE+TIME stop pair is installed internally by distanceDrive() ->
// beginDistance(), NOT pre-populated here (that would double-book it, since
// Superstructure::requestGoal's DISTANCE case re-adds every entry of
// gr.stops[] on top of what beginDistance() already installed).
//
// Architecture note: Goal::DISTANCE is KEPT (not collapsed to VELOCITY) precisely
// to preserve the atomic encoder reset (beginDistance + resetEncoders) that
// Robot::distanceDrive performs.  The Superstructure DISTANCE case routes through
// robot->distanceDrive and applies doneLabel/stops[] after the call.
//
// D11 note: replyOK is called BEFORE requestGoal (converter already replied;
// the queue-drain hop is eliminated).
//
// Falls back to direct distanceDrive() when queue is null (sim path preserved).
static void handleD(const ArgList& args, const char* corrId,
                    ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    MotionCtx* ctx = static_cast<MotionCtx*>(handlerCtx);
    int l  = args.args[0].ival;
    int r  = args.args[1].ival;
    int mm = args.args[2].ival;

    if (ctx->queue != nullptr) {
        // Validate sensor= back-compat clauses BEFORE replying OK (N16 fix, 030-009).
        for (int i = 3; i < args.count; ++i) {
            if (args.args[i].type != ArgType::STR) continue;
            const char* s = args.args[i].sval;
            if (strncmp(s, "sensor=", 7) == 0) {
                uint8_t ch; float thr; StopCondition::Cmp cmp;
                if (!mc_parseSensorToken(s + 7, ch, thr, cmp)) {
                    char rbuf[80];
                    CommandProcessor::replyErr(rbuf, sizeof(rbuf), "badarg", "sensor",
                                               corrId, replyFn, replyCtx);
                    return;
                }
            }
        }

        uint32_t now = ctx->robot->systemTime();

        // Build GoalRequest for DISTANCE — wheel speeds passed as integers,
        // matching distanceDrive's (int32_t vL, int32_t vR, int32_t targetMm) signature.
        GoalRequest gr{};
        gr.goal     = Goal::DISTANCE;
        gr.robot    = ctx->robot;
        gr.now_ms   = now;
        gr.replyFn  = replyFn;
        gr.replyCtx = replyCtx;
        gr.corrId   = corrId;
        gr.leftMms  = (float)l;   // int32_t cast preserved inside Superstructure
        gr.rightMms = (float)r;
        gr.targetMm = (int32_t)mm;
        gr.doneLabel = "EVT done D";
        // NOTE (065-001 / CR-01): do NOT pre-populate gr.stops[0] with a
        // makeDistanceStop(mm) here. distanceDrive() -> beginDistance() already
        // installs its own DISTANCE + TIME stops internally; Superstructure::
        // requestGoal's DISTANCE case re-adds every entry of gr.stops[] on top
        // of those, so a pre-added stop here double-books the primary DISTANCE
        // condition (wasted on plain D; overflows kMaxStopConds once 2+ wire
        // clauses are also supplied). gr.stops[] carries only wire-supplied
        // stop=/sensor= clauses, starting at index 0.

        // Pack any additional stop= / sensor= clauses from args[3..].
        for (int i = 3; i < args.count && gr.nStops < MotionCommand::kMaxStopConds; ++i) {
            if (args.args[i].type != ArgType::STR) continue;
            const char* s = args.args[i].sval;
            StopCondition cond;
            if (strncmp(s, "stop=", 5) == 0 && mc_parseStopTokenInto(s + 5, cond)) {
                gr.stops[gr.nStops++] = cond;
            } else if (strncmp(s, "sensor=", 7) == 0) {
                uint8_t ch; float thr; StopCondition::Cmp cmp;
                if (mc_parseSensorToken(s + 7, ch, thr, cmp))
                    gr.stops[gr.nStops++] = makeSensorStop(ch, thr, cmp);
            }
        }

        // D11: reply before requestGoal (converter already replied;
        // handleVW is no longer called for D).
        char body[48];
        snprintf(body, sizeof(body), "l=%d r=%d mm=%d", l, r, mm);
        char rbuf[80];
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "drive", body, corrId, replyFn, replyCtx);
        ctx->superstructure->requestGoal(gr);
    } else {
        // Queue not available: fall back to direct distanceDrive() (resets enc baseline).
        ctx->robot->distanceDrive((int32_t)l, (int32_t)r, (int32_t)mm,
                                   replyFn, replyCtx, corrId);
        // Apply any stop= / sensor= clauses packed by parseD at args[3..].
        mc_applyStopClauses(args, 3, ctx->mc->activeCmd());
        char body[48];
        snprintf(body, sizeof(body), "l=%d r=%d mm=%d", l, r, mm);
        char rbuf[80];
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "drive", body, corrId, replyFn, replyCtx);
    }
}

// ── G ────────────────────────────────────────────────────────────────────────

// handleG — VW converter with x=<mm>, y=<mm>, speed=<mm/s> stop params.
//
// G is a go-to command: VW args use speed as v, 0 as omega (G's own logic
// computes steering), with "x=<mm>", "y=<mm>", "speed=<mm/s>" stop params.
// Falls back to direct beginGoTo() when queue is null.
static void handleG(const ArgList& args, const char* corrId,
                    ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    MotionCtx* ctx = static_cast<MotionCtx*>(handlerCtx);
    int x     = args.args[0].ival;
    int y     = args.args[1].ival;
    int speed = args.args[2].ival;

    if (ctx->queue != nullptr) {
        ArgList vwArgs;
        vwArgs.count = 2;
        argInt(vwArgs.args[0], speed);   // v = speed
        argInt(vwArgs.args[1], 0);       // omega = 0 (G's steering computed by VW handler)
        vwArgs.count = packKVArg(vwArgs, 2, "x", x);
        vwArgs.count = packKVArg(vwArgs, vwArgs.count, "y", y);
        vwArgs.count = packKVArg(vwArgs, vwArgs.count, "speed", speed);

        char body[64];
        snprintf(body, sizeof(body), "x=%d y=%d speed=%d", x, y, speed);
        char rbuf[96];
        if (!pushVW(ctx, vwArgs, corrId, replyFn, replyCtx)) {
            CommandProcessor::replyErr(rbuf, sizeof(rbuf), "full", nullptr, corrId, replyFn, replyCtx);
            return;
        }
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "goto", body, corrId, replyFn, replyCtx);
    } else {
        // Queue not available: fall back to direct beginGoTo().
        ctx->mc->beginGoTo((float)x, (float)y, (float)speed,
                           ctx->robot->systemTime(),
                           ctx->robot->state.desired,
                           replyFn, replyCtx, corrId);
        char body[64];
        snprintf(body, sizeof(body), "x=%d y=%d speed=%d", x, y, speed);
        char rbuf[96];
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "goto", body, corrId, replyFn, replyCtx);
    }
}

// ── R ────────────────────────────────────────────────────────────────────────

// handleR — direct requestGoal(VELOCITY) for arc command (053-005).
//
// R (arc) is an open-loop twist command: v = speed, omega = speed/radius
// (κ = 1/radius).  Computes omega inline and calls requestGoal(VELOCITY)
// directly — no stringify/re-parse round-trip through pushVW/handleVW.
//
// Any stop= clauses from args[2..] (packed by parseR via mc_packStopKVs)
// are forwarded into gr.stops[] so they fire normally.
//
// D11: replyOK is called before requestGoal (converter already replied;
// the queue-drain hop is eliminated).
//
// Falls back to direct beginVelocity() when queue is null (sim path).
static void handleR(const ArgList& args, const char* corrId,
                    ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    MotionCtx* ctx = static_cast<MotionCtx*>(handlerCtx);
    int speed  = args.args[0].ival;
    int radius = args.args[1].ival;

    // Compute omega = speed / radius (κ = 1/radius; 0 when radius == 0).
    // Sign convention: positive radius ⇒ positive ω ⇒ CCW (left arc).
    float omega_rads = (radius != 0) ? ((float)speed / (float)radius) : 0.0f;

    char body[48];
    snprintf(body, sizeof(body), "speed=%d radius=%d", speed, radius);
    char rbuf[80];

    if (ctx->queue != nullptr) {
        uint32_t now = ctx->robot->systemTime();

        // Build GoalRequest for VELOCITY — arc is an open-loop twist command.
        GoalRequest gr{};
        gr.goal       = Goal::VELOCITY;
        gr.robot      = ctx->robot;
        gr.now_ms     = now;
        gr.replyFn    = replyFn;
        gr.replyCtx   = replyCtx;
        gr.corrId     = corrId;
        gr.v_mms      = (float)speed;
        gr.omega_rads = omega_rads;
        gr.doneLabel  = "EVT done R";
        gr.streamSeed = false;

        // Pack any stop= / sensor= clauses from args[2..] (packed by parseR
        // via mc_packStopKVs; full "stop=<value>" / "sensor=<value>" prefixes).
        for (int i = 2; i < args.count && gr.nStops < MotionCommand::kMaxStopConds; ++i) {
            if (args.args[i].type != ArgType::STR) continue;
            const char* s = args.args[i].sval;
            StopCondition cond;
            if (strncmp(s, "stop=", 5) == 0 && mc_parseStopTokenInto(s + 5, cond)) {
                gr.stops[gr.nStops++] = cond;
            } else if (strncmp(s, "sensor=", 7) == 0) {
                uint8_t ch; float thr; StopCondition::Cmp cmp;
                if (mc_parseSensorToken(s + 7, ch, thr, cmp))
                    gr.stops[gr.nStops++] = makeSensorStop(ch, thr, cmp);
            }
        }

        // D11: reply before requestGoal (no queue hop needed).
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "arc", body, corrId, replyFn, replyCtx);
        ctx->superstructure->requestGoal(gr);
    } else {
        // Queue not available: fall back to direct beginVelocity().
        uint32_t now = ctx->robot->systemTime();
        ctx->mc->beginVelocity((float)speed, omega_rads, now,
                               ctx->robot->state.desired,
                               replyFn, replyCtx, corrId);
        // Set EVT done R label and apply stop= / sensor= clauses.
        ctx->mc->activeCmd().setDoneEvt("EVT done R");
        mc_applyStopClauses(args, 2, ctx->mc->activeCmd());
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "arc", body, corrId, replyFn, replyCtx);
    }
}

// ── TURN ─────────────────────────────────────────────────────────────────────

static ParseResult parseTURN(const char* const* tokens, int ntokens,
                              const KVPair* kvs, int nkv)
{
    ParseResult res;
    if (ntokens < 1) {
        res.ok = false; res.err.code = "badarg"; res.err.detail = nullptr; return res;
    }
    int heading_cdeg = atoi(tokens[0]);
    if (heading_cdeg < -18000 || heading_cdeg > 18000) {
        res.ok = false; res.err.code = "range"; res.err.detail = "heading"; return res;
    }
    // Parse optional eps=<cdeg> kv; default 300.
    int eps_cdeg = 300;
    const KVPair* epsKv = kvFind(kvs, nkv, "eps");
    if (epsKv) {
        eps_cdeg = atoi(epsKv->value);
        if (eps_cdeg < 10 || eps_cdeg > 1800) {
            res.ok = false; res.err.code = "range"; res.err.detail = "eps"; return res;
        }
    }
    res.ok = true;
    res.args.count = 2;
    argInt(res.args.args[0], heading_cdeg);
    argInt(res.args.args[1], eps_cdeg);
    // Collect all stop= and sensor= KV pairs into trailing STR args with full
    // prefixes ("stop=<value>", "sensor=<value>") so handleVW mc_applyStopClauses
    // can process them.  mc_packStopKVs handles both keys in a single pass.
    int idx = 2;
    mc_packStopKVs(kvs, nkv, res.args, idx);
    return res;
}

// handleTURN — VW converter with "h=<cdeg>", "eps=<cdeg>" stop params.
//
// TURN is an absolute-heading rotation: v = 0 (spin-in-place), omega is
// computed by VW handler from heading_cdeg. Stop params "h=<cdeg>" and
// "eps=<cdeg>" tell VW handler to call beginTurn().
// Falls back to direct beginTurn() when queue is null.
static void handleTURN(const ArgList& args, const char* corrId,
                       ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    MotionCtx* ctx = static_cast<MotionCtx*>(handlerCtx);
    int heading_cdeg = args.args[0].ival;
    int eps_cdeg     = args.args[1].ival;

    if (ctx->queue != nullptr) {
        // N16 fix (030-009): validate sensor= back-compat clauses BEFORE replying OK.
        // parseTURN now packs all stop= / sensor= at args[2..] with full prefixes.
        for (int i = 2; i < args.count; ++i) {
            if (args.args[i].type != ArgType::STR) continue;
            const char* s = args.args[i].sval;
            if (strncmp(s, "sensor=", 7) == 0) {
                uint8_t ch; float thr; StopCondition::Cmp cmp;
                if (!mc_parseSensorToken(s + 7, ch, thr, cmp)) {
                    char rbuf[80];
                    CommandProcessor::replyErr(rbuf, sizeof(rbuf), "badarg", "sensor",
                                               corrId, replyFn, replyCtx);
                    return;
                }
            }
        }

        ArgList vwArgs;
        vwArgs.count = 2;
        argInt(vwArgs.args[0], 0);   // v = 0 (spin-in-place; omega computed by VW handler)
        argInt(vwArgs.args[1], 0);   // omega placeholder; VW handler uses "h" param
        vwArgs.count = packKVArg(vwArgs, 2, "h", heading_cdeg);
        vwArgs.count = packKVArg(vwArgs, vwArgs.count, "eps", eps_cdeg);

        // Forward all stop= / sensor= tokens from args[2..] into vwArgs.
        for (int i = 2; i < args.count && vwArgs.count < MAX_ARGS; ++i) {
            vwArgs.args[vwArgs.count++] = args.args[i];
        }

        char body[48];
        snprintf(body, sizeof(body), "heading=%d eps=%d", heading_cdeg, eps_cdeg);
        char rbuf[80];
        if (!pushVW(ctx, vwArgs, corrId, replyFn, replyCtx)) {
            CommandProcessor::replyErr(rbuf, sizeof(rbuf), "full", nullptr, corrId, replyFn, replyCtx);
            return;
        }
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "turn", body, corrId, replyFn, replyCtx);
    } else {
        // Queue not available: fall back to direct beginTurn().
        uint32_t now = ctx->robot->systemTime();
        ctx->mc->beginTurn((float)heading_cdeg, (float)eps_cdeg, now,
                           ctx->robot->state.desired,
                           replyFn, replyCtx, corrId);
        // Apply any stop= / sensor= clauses packed by parseTURN at args[2..].
        mc_applyStopClauses(args, 2, ctx->mc->activeCmd());
        char body[48];
        snprintf(body, sizeof(body), "heading=%d eps=%d", heading_cdeg, eps_cdeg);
        char rbuf[80];
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "turn", body, corrId, replyFn, replyCtx);
    }
}

// ── RT (relative turn, encoder-arc stop) ───────────────────────────────────────

// handleRT — RELATIVE spin-in-place by rel_cdeg, stopped on encoder arc.
// Enqueues a VW with "rot=<cdeg>" so the VW handler (loop context) calls
// beginRotation(). Falls back to a direct call when the queue is null.
static void handleRT(const ArgList& args, const char* corrId,
                     ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    MotionCtx* ctx = static_cast<MotionCtx*>(handlerCtx);
    int rel_cdeg = args.args[0].ival;

    if (ctx->queue != nullptr) {
        ArgList vwArgs;
        vwArgs.count = 2;
        argInt(vwArgs.args[0], 0);   // v = 0 (spin in place)
        argInt(vwArgs.args[1], 0);   // omega placeholder (computed by beginRotation)
        vwArgs.count = packKVArg(vwArgs, 2, "rot", rel_cdeg);
        char body[32];
        snprintf(body, sizeof(body), "rot=%d", rel_cdeg);
        char rbuf[64];
        if (!pushVW(ctx, vwArgs, corrId, replyFn, replyCtx)) {
            CommandProcessor::replyErr(rbuf, sizeof(rbuf), "full", nullptr, corrId, replyFn, replyCtx);
            return;
        }
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "rt", body, corrId, replyFn, replyCtx);
    } else {
        uint32_t now = ctx->robot->systemTime();
        ctx->mc->beginRotation((float)rel_cdeg, now, ctx->robot->state.desired,
                               replyFn, replyCtx, corrId);
        char body[32];
        snprintf(body, sizeof(body), "rot=%d", rel_cdeg);
        char rbuf[64];
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "rt", body, corrId, replyFn, replyCtx);
    }
}

// ── VW ───────────────────────────────────────────────────────────────────────
//
// VW — body-twist velocity command (open-ended, keepalive watchdog).
//
// Wire format: VW <v_mms> <omega_mrads> [key=value ...]
//   args[0].ival = v in mm/s
//   args[1].ival = omega in mrad/s (wire units; converted to rad/s here)
//   args[2..] = optional stop params (ArgType::STR "key=value"):
//     "x=<mm>"+"y=<mm>"+"speed=<mm/s>" → call beginGoTo(x, y, speed, ...)
//     "h=<cdeg>"+"eps=<cdeg>"           → call beginTurn(h_cdeg, eps_cdeg, ...)
//     "rot=<cdeg>"                      → call beginRotation(rot_cdeg, ...)
//     (no stop params) → open-ended velocity (beginVelocity or keepalive re-arm)
//
// Note (053-003): the "stream=1" KV branch has been removed.  S is now routed
// directly through requestGoal(VELOCITY, streamSeed=true) in handleS and no
// longer pushes a VW command onto the queue.
//
// Note (053-004): the "t=<ms>" and "dist=<mm>" KV branches have been removed.
// T and D now call requestGoal directly from handleT/handleD without pushing
// a VW command onto the queue.
//
// Note (053-005): the "radius=<mm>" / "speed=<mm/s>" KV branch has been
// removed.  R now calls requestGoal(VELOCITY) directly from handleR, computing
// omega = speed/radius inline and never pushing a VW command onto the queue.
//
// D11 suppression: when dispatched from a converter push (stop-param branches),
// handleVW does NOT call replyOK — the converter handler already replied.
// Only the open-ended branch (no stop params) emits OK vw.

static ParseResult parseVW(const char* const* tokens, int ntokens,
                            const KVPair* kvs, int nkv)
{
    ParseResult res;
    // Differential build: 2-token only.
    if (ntokens < 2) {
        res.ok = false; res.err.code = "badarg"; res.err.detail = nullptr; return res;
    }
    int v     = atoi(tokens[0]);
    int omega = atoi(tokens[1]);
    if (v < -1000 || v > 1000) {
        res.ok = false; res.err.code = "range"; res.err.detail = "v"; return res;
    }
    if (omega < -3142 || omega > 3142) {
        res.ok = false; res.err.code = "range"; res.err.detail = "omega"; return res;
    }
    res.ok = true;
    res.args.count = 2;
    argInt(res.args.args[0], v);
    argInt(res.args.args[1], omega);
    // Collect any stop= / sensor= KV pairs from the wire into trailing STR args
    // so handleVW can apply them via mc_applyStopClauses after requestGoal.
    int idx = 2;
    mc_packStopKVs(kvs, nkv, res.args, idx);
    return res;
}

static void handleVW(const ArgList& args, const char* corrId,
                     ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    MotionCtx* ctx = static_cast<MotionCtx*>(handlerCtx);
    int v     = args.args[0].ival;
    int omega = args.args[1].ival;
    float omega_rads = (float)omega / 1000.0f;  // mrad/s → rad/s
    uint32_t now = ctx->robot->systemTime();

    // ── Stop-param dispatch ─────────────────────────────────────────────────
    // Converter handlers (S, T, D, G, R, TURN, RT) pack stop params as STR
    // args at args[2..].  Scan them here to select the appropriate begin*() call.
    //
    // D11 suppression: all stop-param branches call begin*() and return WITHOUT
    // calling replyOK.  The converter handler already emitted the reply before
    // pushing this VW ParsedCommand onto the queue.
    // Only the open-ended path (no stop params) emits "OK vw ...".

    // Check for RT (relative rotation): "rot=<cdeg>" present.
    if (argsHasKey(args, "rot")) {
        int rot_cdeg = argsScanKV(args, "rot", 0);
        // Seam 3 (042-001): route through requestGoal — same beginRotation call.
        GoalRequest gr{};
        gr.goal    = Goal::ROTATE;
        gr.robot   = ctx->robot;
        gr.now_ms  = now;
        gr.replyFn = replyFn;
        gr.replyCtx = replyCtx;
        gr.corrId  = corrId;
        gr.relCdeg = (float)rot_cdeg;
        ctx->superstructure->requestGoal(gr);
        // D11: no replyOK here — handleRT already replied.
        return;
    }

    // Check for TURN: "h=<cdeg>" present (and no "x" key).
    if (argsHasKey(args, "h") && !argsHasKey(args, "x")) {
        int h_cdeg  = argsScanKV(args, "h",   0);
        int eps     = argsScanKV(args, "eps", 300);

        // Seam 3 (042-001): route through requestGoal — same beginTurn call.
        GoalRequest gr{};
        gr.goal        = Goal::TURN;
        gr.robot       = ctx->robot;
        gr.now_ms      = now;
        gr.replyFn     = replyFn;
        gr.replyCtx    = replyCtx;
        gr.corrId      = corrId;
        gr.headingCdeg = (float)h_cdeg;
        gr.epsCdeg     = (float)eps;
        ctx->superstructure->requestGoal(gr);

        // Apply all stop= / sensor= stop clauses from args[2..] (packed by
        // parseTURN via mc_packStopKVs; both "stop=..." and "sensor=..." prefixes
        // handled by mc_applyStopClauses — replaces the old single-sensor= loop).
        mc_applyStopClauses(args, 2, ctx->mc->activeCmd());

        // D11: no replyOK here — handleTURN already replied.
        return;
    }

    // Check for G (go-to): "x=<mm>" and "y=<mm>" present.
    if (argsHasKey(args, "x") && argsHasKey(args, "y")) {
        int x_mm    = argsScanKV(args, "x",     0);
        int y_mm    = argsScanKV(args, "y",     0);
        int speed   = argsScanKV(args, "speed", v);  // fallback to v

        // Seam 3 (042-001): route through requestGoal — same beginGoTo call.
        GoalRequest gr{};
        gr.goal     = Goal::GOTO;
        gr.robot    = ctx->robot;
        gr.now_ms   = now;
        gr.replyFn  = replyFn;
        gr.replyCtx = replyCtx;
        gr.corrId   = corrId;
        gr.tx       = (float)x_mm;
        gr.ty       = (float)y_mm;
        gr.speedMms = (float)speed;
        ctx->superstructure->requestGoal(gr);

        // D11: no replyOK here — handleG already replied.
        return;
    }

    // ── No stop params: open-ended velocity ────────────────────────────────
    //
    // Note (053-005): the "radius=<mm>" / "speed=<mm/s>" KV branch that formerly
    // routed R-command pushes through handleVW has been removed.  R now calls
    // requestGoal(VELOCITY) directly from handleR, computing omega = speed/radius
    // inline and never reaching handleVW.
    //
    // Note (053-004): the "t=<ms>" and "dist=<mm>" KV branches have been removed.
    // T and D now call requestGoal directly from handleT/handleD without going
    // through the queue, so handleVW is never dispatched for T or D commands.
    //
    // Direct VW commands with no stop params use beginVelocity (MotionCommand
    // with ramp), which supports X soft + EVT done on completion.
    //
    // D11 note: this is the ONLY branch that emits replyOK.  The open-ended VW
    // path (no stop params) is the direct VW command; it emits exactly one
    // replyOK here.
    //
    // Note (053-003): the "stream=1" KV branch that formerly routed S-command
    // pushes through handleVW has been removed.  S now calls requestGoal
    // directly from handleS with streamSeed=true; it never reaches handleVW.
    if (ctx->mc->hasActiveCommand()) {
        // D6 origin guard: only update the target when the active command is
        // RETARGETABLE (VW-origin).  A FIXED command (TURN, G, T, D, R, RT)
        // must not have its target stomped — e.g. zeroing omega on an active
        // TURN stops the rotation prematurely and silently corrupts navigation.
        //
        // For FIXED commands: reset the system watchdog by returning a busy
        // reply and do NOT call setTarget.
        if (ctx->mc->activeCmd().origin() == MotionCommand::Origin::RETARGETABLE) {
            // VW keepalive: update target and re-arm.
            ctx->mc->activeCmd().setTarget((float)v, omega_rads);
        } else {
            // FIXED command active: reply busy, do not stomp target.
            const char* originName =
                (ctx->mc->activeCmd().origin() == MotionCommand::Origin::RETARGETABLE)
                ? "RETARGETABLE" : "FIXED";
            char rbuf[64];
            char body[32];
            snprintf(body, sizeof(body), "busy=%s", originName);
            CommandProcessor::replyOK(rbuf, sizeof(rbuf), "vw", body, corrId, replyFn, replyCtx);
            return;
        }
    } else {
        // New VW command: configure MotionCommand from scratch.
        // Seam 3 (042-001): route through requestGoal — same beginVelocity call.
        GoalRequest gr{};
        gr.goal       = Goal::VELOCITY;
        gr.robot      = ctx->robot;
        gr.now_ms     = now;
        gr.replyFn    = replyFn;
        gr.replyCtx   = replyCtx;
        gr.corrId     = corrId;
        gr.v_mms      = (float)v;
        gr.omega_rads = omega_rads;
        ctx->superstructure->requestGoal(gr);

        // Apply any stop= clauses from the wire (parseVW collected them at args[2..]).
        mc_applyStopClauses(args, 2, ctx->mc->activeCmd());
    }

    // Open-ended direct VW: emit exactly one OK reply.
    char body[32];
    snprintf(body, sizeof(body), "v=%d omega=%d", v, omega);
    char rbuf[64];
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "vw", body, corrId, replyFn, replyCtx);
}

// ── _VW ──────────────────────────────────────────────────────────────────────
// Raw velocity command: seeds BVC current state immediately (no ramp).
// Fire-and-forget — no MotionCommand, system watchdog handles keepalive.

static ParseResult parse_VW(const char* const* tokens, int ntokens,
                             const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult res;
    if (ntokens < 2) {
        res.ok = false; res.err.code = "badarg"; res.err.detail = nullptr; return res;
    }
    int v     = atoi(tokens[0]);
    int omega = atoi(tokens[1]);
    if (v < -1000 || v > 1000) {
        res.ok = false; res.err.code = "range"; res.err.detail = "v"; return res;
    }
    if (omega < -3142 || omega > 3142) {
        res.ok = false; res.err.code = "range"; res.err.detail = "omega"; return res;
    }
    res.ok = true;
    res.args.count = 2;
    argInt(res.args.args[0], v);
    argInt(res.args.args[1], omega);
    return res;
}

static void handle_VW(const ArgList& args, const char* corrId,
                      ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    MotionCtx* ctx = static_cast<MotionCtx*>(handlerCtx);
    int v     = args.args[0].ival;
    int omega = args.args[1].ival;
    float omega_rads = (float)omega / 1000.0f;  // mrad/s → rad/s
    ctx->mc->beginRawVelocity((float)v, omega_rads);
    char rbuf[64];
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "_VW", nullptr, corrId, replyFn, replyCtx);
}

// ── X and STOP ───────────────────────────────────────────────────────────────

static void handleX(const ArgList& args, const char* corrId,
                    ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    MotionCtx* ctx = static_cast<MotionCtx*>(handlerCtx);
    uint32_t now = ctx->robot->systemTime();

    // Check for "soft" positional arg — soft stop ramps BVC to zero.
    // xSchema is variadic so "soft" arrives as STR arg[0] when present.
    bool isSoft = (args.count >= 1 &&
                   args.args[0].type == ArgType::STR &&
                   strcmp(args.args[0].sval, "soft") == 0);

    if (isSoft) {
        ctx->mc->softStop(now);
    } else {
        ctx->mc->cancel(now, replyFn, replyCtx);
    }
    char rbuf[64];
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "x", nullptr, corrId, replyFn, replyCtx);
}

static void handleSTOP(const ArgList& /*args*/, const char* corrId,
                       ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    MotionCtx* ctx = static_cast<MotionCtx*>(handlerCtx);
    uint32_t now = ctx->robot->systemTime();
    ctx->mc->cancel(now, replyFn, replyCtx);
    char rbuf[64];
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "stop", nullptr, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// getMotionCommands — returns the full set of motion CommandDescriptors.
//
// ctx->vwDesc is initialised here so that converter handlers can build
// ParsedCommands targeting handleVW.
// ctx must remain live for the lifetime of the returned descriptors.
// ---------------------------------------------------------------------------
std::vector<CommandDescriptor> getMotionCommands(MotionCtx* ctx)
{
    // Initialise the stable VW descriptor that converter handlers reference
    // when building ParsedCommands to push_front onto the queue.
    ctx->vwDesc.prefix     = "VW";
    ctx->vwDesc.parseFn    = parseVW;
    ctx->vwDesc.handlerFn  = handleVW;
    ctx->vwDesc.handlerCtx = ctx;
    ctx->vwDesc.errFmt     = "badarg";
    ctx->vwDesc.forceReply = ForceReply::NONE;
    ctx->vwDesc.flags      = CMD_ACCESS_HARDWARE;

    return {
        makeCmd(      "S",    parseS,    handleS,    ctx, "badarg"),                                        // set wheel speeds (mm/s)
        makeCmd(      "T",    parseT,    handleT,    ctx, "badarg"),                                        // timed drive (ms)
        makeCmd(      "D",    parseD,    handleD,    ctx, "badarg"),                                        // distance drive (mm)
        makeSchemaCmd("G",    &gSchema,  handleG,    ctx, "badarg"),                                        // goto encoder position
        makeCmd(      "R",    parseR,    handleR,    ctx, "badarg"),                                        // arc drive: R <speed> <radius_mm>
        makeCmd(      "TURN", parseTURN, handleTURN, ctx, "badarg"),                                        // spin in place to absolute heading
        makeSchemaCmd("RT",   &rtSchema, handleRT,   ctx, "badarg"),                                        // relative spin by <cdeg>
        makeCmd(      "VW",   parseVW,   handleVW,   ctx, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE), // body-twist velocity
        makeCmd(      "_VW",  parse_VW,  handle_VW,  ctx, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE), // raw velocity (no ramp)
        makeSchemaCmd("X",    &xSchema,  handleX,    ctx, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE), // stop / soft stop
        makeCmd(      "STOP", nullptr,   handleSTOP, ctx, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE), // hard stop
    };
}
