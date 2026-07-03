// ---------------------------------------------------------------------------
// SystemCommands.cpp — system command handlers and Robot::buildCommandTable
//
// Split from Robot.cpp (sprint 035 A3). Contains:
//   - file-local static parse*/handle* functions for the system commands
//     (HELLO, PING, ECHO, ID, VER, HELP, SNAP, ZERO, HALT, STREAM, RF,
//      +, SAFE, SI, GET VEL, GET, SET)
//   - Robot::buildCommandTable definition
//
// All handlers remain Robot:: member calls via RobotSysCtx* handlerCtx.
// Class layout and Robot.h are unchanged.
//
// Migration (051-005): no-arg parse stubs removed (parseFn=nullptr);
// ECHO/STREAM/SAFE migrated to variadic ArgSchema; SI/RF migrated to
// positional ArgSchema. ZERO/HALT/BAUD retain custom parseFn.
// ---------------------------------------------------------------------------

#include "Robot.h"
#include "CommandProcessor.h"
#include "MotionCommands.h"
#include "ConfigCommands.h"
#include "DebugCommands.h"
#include "ArgParse.h"

// SimCommands (069-003) is sim-build-only.  This #include is the ONLY place
// (besides tests/_infra/sim/sim_api.cpp, which constructs one) SimCommands.h
// is #included in a compilation of this shared translation unit; on the ARM
// build (HOST_BUILD undefined) it is skipped entirely, so SimCommands is
// never a complete type there and PhysicsWorld/SimHardware never enter the
// ARM link (architecture-update.md Design Rationale Decision 1). The root
// CMakeLists.txt additionally excludes source/commands/SimCommands.cpp
// itself from the ARM (CODAL) source glob, mirroring the existing hal/sim/
// exclusion.
#ifdef HOST_BUILD
#include "SimCommands.h"
#endif

#ifndef HOST_BUILD
#include "MicroBit.h"
#include "MicroBitDevice.h"
#include "LoopScheduler.h"
#include "Communicator.h"
#include "Radio.h"
#include "RadioChannel.h"
#endif

#include <cstdio>
#include <cstring>
#include <cstdlib>

// ---------------------------------------------------------------------------
// HOST_BUILD stubs — microbit_friendly_name / microbit_serial_number.
// system_timer_current_time and g_sim_now_ms stay in Robot.cpp (used by
// Robot::systemTime only).
// ---------------------------------------------------------------------------
#ifdef HOST_BUILD
static const char* microbit_friendly_name() { return "sim"; }
static uint32_t    microbit_serial_number()  { return 0; }
#endif

// ===========================================================================
// buildCommandTable — system command handlers + aggregation
//
// All system command handlers are static functions defined here.
// handlerCtx is always RobotSysCtx* (cast inside each handler).
// ===========================================================================

// ---------------------------------------------------------------------------
// Internal accessor -- cast handlerCtx to RobotSysCtx*.
// ---------------------------------------------------------------------------
namespace {

static RobotSysCtx& ctxFrom(void* p)
{
    return *reinterpret_cast<RobotSysCtx*>(p);
}

// ---------------------------------------------------------------------------
// HELLO -- raw DEVICE banner (no OK wrapper).
//   prefix "HELLO"; parseFn nullptr; no args.
//   Output: DEVICE:NEZHA2:robot:<name>:<serial>
// ---------------------------------------------------------------------------

static void handleHello(const ArgList& /*args*/, const char* /*corrId*/,
                         ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    (void)handlerCtx;
    const char* name   = microbit_friendly_name();
    uint32_t    serial = microbit_serial_number();
    char banner[64];
    snprintf(banner, sizeof(banner),
             "DEVICE:NEZHA2:robot:%s:%lu", name, (unsigned long)serial);
    replyFn(banner, replyCtx);
}

// ---------------------------------------------------------------------------
// PING -- clock-sync probe.
//   prefix "PING"; parseFn nullptr.
//   Reply: OK pong t=<ms>
// ---------------------------------------------------------------------------

static void handlePing(const ArgList& /*args*/, const char* corrId,
                        ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    Robot* robot = ctxFrom(handlerCtx).robot;
    uint32_t t = robot->systemTime();
    char rbuf[64];
    char body[32];
    snprintf(body, sizeof(body), "t=%lu", (unsigned long)t);
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "pong", body, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// Argument schemas — declarative replacements for ECHO, STREAM, SAFE, SI, RF.
// ---------------------------------------------------------------------------

// ECHO — variadic: all tokens passed through as STR args.
static const ArgSchema echoSchema = { nullptr, 0, 0, true, nullptr };

// STREAM — custom parseFn (not a plain variadic ArgSchema): see parseStream
// below, which reconstructs "fields=<csv>" from kvs[] rather than tokens[].

// SAFE — variadic: "off"/"on"/numeric tokens passed as STR args.
static const ArgSchema safeSchema = { nullptr, 0, 0, true, nullptr };

// SI <x_mm> <y_mm> <h_cdeg> — 3 mandatory INTs; ranged=false (plain atoi, no range check).
static const ArgDef siDefs[3] = {
    { "x_mm",   ArgKind::INT, false, 0, 0 },
    { "y_mm",   ArgKind::INT, false, 0, 0 },
    { "h_cdeg", ArgKind::INT, false, 0, 0 },
};
static const ArgSchema siSchema = { siDefs, 3, 3, false, nullptr };

// RF [chan] — 0 or 1 optional INT; ranged=false.
static const ArgDef rfDefs[1] = {
    { "chan", ArgKind::INT, false, 0, 0 },
};
static const ArgSchema rfSchema = { rfDefs, 1, 0, false, nullptr };

// ---------------------------------------------------------------------------
// ECHO -- echo payload tokens back.
//   prefix "ECHO"; variadic ArgSchema; stores tokens as STR args.
//   Reply: OK echo <joined tokens>
// ---------------------------------------------------------------------------

static void handleEcho(const ArgList& args, const char* corrId,
                        ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    (void)handlerCtx;
    // Reassemble tokens into a single space-joined payload.
    char payload[512];
    int pos = 0;
    for (int i = 0; i < args.count && pos < (int)sizeof(payload) - 2; ++i) {
        if (i > 0) payload[pos++] = ' ';
        for (const char* c = args.args[i].sval;
             *c != '\0' && pos < (int)sizeof(payload) - 1; ++c)
            payload[pos++] = *c;
    }
    payload[pos] = '\0';

    char rbuf[520];
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "echo", payload, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// ID -- full identification response.
//   prefix "ID"; parseFn nullptr.
//   Reply: ID model=Nezha2 name=<n> serial=<s> fw=<ver> proto=2 caps=<c>
// ---------------------------------------------------------------------------

static void handleId(const ArgList& /*args*/, const char* corrId,
                      ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    Robot* robot  = ctxFrom(handlerCtx).robot;
    const char* name   = microbit_friendly_name();
    uint32_t    serial = microbit_serial_number();

    char caps[64];
    caps[0] = '\0';
    bool first = true;
    auto addCap = [&](const char* cap) {
        if (!first) {
            int n = (int)strlen(caps);
            caps[n] = ','; caps[n+1] = '\0';
        }
        int rem = (int)(sizeof(caps) - strlen(caps) - 1);
        if (rem > 0) strncat(caps, cap, (size_t)rem);
        first = false;
    };
    if (robot->otos.is_initialized())        addCap("otos");
    if (robot->line.is_initialized())        addCap("line");
    if (robot->colorSensor.is_initialized()) addCap("color");
    addCap("portio");

    char rbuf[520];
    if (corrId && corrId[0] != '\0') {
        snprintf(rbuf, sizeof(rbuf),
                 "ID model=Nezha2 name=%s serial=%lu fw=%s proto=%d caps=%s #%s",
                 name, (unsigned long)serial, FIRMWARE_VERSION, PROTO_VERSION,
                 caps, corrId);
    } else {
        snprintf(rbuf, sizeof(rbuf),
                 "ID model=Nezha2 name=%s serial=%lu fw=%s proto=%d caps=%s",
                 name, (unsigned long)serial, FIRMWARE_VERSION, PROTO_VERSION,
                 caps);
    }
    replyFn(rbuf, replyCtx);
}

// ---------------------------------------------------------------------------
// VER -- firmware/protocol version query.
//   prefix "VER"; parseFn nullptr.
//   Reply: OK ver fw=<ver> proto=2
// ---------------------------------------------------------------------------

static void handleVer(const ArgList& /*args*/, const char* corrId,
                       ReplyFn replyFn, void* replyCtx, void* /*handlerCtx*/)
{
    char rbuf[64];
    char body[64];
    snprintf(body, sizeof(body), "fw=%s proto=%d", FIRMWARE_VERSION, PROTO_VERSION);
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "ver", body, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// HELP -- list all verbs.
//   prefix "HELP"; parseFn nullptr.
//   Reply: OK help <verb list>
// ---------------------------------------------------------------------------

static void handleHelp(const ArgList& /*args*/, const char* corrId,
                        ReplyFn replyFn, void* replyCtx, void* /*handlerCtx*/)
{
    char rbuf[520];
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "help",
        "PING ECHO ID VER HELP SET GET GET VEL STREAM SNAP "
        "S T D G R TURN RT VW RF X STOP GRIP ZERO + SAFE "
        "OI OZ OR OP OV OL OA P PA "
        "[sensor=<ch>:<op>:<thr>]",
        corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// SNAP -- synchronous telemetry frame.
//   prefix "SNAP"; parseFn nullptr.
//   Reply: TLM ... (raw frame, not OK-wrapped)
//
// Tick-ordering limitation (field-024 lead A, 028-001):
//   SNAP is dispatched by cmd.dequeueOne() at the START of loopTickOnce(),
//   BEFORE driveAdvance() runs.  At a mode-transition boundary (e.g. the
//   first tick after a G/T/D command arrives), SNAP reports end-of-last-tick
//   state -- so mode=IDLE and enc=0 are possible even while the robot is
//   physically moving.  After the first post-command tick, SNAP reflects live
//   state correctly.
//
//   The real fix for host-visible frame staleness is D10 seq numbers
//   (ticket 028-005): the shared _tlmSeq counter on both SNAP and STREAM lets
//   the host detect/skip frames from before a motion phase started.
// ---------------------------------------------------------------------------

static void handleSnap(const ArgList& /*args*/, const char* /*corrId*/,
                        ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    Robot* robot = ctxFrom(handlerCtx).robot;
    char tlmBuf[160];
    robot->buildTlmFrame(tlmBuf, sizeof(tlmBuf));
    replyFn(tlmBuf, replyCtx);
}

// ---------------------------------------------------------------------------
// ZERO -- zero encoders and/or odometry.
//   prefix "ZERO"; parseFn passes "enc"/"pose" token args.
//   Reply: OK zero <enc|pose|enc pose>
// ---------------------------------------------------------------------------

static ParseResult parseZero(const char* const* tokens, int ntokens,
                              const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult r;
    if (ntokens < 1) {
        r.ok = false;
        r.err = { "badarg", nullptr };
        return r;
    }
    // Accept enc, pose, T, D. At least one must be present.
    bool hasEnc  = false;
    bool hasPose = false;
    bool hasT    = false;
    bool hasD    = false;
    for (int i = 0; i < ntokens; ++i) {
        if (strcmp(tokens[i], "enc")  == 0) hasEnc  = true;
        if (strcmp(tokens[i], "pose") == 0) hasPose = true;
        if (strcmp(tokens[i], "T")    == 0) hasT    = true;
        if (strcmp(tokens[i], "D")    == 0) hasD    = true;
    }
    if (!hasEnc && !hasPose && !hasT && !hasD) {
        r.ok = false;
        r.err = { "badarg", nullptr };
        return r;
    }
    // Pass tokens as STR args using argStr helper.
    int n = (ntokens > MAX_ARGS) ? MAX_ARGS : ntokens;
    r.ok = true;
    r.args.count = n;
    for (int i = 0; i < n; ++i)
        argStr(r.args.args[i], tokens[i]);
    return r;
}

static void handleZero(const ArgList& args, const char* corrId,
                        ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    Robot* robot = ctxFrom(handlerCtx).robot;

    bool doEnc  = false;
    bool doPose = false;
    bool doT    = false;
    bool doD    = false;
    for (int i = 0; i < args.count; ++i) {
        if (strcmp(args.args[i].sval, "enc")  == 0) doEnc  = true;
        if (strcmp(args.args[i].sval, "pose") == 0) doPose = true;
        if (strcmp(args.args[i].sval, "T")    == 0) doT    = true;
        if (strcmp(args.args[i].sval, "D")    == 0) doD    = true;
    }
    // ZERO enc -- atomic encoder reset: hardware accumulators, MC velocity
    // baselines, outlier-filter baseline, and Odometry encoder snapshot.
    // (N1 fix, sprint 030-001: replaces bare resetEncoderAccumulators() which
    // left state.actual.encLMm/R stale, freezing encoder reads for ~target mm.)
    if (doEnc)  robot->resetEncoders();
    if (doPose) robot->estimate.zero(robot->state.actual);
    // ZERO T -- set timer baseline for HaltController TIME conditions.
    if (doT) {
        robot->haltController.setTimerBaseline(robot->systemTime());
    }
    // ZERO D -- set distance baseline for HaltController DISTANCE conditions.
    if (doD) {
        float enc_avg = (robot->state.actual.encMm[1] + robot->state.actual.encMm[0]) * 0.5f;
        robot->haltController.setDistBaseline(enc_avg);
    }

    // Build response body listing what was zeroed.
    char rbuf[64];
    char body[32];
    int  bpos = 0;
    int  brem = (int)sizeof(body);
    auto append = [&](const char* tok) {
        int n = snprintf(body + bpos, (size_t)brem, "%s%s",
                         bpos > 0 ? " " : "", tok);
        if (n > 0 && n < brem) { bpos += n; brem -= n; }
    };
    if (doEnc)  append("enc");
    if (doPose) append("pose");
    if (doT)    append("T");
    if (doD)    append("D");
    body[bpos] = '\0';
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "zero", body, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// parseStream -- custom parseFn for STREAM (068-001).
//
// STREAM used to be registered with a plain variadic ArgSchema, which copies
// tokens[] into args[].sval verbatim. That is broken for "STREAM fields=...":
// CommandProcessor::parseKV() runs BEFORE dispatch and rewrites any
// "key=value" token IN PLACE, truncating it at the '=' (tokens[i] becomes
// just "fields"; the value only survives separately in kvs[i].value). A
// plain variadic schema therefore never sees the reconstructed
// "fields=<csv>" string handleStream's own "fields=" prefix scan expects --
// this went undetected because every prior test exercised handleStream with
// a hand-built ArgList, never through the real tokenize+parseKV+dispatch
// pipeline. Fix: reconstruct "fields=<value>" from kvs[] here, mirroring the
// parseSet idiom already used for SET in ConfigCommands.cpp. Falls back to
// the original variadic-token behaviour (positional period arg) when no
// "fields" kv is present.
// ---------------------------------------------------------------------------
static ParseResult parseStream(const char* const* tokens, int ntokens,
                               const KVPair* kvs, int nkv)
{
    ParseResult r;
    r.ok = true;

    const KVPair* fieldsKv = kvFind(kvs, nkv, "fields");
    if (fieldsKv != nullptr) {
        char body[64];
        snprintf(body, sizeof(body), "fields=%s", fieldsKv->value ? fieldsKv->value : "");
        argStr(r.args.args[0], body);
        r.args.count = 1;
        r.args.suppliedCount = 1;
        return r;
    }

    // No fields= kv -- replicate the original variadic-schema behaviour
    // (each token copied verbatim as a STR arg; used for "STREAM <ms>").
    int n = (ntokens < MAX_ARGS) ? ntokens : MAX_ARGS;
    r.args.count = n;
    r.args.suppliedCount = n;
    for (int i = 0; i < n; ++i) {
        argStr(r.args.args[i], tokens[i]);
    }
    return r;
}

// ---------------------------------------------------------------------------
// STREAM -- configure telemetry stream period and/or field mask.
//   prefix "STREAM"; custom parseFn (parseStream); passes period int or
//   fields= string.
//   Reply: OK stream period=<ms> | OK stream fields=<csv>
// ---------------------------------------------------------------------------

static void handleStream(const ArgList& args, const char* corrId,
                          ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    Robot* robot = ctxFrom(handlerCtx).robot;
    char rbuf[520];

    // Scan args for a "fields=..." entry.
    for (int i = 0; i < args.count; ++i) {
        const char* sv = args.args[i].sval;
        if (strncmp(sv, "fields=", 7) == 0) {
            const char* fp = sv + 7;
            uint16_t mask = 0;
            char fbuf[64];
            int flen = 0;
            for (const char* c = fp; ; ++c) {
                bool end = (*c == '\0' || *c == ',');
                if (!end && flen < (int)(sizeof(fbuf) - 1))
                    fbuf[flen++] = *c;
                if (end) {
                    fbuf[flen] = '\0';
                    if (strcmp(fbuf, "enc")     == 0) mask |= TLM_FIELD_ENC;
                    if (strcmp(fbuf, "pose")    == 0) mask |= TLM_FIELD_POSE;
                    if (strcmp(fbuf, "vel")     == 0) mask |= TLM_FIELD_VEL;
                    if (strcmp(fbuf, "line")    == 0) mask |= TLM_FIELD_LINE;
                    if (strcmp(fbuf, "color")   == 0) mask |= TLM_FIELD_COLOR;
                    if (strcmp(fbuf, "twist")   == 0) mask |= TLM_FIELD_TWIST;
                    if (strcmp(fbuf, "otos")    == 0) mask |= TLM_FIELD_OTOS;
                    if (strcmp(fbuf, "ekf_rej") == 0) mask |= TLM_FIELD_EKFREJ;
                    if (strcmp(fbuf, "encpose") == 0) mask |= TLM_FIELD_ENCPOSE;
                    flen = 0;
                    if (*c == '\0') break;
                }
            }
            robot->config.tlmFields = mask ? mask : TLM_FIELD_ALL;

            // Reconstruct the fields string for the response body.
            char body[80];
            int bpos = 0;
            bool needComma = false;
            const struct { uint16_t bit; const char* name; } kFieldNames[] = {
                { TLM_FIELD_ENC,     "enc"     },
                { TLM_FIELD_POSE,    "pose"    },
                { TLM_FIELD_VEL,     "vel"     },
                { TLM_FIELD_LINE,    "line"    },
                { TLM_FIELD_COLOR,   "color"   },
                { TLM_FIELD_TWIST,   "twist"   },
                { TLM_FIELD_OTOS,    "otos"    },
                { TLM_FIELD_EKFREJ,  "ekf_rej" },
                { TLM_FIELD_ENCPOSE, "encpose" },
            };
            int brem = (int)sizeof(body);
            int bw = snprintf(body + bpos, (size_t)brem, "fields=");
            if (bw > 0 && bw < brem) { bpos += bw; brem -= bw; }
            for (int fi = 0; fi < 9 && brem > 1; ++fi) {
                if (robot->config.tlmFields & kFieldNames[fi].bit) {
                    if (needComma) { body[bpos++] = ','; --brem; }
                    bw = snprintf(body + bpos, (size_t)brem, "%s", kFieldNames[fi].name);
                    if (bw > 0 && bw < brem) { bpos += bw; brem -= bw; }
                    needComma = true;
                }
            }
            body[bpos] = '\0';
            CommandProcessor::replyOK(rbuf, sizeof(rbuf), "stream", body,
                                      corrId, replyFn, replyCtx);
            return;
        }
    }

    // No fields= -- expect a positional period arg.
    if (args.count < 1) {
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "badarg", "usage: STREAM <ms>",
                                   corrId, replyFn, replyCtx);
        return;
    }
    int32_t ms = (int32_t)atoi(args.args[0].sval);
    if (ms < 0) ms = 0;
    if (ms > 0 && ms < 20) ms = 20;  // clamp to 50 Hz max (D10 028-005: enforced here, NOT in telemetryEmit)
    robot->config.tlmPeriodMs = ms;

    // D10 channel binding (028-005): bind the TLM stream to the channel that
    // issued this STREAM command.  runCommsIn uses _tlmBoundCtx to identify
    // the channel and derive the TLM-appropriate reply fn (serialReplyTlm
    // for serial, radioReply for radio).  Commands on other channels do not
    // redirect the stream.
    //
    // N3 fix (030-003): also store the caller's replyFn as _tlmBoundFn so that
    // telemetryEmit (now using _tlmBoundFn/_tlmBoundCtx directly) has a valid fn
    // in both the sim path (replyFn = storeReply) and firmware path.  In firmware,
    // runCommsIn overwrites _tlmBoundFn on the next iteration with the correct
    // channel fn (serialReplyTlm or radioReply derived from _tlmBoundCtx), so
    // _tlmBoundFn is always the pair that matches _tlmBoundCtx.
    robot->_tlmBoundFn  = replyFn;
    robot->_tlmBoundCtx = replyCtx;

    char body[32];
    snprintf(body, sizeof(body), "period=%d", (int)ms);
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "stream", body, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// BAUD <rate> -- change the USB serial baud rate at runtime.
//   Boots at 115200; "BAUD <rate>" replies "OK baud <rate>" at the OLD baud,
//   then retunes the UART. Supported: 115200, 230400, 921600, 1000000.
//   The HOST must then change its own baud on the OPEN port (do NOT reopen —
//   reopening pulses DTR and resets the robot back to 115200). "BAUD" with no
//   arg queries the supported set. Reply: OK baud <rate>.
// ---------------------------------------------------------------------------

static ParseResult parseBaud(const char* const* tokens, int ntokens,
                              const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult r;
    r.ok = true;
    if (ntokens >= 1) {
        // Store as STR (sval), not ival: Argument's ival/fval share a union, and
        // the common "ival=X; fval=0.0f;" idiom would zero a >16-bit rate. The
        // handler atoi()s sval, which is outside the union.
        r.args.count = 1;
        argStr(r.args.args[0], tokens[0]);
    } else {
        r.args.count = 0;
    }
    return r;
}

static void handleBaud(const ArgList& args, const char* corrId,
                        ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    LoopScheduler* sched = ctxFrom(handlerCtx).sched;
    char rbuf[64];
    if (sched == nullptr) {
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "noserial", nullptr,
                                   corrId, replyFn, replyCtx);
        return;
    }
#ifndef HOST_BUILD
    if (args.count < 1) {
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "baud",
                                  "115200|230400|921600|1000000",
                                  corrId, replyFn, replyCtx);
        return;
    }
    uint32_t rate = (uint32_t)atoi(args.args[0].sval);
    if (rate != 115200 && rate != 230400 && rate != 921600 && rate != 1000000) {
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "range", "baud",
                                   corrId, replyFn, replyCtx);
        return;
    }
    // Reply on the OLD baud FIRST, then retune (setBaud drains TX before switch).
    char body[24];
    snprintf(body, sizeof(body), "%lu", (unsigned long)rate);
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "baud", body,
                              corrId, replyFn, replyCtx);
    sched->comm().serial().setBaud(rate);
#else
    (void)args;
    CommandProcessor::replyErr(rbuf, sizeof(rbuf), "noserial", nullptr,
                               corrId, replyFn, replyCtx);
#endif
}

static void handleRf(const ArgList& args, const char* corrId,
                      ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    LoopScheduler* sched = ctxFrom(handlerCtx).sched;
    char rbuf[64];
    if (sched == nullptr) {
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "noradio", nullptr,
                                   corrId, replyFn, replyCtx);
        return;
    }
#ifndef HOST_BUILD
    Radio& radio = sched->comm().radio();

    if (args.suppliedCount < 1) {
        // Query.
        char body[32];
        snprintf(body, sizeof(body), "chan=%d group=%d",
                 radio.channel(), radiochan::kGroup);
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "rf", body,
                                  corrId, replyFn, replyCtx);
        return;
    }

    int ch = args.args[0].ival;
    if (ch < radiochan::kMin || ch > radiochan::kMax) {
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "range", "chan",
                                   corrId, replyFn, replyCtx);
        return;
    }
    // Persist first, then reply on the OLD channel, then re-tune.
    radiochan::save(sched->uBit().storage, ch);
    char body[32];
    snprintf(body, sizeof(body), "chan=%d group=%d", ch, radiochan::kGroup);
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "rf", body,
                              corrId, replyFn, replyCtx);
    radio.setChannel(ch);
#else
    (void)args;
#endif
}

// ---------------------------------------------------------------------------
// GET VEL / GET / SET -- config-registry commands.
//
// Moved to source/commands/ConfigCommands.cpp (finding A3 split).  The parse*/handle*
// functions live there as file-local statics; their descriptors are registered
// onto the command table below via appendConfigCommands().
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// + -- keepalive command.
//   prefix "+"; parseFn nullptr (no args).
//   Resets the system watchdog timestamp.
//   Reply: OK keepalive
// ---------------------------------------------------------------------------

static void handleKeepalive(const ArgList& /*args*/, const char* /*corrId*/,
                              ReplyFn /*replyFn*/, void* /*replyCtx*/, void* handlerCtx)
{
    // Quiet keepalive (sprint 024-003): suppress the "OK keepalive" reply.
    // At 6.7 Hz the acks competed with TLM frames for the 250-byte TX buffer;
    // the host already filters them.  The watchdog reset (firmware side) and
    // the sim watchdog arm (sim_api.cpp via sim_command) are the only effects.
#ifndef HOST_BUILD
    LoopScheduler* sched = ctxFrom(handlerCtx).sched;
    Robot*         robot = ctxFrom(handlerCtx).robot;
    if (sched != nullptr) {
        sched->resetWatchdog(robot->systemTime());
    }
#else
    (void)handlerCtx;
#endif
    // No reply emitted (quiet keepalive).
}

// ---------------------------------------------------------------------------
// SAFE -- enable/disable the system safety-stop watchdog and set its timeout.
//   SAFE                 -> query: "OK safety on|off timeout=<ms>"
//   SAFE off  (or SAFE 0)-> disable the watchdog (no keepalives required)
//   SAFE on   [<ms>]     -> enable; optional <ms> sets sTimeoutMs
//   SAFE <ms>            -> <ms> > 0: enable + set timeout; 0: disable
// variadic ArgSchema: tokens passed through as STR args.
// ---------------------------------------------------------------------------

static void handleSafe(const ArgList& args, const char* corrId,
                        ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    char rbuf[80];
    Robot* robot = ctxFrom(handlerCtx).robot;
    if (robot == nullptr) {
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "noctx", "SAFE",
                                   corrId, replyFn, replyCtx);
        return;
    }
    RobotConfig& cfg = robot->config;

    if (args.count >= 1) {
        const char* a0 = args.args[0].sval;
        if (strcmp(a0, "off") == 0) {
            // One-shot disable: do NOT clear safetyEnabled directly.
            // Instead arm the one-shot flag in Planner so safety
            // is automatically restored when the next motion command begins.
            // This prevents SAFE off from becoming a permanent foot-gun.
            robot->planner.disableSafetyOneShot();
            // Reflect the transient "off" state in the reply (safetyEnabled
            // will be re-armed by Planner on the next begin*() call,
            // but for the duration of any current-or-next command the watchdog
            // is suppressed via _safeOneShotDisable).
            cfg.safetyEnabled = false;
        } else if (strcmp(a0, "on") == 0) {
            cfg.safetyEnabled = true;
            if (args.count >= 2) {
                int ms = atoi(args.args[1].sval);
                if (ms > 0) cfg.sTimeoutMs = ms;
            }
        } else {
            // Numeric form: SAFE <ms>  (0 -> off, >0 -> on with that timeout).
            int ms = atoi(a0);
            if (ms <= 0) {
                // Same one-shot treatment as "SAFE off".
                robot->planner.disableSafetyOneShot();
                cfg.safetyEnabled = false;
            } else {
                cfg.safetyEnabled = true;
                cfg.sTimeoutMs = ms;
            }
        }
    }

    char body[48];
    snprintf(body, sizeof(body), "%s timeout=%d",
             cfg.safetyEnabled ? "on" : "off", (int)cfg.sTimeoutMs);
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "safety", body,
                              corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// SI -- set the odometry world pose directly (what G reads via getPoseFloat).
//   SI <x_mm> <y_mm> <h_cdeg>
// Establishes the robot's onboard pose from an external fix (e.g. the camera)
// so a subsequent G/D/TURN drives in the correct world frame. This is the pose
// the motion controller reads -- unlike OV, which only nudges the raw OTOS chip.
// Reply: OK setpose x=<mm> y=<mm> h=<cdeg>
// siSchema: 3 mandatory INTs, ranged=false (plain atoi, no range check).
// ---------------------------------------------------------------------------

static void handleSI(const ArgList& args, const char* corrId,
                      ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    char rbuf[80];
    Robot* robot = ctxFrom(handlerCtx).robot;
    if (robot == nullptr) {
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "noctx", "SI",
                                   corrId, replyFn, replyCtx);
        return;
    }
    int32_t x_mm   = args.args[0].ival;
    int32_t y_mm   = args.args[1].ival;
    int32_t h_cdeg = args.args[2].ival;
    // Direct path (legacy + live loop): reset the shared estimate and state.actual.
    robot->estimate.resetPose(robot->state.actual, x_mm, y_mm, h_cdeg);
    // 059-004: also stage via drive.apply(SetPose) so the new-arch Drive subsystem
    // sees the same pose re-anchor.  Drive::tickAction processes the staged command
    // and calls _est.resetPose on its own private _hw estimate.  Both the legacy path
    // (robot->estimate above) and the new-arch path (drive) are consistent.
    // h_cdeg → rad: pi/18000 = 1.74532925e-4.
    {
        msg::DrivetrainCommand siCmd;
        msg::SetPose sp{};
        sp.x  = (float)x_mm;
        sp.y  = (float)y_mm;
        sp.h  = (float)h_cdeg * 1.74532925e-4f;
        siCmd.setPose(sp);
        robot->drive.apply(siCmd);
    }
    // Re-anchor the OTOS to the SAME world fix so its absolute position+heading
    // observations agree with the controller pose, instead of dragging the EKF
    // back toward the OTOS boot frame (the "starts right, then rotates away"
    // bug).  h_cdeg -> rad: pi/18000 = 1.74532925e-4.
    robot->otos.setWorldPose((float)x_mm, (float)y_mm,
                             (float)h_cdeg * 1.74532925e-4f);
    char body[48];
    snprintf(body, sizeof(body), "x=%d y=%d h=%d", (int)x_mm, (int)y_mm, (int)h_cdeg);
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "setpose", body,
                              corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// HALT -- user-facing named stop-condition commands.
//
// Wire formats:
//   HALT TIME <ms>          -> OK HALT id=<n>
//   HALT TIME <ms> SOFT     -> OK HALT id=<n>
//   HALT DIST <mm>          -> OK HALT id=<n>
//   HALT DIST <mm> SOFT     -> OK HALT id=<n>
//   HALT LINE ANY <GE|LE> <threshold>       -> OK HALT id=<n>
//   HALT LINE ANY <GE|LE> <threshold> SOFT  -> OK HALT id=<n>
//   HALT CLEAR              -> OK HALT cleared=<count>
//   HALT LIST               -> one "OK HALT id=<n> str=..." line per entry + OK HALT list
//
// parseFn: passes tokens as STR args (first arg is the sub-verb: TIME, DIST,
// LINE, CLEAR, LIST). Handler dispatches on args[0].sval.
// ---------------------------------------------------------------------------

static ParseResult parseHalt(const char* const* tokens, int ntokens,
                              const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult r;
    if (ntokens < 1) {
        r.ok = false;
        r.err = { "badarg", "usage: HALT TIME|DIST|POS|COLOR|LINE|CLEAR|INFO|LIST ..." };
        return r;
    }
    // Validate sub-verb.
    const char* sv = tokens[0];
    if (strcmp(sv, "TIME")  != 0 && strcmp(sv, "DIST")  != 0 &&
        strcmp(sv, "LINE")  != 0 && strcmp(sv, "CLEAR") != 0 &&
        strcmp(sv, "LIST")  != 0 && strcmp(sv, "POS")   != 0 &&
        strcmp(sv, "COLOR") != 0 && strcmp(sv, "INFO")  != 0) {
        r.ok = false;
        r.err = { "badarg", "usage: HALT TIME|DIST|POS|COLOR|LINE|CLEAR|INFO|LIST ..." };
        return r;
    }
    // Pass all tokens as STR args using argStr helper.
    int n = (ntokens > MAX_ARGS) ? MAX_ARGS : ntokens;
    r.ok = true;
    r.args.count = n;
    for (int i = 0; i < n; ++i)
        argStr(r.args.args[i], tokens[i]);
    return r;
}

static void handleHalt(const ArgList& args, const char* corrId,
                        ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    Robot* robot = ctxFrom(handlerCtx).robot;
    char rbuf[128];

    if (args.count < 1) {
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "badarg",
                                   "usage: HALT TIME|DIST|LINE|CLEAR|LIST ...",
                                   corrId, replyFn, replyCtx);
        return;
    }

    const char* sv = args.args[0].sval;

    // ---- CLEAR ----
    if (strcmp(sv, "CLEAR") == 0) {
        if (args.count >= 2) {
            // HALT CLEAR <id> -- remove one entry by id.
            uint8_t rmid = (uint8_t)atoi(args.args[1].sval);
            bool removed = robot->haltController.remove(rmid);
            if (!removed) {
                CommandProcessor::replyErr(rbuf, sizeof(rbuf), "notfound", "id",
                                           corrId, replyFn, replyCtx);
                return;
            }
            char body[32];
            snprintf(body, sizeof(body), "cleared id=%u", (unsigned)rmid);
            CommandProcessor::replyOK(rbuf, sizeof(rbuf), "HALT", body,
                                       corrId, replyFn, replyCtx);
        } else {
            // HALT CLEAR -- remove all entries.
            int n = robot->haltController.clear();
            char body[32];
            snprintf(body, sizeof(body), "cleared=%d", n);
            CommandProcessor::replyOK(rbuf, sizeof(rbuf), "HALT", body,
                                       corrId, replyFn, replyCtx);
        }
        return;
    }

    // ---- LIST ----
    if (strcmp(sv, "LIST") == 0) {
        robot->haltController.list(replyFn, replyCtx);
        char body[32];
        snprintf(body, sizeof(body), "list count=%d",
                 robot->haltController.count());
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "HALT", body,
                                   corrId, replyFn, replyCtx);
        return;
    }

    // ---- TIME ----
    if (strcmp(sv, "TIME") == 0) {
        if (args.count < 2) {
            CommandProcessor::replyErr(rbuf, sizeof(rbuf), "badarg",
                                       "usage: HALT TIME <ms> [SOFT]",
                                       corrId, replyFn, replyCtx);
            return;
        }
        float ms = (float)atof(args.args[1].sval);
        StopStyle style = StopStyle::HARD;
        if (args.count >= 3 && strcmp(args.args[2].sval, "SOFT") == 0)
            style = StopStyle::SOFT;

        StopCondition cond = makeTimeStop(ms);
        // Build a label string for HALT LIST.
        char label[40];
        snprintf(label, sizeof(label), "TIME %g%s", ms,
                 style == StopStyle::SOFT ? " SOFT" : "");
        // Capture registration-time baseline so the condition fires ~ms after
        // now, not ~ms after boot (N10 fix).
        uint32_t now_ms   = robot->systemTime();
        float    enc_avg  = (robot->state.actual.encMm[1] + robot->state.actual.encMm[0]) * 0.5f;
        int id = robot->haltController.add(cond, style, label, now_ms, enc_avg);
        if (id < 0) {
            CommandProcessor::replyErr(rbuf, sizeof(rbuf), "full",
                                       "halt table full (max 8)",
                                       corrId, replyFn, replyCtx);
            return;
        }
        char body[32];
        snprintf(body, sizeof(body), "id=%d", id);
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "HALT", body,
                                   corrId, replyFn, replyCtx);
        return;
    }

    // ---- DIST ----
    if (strcmp(sv, "DIST") == 0) {
        if (args.count < 2) {
            CommandProcessor::replyErr(rbuf, sizeof(rbuf), "badarg",
                                       "usage: HALT DIST <mm> [SOFT]",
                                       corrId, replyFn, replyCtx);
            return;
        }
        float mm = (float)atof(args.args[1].sval);
        StopStyle style = StopStyle::HARD;
        if (args.count >= 3 && strcmp(args.args[2].sval, "SOFT") == 0)
            style = StopStyle::SOFT;

        StopCondition cond = makeDistanceStop(mm);
        char label[40];
        snprintf(label, sizeof(label), "DIST %g%s", mm,
                 style == StopStyle::SOFT ? " SOFT" : "");
        // Capture registration-time baseline so the condition fires ~mm after
        // the current encoder position, not from boot (N10 fix).
        uint32_t now_ms_d  = robot->systemTime();
        float    enc_avg_d = (robot->state.actual.encMm[1] + robot->state.actual.encMm[0]) * 0.5f;
        int id = robot->haltController.add(cond, style, label, now_ms_d, enc_avg_d);
        if (id < 0) {
            CommandProcessor::replyErr(rbuf, sizeof(rbuf), "full",
                                       "halt table full (max 8)",
                                       corrId, replyFn, replyCtx);
            return;
        }
        char body[32];
        snprintf(body, sizeof(body), "id=%d", id);
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "HALT", body,
                                   corrId, replyFn, replyCtx);
        return;
    }

    // ---- LINE ANY ----
    // Wire: HALT LINE ANY <GE|LE> <threshold> [SOFT]
    if (strcmp(sv, "LINE") == 0) {
        // args: [0]=LINE [1]=ANY [2]=GE|LE [3]=threshold [4]=SOFT?
        if (args.count < 4 ||
            strcmp(args.args[1].sval, "ANY") != 0) {
            CommandProcessor::replyErr(rbuf, sizeof(rbuf), "badarg",
                                       "usage: HALT LINE ANY GE|LE <threshold> [SOFT]",
                                       corrId, replyFn, replyCtx);
            return;
        }
        const char* opStr = args.args[2].sval;
        StopCondition::Cmp op;
        if (strcmp(opStr, "GE") == 0) {
            op = StopCondition::Cmp::GE;
        } else if (strcmp(opStr, "LE") == 0) {
            op = StopCondition::Cmp::LE;
        } else {
            CommandProcessor::replyErr(rbuf, sizeof(rbuf), "badarg",
                                       "op must be GE or LE",
                                       corrId, replyFn, replyCtx);
            return;
        }
        float threshold = (float)atof(args.args[3].sval);
        StopStyle style = StopStyle::HARD;
        if (args.count >= 5 && strcmp(args.args[4].sval, "SOFT") == 0)
            style = StopStyle::SOFT;

        StopCondition cond = makeLineAnyStop(threshold, op);
        // Build label: "LINE ANY GE <thr>" or "LINE ANY LE <thr> SOFT".
        // Use fixed 2-char op abbreviation and integer threshold to keep
        // label within StopEntry.str[40] and silence -Wformat-truncation.
        char label[40];
        {
            const char* opAbbrev = (op == StopCondition::Cmp::GE) ? "GE" : "LE";
            const char* softSfx  = (style == StopStyle::SOFT) ? " SOFT" : "";
            // "LINE ANY GE 65535 SOFT" = 22 chars -- fits comfortably.
            snprintf(label, sizeof(label), "LINE ANY %.2s %d%s",
                     opAbbrev, (int)threshold, softSfx);
        }
        int id = robot->haltController.add(cond, style, label,
                                            robot->systemTime(),
                                            (robot->state.actual.encMm[1] + robot->state.actual.encMm[0]) * 0.5f);
        if (id < 0) {
            CommandProcessor::replyErr(rbuf, sizeof(rbuf), "full",
                                       "halt table full (max 8)",
                                       corrId, replyFn, replyCtx);
            return;
        }
        char body[32];
        snprintf(body, sizeof(body), "id=%d", id);
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "HALT", body,
                                   corrId, replyFn, replyCtx);
        return;
    }

    // ---- POS ----
    if (strcmp(sv, "POS") == 0) {
        // Wire: HALT POS <x_mm> <y_mm> <radius_mm>
        if (args.count < 4) {
            CommandProcessor::replyErr(rbuf, sizeof(rbuf), "badarg",
                                       "usage: HALT POS <x_mm> <y_mm> <radius_mm>",
                                       corrId, replyFn, replyCtx);
            return;
        }
        float x   = (float)atof(args.args[1].sval);
        float y   = (float)atof(args.args[2].sval);
        float rad = (float)atof(args.args[3].sval);
        StopStyle style = StopStyle::HARD;
        if (args.count >= 5 && strcmp(args.args[4].sval, "SOFT") == 0)
            style = StopStyle::SOFT;

        StopCondition cond = makePositionStop(x, y, rad);
        char label[40];
        // Use integer mm to keep label well within StopEntry.str[40].
        // "POS -32000 -32000 32000" = 22 chars -- fits comfortably.
        snprintf(label, sizeof(label), "POS %d %d %d",
                 (int)x, (int)y, (int)rad);
        int id = robot->haltController.add(cond, style, label,
                                            robot->systemTime(),
                                            (robot->state.actual.encMm[1] + robot->state.actual.encMm[0]) * 0.5f);
        if (id < 0) {
            CommandProcessor::replyErr(rbuf, sizeof(rbuf), "full",
                                       "halt table full (max 8)",
                                       corrId, replyFn, replyCtx);
            return;
        }
        char body[32];
        snprintf(body, sizeof(body), "id=%d", id);
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "HALT", body,
                                   corrId, replyFn, replyCtx);
        return;
    }

    // ---- COLOR ----
    if (strcmp(sv, "COLOR") == 0) {
        // Wire: HALT COLOR <h> <s> <v> <dist>
        if (args.count < 5) {
            CommandProcessor::replyErr(rbuf, sizeof(rbuf), "badarg",
                                       "usage: HALT COLOR <h> <s> <v> <dist>",
                                       corrId, replyFn, replyCtx);
            return;
        }
        float h    = (float)atof(args.args[1].sval);
        float s    = (float)atof(args.args[2].sval);
        float v    = (float)atof(args.args[3].sval);
        float dist = (float)atof(args.args[4].sval);
        StopStyle style = StopStyle::HARD;
        if (args.count >= 6 && strcmp(args.args[5].sval, "SOFT") == 0)
            style = StopStyle::SOFT;

        StopCondition cond = makeColorStop(h, s, v, dist);
        char label[40];
        // Format as fixed 2-decimal for HSV floats; keep within StopEntry.str[40].
        // "COLOR 360.00 1.00 1.00 1.00" = 28 chars -- fits comfortably.
        snprintf(label, sizeof(label), "COLOR %.2f %.2f %.2f %.2f",
                 (double)h, (double)s, (double)v, (double)dist);
        int id = robot->haltController.add(cond, style, label,
                                            robot->systemTime(),
                                            (robot->state.actual.encMm[1] + robot->state.actual.encMm[0]) * 0.5f);
        if (id < 0) {
            CommandProcessor::replyErr(rbuf, sizeof(rbuf), "full",
                                       "halt table full (max 8)",
                                       corrId, replyFn, replyCtx);
            return;
        }
        char body[32];
        snprintf(body, sizeof(body), "id=%d", id);
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "HALT", body,
                                   corrId, replyFn, replyCtx);
        return;
    }

    // ---- INFO ----
    if (strcmp(sv, "INFO") == 0) {
        // Wire: HALT INFO <id>
        if (args.count < 2) {
            CommandProcessor::replyErr(rbuf, sizeof(rbuf), "badarg",
                                       "usage: HALT INFO <id>",
                                       corrId, replyFn, replyCtx);
            return;
        }
        uint8_t qid = (uint8_t)atoi(args.args[1].sval);
        char infoBuf[80];
        if (!robot->haltController.info(qid, infoBuf, sizeof(infoBuf))) {
            CommandProcessor::replyErr(rbuf, sizeof(rbuf), "notfound", "id",
                                       corrId, replyFn, replyCtx);
            return;
        }
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "HALT", infoBuf,
                                   corrId, replyFn, replyCtx);
        return;
    }

    // Unknown sub-verb (should not reach here after parseHalt validation).
    CommandProcessor::replyErr(rbuf, sizeof(rbuf), "badarg",
                               "usage: HALT TIME|DIST|POS|COLOR|LINE|CLEAR|INFO|LIST ...",
                               corrId, replyFn, replyCtx);
}

}  // anonymous namespace

// ---------------------------------------------------------------------------
// Robot::buildCommandTable -- aggregate all Commandables + system commands.
// ---------------------------------------------------------------------------

std::vector<CommandDescriptor> Robot::buildCommandTable(
    DebugCommands* dbg, LoopScheduler* sched, SimCommands* sim) const
{
#ifndef HOST_BUILD
    // sim is always nullptr on the ARM target (buildCommandTable's default
    // argument, never overridden by main.cpp) -- silence the unused-parameter
    // warning without referencing the (here-incomplete) SimCommands type.
    (void)sim;
#endif

    // Populate stable context structs (members, so pointers are valid for the
    // lifetime of this Robot).
    // 059-004: wire subsystem pointers so handleSet can route annotated fields
    // to drive.configure() / planner.configure() / sensors.configure().
    _cfgCtx.cfg     = const_cast<RobotConfig*>(&config);
    _cfgCtx.mc      = const_cast<MotorController*>(&motorController);
    _cfgCtx.drive   = const_cast<subsystems::Drive*>(&drive);
    _cfgCtx.planner = const_cast<Planner*>(&planner);
    _cfgCtx.sensors = const_cast<subsystems::Sensors*>(&sensors);
    _sysCtx.robot = const_cast<Robot*>(this);
    _sysCtx.sched = sched;
    // Initialise _motionCtx for this build (sprint 026-002).
    // mc and robot pointers are already set in the constructor; vwDesc is
    // initialised by getMotionCommands() below.
    _motionCtx.mc    = const_cast<Planner*>(&planner);
    _motionCtx.robot = const_cast<Robot*>(this);
    // queue is set by setMotionQueue() from LoopScheduler; preserve it here.

    void* sysCtxPtr = &_sysCtx;

    std::vector<CommandDescriptor> cmds;

    // ---- Commandable members ----
    auto append = [&](std::vector<CommandDescriptor> v) {
        cmds.insert(cmds.end(), v.begin(), v.end());
    };
    append(getMotionCommands(&_motionCtx));
    // 041-002: the seven OTOS-tuning verbs moved out of Odometry (Commandable
    // stripped) into the app-layer OtosCommands.  Aggregate them here in place
    // of the old odometry.getCommands() so dispatch is unchanged.
    append(_otosCommands.getCommands());
    append(portController.getCommands());
    append(servoController.getCommands());
    if (dbg) append(dbg->getCommands());
    // SIMSET/SIMGET (069-003) -- sim-build-only; see the HOST_BUILD-guarded
    // #include above.  On the ARM build this whole statement is skipped by
    // the preprocessor (not merely never entered at runtime), so the ARM
    // compilation of this TU never requires SimCommands to be a complete type.
#ifdef HOST_BUILD
    if (sim) append(sim->getCommands());
#endif

    // ---- System commands ----
    // No-arg commands: parseFn=nullptr — framework passes empty ArgList.
    cmds.push_back(makeCmd("HELLO",  nullptr, handleHello,     sysCtxPtr, "badarg")); // identify firmware + version
    cmds.push_back(makeCmd("PING",   nullptr, handlePing,      sysCtxPtr, "badarg")); // liveness check
    cmds.push_back(makeCmd("ID",     nullptr, handleId,        sysCtxPtr, "badarg")); // report robot identity string
    cmds.push_back(makeCmd("VER",    nullptr, handleVer,       sysCtxPtr, "badarg")); // report firmware version
    cmds.push_back(makeCmd("HELP",   nullptr, handleHelp,      sysCtxPtr, "badarg")); // list available commands
    cmds.push_back(makeCmd("SNAP",   nullptr, handleSnap,      sysCtxPtr, "badarg")); // emit one TLM frame on demand
    cmds.push_back(makeCmd("+",      nullptr, handleKeepalive, sysCtxPtr, "badarg", ForceReply::NONE, CMD_MOTION_WATCHDOG)); // keepalive: reset watchdog
    // Custom-parseFn commands: ZERO/HALT retain sub-verb dispatch; BAUD uses sval for large int.
    cmds.push_back(makeCmd("ZERO",   parseZero, handleZero,   sysCtxPtr, "badarg")); // zero encoders/pose/halt-baselines
    cmds.push_back(makeCmd("HALT",   parseHalt, handleHalt,   sysCtxPtr, "badarg")); // named stop-condition registry
    cmds.push_back(makeCmd("BAUD",   parseBaud, handleBaud,   sysCtxPtr, "badarg")); // set USB serial baud rate
    // Schema-driven commands: variadic STR for ECHO/STREAM/SAFE; positional INT for SI/RF.
    cmds.push_back(makeSchemaCmd("ECHO",   &echoSchema,   handleEcho,   sysCtxPtr, "badarg")); // echo tokens back
    cmds.push_back(makeCmd("STREAM", parseStream, handleStream, sysCtxPtr, "badarg")); // start/stop periodic TLM stream
    cmds.push_back(makeSchemaCmd("SAFE",   &safeSchema,   handleSafe,   sysCtxPtr, "badarg")); // enable/disable safety watchdog + set timeout
    cmds.push_back(makeSchemaCmd("SI",     &siSchema,     handleSI,     sysCtxPtr, "badarg")); // set odometry world pose (x_mm y_mm h_cdeg)
    cmds.push_back(makeSchemaCmd("RF",     &rfSchema,     handleRf,     sysCtxPtr, "badarg")); // set radio channel
    // GET VEL / GET / SET descriptors live in ConfigCommands.cpp (A3 split).
    // GET VEL is registered first so its longer prefix wins the linear scan.
    appendConfigCommands(cmds, &_cfgCtx, sysCtxPtr);

    return cmds;
}
