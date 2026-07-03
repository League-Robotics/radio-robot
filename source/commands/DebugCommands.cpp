// DebugCommands.cpp — Commandable for all diagnostic commands.
//
// Owns: DBG LOOP RESET, DBG LOOP, DBG I2CLOG, DBG I2C, DBG IRQGUARD,
//       DBG WEDGE, DBG OTOS BENCH, DBG OTOS, DBG EST, I2CW, I2CR.
//
// All descriptors use ForceReply::SERIAL.
// Handler logic mirrors the existing switch cases in CommandProcessor.cpp
// exactly.  The old switch cases remain live until T011 cutover.
//
// Migration (051-008): bespoke parse functions replaced with ArgSchema /
// nullptr registrations.  parseDbgLoopReset, parseDbgOtos, parseDbgEst
// deleted (no-arg → nullptr).  parseDbgLoop, parseDbgI2clog, parseDbgI2c
// deleted (variadic STR → makeSchemaCmd).  parseDbgIrqguard deleted
// (ndefs=1, minTokens=0 → makeSchemaCmd).  parseDbgWedge, parseDbgOtosBench,
// parseI2cw, parseI2cr retained (custom logic).  parseDbgOtosBench KV loop
// replaced with kvFloat helpers from ArgParse.h.

#include "DebugCommands.h"
#include "CommandProcessor.h"
#include "Robot.h"
#include "state/EstimateDump.h"
#include "ArgParse.h"
// 034-006: BenchOtosSensor is bench-build only.
#if defined(BENCH_OTOS_ENABLED) || defined(HOST_BUILD)
#include "BenchOtosSensor.h"
#endif
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>

// LoopScheduler, WedgeTest, and NezhaHAL include CODAL/MicroBit headers and
// must NOT be included in HOST_BUILD.  The handlers that use them are guarded
// with #ifndef HOST_BUILD so the file compiles in both.
//
// 044-003 (Phase F): the concrete bus header is no longer included here.  The
// DBG I2C / I2CLOG / IRQGUARD handlers reach the bus through DbgCtx::busDiag
// (IBusDiagnostics*) and the I2CW / I2CR handlers through DbgCtx::busAccess
// (IRawBusAccess*) — both capability interfaces from source/io/capability/,
// sealing the final vendor leak above source/io/.
#ifndef HOST_BUILD
#include "LoopScheduler.h"
#include "WedgeTest.h"
#include "NezhaHAL.h"
#endif

// ---------------------------------------------------------------------------
// Internal helper — cast handlerCtx to DebugCommands* and get DbgCtx.
// handlerCtx is always const_cast<DebugCommands*>(this).
// ---------------------------------------------------------------------------

// Forward declaration of accessor used by handlers (defined at bottom of file).
static DbgCtx dbgCtxFrom(void* p);

// ---------------------------------------------------------------------------
// Argument schemas — declarative replacements for bespoke parse functions.
// ---------------------------------------------------------------------------

// DBG LOOP RESET — no-arg; parseFn=nullptr.
// DBG OTOS       — no-arg; parseFn=nullptr.
// DBG EST        — no-arg; parseFn=nullptr.

// DBG LOOP — variadic: up to 2 STR tokens (x and state).
static const ArgSchema dbgLoopSchema = { nullptr, 0, 0, true, nullptr };

// DBG I2CLOG — variadic: 0 or 1 STR token ("ARM").
static const ArgSchema dbgI2clogSchema = { nullptr, 0, 0, true, nullptr };

// DBG I2C — variadic: 0 or 1 STR token ("RESET").
static const ArgSchema dbgI2cSchema = { nullptr, 0, 0, true, nullptr };

// DBG IRQGUARD — optional INT: 0 or 1 token; ndefs=1, minTokens=0, ranged=false.
static const ArgDef dbgIrqguardDefs[1] = {
    { "enable", ArgKind::INT, false, 0, 0 },
};
static const ArgSchema dbgIrqguardSchema = { dbgIrqguardDefs, 1, 0, false, nullptr };

// ---------------------------------------------------------------------------
// DBG LOOP RESET
//   prefix "DBG LOOP RESET" — parseFn=nullptr (no args).
//   handler: no-op acknowledgement (loop timing stats removed with run_tasks)
// ---------------------------------------------------------------------------

static void handleDbgLoopReset(const ArgList& /*args*/, const char* corrId,
                                ReplyFn replyFn, void* replyCtx, void* /*handlerCtx*/)
{
    char rbuf[64];
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "dbg", "loop reset",
                              corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// DBG LOOP
//   prefix "DBG LOOP" — variadic ArgSchema; tokens stored as STR args.
//   handler: confirm loop is running.
// ---------------------------------------------------------------------------

static void handleDbgLoop(const ArgList& /*args*/, const char* corrId,
                           ReplyFn replyFn, void* replyCtx, void* /*handlerCtx*/)
{
    char rbuf[64];
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "dbg", "loop running",
                              corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// DBG I2CLOG
//   prefix "DBG I2CLOG" — variadic ArgSchema; 0 or 1 STR args.
//   handler: ARM → resetStats + setLogging(true); else → dumpRecent.
// ---------------------------------------------------------------------------

static void handleDbgI2clog(const ArgList& args, const char* corrId,
                              ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
#ifndef HOST_BUILD
    DbgCtx ctx = dbgCtxFrom(handlerCtx);
    char rbuf[64];
    if (ctx.busDiag == nullptr) {
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "noimpl", "no i2c bus",
                                   corrId, replyFn, replyCtx);
        return;
    }
    if (args.count >= 1 && strcmp(args.args[0].sval, "ARM") == 0) {
        ctx.busDiag->resetStats();
        ctx.busDiag->setLogging(true);
    } else {
        ctx.busDiag->dumpRecent(replyFn, replyCtx);
    }
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "dbg", "i2clog",
                              corrId, replyFn, replyCtx);
#else
    (void)args; (void)corrId; (void)handlerCtx;
    char rbuf[64];
    CommandProcessor::replyErr(rbuf, sizeof(rbuf), "noimpl", "no i2c bus",
                               corrId, replyFn, replyCtx);
#endif
}

// ---------------------------------------------------------------------------
// DBG I2C
//   prefix "DBG I2C" — variadic ArgSchema; 0 or 1 STR args.
//   handler: RESET → resetStats + resetStuckCounters;
//            else → emit compact stats line + OK.
// ---------------------------------------------------------------------------

static void handleDbgI2c(const ArgList& args, const char* corrId,
                          ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
#ifndef HOST_BUILD
    DbgCtx ctx = dbgCtxFrom(handlerCtx);
    char rbuf[64];
    if (ctx.busDiag == nullptr) {
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "noimpl", "no i2c bus",
                                   corrId, replyFn, replyCtx);
        return;
    }
    if (args.count >= 1 && strcmp(args.args[0].sval, "RESET") == 0) {
        ctx.busDiag->resetStats();
        ctx.robot->motorController.resetStuckCounters();
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "dbg", "i2c reset",
                                  corrId, replyFn, replyCtx);
        return;
    }
    // Emit compact stats dump.
    uint32_t rV = ctx.busDiag->reentryViolations();
    uint8_t  sL = ctx.robot->motorController.stuckCountL();
    uint8_t  sR = ctx.robot->motorController.stuckCountR();
    char buf[200];
    int n = snprintf(buf, sizeof(buf),
        "I2C 0x10:txn=%lu err=%lu last=%d "
        "0x17:txn=%lu err=%lu last=%d "
        "0x1A:txn=%lu err=%lu last=%d "
        "0x43:txn=%lu err=%lu last=%d "
        "reentry=%lu stuck=L:%u,R:%u",
        (unsigned long)ctx.busDiag->txnCount(0x10),
        (unsigned long)ctx.busDiag->errCount(0x10),
        ctx.busDiag->lastErr(0x10),
        (unsigned long)ctx.busDiag->txnCount(0x17),
        (unsigned long)ctx.busDiag->errCount(0x17),
        ctx.busDiag->lastErr(0x17),
        (unsigned long)ctx.busDiag->txnCount(0x1A),
        (unsigned long)ctx.busDiag->errCount(0x1A),
        ctx.busDiag->lastErr(0x1A),
        (unsigned long)ctx.busDiag->txnCount(0x43),
        (unsigned long)ctx.busDiag->errCount(0x43),
        ctx.busDiag->lastErr(0x43),
        (unsigned long)rV,
        (unsigned)sL,
        (unsigned)sR);
    (void)n;
    replyFn(buf, replyCtx);
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "dbg", "i2c",
                              corrId, replyFn, replyCtx);
#else
    (void)args; (void)corrId; (void)handlerCtx;
    char rbuf[64];
    CommandProcessor::replyErr(rbuf, sizeof(rbuf), "noimpl", "no i2c bus",
                               corrId, replyFn, replyCtx);
#endif
}

// ---------------------------------------------------------------------------
// DBG IRQGUARD
//   prefix "DBG IRQGUARD" — dbgIrqguardSchema (ndefs=1, minTokens=0).
//   handler: if arg → setIrqGuard; always reply OK with state.
// ---------------------------------------------------------------------------

static void handleDbgIrqguard(const ArgList& args, const char* corrId,
                               ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
#ifndef HOST_BUILD
    DbgCtx ctx = dbgCtxFrom(handlerCtx);
    char rbuf[64];
    if (ctx.busDiag == nullptr) {
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "noimpl", "no i2c bus",
                                   corrId, replyFn, replyCtx);
        return;
    }
    if (args.suppliedCount >= 1) ctx.busDiag->setIrqGuard(args.args[0].ival != 0);
    char msg[24];
    snprintf(msg, sizeof(msg), "irqguard=%d", ctx.busDiag->irqGuard() ? 1 : 0);
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "dbg", msg,
                              corrId, replyFn, replyCtx);
#else
    (void)args; (void)corrId; (void)handlerCtx;
    char rbuf[64];
    CommandProcessor::replyErr(rbuf, sizeof(rbuf), "noimpl", "no i2c bus",
                               corrId, replyFn, replyCtx);
#endif
}

// ---------------------------------------------------------------------------
// DBG WEDGE
//   prefix "DBG WEDGE" — argTokens = up to 7 optional ints
//   parseFn: always succeeds; 0..7 INT args.
//   handler: parse optional params with defaults, then runWedgeTest.
// ---------------------------------------------------------------------------

static ParseResult parseDbgWedge(const char* const* tokens, int ntokens,
                                  const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult res;
    res.ok = true;
    // Accept up to 7 optional int args.
    int n = (ntokens > 7) ? 7 : ntokens;
    if (n > MAX_ARGS) n = MAX_ARGS;
    res.args.count = n;
    for (int i = 0; i < n; ++i) {
        res.args.args[i].type = ArgType::INT;
        res.args.args[i].ival = atoi(tokens[i]);
        res.args.args[i].fval = 0.0f;
        res.args.args[i].sval[0] = '\0';
    }
    res.args.suppliedCount = res.args.count;
    return res;
}

static void handleDbgWedge(const ArgList& args, const char* corrId,
                            ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
#ifndef HOST_BUILD
    DbgCtx ctx = dbgCtxFrom(handlerCtx);
    char rbuf[64];
    if (ctx.sched == nullptr) {
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "noimpl", "no scheduler",
                                   corrId, replyFn, replyCtx);
        return;
    }
    int wrate  = (args.count >= 1) ? args.args[0].ival : 50;
    int wwrite = (args.count >= 2) ? args.args[1].ival : 40;
    int wbus   = (args.count >= 3) ? args.args[2].ival : 400;
    int wdith  = (args.count >= 4) ? args.args[3].ival : 3;
    int wreg   = (args.count >= 5) ? args.args[4].ival : 0x46;
    int wsens  = (args.count >= 6) ? args.args[5].ival : 0;
    int wreal  = (args.count >= 7) ? args.args[6].ival : 0;
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "dbg", "wedge start",
                              corrId, replyFn, replyCtx);
    runWedgeTest(ctx.sched->uBit(), wrate, wwrite, wbus, wdith, wreg, wsens,
                 wreal, ctx.robot);
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "dbg", "wedge end",
                              corrId, replyFn, replyCtx);
#else
    (void)args; (void)corrId; (void)handlerCtx;
    char rbuf[64];
    CommandProcessor::replyErr(rbuf, sizeof(rbuf), "noimpl", "no scheduler",
                               corrId, replyFn, replyCtx);
#endif
}

// ---------------------------------------------------------------------------
// DBG OTOS BENCH
//   prefix "DBG OTOS BENCH" — argTokens = ["0"|"1"] + optional KV pairs
//   parseFn: arg[0] = INT (0 or 1); KVs: noiseXY, noiseH, drift (FLOAT).
//   handler: toggle bench mode; optionally apply noise params; reply OK.
//
//   DBG OTOS BENCH 1                  → enable bench mode
//   DBG OTOS BENCH 0                  → disable bench mode
//   DBG OTOS BENCH 1 noiseXY=0.02 noiseH=0.01 drift=0.0001
//
//   Reply: OK dbg otos bench=<0|1>
//
//   Calls hal.setOtosBench()/hal.isBenchMode() via the Hardware interface
//   (034-003/034-004).  HOST_BUILD: MockHAL records the flag; the sim can
//   observe the toggle without a NezhaHAL downcast.
// ---------------------------------------------------------------------------

static ParseResult parseDbgOtosBench(const char* const* tokens, int ntokens,
                                      const KVPair* kvs, int nkv)
{
    ParseResult res;
    res.ok = true;

    // arg[0]: enable flag (INT).
    if (ntokens >= 1) {
        res.args.count = 1;
        res.args.args[0].type  = ArgType::INT;
        res.args.args[0].ival  = atoi(tokens[0]);
        // NOTE: ival/fval are a UNION — do NOT also write fval here; that would
        // zero ival (0.0f == 0x00000000) and make every enable read as 0. (033-002)
        res.args.args[0].sval[0] = '\0';
    } else {
        res.args.count = 0;
    }

    // KV pairs: noiseXY, noiseH, drift — stored as FLOAT args [1], [2], [3]
    // with sentinels (-1.0f) meaning "not provided".
    // Use kvFloat helpers (ArgParse.h) in place of the inline kv loop.
    float noiseXY = kvFloat(kvs, nkv, "noiseXY", -1.0f);
    float noiseH  = kvFloat(kvs, nkv, "noiseH",  -1.0f);
    float drift   = kvFloat(kvs, nkv, "drift",   -1.0f);

    // Pack optional noise params into args [1..3] as FLOAT.
    // A fval of -1.0f signals "not supplied" to the handler.
    int base = (res.args.count >= 1) ? 1 : 0;
    int extra = 0;
    if (base + extra < MAX_ARGS) {
        res.args.args[base + extra].type  = ArgType::FLOAT;
        res.args.args[base + extra].fval  = noiseXY;
        // ival shares a union with fval — do NOT write it (would zero the float).
        res.args.args[base + extra].sval[0] = '\0';
        ++extra;
    }
    if (base + extra < MAX_ARGS) {
        res.args.args[base + extra].type  = ArgType::FLOAT;
        res.args.args[base + extra].fval  = noiseH;
        // ival shares a union with fval — do NOT write it (would zero the float).
        res.args.args[base + extra].sval[0] = '\0';
        ++extra;
    }
    if (base + extra < MAX_ARGS) {
        res.args.args[base + extra].type  = ArgType::FLOAT;
        res.args.args[base + extra].fval  = drift;
        // ival shares a union with fval — do NOT write it (would zero the float).
        res.args.args[base + extra].sval[0] = '\0';
        ++extra;
    }
    if (extra > 0) res.args.count = base + extra;
    res.args.suppliedCount = res.args.count;

    return res;
}

static void handleDbgOtosBench(const ArgList& args, const char* corrId,
                                 ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    DbgCtx ctx = dbgCtxFrom(handlerCtx);
    char rbuf[64];

    // arg[0]: enable flag (0 or 1); default to 0 if omitted.
    int enable = (args.count >= 1) ? args.args[0].ival : 0;

    // Toggle bench mode via the Hardware interface (034-003/034-004).
    // Firmware: NezhaHAL::setOtosBench swaps the active OTOS pointer.
    // HOST_BUILD: MockHAL::setOtosBench records the flag so the sim can
    // observe the toggle (round-trip test: bench=1 comes back correctly).
    ctx.robot->hal.setOtosBench(enable != 0);

// 034-006: benchOtosPtr()/setNoise() are only available with BENCH_OTOS_ENABLED.
#if !defined(HOST_BUILD) && defined(BENCH_OTOS_ENABLED)
    // Apply optional noise/drift params when enabling (firmware bench sensor only).
    // DebugCommands is firmware-only; NezhaHAL downcast is allowed here (034-004).
    // args[1]=noiseXY, args[2]=noiseH, args[3]=drift.  Sentinel = -1.0f.
    if (enable && args.count >= 4) {
        auto* nh = static_cast<NezhaHAL*>(&ctx.robot->hal);
        float noiseXY = args.args[1].fval;
        float noiseH  = args.args[2].fval;
        float drift   = args.args[3].fval;
        if (noiseXY < 0.0f) noiseXY = 0.02f;  // 2% linear sigma default
        if (noiseH  < 0.0f) noiseH  = 0.01f;  // 1% yaw sigma default
        if (drift   < 0.0f) drift   = 0.0f;   // no drift default
        nh->benchOtosPtr()->setNoise(noiseXY, noiseH, drift);
    }
#endif

    // Read back the active state via the Hardware interface (034-004).
    int active = ctx.robot->hal.isBenchMode() ? 1 : 0;
    char msg[32];
    snprintf(msg, sizeof(msg), "otos bench=%d", active);
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "dbg", msg,
                              corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// DBG OTOS
//   prefix "DBG OTOS" — parseFn=nullptr (no args).
//   handler: emit ideal / otos / fused pose line, then OK.
//
//   Reply lines:
//     ideal=<x>,<y>,<h> otos=<x>,<y>,<h> fused=<x>,<y>,<h> err=<dx>,<dy>,<dh>
//     OK dbg otos
//
//   ideal   = BenchOtosSensor noiseless accumulator.
//   otos    = BenchOtosSensor errored accumulator (what readTransformed returned).
//   fused   = state.actual.otosX/Y/H — EKF-fused pose written by otosCorrect().
//   err     = ideal − otos (per-axis).
//
//   In HOST_BUILD / MockHAL, benchOtosPtr() is unavailable; guard and emit
//   0,0,0 for ideal/otos.
// ---------------------------------------------------------------------------

static void handleDbgOtos(const ArgList& /*args*/, const char* corrId,
                           ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    DbgCtx ctx = dbgCtxFrom(handlerCtx);
    char rbuf[64];

    float idealX = 0.0f, idealY = 0.0f, idealH = 0.0f;
    float otosX  = 0.0f, otosY  = 0.0f, otosH  = 0.0f;

// 034-006: benchOtosPtr() is only available when BENCH_OTOS_ENABLED is
// defined.  HOST_BUILD skips NezhaHAL entirely (uses MockHAL).
#if !defined(HOST_BUILD) && defined(BENCH_OTOS_ENABLED)
    // DebugCommands is firmware-only; NezhaHAL downcast is allowed here
    // (034-004).  benchOtosPtr() is a NezhaHAL-specific accessor.
    auto* nh = static_cast<NezhaHAL*>(&ctx.robot->hal);
    BenchOtosSensor* bench = nh->benchOtosPtr();
    if (bench != nullptr) {
        idealX = bench->idealX();
        idealY = bench->idealY();
        idealH = bench->idealH();
        otosX  = bench->otosX();
        otosY  = bench->otosY();
        otosH  = bench->otosH();
    }
#else
    // HOST_BUILD or production (no BENCH_OTOS_ENABLED): ideal and otos are 0,0,0.
    (void)ctx;
#endif

    // Raw OTOS pose from state (written by Robot::otosCorrect into optical.pose).
    float fusedX = ctx.robot->state.actual.optical.pose.x;
    float fusedY = ctx.robot->state.actual.optical.pose.y;
    float fusedH = ctx.robot->state.actual.optical.pose.h;

    // err = ideal − otos (per axis).
    float errX = idealX - otosX;
    float errY = idealY - otosY;
    float errH = idealH - otosH;

    // F1 fix (034-004): CODAL/newlib-nano has no float printf, so %f emits
    // nothing on hardware.  Use scaled integers matching SNAP/TLM convention:
    //   position fields: integer mm  (round to nearest)
    //   heading field:   integer cdeg = rad * kAngleScale = rad * 18000/pi
    // kAngleScale = 18000.0f / 3.14159265f (see Odometry.h).
    // roundf() is available in newlib-nano without float printf.
    static constexpr float kAngleScale = 18000.0f / 3.14159265f;  // [cdeg/rad]

    // Emit the pose comparison line, then the OK reply.
    // Format: ideal=<xmm>,<ymm>,<hcdeg> otos=... fused=... err=...
    // All integer fields: positions in mm, headings in centidegrees (cdeg).
    // Raw OTOS STATUS byte + I2C-read-OK + the validity envelope used by the
    // TLM otos= gate.  Lets a bench probe see why otos= is suppressed.
    uint8_t otosStatus = 0xFF;
    bool statusOk = ctx.robot->hal.otos().readStatus(otosStatus);
    int valid = ctx.robot->state.actual.otos.valid ? 1 : 0;

    char pose_buf[200];
    snprintf(pose_buf, sizeof(pose_buf),
             "ideal=%d,%d,%d otos=%d,%d,%d fused=%d,%d,%d err=%d,%d,%d "
             "status=0x%02X statusOk=%d valid=%d",
             (int)roundf(idealX), (int)roundf(idealY), (int)roundf(idealH * kAngleScale),
             (int)roundf(otosX),  (int)roundf(otosY),  (int)roundf(otosH  * kAngleScale),
             (int)roundf(fusedX), (int)roundf(fusedY), (int)roundf(fusedH * kAngleScale),
             (int)roundf(errX),   (int)roundf(errY),   (int)roundf(errH   * kAngleScale),
             (unsigned)otosStatus, statusOk ? 1 : 0, valid);
    replyFn(pose_buf, replyCtx);

    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "dbg", "otos",
                              corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// DBG EST
//   prefix "DBG EST" — parseFn=nullptr (no args).
//   handler: dump three EstimateDump lines (enc, otos, fuse) via replyFn.
//
//   Reply lines:
//     EST enc   x=.. y=.. h=.. vx=.. vy=.. w=.. age=.. v=1
//     EST otos  x=.. y=.. h=.. vx=.. vy=.. w=.. age=.. v=1
//     EST fuse  x=.. y=.. h=.. vx=.. vy=.. w=.. age=.. v=1
//     OK dbg est
//
//   All fields are integer-scaled: positions in mm, headings in cdeg,
//   velocities in mm/s and mrad/s, age in ms.  Uses snprintf into a
//   stack-local buffer — no heap.
// ---------------------------------------------------------------------------

static void handleDbgEst(const ArgList& /*args*/, const char* corrId,
                          ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    DbgCtx ctx = dbgCtxFrom(handlerCtx);
    char rbuf[64];

    if (ctx.robot == nullptr) {
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "noimpl", "no robot",
                                   corrId, replyFn, replyCtx);
        return;
    }

    EstimateDump dump[3];
    uint32_t now_ms = ctx.robot->systemTime();
    dumpEstimates(ctx.robot->state.actual, now_ms, dump);

    // kAngleScale = 18000/pi — same constant used by Odometry.h.
    static constexpr float kAngleScale = 18000.0f / 3.14159265f;  // [cdeg/rad]
    static constexpr float kRadToMrad  = 1000.0f;

    for (int i = 0; i < 3; ++i) {
        const EstimateDump& d = dump[i];
        // age: cap to 9999999 so it fits in the buffer.
        uint32_t age = d.ageMs;
        if (age > 9999999u) age = 9999999u;
        char line[160];
        snprintf(line, sizeof(line),
                 "EST %-4s x=%d y=%d h=%d vx=%d vy=%d w=%d age=%u v=%d",
                 toString(d.source),
                 (int)d.pose.x,
                 (int)d.pose.y,
                 (int)(d.pose.h * kAngleScale),
                 (int)d.twist.vx_mmps,
                 (int)d.twist.vy_mmps,
                 (int)(d.twist.omega_rads * kRadToMrad),
                 (unsigned)age,
                 d.valid ? 1 : 0);
        replyFn(line, replyCtx);
    }

    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "dbg", "est",
                              corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// I2CW
//   prefix "I2CW" — argTokens = [<addr7-hex>, <byte-hex>, ...]
//   parseFn: validate ≥2 tokens (addr + at least one byte).
//   handler: write bytes to addr via bus.
// ---------------------------------------------------------------------------

static ParseResult parseI2cw(const char* const* tokens, int ntokens,
                              const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult res;
    if (ntokens < 2) {
        res.ok = false;
        res.err = { "badarg", "usage: I2CW <addr> <byte>..." };
        return res;
    }
    res.ok = true;
    // Store all raw tokens as STR args.
    int n = (ntokens > MAX_ARGS) ? MAX_ARGS : ntokens;
    res.args.count = n;
    for (int i = 0; i < n; ++i) {
        res.args.args[i].type = ArgType::STR;
        int j = 0;
        for (; tokens[i][j] != '\0' && j < (int)sizeof(res.args.args[i].sval) - 1; ++j)
            res.args.args[i].sval[j] = tokens[i][j];
        res.args.args[i].sval[j] = '\0';
        res.args.args[i].ival = 0;
        res.args.args[i].fval = 0.0f;
    }
    res.args.suppliedCount = res.args.count;
    return res;
}

static void handleI2cw(const ArgList& args, const char* corrId,
                        ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
#ifndef HOST_BUILD
    DbgCtx ctx = dbgCtxFrom(handlerCtx);
    char rbuf[64];
    if (ctx.busAccess == nullptr) {
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "noimpl", "no i2c bus",
                                   corrId, replyFn, replyCtx);
        return;
    }
    uint8_t addr7 = (uint8_t)strtol(args.args[0].sval, nullptr, 16);
    uint8_t data[24];
    int len = 0;
    for (int i = 1; i < args.count && len < (int)sizeof(data); ++i) {
        data[len++] = (uint8_t)strtol(args.args[i].sval, nullptr, 16);
    }
    int status = ctx.busAccess->write((uint16_t)(addr7 << 1), data, len);
    char body[48];
    snprintf(body, sizeof(body), "addr=0x%02X n=%d status=%d", addr7, len, status);
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "i2cw", body,
                              corrId, replyFn, replyCtx);
#else
    (void)args; (void)corrId; (void)handlerCtx;
    char rbuf[64];
    CommandProcessor::replyErr(rbuf, sizeof(rbuf), "noimpl", "no i2c bus",
                               corrId, replyFn, replyCtx);
#endif
}

// ---------------------------------------------------------------------------
// I2CR
//   prefix "I2CR" — argTokens = [<addr7-hex>, <count>, [<reg-hex>]]
//   parseFn: validate count 1..16, ≥2 tokens.
//   handler: optionally write reg byte with repeated start, then read.
// ---------------------------------------------------------------------------

static ParseResult parseI2cr(const char* const* tokens, int ntokens,
                              const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult res;
    if (ntokens < 2) {
        res.ok = false;
        res.err = { "badarg", "usage: I2CR <addr> <count> [reg]" };
        return res;
    }
    int count = atoi(tokens[1]);
    if (count < 1 || count > 16) {
        res.ok = false;
        res.err = { "range", "count" };
        return res;
    }
    res.ok = true;
    int n = (ntokens > MAX_ARGS) ? MAX_ARGS : ntokens;
    res.args.count = n;
    for (int i = 0; i < n; ++i) {
        res.args.args[i].type = ArgType::STR;
        int j = 0;
        for (; tokens[i][j] != '\0' && j < (int)sizeof(res.args.args[i].sval) - 1; ++j)
            res.args.args[i].sval[j] = tokens[i][j];
        res.args.args[i].sval[j] = '\0';
        res.args.args[i].ival = 0;
        res.args.args[i].fval = 0.0f;
    }
    res.args.suppliedCount = res.args.count;
    return res;
}

static void handleI2cr(const ArgList& args, const char* corrId,
                        ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
#ifndef HOST_BUILD
    DbgCtx ctx = dbgCtxFrom(handlerCtx);
    char rbuf[64];
    if (ctx.busAccess == nullptr) {
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "noimpl", "no i2c bus",
                                   corrId, replyFn, replyCtx);
        return;
    }
    uint8_t addr7 = (uint8_t)strtol(args.args[0].sval, nullptr, 16);
    int count  = atoi(args.args[1].sval);
    int wstatus = 0;
    if (args.count >= 3) {
        uint8_t reg = (uint8_t)strtol(args.args[2].sval, nullptr, 16);
        wstatus = ctx.busAccess->write((uint16_t)(addr7 << 1), &reg, 1, true);
    }
    uint8_t buf[16];
    int status = ctx.busAccess->read((uint16_t)(addr7 << 1), buf, count);
    char body[120];
    int pos = snprintf(body, sizeof(body),
                       "addr=0x%02X n=%d wstatus=%d status=%d data=",
                       addr7, count, wstatus, status);
    for (int i = 0; i < count && pos < (int)sizeof(body) - 4; ++i) {
        pos += snprintf(body + pos, (size_t)((int)sizeof(body) - pos),
                        "%s%02X", i ? "," : "", buf[i]);
    }
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "i2cr", body,
                              corrId, replyFn, replyCtx);
#else
    (void)args; (void)corrId; (void)handlerCtx;
    char rbuf[64];
    CommandProcessor::replyErr(rbuf, sizeof(rbuf), "noimpl", "no i2c bus",
                               corrId, replyFn, replyCtx);
#endif
}

// ---------------------------------------------------------------------------
// dbgCtxFrom — extract DbgCtx from handlerCtx.
// handlerCtx is always const_cast<DebugCommands*>(this).
// ---------------------------------------------------------------------------
static DbgCtx dbgCtxFrom(void* p)
{
    return reinterpret_cast<DebugCommands*>(p)->ctx();
}

// ---------------------------------------------------------------------------
// DebugCommands implementation
// ---------------------------------------------------------------------------

DebugCommands::DebugCommands(DbgCtx ctx)
    : _ctx(ctx)
{
}

std::vector<CommandDescriptor> DebugCommands::getCommands() const
{
    void* ctx = const_cast<DebugCommands*>(this);
    // Longest-prefix entries first within each group so dispatchTable picks
    // the most-specific match (e.g. "DBG LOOP RESET" beats "DBG LOOP").
    return {
        makeCmd(      "DBG LOOP RESET",  nullptr,              handleDbgLoopReset,  ctx, "badarg", ForceReply::SERIAL),                          // reset loop stats counters
        makeSchemaCmd("DBG LOOP",        &dbgLoopSchema,       handleDbgLoop,       ctx, "badarg", ForceReply::SERIAL),                          // report loop timing stats
        makeSchemaCmd("DBG I2CLOG",      &dbgI2clogSchema,     handleDbgI2clog,     ctx, "badarg", ForceReply::SERIAL, CMD_ACCESS_HARDWARE),     // dump I2C transaction log
        makeSchemaCmd("DBG I2C",         &dbgI2cSchema,        handleDbgI2c,        ctx, "badarg", ForceReply::SERIAL, CMD_ACCESS_HARDWARE),     // report I2C bus error counts
        makeSchemaCmd("DBG IRQGUARD",    &dbgIrqguardSchema,   handleDbgIrqguard,   ctx, "badarg", ForceReply::SERIAL),                          // enable/disable IRQ guard
        makeCmd(      "DBG WEDGE",       parseDbgWedge,        handleDbgWedge,      ctx, "badarg", ForceReply::SERIAL, CMD_ACCESS_HARDWARE),     // run encoder wedge self-check
        // 034-006: DBG OTOS BENCH and DBG OTOS are bench-build only.
        // BENCH_OTOS_ENABLED is defined in default firmware builds (PRODUCTION_BUILD=OFF).
        // HOST_BUILD always includes them so the sim suite is unaffected.
#if defined(BENCH_OTOS_ENABLED) || defined(HOST_BUILD)
        // DBG OTOS BENCH must appear BEFORE DBG OTOS — longest prefix wins.
        makeCmd(      "DBG OTOS BENCH",  parseDbgOtosBench,    handleDbgOtosBench,  ctx, "badarg", ForceReply::SERIAL, CMD_ACCESS_HARDWARE),     // enable/disable bench OTOS + set noise
        makeCmd(      "DBG OTOS",        nullptr,               handleDbgOtos,       ctx, "badarg", ForceReply::SERIAL),                          // query ideal/otos/fused pose
#endif // defined(BENCH_OTOS_ENABLED) || defined(HOST_BUILD)
        makeCmd(      "DBG EST",         nullptr,               handleDbgEst,        ctx, "badarg", ForceReply::SERIAL),                          // dump enc/otos/fuse EstimateDump lines
        makeCmd(      "I2CW",            parseI2cw,            handleI2cw,          ctx, "badarg", ForceReply::SERIAL, CMD_ACCESS_HARDWARE),     // raw I2C write (addr reg data…)
        makeCmd(      "I2CR",            parseI2cr,            handleI2cr,          ctx, "badarg", ForceReply::SERIAL, CMD_ACCESS_HARDWARE),     // raw I2C read (addr reg count)
    };
}
