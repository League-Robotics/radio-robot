#include "OtosCommands.h"
#include "CommandProcessor.h"
#include <cstdio>
#include <cstdlib>

// ===========================================================================
// OtosCommands — OI, OZ, OR, OP, OV, OL, OA  (041-002)
//
// Moved VERBATIM from Odometry.cpp's Commandable implementation.  The only
// changes are the context type (OtosCtx* instead of OdomCtx*) and the context
// pointer (&_ctx instead of &_odomCtx).  Parse formats, reply strings, and
// device effects are byte-identical.
//
// Context type: OtosCtx* (cast from handlerCtx); all handlers use otos
// (handleOP additionally reads hwState).
// ===========================================================================

OtosCommands::OtosCommands() : _ctx{nullptr, nullptr} {}

// ---------------------------------------------------------------------------
// Parse functions — strip verb token so tokens[0] is the first argument.
// ---------------------------------------------------------------------------

// OI — no arguments
static ParseResult parseOI(const char* const* /*tokens*/, int /*ntokens*/,
                            const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult r; r.ok = true; r.args.count = 0; return r;
}

// OZ — no arguments
static ParseResult parseOZ(const char* const* /*tokens*/, int /*ntokens*/,
                            const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult r; r.ok = true; r.args.count = 0; return r;
}

// OR — no arguments
static ParseResult parseOR(const char* const* /*tokens*/, int /*ntokens*/,
                            const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult r; r.ok = true; r.args.count = 0; return r;
}

// OP — no arguments
static ParseResult parseOP(const char* const* /*tokens*/, int /*ntokens*/,
                            const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult r; r.ok = true; r.args.count = 0; return r;
}

// OV <x> <y> <h> — three mandatory int16 arguments
static ParseResult parseOV(const char* const* tokens, int ntokens,
                            const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult r;
    if (ntokens < 3) {
        r.ok = false; r.err = { "badarg", nullptr }; return r;
    }
    r.ok = true;
    r.args.count = 3;
    r.args.args[0].type = ArgType::INT; r.args.args[0].ival = (int16_t)atoi(tokens[0]);
    r.args.args[1].type = ArgType::INT; r.args.args[1].ival = (int16_t)atoi(tokens[1]);
    r.args.args[2].type = ArgType::INT; r.args.args[2].ival = (int16_t)atoi(tokens[2]);
    return r;
}

// OL [val] — optional int8 scalar
static ParseResult parseOL(const char* const* tokens, int ntokens,
                            const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult r; r.ok = true;
    if (ntokens >= 1) {
        r.args.count = 1;
        r.args.args[0].type = ArgType::INT;
        r.args.args[0].ival = (int8_t)atoi(tokens[0]);
    } else {
        r.args.count = 0;
    }
    return r;
}

// OA [val] — optional int8 scalar
static ParseResult parseOA(const char* const* tokens, int ntokens,
                            const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult r; r.ok = true;
    if (ntokens >= 1) {
        r.args.count = 1;
        r.args.args[0].type = ArgType::INT;
        r.args.args[0].ival = (int8_t)atoi(tokens[0]);
    } else {
        r.args.count = 0;
    }
    return r;
}

// ---------------------------------------------------------------------------
// Handler functions
// ---------------------------------------------------------------------------

static void handleOI(const ArgList& /*args*/, const char* corrId,
                     ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    OtosCtx* c = reinterpret_cast<OtosCtx*>(handlerCtx);
    char rbuf[64];
    if (!c->otos->is_initialized()) {
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "nodev", "oi",
                                   corrId, replyFn, replyCtx);
        return;
    }
    c->otos->init();
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "oi", nullptr,
                              corrId, replyFn, replyCtx);
}

static void handleOZ(const ArgList& /*args*/, const char* corrId,
                     ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    OtosCtx* c = reinterpret_cast<OtosCtx*>(handlerCtx);
    char rbuf[64];
    if (!c->otos->is_initialized()) {
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "nodev", "oz",
                                   corrId, replyFn, replyCtx);
        return;
    }
    c->otos->setPositionRaw(0, 0, 0);
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "oz", nullptr,
                              corrId, replyFn, replyCtx);
}

static void handleOR(const ArgList& /*args*/, const char* corrId,
                     ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    OtosCtx* c = reinterpret_cast<OtosCtx*>(handlerCtx);
    char rbuf[64];
    if (!c->otos->is_initialized()) {
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "nodev", "or",
                                   corrId, replyFn, replyCtx);
        return;
    }
    c->otos->resetTracking();
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "or", nullptr,
                              corrId, replyFn, replyCtx);
}

// handleOP — report current OTOS pose from cached HardwareState.
//
// Reads hwState->otosX/Y/H (values written by Robot::otosCorrect() each OTOS
// task tick) instead of calling otos->getPositionRaw() on the device.
// This is the only OTOS command that does NOT access hardware (flag = CMD_NONE).
// If hwState is null (test harness without OTOS), returns zeros.
//
// Reply format: OK op x=<mm> y=<mm> h=<mrad>
//   x, y: OTOS position in integer mm.
//   h: OTOS heading in integer mrad (milliradians, for precision).
static void handleOP(const ArgList& /*args*/, const char* corrId,
                     ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    OtosCtx* c = reinterpret_cast<OtosCtx*>(handlerCtx);
    char rbuf[96];

    float x = 0.0f, y = 0.0f, h = 0.0f;
    if (c->hwState != nullptr) {
        x = c->hwState->otosX;
        y = c->hwState->otosY;
        h = c->hwState->otosH;
    }

    // Convert heading from radians to integer milliradians for the reply.
    int x_mm   = (int)x;
    int y_mm   = (int)y;
    int h_mrad = (int)(h * 1000.0f);

    char body[64];
    snprintf(body, sizeof(body), "x=%d y=%d h=%d", x_mm, y_mm, h_mrad);
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "op", body,
                              corrId, replyFn, replyCtx);
}

static void handleOV(const ArgList& args, const char* corrId,
                     ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    OtosCtx* c = reinterpret_cast<OtosCtx*>(handlerCtx);
    char rbuf[96];
    if (!c->otos->is_initialized()) {
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "nodev", "ov",
                                   corrId, replyFn, replyCtx);
        return;
    }
    int16_t ox = (int16_t)args.args[0].ival;
    int16_t oy = (int16_t)args.args[1].ival;
    int16_t oh = (int16_t)args.args[2].ival;
    c->otos->setPositionRaw(ox, oy, oh);
    char body[48];
    snprintf(body, sizeof(body), "x=%d y=%d h=%d", (int)ox, (int)oy, (int)oh);
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "setpos", body,
                              corrId, replyFn, replyCtx);
}

static void handleOL(const ArgList& args, const char* corrId,
                     ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    OtosCtx* c = reinterpret_cast<OtosCtx*>(handlerCtx);
    char rbuf[64];
    if (!c->otos->is_initialized()) {
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "nodev", "ol",
                                   corrId, replyFn, replyCtx);
        return;
    }
    if (args.count >= 1) {
        c->otos->setLinearScalar((int8_t)args.args[0].ival);
    }
    int8_t val = c->otos->getLinearScalar();
    char body[24];
    snprintf(body, sizeof(body), "scalar=%d", (int)val);
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "linear", body,
                              corrId, replyFn, replyCtx);
}

static void handleOA(const ArgList& args, const char* corrId,
                     ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    OtosCtx* c = reinterpret_cast<OtosCtx*>(handlerCtx);
    char rbuf[64];
    if (!c->otos->is_initialized()) {
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "nodev", "oa",
                                   corrId, replyFn, replyCtx);
        return;
    }
    if (args.count >= 1) {
        c->otos->setAngularScalar((int8_t)args.args[0].ival);
    }
    int8_t val = c->otos->getAngularScalar();
    char body[24];
    snprintf(body, sizeof(body), "scalar=%d", (int)val);
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "angular", body,
                              corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
std::vector<CommandDescriptor> OtosCommands::getCommands() const
{
    void* ctx = const_cast<OtosCtx*>(&_ctx);
    return {
        makeCmd("OI", parseOI, handleOI, ctx, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE), // OTOS init: re-initialise sensor
        makeCmd("OZ", parseOZ, handleOZ, ctx, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE), // OTOS zero: reset position to 0,0,0
        makeCmd("OR", parseOR, handleOR, ctx, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE), // OTOS read: one-shot position snapshot
        makeCmd("OP", parseOP, handleOP, ctx, "badarg"), // OTOS position: report current x,y,h (reads cached state)
        makeCmd("OV", parseOV, handleOV, ctx, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE), // OTOS velocity: report vx,vy,omega
        makeCmd("OL", parseOL, handleOL, ctx, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE), // OTOS linear scalar calibration
        makeCmd("OA", parseOA, handleOA, ctx, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE), // OTOS angular scalar calibration
    };
}
