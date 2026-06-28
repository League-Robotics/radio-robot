#include "OtosCommands.h"
#include "CommandProcessor.h"
#include "types/ArgSchema.h"
#include <cstdio>

// ===========================================================================
// OtosCommands — OI, OZ, OR, OP, OV, OL, OA  (041-002, migrated 051-004)
//
// Moved VERBATIM from Odometry.cpp's Commandable implementation.  The only
// changes are the context type (OtosCtx* instead of OdomCtx*) and the context
// pointer (&_ctx instead of &_odomCtx).  Parse formats, reply strings, and
// device effects are byte-identical.
//
// Migration (051-004): bespoke parse functions replaced with ArgSchema /
// nullptr registrations.  A shared otosReady() helper factors the repeated
// nodev guard.
//
// Context type: OtosCtx* (cast from handlerCtx); all handlers use otos
// (handleOP additionally reads hwState).
// ===========================================================================

OtosCommands::OtosCommands() : _ctx{nullptr, nullptr} {}

// ---------------------------------------------------------------------------
// Argument schemas — declarative replacements for the hand-written parsers.
// ---------------------------------------------------------------------------

// OV <x> <y> <h> — three mandatory positional INTs; no range check (silent
// int16_t truncation in the handler, as before).
static const ArgDef ovDefs[3] = {
    { "x", ArgKind::INT, false, 0, 0 },
    { "y", ArgKind::INT, false, 0, 0 },
    { "h", ArgKind::INT, false, 0, 0 },
};
static const ArgSchema ovSchema = { ovDefs, 3, 3, false, nullptr };

// OL [val] — optional int8 scalar; 0 or 1 INT token; no range check.
static const ArgDef olDefs[1] = {
    { "scalar", ArgKind::INT, false, 0, 0 },
};
static const ArgSchema olSchema = { olDefs, 1, 0, false, nullptr };

// OA [val] — optional int8 scalar; 0 or 1 INT token; no range check.
static const ArgDef oaDefs[1] = {
    { "scalar", ArgKind::INT, false, 0, 0 },
};
static const ArgSchema oaSchema = { oaDefs, 1, 0, false, nullptr };

// ---------------------------------------------------------------------------
// otosReady — shared nodev guard.
//
// Returns false and emits "ERR nodev <verb>" when the OTOS sensor has not
// been initialised.  Each handler that touches hardware calls this first;
// handleOP does NOT call it (it only reads cached state).
// ---------------------------------------------------------------------------
static bool otosReady(OtosCtx* c, const char* verb, char* rbuf, int rbsz,
                      const char* corrId, ReplyFn fn, void* ctx)
{
    if (!c->otos->is_initialized()) {
        CommandProcessor::replyErr(rbuf, rbsz, "nodev", verb, corrId, fn, ctx);
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Handler functions
// ---------------------------------------------------------------------------

static void handleOI(const ArgList& /*args*/, const char* corrId,
                     ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    OtosCtx* c = reinterpret_cast<OtosCtx*>(handlerCtx);
    char rbuf[64];
    if (!otosReady(c, "oi", rbuf, sizeof(rbuf), corrId, replyFn, replyCtx)) return;
    c->otos->init();
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "oi", nullptr,
                              corrId, replyFn, replyCtx);
}

static void handleOZ(const ArgList& /*args*/, const char* corrId,
                     ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    OtosCtx* c = reinterpret_cast<OtosCtx*>(handlerCtx);
    char rbuf[64];
    if (!otosReady(c, "oz", rbuf, sizeof(rbuf), corrId, replyFn, replyCtx)) return;
    c->otos->setPositionRaw(0, 0, 0);
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "oz", nullptr,
                              corrId, replyFn, replyCtx);
}

static void handleOR(const ArgList& /*args*/, const char* corrId,
                     ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    OtosCtx* c = reinterpret_cast<OtosCtx*>(handlerCtx);
    char rbuf[64];
    if (!otosReady(c, "or", rbuf, sizeof(rbuf), corrId, replyFn, replyCtx)) return;
    c->otos->resetTracking();
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "or", nullptr,
                              corrId, replyFn, replyCtx);
}

// handleOP — report current OTOS pose from cached HardwareState.
//
// Reads hwState->optical.pose.{x,y,h} (values written by Robot::otosCorrect()
// each OTOS task tick via actual.optical.pose) instead of calling the device.
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
        x = c->hwState->optical.pose.x;
        y = c->hwState->optical.pose.y;
        h = c->hwState->optical.pose.h;
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
    if (!otosReady(c, "ov", rbuf, sizeof(rbuf), corrId, replyFn, replyCtx)) return;
    // Cast to int16_t at use site — preserves existing silent truncation behaviour.
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
    if (!otosReady(c, "ol", rbuf, sizeof(rbuf), corrId, replyFn, replyCtx)) return;
    if (args.count >= 1) {
        // Cast to int8_t at use site — preserves existing silent truncation behaviour.
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
    if (!otosReady(c, "oa", rbuf, sizeof(rbuf), corrId, replyFn, replyCtx)) return;
    if (args.count >= 1) {
        // Cast to int8_t at use site — preserves existing silent truncation behaviour.
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
        makeCmd("OI", nullptr, handleOI, ctx, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE), // OTOS init: re-initialise sensor
        makeCmd("OZ", nullptr, handleOZ, ctx, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE), // OTOS zero: reset position to 0,0,0
        makeCmd("OR", nullptr, handleOR, ctx, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE), // OTOS read: one-shot position snapshot
        makeCmd("OP", nullptr, handleOP, ctx, "badarg"), // OTOS position: report current x,y,h (reads cached state)
        makeSchemaCmd("OV", &ovSchema, handleOV, ctx, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE), // OTOS velocity: set position x,y,h
        makeSchemaCmd("OL", &olSchema, handleOL, ctx, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE), // OTOS linear scalar calibration
        makeSchemaCmd("OA", &oaSchema, handleOA, ctx, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE), // OTOS angular scalar calibration
    };
}
