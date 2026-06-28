// ---------------------------------------------------------------------------
// ConfigCommands.cpp — config-registry command handlers (GET, GET VEL, SET).
//
// Split from source/commands/SystemCommands.cpp (finding A3). Contains:
//   - file-local static parse*/handle* functions for GET VEL, GET, and SET
//   - appendConfigCommands() seam that registers these descriptors onto the
//     command table built by Robot::buildCommandTable (in SystemCommands.cpp)
//
// handleGet/handleSet are free functions declared in ConfigRegistry.h and
// defined in source/robot/ConfigRegistry.cpp — only the parse* halves and the
// GET VEL parse/handle pair live here. Behaviour is unchanged: descriptors are
// wired exactly as buildCommandTable previously wired them.
//
// Migration (051-007): parseGetVel and parseGet removed; GET VEL registered
// with parseFn=nullptr; GET registered with makeSchemaCmd (variadic=true schema);
// parseSet retained (custom KV-to-"k=v" encoding) with body rewritten using
// argStr helper.
// ---------------------------------------------------------------------------

#include "ConfigCommands.h"
#include "Robot.h"
#include "CommandProcessor.h"
#include "ConfigRegistry.h"

#include <cstdio>
#include <cstring>
#include <cstdlib>

namespace {

// ---------------------------------------------------------------------------
// Internal accessor -- cast handlerCtx to RobotSysCtx* (GET VEL only).
// ---------------------------------------------------------------------------
static RobotSysCtx& sysCtxFrom(void* p)
{
    return *reinterpret_cast<RobotSysCtx*>(p);
}

// ---------------------------------------------------------------------------
// GET VEL -- per-wheel velocity readout (separate descriptor from GET).
//   prefix "GET VEL"; parseFn nullptr (no-arg command).
//   Reply: OK get vel=<vL>:E,<vR>:E
// ---------------------------------------------------------------------------

static void handleGetVel(const ArgList& /*args*/, const char* corrId,
                          ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    Robot* robot = sysCtxFrom(handlerCtx).robot;
    // Array convention: [0]=R (FR), [1]=L (FL) — see ActualState.h.
    float vL = robot->state.actual.velMms[1];
    float vR = robot->state.actual.velMms[0];
    char rbuf[64];
    char body[48];
    snprintf(body, sizeof(body), "vel=%d:E,%d:E", (int)vL, (int)vR);
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "get", body, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// GET — variadic ArgSchema: each token becomes args[i].sval (key name).
//   handleGet (defined in ConfigRegistry.cpp) reads args.args[i].sval.
// ---------------------------------------------------------------------------

static const ArgSchema getSchema = { nullptr, 0, 0, true, nullptr };

// ---------------------------------------------------------------------------
// parseSet -- convert kv pairs into "key=value" STR args for handleSet.
//   Custom parseFn retained: reads kvs[], not tokens[].
// ---------------------------------------------------------------------------

static ParseResult parseSet(const char* const* /*tokens*/, int /*ntokens*/,
                             const KVPair* kvs, int nkv)
{
    ParseResult r;
    if (nkv == 0) {
        r.ok = false;
        r.err = { "badarg", "no key=value pairs" };
        return r;
    }
    r.ok = true;
    int n = (nkv > MAX_ARGS) ? MAX_ARGS : nkv;
    r.args.count = 0;
    for (int i = 0; i < n; ++i) {
        if (!kvs[i].key) continue;
        char* dst = r.args.args[r.args.count].sval;
        int cap = (int)(sizeof(r.args.args[0].sval) - 1);
        int written = snprintf(dst, (size_t)(cap + 1), "%s=%s",
                               kvs[i].key, kvs[i].value);
        if (written > cap) dst[cap] = '\0';
        r.args.args[r.args.count].type = ArgType::STR;
        r.args.args[r.args.count].ival = 0;
        r.args.args[r.args.count].fval = 0.0f;
        ++r.args.count;
    }
    return r;
}

}  // anonymous namespace

// ---------------------------------------------------------------------------
// appendConfigCommands -- register GET VEL, GET, and SET descriptors.
//
// GET VEL is registered before GET so the longer prefix wins the linear scan
// in buildCommandTable's command table (it is appended to cmds in the same
// relative order it previously held).
// ---------------------------------------------------------------------------
void appendConfigCommands(std::vector<CommandDescriptor>& cmds,
                          CfgCtx* cfgCtx, void* sysCtx)
{
    cmds.push_back(makeCmd("GET VEL",  nullptr,   handleGetVel, sysCtx,  "badarg")); // get velocity (no-arg)
    cmds.push_back(makeSchemaCmd("GET", &getSchema, handleGet,  cfgCtx,  "badkey")); // get config value by key (variadic)
    cmds.push_back(makeCmd("SET",      parseSet,   handleSet,   cfgCtx,  "badkey")); // set config value by key
}
