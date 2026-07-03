// SimCommands.cpp — SIMSET/SIMGET wire-command surface (069-003, sim-only).
//
// Reuses SET/GET's existing generic KV-parsing machinery verbatim (a local
// copy of ConfigCommands.cpp's file-local parseSet, and GET's variadic
// ArgSchema shape) -- see ConfigCommands.cpp for the machinery this mirrors.
//
// kSimRegistry[] dispatches through named setter/getter FUNCTION POINTERS
// over SimHardware&, not offsetof -- PhysicsWorld is an encapsulated class
// with invariants, not a POD struct (architecture-update.md Design
// Rationale Decision 3).
//
// 069-005: the setter/getter functions themselves live in SimSetters.h
// (namespace simsetters), shared with tests/_infra/sim/sim_api.cpp's legacy
// ctypes forwards -- single source of truth per knob, not two independently-
// maintained call sites into PhysicsWorld/SimOdometer.
//
// handleSimSet's atomicity matches handleSet's real behaviour (confirmed by
// reading ConfigRegistry.cpp's handleSet, not assumed): a SIMSET with ANY
// unknown key or unparsable value applies NONE of the keys in that command
// (a two-pass validate-then-commit split, since there is no single POD
// struct to build a discardable "candidate" copy of the way handleSet does).
// ERR badval reports just the key name (handleSet's actual behaviour), not
// "key=value" as an unread description might suggest.

#include "SimCommands.h"
#include "CommandProcessor.h"
#include "SimSetters.h"
#include "hal/sim/SimHardware.h"

#include <cstring>
#include <cstdio>
#include <cstdlib>

namespace {

// ---------------------------------------------------------------------------
// SimEntry — one kSimRegistry[] row: key name + named setter/getter function
// pointers over SimHardware&.  Decision 3: no offsetof, no POD assumption.
// ---------------------------------------------------------------------------
typedef void  (*SimSetFn)(SimHardware&, float);
typedef float (*SimGetFn)(SimHardware&);

struct SimEntry {
    const char* key;
    SimSetFn    setter;
    SimGetFn    getter;
};

// ---------------------------------------------------------------------------
// 069-005: every row below points DIRECTLY at a simsetters:: free function
// (SimSetters.h) -- the same functions tests/_infra/sim/sim_api.cpp's legacy
// ctypes forwards call for the knobs that have a ctypes counterpart. No
// per-row adapter lives in this file anymore; SimSetters.h is the single
// source of truth per knob (architecture-update.md Design Rationale
// Decision 3, Sprint Changes Summary item 1).
//
// kSimRegistry[] -- ticket 003's first batch (rows 1-5). Ticket 004 appends
// the encoder-report-error / OTOS-error rows below (rows 6-17). Ticket 072-001
// appends the motor stiction/breakaway + optional lag rows (rows 18-21).
// ---------------------------------------------------------------------------
static const SimEntry kSimRegistry[] = {
    { "bodyRotScrub", simsetters::bodyRotScrub, simsetters::getBodyRotScrub },
    { "bodyLinScrub", simsetters::bodyLinScrub, simsetters::getBodyLinScrub },
    { "trackwidthMm", simsetters::trackwidth, simsetters::getTrackwidth },
    { "motorOffsetL", simsetters::motorOffsetL, simsetters::getMotorOffsetL },
    { "motorOffsetR", simsetters::motorOffsetR, simsetters::getMotorOffsetR },
    { "encScaleErrL", simsetters::encoderScaleErrorL, simsetters::getEncoderScaleErrorL },
    { "encScaleErrR", simsetters::encoderScaleErrorR, simsetters::getEncoderScaleErrorR },
    { "encSlipL",     simsetters::encoderSlipL,     simsetters::getEncoderSlipL },
    { "encSlipR",     simsetters::encoderSlipR,     simsetters::getEncoderSlipR },
    { "encNoiseL",    simsetters::encoderNoiseL,    simsetters::getEncoderNoiseL },
    { "encNoiseR",    simsetters::encoderNoiseR,    simsetters::getEncoderNoiseR },
    { "otosLinScaleErr",  simsetters::otosLinScaleErr,  simsetters::getOtosLinScaleErr },
    { "otosAngScaleErr",  simsetters::otosAngScaleErr,  simsetters::getOtosAngScaleErr },
    { "otosLinNoise",     simsetters::otosLinNoise,     simsetters::getOtosLinNoise },
    { "otosYawNoise",     simsetters::otosYawNoise,     simsetters::getOtosYawNoise },
    { "otosLinDriftMmS",  simsetters::otosLinearDrift, simsetters::getOtosLinearDrift },
    { "otosYawDriftDegS", simsetters::otosYawDrift,    simsetters::getOtosYawDrift },
    // 072-001: motor stiction/breakaway gate + optional first-order lag.
    { "stictionPwmL", simsetters::stictionPwmL, simsetters::getStictionPwmL },
    { "stictionPwmR", simsetters::stictionPwmR, simsetters::getStictionPwmR },
    { "motorLagMsL",  simsetters::motorLagMsL,  simsetters::getMotorLagMsL },
    { "motorLagMsR",  simsetters::motorLagMsR,  simsetters::getMotorLagMsR },
};
static const int kSimRegistryCount = (int)(sizeof(kSimRegistry) / sizeof(kSimRegistry[0]));

// ---------------------------------------------------------------------------
// parseSimSet -- local copy of ConfigCommands.cpp's file-local parseSet
// (identical behaviour: convert kv pairs into "key=value" STR args).  Kept
// as a separate copy rather than sharing parseSet across translation units
// per the ticket's explicitly-sanctioned option (SimCommands.h/.cpp must not
// pull in ConfigCommands.h, which is not sim-only).
// ---------------------------------------------------------------------------
static ParseResult parseSimSet(const char* const* /*tokens*/, int /*ntokens*/,
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

// SIMGET -- variadic ArgSchema, identical shape to GET's getSchema
// (ConfigCommands.cpp): each token becomes args[i].sval (a key name).
static const ArgSchema simGetSchema = { nullptr, 0, 0, true, nullptr };

// ---------------------------------------------------------------------------
// handleSimSet -- HandlerFn-compatible SIMSET handler.
//
// args.args[0..count-1].sval carries "key=value" strings (parseSimSet's
// output).  handlerCtx is a SimHardware* (NOT the SimCommands instance --
// SimCommands holds no state beyond the SimHardware& reference itself).
//
// Two-pass validate-then-commit: PhysicsWorld/SimHardware are not a single
// POD struct, so there is no "candidate copy" to build the way handleSet
// does. Pass 1 validates every key/value WITHOUT calling any setter; if any
// key is unknown or any value fails to parse, NOTHING is applied (same
// all-or-nothing guarantee SET gives RobotConfig). Pass 2 (only reached when
// pass 1 found zero errors) calls every entry's setter and builds the
// "applied" reply body.
// ---------------------------------------------------------------------------
static void handleSimSet(const ArgList& args, const char* corrId,
                          ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    SimHardware& hal = *reinterpret_cast<SimHardware*>(handlerCtx);
    char rbuf[128];

    struct Pending {
        const SimEntry* entry;
        float           value;
        char            origStr[32];
    };
    Pending pending[MAX_ARGS];
    int nPending = 0;
    bool anyErr = false;

    for (int i = 0; i < args.count; ++i) {
        // Copy sval so we can split in place (mirrors handleSet).
        char kvbuf[64];
        int kvlen = 0;
        for (const char* p = args.args[i].sval;
             *p && kvlen < (int)sizeof(kvbuf) - 1; ++p, ++kvlen) {
            kvbuf[kvlen] = *p;
        }
        kvbuf[kvlen] = '\0';

        char* eq = strchr(kvbuf, '=');
        if (!eq) continue;  // malformed: no '='; parseSimSet never produces this
        *eq = '\0';
        const char* k = kvbuf;
        const char* v = eq + 1;
        if (k[0] == '\0') continue;

        const SimEntry* entry = nullptr;
        for (int r = 0; r < kSimRegistryCount; ++r) {
            if (strcmp(kSimRegistry[r].key, k) == 0) { entry = &kSimRegistry[r]; break; }
        }
        if (!entry) {
            CommandProcessor::replyErr(rbuf, (int)sizeof(rbuf),
                                       "badkey", k, corrId, replyFn, replyCtx);
            anyErr = true;
            continue;
        }

        if (v[0] == '\0') {
            CommandProcessor::replyErr(rbuf, (int)sizeof(rbuf),
                                       "badval", k, corrId, replyFn, replyCtx);
            anyErr = true;
            continue;
        }
        char* endp = nullptr;
        float fv = strtof(v, &endp);
        if (endp == v || *endp != '\0') {
            CommandProcessor::replyErr(rbuf, (int)sizeof(rbuf),
                                       "badval", k, corrId, replyFn, replyCtx);
            anyErr = true;
            continue;
        }

        if (nPending < MAX_ARGS) {
            pending[nPending].entry = entry;
            pending[nPending].value = fv;
            int j = 0;
            for (; v[j] != '\0' && j < (int)sizeof(pending[nPending].origStr) - 1; ++j) {
                pending[nPending].origStr[j] = v[j];
            }
            pending[nPending].origStr[j] = '\0';
            ++nPending;
        }
    }

    // Any parse/badkey error -- commit NOTHING (all-or-nothing, matches SET).
    if (anyErr) return;

    // Pass 2: commit every validated pair, then reply.
    char applied[256];
    int apos = 0;
    int arem = (int)sizeof(applied);
    for (int i = 0; i < nPending; ++i) {
        pending[i].entry->setter(hal, pending[i].value);
        if (apos > 0 && arem > 1) { applied[apos++] = ' '; --arem; }
        int w = snprintf(applied + apos, (size_t)arem, "%s=%s",
                         pending[i].entry->key, pending[i].origStr);
        if (w > 0 && w < arem) { apos += w; arem -= w; }
    }
    applied[apos] = '\0';
    CommandProcessor::replyOK(rbuf, (int)sizeof(rbuf), "simset", applied,
                              corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// appendSimKeyValue -- append one "key=value" pair to a string buffer.
// Mirrors ConfigRegistry.cpp's appendKeyValue; every kSimRegistry[] row is
// currently float-typed (%.3f), matching CFG_FLOAT's wire format.
// ---------------------------------------------------------------------------
static int appendSimKeyValue(char* buf, int remaining, const SimEntry& entry,
                             SimHardware& hal)
{
    if (remaining <= 1) return 0;
    float v = entry.getter(hal);
    int written = snprintf(buf, (size_t)remaining, "%s=%.3f", entry.key, (double)v);
    if (written < 0 || written >= remaining) return remaining - 1;
    return written;
}

// Maximum content bytes per SIMCFG line for the all-keys dump -- mirrors
// ConfigRegistry.cpp's kCfgChunkMax (stay well under CODAL's 255-byte serial
// TX buffer as the registry grows past this ticket's initial 5 rows).
static const int kSimCfgChunkMax = 200;

// ---------------------------------------------------------------------------
// handleSimGet -- HandlerFn-compatible SIMGET handler.
//
// args.args[0..count-1].sval carries requested key names (empty list = dump
// all).  handlerCtx is a SimHardware*.  Emits one-or-more SIMCFG response
// lines (chunked the same way GET's bare dump chunks CFG), plus one ERR
// badkey per unknown named key.
// ---------------------------------------------------------------------------
static void handleSimGet(const ArgList& args, const char* corrId,
                          ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    SimHardware& hal = *reinterpret_cast<SimHardware*>(handlerCtx);
    char rbuf[128];

    bool anyKey = (args.count == 0);  // no args -> dump all

    if (anyKey) {
        char line[256];
        int pos = 0;
        int rem = (int)sizeof(line);

        int n = snprintf(line + pos, (size_t)rem, "SIMCFG");
        if (n > 0 && n < rem) { pos += n; rem -= n; }

        for (int i = 0; i < kSimRegistryCount; ++i) {
            char probe[48];
            int wProbe = appendSimKeyValue(probe, (int)sizeof(probe) - 1, kSimRegistry[i], hal);
            int entrySize = 1 + wProbe;  // 1 for the leading space

            int contentBytes = pos - 6;  // "SIMCFG" prefix length
            if (contentBytes > 0 && contentBytes + entrySize > kSimCfgChunkMax) {
                if (corrId && corrId[0] != '\0' && rem > 3) {
                    int w = snprintf(line + pos, (size_t)rem, " #%s", corrId);
                    if (w > 0 && w < rem) { pos += w; rem -= w; }
                }
                line[pos] = '\0';
                replyFn(line, replyCtx);

                pos = 0; rem = (int)sizeof(line);
                n = snprintf(line + pos, (size_t)rem, "SIMCFG");
                if (n > 0 && n < rem) { pos += n; rem -= n; }
            }

            if (rem > 2) {
                line[pos++] = ' '; --rem;
                int w = appendSimKeyValue(line + pos, rem, kSimRegistry[i], hal);
                pos += w; rem -= w;
            }
        }

        if (corrId && corrId[0] != '\0' && rem > 3) {
            int w = snprintf(line + pos, (size_t)rem, " #%s", corrId);
            if (w > 0 && w < rem) { pos += w; rem -= w; }
        }
        line[pos] = '\0';
        replyFn(line, replyCtx);

    } else {
        char line[768];
        int pos = 0;
        int rem = (int)sizeof(line);

        int n = snprintf(line + pos, (size_t)rem, "SIMCFG");
        if (n > 0 && n < rem) { pos += n; rem -= n; }

        for (int t = 0; t < args.count && rem > 2; ++t) {
            const char* reqKey = args.args[t].sval;
            bool found = false;
            for (int i = 0; i < kSimRegistryCount; ++i) {
                if (strcmp(kSimRegistry[i].key, reqKey) == 0) {
                    line[pos++] = ' '; --rem;
                    int w = appendSimKeyValue(line + pos, rem, kSimRegistry[i], hal);
                    pos += w; rem -= w;
                    found = true;
                    break;
                }
            }
            if (!found) {
                CommandProcessor::replyErr(rbuf, (int)sizeof(rbuf),
                                           "badkey", reqKey, corrId, replyFn, replyCtx);
            }
        }

        if (corrId && corrId[0] != '\0' && rem > 3) {
            int w = snprintf(line + pos, (size_t)rem, " #%s", corrId);
            if (w > 0 && w < rem) { pos += w; rem -= w; }
        }
        line[pos] = '\0';
        replyFn(line, replyCtx);
    }
}

}  // anonymous namespace

// ---------------------------------------------------------------------------
// SimCommands implementation
// ---------------------------------------------------------------------------

SimCommands::SimCommands(SimHardware& hal) : _hal(hal)
{
}

std::vector<CommandDescriptor> SimCommands::getCommands() const
{
    void* ctx = &_hal;
    return {
        makeCmd("SIMSET",      parseSimSet,   handleSimSet, ctx, "badkey"), // set sim plant/error value by key
        makeSchemaCmd("SIMGET", &simGetSchema, handleSimGet, ctx, "badkey"), // get sim plant/error value by key (variadic)
    };
}
