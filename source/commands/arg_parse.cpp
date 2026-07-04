// arg_parse.cpp — implementation of the schema-driven argument parser.
//
// parseSchema() drives all three parse shapes defined by ArgSchema:
//   variadic, positional (with optional range check and packKv), and no-arg.
//
// Behaviour-preservation invariants (ticket 051-002):
//   - Variadic: ival=0, fval=0.0f before sval copy; count capped at MAX_ARGS;
//     sval capped at sizeof(Argument::sval)-1 == 31 chars + NUL.
//   - Positional INT unranged: stored via atoi with no range check (allows silent
//     truncation to int16 in the handler, preserving OV/SI behaviour).
//   - Positional INT ranged: returns {ok=false, err={nullptr, def.name}} on fail.
//   - minTokens: returns {ok=false, err={nullptr, nullptr}} when ntokens < min.
//   - packKv: appends matching KV value as trailing STR at args[count]; count
//     unchanged when key is absent (mirrors packSensorArg byte-for-byte).
//
// Constraint: -std=c++11 -fno-exceptions -fno-rtti; no heap.

#include "commands/arg_parse.h"
#include <cstdlib>   // atoi, atof

// ---------------------------------------------------------------------------
// parseSchema
// ---------------------------------------------------------------------------
ParseResult parseSchema(const char* const* tokens, int ntokens,
                        const KVPair* kvs, int nkv,
                        const ArgSchema& schema)
{
    ParseResult res;

    // ── minTokens guard ───────────────────────────────────────────────────────
    if (ntokens < schema.minTokens) {
        res.ok          = false;
        res.err.code    = nullptr;
        res.err.detail  = nullptr;
        return res;
    }

    // ── Variadic path ─────────────────────────────────────────────────────────
    if (schema.variadic) {
        res.ok = true;
        int n = (ntokens < MAX_ARGS) ? ntokens : MAX_ARGS;
        res.args.count = n;
        res.args.suppliedCount = n;
        for (int i = 0; i < n; ++i) {
            res.args.args[i].type = ArgType::STR;
            res.args.args[i].ival = 0;
            res.args.args[i].fval = 0.0f;
            int j = 0;
            if (tokens[i]) {
                while (tokens[i][j] != '\0' &&
                       j < (int)(sizeof(res.args.args[i].sval) - 1)) {
                    res.args.args[i].sval[j] = tokens[i][j];
                    ++j;
                }
            }
            res.args.args[i].sval[j] = '\0';
        }
        return res;
    }

    // ── Positional path (includes no-arg when ndefs==0) ───────────────────────
    res.ok = true;
    int count = 0;

    for (int i = 0; i < schema.ndefs; ++i) {
        const ArgDef& def = schema.defs[i];
        const char* tok = (i < ntokens) ? tokens[i] : nullptr;

        switch (def.kind) {
            case ArgKind::INT: {
                int v = tok ? atoi(tok) : 0;
                if (def.ranged && (v < def.lo || v > def.hi)) {
                    res.ok          = false;
                    res.err.code    = nullptr;
                    res.err.detail  = def.name;
                    return res;
                }
                argInt(res.args.args[count], v);
                ++count;
                break;
            }
            case ArgKind::FLOAT: {
                float v = tok ? (float)atof(tok) : 0.0f;
                argFloat(res.args.args[count], v);
                ++count;
                break;
            }
            case ArgKind::STR: {
                argStr(res.args.args[count], tok ? tok : "");
                ++count;
                break;
            }
        }
    }

    res.args.count = count;
    // suppliedCount: how many of the positional slots actually had a token
    // in the incoming call (as opposed to being filled with a default value
    // because the caller omitted them). packKv's trailing append below (if
    // any) is not a positional token, so it is deliberately excluded here.
    res.args.suppliedCount = (ntokens < schema.ndefs) ? ntokens : schema.ndefs;

    // ── packKv: append matching KV value as trailing STR ──────────────────────
    // Mirrors packSensorArg byte-for-byte: scans kvs for schema.packKv;
    // if found, copies value into args[count] as STR using bounded copy,
    // then increments count.  No change when key is absent.
    if (schema.packKv != nullptr) {
        for (int i = 0; i < nkv; ++i) {
            if (kvs[i].key && strcmp(kvs[i].key, schema.packKv) == 0) {
                Argument& a = res.args.args[count];
                a.type = ArgType::STR;
                a.ival = 0;
                a.fval = 0.0f;
                int slen = 0;
                const char* src = kvs[i].value ? kvs[i].value : "";
                while (*src && slen < (int)(sizeof(a.sval) - 1))
                    a.sval[slen++] = *src++;
                a.sval[slen] = '\0';
                ++count;
                res.args.count = count;
                break;
            }
        }
    }

    return res;
}
