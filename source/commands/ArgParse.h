#pragma once
// ---------------------------------------------------------------------------
// ArgParse.h — schema-driven argument parser and inline helpers.
//
// Provides:
//   parseSchema()  — generic parser covering variadic, positional, and packKv
//                    shapes; declared here, implemented in ArgParse.cpp.
//   argStr/argInt/argFloat — ONE-TRUE bounded Argument initializers.
//   kvFind/kvInt/kvFloat/kvHas — KV lookup helpers.
//
// Constraint: -std=c++11 -fno-exceptions -fno-rtti; no heap; inline/stack-only.
// ---------------------------------------------------------------------------

#include "types/ArgSchema.h"
#include "types/CommandTypes.h"
#include "types/Protocol.h"  // KVPair

#include <cstring>  // strcmp

// ---------------------------------------------------------------------------
// parseSchema — generic argument parser driven by an ArgSchema.
//
// Handles three shapes:
//   No-arg:     ndefs==0 && !variadic  → returns empty ArgList.
//   Positional: !variadic              → parses ndefs tokens by ArgKind;
//               applies range check when def.ranged==true; appends packKv value
//               as trailing STR when schema.packKv!=nullptr and key is present.
//   Variadic:   variadic==true         → copies all tokens as STR args, capped
//               at MAX_ARGS; ival=0, fval=0.0f per arg; sval bounded to 31+NUL.
//
// minTokens guard: if ntokens < schema.minTokens returns
//   ParseResult{ok=false, err={nullptr,nullptr}}.
//
// Range check (positional INT, ranged==true): if value outside [lo,hi] returns
//   ParseResult{ok=false, err={nullptr, def.name}}.
// ---------------------------------------------------------------------------
ParseResult parseSchema(const char* const* tokens, int ntokens,
                        const KVPair* kvs, int nkv,
                        const ArgSchema& schema);

// ---------------------------------------------------------------------------
// argStr — initialise one Argument as STR with bounded sval copy.
//
// Copies at most sizeof(Argument::sval)-1 == 31 chars from src, then NUL-
// terminates.  Sets type=STR; leaves ival/fval union members untouched (they
// share storage with the union; callers that need ival/fval zero should use
// argInt / argFloat instead).
// ---------------------------------------------------------------------------
inline void argStr(Argument& a, const char* src)
{
    a.type = ArgType::STR;
    int j = 0;
    if (src) {
        while (src[j] != '\0' && j < (int)(sizeof(a.sval) - 1)) {
            a.sval[j] = src[j];
            ++j;
        }
    }
    a.sval[j] = '\0';
}

// ---------------------------------------------------------------------------
// argInt — initialise one Argument as INT.
// ---------------------------------------------------------------------------
inline void argInt(Argument& a, int32_t v)
{
    a.type    = ArgType::INT;
    a.ival    = v;
    a.sval[0] = '\0';
}

// ---------------------------------------------------------------------------
// argFloat — initialise one Argument as FLOAT.
// ---------------------------------------------------------------------------
inline void argFloat(Argument& a, float v)
{
    a.type    = ArgType::FLOAT;
    a.fval    = v;
    a.sval[0] = '\0';
}

// ---------------------------------------------------------------------------
// kvFind — linear scan of kvs[0..nkv-1] for the first entry whose key equals
// the given key string.  Returns a pointer to the matching KVPair or nullptr.
// ---------------------------------------------------------------------------
inline const KVPair* kvFind(const KVPair* kvs, int nkv, const char* key)
{
    if (!kvs || !key) return nullptr;
    for (int i = 0; i < nkv; ++i) {
        if (kvs[i].key && strcmp(kvs[i].key, key) == 0)
            return &kvs[i];
    }
    return nullptr;
}

// ---------------------------------------------------------------------------
// kvInt — look up a KV pair and return its value as int (atoi), or def if
// the key is not present.
// ---------------------------------------------------------------------------
inline int kvInt(const KVPair* kvs, int nkv, const char* key, int def)
{
    const KVPair* kv = kvFind(kvs, nkv, key);
    if (!kv || !kv->value) return def;
    // atoi is available in <cstdlib>; callers must include it if they use kvInt
    // from a .cpp file.  Here we call it through the standard C linkage that
    // is always available in the translation unit including this header.
    extern int atoi(const char*);
    return atoi(kv->value);
}

// ---------------------------------------------------------------------------
// kvFloat — look up a KV pair and return its value as float (atof), or def.
// ---------------------------------------------------------------------------
inline float kvFloat(const KVPair* kvs, int nkv, const char* key, float def)
{
    const KVPair* kv = kvFind(kvs, nkv, key);
    if (!kv || !kv->value) return def;
    extern double atof(const char*);
    return (float)atof(kv->value);
}

// ---------------------------------------------------------------------------
// kvHas — return true iff a KV pair with the given key is present.
// ---------------------------------------------------------------------------
inline bool kvHas(const KVPair* kvs, int nkv, const char* key)
{
    return kvFind(kvs, nkv, key) != nullptr;
}
