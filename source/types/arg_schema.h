#pragma once
#include <stdint.h>

// ---------------------------------------------------------------------------
// arg_schema.h — declarative argument-schema types for the schema-driven
// command parser layer.
//
// Constraint: -std=c++11 -fno-exceptions -fno-rtti
// No heap allocation; all data is static/const.
//
// Placed in source/types/ (alongside command_types.h) so that command_types.h
// can include it without creating a circular dependency (commands/ -> types/
// -> commands/ would be circular; this direction is types/ -> types/).
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// ArgKind — discriminator for a declared argument slot.
// Mirrors ArgType in command_types.h but belongs to the schema layer, not the
// runtime tagged-union layer.
// ---------------------------------------------------------------------------
// Deliberately separate from CommandTypes::ArgType (the runtime tagged-union
// layer): both enums live in source/types/ so nothing prevents merging them,
// but doing so would couple this declarative schema layer to the runtime
// dispatch layer's type for a three-value enum that costs nothing to
// duplicate and buys nothing by sharing (070-002 Decision 5).
enum class ArgKind : uint8_t { INT, FLOAT, STR };

// ---------------------------------------------------------------------------
// ArgDef — declaration of one positional argument slot.
//
//   name    — argument name string; used in ERR detail replies to identify
//             which argument failed its range check.
//   kind    — expected type: INT, FLOAT, or STR.
//   ranged  — when false, no range check is applied to this argument; an INT
//             value is accepted as-is via atoi with silent truncation — this
//             preserves the existing OV/SI behaviour where the protocol allows
//             any integer and the handler casts to int16_t internally.
//             When true, lo/hi define the inclusive [lo, hi] range; a value
//             outside the range causes parseSchema to return an ERR result.
//   lo, hi  — range bounds (inclusive); meaningful only when ranged == true.
// ---------------------------------------------------------------------------
struct ArgDef {
    const char* name;
    ArgKind     kind;
    bool        ranged;
    int32_t     lo, hi;
};

// ---------------------------------------------------------------------------
// ArgSchema — complete declarative description of a command's argument shape.
//
//   defs       — pointer to an array of ArgDef entries (may be nullptr when
//               ndefs == 0, i.e. a no-arg command).
//   ndefs      — number of entries in defs[].
//   minTokens  — minimum number of positional tokens required; reproduces
//               the ntokens < N badarg guard present in hand-written parsers.
//               Must be <= ndefs for non-variadic schemas.
//   variadic   — when true, tokens beyond ndefs are accepted as trailing STR
//               args; parseSchema appends them up to the MAX_ARGS cap.
//               When false, extra tokens beyond ndefs are silently ignored
//               (consistent with existing parser behaviour).
//   packKv     — nullable key name; when non-null, parseSchema looks up this
//               key in the KV pairs and appends its value as a trailing STR
//               arg after the positional args. This reproduces the
//               packSensorArg pattern used by T, D, and TURN, where a
//               sensor= KV is spliced into the argument list so the handler
//               does not need to call kvFind separately.
// ---------------------------------------------------------------------------
struct ArgSchema {
    const ArgDef* defs;
    int           ndefs;
    int           minTokens;
    bool          variadic;
    const char*   packKv;
};
