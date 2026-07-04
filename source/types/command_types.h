#pragma once
#include <stdint.h>
#include <vector>
#include "protocol.h"
#include "arg_schema.h"

// ---------------------------------------------------------------------------
// command_types.h — foundational types for the registration-based command
// dispatch system.
// Constraint: -std=c++11 -fno-exceptions -fno-rtti
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// ArgType — discriminator for a tagged argument
// ---------------------------------------------------------------------------
enum class ArgType : uint8_t { INT, FLOAT, STR };

// ---------------------------------------------------------------------------
// Argument — tagged union covering int, float, and short string values.
// sval[32] covers all practical protocol tokens.
// ---------------------------------------------------------------------------
struct Argument {
    ArgType type;
    union { int32_t ival; float fval; };
    char sval[32];
};

// ---------------------------------------------------------------------------
// ArgList — fixed-capacity argument list (stack-allocated in process()).
// ---------------------------------------------------------------------------
static constexpr int MAX_ARGS = 10;
struct ArgList {
    Argument args[MAX_ARGS];
    int      count;
    // suppliedCount — number of tokens actually supplied by the caller for
    // the positional slots (<= count). Lets a handler distinguish "token
    // omitted" from "token supplied, value happens to equal the default."
    // Set by parseSchema() and every hand-rolled ParseFn; see arg_parse.cpp.
    int      suppliedCount;
};

// ---------------------------------------------------------------------------
// ParseError — error payload returned when a ParseFn rejects its input.
// ---------------------------------------------------------------------------
struct ParseError {
    const char* code;
    const char* detail;
};

// ---------------------------------------------------------------------------
// ParseResult — discriminated union returned by every ParseFn.
// Both union members are trivially constructible (no user constructors,
// plain data), which satisfies the C++11 unrestricted-union constraint.
// ---------------------------------------------------------------------------
struct ParseResult {
    bool ok;
    union {
        ArgList   args;
        ParseError err;
    };
};

// ---------------------------------------------------------------------------
// ParseFn — signature for a command's argument parser.
//   tokens  — array of ntokens raw token strings (verb already stripped)
//   kvs     — parsed key=value pairs from the same token stream
// ---------------------------------------------------------------------------
typedef ParseResult (*ParseFn)(const char* const* tokens, int ntokens,
                               const KVPair* kvs, int nkv);

// ---------------------------------------------------------------------------
// HandlerFn — signature for a command's handler.
//   args       — parsed argument list (from ParseFn, or empty if parseFn==nullptr)
//   corrId     — correlation id string (may be "" but never nullptr)
//   replyFn    — reply callback; call once per response line
//   replyCtx   — opaque context forwarded to replyFn
//   handlerCtx — subsystem context pointer (cast to subsystem type inside handler)
// ---------------------------------------------------------------------------
typedef void (*HandlerFn)(const ArgList& args, const char* corrId,
                          ReplyFn replyFn, void* replyCtx,
                          void* handlerCtx);

// ---------------------------------------------------------------------------
// ForceReply — controls reply channel override.
//   NONE   — reply on the channel the command arrived on
//   SERIAL — always reply via serial (used for DBG/I2C commands)
// ---------------------------------------------------------------------------
enum class ForceReply : uint8_t { NONE, SERIAL };

// ---------------------------------------------------------------------------
// Command flags — bitmask values for CommandDescriptor::flags.
//   CMD_NONE            — command operates only on cached/config state
//   CMD_ACCESS_HARDWARE — command reads or writes physical hardware
//                         (motors, sensors, GPIO, I2C, servo)
//   CMD_MOTION_WATCHDOG — command legitimately resets the motion watchdog
//                         (keepalive '+' and motion verbs only — see
//                         CommandProcessor::lastCommandResetsWatchdog())
// ---------------------------------------------------------------------------
static constexpr uint8_t CMD_NONE            = 0;
static constexpr uint8_t CMD_ACCESS_HARDWARE = 1;
static constexpr uint8_t CMD_MOTION_WATCHDOG = 2;

// ---------------------------------------------------------------------------
// CommandDescriptor — one entry in the command dispatch table.
// 28 bytes per entry (flags field + 3 bytes pad to 4-byte alignment);
// ~42 commands ≈ 1176 bytes of static BSS.
//
//   prefix      — command prefix string: "S", "DBG LOOP", "DBG LOOP RESET", …
//   parseFn     — nullptr means pass raw tokens directly to handlerFn;
//                 also nullptr when schema != nullptr (schema path is used instead)
//   handlerFn   — command implementation
//   handlerCtx  — subsystem instance pointer; cast inside handlerFn
//   errFmt      — ERR code emitted when parseFn or schema parse returns ok=false
//   forceReply  — channel override (see ForceReply)
//   flags       — CMD_NONE or CMD_ACCESS_HARDWARE
//   schema      — when non-null, dispatchTable calls parseSchema() instead of
//                 parseFn; existing commands leave this nullptr (no behaviour change)
// ---------------------------------------------------------------------------
struct CommandDescriptor {
    const char*     prefix;
    ParseFn         parseFn;
    HandlerFn       handlerFn;
    void*           handlerCtx;
    const char*     errFmt;
    ForceReply      forceReply;
    uint8_t         flags;       // CMD_NONE or CMD_ACCESS_HARDWARE
    const ArgSchema* schema;     // nullptr → use parseFn (legacy path)
};

// ---------------------------------------------------------------------------
// Commandable — interface for subsystems that register their own commands.
// ---------------------------------------------------------------------------
class Commandable {
public:
    virtual std::vector<CommandDescriptor> getCommands() const = 0;
    virtual ~Commandable() {}
};

// ---------------------------------------------------------------------------
// makeCmd — inline helper to construct a CommandDescriptor without relying
// on brace-initialised aggregate literals (which are verbose in C++11).
// schema is always nullptr here; existing call sites are unchanged.
// ---------------------------------------------------------------------------
inline CommandDescriptor makeCmd(const char* prefix, ParseFn parseFn,
                                 HandlerFn handlerFn, void* ctx,
                                 const char* errFmt     = "badarg",
                                 ForceReply  forceReply = ForceReply::NONE,
                                 uint8_t     flags      = CMD_NONE) {
    CommandDescriptor d;
    d.prefix      = prefix;
    d.parseFn     = parseFn;
    d.handlerFn   = handlerFn;
    d.handlerCtx  = ctx;
    d.errFmt      = errFmt;
    d.forceReply  = forceReply;
    d.flags       = flags;
    d.schema      = nullptr;
    return d;
}

// ---------------------------------------------------------------------------
// makeSchemaCmd — inline helper to construct a schema-driven CommandDescriptor.
// parseFn is set to nullptr; dispatch routes through parseSchema() instead.
// Call sites that migrate a command from makeCmd use this factory.
// ---------------------------------------------------------------------------
inline CommandDescriptor makeSchemaCmd(const char* prefix, const ArgSchema* schema,
                                       HandlerFn handlerFn, void* ctx,
                                       const char* errFmt     = "badarg",
                                       ForceReply  forceReply = ForceReply::NONE,
                                       uint8_t     flags      = CMD_NONE) {
    CommandDescriptor d;
    d.prefix      = prefix;
    d.parseFn     = nullptr;
    d.handlerFn   = handlerFn;
    d.handlerCtx  = ctx;
    d.errFmt      = errFmt;
    d.forceReply  = forceReply;
    d.flags       = flags;
    d.schema      = schema;
    return d;
}

// ---------------------------------------------------------------------------
// ParsedCommand — a fully parsed command ready for dispatch or queuing.
//   desc      — points to the registered CommandDescriptor
//   args      — parsed argument list (from parseFn, or empty)
//   replyFn   — reply callback to call for each response line
//   replyCtx  — opaque context forwarded to replyFn
//   corrId    — correlation id string (up to 15 chars + NUL, N14 fix)
//
// N14: widened from char[8] to char[16] to match MotionCommand._corrId[16]
//      and the tokenizer/TargetState corrId fields.  This prevents silent
//      truncation of >7-char correlation ids (e.g. ms-timestamp strings)
//      on the queue path.  ParsedCommand is stack-allocated per dispatch —
//      +8 bytes per call, no heap impact.
// ---------------------------------------------------------------------------
struct ParsedCommand {
    const CommandDescriptor* desc;
    ArgList  args;
    ReplyFn  replyFn;
    void*    replyCtx;
    char     corrId[16];
};
