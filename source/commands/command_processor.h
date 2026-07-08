#pragma once
#include "protocol.h"
#include "command_types.h"
#include "messages/event.h"

/**
 * CommandProcessor — protocol v2 wire-protocol parser and dispatcher.
 *
 * Token grammar:
 *   <verb> [<arg>…] [key=value…] [#<id>]
 *
 * - Only the verb token is upper-cased; all other tokens preserve case.
 * - Optional trailing '#<digits>' token is extracted as a correlation id
 *   and echoed in every response for that command.
 * - 'key=value' tokens are parsed by parseKV().
 *
 * Response taxonomy (see replyOK / replyErr / replyEvt):
 *   OK  <verb> <body> [#id]
 *   ERR <code> <detail> [#id]
 *   EVT <name> <body>
 *   TLM …
 *   CFG …
 *   ID  …
 *
 * Usage (main.cpp):
 *   CommandProcessor cmd(robot.buildCommandTable(...));
 *   cmd.setSerialReply(serialFn, serialCtx);
 *   // in loop:
 *   cmd.process(lineBuf, replyFn, ctx);
 */
class CommandProcessor {
public:
    CommandProcessor() = default;
    explicit CommandProcessor(std::vector<CommandDescriptor> cmds);

    // Parse and dispatch one command line. line must be NUL-terminated.
    // Calls replyFn(msg, ctx) for each response line.
    void process(const char* line, ReplyFn replyFn, void* ctx);

    // Override the serial reply channel for ForceReply::SERIAL descriptors.
    // Optional — if unset, ForceReply::SERIAL uses the incoming replyFn/ctx.
    void setSerialReply(ReplyFn fn, void* ctx) { _serialFn = fn; _serialCtx = ctx; }

    // Returns true if the most recently parsed command's descriptor is
    // flagged CMD_MOTION_WATCHDOG (keepalive '+' or a motion verb). Callers
    // (LoopScheduler::runCommsIn/run_test, the sim's sim_command()) use this
    // to gate the motion-watchdog reset so ambient traffic (GET, SNAP, ...)
    // no longer silently keeps an open-ended motion command alive.
    // Reflects the last call to process(); false before any command has
    // been processed or after a line that matched no descriptor.
    bool lastCommandResetsWatchdog() const { return _lastDispatchFlags & CMD_MOTION_WATCHDOG; }

    // -------------------------------------------------------------------------
    // Static parse helpers — public so dependent tickets can call them
    // from within the dispatch table they add to this translation unit.
    // -------------------------------------------------------------------------

    // Maximum tokens in one command (verb + 23 args/kv + optional #id).
    static constexpr int MAX_TOKENS = 26;
    // Maximum key=value pairs in one command.
    static constexpr int MAX_KV     = 24;

    /**
     * parseTokens — tokenize line into at most maxTokens whitespace-delimited tokens.
     *
     * Copies line into workBuf (which must be at least strlen(line)+1 bytes).
     * Tokens point into workBuf. Upper-cases tokens[0] (the verb) in place.
     * If the last token starts with '#' followed by at least one digit, it is
     * extracted into corr_id (NUL-terminated, max corrIdSize-1 digits) and
     * excluded from the returned token list.
     *
     * Returns the count of tokens (NOT counting the corr_id token).
     */
    static int parseTokens(const char* line, char* workBuf, int workBufSize,
                           char** tokens, int maxTokens,
                           char* corr_id, int corrIdSize);

    /**
     * parseKV — scan tokens[1..ntokens-1] for key=value pairs.
     *
     * Tokens that contain '=' are split and appended to kvs[].
     * '=' without a key yields a "badarg" indicator: kv.key == nullptr.
     * '=' without a value yields: kv.value == "" (empty string).
     * Tokens without '=' are left as positional args (caller handles separately).
     *
     * Returns the count of kv pairs found.
     */
    static int parseKV(char** tokens, int ntokens, KVPair* kvs, int maxKV);

    /**
     * replyOK — write "OK <verb> <body> [#id]\n" into buf.
     *
     * body may be nullptr or "" (no body appended beyond verb).
     * id may be nullptr or "" (no #id suffix).
     */
    static void replyOK(char* buf, int size,
                        const char* verb, const char* body, const char* id,
                        ReplyFn fn, void* ctx);

    /**
     * replyErr — write "ERR <code> <detail> [#id]\n" into buf.
     *
     * detail may be nullptr or "".
     */
    static void replyErr(char* buf, int size,
                         const char* code, const char* detail, const char* id,
                         ReplyFn fn, void* ctx);

    /**
     * replyEvt — write "EVT <name> <body>\n" into buf.
     *
     * Async events carry no correlation id.
     * body may be nullptr or "".
     */
    static void replyEvt(char* buf, int size,
                         const char* name, const char* body,
                         ReplyFn fn, void* ctx);

    /**
     * emitEvent — the ONE place ALL "EVT ..." wire text is assembled
     * (090-004): the single wire-layer authority a subsystem or the main
     * loop hands a typed msg::Event to instead of ever snprintf-ing wire
     * text itself (source/messages/event.h's own doc comment; the
     * command(wire-inbound)/message(internal) boundary,
     * .claude/rules/naming-and-style.md sec 4). Built on replyEvt() above,
     * the same way replyOKf/replyErrf are built on replyOK/replyErr.
     *
     * ev.kind == GOAL_DONE: composes the wire name as "done <ev.verb>" and
     * the body as "[#<ev.corrId> ]reason=<ev.reason>" (corrId prefix omitted
     * when empty).
     *
     * ev.kind == NAMED: uses ev.name verbatim as the wire name; body is
     * "reason=<ev.reason>" when ev.reason is non-empty, otherwise no body
     * at all (e.g. dev_watchdog's bare "EVT dev_watchdog").
     */
    static void emitEvent(const msg::Event& ev, ReplyFn fn, void* ctx);

    /**
     * replyOKf — variadic form of replyOK.
     *
     * Formats body via vsnprintf into buf[0..size-1], then calls replyOK.
     * No heap allocation; safe under -fno-exceptions/-fno-rtti.
     * fmt/... follow printf conventions; the formatted result becomes the body.
     */
    static void replyOKf(char* buf, int size,
                         const char* verb, const char* id,
                         ReplyFn fn, void* ctx,
                         const char* fmt, ...)
        __attribute__((format(printf, 7, 8)));

    /**
     * replyErrf — variadic form of replyErr.
     *
     * Formats detail via vsnprintf into buf[0..size-1], then calls replyErr.
     * No heap allocation; safe under -fno-exceptions/-fno-rtti.
     */
    static void replyErrf(char* buf, int size,
                          const char* code, const char* id,
                          ReplyFn fn, void* ctx,
                          const char* fmt, ...)
        __attribute__((format(printf, 7, 8)));

    /**
     * listVerbs — write every registered descriptor's prefix into
     * buf[0..size-1], space-separated (e.g. "PING VER HELP ... DEV M ...").
     *
     * The sole read path onto the registered command table from outside
     * this class — keeps _cmds private (088-003, Decision 2). HELP's
     * handler is the only caller today, via CommandRouter::listVerbs().
     *
     * Returns the length written (buffer-writing convention, matching
     * replyOK/replyErr above). Truncates silently if buf is too small.
     */
    int listVerbs(char* buf, int size) const;

private:
    std::vector<CommandDescriptor> _cmds;
    ReplyFn                        _serialFn  = nullptr;
    void*                          _serialCtx = nullptr;
    // Flags of the most recently successfully-parsed command's descriptor.
    // Set in dispatchTable() right after a successful parse (schema or
    // parseFn), before handlerFn is called. See lastCommandResetsWatchdog().
    uint8_t                        _lastDispatchFlags = CMD_NONE;

    void dispatchTable(char** tokens, int ntok, KVPair* kvs, int nkv,
                       const char* corrId, ReplyFn replyFn, void* ctx);

    static int clampInt(int v, int lo, int hi);
};
