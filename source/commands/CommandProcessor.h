#pragma once
#include "Protocol.h"
#include "CommandTypes.h"
#include "CommandQueue.h"

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

    // Attach a CommandQueue. When non-null, process() enqueues parsed commands
    // into the queue instead of dispatching them immediately.
    void setQueue(CommandQueue* q) { _queue = q; }

    // Returns true if a queue is currently attached (non-null). Used by tests
    // to assert the queue survives a Phase-3 CommandProcessor reassignment.
    bool hasQueue() const { return _queue != nullptr; }

    // Dispatch one item from q. Returns false if q is empty.
    // Calls the descriptor's handlerFn directly (not process()) to avoid
    // re-enqueuing when _queue is still set.
    bool dequeueOne(CommandQueue& q);

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

private:
    std::vector<CommandDescriptor> _cmds;
    ReplyFn                        _serialFn  = nullptr;
    void*                          _serialCtx = nullptr;
    CommandQueue*                  _queue     = nullptr;
    // Flags of the most recently successfully-parsed command's descriptor.
    // Set in dispatchTable() right after a successful parse (schema or
    // parseFn), before the enqueue-vs-immediate-dispatch branch, so it is
    // correct for both the production queue path and the sim/no-queue
    // fallback. See lastCommandResetsWatchdog().
    uint8_t                        _lastDispatchFlags = CMD_NONE;

    void dispatchTable(char** tokens, int ntok, KVPair* kvs, int nkv,
                       const char* corrId, ReplyFn replyFn, void* ctx);

    static int clampInt(int v, int lo, int hi);
};
