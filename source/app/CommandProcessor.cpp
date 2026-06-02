// CommandProcessor.cpp — protocol v2 wire-protocol parser and dispatcher.
//
// Sprint 009, Ticket 002: v2 tokenizer, verb-only uppercasing, #id
// correlation, OK/ERR/EVT/TLM/CFG/ID response taxonomy.
// Legacy packed parsing (parseSignedArgs, K*, S+/T+/D+, etc.) removed.
// Announcer removed; HELLO returns ERR unknown.

#include "CommandProcessor.h"
#include "Robot.h"
#include "MicroBitDevice.h"
#include "OtosSensor.h"
#include "LineSensor.h"
#include "ColorSensor.h"
#include "Servo.h"
#include "PortIO.h"
#include "MotorController.h"
#include "Odometry.h"
#include "DriveController.h"
#include "Config.h"
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cctype>

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

CommandProcessor::CommandProcessor(Robot& robot)
    : _robot(robot)
{
}

// ---------------------------------------------------------------------------
// Static helpers
// ---------------------------------------------------------------------------

int CommandProcessor::clampInt(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// ---------------------------------------------------------------------------
// parseTokens
// ---------------------------------------------------------------------------

int CommandProcessor::parseTokens(const char* line, char* workBuf, int workBufSize,
                                  char** tokens, int maxTokens,
                                  char* corr_id, int corrIdSize)
{
    // Copy line into workBuf, trimming leading/trailing whitespace.
    int srcLen = 0;
    const char* p = line;
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') ++p;

    for (const char* q = p; *q != '\0' && srcLen < workBufSize - 1; ++q, ++srcLen) {
        workBuf[srcLen] = *q;
    }
    // Trim trailing whitespace.
    while (srcLen > 0 &&
           (workBuf[srcLen - 1] == ' ' || workBuf[srcLen - 1] == '\t' ||
            workBuf[srcLen - 1] == '\r' || workBuf[srcLen - 1] == '\n')) {
        --srcLen;
    }
    workBuf[srcLen] = '\0';

    if (corr_id && corrIdSize > 0) corr_id[0] = '\0';

    if (srcLen == 0) return 0;

    // Tokenize by splitting on whitespace in place.
    int count = 0;
    char* cur  = workBuf;
    char* end  = workBuf + srcLen;

    while (cur < end && count < maxTokens) {
        // Skip leading whitespace.
        while (cur < end && (*cur == ' ' || *cur == '\t')) ++cur;
        if (cur >= end) break;

        tokens[count++] = cur;

        // Advance to next whitespace or end.
        while (cur < end && *cur != ' ' && *cur != '\t') ++cur;
        if (cur < end) {
            *cur = '\0';
            ++cur;
        }
    }

    // Upper-case the verb (tokens[0]).
    if (count > 0) {
        for (char* c = tokens[0]; *c != '\0'; ++c) {
            *c = (char)toupper((unsigned char)*c);
        }
    }

    // Check if the last token is a correlation id: '#' followed by digits only.
    if (count > 0 && corr_id && corrIdSize > 1) {
        const char* last = tokens[count - 1];
        if (last[0] == '#') {
            const char* d = last + 1;
            bool allDigits = (*d != '\0');  // must have at least one digit
            while (*d != '\0' && allDigits) {
                if (*d < '0' || *d > '9') allDigits = false;
                ++d;
            }
            if (allDigits) {
                // Extract digits into corr_id (without the '#').
                int len = (int)(d - (last + 1));
                if (len >= corrIdSize) len = corrIdSize - 1;
                memcpy(corr_id, last + 1, (size_t)len);
                corr_id[len] = '\0';
                --count;  // remove the #id token from the list
            }
        }
    }

    return count;
}

// ---------------------------------------------------------------------------
// parseKV
// ---------------------------------------------------------------------------

int CommandProcessor::parseKV(char** tokens, int ntokens, KVPair* kvs, int maxKV)
{
    int kvCount = 0;
    // Skip tokens[0] (verb) and start from index 1.
    for (int i = 1; i < ntokens && kvCount < maxKV; ++i) {
        char* eq = strchr(tokens[i], '=');
        if (!eq) continue;  // positional arg, not kv

        KVPair kv;
        if (eq == tokens[i]) {
            // '=' at the start: no key.
            kv.key   = nullptr;
            kv.value = eq + 1;
        } else {
            *eq      = '\0';
            kv.key   = tokens[i];
            kv.value = eq + 1;
        }
        kvs[kvCount++] = kv;
    }
    return kvCount;
}

// ---------------------------------------------------------------------------
// Reply builders
// ---------------------------------------------------------------------------

void CommandProcessor::replyOK(char* buf, int size,
                               const char* verb, const char* body, const char* id,
                               ReplyFn fn, void* ctx)
{
    if (body && body[0] != '\0') {
        if (id && id[0] != '\0') {
            snprintf(buf, (size_t)size, "OK %s %s #%s", verb, body, id);
        } else {
            snprintf(buf, (size_t)size, "OK %s %s", verb, body);
        }
    } else {
        if (id && id[0] != '\0') {
            snprintf(buf, (size_t)size, "OK %s #%s", verb, id);
        } else {
            snprintf(buf, (size_t)size, "OK %s", verb);
        }
    }
    fn(buf, ctx);
}

void CommandProcessor::replyErr(char* buf, int size,
                                const char* code, const char* detail, const char* id,
                                ReplyFn fn, void* ctx)
{
    if (detail && detail[0] != '\0') {
        if (id && id[0] != '\0') {
            snprintf(buf, (size_t)size, "ERR %s %s #%s", code, detail, id);
        } else {
            snprintf(buf, (size_t)size, "ERR %s %s", code, detail);
        }
    } else {
        if (id && id[0] != '\0') {
            snprintf(buf, (size_t)size, "ERR %s #%s", code, id);
        } else {
            snprintf(buf, (size_t)size, "ERR %s", code);
        }
    }
    fn(buf, ctx);
}

void CommandProcessor::replyEvt(char* buf, int size,
                                const char* name, const char* body,
                                ReplyFn fn, void* ctx)
{
    if (body && body[0] != '\0') {
        snprintf(buf, (size_t)size, "EVT %s %s", name, body);
    } else {
        snprintf(buf, (size_t)size, "EVT %s", name);
    }
    fn(buf, ctx);
}

// ---------------------------------------------------------------------------
// process — v2 command dispatch
// ---------------------------------------------------------------------------

void CommandProcessor::process(const char* line, ReplyFn replyFn, void* ctx)
{
    // Working buffer for tokenization. parseTokens() copies into this.
    char workBuf[512];
    char* tokens[MAX_TOKENS];
    char  corr_id[16];

    int ntok = parseTokens(line, workBuf, sizeof(workBuf),
                           tokens, MAX_TOKENS,
                           corr_id, sizeof(corr_id));
    if (ntok == 0) return;

    const char* verb = tokens[0];

    // Reply buffer used by all handlers.
    char rbuf[520];

    // ── Check for any bad kv token (key missing) before dispatching ──────────
    // Scan for kv tokens with missing key; these are always badarg.
    KVPair kvs[MAX_KV];
    int    nkv = parseKV(tokens, ntok, kvs, MAX_KV);
    for (int i = 0; i < nkv; ++i) {
        if (kvs[i].key == nullptr) {
            replyErr(rbuf, sizeof(rbuf), "badarg", "missing key", corr_id, replyFn, ctx);
            return;
        }
    }

    // ── Dispatch on verb ─────────────────────────────────────────────────────

    // ── PING ─────────────────────────────────────────────────────────────────
    // Reply: OK pong t=<robot_ms>  (clock-sync probe; t MUST be robot clock)
    if (strcmp(verb, "PING") == 0) {
        uint32_t t = _robot.systemTime();
        char body[32];
        snprintf(body, sizeof(body), "t=%lu", (unsigned long)t);
        replyOK(rbuf, sizeof(rbuf), "pong", body, corr_id, replyFn, ctx);
        return;
    }

    // ── ECHO ─────────────────────────────────────────────────────────────────
    // Reply: OK echo <payload>  — payload is everything after the verb token,
    // preserving case and spacing exactly (extracted from raw line, not tokens).
    if (strcmp(verb, "ECHO") == 0) {
        // Find the payload by skipping past the verb in the original line.
        // line is the original (immutable) parameter; workBuf is our copy.
        // We need to locate the content after "ECHO" in the raw line.
        const char* p = line;
        // Skip leading whitespace.
        while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') ++p;
        // Skip the verb token (any run of non-whitespace).
        while (*p != '\0' && *p != ' ' && *p != '\t') ++p;
        // Skip exactly one whitespace separator (if present).
        if (*p == ' ' || *p == '\t') ++p;
        // p now points at the payload (may be empty).

        // If a corr_id was found, we must strip the trailing "#<id>" from
        // the payload. The parseTokens() pass already removed it from tokens
        // but the raw line still has it. Strip it now.
        char payload[512];
        int plen = 0;
        const char* q = p;
        while (*q != '\0' && *q != '\r' && *q != '\n') {
            payload[plen++] = *q++;
            if (plen >= (int)(sizeof(payload) - 1)) break;
        }
        payload[plen] = '\0';

        // Trim trailing whitespace.
        while (plen > 0 && (payload[plen-1] == ' ' || payload[plen-1] == '\t')) {
            payload[--plen] = '\0';
        }

        // Strip trailing corr_id token ("#<digits>") if present.
        if (corr_id[0] != '\0') {
            // The corr_id token is "#" + corr_id digits at the very end.
            // Look for " #<corr_id>" suffix and remove it.
            char suffix[20];
            snprintf(suffix, sizeof(suffix), " #%s", corr_id);
            int slen = (int)strlen(suffix);
            if (plen >= slen && strcmp(payload + plen - slen, suffix) == 0) {
                plen -= slen;
                payload[plen] = '\0';
            }
        }

        replyOK(rbuf, sizeof(rbuf), "echo", payload, corr_id, replyFn, ctx);
        return;
    }

    // ── ID ───────────────────────────────────────────────────────────────────
    // Reply: ID model=Nezha2 name=<name> serial=<serial> fw=<ver> proto=2
    //        caps=<present subsystems>
    // Tag is "ID", not "OK" — this uses a custom snprintf path.
    if (strcmp(verb, "ID") == 0) {
        // Robot friendly name and serial number from CODAL.
        // microbit_friendly_name() returns a pointer to a static buffer (5 chars).
        // microbit_serial_number() returns a uint32_t hardware ID.
        const char* name   = microbit_friendly_name();
        uint32_t    serial = microbit_serial_number();

        // Build caps= string from runtime-present subsystems.
        char caps[64];
        caps[0] = '\0';
        bool first = true;
        auto addCap = [&](const char* cap) {
            if (!first) {
                int n = (int)strlen(caps);
                caps[n] = ','; caps[n+1] = '\0';
            }
            // strncat safe: caps is 64 bytes, max total caps length is ~50.
            int rem = (int)(sizeof(caps) - strlen(caps) - 1);
            if (rem > 0) strncat(caps, cap, (size_t)rem);
            first = false;
        };
        if (_robot.otos())        addCap("otos");
        if (_robot.lineSensor())  addCap("line");
        if (_robot.colorSensor()) addCap("color");
        if (_robot.servo())       addCap("gripper");
        // portio is always present.
        addCap("portio");

        if (corr_id[0] != '\0') {
            snprintf(rbuf, sizeof(rbuf),
                     "ID model=Nezha2 name=%s serial=%lu fw=%s proto=%d caps=%s #%s",
                     name, (unsigned long)serial, FIRMWARE_VERSION, PROTO_VERSION,
                     caps, corr_id);
        } else {
            snprintf(rbuf, sizeof(rbuf),
                     "ID model=Nezha2 name=%s serial=%lu fw=%s proto=%d caps=%s",
                     name, (unsigned long)serial, FIRMWARE_VERSION, PROTO_VERSION,
                     caps);
        }
        replyFn(rbuf, ctx);
        return;
    }

    // ── VER ──────────────────────────────────────────────────────────────────
    // Reply: OK ver fw=<ver> proto=2
    if (strcmp(verb, "VER") == 0) {
        char body[64];
        snprintf(body, sizeof(body), "fw=%s proto=%d", FIRMWARE_VERSION, PROTO_VERSION);
        replyOK(rbuf, sizeof(rbuf), "ver", body, corr_id, replyFn, ctx);
        return;
    }

    // ── HELP ─────────────────────────────────────────────────────────────────
    // Reply: OK help <verb list>
    if (strcmp(verb, "HELP") == 0) {
        replyOK(rbuf, sizeof(rbuf), "help",
                "PING ECHO ID VER HELP SET GET STREAM SNAP S T D G STOP GRIP ZERO",
                corr_id, replyFn, ctx);
        return;
    }

    // ── Fallback — unrecognized verb ─────────────────────────────────────────
    replyErr(rbuf, sizeof(rbuf), "unknown", verb, corr_id, replyFn, ctx);
}
