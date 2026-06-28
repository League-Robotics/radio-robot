// ConfigRegistry.cpp — config key-to-field registry for the robot.
//
// Moved from CommandProcessor.cpp (Sprint 019, Ticket 002).
// kRegistry[] maps friendly key names to RobotConfig fields by byte offset.
// handleGet and handleSet implement the HandlerFn-compatible GET/SET wire
// protocol handlers; they are called from the CommandProcessor switch until
// the composable dispatch table is in place (T010/T011).

#include "ConfigRegistry.h"
#include "../commands/CommandProcessor.h"
#include "../control/MotorController.h"
#include <cstring>
#include <cstdio>
#include <cstdlib>

// ---------------------------------------------------------------------------
// Helper macros so the table stays readable.
// ---------------------------------------------------------------------------
#define CFG_F(k, field)  { k, CFG_FLOAT,        offsetof(RobotConfig, field) }
#define CFG_I(k, field)  { k, CFG_INT,           offsetof(RobotConfig, field) }
#define CFG_FI(k, field) { k, CFG_FLOAT_AS_INT,  offsetof(RobotConfig, field) }

const ConfigEntry kRegistry[] = {
    // Encoder calibration (mm per degree of motor rotation)
    CFG_F("ml",           mmPerDegL),
    CFG_F("mr",           mmPerDegR),
    // Feed-forward and motor scale factors
    CFG_F("kff",          kFF),
    CFG_F("klf",          kScaleLF),
    CFG_F("klb",          kScaleLB),
    CFG_F("krf",          kScaleRF),
    CFG_F("krb",          kScaleRB),
    // Slower-wheel adjustment
    CFG_F("adjThr",       kAdjThreshold),
    CFG_F("adjGain",      kAdjGain),
    // Geometry — stored as float, displayed as integer (mm)
    CFG_FI("tw",          trackwidthMm),
    // Velocity loop gains (Sprint 010).
    // C++ field names use flat camel-case; SET/GET key strings use dotted form.
    //   velKp  <-> "vel.kP"   velKi  <-> "vel.kI"   velKff <-> "vel.kFF"
    CFG_F("vel.kP",       velKp),
    CFG_F("vel.kI",       velKi),
    CFG_F("vel.kFF",      velKff),
    CFG_F("vel.iMax",     velIMax),        // integrator clamp (PWM%)
    CFG_F("vel.kAw",      velKaw),         // back-calc anti-windup gain (1/s)
    CFG_F("vel.filt",     velFiltAlpha),   // velocity EMA weight (smoothing)
    CFG_F("sync",         syncGain),       // cross-wheel ratio coupling gain
    // Velocity deadband and wheel speed ceiling (Sprint 010)
    CFG_F("minWheelMms",  minWheelMms),
    CFG_F("vWheelMax",    vWheelMax),
    CFG_F("steerHeadroom",steerHeadroom),
    // OTOS complementary fusion (Sprint 010, Ticket 006)
    CFG_F("alphaPos",     alphaPos),
    CFG_F("alphaYaw",     alphaYaw),
    CFG_F("otosGate",     otosGate),
    // Go-to tolerances — stored as float, displayed as integer (mm)
    // Legacy keys retained for backward compatibility.
    CFG_FI("turnThr",     turnThresholdMm),
    CFG_FI("doneTol",     doneTolMm),
    // Pose-control tunables (Sprint 011)
    CFG_F ("aMax",        aMax),
    CFG_F ("aDecel",      aDecel),
    CFG_FI("turnGate",    turnInPlaceGate),   // wire: integer degrees; MotionController converts to radians at use-site
    CFG_FI("arriveTol",   arriveTolMm),       // wire: integer mm
    // Body motion limits (Sprint 017 -- BodyVelocityController)
    CFG_F("vBodyMax",    vBodyMax),           // body forward speed ceiling, mm/s
    CFG_F("yawRateMax",  yawRateMax),         // yaw rate ceiling, deg/s
    CFG_F("yawAccMax",   yawAccMax),          // yaw acceleration limit, deg/s^2
    CFG_F("jMax",        jMax),               // linear jerk limit, mm/s^3 (0=trapezoid)
    CFG_F("yawJerkMax",  yawJerkMax),         // yaw jerk limit, deg/s^3   (0=trapezoid)
    // Timing and speed (int32_t fields)
    CFG_I("minSpeed",     minSpeedMms),
    CFG_I("sTimeout",     sTimeoutMs),
    CFG_I("tick",         tickMs),
    CFG_I("ctrlPeriod",   controlPeriodMs),
    CFG_I("tlmPeriod",    tlmPeriodMs),
    // Sensor lag budgets (ms) for the cooperative scheduler's low-priority tasks.
    // SET lag.* N updates cfg.lag*Ms; LoopScheduler syncs task periodMs live.
    CFG_I("lag.otos",     lagOtosMs),
    CFG_I("lag.line",     lagLineMs),
    CFG_I("lag.color",    lagColorMs),
    CFG_I("lag.ports",    lagPortsMs),
    // OTOS calibration and turn asymmetry (Sprint 012)
    CFG_F("otosLinSc",    otosLinearScale),
    CFG_F("otosAngSc",    otosAngularScale),
    CFG_F("rotGainPos",   rotationGainPos),
    CFG_F("rotGainNeg",   rotationGainNeg),
    CFG_F("rotOffPos",    rotationOffsetDeg),
    CFG_F("rotOffNeg",    rotationOffsetDegNeg),
    CFG_F("rotSlip",      rotationalSlip),
    CFG_F("odomOffX",     odomOffX),
    CFG_F("odomOffY",     odomOffY),
    CFG_F("odomYaw",      odomYawDeg),
    // EKF heading fusion noise (sprint 024-004)
    CFG_F("ekfRHead",     ekfROtosTheta),
};

#undef CFG_F
#undef CFG_I
#undef CFG_FI

const int kRegistryCount = (int)(sizeof(kRegistry) / sizeof(kRegistry[0]));

// ---------------------------------------------------------------------------
// appendKeyValue — append one key=value pair to a string buffer.
// Returns the number of characters written (not counting the NUL).
// ---------------------------------------------------------------------------

static int appendKeyValue(char* buf, int remaining, const ConfigEntry& entry,
                          const RobotConfig& cfg)
{
    if (remaining <= 1) return 0;

    const char* base = reinterpret_cast<const char*>(&cfg);
    int written = 0;

    switch (entry.type) {
    case CFG_FLOAT: {
        const float v = *reinterpret_cast<const float*>(base + entry.offset);
        written = snprintf(buf, (size_t)remaining, "%s=%.3f", entry.key, (double)v);
        break;
    }
    case CFG_INT: {
        const int32_t v = *reinterpret_cast<const int32_t*>(base + entry.offset);
        written = snprintf(buf, (size_t)remaining, "%s=%d", entry.key, (int)v);
        break;
    }
    case CFG_FLOAT_AS_INT: {
        const float v = *reinterpret_cast<const float*>(base + entry.offset);
        written = snprintf(buf, (size_t)remaining, "%s=%d", entry.key, (int)v);
        break;
    }
    }

    if (written < 0 || written >= remaining) return remaining - 1;
    return written;
}

// ---------------------------------------------------------------------------
// handleGet — HandlerFn-compatible GET handler.
//
// args.args[0..args.count-1].sval carries the requested key names.
// An empty args list (count == 0) means dump all keys.
// handlerCtx is cast to CfgCtx*.
//
// N12 (030-010): Full GET dump is ~800 bytes but CODAL's serial TX buffer is
// only 255 bytes (SerialPort.cpp:17).  sendReliable cannot make room for a
// line longer than the buffer — it spins 5 ms then hands the string to ASYNC
// which drops the overflow.  Buffer math: 58 keys × ~14 bytes/key ≈ 805 bytes,
// exceeding the 255-byte limit by ~550 bytes.  BENCH CONFIRM NEEDED: verify
// on hardware that chunked CFG lines arrive complete before removing this note.
//
// Fix: for bare GET (all-keys dump), emit multiple CFG lines each ≤ 200 bytes
// so every line fits the 255-byte TX buffer.  Named-key requests fit a single
// line.  The host-side get_config() already accumulates multiple CFG lines
// via result.update(r.kv) (protocol.py:NezhaProtocol.get_config).
//
// Emits ≥1 CFG response lines. For each unknown key, also emits ERR badkey.
// ---------------------------------------------------------------------------

// Maximum content bytes per CFG line for the all-keys dump.  Set to 200 to
// stay well under the 255-byte CODAL serial TX buffer.
static const int kCfgChunkMax = 200;

void handleGet(const ArgList& args, const char* corrId,
               ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    const CfgCtx* ctx = reinterpret_cast<const CfgCtx*>(handlerCtx);
    const RobotConfig& cfg = *ctx->cfg;

    // replyCtx is the opaque context forwarded to replyFn (used for ERR replies).
    // For ERR replies we need a temporary buffer.
    char rbuf[128];

    bool anyKey = (args.count == 0);  // no args -> dump all

    if (anyKey) {
        // N12: chunk the full dump into multiple CFG lines, each ≤ kCfgChunkMax
        // content bytes, so each transmission fits CODAL's 255-byte TX buffer.
        // The host accumulates multiple CFG lines via result.update().
        //
        // Buffer is sized for one chunk: "CFG" prefix (3) + up to kCfgChunkMax
        // content bytes + " #corrId" (≤ 10) + NUL.  256 bytes is sufficient.
        char line[256];
        int pos = 0;
        int rem = (int)sizeof(line);

        // Start the first CFG chunk.
        int n = snprintf(line + pos, (size_t)rem, "CFG");
        if (n > 0 && n < rem) { pos += n; rem -= n; }

        for (int i = 0; i < kRegistryCount; ++i) {
            // Probe: how many bytes would this key=val entry add?
            char probe[48];
            int wProbe = appendKeyValue(probe, (int)sizeof(probe) - 1, kRegistry[i], cfg);
            int entrySize = 1 + wProbe;  // 1 for the leading space

            // If adding this entry would push the content region over the chunk
            // limit, flush the current line and start a fresh CFG chunk.
            // Content bytes = pos - 3 ("CFG" prefix).
            int contentBytes = pos - 3;
            if (contentBytes > 0 && contentBytes + entrySize > kCfgChunkMax) {
                // Flush: append corrId if present, then emit.
                if (corrId && corrId[0] != '\0' && rem > 3) {
                    int w = snprintf(line + pos, (size_t)rem, " #%s", corrId);
                    if (w > 0 && w < rem) { pos += w; rem -= w; }
                }
                line[pos] = '\0';
                replyFn(line, replyCtx);

                // Start fresh chunk.
                pos = 0; rem = (int)sizeof(line);
                n = snprintf(line + pos, (size_t)rem, "CFG");
                if (n > 0 && n < rem) { pos += n; rem -= n; }
            }

            // Append this entry.
            if (rem > 2) {
                line[pos++] = ' '; --rem;
                int w = appendKeyValue(line + pos, rem, kRegistry[i], cfg);
                pos += w; rem -= w;
            }
        }

        // Flush the final (possibly only) chunk.
        if (corrId && corrId[0] != '\0' && rem > 3) {
            int w = snprintf(line + pos, (size_t)rem, " #%s", corrId);
            if (w > 0 && w < rem) { pos += w; rem -= w; }
        }
        line[pos] = '\0';
        replyFn(line, replyCtx);

    } else {
        // Named-key request: all requested keys fit in one CFG line (bounded by
        // the number of keys the caller can specify in a single command, and any
        // unknown key gets its own ERR).
        char line[768];
        int pos = 0;
        int rem = (int)sizeof(line);

        int n = snprintf(line + pos, (size_t)rem, "CFG");
        if (n > 0 && n < rem) { pos += n; rem -= n; }

        for (int t = 0; t < args.count && rem > 2; ++t) {
            const char* reqKey = args.args[t].sval;
            bool found = false;
            for (int i = 0; i < kRegistryCount; ++i) {
                if (strcmp(kRegistry[i].key, reqKey) == 0) {
                    line[pos++] = ' '; --rem;
                    int w = appendKeyValue(line + pos, rem, kRegistry[i], cfg);
                    pos += w; rem -= w;
                    found = true;
                    break;
                }
            }
            if (!found) {
                // Unknown key in GET -- reply with ERR for that key but continue.
                CommandProcessor::replyErr(rbuf, (int)sizeof(rbuf),
                                           "badkey", reqKey, corrId,
                                           replyFn, replyCtx);
            }
        }

        // Append correlation id if present.
        if (corrId && corrId[0] != '\0' && rem > 3) {
            int w = snprintf(line + pos, (size_t)rem, " #%s", corrId);
            if (w > 0 && w < rem) { pos += w; rem -= w; }
        }

        line[pos] = '\0';
        replyFn(line, replyCtx);
    }
}

// ---------------------------------------------------------------------------
// validateConfig — check RobotConfig invariants before atomic commit.
//
// Returns true if the candidate config is valid.  On failure, sets *badKey to
// a short description of the first failing invariant (key name or "key=value"
// form) and returns false.  Only checks invariants whose violation causes a
// known runtime failure:
//
//   tw > 0          — trackwidthMm divides in odometry arc/heading; zero →
//                     division by zero.
//   ctrlPeriod > 0  — controlPeriodMs is cast to uint32 sleep; zero or
//                     negative wraps to a huge value, starving the control
//                     fiber.
//   vWheelMax > steerHeadroom  — effective ceiling = vWheelMax-steerHeadroom;
//                     when ≤ 0 the saturation ceiling goes negative, clamping
//                     output to a negative value and inverting the wheel.
//   rotationalSlip in {0} ∪ [0.5, 1.0] — 0 is the documented "unset → 1.0"
//                     sentinel (effectiveSlip() maps ≤0 → 1.0); negative is
//                     rejected as meaningless; (0, 0.5) is rejected to catch
//                     implausible values (effectiveSlip would silently clamp
//                     to 0.5); > 1.0 is rejected (inflates arc estimates).
//   aMax > 0        — trapezoid dv_max used as denominator / step; zero stalls
//                     BVC at zero speed (every motion verb looks dead).
//   aDecel > 0      — trapezoid decel step dv_max; negative makes approach()
//                     move away from target (runaway) and sqrtf(negative)→NaN
//                     disables decel caps entirely.
//   vBodyMax > 0    — body forward speed ceiling; zero clamps all motion
//                     targets to zero.
//   yawRateMax > 0  — yaw rate ceiling; zero clamps all yaw targets to zero.
//   yawAccMax > 0   — yaw acceleration limit; zero stalls BVC yaw channel.
//   sTimeoutMs >= STIMEOUT_MIN_MS — watchdog compare fires every tick when
//                     sTimeoutMs ≤ 0 (signed delta ≥ 0 always); even small
//                     values cause X-storms before the host can send a
//                     keepalive.  200 ms provides margin over the firmware
//                     tick budget (~25 ms worst-case) and the minimum
//                     keepalive cadence without being overly restrictive.
// ---------------------------------------------------------------------------

// Minimum allowed sTimeoutMs value.  Below this the watchdog fires before the
// host has any chance to send a keepalive (200 ms >> worst-case tick ~25 ms).
static const int32_t STIMEOUT_MIN_MS = 200;

static bool validateConfig(const RobotConfig& c, const char** badKey)
{
    if (c.trackwidthMm <= 0.0f) {
        *badKey = "tw";
        return false;
    }
    if (c.controlPeriodMs <= 0) {
        *badKey = "ctrlPeriod";
        return false;
    }
    if (c.vWheelMax <= c.steerHeadroom) {
        *badKey = "vWheelMax";
        return false;
    }
    // rotSlip=0 is the documented "unset" sentinel → effectiveSlip() → 1.0.
    // Valid range: exactly 0.0 (unset), or [0.5, 1.0] (calibrated).
    // Reject: negative (meaningless), (0, 0.5) (implausibly low and likely a
    // user mistake — effectiveSlip() would silently clamp to 0.5), > 1.0
    // (would inflate arc estimates).
    if (c.rotationalSlip < 0.0f ||
        (c.rotationalSlip > 0.0f && c.rotationalSlip < 0.5f) ||
        c.rotationalSlip > 1.0f) {
        *badKey = "rotSlip";
        return false;
    }
    if (c.aMax <= 0.0f) {
        *badKey = "aMax";
        return false;
    }
    if (c.aDecel <= 0.0f) {
        *badKey = "aDecel";
        return false;
    }
    if (c.vBodyMax <= 0.0f) {
        *badKey = "vBodyMax";
        return false;
    }
    if (c.yawRateMax <= 0.0f) {
        *badKey = "yawRateMax";
        return false;
    }
    if (c.yawAccMax <= 0.0f) {
        *badKey = "yawAccMax";
        return false;
    }
    if (c.sTimeoutMs < STIMEOUT_MIN_MS) {
        *badKey = "sTimeout";
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// handleSet — HandlerFn-compatible SET handler.
//
// args.args[0..args.count-1].sval carries "key=value" strings (one per pair).
// handlerCtx is cast to CfgCtx*.
//
// Parse/validation strategy (028-004):
//   1. Replace atof/atoi with strtof/strtol end-pointer checks.  A value is
//      rejected (ERR badval <key>) if the end-pointer is not at the end of the
//      string, or the string is empty.
//   2. Build a candidate RobotConfig copy (candidate = cfg) and apply all valid
//      keys to the candidate, NOT to cfg.
//   3. After processing all keys, call validateConfig(candidate).  If it
//      fails, emit ERR badval <key>=<value> and return — cfg is unchanged.
//   4. Only if validateConfig passes: cfg = candidate; emit OK set <applied>.
//
// Emits ERR badval <key> per parse failure (non-numeric / empty value).
// Emits ERR badkey <key> per unknown key.
// Calls MotorController::updatePidGains / updateVelGains when relevant
// params change (only after successful commit).
// ---------------------------------------------------------------------------

void handleSet(const ArgList& args, const char* corrId,
               ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    CfgCtx* ctx = reinterpret_cast<CfgCtx*>(handlerCtx);
    RobotConfig& cfg = *ctx->cfg;
    MotorController& mc = *ctx->mc;

    // Candidate config — all valid key writes go here; committed atomically.
    RobotConfig candidate = cfg;

    // Build "OK set <applied keys>" body.
    char applied[480];
    int apos = 0;
    int arem = (int)sizeof(applied);

    bool velChanged = false;
    bool anyParseErr = false;   // set on strtof/strtol end-pointer failure

    // Each arg's sval holds "key=value"; split on '='.
    for (int i = 0; i < args.count; ++i) {
        // Copy sval so we can split in place.
        char kvbuf[64];
        int kvlen = 0;
        for (const char* p = args.args[i].sval;
             *p && kvlen < (int)sizeof(kvbuf) - 1; ++p, ++kvlen) {
            kvbuf[kvlen] = *p;
        }
        kvbuf[kvlen] = '\0';

        char* eq = strchr(kvbuf, '=');
        if (!eq) continue;  // malformed: no '='; skip

        *eq = '\0';
        const char* k = kvbuf;
        const char* v = eq + 1;

        if (!k || k[0] == '\0') continue;  // no key

        // Find in registry.
        const ConfigEntry* entry = nullptr;
        for (int r = 0; r < kRegistryCount; ++r) {
            if (strcmp(kRegistry[r].key, k) == 0) {
                entry = &kRegistry[r];
                break;
            }
        }

        if (!entry) {
            // Unknown key -- emit ERR and continue processing remaining keys.
            char rbuf[128];
            CommandProcessor::replyErr(rbuf, (int)sizeof(rbuf),
                                       "badkey", k, corrId, replyFn, replyCtx);
            anyParseErr = true;
            continue;
        }

        // Typed parse with end-pointer validation.
        // Reject empty string or any non-numeric suffix.
        char* base = reinterpret_cast<char*>(&candidate);
        switch (entry->type) {
        case CFG_FLOAT: {
            if (v[0] == '\0') {
                char rbuf[128];
                CommandProcessor::replyErr(rbuf, (int)sizeof(rbuf),
                                           "badval", k, corrId, replyFn, replyCtx);
                anyParseErr = true;
                continue;
            }
            char* endp = nullptr;
            float fv = strtof(v, &endp);
            if (endp == v || *endp != '\0') {
                char rbuf[128];
                CommandProcessor::replyErr(rbuf, (int)sizeof(rbuf),
                                           "badval", k, corrId, replyFn, replyCtx);
                anyParseErr = true;
                continue;
            }
            memcpy(base + entry->offset, &fv, sizeof(float));
            break;
        }
        case CFG_INT: {
            if (v[0] == '\0') {
                char rbuf[128];
                CommandProcessor::replyErr(rbuf, (int)sizeof(rbuf),
                                           "badval", k, corrId, replyFn, replyCtx);
                anyParseErr = true;
                continue;
            }
            char* endp = nullptr;
            long lv = strtol(v, &endp, 10);
            if (endp == v || *endp != '\0') {
                char rbuf[128];
                CommandProcessor::replyErr(rbuf, (int)sizeof(rbuf),
                                           "badval", k, corrId, replyFn, replyCtx);
                anyParseErr = true;
                continue;
            }
            int32_t iv = (int32_t)lv;
            memcpy(base + entry->offset, &iv, sizeof(int32_t));
            break;
        }
        case CFG_FLOAT_AS_INT: {
            if (v[0] == '\0') {
                char rbuf[128];
                CommandProcessor::replyErr(rbuf, (int)sizeof(rbuf),
                                           "badval", k, corrId, replyFn, replyCtx);
                anyParseErr = true;
                continue;
            }
            char* endp = nullptr;
            long lv = strtol(v, &endp, 10);
            if (endp == v || *endp != '\0') {
                char rbuf[128];
                CommandProcessor::replyErr(rbuf, (int)sizeof(rbuf),
                                           "badval", k, corrId, replyFn, replyCtx);
                anyParseErr = true;
                continue;
            }
            float fv = (float)lv;
            memcpy(base + entry->offset, &fv, sizeof(float));
            break;
        }
        }

        // Per-wheel velocity gains must be pushed into the live controllers
        // (they hold copies made at construction). filt/sync are read per-tick.
        if (strcmp(k, "vel.kP") == 0 || strcmp(k, "vel.kI") == 0 ||
            strcmp(k, "vel.kFF") == 0 || strcmp(k, "vel.iMax") == 0 ||
            strcmp(k, "vel.kAw") == 0 || strcmp(k, "minWheelMms") == 0) {
            velChanged = true;
        }

        // Append to applied list.
        if (apos > 0 && arem > 1) { applied[apos++] = ' '; --arem; }
        int w = snprintf(applied + apos, (size_t)arem, "%s=%s", k, v);
        if (w > 0 && w < arem) { apos += w; arem -= w; }
    }

    // If any parse / badkey error occurred, do not commit — cfg is unchanged.
    if (anyParseErr) {
        return;
    }

    // Validate the candidate config before committing.
    if (apos > 0) {
        const char* badKey = nullptr;
        if (!validateConfig(candidate, &badKey)) {
            // Find the failing key's current value in the candidate for the
            // ERR badval key=value detail.
            char detail[64];
            // Look up the failing key in the registry to format its candidate value.
            bool found = false;
            for (int r = 0; r < kRegistryCount && !found; ++r) {
                if (strcmp(kRegistry[r].key, badKey) == 0) {
                    const char* cbase = reinterpret_cast<const char*>(&candidate);
                    switch (kRegistry[r].type) {
                    case CFG_FLOAT: {
                        float fv;
                        memcpy(&fv, cbase + kRegistry[r].offset, sizeof(float));
                        snprintf(detail, sizeof(detail), "%s=%.3f", badKey, (double)fv);
                        break;
                    }
                    case CFG_INT: {
                        int32_t iv;
                        memcpy(&iv, cbase + kRegistry[r].offset, sizeof(int32_t));
                        snprintf(detail, sizeof(detail), "%s=%d", badKey, (int)iv);
                        break;
                    }
                    case CFG_FLOAT_AS_INT: {
                        float fv;
                        memcpy(&fv, cbase + kRegistry[r].offset, sizeof(float));
                        snprintf(detail, sizeof(detail), "%s=%d", badKey, (int)fv);
                        break;
                    }
                    }
                    found = true;
                }
            }
            if (!found) {
                // Fallback: key name only (should not happen with invariant keys).
                snprintf(detail, sizeof(detail), "%s", badKey);
            }
            char rbuf[128];
            CommandProcessor::replyErr(rbuf, (int)sizeof(rbuf),
                                       "badval", detail, corrId, replyFn, replyCtx);
            return;  // cfg unchanged
        }

        // Validation passed — commit atomically.
        cfg = candidate;

        if (velChanged) {
            mc.updateVelGains(cfg);
        }

        applied[apos] = '\0';
        char rbuf[520];
        CommandProcessor::replyOK(rbuf, (int)sizeof(rbuf), "set", applied, corrId,
                                  replyFn, replyCtx);
    }
}
