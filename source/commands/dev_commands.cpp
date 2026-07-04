// dev_commands.cpp -- DEV command family implementation. See dev_commands.h
// for the full vocabulary, the argument-parsing design decision (Open
// Question 3), the authority-arbitration rule, the serial-silence watchdog
// contract, and the ROBOT_DEV_BUILD gating rationale.
#include "commands/dev_commands.h"

#if ROBOT_DEV_BUILD

#include "commands/command_processor.h"
#include "commands/arg_parse.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <math.h>

namespace {

// ---------------------------------------------------------------------------
// formatFixed -- render `value` as a fixed-point decimal string with exactly
// `decimals` digits after the point (0..4), using only integer arithmetic --
// NOT "%f"/"%.Nf". CODAL/newlib-nano's printf family has no float-conversion
// support on this toolchain (no -u _printf_float in utils/cmake/toolchains/
// ARM_GCC/compiler-flags.cmake): a "%f" specifier silently emits nothing,
// which would make every DEV reply body carrying a float field (pos=, vel=,
// applied=, DEV DT vel=/vx=/vy=/omega=, DEV M CFG's kp=/slew=/... ack) come
// back with an empty value instead of a number. Confirmed live on hardware
// during this ticket's HITL smoke test. Same constraint, same fix shape as
// source_old/commands/DebugCommands.cpp's "F1 fix" (034-004) -- that fix
// switched to integer-scaled fields (mm, centidegrees) instead; this one
// keeps genuine fixed-point text (matching the issue's locked wire example
// "OK DEV M 1 applied=0.30") since lroundf()/plain integer math (both
// available in newlib-nano without float-printf support, per the F1 fix's
// own note) are enough to build the string by hand.
// ---------------------------------------------------------------------------
void formatFixed(char* buf, int bufSize, float value, int decimals) {
    if (bufSize <= 0) return;
    if (decimals < 0) decimals = 0;
    if (decimals > 4) decimals = 4;

    bool negative = value < 0.0f;
    float absValue = negative ? -value : value;

    int32_t scale = 1;
    for (int i = 0; i < decimals; ++i) scale *= 10;

    int32_t scaled = static_cast<int32_t>(lroundf(absValue * static_cast<float>(scale)));
    int32_t intPart = scaled / scale;
    int32_t fracPart = scaled % scale;

    char frac[8];
    for (int i = decimals - 1; i >= 0; --i) {
        frac[i] = static_cast<char>('0' + (fracPart % 10));
        fracPart /= 10;
    }
    frac[decimals] = '\0';

    if (decimals > 0) {
        snprintf(buf, static_cast<size_t>(bufSize), "%s%ld.%s",
                 negative ? "-" : "", static_cast<long>(intPart), frac);
    } else {
        snprintf(buf, static_cast<size_t>(bufSize), "%s%ld",
                 negative ? "-" : "", static_cast<long>(intPart));
    }
}

// ---------------------------------------------------------------------------
// DEV WD <window> -- the one DEV subcommand with a pure fixed positional
// shape, so it uses ArgSchema (mixed hand-rolled/schema approach, per
// dev_commands.h's Open Question 3 note).
// ---------------------------------------------------------------------------
const ArgDef kDevWdArgs[] = {
    { "window", ArgKind::INT, true, 50, 60000 },
};
const ArgSchema kDevWdSchema = { kDevWdArgs, 1, 1, false, nullptr };

// ---------------------------------------------------------------------------
// Neutral-mode token: "B" (brake) or "C" (coast) -- see dev_commands.h's
// vocabulary table. Case-sensitive, matching every other sub-token in this
// protocol (only the verb itself is upper-cased by CommandProcessor).
// ---------------------------------------------------------------------------
bool neutralModeFromToken(const char* tok, msg::Neutral* mode) {
    if (strcmp(tok, "B") == 0) { *mode = msg::Neutral::BRAKE; return true; }
    if (strcmp(tok, "C") == 0) { *mode = msg::Neutral::COAST; return true; }
    return false;
}

// ---------------------------------------------------------------------------
// DEV M -- sub-mode keyword table and hand-rolled ParseFn/HandlerFn.
// ---------------------------------------------------------------------------
enum class MotorMode : uint8_t {
    DUTY, VEL, POS, VOLT, NEUTRAL, RESET, STATE, CAPS, CFG
};

bool motorModeFromToken(const char* tok, MotorMode* mode) {
    if (strcmp(tok, "DUTY") == 0)    { *mode = MotorMode::DUTY;    return true; }
    if (strcmp(tok, "VEL") == 0)     { *mode = MotorMode::VEL;     return true; }
    if (strcmp(tok, "POS") == 0)     { *mode = MotorMode::POS;     return true; }
    if (strcmp(tok, "VOLT") == 0)    { *mode = MotorMode::VOLT;    return true; }
    if (strcmp(tok, "NEUTRAL") == 0) { *mode = MotorMode::NEUTRAL; return true; }
    if (strcmp(tok, "RESET") == 0)   { *mode = MotorMode::RESET;   return true; }
    if (strcmp(tok, "STATE") == 0)   { *mode = MotorMode::STATE;   return true; }
    if (strcmp(tok, "CAPS") == 0)    { *mode = MotorMode::CAPS;    return true; }
    if (strcmp(tok, "CFG") == 0)     { *mode = MotorMode::CFG;     return true; }
    return false;
}

// parseDevM -- argTokens after the "DEV M" prefix is stripped:
//   tokens[0] = port (1..4)
//   tokens[1] = mode keyword
//   tokens[2..] = mode-specific: a value (DUTY/VEL/POS/VOLT), B|C (NEUTRAL),
//                 nothing (RESET/STATE/CAPS), or nothing here for CFG --
//                 CFG's k=v pairs arrive via kvs (already split out by
//                 CommandProcessor::parseKV over the WHOLE line before
//                 dispatch), re-serialized below into ArgList as STR
//                 "key=value" entries so the handler (which only sees
//                 ArgList, not kvs) can recover them.
ParseResult parseDevM(const char* const* tokens, int ntokens,
                      const KVPair* kvs, int nkv) {
    ParseResult res;

    if (ntokens < 2) {
        res.ok = false; res.err.code = nullptr; res.err.detail = nullptr;
        return res;
    }

    int port = atoi(tokens[0]);
    if (port < 1 || port > 4) {
        res.ok = false; res.err.code = "range"; res.err.detail = "port";
        return res;
    }

    MotorMode mode;
    if (!motorModeFromToken(tokens[1], &mode)) {
        res.ok = false; res.err.code = "unknown"; res.err.detail = tokens[1];
        return res;
    }

    res.ok = true;
    int count = 0;
    argInt(res.args.args[count++], port);
    argStr(res.args.args[count++], tokens[1]);

    switch (mode) {
        case MotorMode::DUTY:
        case MotorMode::VEL:
        case MotorMode::POS:
        case MotorMode::VOLT: {
            if (ntokens < 3) {
                res.ok = false; res.err.code = nullptr; res.err.detail = nullptr;
                return res;
            }
            argFloat(res.args.args[count++], static_cast<float>(atof(tokens[2])));
            break;
        }
        case MotorMode::NEUTRAL: {
            msg::Neutral nm;
            if (ntokens < 3 || !neutralModeFromToken(tokens[2], &nm)) {
                res.ok = false; res.err.code = nullptr; res.err.detail = "neutral";
                return res;
            }
            argStr(res.args.args[count++], tokens[2]);
            break;
        }
        case MotorMode::CFG: {
            if (nkv == 0) {
                res.ok = false; res.err.code = nullptr; res.err.detail = "no keys";
                return res;
            }
            for (int i = 0; i < nkv && count < MAX_ARGS; ++i) {
                char kvbuf[32];
                snprintf(kvbuf, sizeof(kvbuf), "%s=%s",
                         kvs[i].key ? kvs[i].key : "",
                         kvs[i].value ? kvs[i].value : "");
                argStr(res.args.args[count++], kvbuf);
            }
            break;
        }
        case MotorMode::RESET:
        case MotorMode::STATE:
        case MotorMode::CAPS:
        default:
            break;
    }

    res.args.count = count;
    res.args.suppliedCount = count;
    return res;
}

// ---------------------------------------------------------------------------
// applyMotorCfgKey -- one key=value delta onto a shadow msg::MotorConfig.
// Returns false (key untouched) for an unrecognized key; the caller reports
// ERR badkey. appliedOut receives the applied "key=value" text (as actually
// stored, after atof/atoi) for the ack line.
// ---------------------------------------------------------------------------
bool applyMotorCfgKey(msg::MotorConfig& cfg, const char* key, const char* value,
                      char* appliedOut, int appliedOutSize) {
    char numStr[16];
    if (strcmp(key, "kp") == 0) {
        cfg.vel_gains.kp = static_cast<float>(atof(value));
        formatFixed(numStr, sizeof(numStr), cfg.vel_gains.kp, 3);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "kp=%s", numStr);
        return true;
    }
    if (strcmp(key, "ki") == 0) {
        cfg.vel_gains.ki = static_cast<float>(atof(value));
        formatFixed(numStr, sizeof(numStr), cfg.vel_gains.ki, 3);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "ki=%s", numStr);
        return true;
    }
    if (strcmp(key, "kff") == 0) {
        cfg.vel_gains.kff = static_cast<float>(atof(value));
        formatFixed(numStr, sizeof(numStr), cfg.vel_gains.kff, 3);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "kff=%s", numStr);
        return true;
    }
    if (strcmp(key, "i_max") == 0) {
        cfg.vel_gains.i_max = static_cast<float>(atof(value));
        formatFixed(numStr, sizeof(numStr), cfg.vel_gains.i_max, 3);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "i_max=%s", numStr);
        return true;
    }
    if (strcmp(key, "kaw") == 0) {
        cfg.vel_gains.kaw = static_cast<float>(atof(value));
        formatFixed(numStr, sizeof(numStr), cfg.vel_gains.kaw, 3);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "kaw=%s", numStr);
        return true;
    }
    if (strcmp(key, "slew") == 0) {
        cfg.slew_rate = static_cast<float>(atof(value));
        formatFixed(numStr, sizeof(numStr), cfg.slew_rate, 1);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "slew=%s", numStr);
        return true;
    }
    if (strcmp(key, "min_duty") == 0) {
        cfg.min_duty = static_cast<float>(atof(value));
        formatFixed(numStr, sizeof(numStr), cfg.min_duty, 3);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "min_duty=%s", numStr);
        return true;
    }
    if (strcmp(key, "travel_calib") == 0) {
        cfg.travel_calib = static_cast<float>(atof(value));
        formatFixed(numStr, sizeof(numStr), cfg.travel_calib, 4);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "travel_calib=%s", numStr);
        return true;
    }
    if (strcmp(key, "fwd_sign") == 0) {
        cfg.fwd_sign = atoi(value);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "fwd_sign=%d",
                 static_cast<int>(cfg.fwd_sign));
        return true;
    }
    if (strcmp(key, "vel_filt_alpha") == 0) {
        cfg.vel_filt_alpha = static_cast<float>(atof(value));
        formatFixed(numStr, sizeof(numStr), cfg.vel_filt_alpha, 3);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "vel_filt_alpha=%s", numStr);
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// emitMotorState / emitDrivetrainState -- shared line-builders. Used both by
// the per-component STATE handlers (DEV M <n> STATE, DEV DT STATE) and by
// the aggregate DEV STATE handler (one call per component).
// ---------------------------------------------------------------------------
void emitMotorState(Hal::Motor& motor, uint32_t port, const char* corrId,
                    ReplyFn replyFn, void* replyCtx) {
    msg::MotorState s = motor.state();
    char verb[16];
    snprintf(verb, sizeof(verb), "DEV M %u", static_cast<unsigned>(port));

    char posStr[16], velStr[16], appliedStr[16];
    formatFixed(posStr, sizeof(posStr), s.position.has ? s.position.val : 0.0f, 1);
    formatFixed(velStr, sizeof(velStr), s.velocity.has ? s.velocity.val : 0.0f, 1);
    formatFixed(appliedStr, sizeof(appliedStr), s.applied.has ? s.applied.val : 0.0f, 2);

    char rbuf[200];
    CommandProcessor::replyOKf(rbuf, sizeof(rbuf), verb, corrId, replyFn, replyCtx,
        "pos=%s vel=%s applied=%s wedged=%d conn=%d",
        posStr, velStr, appliedStr,
        (s.wedged.has && s.wedged.val) ? 1 : 0,
        s.connected ? 1 : 0);
}

void emitDrivetrainState(DevLoopState& state, const char* corrId,
                         ReplyFn replyFn, void* replyCtx) {
    msg::DrivetrainState s = state.drivetrain->state();
    float vL = (s.vel_count_val() > 0) ? s.vel()[0] : 0.0f;
    float vR = (s.vel_count_val() > 1) ? s.vel()[1] : 0.0f;

    char vLStr[16], vRStr[16];
    formatFixed(vLStr, sizeof(vLStr), vL, 1);
    formatFixed(vRStr, sizeof(vRStr), vR, 1);

    char rbuf[200];
    CommandProcessor::replyOKf(rbuf, sizeof(rbuf), "DEV DT", corrId, replyFn, replyCtx,
        "active=%d ports=%u,%u vel=%s,%s",
        state.drivetrainActive ? 1 : 0,
        static_cast<unsigned>(state.leftPort), static_cast<unsigned>(state.rightPort),
        vLStr, vRStr);
}

// handleDevMCfg -- CFG delta: apply each supplied key onto the port's shadow
// MotorConfig, ERR badkey per unrecognized key, one OK ack line listing only
// the keys that actually applied (if any).
void handleDevMCfg(DevLoopState& state, uint32_t port, const ArgList& args,
                   const char* corrId, ReplyFn replyFn, void* replyCtx) {
    msg::MotorConfig& cfg = state.motorConfigShadow[port - 1];
    char appliedBody[256];
    int bodyLen = 0;
    appliedBody[0] = '\0';
    bool anyApplied = false;

    for (int i = 2; i < args.count; ++i) {
        const char* kvtok = args.args[i].sval;
        const char* eq = strchr(kvtok, '=');
        if (!eq) continue;   // shouldn't happen -- parseDevM only packs "key=value"

        char key[24];
        int klen = static_cast<int>(eq - kvtok);
        if (klen >= static_cast<int>(sizeof(key))) klen = sizeof(key) - 1;
        memcpy(key, kvtok, static_cast<size_t>(klen));
        key[klen] = '\0';
        const char* value = eq + 1;

        char oneApplied[40];
        if (applyMotorCfgKey(cfg, key, value, oneApplied, sizeof(oneApplied))) {
            anyApplied = true;
            if (bodyLen > 0 && bodyLen < static_cast<int>(sizeof(appliedBody)) - 1) {
                appliedBody[bodyLen++] = ' ';
            }
            int n = snprintf(appliedBody + bodyLen, sizeof(appliedBody) - static_cast<size_t>(bodyLen),
                             "%s", oneApplied);
            if (n > 0) bodyLen += n;
        } else {
            char rbuf[64];
            CommandProcessor::replyErr(rbuf, sizeof(rbuf), "badkey", key, corrId, replyFn, replyCtx);
        }
    }

    if (anyApplied) {
        state.hal->motor(port).configure(cfg);
        char verb[16];
        snprintf(verb, sizeof(verb), "DEV M %u", static_cast<unsigned>(port));
        char rbuf[300];
        CommandProcessor::replyOKf(rbuf, sizeof(rbuf), verb, corrId, replyFn, replyCtx,
                                   "%s", appliedBody);
    }
}

// handleDevM -- dispatches on the mode keyword parseDevM already validated.
void handleDevM(const ArgList& args, const char* corrId,
                ReplyFn replyFn, void* replyCtx, void* handlerCtx) {
    DevLoopState& state = *static_cast<DevLoopState*>(handlerCtx);
    uint32_t port = static_cast<uint32_t>(args.args[0].ival);
    MotorMode mode;
    motorModeFromToken(args.args[1].sval, &mode);   // already validated by parseDevM

    Hal::Motor& motor = state.hal->motor(port);
    char verb[16];
    snprintf(verb, sizeof(verb), "DEV M %u", static_cast<unsigned>(port));
    char rbuf[200];

    switch (mode) {
        case MotorMode::DUTY: {
            float duty = args.args[2].fval / 100.0f;
            msg::MotorCommand cmd;
            cmd.setDutyCycle(duty);
            if (!motor.apply(cmd)) {
                CommandProcessor::replyErr(rbuf, sizeof(rbuf), "unsupported", "duty", corrId, replyFn, replyCtx);
                break;
            }
            state.drivetrainActive = false;
            char dutyStr[16];
            formatFixed(dutyStr, sizeof(dutyStr), duty, 2);
            CommandProcessor::replyOKf(rbuf, sizeof(rbuf), verb, corrId, replyFn, replyCtx,
                                       "applied=%s", dutyStr);
            break;
        }
        case MotorMode::VEL: {
            float velocity = args.args[2].fval;
            msg::MotorCommand cmd;
            cmd.setVelocity(velocity);
            if (!motor.apply(cmd)) {
                CommandProcessor::replyErr(rbuf, sizeof(rbuf), "unsupported", "vel", corrId, replyFn, replyCtx);
                break;
            }
            state.drivetrainActive = false;
            char velStr[16];
            formatFixed(velStr, sizeof(velStr), velocity, 1);
            CommandProcessor::replyOKf(rbuf, sizeof(rbuf), verb, corrId, replyFn, replyCtx,
                                       "vel=%s", velStr);
            break;
        }
        case MotorMode::POS: {
            float position = args.args[2].fval;
            msg::MotorCommand cmd;
            cmd.setPosition(position);
            if (!motor.apply(cmd)) {
                CommandProcessor::replyErr(rbuf, sizeof(rbuf), "unsupported", "pos", corrId, replyFn, replyCtx);
                break;
            }
            state.drivetrainActive = false;
            char posStr[16];
            formatFixed(posStr, sizeof(posStr), position, 1);
            CommandProcessor::replyOKf(rbuf, sizeof(rbuf), verb, corrId, replyFn, replyCtx,
                                       "pos=%s", posStr);
            break;
        }
        case MotorMode::VOLT: {
            float voltage = args.args[2].fval;
            msg::MotorCommand cmd;
            cmd.setVoltage(voltage);
            if (!motor.apply(cmd)) {
                // Expected on Nezha: capabilities().voltage == false -- proves
                // apply()'s capability gate, not a DEV-layer special case.
                CommandProcessor::replyErr(rbuf, sizeof(rbuf), "unsupported", "volt", corrId, replyFn, replyCtx);
                break;
            }
            state.drivetrainActive = false;
            char voltStr[16];
            formatFixed(voltStr, sizeof(voltStr), voltage, 2);
            CommandProcessor::replyOKf(rbuf, sizeof(rbuf), verb, corrId, replyFn, replyCtx,
                                       "volt=%s", voltStr);
            break;
        }
        case MotorMode::NEUTRAL: {
            const char* bc = args.args[2].sval;
            msg::Neutral nm = msg::Neutral::BRAKE;   // safe fallback; parseDevM already validated bc
            neutralModeFromToken(bc, &nm);
            msg::MotorCommand cmd;
            cmd.setNeutral(nm);
            motor.apply(cmd);   // NEUTRAL is never capability-gated -- always accepted
            state.drivetrainActive = false;
            CommandProcessor::replyOKf(rbuf, sizeof(rbuf), verb, corrId, replyFn, replyCtx,
                                       "neutral=%s", bc);
            break;
        }
        case MotorMode::RESET: {
            msg::MotorCommand cmd;
            cmd.setResetPosition(true);
            motor.apply(cmd);
            state.drivetrainActive = false;
            CommandProcessor::replyOKf(rbuf, sizeof(rbuf), verb, corrId, replyFn, replyCtx, "reset=1");
            break;
        }
        case MotorMode::STATE:
            emitMotorState(motor, port, corrId, replyFn, replyCtx);
            break;
        case MotorMode::CAPS: {
            msg::MotorCapabilities caps = motor.capabilities();
            CommandProcessor::replyOKf(rbuf, sizeof(rbuf), verb, corrId, replyFn, replyCtx,
                "duty=%d volt=%d vel=%d pos=%d enc=%d",
                caps.duty_cycle ? 1 : 0, caps.voltage ? 1 : 0, caps.velocity ? 1 : 0,
                caps.position ? 1 : 0, caps.has_encoder ? 1 : 0);
            break;
        }
        case MotorMode::CFG:
            handleDevMCfg(state, port, args, corrId, replyFn, replyCtx);
            break;
    }
}

// ---------------------------------------------------------------------------
// DEV DT -- sub-mode keyword table and hand-rolled ParseFn/HandlerFn.
// ---------------------------------------------------------------------------
enum class DtMode : uint8_t { PORTS, VW, WHEELS, NEUTRAL, STATE, STOP };

bool dtModeFromToken(const char* tok, DtMode* mode) {
    if (strcmp(tok, "PORTS") == 0)   { *mode = DtMode::PORTS;   return true; }
    if (strcmp(tok, "VW") == 0)      { *mode = DtMode::VW;      return true; }
    if (strcmp(tok, "WHEELS") == 0)  { *mode = DtMode::WHEELS;  return true; }
    if (strcmp(tok, "NEUTRAL") == 0) { *mode = DtMode::NEUTRAL; return true; }
    if (strcmp(tok, "STATE") == 0)   { *mode = DtMode::STATE;   return true; }
    if (strcmp(tok, "STOP") == 0)    { *mode = DtMode::STOP;    return true; }
    return false;
}

// parseDevDt -- argTokens after the "DEV DT" prefix is stripped:
//   tokens[0] = sub-mode keyword
//   tokens[1..] = mode-specific positional values (ports/twist/wheels/neutral)
ParseResult parseDevDt(const char* const* tokens, int ntokens,
                       const KVPair* /*kvs*/, int /*nkv*/) {
    ParseResult res;

    if (ntokens < 1) {
        res.ok = false; res.err.code = nullptr; res.err.detail = nullptr;
        return res;
    }

    DtMode mode;
    if (!dtModeFromToken(tokens[0], &mode)) {
        res.ok = false; res.err.code = "unknown"; res.err.detail = tokens[0];
        return res;
    }

    res.ok = true;
    int count = 0;
    argStr(res.args.args[count++], tokens[0]);

    switch (mode) {
        case DtMode::PORTS: {
            if (ntokens < 3) {
                res.ok = false; res.err.code = nullptr; res.err.detail = nullptr;
                return res;
            }
            int left = atoi(tokens[1]);
            int right = atoi(tokens[2]);
            if (left < 1 || left > 4) {
                res.ok = false; res.err.code = "range"; res.err.detail = "left";
                return res;
            }
            if (right < 1 || right > 4) {
                res.ok = false; res.err.code = "range"; res.err.detail = "right";
                return res;
            }
            argInt(res.args.args[count++], left);
            argInt(res.args.args[count++], right);
            break;
        }
        case DtMode::VW: {
            if (ntokens < 4) {
                res.ok = false; res.err.code = nullptr; res.err.detail = nullptr;
                return res;
            }
            argFloat(res.args.args[count++], static_cast<float>(atof(tokens[1])));
            argFloat(res.args.args[count++], static_cast<float>(atof(tokens[2])));
            argFloat(res.args.args[count++], static_cast<float>(atof(tokens[3])));
            break;
        }
        case DtMode::WHEELS: {
            if (ntokens < 3) {
                res.ok = false; res.err.code = nullptr; res.err.detail = nullptr;
                return res;
            }
            argFloat(res.args.args[count++], static_cast<float>(atof(tokens[1])));
            argFloat(res.args.args[count++], static_cast<float>(atof(tokens[2])));
            break;
        }
        case DtMode::NEUTRAL: {
            msg::Neutral nm;
            if (ntokens < 2 || !neutralModeFromToken(tokens[1], &nm)) {
                res.ok = false; res.err.code = nullptr; res.err.detail = "neutral";
                return res;
            }
            argStr(res.args.args[count++], tokens[1]);
            break;
        }
        case DtMode::STATE:
        case DtMode::STOP:
        default:
            break;
    }

    res.args.count = count;
    res.args.suppliedCount = count;
    return res;
}

void handleDevDt(const ArgList& args, const char* corrId,
                 ReplyFn replyFn, void* replyCtx, void* handlerCtx) {
    DevLoopState& state = *static_cast<DevLoopState*>(handlerCtx);
    DtMode mode;
    dtModeFromToken(args.args[0].sval, &mode);   // already validated by parseDevDt
    char rbuf[200];

    switch (mode) {
        case DtMode::PORTS: {
            uint32_t left = static_cast<uint32_t>(args.args[1].ival);
            uint32_t right = static_cast<uint32_t>(args.args[2].ival);
            state.leftPort = left;
            state.rightPort = right;
            // Refresh the capabilities cache so DrivetrainCapabilities.
            // onboard_position stays accurate for the newly-bound pair --
            // see drivetrain.h's setMotorCapabilities() doc comment.
            state.drivetrain->setMotorCapabilities(state.hal->motor(left).capabilities(),
                                                    state.hal->motor(right).capabilities());
            CommandProcessor::replyOKf(rbuf, sizeof(rbuf), "DEV DT", corrId, replyFn, replyCtx,
                                       "ports=%u,%u", static_cast<unsigned>(left), static_cast<unsigned>(right));
            break;
        }
        case DtMode::VW: {
            float vx = args.args[1].fval;
            float vy = args.args[2].fval;
            float omega = args.args[3].fval;
            msg::BodyTwist3 twist;
            twist.v_x = vx;
            twist.v_y = vy;
            twist.omega = omega;
            msg::DrivetrainCommand cmd;
            cmd.setTwist(twist);
            state.drivetrain->apply(cmd);
            state.drivetrainActive = true;
            char vxStr[16], vyStr[16], omegaStr[16];
            formatFixed(vxStr, sizeof(vxStr), vx, 1);
            formatFixed(vyStr, sizeof(vyStr), vy, 1);
            formatFixed(omegaStr, sizeof(omegaStr), omega, 3);
            CommandProcessor::replyOKf(rbuf, sizeof(rbuf), "DEV DT", corrId, replyFn, replyCtx,
                                       "vx=%s vy=%s omega=%s", vxStr, vyStr, omegaStr);
            break;
        }
        case DtMode::WHEELS: {
            float left = args.args[1].fval;
            float right = args.args[2].fval;
            msg::WheelTargets wt;
            wt.w_[0].speed.has = true; wt.w_[0].speed.val = left;
            wt.w_[1].speed.has = true; wt.w_[1].speed.val = right;
            wt.w_count = 2;
            msg::DrivetrainCommand cmd;
            cmd.setWheels(wt);
            state.drivetrain->apply(cmd);
            state.drivetrainActive = true;
            char leftStr[16], rightStr[16];
            formatFixed(leftStr, sizeof(leftStr), left, 1);
            formatFixed(rightStr, sizeof(rightStr), right, 1);
            CommandProcessor::replyOKf(rbuf, sizeof(rbuf), "DEV DT", corrId, replyFn, replyCtx,
                                       "left=%s right=%s", leftStr, rightStr);
            break;
        }
        case DtMode::NEUTRAL: {
            const char* bc = args.args[1].sval;
            msg::Neutral nm = msg::Neutral::BRAKE;   // safe fallback; parseDevDt already validated bc
            neutralModeFromToken(bc, &nm);
            msg::DrivetrainCommand cmd;
            cmd.setNeutral(nm);
            state.drivetrain->apply(cmd);
            state.drivetrainActive = true;
            CommandProcessor::replyOKf(rbuf, sizeof(rbuf), "DEV DT", corrId, replyFn, replyCtx,
                                       "neutral=%s", bc);
            break;
        }
        case DtMode::STATE:
            emitDrivetrainState(state, corrId, replyFn, replyCtx);
            break;
        case DtMode::STOP: {
            neutralizeDrivetrain(state);
            CommandProcessor::replyOK(rbuf, sizeof(rbuf), "DEV DT STOP", nullptr, corrId, replyFn, replyCtx);
            break;
        }
    }
}

// ---------------------------------------------------------------------------
// DEV STATE -- aggregate: one line per motor (ports 1..4) + one drivetrain
// line. No authority change (a pure query).
// ---------------------------------------------------------------------------
void handleDevState(const ArgList& /*args*/, const char* corrId,
                    ReplyFn replyFn, void* replyCtx, void* handlerCtx) {
    DevLoopState& state = *static_cast<DevLoopState*>(handlerCtx);
    for (uint32_t port = 1; port <= Hal::NezhaHal::kPortCount; ++port) {
        emitMotorState(state.hal->motor(port), port, corrId, replyFn, replyCtx);
    }
    emitDrivetrainState(state, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// DEV STOP -- global: all four motors neutral, drivetrain idle, authority
// dropped. Shares neutralizeAll() with the watchdog fire path.
// ---------------------------------------------------------------------------
void handleDevStop(const ArgList& /*args*/, const char* corrId,
                   ReplyFn replyFn, void* replyCtx, void* handlerCtx) {
    DevLoopState& state = *static_cast<DevLoopState*>(handlerCtx);
    neutralizeAll(state);
    char rbuf[32];
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "DEV STOP", nullptr, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// DEV WD <window> -- set the serial-silence watchdog window.
// ---------------------------------------------------------------------------
void handleDevWd(const ArgList& args, const char* corrId,
                 ReplyFn replyFn, void* replyCtx, void* handlerCtx) {
    DevLoopState& state = *static_cast<DevLoopState*>(handlerCtx);
    uint32_t window = static_cast<uint32_t>(args.args[0].ival);
    state.watchdog->setWindow(window);
    char rbuf[48];
    CommandProcessor::replyOKf(rbuf, sizeof(rbuf), "DEV WD", corrId, replyFn, replyCtx,
                               "window=%u", static_cast<unsigned>(window));
}

}  // namespace

// ---------------------------------------------------------------------------
// neutralizeAll / neutralizeDrivetrain -- see dev_commands.h.
// ---------------------------------------------------------------------------
void neutralizeAll(DevLoopState& state, msg::Neutral mode) {
    for (uint32_t port = 1; port <= Hal::NezhaHal::kPortCount; ++port) {
        msg::MotorCommand cmd;
        cmd.setNeutral(mode);
        state.hal->motor(port).apply(cmd);
    }
    msg::DrivetrainCommand cmd;
    cmd.setNeutral(mode);
    state.drivetrain->apply(cmd);
    state.drivetrainActive = false;
}

void neutralizeDrivetrain(DevLoopState& state, msg::Neutral mode) {
    msg::DrivetrainCommand cmd;
    cmd.setNeutral(mode);
    state.drivetrain->apply(cmd);

    msg::MotorCommand leftCmd;
    leftCmd.setNeutral(mode);
    state.hal->motor(state.leftPort).apply(leftCmd);

    msg::MotorCommand rightCmd;
    rightCmd.setNeutral(mode);
    state.hal->motor(state.rightPort).apply(rightCmd);

    state.drivetrainActive = false;
}

// ---------------------------------------------------------------------------
// devCommands -- the DEV command table.
// ---------------------------------------------------------------------------
std::vector<CommandDescriptor> devCommands(DevLoopState& state) {
    std::vector<CommandDescriptor> cmds;
    cmds.push_back(makeCmd("DEV M", parseDevM, handleDevM, &state,
                           "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE));
    cmds.push_back(makeCmd("DEV DT", parseDevDt, handleDevDt, &state,
                           "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE));
    cmds.push_back(makeCmd("DEV STATE", nullptr, handleDevState, &state,
                           "badarg", ForceReply::NONE, CMD_NONE));
    cmds.push_back(makeCmd("DEV STOP", nullptr, handleDevStop, &state,
                           "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE));
    cmds.push_back(makeSchemaCmd("DEV WD", &kDevWdSchema, handleDevWd, &state,
                                 "badarg", ForceReply::NONE, CMD_NONE));
    return cmds;
}

#endif  // ROBOT_DEV_BUILD
