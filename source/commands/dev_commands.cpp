// dev_commands.cpp -- DEV command family implementation. See dev_commands.h
// for the full vocabulary, the argument-parsing design decision (Open
// Question 3), the authority-arbitration rule, the serial-silence watchdog
// contract, the ROBOT_DEV_BUILD gating rationale, and (087-006) the
// pointerless-translator reshape: every handler below reads/posts against
// Rt::Blackboard only, reached via Rt::CommandRouter::blackboard() --
// nothing here holds or dereferences a Subsystems::* pointer.
#include "commands/dev_commands.h"


#include "commands/command_processor.h"
#include "commands/arg_parse.h"
#include "hal/capability/motor.h"
#include "messages/drivetrain.h"
#include "messages/motor.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <math.h>

namespace {

// bb -- every handler's one and only route into subsystem-observed state and
// command posting. handlerCtx is always a Rt::CommandRouter* (bound once,
// at table-construction time, to every descriptor this file registers --
// see command_router.h's class comment for why it is not &bb directly).
Rt::Blackboard& bb(void* handlerCtx) { return static_cast<Rt::CommandRouter*>(handlerCtx)->blackboard(); }

// ---------------------------------------------------------------------------
// formatFixed -- render `value` as a fixed-point decimal string with exactly
// `decimals` digits after the point (0..4), using only integer arithmetic --
// NOT "%f"/"%.Nf". CODAL/newlib-nano's printf family has no float-conversion
// support on this toolchain (no -u _printf_float in utils/cmake/toolchains/
// ARM_GCC/compiler-flags.cmake): a "%f" specifier silently emits nothing,
// which would make every DEV reply body carrying a float field (pos=, vel=,
// applied=, DEV DT vel=/vx=/vy=/omega=, DEV M CFG's kp=/slew=/... ack) come
// back with an empty value instead of a number. Confirmed live on hardware
// during ticket 077-005's HITL smoke test.
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
//                 ArgList, not kvs) can recover them. Unaffected by this
//                 rewrite -- pure parsing, no state.
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
// applyMotorCfgKey -- one key=value delta onto a CANDIDATE msg::MotorConfig,
// tracking which Rt::MotorConfigField bits it touched in `mask` (087-006:
// the candidate/mask pair becomes one Rt::ConfigDelta posted to bb.configIn
// on success, replacing the old direct motor.configure() call). Returns
// false (key untouched) for an unrecognized key; the caller reports ERR
// badkey. appliedOut receives the applied "key=value" text (as actually
// stored, after atof/atoi) for the ack line.
// ---------------------------------------------------------------------------
bool applyMotorCfgKey(msg::MotorConfig& cfg, uint64_t& mask, const char* key, const char* value,
                      char* appliedOut, int appliedOutSize) {
    char numStr[16];
    if (strcmp(key, "kp") == 0) {
        cfg.vel_gains.kp = static_cast<float>(atof(value));
        mask |= Rt::bitOf(Rt::MotorConfigField::kVelGainsKp);
        formatFixed(numStr, sizeof(numStr), cfg.vel_gains.kp, 3);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "kp=%s", numStr);
        return true;
    }
    if (strcmp(key, "ki") == 0) {
        cfg.vel_gains.ki = static_cast<float>(atof(value));
        mask |= Rt::bitOf(Rt::MotorConfigField::kVelGainsKi);
        formatFixed(numStr, sizeof(numStr), cfg.vel_gains.ki, 3);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "ki=%s", numStr);
        return true;
    }
    if (strcmp(key, "kff") == 0) {
        cfg.vel_gains.kff = static_cast<float>(atof(value));
        mask |= Rt::bitOf(Rt::MotorConfigField::kVelGainsKff);
        formatFixed(numStr, sizeof(numStr), cfg.vel_gains.kff, 3);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "kff=%s", numStr);
        return true;
    }
    if (strcmp(key, "i_max") == 0) {
        cfg.vel_gains.i_max = static_cast<float>(atof(value));
        mask |= Rt::bitOf(Rt::MotorConfigField::kVelGainsIMax);
        formatFixed(numStr, sizeof(numStr), cfg.vel_gains.i_max, 3);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "i_max=%s", numStr);
        return true;
    }
    if (strcmp(key, "kaw") == 0) {
        cfg.vel_gains.kaw = static_cast<float>(atof(value));
        mask |= Rt::bitOf(Rt::MotorConfigField::kVelGainsKaw);
        formatFixed(numStr, sizeof(numStr), cfg.vel_gains.kaw, 3);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "kaw=%s", numStr);
        return true;
    }
    if (strcmp(key, "slew") == 0) {
        cfg.slew_rate = static_cast<float>(atof(value));
        mask |= Rt::bitOf(Rt::MotorConfigField::kSlewRate);
        formatFixed(numStr, sizeof(numStr), cfg.slew_rate, 1);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "slew=%s", numStr);
        return true;
    }
    if (strcmp(key, "min_duty") == 0) {
        cfg.min_duty = static_cast<float>(atof(value));
        mask |= Rt::bitOf(Rt::MotorConfigField::kMinDuty);
        formatFixed(numStr, sizeof(numStr), cfg.min_duty, 3);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "min_duty=%s", numStr);
        return true;
    }
    if (strcmp(key, "travel_calib") == 0) {
        cfg.travel_calib = static_cast<float>(atof(value));
        mask |= Rt::bitOf(Rt::MotorConfigField::kTravelCalib);
        formatFixed(numStr, sizeof(numStr), cfg.travel_calib, 4);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "travel_calib=%s", numStr);
        return true;
    }
    if (strcmp(key, "fwd_sign") == 0) {
        cfg.fwd_sign = atoi(value);
        mask |= Rt::bitOf(Rt::MotorConfigField::kFwdSign);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "fwd_sign=%d",
                 static_cast<int>(cfg.fwd_sign));
        return true;
    }
    if (strcmp(key, "vel_filt_alpha") == 0) {
        cfg.vel_filt_alpha = static_cast<float>(atof(value));
        mask |= Rt::bitOf(Rt::MotorConfigField::kVelFiltAlpha);
        formatFixed(numStr, sizeof(numStr), cfg.vel_filt_alpha, 3);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "vel_filt_alpha=%s", numStr);
        return true;
    }
    if (strcmp(key, "dwell") == 0) {
        cfg.reversal_dwell.has = true;
        cfg.reversal_dwell.val = static_cast<float>(atof(value));
        mask |= Rt::bitOf(Rt::MotorConfigField::kReversalDwell);
        formatFixed(numStr, sizeof(numStr), cfg.reversal_dwell.val, 1);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "dwell=%s", numStr);
        return true;
    }
    if (strcmp(key, "deadband") == 0) {
        cfg.output_deadband.has = true;
        cfg.output_deadband.val = static_cast<float>(atof(value));
        mask |= Rt::bitOf(Rt::MotorConfigField::kOutputDeadband);
        formatFixed(numStr, sizeof(numStr), cfg.output_deadband.val, 3);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "deadband=%s", numStr);
        return true;
    }
    if (strcmp(key, "polled") == 0) {
        // 091-002: the config-plane poll-set escape hatch (architecture-
        // update.md Decision 1) -- opts a port into (or out of) NezhaHardware's
        // I2C flip-flop schedule. No existing bool-valued CFG key exists to
        // mirror, so this accepts "true"/"1" as true and anything else
        // (including "false"/"0") as false -- the same lenient,
        // no-strict-validation convention atof()/atoi() already apply to
        // every numeric key above (a malformed token silently becomes the
        // zero value, never an ERR).
        bool polled = (strcmp(value, "true") == 0) || (strcmp(value, "1") == 0);
        cfg.polled = polled;
        mask |= Rt::bitOf(Rt::MotorConfigField::kPolled);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "polled=%d", polled ? 1 : 0);
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// applyDrivetrainCfgKey -- one key=value delta onto a CANDIDATE
// msg::DrivetrainConfig, tracking Rt::DrivetrainConfigField bits (mirrors
// applyMotorCfgKey above). Only recognizes the two keys the bench rig needs
// live: sync_gain and trackwidth. Returns false (key untouched) for an
// unrecognized key; the caller reports ERR badkey.
// ---------------------------------------------------------------------------
bool applyDrivetrainCfgKey(msg::DrivetrainConfig& cfg, uint64_t& mask, const char* key,
                           const char* value, char* appliedOut, int appliedOutSize) {
    char numStr[16];
    if (strcmp(key, "sync_gain") == 0) {
        cfg.sync_gain = static_cast<float>(atof(value));
        mask |= Rt::bitOf(Rt::DrivetrainConfigField::kSyncGain);
        formatFixed(numStr, sizeof(numStr), cfg.sync_gain, 3);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "sync_gain=%s", numStr);
        return true;
    }
    if (strcmp(key, "trackwidth") == 0) {
        cfg.trackwidth = static_cast<float>(atof(value));
        mask |= Rt::bitOf(Rt::DrivetrainConfigField::kTrackwidth);
        formatFixed(numStr, sizeof(numStr), cfg.trackwidth, 1);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "trackwidth=%s", numStr);
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// emitMotorState / emitDrivetrainState -- shared line-builders. Used both by
// the per-component STATE handlers (DEV M <n> STATE, DEV DT STATE) and by
// the aggregate DEV STATE handler (one call per component). Read directly
// from bb's committed state cells -- never a Hal::Motor/Drivetrain
// reference.
// ---------------------------------------------------------------------------
void emitMotorState(const msg::MotorState& s, uint32_t port, const char* corrId,
                    ReplyFn replyFn, void* replyCtx) {
    char verb[16];
    snprintf(verb, sizeof(verb), "DEV M %u", static_cast<unsigned>(port));

    char posStr[16], velStr[16], appliedStr[16];
    formatFixed(posStr, sizeof(posStr), s.position.has ? s.position.val : 0.0f, 1);
    formatFixed(velStr, sizeof(velStr), s.velocity.has ? s.velocity.val : 0.0f, 1);
    formatFixed(appliedStr, sizeof(appliedStr), s.applied.has ? s.applied.val : 0.0f, 2);

    char rbuf[200];
    CommandProcessor::replyOKf(rbuf, sizeof(rbuf), verb, corrId, replyFn, replyCtx,
        "pos=%s vel=%s applied=%s wedged=%d wsus=%d hrc=%u src=%u conn=%d",
        posStr, velStr, appliedStr,
        (s.wedged.has && s.wedged.val) ? 1 : 0,
        (s.wedge_suspect.has && s.wedge_suspect.val) ? 1 : 0,
        s.hard_reset_count.has ? static_cast<unsigned>(s.hard_reset_count.val) : 0u,
        s.soft_reset_count.has ? static_cast<unsigned>(s.soft_reset_count.val) : 0u,
        s.connected ? 1 : 0);
}

void emitDrivetrainState(const Rt::Blackboard& b, const char* corrId,
                         ReplyFn replyFn, void* replyCtx) {
    const msg::DrivetrainState& s = b.drivetrain;
    float vL = (s.vel_count_val() > 0) ? s.vel()[0] : 0.0f;
    float vR = (s.vel_count_val() > 1) ? s.vel()[1] : 0.0f;

    char vLStr[16], vRStr[16];
    formatFixed(vLStr, sizeof(vLStr), vL, 1);
    formatFixed(vRStr, sizeof(vRStr), vR, 1);

    char rbuf[200];
    CommandProcessor::replyOKf(rbuf, sizeof(rbuf), "DEV DT", corrId, replyFn, replyCtx,
        "active=%d ports=%u,%u vel=%s,%s",
        s.active ? 1 : 0,
        static_cast<unsigned>(b.drivetrainConfig.left_port),
        static_cast<unsigned>(b.drivetrainConfig.right_port),
        vLStr, vRStr);
}

// ---------------------------------------------------------------------------
// stealDrivetrainAuthority -- post {control_kind=NONE, standby=true} to
// bb.driveIn: authority-steal only, mode_/the last commanded target are left
// untouched (see drivetrain.h's "Authority arbitration" and dev_commands.h's
// authority-arbitration note). Called when an ACCEPTED DEV M motion verb
// targets one of the Drivetrain's currently-bound ports (087-006: shares
// bb.driveIn with DEV DT's own posts -- Decision 1's coalescing mailbox).
// ---------------------------------------------------------------------------
void stealDrivetrainAuthority(Rt::Blackboard& b) {
    msg::DrivetrainCommand cmd;   // control_kind defaults to NONE
    cmd.setStandby(true);
    b.driveIn.post(cmd);
}

// isBoundPort -- true if `port` is one of the Drivetrain's currently-bound
// left/right ports, read from bb.drivetrainConfig.left_port/right_port
// (087-006: the published snapshot of DrivetrainConfig, replacing a
// Drivetrain* ports() call -- Decision 7's router-half pattern).
bool isBoundPort(const Rt::Blackboard& b, uint32_t port) {
    return port == b.drivetrainConfig.left_port || port == b.drivetrainConfig.right_port;
}

// portIsPolled -- true if `port`'s current bb.motorConfig[] snapshot (the
// Configurator's own published value) marks it a member of NezhaHardware's
// I2C flip-flop poll-set (091-002: architecture-update.md Decision 2).
// DUTY/VEL/POS pre-validate against this BEFORE the existing capability
// gate -- see handleDevM() below. NEUTRAL/RESET/STATE/CAPS/CFG never
// consult it; a port's poll membership is orthogonal to whether it can be
// neutralized/reset/queried/reconfigured.
bool portIsPolled(const Rt::Blackboard& b, uint32_t port) {
    return b.motorConfig[port - 1].polled;
}

// handleDevMCfg -- CFG delta: apply each supplied key onto a CANDIDATE
// msg::MotorConfig seeded from bb.motorConfig[port-1] (087-006: replaces the
// old motorConfigShadow[] read-modify-write shadow -- bb.motorConfig[] is
// the Configurator's own published current value), ERR badkey per
// unrecognized key, one OK ack line listing only the keys that actually
// applied (if any). On success, posts ONE Rt::ConfigDelta (kMotor) carrying
// only the touched fields (mask) to bb.configIn -- the Configurator (ticket
// 005) folds+applies it.
void handleDevMCfg(Rt::Blackboard& b, uint32_t port, const ArgList& args,
                   const char* corrId, ReplyFn replyFn, void* replyCtx) {
    msg::MotorConfig cfg = b.motorConfig[port - 1];
    uint64_t mask = 0;
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
        if (applyMotorCfgKey(cfg, mask, key, value, oneApplied, sizeof(oneApplied))) {
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
        Rt::ConfigDelta delta;
        delta.target = Rt::ConfigDelta::kMotor;
        delta.port = port;
        delta.mask = mask;
        delta.motor = cfg;
        b.configIn.post(delta);

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
    Rt::Blackboard& b = bb(handlerCtx);
    uint32_t port = static_cast<uint32_t>(args.args[0].ival);
    MotorMode mode;
    motorModeFromToken(args.args[1].sval, &mode);   // already validated by parseDevM

    const msg::MotorCapabilities& caps = b.motorCaps[port - 1];
    char verb[16];
    snprintf(verb, sizeof(verb), "DEV M %u", static_cast<unsigned>(port));
    char rbuf[200];

    switch (mode) {
        case MotorMode::DUTY: {
            if (!portIsPolled(b, port)) {
                CommandProcessor::replyErr(rbuf, sizeof(rbuf), "nodev", "duty", corrId, replyFn, replyCtx);
                break;
            }
            float duty = args.args[2].fval / 100.0f;
            msg::MotorCommand cmd;
            cmd.setDutyCycle(duty);
            if (!Hal::motorCommandAllowed(caps, cmd.control_kind)) {
                CommandProcessor::replyErr(rbuf, sizeof(rbuf), "unsupported", "duty", corrId, replyFn, replyCtx);
                break;
            }
            if (isBoundPort(b, port)) { stealDrivetrainAuthority(b); }
            char dutyStr[16];
            formatFixed(dutyStr, sizeof(dutyStr), duty, 2);
            CommandProcessor::replyOKf(rbuf, sizeof(rbuf), verb, corrId, replyFn, replyCtx,
                                       "applied=%s", dutyStr);
            break;
        }
        case MotorMode::VEL: {
            if (!portIsPolled(b, port)) {
                CommandProcessor::replyErr(rbuf, sizeof(rbuf), "nodev", "vel", corrId, replyFn, replyCtx);
                break;
            }
            float velocity = args.args[2].fval;
            msg::MotorCommand cmd;
            cmd.setVelocity(velocity);
            if (!Hal::motorCommandAllowed(caps, cmd.control_kind)) {
                CommandProcessor::replyErr(rbuf, sizeof(rbuf), "unsupported", "vel", corrId, replyFn, replyCtx);
                break;
            }
            if (isBoundPort(b, port)) { stealDrivetrainAuthority(b); }
            char velStr[16];
            formatFixed(velStr, sizeof(velStr), velocity, 1);
            CommandProcessor::replyOKf(rbuf, sizeof(rbuf), verb, corrId, replyFn, replyCtx,
                                       "vel=%s", velStr);
            break;
        }
        case MotorMode::POS: {
            if (!portIsPolled(b, port)) {
                CommandProcessor::replyErr(rbuf, sizeof(rbuf), "nodev", "pos", corrId, replyFn, replyCtx);
                break;
            }
            float position = args.args[2].fval;
            msg::MotorCommand cmd;
            cmd.setPosition(position);
            if (!Hal::motorCommandAllowed(caps, cmd.control_kind)) {
                CommandProcessor::replyErr(rbuf, sizeof(rbuf), "unsupported", "pos", corrId, replyFn, replyCtx);
                break;
            }
            if (isBoundPort(b, port)) { stealDrivetrainAuthority(b); }
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
            if (!Hal::motorCommandAllowed(caps, cmd.control_kind)) {
                // Expected on Nezha: capabilities().voltage == false -- proves
                // the shared capability gate, not a DEV-layer special case.
                CommandProcessor::replyErr(rbuf, sizeof(rbuf), "unsupported", "volt", corrId, replyFn, replyCtx);
                break;
            }
            if (isBoundPort(b, port)) { stealDrivetrainAuthority(b); }
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
            // NEUTRAL is never capability-gated -- motorCommandAllowed() always
            // accepts it, but it still runs through the same one path as every
            // other motion verb (pre-validate, then post) rather than a
            // special case.
            if (!Hal::motorCommandAllowed(caps, cmd.control_kind)) {
                CommandProcessor::replyErr(rbuf, sizeof(rbuf), "unsupported", "neutral", corrId, replyFn, replyCtx);
                break;
            }
            if (isBoundPort(b, port)) { stealDrivetrainAuthority(b); }
            CommandProcessor::replyOKf(rbuf, sizeof(rbuf), verb, corrId, replyFn, replyCtx,
                                       "neutral=%s", bc);
            break;
        }
        case MotorMode::RESET: {
            msg::MotorCommand cmd;
            cmd.setResetPosition(true);
            // RESET's control_kind is NONE (reset_position rides the side
            // channel) -- motorCommandAllowed() always accepts NONE, but this
            // still goes through the same pre-validate-then-post path.
            if (!Hal::motorCommandAllowed(caps, cmd.control_kind)) {
                CommandProcessor::replyErr(rbuf, sizeof(rbuf), "unsupported", "reset", corrId, replyFn, replyCtx);
                break;
            }
            if (isBoundPort(b, port)) { stealDrivetrainAuthority(b); }
            CommandProcessor::replyOKf(rbuf, sizeof(rbuf), verb, corrId, replyFn, replyCtx, "reset=1");
            break;
        }
        case MotorMode::STATE:
            emitMotorState(b.motors[port - 1], port, corrId, replyFn, replyCtx);
            break;
        case MotorMode::CAPS: {
            CommandProcessor::replyOKf(rbuf, sizeof(rbuf), verb, corrId, replyFn, replyCtx,
                "duty=%d volt=%d vel=%d pos=%d enc=%d",
                caps.duty_cycle ? 1 : 0, caps.voltage ? 1 : 0, caps.velocity ? 1 : 0,
                caps.position ? 1 : 0, caps.has_encoder ? 1 : 0);
            break;
        }
        case MotorMode::CFG:
            handleDevMCfg(b, port, args, corrId, replyFn, replyCtx);
            break;
    }
}

// ---------------------------------------------------------------------------
// DEV DT -- sub-mode keyword table and hand-rolled ParseFn/HandlerFn.
// ---------------------------------------------------------------------------
enum class DtMode : uint8_t { PORTS, VW, WHEELS, NEUTRAL, STATE, STOP, CFG };

bool dtModeFromToken(const char* tok, DtMode* mode) {
    if (strcmp(tok, "PORTS") == 0)   { *mode = DtMode::PORTS;   return true; }
    if (strcmp(tok, "VW") == 0)      { *mode = DtMode::VW;      return true; }
    if (strcmp(tok, "WHEELS") == 0)  { *mode = DtMode::WHEELS;  return true; }
    if (strcmp(tok, "NEUTRAL") == 0) { *mode = DtMode::NEUTRAL; return true; }
    if (strcmp(tok, "STATE") == 0)   { *mode = DtMode::STATE;   return true; }
    if (strcmp(tok, "STOP") == 0)    { *mode = DtMode::STOP;    return true; }
    if (strcmp(tok, "CFG") == 0)     { *mode = DtMode::CFG;     return true; }
    return false;
}

// parseDevDt -- argTokens after the "DEV DT" prefix is stripped. Unaffected
// by this rewrite -- pure parsing, no state (see dev_commands.h's Open
// Question 3 note).
ParseResult parseDevDt(const char* const* tokens, int ntokens,
                       const KVPair* kvs, int nkv) {
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
        case DtMode::CFG: {
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
        case DtMode::STATE:
        case DtMode::STOP:
        default:
            break;
    }

    res.args.count = count;
    res.args.suppliedCount = count;
    return res;
}

// handleDevDtCfg -- CFG delta: apply each supplied key onto a CANDIDATE
// msg::DrivetrainConfig seeded from bb.drivetrainConfig (087-006: replaces
// the old drivetrainConfigShadow), ERR badkey per unrecognized key, one OK
// ack line listing only the keys that actually applied (if any). On
// success, posts ONE Rt::ConfigDelta (kDrivetrain) to bb.configIn.
void handleDevDtCfg(Rt::Blackboard& b, const ArgList& args,
                    const char* corrId, ReplyFn replyFn, void* replyCtx) {
    msg::DrivetrainConfig cfg = b.drivetrainConfig;
    uint64_t mask = 0;
    char appliedBody[256];
    int bodyLen = 0;
    appliedBody[0] = '\0';
    bool anyApplied = false;

    for (int i = 1; i < args.count; ++i) {
        const char* kvtok = args.args[i].sval;
        const char* eq = strchr(kvtok, '=');
        if (!eq) continue;   // shouldn't happen -- parseDevDt only packs "key=value"

        char key[24];
        int klen = static_cast<int>(eq - kvtok);
        if (klen >= static_cast<int>(sizeof(key))) klen = sizeof(key) - 1;
        memcpy(key, kvtok, static_cast<size_t>(klen));
        key[klen] = '\0';
        const char* value = eq + 1;

        char oneApplied[40];
        if (applyDrivetrainCfgKey(cfg, mask, key, value, oneApplied, sizeof(oneApplied))) {
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
        Rt::ConfigDelta delta;
        delta.target = Rt::ConfigDelta::kDrivetrain;
        delta.mask = mask;
        delta.drivetrain = cfg;
        b.configIn.post(delta);

        char rbuf[300];
        CommandProcessor::replyOKf(rbuf, sizeof(rbuf), "DEV DT", corrId, replyFn, replyCtx,
                                   "%s", appliedBody);
    }
}

void handleDevDt(const ArgList& args, const char* corrId,
                 ReplyFn replyFn, void* replyCtx, void* handlerCtx) {
    Rt::Blackboard& b = bb(handlerCtx);
    DtMode mode;
    dtModeFromToken(args.args[0].sval, &mode);   // already validated by parseDevDt
    char rbuf[200];

    switch (mode) {
        case DtMode::PORTS: {
            // Config-plane, like every other CFG-shaped verb (087-006): a
            // Rt::ConfigDelta (kDrivetrain), field-masked to just left_port/
            // right_port, posted to bb.configIn for the Configurator to fold
            // and apply -- replaces the old direct
            // drivetrainConfigShadow-then-configure() call. The reply still
            // echoes the REQUESTED values (not a bb read-back), matching
            // today's wire text exactly.
            uint32_t left = static_cast<uint32_t>(args.args[1].ival);
            uint32_t right = static_cast<uint32_t>(args.args[2].ival);
            Rt::ConfigDelta delta;
            delta.target = Rt::ConfigDelta::kDrivetrain;
            delta.mask = Rt::bitOf(Rt::DrivetrainConfigField::kLeftPort) |
                         Rt::bitOf(Rt::DrivetrainConfigField::kRightPort);
            delta.drivetrain.left_port = left;
            delta.drivetrain.right_port = right;
            b.configIn.post(delta);
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
            // Command-plane: post, do not call drivetrain.apply() directly --
            // Subsystems::Drivetrain::tick() drains bb.driveIn once per pass
            // (see drivetrain.h's tick() doc comment). DEV DT ALWAYS posts,
            // unconditionally -- today's contract is "DEV DT verbs
            // (re)activate authority" (drivetrain.h's own "Authority
            // arbitration" section); Decision 1's authority gate belongs to
            // driveIn's OTHER producer (Planner's own output, drained by the
            // loop's routeOutputs, ticket 007), not to DEV DT's own posts --
            // preserved exactly, not changed by this rewrite.
            b.driveIn.post(cmd);
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
            b.driveIn.post(cmd);
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
            b.driveIn.post(cmd);
            CommandProcessor::replyOKf(rbuf, sizeof(rbuf), "DEV DT", corrId, replyFn, replyCtx,
                                       "neutral=%s", bc);
            break;
        }
        case DtMode::STATE:
            emitDrivetrainState(b, corrId, replyFn, replyCtx);
            break;
        case DtMode::STOP: {
            // (093/094 teardown) This family is unregistered/parked (see
            // dev_commands.h's file header) and no longer posts to a
            // hardware queue -- Rt::Blackboard's motorIn[]/motorResetIn[]
            // are gone (blackboard.h's file header). Only the Drivetrain
            // side remains: buildDrivetrainStop() (identical
            // {NEUTRAL, standby=true} shape as DEV STOP's).
            b.driveIn.post(buildDrivetrainStop(msg::Neutral::BRAKE));

            CommandProcessor::replyOK(rbuf, sizeof(rbuf), "DEV DT STOP", nullptr, corrId, replyFn, replyCtx);
            break;
        }
        case DtMode::CFG:
            handleDevDtCfg(b, args, corrId, replyFn, replyCtx);
            break;
    }
}

// ---------------------------------------------------------------------------
// DEV STATE -- aggregate: one line per motor (ports 1..4) + one drivetrain
// line. No authority change (a pure query).
// ---------------------------------------------------------------------------
void handleDevState(const ArgList& /*args*/, const char* corrId,
                    ReplyFn replyFn, void* replyCtx, void* handlerCtx) {
    Rt::Blackboard& b = bb(handlerCtx);
    for (uint32_t port = 1; port <= Rt::kPortCount; ++port) {
        emitMotorState(b.motors[port - 1], port, corrId, replyFn, replyCtx);
    }
    emitDrivetrainState(b, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// DEV STOP -- global: drivetrain idle, authority dropped. (093/094 teardown)
// This family is unregistered/parked (see dev_commands.h's file header) and
// no longer posts a broadcast neutral to hardware -- bb.hardwareBroadcastIn
// is gone along with the rest of Rt::Blackboard's motor/hardware inbound
// queues (blackboard.h's file header). Only the Drivetrain side remains.
// ---------------------------------------------------------------------------
void handleDevStop(const ArgList& /*args*/, const char* corrId,
                   ReplyFn replyFn, void* replyCtx, void* handlerCtx) {
    Rt::Blackboard& b = bb(handlerCtx);
    b.driveIn.post(buildDrivetrainStop(msg::Neutral::BRAKE));
    char rbuf[32];
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "DEV STOP", nullptr, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// DEV WD <window> -- set the serial-silence watchdog window. Posts the
// requested window to bb.devWatchdogWindowIn (087-006: the watchdog is
// loop-owned, not one of the Configurator's four targets -- see
// dev_commands.h's file header); the loop drains it into its own
// SerialSilenceWatchdog instance. The reply echoes the requested value
// directly, matching today's wire text exactly.
// ---------------------------------------------------------------------------
void handleDevWd(const ArgList& args, const char* corrId,
                 ReplyFn replyFn, void* replyCtx, void* handlerCtx) {
    Rt::Blackboard& b = bb(handlerCtx);
    uint32_t window = static_cast<uint32_t>(args.args[0].ival);
    b.devWatchdogWindowIn.post(window);
    char rbuf[48];
    CommandProcessor::replyOKf(rbuf, sizeof(rbuf), "DEV WD", corrId, replyFn, replyCtx,
                               "window=%u", static_cast<unsigned>(window));
}

}  // namespace

// ---------------------------------------------------------------------------
// buildBroadcastNeutral / buildDrivetrainStop -- see dev_commands.h. Used by
// the loop's watchdog-fire path (applies the result IMMEDIATELY, bypassing
// every bb queue) and, for buildDrivetrainStop() alone, by DEV STOP/DEV DT
// STOP's own bb.driveIn post above.
// ---------------------------------------------------------------------------
Hal::CommandProcessorToHardwareCommand buildBroadcastNeutral(msg::Neutral mode) {
    Hal::CommandProcessorToHardwareCommand cmd;
    cmd.allPorts = true;
    cmd.count = 0;
    msg::MotorCommand neutralCmd;
    neutralCmd.setNeutral(mode);
    cmd.addressed[0].command = neutralCmd;   // port unused for a broadcast
    return cmd;
}

msg::DrivetrainCommand buildDrivetrainStop(msg::Neutral mode) {
    msg::DrivetrainCommand cmd;
    cmd.setNeutral(mode);
    cmd.setStandby(true);
    return cmd;
}

// ---------------------------------------------------------------------------
// devCommands -- the DEV command table, bound to `router`.
// ---------------------------------------------------------------------------
std::vector<CommandDescriptor> devCommands(Rt::CommandRouter& router) {
    std::vector<CommandDescriptor> cmds;
    cmds.push_back(makeCmd("DEV M", parseDevM, handleDevM, &router,
                           "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE));
    cmds.push_back(makeCmd("DEV DT", parseDevDt, handleDevDt, &router,
                           "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE));
    cmds.push_back(makeCmd("DEV STATE", nullptr, handleDevState, &router,
                           "badarg", ForceReply::NONE, CMD_NONE));
    cmds.push_back(makeCmd("DEV STOP", nullptr, handleDevStop, &router,
                           "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE));
    cmds.push_back(makeSchemaCmd("DEV WD", &kDevWdSchema, handleDevWd, &router,
                                 "badarg", ForceReply::NONE, CMD_NONE));
    return cmds;
}

