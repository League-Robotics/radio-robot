// config_commands.cpp -- top-level SET/GET command family implementation.
// See config_commands.h for the full vocabulary, the approved key table
// (architecture-update.md (084) Decision 2), the atomic-SET design, and the
// ROBOT_DEV_BUILD gating rationale.
#include "commands/config_commands.h"


#include "commands/command_processor.h"
#include "commands/arg_parse.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <math.h>

namespace {

Rt::Blackboard& bb(void* handlerCtx) { return static_cast<Rt::CommandRouter*>(handlerCtx)->blackboard(); }

// ---------------------------------------------------------------------------
// formatFixed -- render `value` as a fixed-point decimal string with exactly
// `decimals` digits after the point, using only integer arithmetic -- NOT
// "%f"/"%.Nf" (CODAL/newlib-nano has no float-conversion printf support).
// Duplicated here, not shared, matching this codebase's existing per-file
// convention (see dev_commands.cpp's own formatFixed()).
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
// parseFloatStrict / parseLongStrict -- docs/protocol-v2.md §7's "SET accepts
// float text for float keys ... Trailing non-numeric characters or empty
// values are rejected with ERR badval <key>." Unaffected by this rewrite.
// ---------------------------------------------------------------------------
bool parseFloatStrict(const char* value, float* out) {
    if (!value || value[0] == '\0') return false;
    char* endp = nullptr;
    float v = strtof(value, &endp);
    if (endp == value || *endp != '\0') return false;
    *out = v;
    return true;
}

bool parseLongStrict(const char* value, long* out) {
    if (!value || value[0] == '\0') return false;
    char* endp = nullptr;
    long v = strtol(value, &endp, 10);
    if (endp == value || *endp != '\0') return false;
    *out = v;
    return true;
}

// ---------------------------------------------------------------------------
// ConfigCandidate -- a candidate-config bundle: one copy of every published
// bb cell a SET line's keys might touch, plus a touched* flag and a
// per-target field mask (087-006: replaces the pre-087 shadow's direct
// motor.configure()/drivetrain.configure() calls -- a mask is now needed so
// the ACCEPTED path can build one field-masked Rt::ConfigDelta per touched
// target instead of calling configure() itself). sTimeoutRaw is a SIGNED
// long, not the target uint32_t, so a negative parsed value is caught by
// validateCandidate() below rather than silently wrapping to a huge window.
// ---------------------------------------------------------------------------
struct ConfigCandidate {
    msg::DrivetrainConfig dt;
    msg::MotorConfig left;
    msg::MotorConfig right;
    msg::PlannerConfig planner;
    long sTimeoutRaw = 0;

    bool touchedDt = false;
    bool touchedLeft = false;
    bool touchedRight = false;
    bool touchedPlanner = false;
    bool touchedSTimeout = false;

    uint64_t dtMask = 0;
    uint64_t leftMask = 0;
    uint64_t rightMask = 0;
    uint64_t plannerMask = 0;
};

enum class KeyResult : uint8_t { OK, UNKNOWN, BADVAL };

// ---------------------------------------------------------------------------
// applyConfigKey -- one key=value delta onto a ConfigCandidate. Returns
// UNKNOWN for an unregistered key (caller replies ERR badkey) or BADVAL for
// a non-numeric/empty value (caller replies ERR badval <key>, no value
// shown). On OK, writes the applied "key=value" text into appliedOut and
// sets the matching touched*/*, *Mask flag(s) -- pid.* touches BOTH left and
// right identically, per the approved key table.
// ---------------------------------------------------------------------------
KeyResult applyConfigKey(ConfigCandidate& cand, const char* key, const char* value,
                         char* appliedOut, int appliedOutSize) {
    char numStr[16];

    if (strcmp(key, "tw") == 0) {
        long v;
        if (!parseLongStrict(value, &v)) return KeyResult::BADVAL;
        cand.dt.trackwidth = static_cast<float>(v);
        cand.touchedDt = true;
        cand.dtMask |= Rt::bitOf(Rt::DrivetrainConfigField::kTrackwidth);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "tw=%ld", v);
        return KeyResult::OK;
    }
    if (strcmp(key, "ml") == 0) {
        float v;
        if (!parseFloatStrict(value, &v)) return KeyResult::BADVAL;
        cand.left.travel_calib = v;
        cand.touchedLeft = true;
        cand.leftMask |= Rt::bitOf(Rt::MotorConfigField::kTravelCalib);
        formatFixed(numStr, sizeof(numStr), v, 3);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "ml=%s", numStr);
        return KeyResult::OK;
    }
    if (strcmp(key, "mr") == 0) {
        float v;
        if (!parseFloatStrict(value, &v)) return KeyResult::BADVAL;
        cand.right.travel_calib = v;
        cand.touchedRight = true;
        cand.rightMask |= Rt::bitOf(Rt::MotorConfigField::kTravelCalib);
        formatFixed(numStr, sizeof(numStr), v, 3);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "mr=%s", numStr);
        return KeyResult::OK;
    }
    if (strcmp(key, "pid.kp") == 0) {
        float v;
        if (!parseFloatStrict(value, &v)) return KeyResult::BADVAL;
        cand.left.vel_gains.kp = v;
        cand.right.vel_gains.kp = v;
        cand.touchedLeft = true;
        cand.touchedRight = true;
        cand.leftMask |= Rt::bitOf(Rt::MotorConfigField::kVelGainsKp);
        cand.rightMask |= Rt::bitOf(Rt::MotorConfigField::kVelGainsKp);
        formatFixed(numStr, sizeof(numStr), v, 3);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "pid.kp=%s", numStr);
        return KeyResult::OK;
    }
    if (strcmp(key, "pid.ki") == 0) {
        float v;
        if (!parseFloatStrict(value, &v)) return KeyResult::BADVAL;
        cand.left.vel_gains.ki = v;
        cand.right.vel_gains.ki = v;
        cand.touchedLeft = true;
        cand.touchedRight = true;
        cand.leftMask |= Rt::bitOf(Rt::MotorConfigField::kVelGainsKi);
        cand.rightMask |= Rt::bitOf(Rt::MotorConfigField::kVelGainsKi);
        formatFixed(numStr, sizeof(numStr), v, 3);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "pid.ki=%s", numStr);
        return KeyResult::OK;
    }
    if (strcmp(key, "pid.kff") == 0) {
        float v;
        if (!parseFloatStrict(value, &v)) return KeyResult::BADVAL;
        cand.left.vel_gains.kff = v;
        cand.right.vel_gains.kff = v;
        cand.touchedLeft = true;
        cand.touchedRight = true;
        cand.leftMask |= Rt::bitOf(Rt::MotorConfigField::kVelGainsKff);
        cand.rightMask |= Rt::bitOf(Rt::MotorConfigField::kVelGainsKff);
        formatFixed(numStr, sizeof(numStr), v, 3);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "pid.kff=%s", numStr);
        return KeyResult::OK;
    }
    if (strcmp(key, "pid.iMax") == 0) {
        float v;
        if (!parseFloatStrict(value, &v)) return KeyResult::BADVAL;
        cand.left.vel_gains.i_max = v;
        cand.right.vel_gains.i_max = v;
        cand.touchedLeft = true;
        cand.touchedRight = true;
        cand.leftMask |= Rt::bitOf(Rt::MotorConfigField::kVelGainsIMax);
        cand.rightMask |= Rt::bitOf(Rt::MotorConfigField::kVelGainsIMax);
        formatFixed(numStr, sizeof(numStr), v, 3);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "pid.iMax=%s", numStr);
        return KeyResult::OK;
    }
    if (strcmp(key, "pid.kaw") == 0) {
        float v;
        if (!parseFloatStrict(value, &v)) return KeyResult::BADVAL;
        cand.left.vel_gains.kaw = v;
        cand.right.vel_gains.kaw = v;
        cand.touchedLeft = true;
        cand.touchedRight = true;
        cand.leftMask |= Rt::bitOf(Rt::MotorConfigField::kVelGainsKaw);
        cand.rightMask |= Rt::bitOf(Rt::MotorConfigField::kVelGainsKaw);
        formatFixed(numStr, sizeof(numStr), v, 3);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "pid.kaw=%s", numStr);
        return KeyResult::OK;
    }
    if (strcmp(key, "rotSlip") == 0) {
        float v;
        if (!parseFloatStrict(value, &v)) return KeyResult::BADVAL;
        cand.dt.rotational_slip = v;
        cand.touchedDt = true;
        cand.dtMask |= Rt::bitOf(Rt::DrivetrainConfigField::kRotationalSlip);
        formatFixed(numStr, sizeof(numStr), v, 3);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "rotSlip=%s", numStr);
        return KeyResult::OK;
    }
    if (strcmp(key, "ekfQxy") == 0) {
        float v;
        if (!parseFloatStrict(value, &v)) return KeyResult::BADVAL;
        cand.dt.ekf_q_xy = v;
        cand.touchedDt = true;
        cand.dtMask |= Rt::bitOf(Rt::DrivetrainConfigField::kEkfQXy);
        formatFixed(numStr, sizeof(numStr), v, 3);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "ekfQxy=%s", numStr);
        return KeyResult::OK;
    }
    if (strcmp(key, "ekfQtheta") == 0) {
        float v;
        if (!parseFloatStrict(value, &v)) return KeyResult::BADVAL;
        cand.dt.ekf_q_theta = v;
        cand.touchedDt = true;
        cand.dtMask |= Rt::bitOf(Rt::DrivetrainConfigField::kEkfQTheta);
        formatFixed(numStr, sizeof(numStr), v, 3);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "ekfQtheta=%s", numStr);
        return KeyResult::OK;
    }
    if (strcmp(key, "ekfROtosXy") == 0) {
        float v;
        if (!parseFloatStrict(value, &v)) return KeyResult::BADVAL;
        cand.dt.ekf_r_otos_xy = v;
        cand.touchedDt = true;
        cand.dtMask |= Rt::bitOf(Rt::DrivetrainConfigField::kEkfROtosXy);
        formatFixed(numStr, sizeof(numStr), v, 3);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "ekfROtosXy=%s", numStr);
        return KeyResult::OK;
    }
    if (strcmp(key, "ekfROtosTheta") == 0) {
        float v;
        if (!parseFloatStrict(value, &v)) return KeyResult::BADVAL;
        cand.dt.ekf_r_otos_theta = v;
        cand.touchedDt = true;
        cand.dtMask |= Rt::bitOf(Rt::DrivetrainConfigField::kEkfROtosTheta);
        formatFixed(numStr, sizeof(numStr), v, 3);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "ekfROtosTheta=%s", numStr);
        return KeyResult::OK;
    }
    if (strcmp(key, "minSpeed") == 0) {
        long v;
        if (!parseLongStrict(value, &v)) return KeyResult::BADVAL;
        cand.planner.min_speed = static_cast<float>(v);
        cand.touchedPlanner = true;
        cand.plannerMask |= Rt::bitOf(Rt::PlannerConfigField::kMinSpeed);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "minSpeed=%ld", v);
        return KeyResult::OK;
    }
    if (strcmp(key, "sTimeout") == 0) {
        long v;
        if (!parseLongStrict(value, &v)) return KeyResult::BADVAL;
        cand.sTimeoutRaw = v;
        cand.touchedSTimeout = true;
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "sTimeout=%ld", v);
        return KeyResult::OK;
    }

    return KeyResult::UNKNOWN;
}

// ---------------------------------------------------------------------------
// validateCandidate -- invariant checks run ONCE, after every key in a SET
// line has parsed successfully, against the FULL candidate. Unaffected by
// this rewrite.
// ---------------------------------------------------------------------------
bool validateCandidate(const ConfigCandidate& cand, const char** badKeyOut,
                        char* detailOut, int detailOutSize) {
    if (cand.touchedDt && cand.dt.trackwidth <= 0.0f) {
        *badKeyOut = "tw";
        snprintf(detailOut, static_cast<size_t>(detailOutSize), "tw=%d",
                 static_cast<int>(cand.dt.trackwidth));
        return false;
    }
    if (cand.touchedDt) {
        float slip = cand.dt.rotational_slip;
        bool slipOk = (slip == 0.0f) || (slip >= 0.5f && slip <= 1.0f);
        if (!slipOk) {
            *badKeyOut = "rotSlip";
            char numStr[16];
            formatFixed(numStr, sizeof(numStr), slip, 3);
            snprintf(detailOut, static_cast<size_t>(detailOutSize), "rotSlip=%s", numStr);
            return false;
        }
    }
    if (cand.touchedSTimeout && cand.sTimeoutRaw <= 0) {
        *badKeyOut = "sTimeout";
        snprintf(detailOut, static_cast<size_t>(detailOutSize), "sTimeout=%ld", cand.sTimeoutRaw);
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// kAllKeys -- the complete, ordered list of this file's registered keys, used
// by GET's "dump all" path. Unaffected by this rewrite.
// ---------------------------------------------------------------------------
const char* const kAllKeys[] = {
    "tw", "ml", "mr",
    "pid.kp", "pid.ki", "pid.kff", "pid.iMax", "pid.kaw",
    "rotSlip",
    "ekfQxy", "ekfQtheta", "ekfROtosXy", "ekfROtosTheta",
    "minSpeed", "sTimeout",
};
const int kAllKeysCount = static_cast<int>(sizeof(kAllKeys) / sizeof(kAllKeys[0]));

// ---------------------------------------------------------------------------
// formatConfigKeyFromBb -- format one registered key's CURRENT PUBLISHED
// value (bb.drivetrainConfig/bb.motorConfig[]/bb.plannerConfig/
// bb.streamWatchdogWindow -- 087-006: replaces the pre-087 shadow read)
// as "key=value" into outBuf. leftIdx/rightIdx (0-based Hardware motor
// indices) are read ONCE by the caller (handleGet), converted from the
// bound pair's wire-shaped bb.drivetrainConfig.left_port/right_port at that
// caller's own boundary, so every key in one GET line/dump resolves against
// the SAME bound pair. Returns false for an unrecognized key (caller
// replies ERR badkey). pid.* reads the LEFT bound motor's published config
// -- SET always applies the same value to both bound motors identically
// (applyConfigKey above), so left/right can only disagree if something
// OUTSIDE this file's SET (e.g. `DEV M <n> CFG kp=...`) wrote just one side.
// ---------------------------------------------------------------------------
bool formatConfigKeyFromBb(const Rt::Blackboard& b, uint32_t leftIdx, uint32_t rightIdx,
                            const char* key, char* outBuf, int outBufSize) {
    char numStr[16];
    const msg::MotorConfig& left = b.motorConfig[leftIdx];
    const msg::MotorConfig& right = b.motorConfig[rightIdx];

    if (strcmp(key, "tw") == 0) {
        snprintf(outBuf, static_cast<size_t>(outBufSize), "tw=%d",
                 static_cast<int>(b.drivetrainConfig.trackwidth));
        return true;
    }
    if (strcmp(key, "ml") == 0) {
        formatFixed(numStr, sizeof(numStr), left.travel_calib, 3);
        snprintf(outBuf, static_cast<size_t>(outBufSize), "ml=%s", numStr);
        return true;
    }
    if (strcmp(key, "mr") == 0) {
        formatFixed(numStr, sizeof(numStr), right.travel_calib, 3);
        snprintf(outBuf, static_cast<size_t>(outBufSize), "mr=%s", numStr);
        return true;
    }
    if (strcmp(key, "pid.kp") == 0) {
        formatFixed(numStr, sizeof(numStr), left.vel_gains.kp, 3);
        snprintf(outBuf, static_cast<size_t>(outBufSize), "pid.kp=%s", numStr);
        return true;
    }
    if (strcmp(key, "pid.ki") == 0) {
        formatFixed(numStr, sizeof(numStr), left.vel_gains.ki, 3);
        snprintf(outBuf, static_cast<size_t>(outBufSize), "pid.ki=%s", numStr);
        return true;
    }
    if (strcmp(key, "pid.kff") == 0) {
        formatFixed(numStr, sizeof(numStr), left.vel_gains.kff, 3);
        snprintf(outBuf, static_cast<size_t>(outBufSize), "pid.kff=%s", numStr);
        return true;
    }
    if (strcmp(key, "pid.iMax") == 0) {
        formatFixed(numStr, sizeof(numStr), left.vel_gains.i_max, 3);
        snprintf(outBuf, static_cast<size_t>(outBufSize), "pid.iMax=%s", numStr);
        return true;
    }
    if (strcmp(key, "pid.kaw") == 0) {
        formatFixed(numStr, sizeof(numStr), left.vel_gains.kaw, 3);
        snprintf(outBuf, static_cast<size_t>(outBufSize), "pid.kaw=%s", numStr);
        return true;
    }
    if (strcmp(key, "rotSlip") == 0) {
        formatFixed(numStr, sizeof(numStr), b.drivetrainConfig.rotational_slip, 3);
        snprintf(outBuf, static_cast<size_t>(outBufSize), "rotSlip=%s", numStr);
        return true;
    }
    if (strcmp(key, "ekfQxy") == 0) {
        formatFixed(numStr, sizeof(numStr), b.drivetrainConfig.ekf_q_xy, 3);
        snprintf(outBuf, static_cast<size_t>(outBufSize), "ekfQxy=%s", numStr);
        return true;
    }
    if (strcmp(key, "ekfQtheta") == 0) {
        formatFixed(numStr, sizeof(numStr), b.drivetrainConfig.ekf_q_theta, 3);
        snprintf(outBuf, static_cast<size_t>(outBufSize), "ekfQtheta=%s", numStr);
        return true;
    }
    if (strcmp(key, "ekfROtosXy") == 0) {
        formatFixed(numStr, sizeof(numStr), b.drivetrainConfig.ekf_r_otos_xy, 3);
        snprintf(outBuf, static_cast<size_t>(outBufSize), "ekfROtosXy=%s", numStr);
        return true;
    }
    if (strcmp(key, "ekfROtosTheta") == 0) {
        formatFixed(numStr, sizeof(numStr), b.drivetrainConfig.ekf_r_otos_theta, 3);
        snprintf(outBuf, static_cast<size_t>(outBufSize), "ekfROtosTheta=%s", numStr);
        return true;
    }
    if (strcmp(key, "minSpeed") == 0) {
        snprintf(outBuf, static_cast<size_t>(outBufSize), "minSpeed=%d",
                 static_cast<int>(b.plannerConfig.min_speed));
        return true;
    }
    if (strcmp(key, "sTimeout") == 0) {
        snprintf(outBuf, static_cast<size_t>(outBufSize), "sTimeout=%u",
                 static_cast<unsigned>(b.streamWatchdogWindow));
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// SET -- custom ParseFn reading kvs[] directly. Unaffected by this rewrite.
// ---------------------------------------------------------------------------
ParseResult parseSet(const char* const* /*tokens*/, int /*ntokens*/,
                      const KVPair* kvs, int nkv) {
    ParseResult r;
    if (nkv == 0) {
        r.ok = false;
        r.err.code = "badarg";
        r.err.detail = "no key=value pairs";
        return r;
    }
    r.ok = true;
    int n = (nkv > MAX_ARGS) ? MAX_ARGS : nkv;
    r.args.count = 0;
    for (int i = 0; i < n; ++i) {
        if (!kvs[i].key) continue;
        char kvbuf[40];
        snprintf(kvbuf, sizeof(kvbuf), "%s=%s", kvs[i].key, kvs[i].value ? kvs[i].value : "");
        argStr(r.args.args[r.args.count], kvbuf);
        ++r.args.count;
    }
    r.args.suppliedCount = r.args.count;
    return r;
}

// ---------------------------------------------------------------------------
// handleSet -- see config_commands.h's file header for the full atomic-SET
// design (candidate-then-commit, per-target field masks).
// ---------------------------------------------------------------------------
void handleSet(const ArgList& args, const char* corrId,
               ReplyFn replyFn, void* replyCtx, void* handlerCtx) {
    Rt::Blackboard& b = bb(handlerCtx);

    // Read the CURRENTLY bound pair at SET-time, not a hardcoded port --
    // see config_commands.h's file header. THE conversion boundary
    // (0-based motor indices, OOP refactor): bb.drivetrainConfig.left_port/
    // right_port are wire/serialized 1-based labels (msg::DrivetrainConfig,
    // unchanged) -- converted to 0-based Hardware motor indices here, once,
    // for this handler's own use; every bb.motorConfig[]/ConfigDelta::port
    // access below uses leftIdx/rightIdx, never leftPort/rightPort.
    uint32_t leftPort = b.drivetrainConfig.left_port;
    uint32_t rightPort = b.drivetrainConfig.right_port;
    uint32_t leftIdx = leftPort - 1;
    uint32_t rightIdx = rightPort - 1;

    ConfigCandidate cand;
    cand.dt = b.drivetrainConfig;
    cand.left = b.motorConfig[leftIdx];
    cand.right = b.motorConfig[rightIdx];
    cand.planner = b.plannerConfig;
    cand.sTimeoutRaw = static_cast<long>(b.streamWatchdogWindow);

    char applied[300];
    int appliedLen = 0;
    applied[0] = '\0';
    bool anyError = false;

    for (int i = 0; i < args.count; ++i) {
        const char* kvtok = args.args[i].sval;
        const char* eq = strchr(kvtok, '=');
        if (!eq) continue;   // shouldn't happen -- parseSet only packs "key=value"

        char key[24];
        int klen = static_cast<int>(eq - kvtok);
        if (klen >= static_cast<int>(sizeof(key))) klen = sizeof(key) - 1;
        memcpy(key, kvtok, static_cast<size_t>(klen));
        key[klen] = '\0';
        const char* value = eq + 1;

        char oneApplied[40];
        KeyResult r = applyConfigKey(cand, key, value, oneApplied, sizeof(oneApplied));
        char rbuf[64];
        if (r == KeyResult::UNKNOWN) {
            CommandProcessor::replyErr(rbuf, sizeof(rbuf), "badkey", key, corrId, replyFn, replyCtx);
            anyError = true;
        } else if (r == KeyResult::BADVAL) {
            CommandProcessor::replyErr(rbuf, sizeof(rbuf), "badval", key, corrId, replyFn, replyCtx);
            anyError = true;
        } else {
            if (appliedLen > 0 && appliedLen < static_cast<int>(sizeof(applied)) - 1) {
                applied[appliedLen++] = ' ';
            }
            int n = snprintf(applied + appliedLen, sizeof(applied) - static_cast<size_t>(appliedLen),
                             "%s", oneApplied);
            if (n > 0) appliedLen += n;
        }
    }

    // Any unknown key or parse failure -- do not validate, do not post
    // anything, no OK reply (atomic all-or-nothing).
    if (anyError) return;
    if (appliedLen == 0) return;   // no "key=value" tokens were ever supplied

    const char* badKey = nullptr;
    char detail[48];
    if (!validateCandidate(cand, &badKey, detail, sizeof(detail))) {
        char rbuf[80];
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "badval", detail, corrId, replyFn, replyCtx);
        return;
    }

    // Validation passed -- post one Rt::ConfigDelta per touched target
    // (the Configurator folds+applies each); posting is the ONLY effect
    // (087-006 replaces the old direct configure() calls).
    if (cand.touchedDt) {
        Rt::ConfigDelta delta;
        delta.target = Rt::ConfigDelta::kDrivetrain;
        delta.mask = cand.dtMask;
        delta.drivetrain = cand.dt;
        b.configIn.post(delta);
    }
    if (cand.touchedLeft) {
        Rt::ConfigDelta delta;
        delta.target = Rt::ConfigDelta::kMotor;
        delta.port = leftIdx;   // 0-based index (commands.h's ConfigDelta::port contract)
        delta.mask = cand.leftMask;
        delta.motor = cand.left;
        b.configIn.post(delta);
    }
    if (cand.touchedRight) {
        Rt::ConfigDelta delta;
        delta.target = Rt::ConfigDelta::kMotor;
        delta.port = rightIdx;   // 0-based index (commands.h's ConfigDelta::port contract)
        delta.mask = cand.rightMask;
        delta.motor = cand.right;
        b.configIn.post(delta);
    }
    if (cand.touchedPlanner) {
        Rt::ConfigDelta delta;
        delta.target = Rt::ConfigDelta::kPlanner;
        delta.mask = cand.plannerMask;
        delta.planner = cand.planner;
        b.configIn.post(delta);
    }
    if (cand.touchedSTimeout) {
        b.streamWatchdogWindowIn.post(static_cast<uint32_t>(cand.sTimeoutRaw));
    }

    char rbuf[360];
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "set", applied, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// GET -- variadic ArgSchema: each bare token becomes args[i].sval (a
// requested key name). No args (bare `GET`) -> dump every registered key.
// ---------------------------------------------------------------------------
const ArgSchema kGetSchema = { nullptr, 0, 0, true, nullptr };

void handleGet(const ArgList& args, const char* corrId,
               ReplyFn replyFn, void* replyCtx, void* handlerCtx) {
    Rt::Blackboard& b = bb(handlerCtx);
    // Read once, at GET-time, so every key in this one reply resolves
    // against the SAME bound pair -- see config_commands.h's file header.
    // THE conversion boundary (0-based motor indices, OOP refactor):
    // bb.drivetrainConfig.left_port/right_port are wire/serialized 1-based
    // labels -- converted to 0-based Hardware motor indices here, once;
    // formatConfigKeyFromBb() below takes the already-converted indices.
    uint32_t leftIdx = b.drivetrainConfig.left_port - 1;
    uint32_t rightIdx = b.drivetrainConfig.right_port - 1;

    char line[400];
    int pos = 0;
    int rem = static_cast<int>(sizeof(line));
    int n = snprintf(line + pos, static_cast<size_t>(rem), "CFG");
    if (n > 0 && n < rem) { pos += n; rem -= n; }

    if (args.count == 0) {
        // Dump all registered keys, in kAllKeys order.
        for (int i = 0; i < kAllKeysCount && rem > 2; ++i) {
            char kv[40];
            formatConfigKeyFromBb(b, leftIdx, rightIdx, kAllKeys[i], kv, sizeof(kv));
            line[pos++] = ' '; --rem;
            int w = snprintf(line + pos, static_cast<size_t>(rem), "%s", kv);
            if (w > 0 && w < rem) { pos += w; rem -= w; }
        }
    } else {
        for (int t = 0; t < args.count && rem > 2; ++t) {
            const char* reqKey = args.args[t].sval;
            char kv[40];
            if (formatConfigKeyFromBb(b, leftIdx, rightIdx, reqKey, kv, sizeof(kv))) {
                line[pos++] = ' '; --rem;
                int w = snprintf(line + pos, static_cast<size_t>(rem), "%s", kv);
                if (w > 0 && w < rem) { pos += w; rem -= w; }
            } else {
                char rbuf[48];
                CommandProcessor::replyErr(rbuf, sizeof(rbuf), "badkey", reqKey, corrId, replyFn, replyCtx);
            }
        }
    }

    if (corrId && corrId[0] != '\0' && rem > 3) {
        int w = snprintf(line + pos, static_cast<size_t>(rem), " #%s", corrId);
        if (w > 0 && w < rem) { pos += w; rem -= w; }
    }
    line[pos] = '\0';
    replyFn(line, replyCtx);
}

}  // namespace

// ---------------------------------------------------------------------------
// configCommands -- the SET/GET command table, bound to `router`.
// ---------------------------------------------------------------------------
std::vector<CommandDescriptor> configCommands(Rt::CommandRouter& router) {
    std::vector<CommandDescriptor> cmds;
    cmds.push_back(makeSchemaCmd("GET", &kGetSchema, handleGet, &router,
                                 "badkey", ForceReply::NONE, CMD_NONE));
    cmds.push_back(makeCmd("SET", parseSet, handleSet, &router,
                           "badkey", ForceReply::NONE, CMD_ACCESS_HARDWARE));
    return cmds;
}

