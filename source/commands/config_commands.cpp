// config_commands.cpp -- top-level SET/GET command family implementation.
// See config_commands.h for the full vocabulary, the approved key table
// (architecture-update.md (084) Decision 2), the atomic-SET design, and the
// ROBOT_DEV_BUILD gating rationale.
#include "commands/config_commands.h"

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
// `decimals` digits after the point, using only integer arithmetic -- NOT
// "%f"/"%.Nf". CODAL/newlib-nano's printf family has no float-conversion
// support on this toolchain (no -u _printf_float) -- a "%f" specifier
// silently emits nothing. Same technique, same rationale as
// dev_commands.cpp's own formatFixed() (duplicated here, not shared,
// matching this codebase's existing per-file convention -- there is no
// shared formatting helper today).
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
// float text for float keys (strtof with end-pointer validation) and integer
// text for int and float-as-int keys (strtol base-10 with end-pointer
// validation). Trailing non-numeric characters or empty values are rejected
// with ERR badval <key>." Both return false on empty/non-numeric text --
// the caller reports that as KeyResult::BADVAL with no value shown (a PARSE
// failure, distinct from an invariant/range failure caught later by
// validateCandidate(), which DOES show the candidate value).
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
// ConfigCandidate -- a candidate-config bundle: one copy of every shadow a
// SET line's keys might touch, plus a touched* flag per shadow so handleSet
// knows which shadows to copy back and which subsystems'/motors'
// configure() to call on a successful commit. sTimeoutRaw is a SIGNED long,
// not the target uint32_t, so a negative parsed value is caught by
// validateCandidate() below rather than silently wrapping to a huge window
// the moment it is cast to unsigned.
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
};

enum class KeyResult : uint8_t { OK, UNKNOWN, BADVAL };

// ---------------------------------------------------------------------------
// applyConfigKey -- one key=value delta onto a ConfigCandidate. Returns
// UNKNOWN for an unregistered key (caller replies ERR badkey) or BADVAL for
// a non-numeric/empty value (caller replies ERR badval <key>, no value
// shown -- a parse failure, per docs/protocol-v2.md §7). On OK, writes the
// applied "key=value" text (as actually parsed, matching the wire format
// each key documents) into appliedOut and sets the matching touched* flag(s)
// -- pid.* touches BOTH left and right (applied identically to both bound
// motors' Gains, per the approved key table).
// ---------------------------------------------------------------------------
KeyResult applyConfigKey(ConfigCandidate& cand, const char* key, const char* value,
                         char* appliedOut, int appliedOutSize) {
    char numStr[16];

    if (strcmp(key, "tw") == 0) {
        long v;
        if (!parseLongStrict(value, &v)) return KeyResult::BADVAL;
        cand.dt.trackwidth = static_cast<float>(v);
        cand.touchedDt = true;
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "tw=%ld", v);
        return KeyResult::OK;
    }
    if (strcmp(key, "ml") == 0) {
        float v;
        if (!parseFloatStrict(value, &v)) return KeyResult::BADVAL;
        cand.left.travel_calib = v;
        cand.touchedLeft = true;
        formatFixed(numStr, sizeof(numStr), v, 3);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "ml=%s", numStr);
        return KeyResult::OK;
    }
    if (strcmp(key, "mr") == 0) {
        float v;
        if (!parseFloatStrict(value, &v)) return KeyResult::BADVAL;
        cand.right.travel_calib = v;
        cand.touchedRight = true;
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
        formatFixed(numStr, sizeof(numStr), v, 3);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "pid.kaw=%s", numStr);
        return KeyResult::OK;
    }
    if (strcmp(key, "rotSlip") == 0) {
        float v;
        if (!parseFloatStrict(value, &v)) return KeyResult::BADVAL;
        cand.dt.rotational_slip = v;
        cand.touchedDt = true;
        formatFixed(numStr, sizeof(numStr), v, 3);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "rotSlip=%s", numStr);
        return KeyResult::OK;
    }
    if (strcmp(key, "ekfQxy") == 0) {
        float v;
        if (!parseFloatStrict(value, &v)) return KeyResult::BADVAL;
        cand.dt.ekf_q_xy = v;
        cand.touchedDt = true;
        formatFixed(numStr, sizeof(numStr), v, 3);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "ekfQxy=%s", numStr);
        return KeyResult::OK;
    }
    if (strcmp(key, "ekfQtheta") == 0) {
        float v;
        if (!parseFloatStrict(value, &v)) return KeyResult::BADVAL;
        cand.dt.ekf_q_theta = v;
        cand.touchedDt = true;
        formatFixed(numStr, sizeof(numStr), v, 3);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "ekfQtheta=%s", numStr);
        return KeyResult::OK;
    }
    if (strcmp(key, "ekfROtosXy") == 0) {
        float v;
        if (!parseFloatStrict(value, &v)) return KeyResult::BADVAL;
        cand.dt.ekf_r_otos_xy = v;
        cand.touchedDt = true;
        formatFixed(numStr, sizeof(numStr), v, 3);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "ekfROtosXy=%s", numStr);
        return KeyResult::OK;
    }
    if (strcmp(key, "ekfROtosTheta") == 0) {
        float v;
        if (!parseFloatStrict(value, &v)) return KeyResult::BADVAL;
        cand.dt.ekf_r_otos_theta = v;
        cand.touchedDt = true;
        formatFixed(numStr, sizeof(numStr), v, 3);
        snprintf(appliedOut, static_cast<size_t>(appliedOutSize), "ekfROtosTheta=%s", numStr);
        return KeyResult::OK;
    }
    if (strcmp(key, "minSpeed") == 0) {
        long v;
        if (!parseLongStrict(value, &v)) return KeyResult::BADVAL;
        cand.planner.min_speed = static_cast<float>(v);
        cand.touchedPlanner = true;
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
// line has parsed successfully, against the FULL candidate. Only checks
// invariants whose violation causes a known downstream failure (docs/
// protocol-v2.md §7's "Validated invariants" table, scoped to the keys this
// file actually registers):
//
//   tw > 0        -- division by zero in BodyKinematics/odometry arc-heading
//                    math (Drivetrain::commandedWheelTargets(),
//                    PoseEstimator's dead-reckoning kinematics).
//   rotSlip       -- "Nonsensical arc estimates break odometry." 0.0f is the
//                    documented unset sentinel (PoseEstimator::
//                    effectiveSlip() maps it to 1.0, no correction);
//                    [0.5, 1.0] is the calibrated range; negative, (0, 0.5),
//                    and > 1.0 are all rejected.
//   sTimeout > 0  -- a non-positive window either wraps to a huge uint32_t
//                    once cast (negative) or fires on every single tick
//                    (zero), silently defeating ticket 002's streaming-drive
//                    watchdog rather than tuning it.
//
// On failure, sets *badKeyOut to the failing key and formats "key=value"
// (the CANDIDATE's value, matching docs/protocol-v2.md §7's `ERR badval
// tw=0` example) into detailOut.
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
// kAllKeys -- the complete, ordered list of this file's registered keys
// (architecture-update.md (084) Decision 2's approved table), used by GET's
// "dump all" path (bare `GET`, no args). Keep in sync with applyConfigKey()/
// formatConfigKeyFromShadow()'s strcmp chains by hand -- this file's closed,
// 15-key surface is small enough that a generic table-driven mechanism (the
// old tree's offsetof-based kRegistry[], which needed one monolithic
// RobotConfig struct to take an offset against) buys nothing here, per
// architecture-update.md Decision 2's own rationale for why that mechanism
// has no equivalent in this per-component message tree.
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
// formatConfigKeyFromShadow -- format one registered key's CURRENT shadow
// value as "key=value" into outBuf. `ports` is read ONCE by the caller
// (handleGet) so every key in one GET line/dump resolves against the SAME
// bound pair. Returns false for an unrecognized key (caller replies
// ERR badkey). pid.* reads the LEFT bound motor's shadow -- SET always
// applies the same value to both bound motors identically (applyConfigKey
// above), so left/right can only disagree if something OUTSIDE this file's
// SET (e.g. `DEV M <n> CFG kp=...`) wrote just one side -- an accepted
// consequence of two independent config-plane shadows (config_commands.h's
// file header).
// ---------------------------------------------------------------------------
bool formatConfigKeyFromShadow(const ConfigCommandState& state, Subsystems::DrivetrainPorts ports,
                                const char* key, char* outBuf, int outBufSize) {
    char numStr[16];
    const msg::MotorConfig& left = state.motorShadow[ports.left - 1];
    const msg::MotorConfig& right = state.motorShadow[ports.right - 1];

    if (strcmp(key, "tw") == 0) {
        snprintf(outBuf, static_cast<size_t>(outBufSize), "tw=%d",
                 static_cast<int>(state.drivetrainShadow.trackwidth));
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
        formatFixed(numStr, sizeof(numStr), state.drivetrainShadow.rotational_slip, 3);
        snprintf(outBuf, static_cast<size_t>(outBufSize), "rotSlip=%s", numStr);
        return true;
    }
    if (strcmp(key, "ekfQxy") == 0) {
        formatFixed(numStr, sizeof(numStr), state.drivetrainShadow.ekf_q_xy, 3);
        snprintf(outBuf, static_cast<size_t>(outBufSize), "ekfQxy=%s", numStr);
        return true;
    }
    if (strcmp(key, "ekfQtheta") == 0) {
        formatFixed(numStr, sizeof(numStr), state.drivetrainShadow.ekf_q_theta, 3);
        snprintf(outBuf, static_cast<size_t>(outBufSize), "ekfQtheta=%s", numStr);
        return true;
    }
    if (strcmp(key, "ekfROtosXy") == 0) {
        formatFixed(numStr, sizeof(numStr), state.drivetrainShadow.ekf_r_otos_xy, 3);
        snprintf(outBuf, static_cast<size_t>(outBufSize), "ekfROtosXy=%s", numStr);
        return true;
    }
    if (strcmp(key, "ekfROtosTheta") == 0) {
        formatFixed(numStr, sizeof(numStr), state.drivetrainShadow.ekf_r_otos_theta, 3);
        snprintf(outBuf, static_cast<size_t>(outBufSize), "ekfROtosTheta=%s", numStr);
        return true;
    }
    if (strcmp(key, "minSpeed") == 0) {
        snprintf(outBuf, static_cast<size_t>(outBufSize), "minSpeed=%d",
                 static_cast<int>(state.plannerShadow.min_speed));
        return true;
    }
    if (strcmp(key, "sTimeout") == 0) {
        snprintf(outBuf, static_cast<size_t>(outBufSize), "sTimeout=%u",
                 static_cast<unsigned>(state.sTimeoutWatchdog->window()));
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// SET -- custom ParseFn reading kvs[] directly (not tokens[]), mirroring
// source_old/commands/ConfigCommands.cpp's parseSet(): re-serializes each
// "key=value" kv pair into a STR ArgList entry so handleSet (which only sees
// ArgList, not kvs) can recover them. `SET` with no key=value pairs at all
// -> ERR badarg "no key=value pairs" (docs/protocol-v2.md §7's own example),
// not ERR badkey.
//
// MAX_ARGS (10) caps how many key=value pairs one SET line can carry through
// to handleSet -- a known, project-wide limitation of this ArgList-repacking
// mechanism every kv-bearing command (DEV M/DEV DT CFG) already shares, not
// something new here (see .clasi/knowledge/simset-max-args-truncation.md).
// This file's own approved key table never requires more than a handful of
// keys in one SET line to satisfy its acceptance criteria.
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
// design (candidate-then-commit, per-shadow touched flags).
// ---------------------------------------------------------------------------
void handleSet(const ArgList& args, const char* corrId,
               ReplyFn replyFn, void* replyCtx, void* handlerCtx) {
    ConfigCommandState& state = *static_cast<ConfigCommandState*>(handlerCtx);

    // Read the CURRENTLY bound pair at SET-time, not a hardcoded port --
    // see config_commands.h's file header.
    Subsystems::DrivetrainPorts ports = state.drivetrain->ports();

    ConfigCandidate cand;
    cand.dt = state.drivetrainShadow;
    cand.left = state.motorShadow[ports.left - 1];
    cand.right = state.motorShadow[ports.right - 1];
    cand.planner = state.plannerShadow;
    cand.sTimeoutRaw = static_cast<long>(state.sTimeoutWatchdog->window());

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

    // Any unknown key or parse failure -- do not validate, do not commit,
    // no OK reply (mirrors source_old/robot/ConfigRegistry.cpp's handleSet:
    // "If any parse / badkey error occurred, do not commit -- cfg is
    // unchanged").
    if (anyError) return;
    if (appliedLen == 0) return;   // no "key=value" tokens were ever supplied

    const char* badKey = nullptr;
    char detail[48];
    if (!validateCandidate(cand, &badKey, detail, sizeof(detail))) {
        char rbuf[80];
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "badval", detail, corrId, replyFn, replyCtx);
        return;
    }

    // Validation passed -- commit atomically: copy each touched candidate
    // back into its shadow and re-propagate via the matching subsystem's/
    // motor's configure().
    if (cand.touchedDt) {
        state.drivetrainShadow = cand.dt;
        state.drivetrain->configure(cand.dt);
        // PoseEstimator::configure() takes the SAME msg::DrivetrainConfig
        // Drivetrain::configure() does -- mirrors main.cpp's/sim_api.cpp's
        // own boot wiring (one shared config source, both subsystems
        // configured). See config_commands.h's file header for the known
        // EKF-reset consequence.
        state.poseEstimator->configure(cand.dt);
    }
    if (cand.touchedLeft) {
        state.motorShadow[ports.left - 1] = cand.left;
        state.hardware->motor(ports.left).configure(cand.left);
    }
    if (cand.touchedRight) {
        state.motorShadow[ports.right - 1] = cand.right;
        state.hardware->motor(ports.right).configure(cand.right);
    }
    if (cand.touchedPlanner) {
        state.plannerShadow = cand.planner;
        state.planner->configure(cand.planner);
    }
    if (cand.touchedSTimeout) {
        state.sTimeoutWatchdog->setWindow(static_cast<uint32_t>(cand.sTimeoutRaw));
    }

    char rbuf[360];
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "set", applied, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// GET -- variadic ArgSchema: each bare token becomes args[i].sval (a
// requested key name). No args (bare `GET`) -> dump every registered key.
// Mirrors source_old/commands/ConfigCommands.cpp's getSchema.
// ---------------------------------------------------------------------------
const ArgSchema kGetSchema = { nullptr, 0, 0, true, nullptr };

void handleGet(const ArgList& args, const char* corrId,
               ReplyFn replyFn, void* replyCtx, void* handlerCtx) {
    ConfigCommandState& state = *static_cast<ConfigCommandState*>(handlerCtx);
    // Read once, at GET-time, so every key in this one reply resolves
    // against the SAME bound pair -- see config_commands.h's file header.
    Subsystems::DrivetrainPorts ports = state.drivetrain->ports();

    char line[400];
    int pos = 0;
    int rem = static_cast<int>(sizeof(line));
    int n = snprintf(line + pos, static_cast<size_t>(rem), "CFG");
    if (n > 0 && n < rem) { pos += n; rem -= n; }

    if (args.count == 0) {
        // Dump all registered keys, in kAllKeys order. This file's 15-key
        // surface totals well under the 255-byte CODAL serial TX buffer
        // (source_old/robot/ConfigRegistry.cpp's N12 chunking precedent
        // does not apply at this size) -- a single CFG line is enough.
        for (int i = 0; i < kAllKeysCount && rem > 2; ++i) {
            char kv[40];
            formatConfigKeyFromShadow(state, ports, kAllKeys[i], kv, sizeof(kv));
            line[pos++] = ' '; --rem;
            int w = snprintf(line + pos, static_cast<size_t>(rem), "%s", kv);
            if (w > 0 && w < rem) { pos += w; rem -= w; }
        }
    } else {
        for (int t = 0; t < args.count && rem > 2; ++t) {
            const char* reqKey = args.args[t].sval;
            char kv[40];
            if (formatConfigKeyFromShadow(state, ports, reqKey, kv, sizeof(kv))) {
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
// configCommands -- the SET/GET command table.
// ---------------------------------------------------------------------------
std::vector<CommandDescriptor> configCommands(ConfigCommandState& state) {
    std::vector<CommandDescriptor> cmds;
    cmds.push_back(makeSchemaCmd("GET", &kGetSchema, handleGet, &state,
                                 "badkey", ForceReply::NONE, CMD_NONE));
    cmds.push_back(makeCmd("SET", parseSet, handleSet, &state,
                           "badkey", ForceReply::NONE, CMD_ACCESS_HARDWARE));
    return cmds;
}

#endif  // ROBOT_DEV_BUILD
