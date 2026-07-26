#include "devices/nezha_motor.h"

#include <cmath>
// HOST_BUILD-only debug tracing: <iostream> must NEVER reach the ARM build --
// newlib-nano has no wide-char stdio (undefined putwc/getwc/swprintf at link)
// and iostream's locale machinery alone overflows FLASH by ~77KB (observed
// 2026-07-18: 450KB into the 364KB region).
#ifdef HOST_BUILD
#include <iostream>
#endif

// ---------------------------------------------------------------------------
// I2C wire protocol constants (verified against PlanetX pxt-nezha2/main.ts).
//
// Every command is an 8-byte write to address 0x10. The frame always starts
// with 0xFF 0xF9 followed by the motorId byte (== config_.port, 1..4 on a
// four-channel Nezha V2 board), then a command-specific payload in bytes
// [3..7].
//
// Motor start / coast (0x60):
//   [0xFF, 0xF9, motorId, direction, 0x60, speed, 0xF5, 0x00]
//   direction: 1 = CW (positive), 2 = CCW (negative); speed: 0-100.
//   speed 0 is the COAST path — NOT 0x5F, which wedges subsequent encoder
//   reads (see writeMotorRun()).
//
// Encoder read (0x46):
//   Write: [0xFF, 0xF9, motorId, 0x00, 0x46, 0x00, 0xF5, 0x00]
//   Read:  4 bytes, signed int32 little-endian, units = tenths of degrees.
// ---------------------------------------------------------------------------

namespace Devices {

namespace {
// CODAL's well-known convention: 0 == success.
constexpr int kOk = 0;

float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// clampStep — bound |target - lastWritten| to at most maxDelta. Kept local
// to its one call site (writeRawDuty() below) rather than split into its
// own file — small, pure, dependency-free. Has NO concept of a "stop"
// command -- pct==0 is just another target value here; the caller
// special-cases pct==0 as an immediate, unclamped, full write.
int8_t clampStep(int8_t lastWritten, int8_t target, uint8_t maxDelta) {
    int16_t delta = static_cast<int16_t>(target) - static_cast<int16_t>(lastWritten);
    if (delta > static_cast<int16_t>(maxDelta)) {
        return static_cast<int8_t>(static_cast<int16_t>(lastWritten) + static_cast<int16_t>(maxDelta));
    }
    if (delta < -static_cast<int16_t>(maxDelta)) {
        return static_cast<int8_t>(static_cast<int16_t>(lastWritten) - static_cast<int16_t>(maxDelta));
    }
    return target;
}

// median3 -- the middle of three int32 values. Local helper for hardReset()'s
// median-of-3 snapshot (below) -- extracted so that function states its own
// two-attempt-then-fallback shape once, not as three inlined sort blocks.
int32_t median3(int32_t a, int32_t b, int32_t c) {
    int32_t lo = a, mid = b, hi = c;
    if (lo > hi) { int32_t tmp = lo; lo = hi; hi = tmp; }
    if (lo > mid) { int32_t tmp = lo; lo = mid; mid = tmp; }
    if (mid > hi) { mid = hi; }
    return mid;
}

// decodeRawEncoder -- little-endian signed int32 decode of the Nezha 0x46
// register's 4-byte reply (tenths of degrees). Shared by
// readEncoderAtomicRaw()/collectEncoder() below -- the only difference
// between those two is HOW the 4 bytes were obtained (a fresh preClear/
// postClear-settled transaction vs. the split-phase collect half), not how
// they decode.
int32_t decodeRawEncoder(const uint8_t resp[4]) {
    return static_cast<int32_t>(
        (static_cast<uint32_t>(resp[3]) << 24) |
        (static_cast<uint32_t>(resp[2]) << 16) |
        (static_cast<uint32_t>(resp[1]) << 8) |
        static_cast<uint32_t>(resp[0]));
}
}  // namespace

// REVISION 1 (114-001, motor.h): the constructor now delegates entirely to
// reconfigure() -- mode_'s own member initializer (Mode::None) applies
// before this constructor body runs, so the guard below always succeeds at
// construction time. Do not keep a duplicate copy of the substitution logic
// here; reconfigure() is the one place it lives.
NezhaMotor::NezhaMotor(I2CBus& bus, const MotorConfig& config)
    : bus_(bus)
{
    // Always succeeds here (mode_'s member initializer is Mode::None before
    // this body runs) -- discard the [[nodiscard]] result explicitly.
    (void)reconfigure(config);
}

// reconfigure -- REVISION 1 (114-001, motor.h): guarded, post-construction,
// whole-config replacement. Refuses (returns false, leaves config_
// unchanged) unless this motor has never yet been commanded (mode_ ==
// Mode::None) or is independently verified at rest (measured velocity below
// kReconfigureRestVelocity AND nothing currently applied to the bus). On
// success, reassigns config_ wholesale and re-derives the same slew-rate/
// write-shaping substitution the constructor used to compute inline.
bool NezhaMotor::reconfigure(const MotorConfig& config)
{
    bool atRest = std::fabs(velocity_) < kReconfigureRestVelocity &&
                  appliedDuty() == 0.0f;
    if (mode_ != Mode::None && !atRest) {
        return false;
    }

    config_ = config;
    if (config_.slewRate <= 0.0f) {
        // MotorConfig.slewRate defaults to the existing kMaxDeltaPwmPerWrite
        // value (25) when unconfigured (zero-initialized).
        config_.slewRate = kDefaultSlewRate;
    }
    // Write-shaping fields (folded from the old MotorArmor base): required,
    // config-as-truth as of sprint 114 ticket 003 -- no more code-side ship-
    // default substitution here. gen_boot_config.py always emits real values
    // (data/robots/*.json's control.reversal_dwell_ms/output_deadband); an
    // explicit 0 is still a valid off-configuration for both.
    reversalDwell_ = config.reversalDwell;
    outputDeadband_ = config.outputDeadband;
    return true;
}

void NezhaMotor::begin()
{
    // Un-freezes the 0x46 readback and zeros the software offset -- the
    // correct initial state for a fresh boot.
    hardReset();
}

// Bare-motor reset semantics (motor.h): resetPosition() acts IMMEDIATELY --
// the caller (or a wrapping MotorArmor, which overrides resetPosition()
// with the staged, standstill-guarded dispatch) owns any at-rest
// discipline. rebaseline() is the software-only re-anchor.
void NezhaMotor::resetPosition() { hardReset(); }

void NezhaMotor::rebaseline() { softRebaseline(); }

// ---------------------------------------------------------------------------
// Primitive setters — stage the command; tick() executes it.
// ---------------------------------------------------------------------------

void NezhaMotor::setDuty(float duty)
{
    dutyTarget_ = duty;
    mode_ = Mode::Active;
}

void NezhaMotor::setNeutral(Neutral mode)
{
    neutralTarget_ = mode;
    mode_ = Mode::Neutral;
}

// applyTravelCalib -- 125-003 (nezha_motor.h's own header): narrowed from
// the pre-125-003 applyGains(Gains, Opt<float>). Plain field mutation --
// tick()'s own position() conversion reads config_.wheelTravelCalib fresh
// on its very next call, so this takes effect on the SAME boot with no
// reflash.
void NezhaMotor::applyTravelCalib(float travelCalib)
{
    config_.wheelTravelCalib = travelCalib;
}

// ---------------------------------------------------------------------------
// Primitive getters.
// ---------------------------------------------------------------------------

float NezhaMotor::position() const { return lastPosition_; }
float NezhaMotor::velocity() const { return velocity_; }

float NezhaMotor::appliedDuty() const
{
    if (lastWrittenPct_ == -128) return 0.0f;   // no write yet
    return static_cast<float>(lastWrittenPct_) / 100.0f;
}

// ---------------------------------------------------------------------------
// tick() — see nezha_motor.h's class-level comment for the 2-step contract
// (the old base-armor steps — reset dispatch, wedge detector, rest
// tracking — now live in the MotorArmor DECORATOR's own tick()).
// ---------------------------------------------------------------------------
void NezhaMotor::tick(uint64_t nowUs)
{
    uint32_t nowMs = static_cast<uint32_t>(nowUs / 1000);

    // 1. Per-tick position sample -- collects a sample that was REQUESTED
    // in a previous slice (requestSample() -> requestEncoder()) by the
    // loop's own cycle. Non-blocking: no write here, no
    // spin, just the 4-byte read.
    int32_t raw = collectEncoder();
    float pos = (static_cast<float>(raw) / 10.0f)
              * config_.wheelTravelCalib * static_cast<float>(config_.fwdSign);

    // Per-TICK elapsed time from this leaf's own us time seam (nowUs), NOT
    // the ms derivation above -- a ms-only clock's +/-1ms quantization
    // would inject noise into a us-scale dt. 125-003: this is now a NAIVE
    // per-tick difference quotient -- the freshness gate that used to key
    // this computation off the LAST FRESH sample (rather than this tick's
    // own dt) is DELETED, not relocated -- and the characterization
    // (docs/design/encoder-refresh-characterization.md) says that is
    // CORRECT on the clean schedule: the register is live, every tick's
    // collect is fresh, and per-tick dt is the honest denominator.
    if (hasLastTick_) {
        float elapsedTime = static_cast<float>(nowUs - lastTickUs_) / 1e6f;   // [s]
        if (elapsedTime > 0.0f) {
            velocity_ = (pos - lastPosition_) / elapsedTime;
        }
        // else: non-positive elapsed time (clock regression) -- defensive
        // only, should not occur in practice; hold the previous velocity_.
    } else {
        hasLastTick_ = true;
        // Boot anchor -- the first-ever sample seeds the baseline WITHOUT
        // computing a velocity (there is no prior sample to diff against).
    }
    lastPosition_ = pos;
    lastTickUs_ = nowUs;

    // 2. Mode dispatch. Mode::Active writes the staged raw duty via
    // writeShapedDuty(); Mode::Neutral writes 0 via writeShapedDuty();
    // Mode::None dispatches nothing.
    switch (mode_) {
        case Mode::Active:
            writeShapedDuty(dutyTarget_, nowMs);
            break;
        case Mode::Neutral:
            // Nezha has no distinct brake register (see writeMotorRun()'s
            // 0x5F note) -- both BRAKE and COAST map to the same 0x60
            // speed-0 coast path, the only safe stop the vendor register
            // map provides.
            (void)neutralTarget_;
            writeShapedDuty(0.0f, nowMs);
            break;
        case Mode::None:
        default:
            break;
    }
}

// ---------------------------------------------------------------------------
// writeShapedDuty() — output-deadband boost, then reversal dwell, then
// writeRawDuty(). Both policies are Nezha-brick wedge PROTECTION — an
// instantaneous H-bridge sign flip written to 0x60 while the motor is under
// way latches the 0x46 encoder readback (the reversal write train,
// docs/knowledge/2026-07-04-encoder-wedge.md), and near-zero dither would
// request such flips every tick without the deadband.
//
// TWO distinct cases (sprint 114 ticket 005,
// deadband-compensation-small-commands-must-produce-real-motion.md):
//   - duty == 0.0f EXACTLY: a genuine "stop"/"on target" command. Immediate,
//     unclamped, cancels any dwell in progress, even mid-dwell. NOT
//     boosted -- boosting an intentional zero would make the robot buzz
//     at rest.
//   - 0 < |duty| < outputDeadband_: a genuine NONZERO command, but smaller
//     than the plant can actually produce. Boosted (sign-preserving,
//     std::copysign) up to outputDeadband_ instead of being zeroed, then
//     falls through into the SAME reversal-dwell/same-sign logic below as
//     any other nonzero duty — wedge protection is never bypassed by a
//     small command.
//
// |duty| >= outputDeadband_ passes straight into the dwell/same-sign logic
// below, unaffected either way.
// ---------------------------------------------------------------------------
void NezhaMotor::writeShapedDuty(float duty, uint32_t now)
{
    if (duty == 0.0f) {
        // Exact zero always wins: immediate, unclamped, cancels any dwell.
        dwelling_ = false;
        lastRequestedDuty_ = 0.0f;
        writeRawDuty(0.0f);
        return;
    }

    if (fabsf(duty) < outputDeadband_) {
        // Genuine nonzero command, smaller than the plant can actually
        // produce -- boost to the deadband floor (sign-preserving) instead
        // of zeroing it, then fall through into the same dwell/same-sign
        // logic below as any other nonzero duty.
        duty = std::copysign(outputDeadband_, duty);
    }

    if (dwelling_) {
        if (now < dwellDeadline_) {
            // Still holding at commanded-zero through the dwell window.
            lastRequestedDuty_ = 0.0f;
            writeRawDuty(0.0f);
            return;
        }
        // Dwell elapsed — proceed in the new direction below.
        dwelling_ = false;
    } else if (reversalDwell_ > 0.0f && lastRequestedDuty_ != 0.0f &&
               ((duty > 0.0f) != (lastRequestedDuty_ > 0.0f))) {
        // Commanded sign change relative to the last duty actually
        // forwarded — write 0 now and arm the dwell; the new direction is
        // withheld until the dwell deadline.
        dwelling_ = true;
        dwellDeadline_ = now + static_cast<uint32_t>(reversalDwell_);
        lastRequestedDuty_ = 0.0f;
        writeRawDuty(0.0f);
        return;
    }

    // Same-sign duty (or no prior direction, or the dwell just elapsed):
    // forward as-is.
    lastRequestedDuty_ = duty;
    writeRawDuty(duty);
}

// ---------------------------------------------------------------------------
// Write path — write-on-change guard, write-rate limit, slew cap, and
// coast-at-zero exemption, including the -128 sentinel's interaction with
// the slew clamp on the very first write. lastWrittenPct_/lastWriteTimeUs_
// commit ONLY when the bus write actually succeeds (status == kOk) -- see
// the bottom of this function for why.
//
// Time source: `now` below reads lastTickUs_, which tick() step 1 already
// set to THIS tick's nowUs before step 2's dispatch calls down into here
// via writeShapedDuty() -- writeRawDuty() is only ever reached synchronously
// from within tick(), after step 1 has run, so no clock ever advances
// between the two reads.
// ---------------------------------------------------------------------------
void NezhaMotor::writeRawDuty(float duty)
{
    duty = clampf(duty, -1.0f, 1.0f);
    int8_t pct = static_cast<int8_t>(lroundf(duty * 100.0f));
    if (pct > 100) pct = 100;
    if (pct < -100) pct = -100;

    // Write-on-change: skip the I2C write if the command is unchanged.
    if (pct == lastWrittenPct_) {
        return;
    }

    // Write-rate limit -- bus hygiene only. Stop is the only throttle
    // exemption. 35ms == App::RobotLoop::kCycle(40ms) minus a 5ms jitter
    // margin (118 ticket 003): comfortably inside one real cycle even under
    // microbit timer jitter, without doubling to a second cycle's worth of
    // headroom (Devices cannot reference App::RobotLoop::kCycle directly --
    // App -> Devices is the one allowed layering direction -- so the two
    // constants are coupled only by this comment).
    static constexpr uint64_t kMinWriteIntervalUs = 35000;   // [us] kCycle(40ms) - 5ms jitter margin
    bool stopping = (pct == 0);
    uint64_t now = lastTickUs_;   // [us] this tick's timestamp (see file-header note)
    if (!stopping && (now - lastWriteTimeUs_) < kMinWriteIntervalUs) {
        return;
    }

    // |delta PWM| slew cap -- a stop is the explicit, unclamped, immediate-
    // write exemption; every other write is stepped by at most
    // config_.slewRate toward the requested target. The very first write
    // for a port sees lastWrittenPct_ still at its -128 "no write yet"
    // sentinel -- treated as the same unclamped-write exemption stop()
    // already gets (there is no prior direction to slew from).
    bool firstWrite = (lastWrittenPct_ == -128);
    int8_t written = (stopping || firstWrite)
        ? pct
        : clampStep(lastWrittenPct_, pct, static_cast<uint8_t>(config_.slewRate));

    // Apply fwdSign: positive written = logical forward.
    int16_t effective = static_cast<int16_t>(config_.fwdSign) * static_cast<int16_t>(written);
    int status;
    if (effective == 0) {
        // Zero speed: COAST via 0x60 speed 0 -- NOT 0x5F, which wedges
        // subsequent encoder reads (see writeMotorRun()).
        status = writeMotorRun(kDirCw, 0);
    } else {
        uint8_t dir = (effective > 0) ? kDirCw : kDirCcw;
        uint8_t speed = (effective > 0) ? static_cast<uint8_t>(effective)
                                         : static_cast<uint8_t>(-effective);
        status = writeMotorRun(dir, speed);
    }

    // Commit lastWrittenPct_/lastWriteTimeUs_ ONLY on a successful write --
    // an unconditional commit would latch a NAK'd write as "already
    // written," so write-on-change (above) would then suppress every retry
    // of the SAME value forever (catastrophic for a failed STOP: the
    // wheel keeps spinning with no further bus traffic to fix it). Leaving
    // lastWrittenPct_ at its PRIOR value on failure also keeps
    // appliedDuty() truthful -- a NAK'd write means the PREVIOUS duty is
    // still the one physically applied, not the one just attempted.
    if (status == kOk) {
        lastWriteTimeUs_ = now;
        lastWrittenPct_ = written;
    }
}

int NezhaMotor::writeMotorRun(uint8_t direction, uint8_t speed)
{
    uint8_t buf[8] = {
        0xFF, 0xF9,
        static_cast<uint8_t>(config_.port),
        direction,
        0x60,
        speed,
        0xF5,
        0x00
    };
    // postClear=4000 holds off the next transaction to 0x10 for a real
    // >=4ms gap after this duty write, mirroring requestEncoder()'s own
    // preClear on the other side of the cycle -- see
    // docs/knowledge/2026-07-04-encoder-wedge.md for the stall this
    // clearance window prevents.
    return bus_.write(static_cast<uint16_t>(kNezhaDeviceAddr << 1), buf, 8, false, /*preClear=*/0, /*postClear=*/4000);
}

// ---------------------------------------------------------------------------
// Encoder reads
// ---------------------------------------------------------------------------

int32_t NezhaMotor::readEncoderAtomicRaw()
{
    // Full vendor timing: preClear/postClear-settled 0x46 write -> read 4
    // bytes. Used only by hardReset()'s median-of-3 snapshot + readback
    // verification.
    static constexpr uint32_t kDelayUs = 4000;

    uint8_t cmd[8] = { 0xFF, 0xF9, static_cast<uint8_t>(config_.port), 0x00, 0x46, 0x00, 0xF5, 0x00 };
    int writeResult = bus_.write(static_cast<uint16_t>(kNezhaDeviceAddr << 1), cmd, 8, false,
                                 /*preClear=*/kDelayUs, /*postClear=*/kDelayUs);

    uint8_t resp[4] = { 0, 0, 0, 0 };
    int readResult = bus_.read(static_cast<uint16_t>(kNezhaDeviceAddr << 1), resp, 4, false);

    connected_ = (writeResult == kOk && readResult == kOk);
    if (!connected_) {
        return lastGoodRawEnc_;
    }

    int32_t result = decodeRawEncoder(resp) - encOffset_;
    lastGoodRawEnc_ = result;
    return result;
}

void NezhaMotor::requestSample()
{
    // Public split-phase phase-1 entry point -- the ONLY caller is
    // the loop's own cycle, once per bus slice, only for the
    // currently-active in-use port.
    requestEncoder();
}

void NezhaMotor::requestEncoder()
{
    // Split-phase phase 1. preClear=4000 holds this
    // write back until a real >=4ms has elapsed since the LAST transaction
    // to 0x10; postClear=4000 attaches the settle window to THIS write's
    // I2CBus deadline, holding off any subsequent transaction to 0x10 until
    // collectEncoder() (or any other 0x10 call) is due. See
    // docs/knowledge/2026-07-04-encoder-wedge.md for the stall this
    // clearance pair prevents.
    uint8_t cmd[8] = { 0xFF, 0xF9, static_cast<uint8_t>(config_.port), 0x00, 0x46, 0x00, 0xF5, 0x00 };
    int writeResult = bus_.write(static_cast<uint16_t>(kNezhaDeviceAddr << 1), cmd, 8, false,
                                 /*preClear=*/4000, /*postClear=*/4000);
    pendingEncRequestOk_ = (writeResult == kOk);
}

int32_t NezhaMotor::collectEncoder()
{
    // Split-phase phase 2 -- ported byte-for-byte. connected_ reflects BOTH
    // halves of the split transaction (the request's write status AND this
    // collect's read status) -- a request whose 0x46 write itself failed
    // must not be reported as connected merely because the subsequent read
    // happened to succeed against stale/garbage state.
    uint8_t resp[4] = { 0, 0, 0, 0 };
    int readResult = bus_.read(static_cast<uint16_t>(kNezhaDeviceAddr << 1), resp, 4, false);

    connected_ = pendingEncRequestOk_ && (readResult == kOk);

    if (!connected_) {
        return lastGoodRawEnc_;
    }

    int32_t result = decodeRawEncoder(resp) - encOffset_;
    lastGoodRawEnc_ = result;
    return result;
}

void NezhaMotor::hardReset()
{
    // Median-of-3 atomic-read snapshot + readback-verify + retry. Reset
    // COUNTING lives in the MotorArmor decorator (its own
    // hardResetCount()/softResetCount()) -- the bare leaf keeps none.
    static constexpr int kMaxRetries = 2;
    static constexpr int32_t kReadbackThreshold = 2;

    for (int attempt = 0; attempt <= kMaxRetries; ++attempt) {
        int32_t snapshot = median3(readEncoderAtomicRaw(), readEncoderAtomicRaw(), readEncoderAtomicRaw());
        encOffset_ += snapshot;

        int32_t readback = readEncoderAtomicRaw();
        if (readback >= -kReadbackThreshold && readback <= kReadbackThreshold) {
            lastPosition_ = 0.0f;
            velocity_ = 0.0f;
            hasLastTick_ = false;
            lastGoodRawEnc_ = 0;
            return;
        }
        encOffset_ -= snapshot;
    }

    // Retries exhausted: apply the last snapshot anyway (best-effort
    // fallback).
    encOffset_ += median3(readEncoderAtomicRaw(), readEncoderAtomicRaw(), readEncoderAtomicRaw());
    lastPosition_ = 0.0f;
    velocity_ = 0.0f;
    hasLastTick_ = false;
    lastGoodRawEnc_ = 0;
}

void NezhaMotor::softRebaseline()
{
    // Software-only encoder rebaseline -- folds the
    // already-tick-cached lastPosition_ (populated by this tick's
    // collectEncoder() call, not a new atomic read) back into raw
    // tenths-of-degrees and adds it to encOffset_, then zeroes the cache
    // exactly as hardReset()'s success path does. Issues NO I2C
    // transaction at all.
    //
    // Inverse of tick()'s collectEncoder()-based conversion (mm = (raw/10)
    // * wheelTravelCalib * fwdSign): rawDelta = (mm / (wheelTravelCalib *
    // fwdSign)) * 10.
    if (config_.wheelTravelCalib != 0.0f) {
        float rawDeltaF = (lastPosition_ / (config_.wheelTravelCalib * static_cast<float>(config_.fwdSign))) * 10.0f;
        encOffset_ += static_cast<int32_t>(rawDeltaF);
    }
    lastPosition_ = 0.0f;
    velocity_ = 0.0f;
    hasLastTick_ = false;
    lastGoodRawEnc_ = 0;
}

}  // namespace Devices
