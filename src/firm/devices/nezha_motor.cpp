#include "devices/nezha_motor.h"

#include <cmath>

// ---------------------------------------------------------------------------
// I2C wire protocol constants (verified against PlanetX pxt-nezha2/main.ts,
// via source/hal/nezha/nezha_motor.cpp / source_old/hal/real/Motor.cpp —
// ported byte-for-byte, not re-derived).
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
// Max physically-plausible wheel speed. An occasional corrupt encoder read
// produces a huge bogus delta; reject any sample beyond this bound and hold
// the previous filtered value.
constexpr float kMaxPlausibleSpeed = 1000.0f;   // [mm/s]

// Position-step plausibility gate (source-side outlier rejection -- see
// tick() step 2). Deliberately MORE generous than kMaxPlausibleSpeed (which
// equals the max commandable wheel speed, so real full-speed motion sits
// right at that bound): a step implying twice any commandable speed is
// always a corrupted read. A sample rejected here never reaches
// lastPosition_/filteredVelocity_ at all.
constexpr float kMaxPlausibleStepSpeed = 2000.0f;   // [mm/s]

// After this many CONSECUTIVE step-gate rejections, accept the new position
// as the new truth (re-anchor) instead of rejecting forever -- a persistent
// "glitch" is a real discontinuity (e.g. device-side re-anchor), not noise.
constexpr uint8_t kGlitchStreakAccept = 3;

// Nominal loop period used before the first real dt measurement exists.
constexpr float kNominalDt = 0.024f;   // [s]

float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// clampStep — bound |target - lastWritten| to at most maxDelta. Ported
// verbatim from source/hal/nezha/motor_slew.h's MotorSlew::clampStep()
// (itself ported unchanged from source_old/hal/real/MotorSlew.h, 064-002).
// Not split into its own devices/motor_slew.h file — device-bus-tickets.md's
// DB-004 "Files" list scopes this ticket to velocity_pid.{h,cpp}/
// motor_armor.h/nezha_motor.{h,cpp}; this pure, dependency-free helper is
// small enough to keep local to its one call site (writeRawDuty() below)
// rather than add an unlisted file for a three-line function. Has NO
// concept of a "stop" command -- pct==0 is just another target value here;
// the caller special-cases pct==0 as an immediate, unclamped, full write.
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

// CODAL's well-known convention: 0 == success.
constexpr int kOk = 0;
}  // namespace

NezhaMotor::NezhaMotor(I2CBus& bus, const MotorConfig& config)
    : bus_(bus)
{
    // configureArmor() (MotorArmor, base) caches the two armor fields
    // (reversalDwell/outputDeadband, defaulting when unset); this
    // constructor then caches the rest of config_ itself (no separate
    // configureDevice() virtual dispatch step -- NezhaMotor is this
    // ticket's only leaf, so the extra indirection the pre-port file used
    // to support future leaves overriding a virtual configureDevice() adds
    // nothing here).
    configureArmor(config);
    config_ = config;
    if (config_.slewRate <= 0.0f) {
        // MotorConfig.slewRate defaults to the existing kMaxDeltaPwmPerWrite
        // value (25) when unconfigured (zero-initialized).
        config_.slewRate = kDefaultSlewRate;
    }
}

void NezhaMotor::begin()
{
    // Un-freezes the 0x46 readback and zeros the software offset -- the
    // correct initial state for a fresh boot.
    hardReset();
}

// ---------------------------------------------------------------------------
// Primitive setters — stage the command; tick() executes it.
// ---------------------------------------------------------------------------

void NezhaMotor::setVelocity(float velocity)
{
    velocityTarget_ = velocity;
    mode_ = Mode::Active;
}

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

void NezhaMotor::setPidEnabled(bool on)
{
    pidEnabled_ = on;
}

// applyGains() -- see nezha_motor.h's own comment for the isolation-
// invariant rationale. Plain field mutation, no armor/write-path
// interaction: pid_.compute() (called from tick()'s own dispatch) reads
// config_.velGains fresh on its very next call, so this takes effect on
// the SAME boot with no reflash.
void NezhaMotor::applyGains(const Gains& gains, Opt<float> travelCalib)
{
    config_.velGains = gains;
    if (travelCalib.has) config_.wheelTravelCalib = travelCalib.val;
}

void NezhaMotor::setVelEstimator(uint8_t mode, uint8_t window)
{
    velEstMode_ = (mode == kVelEstLineFit) ? kVelEstLineFit : kVelEstEma;
    if (window < 3) window = 3;
    if (window > kMaxVelWindow) window = kMaxVelWindow;
    velWindow_ = window;
}

// ---------------------------------------------------------------------------
// Velocity-estimator helpers (sprint 101). pushVelSample()/clearVelWindow()
// maintain the fresh-sample ring; lineFitVelocity() is the mode-1 estimator.
// ---------------------------------------------------------------------------
void NezhaMotor::pushVelSample(uint64_t t, float position)
{
    velWinT_[velWinHead_] = t;
    velWinP_[velWinHead_] = position;
    velWinHead_ = static_cast<uint8_t>((velWinHead_ + 1) % kMaxVelWindow);
    if (velWinCount_ < kMaxVelWindow) ++velWinCount_;
}

void NezhaMotor::clearVelWindow()
{
    velWinCount_ = 0;
    velWinHead_ = 0;
    dutyRingCount_ = 0;   // output-average ring is derived state too; clear together
    dutyRingHead_ = 0;
}

void NezhaMotor::setDutyAvg(uint8_t window)
{
    if (window < 1) window = 1;
    if (window > kMaxDutyAvg) window = kMaxDutyAvg;
    dutyAvgWindow_ = window;
}

// averageDuty() — boxcar moving average of the last dutyAvgWindow_ PID duty
// outputs (sprint 101). window 1 short-circuits to the raw duty (default /
// unchanged behavior). Pushes every call so a live window change has history.
float NezhaMotor::averageDuty(float duty)
{
    dutyRing_[dutyRingHead_] = duty;
    dutyRingHead_ = static_cast<uint8_t>((dutyRingHead_ + 1) % kMaxDutyAvg);
    if (dutyRingCount_ < kMaxDutyAvg) ++dutyRingCount_;

    if (dutyAvgWindow_ <= 1) return duty;
    uint8_t n = (dutyRingCount_ < dutyAvgWindow_) ? dutyRingCount_ : dutyAvgWindow_;
    float sum = 0.0f;
    for (uint8_t k = 0; k < n; ++k) {
        uint8_t idx = static_cast<uint8_t>((dutyRingHead_ + kMaxDutyAvg - 1 - k) % kMaxDutyAvg);
        sum += dutyRing_[idx];
    }
    return sum / static_cast<float>(n);
}

// lineFitVelocity() — least-squares slope of position vs. time over the last
// min(velWinCount_, velWindow_) accepted fresh samples (Savitzky-Golay order
// 1). Closed-form (no matrix work): four running sums + one divide, ~O(N).
// Times are taken relative to the oldest sample in the window and scaled to
// seconds before summing, so the sums stay small (a us-scale absolute stamp
// squared would lose float precision). Fewer than 3 points cannot smooth a
// line, so it falls back to the last filtered value (matching the EMA path's
// "hold on no new information" behavior).
float NezhaMotor::lineFitVelocity() const
{
    uint8_t n = (velWinCount_ < velWindow_) ? velWinCount_ : velWindow_;
    if (n < 3) return filteredVelocity_;

    uint8_t oldest = static_cast<uint8_t>((velWinHead_ + kMaxVelWindow - n) % kMaxVelWindow);
    uint64_t tref = velWinT_[oldest];

    float sx = 0.0f, sy = 0.0f, sxx = 0.0f, sxy = 0.0f;
    for (uint8_t k = 0; k < n; ++k) {
        uint8_t idx = static_cast<uint8_t>((oldest + k) % kMaxVelWindow);
        float x = static_cast<float>(velWinT_[idx] - tref) / 1e6f;   // [s]
        float y = velWinP_[idx];                                     // [mm]
        sx += x; sy += y; sxx += x * x; sxy += x * y;
    }
    float fn = static_cast<float>(n);
    float denom = fn * sxx - sx * sx;
    if (fabsf(denom) < 1e-9f) return filteredVelocity_;
    return (fn * sxy - sx * sy) / denom;   // [mm/s]
}

// ---------------------------------------------------------------------------
// Primitive getters.
// ---------------------------------------------------------------------------

float NezhaMotor::position() const { return lastPosition_; }
float NezhaMotor::velocity() const { return filteredVelocity_; }

float NezhaMotor::appliedDuty() const
{
    if (lastWrittenPct_ == -128) return 0.0f;   // no write yet
    return static_cast<float>(lastWrittenPct_) / 100.0f;
}

// ---------------------------------------------------------------------------
// tick() — see nezha_motor.h's class-level comment for the 5-step
// call-order contract.
// ---------------------------------------------------------------------------
void NezhaMotor::tick(uint64_t nowUs)
{
    uint32_t nowMs = static_cast<uint32_t>(nowUs / 1000);

    // 1. Standstill-guarded reset dispatch.
    processResetIfPending(nowMs);

    // 2. Per-tick position sample -- collects a sample that was REQUESTED
    // in a previous slice (requestSample() -> requestEncoder()) by the
    // loop's own cycle. Non-blocking: no write here, no
    // spin, just the 4-byte read.
    int32_t raw = collectEncoder();
    float pos = (static_cast<float>(raw) / 10.0f)
              * config_.wheelTravelCalib * static_cast<float>(config_.fwdSign);

    // Per-TICK elapsed time from this leaf's own us time seam (nowUs), NOT
    // the ms derivation above -- a ms-only clock's +/-1ms quantization
    // would inject noise into a us-scale dt. Feeds ONLY the embedded PID's
    // dt at step 4 below (kNominalDt fallback before the first tick) --
    // deliberately NOT used for the velocity/glitch computation below; see
    // the freshness gate's own comment for why.
    float elapsedTime = 0.0f;   // [s]
    bool haveElapsed = false;
    if (hasLastTick_) {
        elapsedTime = static_cast<float>(nowUs - lastTickUs_) / 1e6f;
        if (elapsedTime > 0.0f) haveElapsed = true;
    } else {
        hasLastTick_ = true;
    }
    lastTickUs_ = nowUs;

    // Freshness gate (HARDWARE-CONFIRMED fix -- DB-009 bring-up image: `M 1
    // STATE` reported vel=0.000 ALWAYS, glitch count climbing, wedged=1
    // false-latching, even while a raw DUTY command physically moved the
    // wheel ~717mm). The Nezha brick's 0x46 register refreshes only every
    // ~80ms; the loop's own cycle runs every ~16ms.
    // Running the velocity/glitch computation on every TICK (as before)
    // meant most cycles re-collected an IDENTICAL raw count (step==0,
    // rawVel==0 -- decaying filteredVelocity_ toward 0 every stale cycle),
    // and when a genuinely fresh count finally landed, the ~5-cycle-
    // accumulated step divided by a SINGLE cycle's elapsedTime looked ~5x
    // too fast -- permanently rejected by kMaxPlausibleStepSpeed as a
    // "glitch". Fix: run the velocity/glitch computation ONLY on a sample
    // whose raw count differs from the last FRESH raw count, using the
    // elapsed time SINCE THAT sample (lastFreshUs_), never this tick's own
    // (much shorter) elapsedTime. Compared at the raw wire-count level (not
    // the derived `pos`) -- collectEncoder() carries no brick-side sample
    // timestamp to key off, so an unchanged raw count is the direct,
    // unambiguous signal that the brick has not refreshed yet.
    bool freshSample = !hasFreshSample_ || (raw != lastFreshRawEnc_);

    if (freshSample && !hasFreshSample_) {
        // Boot/reset anchor -- the first-ever fresh sample (or the first
        // after hardReset()/softRebaseline(), which clear hasFreshSample_
        // the same way they clear hasLastTick_) seeds the baseline WITHOUT
        // computing a velocity or running the plausibility gate. The brick
        // reports a large lifetime-accumulated raw count on first contact
        // (hardware-observed: ~-33526mm) -- NORMAL, not a glitch -- so
        // anchor directly to it rather than diffing against an assumed
        // lastPosition_ of 0.
        lastPosition_ = pos;
        lastFreshRawEnc_ = raw;
        lastFreshUs_ = nowUs;
        hasFreshSample_ = true;
        pushVelSample(nowUs, pos);   // seed the line-fit window with the anchor
    } else if (freshSample) {
        float freshElapsed = static_cast<float>(nowUs - lastFreshUs_) / 1e6f;   // [s] since the last FRESH sample
        bool sampleOk = true;
        if (freshElapsed > 0.0f) {
            // Source-side outlier rejection, now gated on genuinely fresh
            // samples: a step implying a speed beyond kMaxPlausibleStepSpeed
            // is a corrupted I2C read, not motion. Reject the WHOLE sample:
            // hold last-good position AND velocity, count it. If
            // kGlitchStreakAccept consecutive FRESH samples all "glitch",
            // the encoder genuinely moved (e.g. an external re-anchor) --
            // accept and re-anchor rather than rejecting forever.
            float step = fabsf(pos - lastPosition_);   // [mm]
            if (step > kMaxPlausibleStepSpeed * freshElapsed) {
                ++encGlitchCount_;
                if (++encGlitchStreak_ < kGlitchStreakAccept) {
                    sampleOk = false;
                } else {
                    encGlitchStreak_ = 0;   // re-anchor to the new reality
                }
            } else {
                encGlitchStreak_ = 0;
            }

            if (sampleOk) {
                // Ring is maintained for BOTH modes (so a live switch to
                // line-fit has history to fit immediately); the mode only
                // decides which estimate feeds filteredVelocity_.
                pushVelSample(nowUs, pos);
                if (velEstMode_ == kVelEstLineFit) {
                    float v = lineFitVelocity();
                    if (fabsf(v) <= kMaxPlausibleSpeed) filteredVelocity_ = v;
                    // else: implausible -- hold filteredVelocity_.
                } else {
                    float rawVel = (pos - lastPosition_) / freshElapsed;
                    if (fabsf(rawVel) <= kMaxPlausibleSpeed) {
                        float a = config_.velFiltAlpha;   // EMA smoothing
                        filteredVelocity_ = a * rawVel + (1.0f - a) * filteredVelocity_;
                    }
                    // else: implausible-but-sub-step velocity -- hold filteredVelocity_.
                }
            }
        }
        // else: non-positive elapsed time since the last fresh sample
        // (clock regression) -- defensive only, should not occur in
        // practice; sampleOk stays true so the anchor below still advances,
        // but no velocity is computed from a non-positive dt.

        if (sampleOk) {
            lastPosition_ = pos;
            lastFreshRawEnc_ = raw;
            lastFreshUs_ = nowUs;
        }
        // else: glitch rejected -- hold lastPosition_/filteredVelocity_/
        // lastFreshRawEnc_/lastFreshUs_, so the NEXT fresh sample's elapsed
        // time is measured from the last ACCEPTED anchor, not this
        // rejected one.
    }
    // else (!freshSample && hasFreshSample_): repeated raw value -- the
    // brick has not refreshed since the last fresh sample. Hold
    // lastPosition_/filteredVelocity_ unchanged this cycle -- running the
    // plausibility gate against a same-value, near-zero-elapsed step here
    // is exactly the false-glitch bug this fix removes.

    // 3. Wedge detector -- reads position() (== lastPosition_, just
    // maintained above by the freshness gate) and appliedDuty() (last
    // tick's write; this tick's mode dispatch has not run yet). A repeated
    // raw sample between two brick refreshes now holds position() constant
    // for only the handful of fiber cycles between refreshes (matching the
    // brick's own refresh cadence), well under kWedgeThreshold's 10-
    // consecutive-identical-reads bound -- normal driving no longer false-
    // latches (the pre-fix false latch came from the OLD per-tick glitch
    // path silently holding lastPosition_ across MULTIPLE refresh windows
    // at a time, a much longer stall than one brick-refresh interval).
    updateWedgeDetector();

    // 4. Mode dispatch. Mode::Active covers both PID-on (chase
    // velocityTarget_) and PID-off (raw dutyTarget_ passthrough) -- OQ2:
    // armoredWrite() gates BOTH paths identically.
    switch (mode_) {
        case Mode::Active:
            if (pidEnabled_) {
                float dt = haveElapsed ? elapsedTime : kNominalDt;
                float duty = pid_.compute(velocityTarget_, filteredVelocity_, dt,
                                           config_.velGains, config_.velDeadband);
                duty = averageDuty(duty);   // boxcar output smoothing (DUTYAVG; no-op at window 1)
                armoredWrite(duty, nowMs);
            } else {
                armoredWrite(dutyTarget_, nowMs);
            }
            break;
        case Mode::Neutral:
            // Nezha has no distinct brake register (see writeMotorRun()'s
            // 0x5F note) -- both BRAKE and COAST map to the same 0x60
            // speed-0 coast path, the only safe stop the vendor register
            // map provides.
            (void)neutralTarget_;
            armoredWrite(0.0f, nowMs);
            break;
        case Mode::None:
        default:
            break;
    }

    // 5. Rest tracking.
    updateRestTracking();
}

// ---------------------------------------------------------------------------
// Write path — ported from the pre-port file's writeRawDuty(), itself
// ported from source_old's Motor::setSpeed(). Write-on-change guard,
// write-rate limit, slew cap, and coast-at-zero exemption, including the
// -128 sentinel's interaction with the slew clamp on the very first write,
// are all unchanged from the port. ONE behavior is new (103-002, C1 fix,
// 2026-07-13 code review): lastWrittenPct_/lastWriteTimeUs_ now commit ONLY
// when the bus write actually succeeds (status == kOk) -- see the bottom of
// this function for why.
//
// Time source: `now` below reads lastTickUs_, which tick() step 2 already
// set to THIS tick's nowUs before step 4's dispatch calls down into here
// via armoredWrite() -- see nezha_motor.h's file-header note on why that is
// exactly equivalent to a fresh clock read at this point (writeRawDuty() is
// only ever reached synchronously from within tick(), after step 2 has run,
// so no clock ever advances between the two reads).
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
    // exemption.
    static constexpr uint64_t kMinWriteIntervalUs = 40000;   // [us] 40 ms ~= 25 Hz max
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

    // 103-002 (C1 fix, 2026-07-13 code review): commit lastWrittenPct_/
    // lastWriteTimeUs_ ONLY on a successful write. The old unconditional
    // commit latched a NAK'd write as "already written" -- write-on-change
    // (above) then suppressed every retry of the SAME value forever. That
    // was catastrophic specifically for a failed STOP (pct==0): the
    // watchdog's "re-assert Neutral every cycle" robustness (the loop's
    // stale-target gate) calls armoredWrite(0) every cycle, which
    // always dispatches straight to writeRawDuty(0) (motor_armor.h) --  but
    // once pct==0==lastWrittenPct_ was falsely latched, every subsequent
    // call hit the write-on-change guard above and returned before ever
    // touching the bus again, even though the wheel was still physically
    // spinning. Leaving lastWrittenPct_ at its PRIOR value on failure is
    // also the correct MotorArmor semantics: appliedDuty() must reflect
    // what the hardware is actually doing, and a NAK'd write means the
    // PREVIOUS duty is still the one physically applied, not the one just
    // attempted.
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
// Encoder reads — all ported byte-for-byte from the pre-port file.
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

    int32_t raw = static_cast<int32_t>(
        (static_cast<uint32_t>(resp[3]) << 24) |
        (static_cast<uint32_t>(resp[2]) << 16) |
        (static_cast<uint32_t>(resp[1]) << 8) |
        static_cast<uint32_t>(resp[0]));
    int32_t result = raw - encOffset_;
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
    // Split-phase phase 1 -- ported byte-for-byte. preClear=4000 holds this
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

    int32_t raw = static_cast<int32_t>(
        (static_cast<uint32_t>(resp[3]) << 24) |
        (static_cast<uint32_t>(resp[2]) << 16) |
        (static_cast<uint32_t>(resp[1]) << 8) |
        static_cast<uint32_t>(resp[0]));
    int32_t result = raw - encOffset_;
    lastGoodRawEnc_ = result;
    return result;
}

void NezhaMotor::hardReset()
{
    // Ported unchanged: median-of-3 atomic-read snapshot + readback-verify
    // + retry. processResetIfPending() (base) increments hardResetCount_
    // after calling this.
    static constexpr int kMaxRetries = 2;
    static constexpr int32_t kReadbackThreshold = 2;

    for (int attempt = 0; attempt <= kMaxRetries; ++attempt) {
        int32_t s0 = readEncoderAtomicRaw();
        int32_t s1 = readEncoderAtomicRaw();
        int32_t s2 = readEncoderAtomicRaw();

        int32_t lo = s0, mid = s1, hi = s2;
        if (lo > hi) { int32_t tmp = lo; lo = hi; hi = tmp; }
        if (lo > mid) { int32_t tmp = lo; lo = mid; mid = tmp; }
        if (mid > hi) { mid = hi; }
        int32_t snapshot = mid;

        encOffset_ += snapshot;

        int32_t readback = readEncoderAtomicRaw();
        if (readback >= -kReadbackThreshold && readback <= kReadbackThreshold) {
            lastPosition_ = 0.0f;
            filteredVelocity_ = 0.0f;
            hasLastTick_ = false;
            lastGoodRawEnc_ = 0;
            // Clear the fresh-sample anchor too -- the next collectEncoder()
            // after a hard reset must re-run the boot-anchor branch (tick()
            // step 2), not diff against a pre-reset raw count that encOffset_
            // has just invalidated.
            hasFreshSample_ = false;
            lastFreshRawEnc_ = 0;
            lastFreshUs_ = 0;
            clearVelWindow();
            return;
        }
        encOffset_ -= snapshot;
    }

    // Retries exhausted: apply the last snapshot anyway (best-effort
    // fallback).
    int32_t s0 = readEncoderAtomicRaw();
    int32_t s1 = readEncoderAtomicRaw();
    int32_t s2 = readEncoderAtomicRaw();
    int32_t lo = s0, mid = s1, hi = s2;
    if (lo > hi) { int32_t tmp = lo; lo = hi; hi = tmp; }
    if (lo > mid) { int32_t tmp = lo; lo = mid; mid = tmp; }
    if (mid > hi) { mid = hi; }
    encOffset_ += mid;
    lastPosition_ = 0.0f;
    filteredVelocity_ = 0.0f;
    hasLastTick_ = false;
    lastGoodRawEnc_ = 0;
    hasFreshSample_ = false;
    lastFreshRawEnc_ = 0;
    lastFreshUs_ = 0;
    clearVelWindow();
}

void NezhaMotor::softRebaseline()
{
    // Ported unchanged: software-only encoder rebaseline -- folds the
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
    filteredVelocity_ = 0.0f;
    hasLastTick_ = false;
    lastGoodRawEnc_ = 0;
    hasFreshSample_ = false;
    lastFreshRawEnc_ = 0;
    lastFreshUs_ = 0;
    clearVelWindow();

    // softResetCount_ is base-owned (MotorArmor); this leaf increments it
    // directly (it is protected, inherited) rather than duplicating a
    // counter here.
    ++softResetCount_;
}

}  // namespace Devices
