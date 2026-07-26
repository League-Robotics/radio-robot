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
// Max physically-plausible wheel speed. An occasional corrupt encoder read
// produces a huge bogus delta; reject any sample beyond this bound and hold
// the previous filtered value.
constexpr float kMaxPlausibleSpeed = 600.0f;   // [mm/s]

// Position-step plausibility gate (source-side outlier rejection -- see
// tick() step 2). Deliberately MORE generous than kMaxPlausibleSpeed (which
// equals the max commandable wheel speed, so real full-speed motion sits
// right at that bound): a step implying twice any commandable speed is
// always a corrupted read. A sample rejected here never reaches
// lastPosition_/filteredVelocity_ at all.
constexpr float kMaxPlausibleStepSpeed = 1200.0f;   // [mm/s]

// After this many CONSECUTIVE step-gate rejections, accept the new position
// as the new truth (re-anchor) instead of rejecting forever -- a persistent
// "glitch" is a real discontinuity (e.g. device-side re-anchor), not noise.
constexpr uint8_t kGlitchStreakAccept = 3;

// Nominal loop period used before the first real dt measurement exists.
constexpr float kNominalDt = 0.024f;   // [s]

// Stale-encoder rest-snap timeout (2026-07-22 bench fix, hardware-confirmed
// same day as c98be2e9): how long since the LAST fresh sample, with the
// commanded target held at exact 0.0f, before a still-elevated
// filteredVelocity_ is forced to 0 regardless of the c98be2e9 rest-noise
// floor. c98be2e9's own gate (velocity_pid.cpp) only resets when the
// CURRENT filteredVelocity_ already reads <= the noise floor -- but once
// the encoder stops producing fresh samples at all (tick()'s own freshness
// gate then holds filteredVelocity_ at whatever it last computed, forever,
// per its own "hold on no new information" comment), a frozen estimate
// that happened to land ABOVE the floor on the last fresh sample before
// the wheel actually went still never gets another chance to be
// recomputed down through it -- confirmed on the bench (2026-07-22): one
// wheel's reported velocity froze at a constant, never-decaying 36mm/s for
// 12+ consecutive ticks after its own encoder position (and the other
// wheel's already-zeroed velocity) had gone flat. This is the literal
// "keeps its filtered tail when encoder deltas go to zero" case the
// defect's own report named. kStaleRestTimeoutUs is set well above the
// brick's own ~80ms 0x46 refresh cadence (see tick()'s own freshness-gate
// comment) -- at ANY real commanded speed the encoder produces a fresh
// sample well inside 80ms (quantization is 0.1mm; even a slow crawl clears
// that within one refresh), so this many consecutive stale ticks with
// target==0.0f is unambiguous evidence of genuine rest, never a
// still-in-flight deceleration (which keeps producing fresh samples every
// refresh and so never reaches this timeout -- scenario 16's own active-
// braking-from-speed case is unaffected).
constexpr uint64_t kStaleRestTimeoutUs = 150000;   // [us]

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

// CODAL's well-known convention: 0 == success.
constexpr int kOk = 0;
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
    bool atRest = std::fabs(filteredVelocity_) < kReconfigureRestVelocity &&
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

void NezhaMotor::setVelocity(float velocity)
{
    velocityTarget_ = velocity;
    mode_ = Mode::Active;
    activeSource_ = ActiveSource::Velocity;
}

void NezhaMotor::setDuty(float duty)
{
    dutyTarget_ = duty;
    mode_ = Mode::Active;
    activeSource_ = ActiveSource::Duty;
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
// Velocity-estimator helpers. pushVelSample()/clearVelWindow() maintain the
// fresh-sample ring; lineFitVelocity() is the mode-1 estimator.
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
// outputs. window 1 short-circuits to the raw duty (default /
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

    // Freshness gate (HARDWARE-CONFIRMED fix -- a raw DUTY command that
    // physically moved the wheel was previously reported as vel=0.000
    // ALWAYS, with the glitch count climbing and a false wedge latch). The
    // Nezha brick's 0x46 register refreshes only every ~80ms; the loop's
    // own cycle runs every ~16ms.
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

    // 2. Mode dispatch (stakeholder 2026-07-18 -- see nezha_motor.h's
    // file-header bullet). Mode::Active dispatches by which setter staged
    // the command: setDuty() -> raw passthrough; setVelocity() -> PID chase
    // while enabled, open-loop feedforward (kff [duty per mm/s] *
    // velocityTarget_) while disabled -- "no PID" drives the nominal duty
    // for the target, it does not go dead. writeShapedDuty() gates every
    // path identically.
    switch (mode_) {
        case Mode::Active:
            if (activeSource_ == ActiveSource::Duty) {
                writeShapedDuty(dutyTarget_, nowMs);
            } else if (pidEnabled_) {
                float dt = haveElapsed ? elapsedTime : kNominalDt;
                float measured = filteredVelocity_;
                // Stale-encoder rest snap -- see kStaleRestTimeoutUs's own
                // comment. Feeding a synthetic 0.0f into compute() (rather
                // than writing filteredVelocity_ directly here) means the
                // SAME restGateEngaged()-driven reset block below fires
                // naturally, so this is one mechanism producing a
                // consistent duty AND reported velocity, not two.
                if (velocityTarget_ == 0.0f && hasFreshSample_ &&
                    (nowUs - lastFreshUs_) > kStaleRestTimeoutUs) {
                    measured = 0.0f;
                }
                float duty = pid_.compute(velocityTarget_, measured, dt,
                                           config_.velGains, config_.velDeadband);
                if (pid_.restGateEngaged()) {
                    // c98be2e9's rest gate just hard-zeroed duty -- either
                    // because the REAL measured velocity was already within
                    // the rest-noise floor, or because the stale-encoder
                    // check just above substituted a synthetic 0.0f for it
                    // (the encoder itself has gone quiet for well over one
                    // brick-refresh interval with target held at exact
                    // zero). Either way this is a genuine "settled at a
                    // stop" state, not a still-decelerating one (see that
                    // gate's own comment in velocity_pid.cpp). Physical
                    // motion has already stopped, but filteredVelocity_ is the
                    // EMA/line-fit's own filtered TAIL of the noisy readings
                    // that led up to this point -- nothing upstream of this
                    // clears it, so it lingers and visibly decays over
                    // several seconds instead of reporting the wheel's
                    // actual (stopped) state immediately. Snap the reported
                    // velocity to 0.0f and drop the line-fit ring (so the
                    // NEXT fresh sample can't re-derive a nonzero slope from
                    // this now-stale, at-rest window) every tick the gate
                    // stays engaged -- idempotent once already zeroed, and
                    // scoped to gate-engaged ticks only, so real in-motion
                    // estimates (gate never engages while target != 0.0f or
                    // measured is still outside the rest floor) are
                    // untouched. lastPosition_/the freshness anchor are
                    // deliberately left alone -- position must keep holding.
                    filteredVelocity_ = 0.0f;
                    clearVelWindow();
                }
                duty = averageDuty(duty);   // boxcar output smoothing (DUTYAVG; no-op at window 1)
                writeShapedDuty(duty, nowMs);
            } else {
                // Open-loop: the same signed feedforward term the PID's own
                // output starts from (kff * target), with every feedback
                // term bypassed. writeRawDuty() clamps to [-1, 1].
                writeShapedDuty(config_.velGains.kff * velocityTarget_, nowMs);
            }
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
// writeRawDuty(). Folded from the old MotorArmor::armoredWrite() (2026-07-18
// restructure) because both policies are Nezha-brick wedge PROTECTION — an
// instantaneous H-bridge sign flip written to 0x60 while the motor is under
// way latches the 0x46 encoder readback (the reversal write train,
// docs/knowledge/2026-07-04-encoder-wedge.md), and near-zero PID dither
// would request such flips every tick without the deadband.
//
// TWO distinct cases where armoredWrite() used to have one (sprint 114
// ticket 005, deadband-compensation-small-commands-must-produce-real-motion.md):
//   - duty == 0.0f EXACTLY: a genuine "stop"/"on target" command (STOP,
//     Mode::Neutral, or App::Pilot's own exact-zero twist on completion).
//     Immediate, unclamped, cancels any dwell in progress, even mid-dwell.
//     NOT boosted -- boosting an intentional zero would make the robot buzz
//     at rest.
//   - 0 < |duty| < outputDeadband_: a genuine NONZERO command, but smaller
//     than the plant can actually produce. Boosted (sign-preserving,
//     std::copysign) up to outputDeadband_ instead of being zeroed, so a
//     real, wanted correction (e.g. a terminal heading/distance trim) still
//     moves the wheel instead of stalling the outer loop until it gives up
//     on an arrive-timeout. The boosted duty then falls through into the
//     SAME reversal-dwell/same-sign logic below as any other nonzero duty —
//     no special-casing there, so a boosted duty that also happens to be a
//     sign reversal still arms/holds/releases through the dwell exactly as
//     an unboosted duty of the same magnitude would (wedge protection is
//     never bypassed by a small command). See sprint 114's own Design
//     Rationale Decision 4 (sprint.md) for why this sits INSIDE NezhaMotor's
//     velocity-PID closed loop (real measured velocity feeds back every
//     tick) rather than at the planner layer (the shape of floor 112-004
//     deleted for hunting) — and is one-sided (only ever lifts a nonzero
//     command toward the threshold, never floors a genuine zero), so it
//     cannot manufacture a new zero-crossing the way a symmetric minimum-
//     speed clamp can.
//
// |duty| >= outputDeadband_ is unaffected either way — passes straight into
// the dwell/same-sign logic below, exactly as before this ticket.
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
    //
    // 118 ticket 003 (sim-cycle-must-match-firmware-period.md): this bound
    // used to sit at a bare 40000 (exactly App::RobotLoop::kCycle, 40ms,
    // converted to [us]) -- comfortable back when firmware's own cycle
    // period was 20ms (2x this interval, plenty of slack), but now that
    // App::RobotLoop::kCycle is ALSO 40ms (118 tickets 001/002; robot_loop.h
    // -- Devices cannot include App:: headers to reference it directly,
    // this project's layering runs App -> Devices, never the reverse, so
    // the two constants are coupled only by this comment, not the
    // compiler), an on-schedule write landing at EXACTLY the throttle
    // interval is no longer comfortably inside it: real-hardware timing
    // jitter (e.g. a 39.x ms cycle, `now` sampled a hair early) makes the
    // `<` comparison below drop that write, every such cycle, forever (the
    // comparison never catches up -- a dropped write does not advance
    // lastWriteTimeUs_). 5ms is a jitter margin, not a second cycle
    // period's worth of headroom -- generous against plausible microbit
    // timer/scheduling jitter (single-digit ms) while still comfortably
    // above the vendor's own duty-write turnaround. Sim's exact virtual
    // steps cannot exercise this hazard (every sim cycle lands at EXACTLY
    // kCycleDtUs, never early) -- reasoned through here in code review,
    // verified on the bench in phase-B (fault/skip counter or encoder
    // smoothness while driving; see this ticket's own Testing section).
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

    // Commit lastWrittenPct_/lastWriteTimeUs_ ONLY on a successful write.
    // An unconditional commit here would latch a NAK'd write as "already
    // written" -- write-on-change (above) would then suppress every retry
    // of the SAME value forever. That
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
    // Median-of-3 atomic-read snapshot + readback-verify + retry. Reset
    // COUNTING lives in the MotorArmor decorator (its own
    // hardResetCount()/softResetCount()) -- the bare leaf keeps none.
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
    filteredVelocity_ = 0.0f;
    hasLastTick_ = false;
    lastGoodRawEnc_ = 0;
    hasFreshSample_ = false;
    lastFreshRawEnc_ = 0;
    lastFreshUs_ = 0;
    clearVelWindow();
}

}  // namespace Devices
