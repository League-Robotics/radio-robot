#include "hal/nezha/nezha_motor.h"
#include "hal/nezha/motor_slew.h"

#include "MicroBit.h"   // MICROBIT_OK, system_timer_current_time_us() (vendor SDK; excluded from the
                        // no-units-in-identifiers rename per .claude/rules/coding-standards.md)
#include <math.h>

// ---------------------------------------------------------------------------
// I2C wire protocol constants (verified against PlanetX pxt-nezha2/main.ts,
// via source_old/hal/real/Motor.cpp — ported byte-for-byte, not re-derived).
//
// Every command is an 8-byte write to address 0x10. The frame always starts
// with 0xFF 0xF9 followed by the motorId byte (== config_.port, 1..4 on a
// four-channel Nezha V2 board — see docs/architecture/architecture-update-
// 046.md's MecanumHAL port wiring for confirmation that motorId ranges over
// all four channels on the same address), then a command-specific payload
// in bytes [3..7].
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

namespace Hal {

namespace {
// Max physically-plausible wheel speed. An occasional corrupt encoder read
// produces a huge bogus delta; reject any sample beyond this bound and hold
// the previous filtered value (ported from
// source_old/control/MotorController.cpp's kMaxPlausibleSpeed).
constexpr float kMaxPlausibleSpeed = 1000.0f;   // [mm/s]

// Nominal loop period used before the first real dt measurement exists
// (ported from VelocityController.cpp's kNominalDt).
constexpr float kNominalDt = 0.024f;   // [s]

// Consecutive-identical-reading threshold for the wedge latch (ported from
// MotorController.cpp's kWedgeThreshold).
constexpr uint8_t kWedgeThreshold = 10;

float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
}  // namespace

NezhaMotor::NezhaMotor(I2CBus& bus, const msg::MotorConfig& config)
    : bus_(bus), config_(config)
{
    if (config_.slew_rate <= 0.0f) {
        // architecture-update.md Design Rationale 2: MotorConfig.slew_rate
        // defaults to the existing kMaxDeltaPwmPerWrite value (25) when
        // unconfigured (zero-initialized), matching source_old's file-local
        // constexpr default.
        config_.slew_rate = kDefaultSlewRate;
    }
}

void NezhaMotor::begin()
{
    // Ported from Motor::begin(): un-freezes the 0x46 readback and zeros
    // the software offset — the correct initial state for a fresh boot.
    hardResetEncoder();
}

void NezhaMotor::configure(const msg::MotorConfig& config)
{
    config_ = config;
    if (config_.slew_rate <= 0.0f) {
        config_.slew_rate = kDefaultSlewRate;
    }
}

msg::MotorCapabilities NezhaMotor::capabilities() const
{
    msg::MotorCapabilities caps;
    caps.duty_cycle = true;
    caps.voltage = false;     // Nezha has no voltage-mode register
    caps.velocity = true;     // embedded PID, closed over the encoder
    caps.position = true;     // onboard 0x5D absolute-angle move
    caps.has_encoder = true;
    return caps;
}

// ---------------------------------------------------------------------------
// Primitive setters — stage the command; tick() executes it.
// ---------------------------------------------------------------------------

void NezhaMotor::setDutyCycle(float dutyCycle)
{
    mode_ = Mode::DUTY;
    dutyTarget_ = dutyCycle;
}

void NezhaMotor::setVoltage(float voltage)
{
    // Unsupported on Nezha (capabilities().voltage == false). apply() gates
    // this before it is ever called; the body is a documented no-op rather
    // than an assert so a direct (non-apply()) call from test code cannot
    // crash the firmware.
    (void)voltage;
}

void NezhaMotor::setVelocity(float velocity)
{
    mode_ = Mode::VELOCITY;
    velocityTarget_ = velocity;
}

void NezhaMotor::setPosition(float position)
{
    mode_ = Mode::POSITION;
    positionTarget_ = position;
    positionCommandPending_ = true;   // write-on-change: only issue 0x5D once per new target
}

void NezhaMotor::setNeutral(msg::Neutral mode)
{
    mode_ = Mode::NEUTRAL;
    neutralTarget_ = mode;
}

void NezhaMotor::setFeedforward(float feedforward)
{
    feedforward_ = feedforward;
}

void NezhaMotor::resetPosition()
{
    // Staged, not immediate: "reset_position rides beside any arm and
    // zeroes the encoder that tick" (ticket 003 acceptance criteria) —
    // processed at the top of the next tick(), alongside whichever mode is
    // active.
    resetPending_ = true;
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

bool NezhaMotor::connected() const { return connected_; }
bool NezhaMotor::wedged() const { return wedgeLatched_; }

// ---------------------------------------------------------------------------
// tick() — sample the encoder, then execute the staged command per mode.
// ---------------------------------------------------------------------------

void NezhaMotor::tick(uint32_t nowMs)
{
    if (resetPending_) {
        hardResetEncoder();
        resetPending_ = false;
    }

    // Per-tick position sample — settle-read path (0x46 write -> 4ms
    // post-write busy-wait -> 4-byte read, no pre-write idle), exactly as
    // source_old's Motor::tick() via readEncoderSettle().
    float pos = readEncoderSettle();

    // Wedge detector — unconditional, every tick (ported from
    // MotorController::controlTick's per-wheel detector).
    updateWedgeDetector(pos);

    float elapsedTime = 0.0f;
    bool haveElapsed = false;
    if (hasLastTick_) {
        elapsedTime = static_cast<float>(nowMs - lastTick_) / 1000.0f;
        if (elapsedTime > 0.0f) {
            haveElapsed = true;
            float rawVel = (pos - lastPosition_) / elapsedTime;
            if (fabsf(rawVel) <= kMaxPlausibleSpeed) {
                float a = config_.vel_filt_alpha;   // EMA smoothing
                filteredVelocity_ = a * rawVel + (1.0f - a) * filteredVelocity_;
            }
            // else: garbage read — reject, hold filteredVelocity_ (matches
            // source_old's plausibility gate).
        }
    } else {
        hasLastTick_ = true;
    }
    lastPosition_ = pos;
    lastTick_ = nowMs;

    switch (mode_) {
        case Mode::DUTY:
            writeDuty(dutyTarget_);
            break;
        case Mode::VELOCITY: {
            float dt = haveElapsed ? elapsedTime : kNominalDt;
            float duty = runVelocityPid(velocityTarget_, filteredVelocity_, dt);
            writeDuty(clampf(duty + feedforward_, -1.0f, 1.0f));
            break;
        }
        case Mode::POSITION:
            if (positionCommandPending_) {
                writePositionMove(positionTarget_);
                positionCommandPending_ = false;
            }
            break;
        case Mode::NEUTRAL:
            // Nezha has no distinct brake register (see writeMotorRun()'s
            // 0x5F note) — both BRAKE and COAST map to the same 0x60
            // speed-0 coast path, the only safe stop source_old provides.
            (void)neutralTarget_;
            writeDuty(0.0f);
            break;
        case Mode::NONE:
        default:
            break;
    }
}

// ---------------------------------------------------------------------------
// Embedded velocity PID — ported control law from
// source_old/control/VelocityController.cpp::update().
//
// VelocityController composes cmon-pid's backcalculation_t<pid_bwe> with
// Kd=0, Tf=kTinyTf (~1e-6s). For that configuration, pid_bwe's transfer-
// function coefficients collapse (A1 = Tf/(h+Tf) ~ 0, C3 = Kd/Tf = 0, A3 ~
// Kp), so the general transfer-function machinery reduces to a plain
// discrete PI with back-calculation anti-windup. This function implements
// that reduced form directly rather than pulling in cmon-pid, avoiding a
// second vendored dependency for a fresh (not-yet-bench-tuned) config
// surface (MotorConfig.vel_gains) with no established prior calibration to
// match bit-for-bit. Ticket 003's Verification section gates this ticket
// on "compiles, and the ported [I2C] sequencing matches source_old
// byte-for-byte" — not PID numerical fidelity, which is ticket 7's bench
// pass.
//
// One documented divergence from source_old's literal behavior: in the
// deadband (integrator-freeze) branch, source_old calls cmon-pid's
// ReInit(0, I_old), which (per the derivation above, with the D register
// holding a stale Kp*err term from the last non-deadband tick) does not
// exactly hold the integrator at I_old despite the code comment's stated
// intent ("keep I where it is"). This port implements that STATED intent
// literally (freeze the integrator unchanged) rather than reproducing the
// stale-D subtraction, which reads as an unintended quirk of the composed
// transfer-function shim rather than a deliberate design element.
//
// Output domain: duty fraction [-1, 1] (matching Hal::Motor::setDutyCycle's
// contract), not the old [-100, 100] PWM-percent domain — MotorConfig.vel_
// gains is a brand-new surface this sprint, so there is no compatibility
// requirement to preserve the old scale; Gains are tuned against this scale
// in ticket 7's bench pass.
// ---------------------------------------------------------------------------
float NezhaMotor::runVelocityPid(float target, float measured, float dt)
{
    if (dt <= 0.0f) dt = kNominalDt;

    const msg::Gains& gains = config_.vel_gains;
    float err = target - measured;
    float spAbs = fabsf(target);
    float spSign = (target >= 0.0f) ? 1.0f : -1.0f;
    float ff = gains.kff * spAbs;

    // Output uses the OLD integrator (pre-update), matching
    // VelocityController::update()'s I_old ordering.
    float iOld = integral_;
    float rawDuty = spSign * ff + gains.kp * err + iOld;
    float output = clampf(rawDuty, -1.0f, 1.0f);

    // config_.min_duty plays minWheelSpeed's role here (integrator-freeze
    // deadband threshold on |target|) despite its proto name — see
    // nezha_motor.h's field comment and the ticket's own note that
    // MotorConfig.min_duty's doc string ("stiction floor / integrator-
    // freeze threshold") is exactly VelocityController's minWheelSpeed
    // semantics, just carried under a different generated field name.
    bool inDeadband = spAbs < config_.min_duty;
    if (!inDeadband) {
        float newIntegral = iOld + gains.ki * dt * err;
        // Anti-windup back-calculation against +/- i_max (mirrors
        // backcalculation_t<pid_bwe>::Update's saturation check on
        // kp*err + newIntegral, with C3=0 and D~=kp*err for Kd=0).
        float u = gains.kp * err + newIntegral;
        float tw = (gains.kaw > 0.0f) ? (1.0f / gains.kaw) : 1e6f;
        float cW = (tw > dt) ? (dt / tw) : 1.0f;
        if (u > gains.i_max) {
            newIntegral += cW * (gains.i_max - u);
        } else if (u < -gains.i_max) {
            newIntegral += cW * (-gains.i_max - u);
        }
        integral_ = newIntegral;
    }
    // else: frozen — integral_ left unchanged (see file-level comment on
    // the deliberate divergence from source_old's ReInit() call here).

    return output;
}

// ---------------------------------------------------------------------------
// Wedge detector — ported from MotorController::controlTick's per-wheel
// stuck-encoder detector (source_old/control/MotorController.cpp), folded
// to a single motor here (no L/R pairing at this tier). Unconditional: not
// gated by commanded target, not gated by an arming grace (both blind
// spots were removed upstream in 064-004; this port preserves the fixed
// version, not the original one).
// ---------------------------------------------------------------------------
void NezhaMotor::updateWedgeDetector(float pos)
{
    if (wedgePrevValid_) {
        if (pos != wedgePrevEnc_) {
            stuckCount_ = 0;
            wedgeLatched_ = false;
        } else if (stuckCount_ < 255) {
            ++stuckCount_;
        }
    }
    wedgePrevEnc_ = pos;
    wedgePrevValid_ = true;

    if (stuckCount_ >= kWedgeThreshold) {
        wedgeLatched_ = true;
    }
}

// ---------------------------------------------------------------------------
// Write path — ported from Motor::setSpeed(). Byte-for-byte identical
// write-on-change guard, write-rate limit, slew cap, and coast-at-zero
// exemption, including the -128 sentinel's interaction with the slew clamp
// on the very first write (source_old passes _lastWrittenSpeed into
// MotorSlew::clampStep() unconditionally, sentinel included — this port
// does the same rather than special-casing the sentinel, per the ticket's
// "port the current behavior... unchanged" directive).
// ---------------------------------------------------------------------------
void NezhaMotor::writeDuty(float duty)
{
    duty = clampf(duty, -1.0f, 1.0f);
    int8_t pct = static_cast<int8_t>(lroundf(duty * 100.0f));
    if (pct > 100) pct = 100;
    if (pct < -100) pct = -100;

    // Write-on-change: skip the I2C write if the command is unchanged.
    if (pct == lastWrittenPct_) {
        return;
    }

    // Write-rate limit — bus hygiene only (NOT a wedge fix; see
    // motor_slew.h and docs/knowledge/2026-07-04-encoder-latch-reversal-
    // write-train.md for the actual trigger). Stop and reversal are exempt.
    static constexpr uint32_t kMinWriteIntervalUs = 40000;   // [us] 40 ms ~= 25 Hz max
    bool stopping = (pct == 0);
    bool reversal = (pct != 0 && lastWrittenPct_ != 0 &&
                     ((pct > 0) != (lastWrittenPct_ > 0)));
    uint64_t now = system_timer_current_time_us();   // [us]
    if (!stopping && !reversal &&
        (now - lastWriteTimeUs_) < kMinWriteIntervalUs) {
        return;
    }

    // |ΔPWM| slew cap — a stop is the explicit, unclamped, immediate-write
    // exemption; every other write (including a full reversal) is stepped
    // by at most config_.slew_rate toward the requested target.
    int8_t written = stopping
        ? pct
        : MotorSlew::clampStep(lastWrittenPct_, pct,
                                static_cast<uint8_t>(config_.slew_rate));

    lastWriteTimeUs_ = now;
    lastWrittenPct_ = written;

    // Apply fwd_sign: positive written = logical forward.
    int16_t effective = static_cast<int16_t>(config_.fwd_sign) * static_cast<int16_t>(written);
    if (effective == 0) {
        // Zero speed: COAST via 0x60 speed 0 — NOT 0x5F, which wedges
        // subsequent encoder reads (see writeMotorRun()).
        writeMotorRun(kDirCw, 0);
    } else {
        uint8_t dir = (effective > 0) ? kDirCw : kDirCcw;
        uint8_t speed = (effective > 0) ? static_cast<uint8_t>(effective)
                                         : static_cast<uint8_t>(-effective);
        writeMotorRun(dir, speed);
    }
}

void NezhaMotor::writeMotorRun(uint8_t direction, uint8_t speed)
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
    bus_.write((kAddr << 1), buf, 8, false);
}

void NezhaMotor::writePositionMove(float positionDeg)
{
    // Frame verified against pxt-nezha2/main.ts moveToAbsAngle() (ported
    // from Motor::moveToAngle()):
    //   [0xFF, 0xF9, motorId, 0x00, 0x5D, angleHigh, mode, angleLow]
    // angle: 0-359 (big-endian across buf[5]/buf[7]); mode: 1=shortest
    // path, 2=CW, 3=CCW (ServoMotionMode). MotorCommand carries no mode
    // selector, so this always requests the shortest path (mode 1).
    int angleInt = static_cast<int>(lroundf(positionDeg)) % 360;
    if (angleInt < 0) angleInt += 360;
    uint16_t angle = static_cast<uint16_t>(angleInt);
    static constexpr uint8_t kShortestPath = 1;

    uint8_t buf[8] = {
        0xFF, 0xF9,
        static_cast<uint8_t>(config_.port),
        0x00,
        0x5D,
        static_cast<uint8_t>(angle >> 8),
        kShortestPath,
        static_cast<uint8_t>(angle & 0xFF)
    };
    bus_.write((kAddr << 1), buf, 8, false);

    // BUG-CRITICAL post-write busy-wait (ported verbatim — see
    // source_old/hal/real/Motor.cpp::moveToAngle() for the vendor
    // rationale: no task/fiber may interleave during this window).
    uint64_t deadline = system_timer_current_time_us() + 4000;
    while (system_timer_current_time_us() < deadline) {}
}

// ---------------------------------------------------------------------------
// Encoder reads — all ported byte-for-byte from source_old/hal/real/
// Motor.cpp. See nezha_motor.h for which of these tick() actually calls.
// ---------------------------------------------------------------------------

float NezhaMotor::readEncoderSettle()
{
    // Settle-only read — skips the 4ms pre-write bus-idle (redundant in the
    // fixed-rate tick loop, which leaves the bus naturally idle between
    // ticks). Cost: ~4ms. This is the path tick() uses every cycle.
    static constexpr uint32_t kSettleUs = 4000;
    uint8_t cmd[8] = { 0xFF, 0xF9, static_cast<uint8_t>(config_.port), 0x00, 0x46, 0x00, 0xF5, 0x00 };
    int writeResult = bus_.write((kAddr << 1), cmd, 8, false);
    uint64_t deadline = system_timer_current_time_us() + kSettleUs;
    while (system_timer_current_time_us() < deadline) {}
    uint8_t resp[4] = { 0, 0, 0, 0 };
    int readResult = bus_.read((kAddr << 1), resp, 4, false);

    connected_ = (writeResult == MICROBIT_OK && readResult == MICROBIT_OK);

    int32_t raw;
    if (!connected_) {
        // CR-03 pattern: hold the last known-good value instead of
        // computing from a zeroed response buffer (which would read as a
        // fabricated large jump).
        raw = lastGoodRawEnc_;
    } else {
        raw = static_cast<int32_t>(
            (static_cast<uint32_t>(resp[3]) << 24) |
            (static_cast<uint32_t>(resp[2]) << 16) |
            (static_cast<uint32_t>(resp[1]) << 8) |
            static_cast<uint32_t>(resp[0]));
        raw -= encOffset_;
        lastGoodRawEnc_ = raw;
    }
    return (static_cast<float>(raw) / 10.0f) * config_.travel_calib * static_cast<float>(config_.fwd_sign);
}

int32_t NezhaMotor::readEncoderAtomicRaw()
{
    // Full vendor timing: 4ms pre-write bus-idle -> 0x46 write -> 4ms
    // post-write settle -> read 4 bytes. Cost: ~8ms. Used only by
    // hardResetEncoder()'s median-of-3 snapshot + readback verification.
    static constexpr uint32_t kDelayUs = 4000;

    uint64_t deadline = system_timer_current_time_us() + kDelayUs;
    while (system_timer_current_time_us() < deadline) {}

    uint8_t cmd[8] = { 0xFF, 0xF9, static_cast<uint8_t>(config_.port), 0x00, 0x46, 0x00, 0xF5, 0x00 };
    int writeResult = bus_.write((kAddr << 1), cmd, 8, false);

    deadline = system_timer_current_time_us() + kDelayUs;
    while (system_timer_current_time_us() < deadline) {}

    uint8_t resp[4] = { 0, 0, 0, 0 };
    int readResult = bus_.read((kAddr << 1), resp, 4, false);

    connected_ = (writeResult == MICROBIT_OK && readResult == MICROBIT_OK);
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

void NezhaMotor::requestEncoder()
{
    // Split-phase phase 1 — ported byte-for-byte from Motor::
    // requestEncoder(). Not called by tick() this ticket: source_old's own
    // live NezhaHAL::tick() also does not call this pair (it uses
    // readEncoderSettle() per motor, same as this class's tick()) — this
    // pair exists in source_old today as an available-but-currently-unused
    // API surface (still required by IVelocityMotor there), and this port
    // preserves that same shape/availability rather than inventing a new
    // call site for it.
    uint8_t cmd[8] = { 0xFF, 0xF9, static_cast<uint8_t>(config_.port), 0x00, 0x46, 0x00, 0xF5, 0x00 };
    int writeResult = bus_.write((kAddr << 1), cmd, 8, false);
    pendingEncRequestOk_ = (writeResult == MICROBIT_OK);
}

int32_t NezhaMotor::collectEncoder()
{
    // Split-phase phase 2 — ported byte-for-byte from Motor::
    // collectEncoder(). See requestEncoder()'s comment for why this is not
    // wired into tick() this ticket.
    uint8_t resp[4] = { 0, 0, 0, 0 };
    int readResult = bus_.read((kAddr << 1), resp, 4, false);

    if (!pendingEncRequestOk_ || readResult != MICROBIT_OK) {
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

void NezhaMotor::hardResetEncoder()
{
    // Ported from Motor::resetEncoder(): median-of-3 atomic-read snapshot +
    // readback-verify + retry, matching source_old's kMaxRetries/
    // kReadbackThreshold constants exactly.
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
            return;
        }
        encOffset_ -= snapshot;
    }

    // Retries exhausted: apply the last snapshot anyway (matches
    // source_old's best-effort fallback).
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
}

// ---------------------------------------------------------------------------
// Vendor register wrappers ported for completeness (matching source_old's
// coverage) but not reachable from the public faceplate this sprint.
// ---------------------------------------------------------------------------

void NezhaMotor::timedMove(uint8_t dir, int16_t value, uint8_t mode)
{
    // Frame verified against pxt-nezha2/main.ts __move(): value is
    // big-endian across buf[5]/buf[7] with mode interleaved at buf[6].
    uint8_t buf[8] = {
        0xFF, 0xF9,
        static_cast<uint8_t>(config_.port),
        dir,
        0x70,
        static_cast<uint8_t>(static_cast<uint16_t>(value) >> 8),
        mode,
        static_cast<uint8_t>(static_cast<uint16_t>(value) & 0xFF)
    };
    bus_.write((kAddr << 1), buf, 8, false);
}

void NezhaMotor::resetHome()
{
    // Frame verified against pxt-nezha2/main.ts reset(). Vendor also waits
    // ~1s for the motor to physically reach home; callers of this
    // (currently unreachable) wrapper are responsible for that delay.
    uint8_t buf[8] = {
        0xFF, 0xF9,
        static_cast<uint8_t>(config_.port),
        0x00, 0x1D,
        0x00, 0xF5,
        0x00
    };
    bus_.write((kAddr << 1), buf, 8, false);
}

void NezhaMotor::setGlobalSpeed(uint8_t speed)
{
    // Frame verified against pxt-nezha2/main.ts setServoSpeed(); board-
    // global (motorId byte is 0x00, not config_.port).
    if (speed > 100) speed = 100;
    uint16_t speedEnc = static_cast<uint16_t>(speed) * 9;   // 0-900

    uint8_t buf[8] = {
        0xFF, 0xF9,
        0x00,
        0x00,
        0x77,
        static_cast<uint8_t>(speedEnc >> 8),
        0x00,
        static_cast<uint8_t>(speedEnc & 0xFF)
    };
    bus_.write((kAddr << 1), buf, 8, false);
}

bool NezhaMotor::readVersion(uint8_t& maj, uint8_t& min, uint8_t& patch)
{
    // Frame verified against pxt-nezha2/main.ts readVersion(); board-global.
    uint8_t cmd[8] = {
        0xFF, 0xF9,
        0x00, 0x00,
        0x88,
        0x00, 0x00, 0x00
    };
    int writeResult = bus_.write((kAddr << 1), cmd, 8, false);
    if (writeResult != MICROBIT_OK) {
        maj = min = patch = 0;
        return false;
    }
    uint8_t resp[3] = { 0, 0, 0 };
    int readResult = bus_.read((kAddr << 1), resp, 3, false);
    if (readResult != MICROBIT_OK) {
        maj = min = patch = 0;
        return false;
    }
    maj = resp[0];
    min = resp[1];
    patch = resp[2];
    return true;
}

}  // namespace Hal
