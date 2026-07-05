#include "hal/nezha/nezha_motor.h"
#include "hal/nezha/motor_slew.h"

#ifndef HOST_BUILD
#include "MicroBit.h"   // MICROBIT_OK, system_timer_current_time_us() (vendor SDK; excluded from the
                        // no-units-in-identifiers rename per .claude/rules/coding-standards.md)
#else
// HOST_BUILD (sprint 079-004's flip-flop host harness,
// tests/sim/unit/nezha_flipflop_harness.cpp): no CODAL, so no MicroBit.h.
// Mirrors i2c_bus.h's own #ifndef HOST_BUILD guard. MICROBIT_OK is CODAL's
// well-known convention (0 == success — see i2c_bus_host.cpp's identical
// note); system_timer_current_time_us() delegates to I2CBus's own shared,
// test-settable fake clock (nezha_motor.h already pulls in com/i2c_bus.h)
// so writeRawDuty()'s 40ms write-rate throttle stays deterministic and
// test-controllable under the SAME clock the scripted bus itself runs
// against, rather than reading a wall clock that does not exist here.
namespace {
constexpr int MICROBIT_OK = 0;
uint64_t system_timer_current_time_us() { return I2CBus::clock(); }   // [us]
}  // namespace
#endif
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

float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
}  // namespace

NezhaMotor::NezhaMotor(I2CBus& bus, const msg::MotorConfig& config)
    : bus_(bus)
{
    // configure() (Hal::Motor, base) caches the two armor fields
    // (reversal_dwell/output_deadband, defaulting when unset) then calls
    // back into configureDevice() (this class) for the device-specific
    // fields (slew_rate defaulting, etc.). Called from the constructor
    // BODY, not a Motor(...) base constructor — architecture-update.md's
    // Construction note: by this point the object's dynamic type is already
    // NezhaMotor, so the virtual dispatch into configureDevice() resolves
    // correctly (unlike a call from Motor's own constructor, which would
    // not dispatch to this override).
    configure(config);
}

void NezhaMotor::begin()
{
    // Ported from Motor::begin(): un-freezes the 0x46 readback and zeros
    // the software offset — the correct initial state for a fresh boot.
    hardReset();
}

void NezhaMotor::configureDevice(const msg::MotorConfig& config)
{
    config_ = config;
    if (config_.slew_rate <= 0.0f) {
        // architecture-update.md Design Rationale 2: MotorConfig.slew_rate
        // defaults to the existing kMaxDeltaPwmPerWrite value (25) when
        // unconfigured (zero-initialized), matching source_old's file-local
        // constexpr default.
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

// ---------------------------------------------------------------------------
// tick() — the leaf's 5-step call-order contract (architecture-update.md,
// "The base/leaf split — exact contract"):
//   1. processResetIfPending(now)  — base armor policy, before this tick's
//      encoder sample.
//   2. sample + cache this motor's own encoder (device-specific, unchanged).
//   3. updateWedgeDetector()       — base armor policy; reads position()/
//      appliedDuty(), both now reflecting this tick's fresh sample and last
//      tick's write.
//   4. mode dispatch — DUTY/VELOCITY/NEUTRAL route through armoredWrite();
//      POSITION calls writePositionMove() directly (out of the armor's
//      scope — a discrete onboard move, not a streamed signed duty).
//   5. updateRestTracking()        — base armor policy; reads velocity()
//      and lastRequestedDuty_, the latter possibly just updated by step 4.
// ---------------------------------------------------------------------------
void NezhaMotor::tick(uint32_t nowMs)
{
    // 1. Standstill-guarded reset dispatch.
    processResetIfPending(nowMs);

    // 2. Per-tick position sample — sprint 079-004: collects a sample that
    // NezhaHardware's brick flip-flop REQUESTED in a previous slice (requestSample()
    // -> requestEncoder()) and has already confirmed is safe to collect
    // (bus_.clear(kNezhaDeviceAddr) gated the call into this tick() at all —
    // see subsystems/nezha_hardware.cpp). Non-blocking: no write here, no spin, just the 4-byte
    // read. Replaces the former fused, always-blocking readEncoderSettle()
    // (deleted — see nezha_motor.h).
    int32_t raw = collectEncoder();
    float pos = (static_cast<float>(raw) / 10.0f)
              * config_.travel_calib * static_cast<float>(config_.fwd_sign);

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

    // 3. Wedge detector — reads position() (== pos, just cached above) and
    // appliedDuty() (last tick's write; this tick's mode dispatch has not
    // run yet).
    updateWedgeDetector();

    // 4. Mode dispatch.
    switch (mode_) {
        case Mode::DUTY:
            armoredWrite(dutyTarget_, nowMs);
            break;
        case Mode::VELOCITY: {
            float dt = haveElapsed ? elapsedTime : kNominalDt;
            float duty = runVelocityPid(velocityTarget_, filteredVelocity_, dt);
            armoredWrite(clampf(duty + feedforward_, -1.0f, 1.0f), nowMs);
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
            armoredWrite(0.0f, nowMs);
            break;
        case Mode::NONE:
        default:
            break;
    }

    // 5. Rest tracking.
    updateRestTracking();
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
// Write path — ported from Motor::setSpeed(). Byte-for-byte identical
// write-on-change guard, write-rate limit, slew cap, and coast-at-zero
// exemption, including the -128 sentinel's interaction with the slew clamp
// on the very first write (source_old passes _lastWrittenSpeed into
// MotorSlew::clampStep() unconditionally, sentinel included — this port
// does the same rather than special-casing the sentinel, per the ticket's
// "port the current behavior... unchanged" directive).
//
// Sprint 078: this is now writeRawDuty(), Hal::Motor's protected device-
// specific primitive, called only from armoredWrite() (base). The former
// `reversal` boolean and its 40-ms-throttle exemption branch are DELETED,
// not preserved as dead code (architecture-update.md Design Rationale 6):
// armoredWrite() never forwards a raw sign flip here any more — it always
// writes 0 first and holds through the dwell, so by the time a new-
// direction duty reaches this function, lastWrittenPct_ is already 0 (from
// the dwell's own zero-writes). Every write this function ever sees is
// either unchanged, a transition to/from zero, or a same-sign change —
// never a direct opposite-sign jump.
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

    // Write-rate limit — bus hygiene only (NOT a wedge fix; see
    // motor_slew.h and docs/knowledge/2026-07-04-encoder-wedge.md for the
    // actual trigger). Stop is the only throttle exemption; see the
    // function-level comment above for why a reversal exemption is no
    // longer needed here.
    static constexpr uint32_t kMinWriteIntervalUs = 40000;   // [us] 40 ms ~= 25 Hz max
    bool stopping = (pct == 0);
    uint64_t now = system_timer_current_time_us();   // [us]
    if (!stopping && (now - lastWriteTimeUs_) < kMinWriteIntervalUs) {
        return;
    }

    // |ΔPWM| slew cap — a stop is the explicit, unclamped, immediate-write
    // exemption; every other write is stepped by at most config_.slew_rate
    // toward the requested target.
    //
    // 079-006 root-cause fix: the very first write for a port sees
    // lastWrittenPct_ still at its -128 "no write yet" sentinel.
    // MotorSlew::clampStep() has no concept of that sentinel — fed -128
    // unconditionally, it computes a step toward pct that OVERSHOOTS the
    // valid ±100 range on the low side (e.g. clampStep(-128, 30, 25) = -103)
    // and, critically, has the OPPOSITE SIGN of the actually-requested
    // duty. writeMotorRun() below dispatches direction from written's sign,
    // so this sent a full-swing, wrong-direction 0x60 write (an unrequested
    // reversal, with an out-of-spec speed byte > the register's documented
    // 0-100 range) as literally the first command ever issued to a fresh
    // port -- exactly docs/knowledge/2026-07-04-encoder-wedge.md's confirmed
    // reversal-write-train latch trigger, on the very first tick a
    // never-before-addressed port is ever driven. Confirmed on hardware
    // (079-006 stand campaign): every fresh port's first DUTY/VEL command
    // latched its 0x46 readback (wedged=1 within ~1s, pos/vel frozen at the
    // post-reset baseline forever after, surviving even a genuine
    // standstill-guarded hard reset -- consistent with the doc's
    // "repeated abuse escalates to a persistent latch" escalation path
    // once this fires on every cold-started test). The armor's own
    // reversal-dwell gate (Motor::armoredWrite(), 078) cannot catch this:
    // it compares the commanded float duty against lastRequestedDuty_
    // (sane 0.0f default), sees no sign change relative to that baseline,
    // and forwards straight to writeRawDuty() -- the bogus reversal is
    // manufactured one layer further down, from a SEPARATE sentinel this
    // function alone owns. Fix: treat "no write yet" as the same
    // unclamped-write exemption stop() already gets -- there is no prior
    // direction to slew from, so the first write goes straight to the
    // requested pct, matching what a genuinely fresh motor should see.
    bool firstWrite = (lastWrittenPct_ == -128);
    int8_t written = (stopping || firstWrite)
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
    // 079-006 root-cause fix: postClear=4000 holds off the next transaction
    // to 0x10 (in this single-in-use-port flip-flop, that's the SAME port's
    // own next 0x46 encoder request) for a real >=4ms gap after this duty
    // write, mirroring requestEncoder()'s own postClear on the other side of
    // the cycle. Before this fix, writeMotorRun() carried no clearance at
    // all: a single in-use port's own REQUEST_DUE fires again on the very
    // next NezhaHardware::tick() call with no other port to interleave, so the
    // 0x46 request could re-issue with ~0us real gap since this write ended.
    // Confirmed on hardware (079-006 stand campaign, via pyOCD/gdb
    // backtraces caught mid-stall): that back-to-back cadence reliably
    // parked the firmware for several seconds at a time inside vendor CODAL
    // code (libraries/codal-nrf52/source/NRF52I2C.cpp's waitForStop(),
    // stuck spinning up to its own ~10s internal timeout waiting for a
    // TWIM STOPPED event that never arrived) -- the entire main loop,
    // including serial command processing, froze for the stall's duration.
    // The OLD fused/blocking readEncoderSettle() never hit this because its
    // own hand-rolled spins gave every 0x10 transaction real elapsed time on
    // both sides; the split-phase design's whole point (issuing more bus
    // work per main-loop pass) removed that incidental protection. Long-
    // duration hardware soaks (tests/bench/, see this ticket's stand
    // results) confirmed this single postClear addition (paired with
    // requestEncoder()'s own preClear=4000 below) collapses the multi-
    // second stalls down to the ordinary, already-documented USB-CDC
    // transport drop rate.
    bus_.write((kNezhaDeviceAddr << 1), buf, 8, false, /*preClear=*/0, /*postClear=*/4000);
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
    // BUG-CRITICAL post-write settle (ported verbatim — see source_old/
    // hal/real/Motor.cpp::moveToAngle() for the vendor rationale: no task/
    // fiber may interleave during this window). Sprint 079: the busy-wait
    // moves into I2CBus itself as a lazy postClear deadline on this write
    // (architecture-update.md's "I2CBus lazy-clearance mechanism" section)
    // instead of a hand-rolled spin here.
    bus_.write((kNezhaDeviceAddr << 1), buf, 8, false, /*preClear=*/0, /*postClear=*/4000);
}

// ---------------------------------------------------------------------------
// Encoder reads — all ported byte-for-byte from source_old/hal/real/
// Motor.cpp. See nezha_motor.h for which of these tick() actually calls.
//
// readEncoderSettle() (the fused, always-blocking write -> 4ms spin -> read)
// is DELETED as of sprint 079-004 — its sole caller was tick()'s step 2,
// which now calls collectEncoder() instead (see tick() above). Deleting
// rather than keeping it as unreferenced dead code matches 078's Design
// Rationale 6 precedent (writeDuty()'s reversal-exemption branch).
// ---------------------------------------------------------------------------

int32_t NezhaMotor::readEncoderAtomicRaw()
{
    // Full vendor timing: 4ms pre-write bus-idle -> 0x46 write -> 4ms
    // post-write settle -> read 4 bytes. Cost: ~8ms. Used only by
    // hardReset()'s median-of-3 snapshot + readback verification. Sprint
    // 079: both hand-rolled spins move into I2CBus itself as lazy
    // preClear/postClear deadlines on the write (architecture-update.md's
    // "I2CBus lazy-clearance mechanism" section) instead of manual
    // while-loops here.
    static constexpr uint32_t kDelayUs = 4000;

    uint8_t cmd[8] = { 0xFF, 0xF9, static_cast<uint8_t>(config_.port), 0x00, 0x46, 0x00, 0xF5, 0x00 };
    int writeResult = bus_.write((kNezhaDeviceAddr << 1), cmd, 8, false,
                                 /*preClear=*/kDelayUs, /*postClear=*/kDelayUs);

    uint8_t resp[4] = { 0, 0, 0, 0 };
    int readResult = bus_.read((kNezhaDeviceAddr << 1), resp, 4, false);

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

void NezhaMotor::requestSample()
{
    // Public split-phase phase-1 entry point (sprint 079-004) — the ONLY
    // caller is NezhaHardware's brick flip-flop sequencer (subsystems/nezha_hardware.cpp's
    // REQUEST_DUE case), once per bus slice, only for the currently-active
    // in-use port. See nezha_motor.h's declaration for why this is not a
    // Hal::Motor virtual.
    requestEncoder();
}

void NezhaMotor::requestEncoder()
{
    // Split-phase phase 1 — ported byte-for-byte from Motor::
    // requestEncoder(). As of sprint 079-004 this IS wired into the live
    // schedule: NezhaHardware::tick()'s REQUEST_DUE case calls requestSample()
    // (above), which wraps this. postClear=4000 attaches the settle window
    // to THIS write's I2CBus deadline (per-device, not per-call-site),
    // holding off any subsequent transaction to 0x10 -- including a stray
    // 0x60 velocity write mid-settle -- until collectEncoder() (or any
    // other 0x10 call) is due (architecture-update.md's "I2CBus
    // lazy-clearance mechanism" section, constraint 4).
    //
    // 079-006 root-cause fix: preClear=4000 (originally 0) holds THIS write
    // back until a real >=4ms has elapsed since the LAST transaction to
    // 0x10 -- whatever it was (this same port's own preceding duty write in
    // the common single-in-use-port case; another port's traffic when 2+
    // are in use). Ticket 004/005 landed this write with no preClear at
    // all, on the theory the flip-flop's own scheduling gap would always
    // be ample (the way the old fused readEncoderSettle()'s per-tick
    // cadence always left slack) -- confirmed FALSE on hardware for a
    // single in-use port: REQUEST_DUE fires again on the very next
    // NezhaHardware::tick() call with no other port to interleave, so this
    // write could re-issue with ~0us real gap since the preceding
    // COLLECT_DUE's duty write. Root-caused via pyOCD/gdb backtraces caught
    // mid-stall (this ticket's stand campaign): the firmware was parked for
    // several seconds inside vendor CODAL's NRF52I2C::waitForStop()
    // (libraries/codal-nrf52/source/NRF52I2C.cpp), spinning toward its own
    // ~10s internal timeout waiting for a TWIM STOPPED event that never
    // arrived -- the whole main loop (serial included) froze for the
    // stall's duration. Restoring the real ">=4ms since the last 0x10
    // transaction" gap the old fused code always had (paired with
    // writeMotorRun()'s new postClear=4000 below) collapsed these stalls
    // down to the ordinary, already-documented USB-CDC transport drop rate
    // in long-duration hardware soaks (see this ticket's stand results).
    uint8_t cmd[8] = { 0xFF, 0xF9, static_cast<uint8_t>(config_.port), 0x00, 0x46, 0x00, 0xF5, 0x00 };
    int writeResult = bus_.write((kNezhaDeviceAddr << 1), cmd, 8, false,
                                 /*preClear=*/4000, /*postClear=*/4000);
    pendingEncRequestOk_ = (writeResult == MICROBIT_OK);
}

int32_t NezhaMotor::collectEncoder()
{
    // Split-phase phase 2 — ported byte-for-byte from Motor::
    // collectEncoder(), PLUS the one line it was missing to be tick()-safe
    // (sprint 079-004): connected_ now reflects BOTH halves of the split
    // transaction (the request's write status AND this collect's read
    // status), not just the read — a request whose 0x46 write itself
    // failed must not be reported as connected merely because the
    // subsequent read happened to succeed against stale/garbage state.
    // As of this ticket, tick()'s step 2 calls this directly (see tick()
    // above) once NezhaHardware's flip-flop has confirmed the settle window
    // elapsed (bus_.clear(kNezhaDeviceAddr)).
    uint8_t resp[4] = { 0, 0, 0, 0 };
    int readResult = bus_.read((kNezhaDeviceAddr << 1), resp, 4, false);

    connected_ = pendingEncRequestOk_ && (readResult == MICROBIT_OK);

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
    // Ported from Motor::resetEncoder(): median-of-3 atomic-read snapshot +
    // readback-verify + retry, matching source_old's kMaxRetries/
    // kReadbackThreshold constants exactly. Unchanged from the pre-078
    // hardResetEncoder() body — see architecture-update.md's leaf mapping
    // table. processResetIfPending() (base) increments hardResetCount_
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

void NezhaMotor::softRebaseline()
{
    // Ported from source_old's Motor::rebaselineSoft() (064-003):
    // software-only encoder rebaseline — folds the already-tick-cached
    // lastPosition_ (populated by this tick's collectEncoder() call, not a
    // new atomic read) back into raw tenths-of-degrees and adds it to
    // encOffset_, then zeroes the cache exactly as hardReset()'s success
    // path does. Issues NO I2C transaction at all.
    //
    // Inverse of tick()'s collectEncoder()-based conversion (mm = (raw/10) *
    // travel_calib * fwd_sign): rawDelta = (mm / (travel_calib * fwd_sign))
    // * 10. The new tree has one travel_calib/fwd_sign per port, not per
    // L/R side (nezha_motor.h's class comment, Design Rationale 3), so no
    // L/R selection is needed here (unlike source_old's _motorId-gated
    // wheelTravelCalibL/wheelTravelCalibR pick).
    if (config_.travel_calib != 0.0f) {
        float rawDeltaF = (lastPosition_ / (config_.travel_calib * static_cast<float>(config_.fwd_sign))) * 10.0f;
        encOffset_ += static_cast<int32_t>(rawDeltaF);
    }
    lastPosition_ = 0.0f;
    filteredVelocity_ = 0.0f;
    hasLastTick_ = false;
    lastGoodRawEnc_ = 0;

    // softResetCount_ is base-owned (Hal::Motor); this leaf increments it
    // directly (it is protected, inherited) rather than duplicating a
    // counter here — mirrors source_old's rebaselineSoft(), which
    // increments its own _softResetCount as its last statement.
    ++softResetCount_;
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
    bus_.write((kNezhaDeviceAddr << 1), buf, 8, false);
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
    bus_.write((kNezhaDeviceAddr << 1), buf, 8, false);
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
    bus_.write((kNezhaDeviceAddr << 1), buf, 8, false);
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
    int writeResult = bus_.write((kNezhaDeviceAddr << 1), cmd, 8, false);
    if (writeResult != MICROBIT_OK) {
        maj = min = patch = 0;
        return false;
    }
    uint8_t resp[3] = { 0, 0, 0 };
    int readResult = bus_.read((kNezhaDeviceAddr << 1), resp, 3, false);
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
