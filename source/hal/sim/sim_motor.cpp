#include "hal/sim/sim_motor.h"

#include <math.h>

namespace Hal {

namespace {
float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
}  // namespace

SimMotor::SimMotor(PhysicsWorld& plant, Side side, const msg::MotorConfig& config)
    : plant_(&plant), side_(side)
{
    // configure() (Hal::Motor, base) caches the two armor fields then calls
    // back into configureDevice() (this class) — called from the
    // constructor BODY, not a Motor(...) base constructor, so the object's
    // dynamic type is already SimMotor and the virtual dispatch resolves
    // correctly (same construction-order note as NezhaMotor's constructor).
    configure(config);
}

SimMotor::SimMotor(const msg::MotorConfig& config)
    : plant_(nullptr), side_(Side::LEFT)
{
    configure(config);
}

void SimMotor::begin()
{
    // Parity with NezhaMotor::begin(): zero whichever accumulator this
    // motor is bound to.
    hardReset();
}

void SimMotor::bindToPlant(PhysicsWorld* plant, Side side)
{
    plant_ = plant;
    side_ = side;
    // Rebinding changes the position source out from under the cached
    // encoder sample -- rebaseline the tick cache so the next tick() does
    // not compute a bogus velocity spike from a now-meaningless
    // lastPosition_. Does NOT touch the underlying accumulator (the
    // plant's reported encoder, or this motor's own standalone
    // accumulator) -- only this motor's own sampling cache.
    lastPosition_ = 0.0f;
    filteredVelocity_ = 0.0f;
    hasLastTick_ = false;
}

void SimMotor::configureDevice(const msg::MotorConfig& config)
{
    // No slew_rate defaulting needed here (unlike NezhaMotor): DUTY mode
    // stages straight into the plant with no motor-side write shaping —
    // see the file header. config_ still carries vel_gains/min_duty, the
    // two fields tick()'s VELOCITY case actually consumes.
    config_ = config;
}

msg::MotorCapabilities SimMotor::capabilities() const
{
    msg::MotorCapabilities caps;
    caps.duty_cycle = true;
    caps.voltage = false;     // no voltage-mode register to model
    caps.velocity = true;     // Hal::MotorVelocityPid, closed over the plant's reported encoder
    caps.position = false;    // no onboard absolute-angle move in the sim (see file header)
    caps.has_encoder = true;
    return caps;
}

// ---------------------------------------------------------------------------
// Primitive setters — stage the command; tick() executes it.
// ---------------------------------------------------------------------------

void SimMotor::setDutyCycle(float dutyCycle)
{
    mode_ = Mode::DUTY;
    dutyTarget_ = dutyCycle;
}

void SimMotor::setVoltage(float voltage)
{
    // Unsupported (capabilities().voltage == false). apply() gates this
    // before it is ever called; the body is a documented no-op rather than
    // an assert so a direct (non-apply()) call from test code cannot crash.
    (void)voltage;
}

void SimMotor::setVelocity(float velocity)
{
    mode_ = Mode::VELOCITY;
    velocityTarget_ = velocity;
}

void SimMotor::setPosition(float position)
{
    // Unsupported (capabilities().position == false) — see file header.
    (void)position;
}

void SimMotor::setNeutral(msg::Neutral mode)
{
    mode_ = Mode::NEUTRAL;
    neutralTarget_ = mode;
}

void SimMotor::setFeedforward(float feedforward)
{
    feedforward_ = feedforward;
}

// ---------------------------------------------------------------------------
// Primitive getters.
// ---------------------------------------------------------------------------

float SimMotor::position() const { return lastPosition_; }
float SimMotor::velocity() const { return filteredVelocity_; }

// Reads the base's own protected lastRequestedDuty_ directly: the last duty
// armoredWrite() actually forwarded to writeRawDuty() (base-maintained,
// architecture-update.md's shared armor policy). Unlike NezhaMotor (which
// slew-limits its I2C write and so tracks its OWN lastWrittenPct_,
// deliberately different from the commanded value mid-ramp), SimMotor's
// writeRawDuty() stages duty STRAIGHT into the plant with no shaping of its
// own, so the base's "last commanded" value and "what's driving the plant"
// are the same fraction (mod the plant's own int8 percent rounding) —
// reusing lastRequestedDuty_ here avoids a redundant duplicate field.
float SimMotor::appliedDuty() const { return lastRequestedDuty_; }

bool SimMotor::connected() const { return true; }   // no I2C link to fail

// ---------------------------------------------------------------------------
// tick() — mirrors NezhaMotor::tick()'s call-order contract, MINUS the wedge
// detector (see file header for why): 1. standstill-guarded reset dispatch;
// 2. sample + cache this motor's encoder; 3. mode dispatch (DUTY/VELOCITY/
// NEUTRAL route through armoredWrite()); 4. rest tracking; 5. (unbound only)
// advance the trivial standalone integrator with this tick's just-staged
// duty, for the NEXT tick to sample -- the same one-tick latency the
// plant-bound path gets for free from Subsystems::SimHardware calling
// Hal::PhysicsWorld::update() once, AFTER every motor's tick(), each pass.
// ---------------------------------------------------------------------------
void SimMotor::tick(uint32_t nowMs)
{
    // 1. Standstill-guarded reset dispatch.
    processResetIfPending(nowMs);

    // 2. Per-tick position sample.
    float pos = encoderPosition();

    float elapsedTime = 0.0f;   // [s]
    uint32_t elapsedMs = 0;     // [ms]
    bool haveElapsed = false;
    if (hasLastTick_) {
        elapsedMs = nowMs - lastTick_;
        elapsedTime = static_cast<float>(elapsedMs) / 1000.0f;
        if (elapsedTime > 0.0f) {
            haveElapsed = true;
            float rawVel = (pos - lastPosition_) / elapsedTime;
            float a = config_.vel_filt_alpha;   // EMA smoothing
            filteredVelocity_ = a * rawVel + (1.0f - a) * filteredVelocity_;

            // 099-003: generic per-motor acceleration EMA (base policy,
            // deliberately separate from Drivetrain's own accelEma_ — see
            // architecture-update.md Decision 3). Converts the existing
            // millisecond elapsedMs to microseconds.
            trackAcceleration(velocity(), elapsedMs * 1000);
        }
    } else {
        hasLastTick_ = true;
    }
    lastPosition_ = pos;
    lastTick_ = nowMs;

    // 3. Mode dispatch.
    switch (mode_) {
        case Mode::DUTY:
            armoredWrite(dutyTarget_, nowMs);
            break;
        case Mode::VELOCITY: {
            // dt<=0 (haveElapsed false -- the very first tick, or a clock
            // glitch) is passed straight through as 0.0f: Hal::
            // MotorVelocityPid::compute() already substitutes its own
            // kNominalDt fallback internally, so no local duplicate of that
            // constant is needed here (unlike NezhaMotor, which pre-
            // substitutes explicitly — functionally equivalent either way).
            float dt = haveElapsed ? elapsedTime : 0.0f;
            float duty = pid_.compute(velocityTarget_, filteredVelocity_, dt,
                                       config_.vel_gains, config_.min_duty);
            armoredWrite(clampf(duty + feedforward_, -1.0f, 1.0f), nowMs);
            break;
        }
        case Mode::NEUTRAL:
            // One physical stop path, same as a Nezha with no distinct
            // brake register — both BRAKE and COAST land here.
            (void)neutralTarget_;
            armoredWrite(0.0f, nowMs);
            break;
        case Mode::NONE:
        default:
            break;
    }

    // 4. Rest tracking (base armor policy).
    updateRestTracking();

    // 5. Standalone-only: advance the trivial integrator with this tick's
    // just-staged duty (writeRawDuty() above already updated standalonePwm_
    // when plant_ == nullptr).
    if (plant_ == nullptr && haveElapsed) {
        tickStandalone(elapsedMs);
    }
}

// ---------------------------------------------------------------------------
// Write path — DUTY straight to the plant (or the standalone integrator);
// no slew limiting, no write-rate throttle (those are Nezha-specific I2C-bus
// hygiene concerns; the plant applies its OWN stiction gate + optional lag
// inside Hal::PhysicsWorld::update(), not here).
// ---------------------------------------------------------------------------
void SimMotor::writeRawDuty(float duty)
{
    duty = clampf(duty, -1.0f, 1.0f);
    int32_t pctRounded = static_cast<int32_t>(lroundf(duty * 100.0f));
    if (pctRounded > 100) pctRounded = 100;
    if (pctRounded < -100) pctRounded = -100;
    int8_t pct = static_cast<int8_t>(pctRounded);

    if (plant_ != nullptr) {
        plant_->setActuator(sideIndex(), pct);
    } else {
        standalonePwm_ = pct;
    }
}

void SimMotor::hardReset()
{
    if (plant_ != nullptr) {
        plant_->resetReportedEncoder(sideIndex());
    } else {
        standaloneEnc_ = 0.0f;
    }
    lastPosition_ = 0.0f;
    filteredVelocity_ = 0.0f;
    hasLastTick_ = false;
    // hardResetCount_ is incremented by the BASE's processResetIfPending()
    // (motor.h), not here — mirrors NezhaMotor::hardReset()'s contract.
}

void SimMotor::softRebaseline()
{
    // The sim has no I2C timing race to avoid (no real bus, no atomic-read
    // burst to latch), so this performs the SAME effect hardReset() already
    // does above. Only the reset-kind counter differs (softResetCount_, not
    // hardResetCount_) — mirrors NezhaMotor::softRebaseline()'s contract.
    if (plant_ != nullptr) {
        plant_->resetReportedEncoder(sideIndex());
    } else {
        standaloneEnc_ = 0.0f;
    }
    lastPosition_ = 0.0f;
    filteredVelocity_ = 0.0f;
    hasLastTick_ = false;
    ++softResetCount_;
}

float SimMotor::encoderPosition() const
{
    if (plant_ != nullptr) {
        return (side_ == Side::LEFT) ? plant_->reportedEncL() : plant_->reportedEncR();
    }
    return standaloneEnc_;
}

void SimMotor::tickStandalone(uint32_t dt)
{
    if (dt == 0) return;
    float dt_s = static_cast<float>(dt) / 1000.0f;
    // Deliberately trivial: no slip/noise/stiction/lag — see file header.
    float vel = (static_cast<float>(standalonePwm_) / 100.0f) * PhysicsWorld::kNominalMaxSpeed;
    standaloneEnc_ += vel * dt_s;
}

}  // namespace Hal
