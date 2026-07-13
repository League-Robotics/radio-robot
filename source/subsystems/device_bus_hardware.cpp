// device_bus_hardware.cpp -- see device_bus_hardware.h for the full design
// (100-DBX, the COMPLETE CUTOVER).
//
// #ifndef HOST_BUILD split (DeviceBusHardware's constructor, mirroring
// Devices::DeviceBus's own device_bus.h split exactly): Devices::DeviceBus's
// real-build constructor takes a raw MicroBitI2C& (no project I2CBus
// wrapper); its HOST_BUILD constructor takes no bus argument at all (it
// constructs its own internal Devices::I2CBus against the HOST_BUILD
// scripted fake). DeviceBusHardware forwards whichever shape this build
// defines.
#include "subsystems/device_bus_hardware.h"

namespace Subsystems {

// ---------------------------------------------------------------------------
// Conversion helpers -- see device_bus_hardware.h for each function's own
// doc comment.
// ---------------------------------------------------------------------------

msg::MotorConfig deviceBusMotorConfigToMsg(const Devices::MotorConfig& cfg) {
    msg::MotorConfig out;
    out.travel_calib = cfg.wheelTravelCalib;
    out.fwd_sign = cfg.fwdSign;
    out.vel_gains.kp = cfg.velGains.kp;
    out.vel_gains.ki = cfg.velGains.ki;
    out.vel_gains.kff = cfg.velGains.kff;
    out.vel_gains.i_max = cfg.velGains.iMax;
    out.vel_gains.kaw = cfg.velGains.kaw;
    out.vel_filt_alpha = cfg.velFiltAlpha;
    // min_duty <-> velDeadband: the SAME quantity under two different names
    // -- see device_config.h's own MotorConfig::velDeadband doc comment
    // ("minDuty plays minWheelSpeed's role here ... despite its proto name").
    out.min_duty = cfg.velDeadband;
    out.slew_rate = cfg.slewRate;
    out.port = cfg.port;
    if (cfg.reversalDwell.has) {
        out.reversal_dwell.has = true;
        out.reversal_dwell.val = cfg.reversalDwell.val;
    }
    if (cfg.outputDeadband.has) {
        out.output_deadband.has = true;
        out.output_deadband.val = cfg.outputDeadband.val;
    }
    out.polled = cfg.polled;
    return out;
}

Devices::MotorConfig msgToDeviceBusMotorConfig(const msg::MotorConfig& cfg) {
    Devices::MotorConfig out;
    out.wheelTravelCalib = cfg.travel_calib;
    out.fwdSign = cfg.fwd_sign;
    out.velGains.kp = cfg.vel_gains.kp;
    out.velGains.ki = cfg.vel_gains.ki;
    out.velGains.kff = cfg.vel_gains.kff;
    out.velGains.iMax = cfg.vel_gains.i_max;
    out.velGains.kaw = cfg.vel_gains.kaw;
    out.velFiltAlpha = cfg.vel_filt_alpha;
    out.velDeadband = cfg.min_duty;
    out.slewRate = cfg.slew_rate;
    out.port = cfg.port;
    if (cfg.reversal_dwell.has) {
        out.reversalDwell.has = true;
        out.reversalDwell.val = cfg.reversal_dwell.val;
    }
    if (cfg.output_deadband.has) {
        out.outputDeadband.has = true;
        out.outputDeadband.val = cfg.output_deadband.val;
    }
    out.polled = cfg.polled;
    return out;
}

Devices::OtosConfig otosBootConfigToDeviceBus(const Config::OtosBootConfig& cfg) {
    Devices::OtosConfig out;
    out.offsetX = cfg.offsetX;
    out.offsetY = cfg.offsetY;
    out.offsetYaw = cfg.offsetYaw;
    out.linearScale = cfg.linearScale;
    out.angularScale = cfg.angularScale;
    return out;
}

msg::PoseEstimate deviceBusPoseToEstimate(const Devices::Sample<Devices::PoseReading>& sample) {
    msg::PoseEstimate out;
    out.pose.x = sample.value.x;
    out.pose.y = sample.value.y;
    out.pose.h = sample.value.heading;
    out.twist.v_x = sample.value.v_x;
    out.twist.v_y = sample.value.v_y;
    out.twist.omega = sample.value.omega;
    out.stamp.valid = sample.valid;
    out.stamp.last_upd = static_cast<uint32_t>(sample.stamp / 1000);   // [us] -> [ms]
    out.stamp.lag = 0;
    return out;
}

Devices::Neutral msgNeutralToDeviceBus(msg::Neutral mode) {
    return (mode == msg::Neutral::BRAKE) ? Devices::Neutral::Brake : Devices::Neutral::Coast;
}

// ---------------------------------------------------------------------------
// DeviceBusMotor
// ---------------------------------------------------------------------------

void DeviceBusMotor::setDutyCycle(float dutyCycle)
{
    // DUTY_CYCLE mode must explicitly disable the DeviceBus-side embedded
    // PID or it keeps overwriting this staged raw duty every fiber cycle --
    // see this file's header, "PID on/off routing".
    handle_.setPidEnabled(false);
    handle_.setDuty(dutyCycle);
}

void DeviceBusMotor::setVoltage(float voltage)
{
    // Unsupported (capabilities().voltage == false); apply() gates this
    // before it is ever called -- a documented no-op, not an assert, so a
    // direct (non-apply()) call from test code cannot crash the firmware
    // (mirrors Hal::NezhaMotor::setVoltage()'s own precedent).
    (void)voltage;
}

void DeviceBusMotor::setVelocity(float velocity)
{
    // Re-arm PID -- a motor previously left in DUTY_CYCLE mode (PID
    // disabled) that is now commanded VELOCITY must resume PID control.
    handle_.setPidEnabled(true);
    handle_.setVelocity(velocity);
}

void DeviceBusMotor::setPosition(float position)
{
    // Unsupported (capabilities().position == false) -- the public
    // Devices::Motor handle has no position-move primitive at all (the
    // internal Devices::NezhaMotor leaf's own file header: "POSITION mode
    // ... is NOT ported").
    (void)position;
}

void DeviceBusMotor::setNeutral(msg::Neutral mode)
{
    handle_.setNeutral(msgNeutralToDeviceBus(mode));
}

void DeviceBusMotor::setFeedforward(float feedforward)
{
    // Unsupported -- the internal Devices::NezhaMotor leaf carries no
    // feedforward term (its own file header: "setFeedforward()/feedforward_
    // is NOT ported").
    (void)feedforward;
}

float DeviceBusMotor::position() const { return handle_.latest().value.position; }
float DeviceBusMotor::velocity() const { return handle_.latest().value.velocity; }
float DeviceBusMotor::appliedDuty() const { return handle_.latest().value.appliedDuty; }
bool DeviceBusMotor::connected() const { return handle_.connected(); }
uint32_t DeviceBusMotor::encGlitchCount() const { return handle_.encGlitchCount(); }

uint32_t DeviceBusMotor::sampleTime() const
{
    return static_cast<uint32_t>(handle_.updatedAt() / 1000);   // [us] -> [ms]
}

void DeviceBusMotor::tick(uint32_t now)
{
    // ONE narrow, deliberate exception to full no-op -- see this file's
    // header, "Known limitations" #2. Drains a RESET (DEV M <n> RESET)
    // request staged via the base's own resetPosition(); runs NO PID and NO
    // armor of its own (no armoredWrite()/updateWedgeDetector()/
    // updateRestTracking()/trackAcceleration() calls) -- the DeviceBus fiber
    // remains the ONLY thing that ever drives a duty write or a wedge
    // verdict for this motor.
    processResetIfPending(now);
}

msg::MotorCapabilities DeviceBusMotor::capabilities() const
{
    msg::MotorCapabilities caps;
    caps.duty_cycle = true;
    caps.voltage = false;
    caps.velocity = true;
    caps.position = false;   // handle has no onboard position-move primitive
    caps.has_encoder = true;
    return caps;
}

void DeviceBusMotor::writeRawDuty(float duty)
{
    // Structurally unreachable in normal operation: armoredWrite() (the
    // base's own caller of this hook) is invoked only from a leaf's own
    // tick(), and DeviceBusMotor::tick() never calls it (see above).
    // Forwards defensively anyway, matching this class's "pure passthrough"
    // spirit, should some future caller reach this protected hook directly.
    handle_.setPidEnabled(false);
    handle_.setDuty(duty);
}

void DeviceBusMotor::hardReset()
{
    // Structurally unreachable (restTicks_ never advances -- see this
    // file's header, "Known limitations" #2); implemented identically to
    // softRebaseline() below for defensive completeness.
    handle_.resetPosition();
}

void DeviceBusMotor::softRebaseline()
{
    handle_.resetPosition();
    // Base-owned counter (protected, Hal::Motor) -- mirrors
    // Hal::NezhaMotor::softRebaseline()'s own bookkeeping so
    // Motor::softResetCount() reports a real, live count.
    ++softResetCount_;
}

void DeviceBusMotor::configureDevice(const msg::MotorConfig& config)
{
    // No-op -- see this file's header, "Known limitations" #3. The public
    // Devices::Motor handle exposes no live-reconfigure primitive; real
    // calibration is baked once into DeviceBus's own Devices::MotorConfig
    // at DeviceBusHardware construction (boot config).
    (void)config;
}

// ---------------------------------------------------------------------------
// DeviceBusOdometer
// ---------------------------------------------------------------------------

msg::PoseEstimate DeviceBusOdometer::pose() const
{
    return deviceBusPoseToEstimate(handle_.latest());
}

bool DeviceBusOdometer::connected() const { return handle_.connected(); }

void DeviceBusOdometer::tick(uint32_t now)
{
    (void)now;   // NO-OP -- the fiber owns all I/O, asynchronously
}

void DeviceBusOdometer::init()
{
    // No-op -- see this file's header, "Known limitations" #4. The public
    // Devices::Odometer handle exposes no init()/OI primitive.
}

void DeviceBusOdometer::resetTracking()
{
    // No-op -- see this file's header, "Known limitations" #4. The public
    // Devices::Odometer handle exposes no resetTracking()/OR primitive.
}

void DeviceBusOdometer::setPose(const msg::Pose2D& pose)
{
    handle_.setPose(pose.x, pose.y, pose.h);
}

void DeviceBusOdometer::setLinearScalar(float scalar)
{
    // No-op -- see this file's header, "Known limitations" #4. The public
    // Devices::Odometer handle exposes no setLinearScalar()/OL primitive
    // (boot-time linear scale IS still applied once, via this class's own
    // Devices::OtosConfig conversion at DeviceBusHardware construction).
    (void)scalar;
}

void DeviceBusOdometer::setAngularScalar(float scalar)
{
    // No-op -- mirrors setLinearScalar() above; see "Known limitations" #4.
    (void)scalar;
}

bool DeviceBusOdometer::fusableThisPass()
{
    // One-shot, read-and-clear, freshness-derived -- see this file's header
    // for the full rationale (mirrors Hal::OtosOdometer::fusableThisPass()'s
    // CONTRACT via a different mechanism, since this leaf's tick() cannot
    // rate-limit the way OtosOdometer's own tick() does).
    uint64_t stamp = handle_.updatedAt();
    if (stamp == lastFusedStamp_) return false;
    lastFusedStamp_ = stamp;
    return true;
}

// ---------------------------------------------------------------------------
// DeviceBusHardware
// ---------------------------------------------------------------------------

#ifndef HOST_BUILD
DeviceBusHardware::DeviceBusHardware(MicroBitI2C& i2c, const msg::MotorConfig configs[kMotorCount],
                                      const Config::OtosBootConfig& otosConfig)
    : deviceBus_(i2c, msgToDeviceBusMotorConfig(configs[0]), msgToDeviceBusMotorConfig(configs[1]),
                 otosBootConfigToDeviceBus(otosConfig), Devices::ColorConfig{}, Devices::LineConfig{}),
      motors_{{ DeviceBusMotor(deviceBus_.motor(1)), DeviceBusMotor(deviceBus_.motor(2)),
                DeviceBusMotor(deviceBus_.motor(3)), DeviceBusMotor(deviceBus_.motor(4)) }},
      odometer_(deviceBus_.odometer())
{
    for (uint32_t i = 0; i < kMotorCount; ++i) {
        motorConfigs_[i] = configs[i];
    }
}
#else
DeviceBusHardware::DeviceBusHardware(const msg::MotorConfig configs[kMotorCount],
                                      const Config::OtosBootConfig& otosConfig)
    : deviceBus_(msgToDeviceBusMotorConfig(configs[0]), msgToDeviceBusMotorConfig(configs[1]),
                 otosBootConfigToDeviceBus(otosConfig), Devices::ColorConfig{}, Devices::LineConfig{}),
      motors_{{ DeviceBusMotor(deviceBus_.motor(1)), DeviceBusMotor(deviceBus_.motor(2)),
                DeviceBusMotor(deviceBus_.motor(3)), DeviceBusMotor(deviceBus_.motor(4)) }},
      odometer_(deviceBus_.odometer())
{
    for (uint32_t i = 0; i < kMotorCount; ++i) {
        motorConfigs_[i] = configs[i];
    }
}
#endif

void DeviceBusHardware::begin()
{
    deviceBus_.start();
}

void DeviceBusHardware::tick(uint32_t now)
{
    (void)now;   // NO-OP -- the fiber owns all bus scheduling asynchronously
}

Hal::Motor& DeviceBusHardware::motor(uint32_t i)
{
    return motors_[clampIndex(i)];
}

void DeviceBusHardware::apply(const Hal::CommandProcessorToHardwareCommand& cmd)
{
    if (cmd.allPorts) {
        for (uint32_t i = 0; i < kMotorCount; ++i) {
            motors_[i].apply(cmd.addressed[0].command);
        }
        return;
    }
    for (uint8_t i = 0; i < cmd.count; ++i) {
        motors_[clampIndex(cmd.addressed[i].port)].apply(cmd.addressed[i].command);
    }
}

void DeviceBusHardware::apply(const Hal::DrivetrainToHardwareCommand& cmd)
{
    for (int i = 0; i < 2; ++i) {
        motors_[clampIndex(cmd.wheel[i].port)].apply(cmd.wheel[i].command);
    }
}

Hal::Odometer* DeviceBusHardware::odometer()
{
    return &odometer_;
}

msg::MotorConfig DeviceBusHardware::motorConfig(uint32_t i) const
{
    return motorConfigs_[clampIndex(i)];
}

msg::MotorState DeviceBusHardware::motorState(uint32_t i) const
{
    return motors_[clampIndex(i)].state();
}

}  // namespace Subsystems
