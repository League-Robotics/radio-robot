#include "devices/device_bus.h"

namespace Devices {

// ---------------------------------------------------------------------------
// Construction -- see device_bus.h's own comment on why declaration order
// (bus_, clock_, sleeper_, then every leaf, then every ring, then every
// handle) IS the construction order this initializer list relies on.
// ---------------------------------------------------------------------------

#ifndef HOST_BUILD
DeviceBus::DeviceBus(MicroBitI2C& i2c, const MotorConfig& motor1Config,
                      const MotorConfig& motor2Config,
                      const OtosConfig& otosConfig,
                      const ColorConfig& colorConfig,
                      const LineConfig& lineConfig)
    : bus_(i2c),
      motor1_(bus_, motor1Config),
      motor2_(bus_, motor2Config),
      otos_(bus_, otosConfig),
      color_(bus_, colorConfig),
      line_(bus_, lineConfig),
      motor1Handle_(motor1_, motor1Ring_, clock_),
      motor2Handle_(motor2_, motor2Ring_, clock_),
      colorHandle_(color_, colorRing_),
      lineHandle_(line_, lineRing_),
      odometerHandle_(otos_, otosRing_) {}
#else
DeviceBus::DeviceBus(const MotorConfig& motor1Config,
                      const MotorConfig& motor2Config,
                      const OtosConfig& otosConfig,
                      const ColorConfig& colorConfig,
                      const LineConfig& lineConfig)
    : motor1_(bus_, motor1Config),
      motor2_(bus_, motor2Config),
      otos_(bus_, otosConfig),
      color_(bus_, colorConfig),
      line_(bus_, lineConfig),
      motor1Handle_(motor1_, motor1Ring_, clock_),
      motor2Handle_(motor2_, motor2Ring_, clock_),
      colorHandle_(color_, colorRing_),
      lineHandle_(line_, lineRing_),
      odometerHandle_(otos_, otosRing_) {}
#endif

// ---------------------------------------------------------------------------
// The public surface.
// ---------------------------------------------------------------------------

Motor& DeviceBus::motor(uint8_t port) {
  if (port == 2) return motor2Handle_;
  return motor1Handle_;
}

#ifdef HOST_BUILD
NezhaMotor& DeviceBus::motorLeaf(uint8_t port) {
  if (port == 2) return motor2_;
  return motor1_;
}
#endif

// ---------------------------------------------------------------------------
// runCycleOnce() -- see device_bus.h's own header comment for the exact
// schedule and the structural REQUEST->COLLECT hazard-exclusion argument.
// ---------------------------------------------------------------------------

void DeviceBus::runCycleOnce() {
  drainStagedInputs();  // targets, watchdog gate -- no bus

  requestEncoders();                      // 0x46 write, motor1 then motor2
  sleeper_.sleepMillis(kEncoderSettleMs);  // [ms] vendor settle -- YIELDS, never spins

  const uint64_t nowUs = clock_.nowMicros();  // [us] one "now" for this whole cycle's collect/perceive/publish
  collectAndDrive(nowUs);                     // collect+PID+armored write, motor1 then motor2

  perceptionSlotStep(nowUs);  // round-robin ONE of: line | color | OTOS
  publishSamples(nowUs);      // ring publishes -- plain stores, no yield

  sleeper_.sleepMillis(kCyclePaceMs);  // [ms] pace the cycle
}

// ---------------------------------------------------------------------------
// drainStagedInputs() -- the stale-target/RX-watchdog neutralize gate. Motor
// handle setters (handles.h) already relay directly onto NezhaMotor's own
// staged fields (a passthrough, not a second staging layer -- see handles.h's
// Motor design note), so THIS step's only remaining job is the watchdog: a
// motor whose most recent Motor::setVelocity() call is older than
// kVelocityStaleUs gets forced back to Neutral::Coast, overriding whatever
// stale velocity target the (possibly crashed/disconnected) caller last
// staged. Odometer::setPose() likewise relays directly into Otos's own
// posePending_ cell (see handles.h's Odometer design note) -- drained by
// Otos::tick() itself at this cycle's perceptionSlotStep(), not here.
// ---------------------------------------------------------------------------

void DeviceBus::drainStagedInputs() {
  const uint64_t nowUs = clock_.nowMicros();
  applyStaleGate(motor1Handle_, motor1_, nowUs);
  applyStaleGate(motor2Handle_, motor2_, nowUs);
}

// applyStaleGate() re-asserts Neutral::Coast EVERY cycle for as long as the
// staleness condition holds (it does not clear Motor::velocityStaged_ itself
// -- only an explicit Motor::setNeutral() call does, see handles.h). This is
// the more robust choice over a one-shot latch: a repeated setNeutral(Coast)
// call is harmless (NezhaMotor::armoredWrite()'s duty==0 path is always
// immediate and idempotent -- motor_armor.h), and re-checking every cycle
// means the gate keeps holding the wheels safe for as long as the watchdog
// condition is true, with no separate "did I already latch this" state to
// get out of sync.
void DeviceBus::applyStaleGate(Motor& handle, NezhaMotor& leaf, uint64_t nowUs) {
  if (handle.velocityStaged_ &&
      (nowUs - handle.velocityStagedUs_) > kVelocityStaleUs) {
    leaf.setNeutral(Neutral::Coast);
  }
}

// ---------------------------------------------------------------------------
// requestEncoders() / collectAndDrive() -- split across the settle sleep;
// see device_bus.h's header comment for why NO duty write can ever land
// between one motor's own request and its own collect.
// ---------------------------------------------------------------------------

void DeviceBus::requestEncoders() {
  motor1_.requestSample();
  motor2_.requestSample();
}

void DeviceBus::collectAndDrive(uint64_t nowUs) {
  motor1_.tick(nowUs);
  motor2_.tick(nowUs);
}

// ---------------------------------------------------------------------------
// perceptionSlotStep() -- one leaf serviced per cycle, round robin. Each
// leaf's own tick() is independently rate-limited (readDue()) and gated on
// its own present()/initialized_ state (color_.h/line_.h/otos.h), so a call
// here is always safe even before any detection has ever run (DB-008) or on
// a leaf that never answered -- both are total no-ops.
// ---------------------------------------------------------------------------

void DeviceBus::perceptionSlotStep(uint64_t nowUs) {
  lastPerceptionSlot_ = perceptionSlot_;

  switch (perceptionSlot_) {
    case PerceptionSlot::Line:
      line_.tick(nowUs);
      break;
    case PerceptionSlot::Color:
      color_.tick(nowUs);
      break;
    case PerceptionSlot::Otos:
      otos_.tick(nowUs);
      break;
    case PerceptionSlot::kCount:
    default:
      break;
  }

  perceptionSlot_ = static_cast<PerceptionSlot>(
      (static_cast<uint8_t>(perceptionSlot_) + 1) %
      static_cast<uint8_t>(PerceptionSlot::kCount));
}

// ---------------------------------------------------------------------------
// publishSamples() -- the fiber's one ring-publish point (concurrency
// contract: single-writer rings, plain stores, no yield). Motor rings are
// refreshed every cycle unconditionally (NezhaMotor::tick()'s collectEncoder()
// step always issues its read, with no internal rate limit, unlike the
// perception leaves) so they always publish. The perception rings publish
// ONLY the ONE leaf perceptionSlotStep() just serviced THIS cycle (via
// lastPerceptionSlot_) -- checking every leaf's own *Fresh() flag
// unconditionally every cycle would be wrong: the two leaves NOT serviced
// this cycle still carry whatever Fresh state their last (several cycles
// ago) tick() call left behind, and republishing that unchanged sample under
// a brand-new timestamp would fabricate a reading that was never actually
// refreshed this cycle.
// ---------------------------------------------------------------------------

void DeviceBus::publishSamples(uint64_t nowUs) {
  motor1Ring_.publish(
      MotorReading{motor1_.position(), motor1_.velocity(), motor1_.appliedDuty()},
      nowUs);
  motor2Ring_.publish(
      MotorReading{motor2_.position(), motor2_.velocity(), motor2_.appliedDuty()},
      nowUs);

  switch (lastPerceptionSlot_) {
    case PerceptionSlot::Line:
      if (line_.readingFresh()) lineRing_.publish(line_.reading(), nowUs);
      break;
    case PerceptionSlot::Color:
      if (color_.readingFresh()) colorRing_.publish(color_.reading(), nowUs);
      break;
    case PerceptionSlot::Otos:
      if (otos_.poseFresh()) otosRing_.publish(otos_.pose(), nowUs);
      break;
    case PerceptionSlot::kCount:
    default:
      break;
  }
}

}  // namespace Devices
