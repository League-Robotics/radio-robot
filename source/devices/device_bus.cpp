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

  serviceMotor(motor1_);  // alternating ports: one request->settle->collect pair
  serviceMotor(motor2_);  // EACH -- the brick holds only ONE pending 0x46 request
                          // (DB-009 HITL); pipelining both starved motor 2.

  const uint64_t nowUs = clock_.nowMicros();  // [us] one "now" for this cycle's perceive/publish
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
// serviceMotor() -- one motor's uncontested request -> settle -> collect+drive
// pair. The Nezha brick holds only ONE pending 0x46 encoder request at a time:
// DB-009's HITL bench proved the pipelined "request BOTH motors, then collect
// both" form (device_bus.h's original schedule) permanently starved motor 2 --
// its 0x46 request was dropped every cycle while motor 1's stayed pending, so
// motor 2's encoder never refreshed (frozen position, vel=0, false wedge
// latch) even though its wheel spun. So each motor gets its OWN settle window,
// serviced in alternation -- exactly the fallback device_bus.h's
// "pipelined-vs-alternating" note pre-specified.
//
// This also keeps the 093 REQUEST->COLLECT hazard structurally absent, and
// more tightly than the pipelined form did: NOTHING now touches the bus
// between a motor's own 0x46 request and its own collect (previously the other
// motor's request sat in that window). Each motor's tick() does the collect
// (read) then, if a target is staged, its own armored 0x60 duty write -- still
// that motor's LAST bus action before the next motor is serviced.
// ---------------------------------------------------------------------------

void DeviceBus::serviceMotor(NezhaMotor& motor) {
  motor.requestSample();                   // 0x46 encoder-select write
  sleeper_.sleepMillis(kEncoderSettleMs);  // [ms] vendor settle -- YIELDS, never spins
  motor.tick(clock_.nowMicros());          // [us] collect (read) + PID + armored duty write
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

// ---------------------------------------------------------------------------
// Fiber lifecycle (DB-008) -- start()/stop()/running(), the detection
// preamble, and the neutralize epilogue. See device_bus.h's own declaration
// comments for start()/stop()/runPreamble()/neutralizeAllMotors() for the
// full contract; this section is the implementation those comments describe.
// ---------------------------------------------------------------------------

void DeviceBus::start() {
  stopRequested_ = false;
  loopExited_ = false;
  running_ = true;
  fiberRunner_->run(*this);  // real: spawns async, returns immediately.
                              // host: runs the preamble + a bounded number
                              // of cycles synchronously, in place, THEN
                              // returns -- see fiber_runner.h.
}

void DeviceBus::stop() {
  stopRequested_ = true;  // (1) request exit -- the cycle loop (wherever it
                           //     is actually running) exits at its next
                           //     while-condition check.

  while (!loopExited_) {  // (2) join -- cooperative yield-poll. A no-op in
    sleeper_.yield();     //     host builds: the bounded loop already ran
  }                        //     to completion inside start(), so
                           //     loopExited_ is already true by the time
                           //     stop() is ever called.

  neutralizeAllMotors();  // (3) neutralize -- see this method's own header
                           //     comment (device_bus.h) for why this call,
                           //     not the fiber body's own tail, is what
                           //     guarantees "wheels never left driven."

  running_ = false;
}

// ---------------------------------------------------------------------------
// runPreamble() -- the detection preamble (issue "The fiber and its cycle"
// step 1). See device_bus.h's own declaration comment for the full contract.
//
// --- Why nowUs is advanced BY HAND, never re-read from clock_, between
// color_/line_ retry attempts ---
// color_.beginStep(nowUs)/line_.beginStep(nowUs) each gate their own next
// attempt on `nowUs - lastAttemptUs_ >= <their own retry period>` (color_
// sensor.cpp/line_sensor.cpp) -- a REAL elapsed-time check, not a call
// counter. On real hardware this "just works": sleeper_.sleepMillis()
// really blocks/yields for the requested duration, so a fresh
// clock_.nowMicros() read after it genuinely reflects that much elapsed
// time. HOST_BUILD's Sleeper is a pure bookkeeping no-op (clock.h/
// clock_host.cpp: "No wall-clock block") and HOST_BUILD's Clock only moves
// when a test calls setMicros()/advanceMicros() directly -- it does NOT
// self-advance the way I2CBus's OWN separate internal fake clock does
// during a live entry-spin (i2c_bus_host.cpp). So if this method re-read
// clock_.nowMicros() after each sleeper_.sleepMillis() call instead, nowUs
// would never actually change in a host test (nothing here calls
// clock_.advanceMicros()), every attempt after the first would perpetually
// read as "not due yet," and a scripted multi-attempt retry could never
// reach detectDone(). Advancing a LOCAL nowUs by the exact requested sleep
// duration instead needs no HOST_BUILD-specific branch at all: it is
// correct on real hardware (fiber_sleep() reliably sleeps AT LEAST that
// long) and it is what makes retry pacing deterministic and CODAL-free in
// a host test -- the identical "cache now once, advance deliberately"
// discipline runCycleOnce() itself already uses for its own nowUs.
// ---------------------------------------------------------------------------

void DeviceBus::runPreamble() {
  sleeper_.sleepMillis(kPowerSettleMs);  // power-settle wait

  // Motor/OTOS detection: one self-contained, already-bounded begin() call
  // each (NezhaMotor::begin()'s hardReset() has its own internal median-of-3
  // retry; Otos::begin() is a single product-ID probe with no retry at all)
  // -- see device_bus.h's own runPreamble() comment for why this method does
  // not loop either of them itself.
  motor1_.begin();
  motor2_.begin();

  // OTOS (SparkFun) needs ~1s after power-on before its product-ID register
  // (0x17 reg 0x00) reads 0x5F. Otos::begin() is a single probe with no retry,
  // so if it ran before the chip booted the OTOS was marked absent FOREVER --
  // the DeviceBus `connected=False` root cause found on the bench rig (101-001).
  // Retry the probe with pacing until it detects or attempts run out. begin()
  // early-returns cheaply on a non-match, so retries cost only the paced reads.
  for (int attempt = 0; attempt < kOtosBeginAttempts; ++attempt) {
    otos_.begin();
    if (otos_.connected()) break;
    sleeper_.sleepMillis(kOtosBeginRetryPacingMs);
  }

  // Color/line detection: a bounded, LOCAL retry-pacing loop over each
  // leaf's own non-blocking beginStep(nowUs) state machine -- see this
  // function's own header comment above for why nowUs is advanced by hand.
  // Absent devices are marked (present() false) and structurally skipped
  // from then on (device_bus.h's runPreamble() comment) -- this loop's only
  // job is to keep calling beginStep() until EVERY leaf's own detectDone()
  // is true (found, or retries exhausted), never to decide presence itself.
  uint64_t nowUs = clock_.nowMicros();
  for (int tick = 0;
       tick < kMaxPreambleTicks && !(color_.detectDone() && line_.detectDone());
       ++tick) {
    if (!color_.detectDone()) color_.beginStep(nowUs);
    if (!line_.detectDone()) line_.beginStep(nowUs);
    if (color_.detectDone() && line_.detectDone()) break;

    sleeper_.sleepMillis(kPreambleRetryPacingMs);
    nowUs += static_cast<uint64_t>(kPreambleRetryPacingMs) * 1000ULL;  // [us]
  }
}

// ---------------------------------------------------------------------------
// neutralizeAllMotors() -- stop()'s own epilogue. See device_bus.h's own
// declaration comment for the full contract (why this reuses requestEncoders()/
// collectAndDrive() rather than reaching into NezhaMotor's write path
// directly, and why the resulting write is unconditionally each motor's LAST
// bus action from this call).
// ---------------------------------------------------------------------------

void DeviceBus::neutralizeAllMotors() {
  motor1_.setNeutral(Neutral::Coast);
  motor2_.setNeutral(Neutral::Coast);

  // Alternating request->settle->collect+neutral-write per motor -- the SAME
  // serviceMotor() step runCycleOnce() uses (the brick holds only one pending
  // 0x46 request). Each motor's tick() collects (read) THEN dispatches its
  // staged Neutral::Coast -> armoredWrite(0, ...); that neutral write is
  // always the LAST bus action per motor here.
  serviceMotor(motor1_);
  serviceMotor(motor2_);
}

// ---------------------------------------------------------------------------
// FiberRunner implementations (fiber_runner.h) -- defined here, not in
// fiber_runner.h itself, because both need DeviceBus to be a COMPLETE type
// (to call runPreamble()/runCycleOnce()/stopRequested()/markLoopExited()),
// which is only true once this file's own #include "devices/device_bus.h"
// has pulled in the full class definition above.
// ---------------------------------------------------------------------------

#ifndef HOST_BUILD
// The trampoline create_fiber() actually invokes (real CODAL builds only).
// DB-009 verifies this against real hardware -- in particular, that
// create_fiber()'s signature (a bare `void(*)(void*)` entry point plus a
// void* context argument, the common CODAL-core convention this project's
// own clock_real.cpp precedent for fiber_sleep()/schedule() already relies
// on being available via a plain "MicroBit.h" include) matches what actually
// ships in this project's vendored CODAL; this is the ONE call site in the
// whole subsystem that touches create_fiber(), by design (fiber_runner.h's
// own header comment).
//
// A STATIC MEMBER of CodalFiberRunner (fiber_runner.h), not a free function
// in an anonymous namespace (DB-008's original form here) -- DB-009's first
// real (non-HOST_BUILD) ARM compile of this file caught that the free-
// function form cannot reach DeviceBus::runPreamble()/stopRequested()/
// markLoopExited() (all private): device_bus.h's `friend class
// CodalFiberRunner;` grants friendship to that CLASS, not to an unrelated
// free function. See fiber_runner.h's own declaration comment for the full
// reasoning.
void CodalFiberRunner::codalFiberEntry(void* arg) {
  DeviceBus* bus = static_cast<DeviceBus*>(arg);
  bus->runPreamble();
  while (!bus->stopRequested()) {
    bus->runCycleOnce();
  }
  bus->markLoopExited();
}

void CodalFiberRunner::run(DeviceBus& bus) {
  create_fiber(&codalFiberEntry, static_cast<void*>(&bus));
}
#endif

#ifdef HOST_BUILD
void HostFiberRunner::run(DeviceBus& bus) {
  bus.runPreamble();
  for (int i = 0; i < maxCycles_ && !bus.stopRequested(); ++i) {
    bus.runCycleOnce();
  }
  bus.markLoopExited();
}
#endif

}  // namespace Devices
