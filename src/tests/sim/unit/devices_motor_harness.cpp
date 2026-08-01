// devices_motor_harness.cpp — off-hardware acceptance harness (originally
// ticket DB-004; restructured 2026-07-18 with the Motor-interface split):
// exercises Devices::NezhaMotor's OWN write shaping (reversal dwell +
// output deadband, folded into the leaf's writeShapedDuty()), its
// request/collect encoder pairing, and duty passthrough through the real
// leaf against a TestSim::SimPlant (108-002), scripted deterministically
// via TestSim::ScriptedI2CHook (108-009) — AND the Devices::MotorArmor
// DECORATOR's observation/recovery policy (standstill-guarded resets,
// motion-qualified wedge reporting) through a dependency-free MockMotor
// inner double.
//
// 125-003 (sprint 125 Decision 2, "protection vs. control" — see
// sprint.md): SHRUNK alongside NezhaMotor's own shrink. Every scenario that
// exercised the embedded MotorVelocityPid, the freshness gate, source-side
// glitch rejection, or the EMA/least-squares velocity-estimator pair is
// DELETED, not adapted — those mechanisms are gone from NezhaMotor
// (relocated to a motion-local wheel-velocity PID class -- itself deleted
// outright by 128-015, see src/motion/DESIGN.md's "wheel control
// generations" note -- / pending ticket 004's App::WheelObserver, see
// nezha_motor.cpp's own file header). What survives: protocol/bus-hygiene/
// dwell/deadband/clamp scenarios (unchanged
// behavior) plus a NEW duty-passthrough scenario (scenario 5 below)
// replacing the deleted PID-chase coverage.
//
// Migrated by sprint 108 ticket 009
// (clasi/sprints/108-pure-i2cbus-clock-interfaces-and-a-real-simplant-
// simulator-sim-mode-tours/tickets/009-migrate-the-13-register-level-unit-
// tests-to-python-simplant-hook-tests-delete-c-harnesses.md) off the deleted
// src/firm/devices/i2c_bus_host.cpp scripted-FIFO Devices::I2CBus fake (ticket
// 001 reduced Devices::I2CBus to a pure interface and removed it).
//
// Modeled on src/tests/sim/unit/motor_policy_harness.cpp (the MockMotor-style
// armor scenarios), per device-bus-tickets.md's DB-004 acceptance criteria.
// Unlike those pre-port harnesses, this one #includes ONLY
// src/firm/devices/ headers (isolation invariant) plus plain C/C++ stdlib
// -- no messages/*.h, no com/i2c_bus.h.
//
// Plain C++ program, hand-rolled assertions -- prints a PASS/FAIL line per
// scenario and exits nonzero if any assertion failed. Run by the pytest
// wrapper in test_devices_motor.py, which compiles this file together with
// tests/_infra/sim/sim_plant.cpp, src/tests/sim/plant/{wheel,otos}_plant.cpp,
// and src/firm/devices/nezha_motor.cpp under -DHOST_BUILD, then runs the
// resulting binary via subprocess and asserts exit code 0.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include "devices/device_config.h"
#include "devices/device_types.h"
#include "devices/motor_armor.h"
#include "devices/nezha_motor.h"
#include "scripted_i2c_hook.h"
#include "sim_plant.h"

namespace {

// --- Hand-rolled assertion plumbing (mirrors motor_policy_harness.cpp) ---

int g_failureCount = 0;
std::string g_scenarioName;

void beginScenario(const std::string& name) {
  g_scenarioName = name;
  std::printf("--- %s\n", name.c_str());
}

void fail(const std::string& what) {
  ++g_failureCount;
  std::printf("  FAIL [%s]: %s\n", g_scenarioName.c_str(), what.c_str());
}

void checkTrue(bool condition, const std::string& what) {
  if (!condition) fail(what + " — expected true, got false");
}

void checkFloatEq(float actual, float expected, const std::string& what,
                   float tol = 1e-3f) {
  if (std::fabs(actual - expected) > tol) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s — expected %g, got %g", what.c_str(),
                  static_cast<double>(expected), static_cast<double>(actual));
    fail(buf);
  }
}

void checkUintEq(uint32_t actual, uint32_t expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s — expected %u, got %u", what.c_str(),
                  static_cast<unsigned>(expected),
                  static_cast<unsigned>(actual));
    fail(buf);
  }
}

// --- MockMotor ---------------------------------------------------------
//
// A dependency-free Devices::Motor double for the MotorArmor DECORATOR
// scenarios (2026-07-18 restructure: MotorArmor composes a Motor& instead
// of being the leaf's base class). Test-settable position/velocity/
// appliedDuty; counts the reset verbs the armor's standstill guard
// dispatches. No I2C, no CODAL — devices/motor.h + device_config.h only.
class MockMotor : public Devices::Motor {
 public:
  // --- Motor faceplate (trivial forwarding/recording) ---
  void begin() override {}
  void requestSample() override {}
  void setDuty(float duty) override { lastDutyCmd = duty; }
  void setNeutral(Devices::Neutral) override {}
  void applyTravelCalib(float) override {}
  // REVISION 1 (114-001, motor.h): trivial always-succeeds stand-in --
  // MockMotor has no boot-identity config of its own to actually reassign,
  // it only needs to satisfy the new pure virtual and let scenarios assert
  // it was called.
  bool reconfigure(const Devices::MotorConfig&) override {
    ++reconfigureCalls;
    return true;
  }
  void tick(uint64_t) override { ++tickCalls; }
  float position() const override { return mockPosition_; }
  float velocity() const override { return mockVelocity_; }
  float appliedDuty() const override { return mockAppliedDuty_; }
  bool connected() const override { return true; }
  uint64_t sampleTime() const override { return mockSampleTimeUs_; }  // [us]
  void resetPosition() override { ++resetPositionCalls; }   // bare = hard, immediate
  void rebaseline() override { ++rebaselineCalls; }

  // --- Test-driving surface ---
  void setMockPosition(float position) { mockPosition_ = position; }
  void setMockVelocity(float velocity) { mockVelocity_ = velocity; }
  void setMockAppliedDuty(float duty) { mockAppliedDuty_ = duty; }
  void setMockSampleTime(uint64_t sampleTimeUs) { mockSampleTimeUs_ = sampleTimeUs; }  // [us]

  // --- Call recording (scenarios assert on these directly) ---
  int resetPositionCalls = 0;
  int rebaselineCalls = 0;
  int tickCalls = 0;
  int reconfigureCalls = 0;
  float lastDutyCmd = 0.0f;

 private:
  float mockPosition_ = 0.0f;
  float mockVelocity_ = 0.0f;
  float mockAppliedDuty_ = 0.0f;
  uint64_t mockSampleTimeUs_ = 0;  // [us]
};

// Config for the MotorArmor decorator scenarios below (MockMotor inner --
// only outputDeadband is functionally relevant here, MotorArmor::
// reconfigure() reads it straight into its own motionThreshold_ motion-gate
// cache; MockMotor ignores config entirely, so reversalDwell is moot).
Devices::MotorConfig defaultArmorConfig() {
  Devices::MotorConfig cfg;
  cfg.outputDeadband = 0.03f;   // [-1,1] fraction
  return cfg;
}

// --- Write-shaping scenarios (real NezhaMotor — the dwell/deadband gate
// lives in the leaf's own writeShapedDuty()) ------------------------------

// Forward declarations — defined with the NezhaMotor scenario helpers below.
void scriptEncoderRequestCollect(TestSim::ScriptedI2CHook& bus, uint16_t wireAddr,
                                  float positionMm);
Devices::MotorConfig baseNezhaConfig();

// Scripts one encoder request/collect cycle and drives one setDuty+tick
// pass; returns nothing — callers assert on appliedDuty() transitions
// (write-on-change/throttle make exact write-call sequences a raw-path
// concern; appliedDuty() is the shaped outcome).
void dutyTick(Devices::NezhaMotor& motor, TestSim::ScriptedI2CHook& bus,
              uint16_t wireAddr, float duty, uint64_t nowUs) {
  scriptEncoderRequestCollect(bus, wireAddr, 0.0f);   // stationary plant
  motor.setDuty(duty);
  motor.requestSample();
  motor.tick(nowUs);
}

// 1. A commanded sign change writes 0 immediately, suppresses further
//    non-zero writes until reversalDwell has elapsed, then forwards the new
//    direction. Also proves a commanded stop (duty == 0) is immediate and
//    cancels an in-progress dwell. Times start at 50ms (not 0): the write
//    throttle compares against lastWriteTimeUs_'s zero-init, and every
//    non-stop step below leaves >=40ms since the last landed write.
void scenarioReversalDwellWritesZeroThenHoldsThroughDeadline() {
  beginScenario("reversal dwell writes 0 then holds through the deadline");
  {
    TestSim::SimPlant plant;
    TestSim::ScriptedI2CHook bus(plant);
    const uint16_t wireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);
    Devices::MotorConfig cfg = baseNezhaConfig();   // dwell/deadband = 100ms/0.03 (baseNezhaConfig()'s own explicit set)
    cfg.slewRate = 100.0f;                          // no slew clamping — isolates the dwell
    Devices::NezhaMotor m(plant, cfg);

    dutyTick(m, bus, wireAddr, 0.5f, 50000);     // no prior direction — forwarded
    checkFloatEq(m.appliedDuty(), 0.5f, "initial direction forwarded immediately");
    dutyTick(m, bus, wireAddr, -0.5f, 100000);   // sign flip — write 0, arm 100ms dwell
    checkFloatEq(m.appliedDuty(), 0.0f, "sign flip wrote 0 immediately (dwell armed)");
    dutyTick(m, bus, wireAddr, -0.5f, 140000);   // inside the dwell (140 < 200ms)
    checkFloatEq(m.appliedDuty(), 0.0f, "held at 0 through the dwell window");
    dutyTick(m, bus, wireAddr, -0.5f, 199000);   // still inside (199 < 200ms)
    checkFloatEq(m.appliedDuty(), 0.0f, "still held at 0 just before the deadline");
    dutyTick(m, bus, wireAddr, -0.5f, 240000);   // dwell elapsed — forwarded
    checkFloatEq(m.appliedDuty(), -0.5f, "new direction forwarded once the dwell elapsed");
  }

  // A commanded stop mid-dwell is immediate and cancels the dwell.
  {
    TestSim::SimPlant plant;
    TestSim::ScriptedI2CHook bus(plant);
    const uint16_t wireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);
    Devices::MotorConfig cfg = baseNezhaConfig();
    cfg.slewRate = 100.0f;
    Devices::NezhaMotor m(plant, cfg);

    dutyTick(m, bus, wireAddr, 0.5f, 50000);     // establish a direction
    dutyTick(m, bus, wireAddr, -0.5f, 100000);   // sign flip — arms dwell, deadline 200ms
    dutyTick(m, bus, wireAddr, 0.0f, 110000);    // explicit stop mid-dwell — immediate, cancels
    checkFloatEq(m.appliedDuty(), 0.0f, "stop wrote 0 immediately");
    // Before the ORIGINAL 200ms deadline (and >=40ms past the last landed
    // write, for the throttle): forwarded immediately — dwell was cancelled.
    dutyTick(m, bus, wireAddr, -0.5f, 150000);
    checkFloatEq(m.appliedDuty(), -0.5f,
                 "post-stop command forwarded immediately, dwell was cancelled");
  }
}

// 2. writeShapedDuty()'s two-case deadband boost (sprint 114 ticket 005,
//    deadband-compensation-small-commands-must-produce-real-motion.md):
//    exact duty==0.0f stays an immediate hard zero; a genuine nonzero
//    sub-deadband duty is boosted (sign-preserving) to outputDeadband_
//    instead of being zeroed; a duty already at/above the deadband passes
//    through unmodified (never floored DOWN); and a boosted duty that also
//    represents a sign reversal (relative to lastRequestedDuty_) still
//    arms/holds/releases through the SAME reversal-dwell mechanism as any
//    other nonzero-duty reversal would.
void scenarioOutputDeadbandBoostsSubDeadbandNonzeroDutyExactZeroStaysZero() {
  beginScenario("output deadband boosts sub-deadband nonzero duty; exact zero stays zero");

  // (a) Exact zero, as the very first command: immediate hard zero.
  {
    TestSim::SimPlant plant;
    TestSim::ScriptedI2CHook bus(plant);
    const uint16_t wireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);
    Devices::MotorConfig cfg = baseNezhaConfig();
    cfg.slewRate = 100.0f;
    Devices::NezhaMotor m(plant, cfg);
    dutyTick(m, bus, wireAddr, 0.0f, 50000);
    checkFloatEq(m.appliedDuty(), 0.0f, "exact duty==0.0f writes 0 immediately");
  }

  // (b) Sub-deadband nonzero, positive: boosted to +outputDeadband_ (0.03),
  //     not zeroed.
  {
    TestSim::SimPlant plant;
    TestSim::ScriptedI2CHook bus(plant);
    const uint16_t wireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);
    Devices::MotorConfig cfg = baseNezhaConfig();   // outputDeadband = 0.03
    cfg.slewRate = 100.0f;
    Devices::NezhaMotor m(plant, cfg);
    dutyTick(m, bus, wireAddr, 0.01f, 50000);   // 0 < 0.01 < 0.03 -- genuine nonzero, sub-deadband
    checkFloatEq(m.appliedDuty(), 0.03f,
                 "sub-deadband positive duty boosted to +outputDeadband_, not zeroed");
  }

  // (c) Sub-deadband nonzero, negative: boosted to -outputDeadband_
  //     (sign-preserving).
  {
    TestSim::SimPlant plant;
    TestSim::ScriptedI2CHook bus(plant);
    const uint16_t wireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);
    Devices::MotorConfig cfg = baseNezhaConfig();
    cfg.slewRate = 100.0f;
    Devices::NezhaMotor m(plant, cfg);
    dutyTick(m, bus, wireAddr, -0.02f, 50000);   // 0 < 0.02 < 0.03
    checkFloatEq(m.appliedDuty(), -0.03f,
                 "sub-deadband negative duty boosted to -outputDeadband_ (sign preserved)");
  }

  // (d) At/above deadband: unaffected, passes through unmodified (never
  //     floored down to outputDeadband_, and the boundary value itself
  //     takes the passthrough branch, not the boost branch, since
  //     `fabsf(duty) < outputDeadband_` is false when they are equal).
  {
    TestSim::SimPlant plant;
    TestSim::ScriptedI2CHook bus(plant);
    const uint16_t wireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);
    Devices::MotorConfig cfg = baseNezhaConfig();
    cfg.slewRate = 100.0f;
    Devices::NezhaMotor m(plant, cfg);
    dutyTick(m, bus, wireAddr, 0.5f, 50000);
    checkFloatEq(m.appliedDuty(), 0.5f, "above-deadband duty passes through unmodified");
    dutyTick(m, bus, wireAddr, 0.03f, 110000);   // exactly AT the deadband boundary
    checkFloatEq(m.appliedDuty(), 0.03f,
                 "duty exactly at outputDeadband_ passes through (not re-boosted/altered)");
  }

  // (e) Reversal-dwell interaction: a boosted duty that is ALSO a sign
  //     reversal (relative to lastRequestedDuty_) arms/holds/releases
  //     through the SAME dwell mechanism scenario 1 above already proves
  //     for an unboosted reversal.
  {
    TestSim::SimPlant plant;
    TestSim::ScriptedI2CHook bus(plant);
    const uint16_t wireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);
    Devices::MotorConfig cfg = baseNezhaConfig();   // reversalDwell = 100ms, outputDeadband = 0.03
    cfg.slewRate = 100.0f;
    Devices::NezhaMotor m(plant, cfg);

    dutyTick(m, bus, wireAddr, 0.5f, 50000);      // establish a direction
    checkFloatEq(m.appliedDuty(), 0.5f, "initial direction forwarded");

    dutyTick(m, bus, wireAddr, -0.01f, 100000);   // sub-deadband, OPPOSITE sign -- boosts to -0.03, which IS a reversal
    checkFloatEq(m.appliedDuty(), 0.0f,
                 "boosted-duty reversal writes 0 immediately (dwell armed), exactly like an unboosted reversal");

    dutyTick(m, bus, wireAddr, -0.01f, 150000);   // still inside the 100ms dwell (150 < 200ms deadline)
    checkFloatEq(m.appliedDuty(), 0.0f, "held at 0 through the dwell window");

    dutyTick(m, bus, wireAddr, -0.01f, 240000);   // dwell elapsed -- forwards the boosted duty
    checkFloatEq(m.appliedDuty(), -0.03f,
                 "dwell elapsed -- forwards the boosted new-direction duty (-outputDeadband_)");
  }
}

// --- MotorArmor decorator scenarios (via MockMotor) ---------------------

// 3. resetPosition() while moving (restTicks_ never accumulates to
//    kRestTicksRequired) dispatches inner.rebaseline(), never
//    inner.resetPosition(); at verified standstill it dispatches
//    inner.resetPosition() (hard). "Standstill-guarded reset gates on rest
//    ticks."
void scenarioStandstillGuardedResetGatesOnRestTicks() {
  beginScenario("standstill-guarded reset gates on rest ticks");

  // (a) Moving: velocity and applied duty both nonzero — restTicks_ stays 0.
  {
    MockMotor inner;
    Devices::MotorArmor armor(inner);
    (void)armor.reconfigure(defaultArmorConfig());
    inner.setMockVelocity(80.0f);      // well above kRestVelocity
    inner.setMockAppliedDuty(0.5f);    // being driven

    armor.tick(4000000);
    armor.tick(4020000);

    armor.resetPosition();             // stages; next tick dispatches
    armor.tick(4040000);

    checkUintEq(static_cast<uint32_t>(inner.rebaselineCalls), 1,
                "inner rebaseline() called exactly once while moving");
    checkUintEq(static_cast<uint32_t>(inner.resetPositionCalls), 0,
                "inner resetPosition() (hard) never called while moving");
    checkUintEq(armor.softResetCount(), 1, "armor softResetCount() reflects the call");
    checkUintEq(armor.hardResetCount(), 0, "armor hardResetCount() stays 0");
  }

  // (b) Verified standstill: well past kRestTicksRequired (5) ticks at rest.
  {
    MockMotor inner;
    Devices::MotorArmor armor(inner);
    (void)armor.reconfigure(defaultArmorConfig());
    inner.setMockVelocity(0.0f);       // below kRestVelocity throughout
    // appliedDuty stays 0 — never commanded to move.

    uint64_t now = 5000000;
    for (int i = 0; i < 8; ++i) {
      armor.tick(now);
      now += 20000;
    }

    armor.resetPosition();
    armor.tick(now);

    checkUintEq(static_cast<uint32_t>(inner.resetPositionCalls), 1,
                "inner resetPosition() (hard) called exactly once at verified standstill");
    checkUintEq(static_cast<uint32_t>(inner.rebaselineCalls), 0,
                "inner rebaseline() never called at verified standstill");
    checkUintEq(armor.hardResetCount(), 1, "armor hardResetCount() reflects the call");
    checkUintEq(armor.softResetCount(), 0, "armor softResetCount() stays 0");
  }
}

// 4. wedged() is the raw, unconditional stuck-encoder latch; wedgeSuspect()
//    is the same test additionally gated on |appliedDuty()| above the motion
//    threshold — an idle parked motor with a frozen position never reports
//    suspect.
void scenarioWedgeLatchAndSuspectDeriveAsBefore() {
  beginScenario("wedge latch + wedge-suspect derive as before");

  // (a) Idle parked motor: frozen position, zero applied duty throughout.
  {
    MockMotor inner;
    Devices::MotorArmor idle(inner);
    (void)idle.reconfigure(defaultArmorConfig());
    inner.setMockPosition(100.0f);   // never changes
    inner.setMockVelocity(0.0f);
    uint64_t now = 6000000;
    for (int i = 0; i < 15; ++i) {   // well past kWedgeThreshold (10)
      idle.tick(now);
      now += 20000;
    }
    checkTrue(idle.wedged(), "idle parked motor: wedged() latches (unconditional)");
    checkTrue(!idle.wedgeSuspect(),
              "idle parked motor: wedgeSuspect() stays false (never moving)");
  }

  // (b) Same frozen position, but driven above the motion threshold every
  //     tick — the motion-qualified counter runs alongside the
  //     unconditional one.
  {
    MockMotor inner;
    Devices::MotorArmor moving(inner);
    (void)moving.reconfigure(defaultArmorConfig());
    inner.setMockPosition(100.0f);   // still never changes — genuinely stuck
    inner.setMockVelocity(0.0f);
    inner.setMockAppliedDuty(0.5f);  // above the motion threshold every tick
    uint64_t now = 7000000;
    for (int i = 0; i < 15; ++i) {
      moving.tick(now);
      now += 20000;
    }
    checkTrue(moving.wedged(), "moving-but-stuck motor: wedged() latches");
    checkTrue(moving.wedgeSuspect(),
              "moving-but-stuck motor: wedgeSuspect() also latches");
  }
}

// --- Devices::NezhaMotor scenarios (real leaf, scripted Devices::I2CBus) ---

// Packs positionMm into the little-endian int32 tenths-of-degree raw
// encoder reading NezhaMotor::collectEncoder() decodes (mirrors
// motor_policy_harness.cpp's scriptNezhaEncoderReading() convention).
// wheelTravelCalib=1.0, fwdSign=+1 in every scenario below, so raw ==
// positionMm*10 exactly (no rounding drift).
//
// Scripts TWO writes, not one: requestEncoder() (phase 1) always issues one
// 0x46 register-select write, but tick()'s own dispatch (phase 2, same
// cycle) may ALSO land a duty write via writeMotorRun() (0x60) — the
// scripted I2CBus's writes/reads are independent FIFOs matched on address
// only (not on payload), so both the request write and a possible duty
// write draw from the SAME scriptedWrites_ queue, in call order. Scripting
// one slack entry per cycle is always sufficient and harmless when the duty
// write doesn't land that cycle (write-on-change/throttle skip).
void scriptEncoderRequestCollect(TestSim::ScriptedI2CHook& bus, uint16_t wireAddr,
                                  float positionMm) {
  bus.queueWrite(wireAddr, /*status=*/0);   // requestEncoder()'s 0x46 write
  bus.queueWrite(wireAddr, /*status=*/0);   // slack: a possible same-cycle duty write (0x60)

  int32_t raw = static_cast<int32_t>(std::lround(positionMm * 10.0f));
  uint8_t data[4] = {
      static_cast<uint8_t>(raw & 0xFF),
      static_cast<uint8_t>((raw >> 8) & 0xFF),
      static_cast<uint8_t>((raw >> 16) & 0xFF),
      static_cast<uint8_t>((raw >> 24) & 0xFF),
  };
  bus.queueRead(wireAddr, data, 4, /*status=*/0);   // collectEncoder()'s 4-byte read
}

Devices::MotorConfig baseNezhaConfig() {
  Devices::MotorConfig cfg;
  cfg.port = 1;
  cfg.fwdSign = 1;
  cfg.wheelTravelCalib = 1.0f;
  // Write-shaping — sprint 114 ticket 003: reversalDwell/outputDeadband are
  // now plain required floats (no more NezhaMotor ctor ship-default
  // substitution), so this harness sets the historical ship-default values
  // (100ms/0.03) explicitly, matching exactly what every scenario below got
  // implicitly before this ticket.
  cfg.reversalDwell = 100.0f;    // [ms]
  cfg.outputDeadband = 0.03f;    // [-1,1] fraction
  return cfg;
}

// 5. request->collect encoder pairing produces expected position()/
//    velocity(): requestSample() (phase 1, a scripted write) followed by
//    tick() (phase 2, collectEncoder() — a scripted read) yields the
//    expected position, and velocity() reflects the naive difference
//    quotient across two paired cycles (125-003: NezhaMotor's own velocity()
//    is now a plain per-tick diff -- no freshness gate, no EMA -- see
//    nezha_motor.cpp's own file header).
void scenarioRequestCollectPairingYieldsExpectedPositionVelocity() {
  beginScenario("request->collect encoder pairing yields expected position()/velocity()");
  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  const uint16_t wireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);

  Devices::NezhaMotor motor(plant, baseNezhaConfig());

  // Prime cycle: request -> collect at position 0.
  scriptEncoderRequestCollect(bus, wireAddr, 0.0f);
  motor.requestSample();
  motor.tick(0);   // nowUs=0

  checkFloatEq(motor.position(), 0.0f, "primed position is 0");

  // Second cycle, 20ms later: position steps to 10mm (500 mm/s, well within
  // plausibility).
  scriptEncoderRequestCollect(bus, wireAddr, 10.0f);
  motor.requestSample();
  motor.tick(20000);   // nowUs=20000 (20ms later)

  checkFloatEq(motor.position(), 10.0f, "position reflects the collected sample");
  checkFloatEq(motor.velocity(), 500.0f, "velocity == delta-position / delta-time");
  checkTrue(motor.connected(), "request+collect pairing reports connected");
  checkUintEq(bus.errCount(Devices::kNezhaDeviceAddr), 0, "no script under-run across the pairing");
}

// 5b. Ticket 125-001 (telemetry-emit-policy-rebuild-spec.md, Part 2):
//     "motor velocity must read 0 until it is real." velocity() is a
//     difference quotient -- with only one collected sample there is no
//     prior sample to diff against, so NezhaMotor::tick() must treat the
//     FIRST post-construction sample as a baseline-only anchor and leave
//     velocity() at 0.0f, never fabricating a rate from an uninitialized/
//     phantom pre-boot position. Deliberately scripts a NONZERO position
//     for that first sample (3.0mm) at a NONZERO nowUs (20ms): a naive
//     per-tick diff with no anchor concept would compute (3.0 - 0.0) /
//     0.020 == 150 mm/s against the phantom zero-initialized baseline --
//     exactly the bogus-nonzero-velocity-on-a-never-moved-robot defect
//     this ticket fixes at the source. The second sample then must yield
//     the real, ordinary difference quotient -- unchanged behavior from
//     today, just no longer computed off a meaningless first sample.
void scenarioVelocityReadsZeroUntilTwoValidSamplesCollected() {
  beginScenario("velocity() reads 0 until two valid samples are collected (125-001, Part 2)");
  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  const uint16_t wireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);

  Devices::NezhaMotor motor(plant, baseNezhaConfig());

  // First sample: connect + collect one reading. Nonzero position, nonzero
  // nowUs -- if this were still fabricating a rate from the pre-boot
  // baseline, velocity() would read 150.0f here, not 0.0f.
  scriptEncoderRequestCollect(bus, wireAddr, 3.0f);
  motor.requestSample();
  motor.tick(20000);   // nowUs=20000 (20ms)

  checkTrue(motor.connected(), "first sample: request+collect pairing reports connected");
  checkFloatEq(motor.position(), 3.0f, "first sample: position reflects the collected sample");
  checkFloatEq(motor.velocity(), 0.0f,
               "first sample: velocity() is 0 -- one sample is a baseline anchor only, "
               "never a fabricated rate against the pre-boot phantom baseline");

  // Second sample: a genuinely new reading -- velocity() now reflects the
  // real difference quotient between the two collected samples.
  scriptEncoderRequestCollect(bus, wireAddr, 8.0f);
  motor.requestSample();
  motor.tick(40000);   // nowUs=40000 (20ms later)

  checkFloatEq(motor.position(), 8.0f, "second sample: position reflects the new reading");
  checkFloatEq(motor.velocity(), 250.0f,
               "second sample: velocity() == (8.0 - 3.0)mm / 0.020s -- the real, unchanged "
               "difference-quotient behavior, now correctly gated on having two samples");

  checkUintEq(bus.errCount(Devices::kNezhaDeviceAddr), 0,
              "no script under-run across the two-sample-floor sequence");
}

// 6. Duty passthrough (NEW, 125-003 -- replaces the deleted PID-chase
//    coverage): setDuty()->tick() writes EXACTLY the given duty through the
//    dwell/deadband shaping -- NezhaMotor no longer has any velocity
//    decision of its own, so appliedDuty() must equal the staged duty
//    (once above the deadband and with no dwell in play) on the very next
//    landed write, regardless of what the plant reports.
void scenarioSetDutyTickWritesExactlyTheGivenDutyThroughShaping() {
  beginScenario("setDuty()->tick() writes exactly the given duty through dwell/deadband shaping");
  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  const uint16_t wireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);

  Devices::MotorConfig cfg = baseNezhaConfig();
  cfg.slewRate = 100.0f;   // no slew clamping -- isolates the passthrough

  Devices::NezhaMotor motor(plant, cfg);

  // A sequence of well-spaced (>=35ms apart, clears the write-rate
  // throttle), above-deadband, same-sign duties (no dwell in play) --
  // appliedDuty() must track each one exactly, on the very next tick.
  uint64_t nowUs = 50000;
  const float duties[] = {0.25f, 0.60f, 0.10f, 0.35f};
  for (float duty : duties) {
    scriptEncoderRequestCollect(bus, wireAddr, 0.0f);   // stationary plant -- irrelevant to duty passthrough
    motor.setDuty(duty);
    motor.requestSample();
    motor.tick(nowUs);
    checkFloatEq(motor.appliedDuty(), duty,
                 "appliedDuty() reflects the exact staged duty, no PID/velocity decision in the way");
    nowUs += 50000;
  }

  // A genuine reversal (opposite sign) still passes through EXACT duty
  // magnitudes once the dwell elapses -- proves the passthrough holds even
  // across the one write-shaping mechanism this leaf still owns.
  scriptEncoderRequestCollect(bus, wireAddr, 0.0f);
  motor.setDuty(-0.35f);
  motor.requestSample();
  motor.tick(nowUs);   // sign flip -- dwell writes 0 first
  checkFloatEq(motor.appliedDuty(), 0.0f, "sign flip dwells at 0 first (unchanged dwell behavior)");
  nowUs += 150000;      // past the 100ms dwell
  scriptEncoderRequestCollect(bus, wireAddr, 0.0f);
  motor.requestSample();
  motor.tick(nowUs);
  checkFloatEq(motor.appliedDuty(), -0.35f,
               "dwell elapsed -- exact staged duty forwarded, unchanged in magnitude");

  checkUintEq(bus.errCount(Devices::kNezhaDeviceAddr), 0, "no script under-run across the duty-passthrough sequence");
}

// 7. A NAK'd STOP write (pct==0) must NOT be latched as "already written" --
//    write-on-change (nezha_motor.cpp's writeRawDuty()) must retry the SAME
//    value next tick instead of permanently suppressing it, and
//    appliedDuty() must keep reporting the PREVIOUS (still physically
//    applied) duty until a write actually succeeds -- 103-002, 2026-07-13
//    code review C1 fix.
void scenarioNakedStopWriteIsRetriedNextTickNotLatched() {
  beginScenario("a NAK'd stop write is retried next tick, not permanently latched-as-written");
  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  const uint16_t wireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);

  Devices::MotorConfig cfg = baseNezhaConfig();
  cfg.slewRate = 100.0f;   // no slew clamping -- isolates the write-status behavior
  Devices::NezhaMotor motor(plant, cfg);

  // Starts at 50ms (not 0): writeRawDuty()'s write-rate throttle compares
  // this tick's nowUs against lastWriteTimeUs_'s zero-init value, so a
  // first tick at nowUs==0 would itself read as "0us since the last write"
  // and be throttled away.
  uint64_t nowUs = 50000;
  const float stationaryPosition = 0.0f;

  // Cycle 1: establish a nonzero applied duty via a successful write.
  motor.setDuty(0.5f);
  scriptEncoderRequestCollect(bus, wireAddr, stationaryPosition);
  motor.requestSample();
  motor.tick(nowUs);
  checkFloatEq(motor.appliedDuty(), 0.5f, "nonzero duty established by a successful write");

  // Cycle 2: command a stop, but script its 0x60 duty write to FAIL (NAK).
  // requestEncoder()'s own 0x46 write and collectEncoder()'s read both
  // succeed -- only the duty write NAKs.
  motor.setDuty(0.0f);
  nowUs += 50000;   // clears the 35ms write-rate throttle (stop is exempt anyway; 118 ticket 003)
  bus.queueWrite(wireAddr, /*status=*/0);    // requestEncoder()'s 0x46 write
  uint8_t data[4] = {0, 0, 0, 0};              // unchanged raw count -- stale sample, irrelevant here
  bus.queueRead(wireAddr, data, 4, /*status=*/0);   // collectEncoder()
  bus.queueWrite(wireAddr, /*status=*/-5);    // the stop's 0x60 write -- NAK'd
  motor.requestSample();
  motor.tick(nowUs);

  checkFloatEq(motor.appliedDuty(), 0.5f,
               "a NAK'd stop write does NOT latch -- appliedDuty() still reflects the "
               "PREVIOUS (still physically applied) duty, not the failed 0.0 attempt");
  checkUintEq(bus.errCount(Devices::kNezhaDeviceAddr), 1,
              "exactly one scripted error (the NAK'd stop write) recorded so far");

  // Cycle 3: the SAME stop target is retried -- write-on-change must NOT
  // suppress it just because pct==0 was already attempted (only a
  // SUCCESSFUL write may latch lastWrittenPct_). This time the write
  // succeeds.
  nowUs += 50000;
  bus.queueWrite(wireAddr, /*status=*/0);    // requestEncoder()
  bus.queueRead(wireAddr, data, 4, /*status=*/0);    // collectEncoder()
  bus.queueWrite(wireAddr, /*status=*/0);    // the retried stop write -- succeeds this time
  motor.requestSample();
  motor.tick(nowUs);

  checkFloatEq(motor.appliedDuty(), 0.0f,
               "the retried stop write actually reaches the bus and succeeds -- appliedDuty() is now 0");
  checkUintEq(bus.errCount(Devices::kNezhaDeviceAddr), 1,
              "no new error -- the retry succeeded; errCount stays at the one earlier NAK");
}

// 8. applyTravelCalib() (125-003: narrowed from the pre-125-003
//    applyGains(Gains, Opt<float>) -- see motor.h's own header) takes
//    effect on the SAME instance, same boot, no reflash: a raw step
//    scripted under wheelTravelCalib=1.0 decodes to double the mm once
//    applyTravelCalib(2.0) has been called.
void scenarioApplyTravelCalibTakesEffectSameBootNoReflash() {
  beginScenario("applyTravelCalib() changes subsequent position() decode on the same boot, no reflash");
  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  const uint16_t wireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);

  Devices::MotorConfig cfg = baseNezhaConfig();   // wheelTravelCalib = 1.0, fwdSign = 1
  Devices::NezhaMotor motor(plant, cfg);

  // Prime cycle: anchor at raw=0.
  scriptEncoderRequestCollect(bus, wireAddr, 0.0f);
  motor.requestSample();
  motor.tick(0);

  scriptEncoderRequestCollect(bus, wireAddr, 10.0f);   // raw=100 -- at calib=1.0 decodes to 10.0mm
  motor.requestSample();
  motor.tick(20000);
  checkFloatEq(motor.position(), 10.0f, "position decodes at the constructed wheelTravelCalib (1.0)");

  // Live apply, no reconstruction -- the SAME motor instance.
  motor.applyTravelCalib(2.0f);

  // The SAME raw register value (this helper's own positionMm*10 raw
  // convention, still assuming calib=1.0 to construct the raw bytes) now
  // decodes to DOUBLE the mm, proving the change landed live, same boot.
  scriptEncoderRequestCollect(bus, wireAddr, 15.0f);   // raw=150 -- at calib=2.0 this decodes to 30.0mm
  motor.requestSample();
  motor.tick(40000);
  checkFloatEq(motor.position(), 30.0f,
               "travelCalib=2.0 doubles the SAME raw-derived reading into mm -- confirms the applied change");

  checkUintEq(bus.errCount(Devices::kNezhaDeviceAddr), 0, "no script under-run across the travelCalib sequence");
}

// 9. reconfigure() -- REVISION 1 (114-001, motor.h): guarded, whole-config
//    replacement. Succeeds and fully replaces config_ (fwdSign/
//    wheelTravelCalib -- NOT just the narrow applyTravelCalib() surface)
//    when the motor has never been commanded (mode_ == Mode::None); fails
//    and leaves config_ UNCHANGED when the motor is actively driving and
//    not at rest; succeeds again once the motor returns to rest.
void scenarioReconfigureGuardedWholeConfigReplacement() {
  beginScenario("reconfigure(): succeeds pre-command (whole config_ replace), fails while driving "
                "and not at rest, succeeds again once at rest");
  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  const uint16_t wireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);

  Devices::MotorConfig cfg = baseNezhaConfig();   // port=1, fwdSign=1, wheelTravelCalib=1.0
  Devices::NezhaMotor motor(plant, cfg);

  // --- Step 1: never commanded (mode_ == Mode::None) -- reconfigure()
  //     succeeds and replaces config_ WHOLESALE (fwdSign/wheelTravelCalib
  //     have no other runtime setter besides applyTravelCalib(), which only
  //     touches wheelTravelCalib -- this is the ONLY path that can change
  //     fwdSign post-construction). ---
  Devices::MotorConfig cfgA = baseNezhaConfig();
  cfgA.fwdSign = -1;
  cfgA.wheelTravelCalib = 2.0f;
  bool ok1 = motor.reconfigure(cfgA);
  checkTrue(ok1, "reconfigure() succeeds on a never-yet-commanded motor (mode_ == Mode::None)");

  // raw = positionMm*10 (the helper's own convention, see scriptEncoderRequestCollect()'s
  // header) -- at fwdSign=-1/wheelTravelCalib=2.0, positionMm=10.0 (raw=100) decodes to
  // (100/10)*2.0*(-1) = -20.0mm, distinct from what the OLD config (fwdSign=1/calib=1.0)
  // would have produced (10.0mm) -- an unambiguous proof both fields actually landed.
  scriptEncoderRequestCollect(bus, wireAddr, 10.0f);
  motor.requestSample();
  motor.tick(0);
  checkFloatEq(motor.position(), -20.0f,
               "fwdSign=-1 AND wheelTravelCalib=2.0 both took effect (whole-config replace, "
               "not a partial merge)");

  // --- Step 2: drive the motor (setDuty() + a real landed write) --
  //     mode_ != Mode::None and appliedDuty() != 0 -- NOT at rest.
  //     reconfigure() must now refuse and leave config_ untouched.
  motor.setDuty(0.5f);
  scriptEncoderRequestCollect(bus, wireAddr, 10.0f);   // stationary (stale raw) -- unchanged raw
  motor.requestSample();
  motor.tick(50000);   // first write is slew-exempt -- lands immediately, nonzero
  checkTrue(motor.appliedDuty() != 0.0f, "setup: a real nonzero duty actually landed -- not at rest");

  Devices::MotorConfig cfgB = baseNezhaConfig();
  cfgB.fwdSign = 1;
  cfgB.wheelTravelCalib = 1.0f;
  bool ok2 = motor.reconfigure(cfgB);
  checkTrue(!ok2, "reconfigure() refuses while the motor is actively driving and not at rest");

  // --- Step 3: return to rest (a commanded stop is immediate/unclamped,
  //     exempt from both the slew cap and the write-rate throttle) --
  //     reconfigure() succeeds again. ---
  motor.setDuty(0.0f);
  scriptEncoderRequestCollect(bus, wireAddr, 10.0f);   // still stationary (stale raw)
  motor.requestSample();
  motor.tick(100000);   // stop is immediate -- no 40ms spacing needed
  checkFloatEq(motor.appliedDuty(), 0.0f, "setup: the motor is genuinely at rest again (stop landed)");

  bool ok3 = motor.reconfigure(cfgB);
  checkTrue(ok3, "reconfigure() succeeds again once the motor has returned to rest");

  // A FRESH scripted sample (a genuinely new raw value) now decodes under
  // cfgB's fwdSign=1/wheelTravelCalib=1.0, not cfgA's (-1/2.0) -- the
  // recovery reconfigure() genuinely took effect.
  scriptEncoderRequestCollect(bus, wireAddr, 15.0f);   // raw=150
  motor.requestSample();
  motor.tick(150000);
  checkFloatEq(motor.position(), 15.0f,
               "position now decodes under cfgB's fwdSign=1/wheelTravelCalib=1.0 -- the recovery "
               "reconfigure() genuinely took effect");

  checkUintEq(bus.errCount(Devices::kNezhaDeviceAddr), 0, "no script under-run across the reconfigure() sequence");
}

// 10. Explicit all-zero write shaping (reversalDwell=0/outputDeadband=0) is
//     a pure pass-through -- proves the Opt<float> -> float collapse
//     (sprint 114 ticket 003) did not silently change the MEANING of an
//     explicit zero. A duty that would have been zeroed under the old ship
//     default (0.03) lands unmodified, and a sign flip forwards immediately
//     with no intermediate zero write.
void scenarioExplicitZeroWriteShapingIsPassThrough() {
  beginScenario("explicit reversalDwell=0/outputDeadband=0 is a pure pass-through");
  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  const uint16_t wireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);

  Devices::MotorConfig cfg = baseNezhaConfig();
  cfg.reversalDwell = 0.0f;    // [ms] explicit off, distinct from "unset"
  cfg.outputDeadband = 0.0f;   // [-1,1] explicit off, distinct from "unset"
  cfg.slewRate = 100.0f;       // no slew clamping -- isolates write shaping
  Devices::NezhaMotor m(plant, cfg);

  // A tiny duty that would have been zeroed under the 0.03 ship default
  // lands unmodified -- outputDeadband_ == 0 never suppresses a nonzero duty.
  dutyTick(m, bus, wireAddr, 0.01f, 50000);
  checkFloatEq(m.appliedDuty(), 0.01f,
               "sub-old-deadband duty passes through unmodified at outputDeadband=0");

  // A commanded sign change forwards IMMEDIATELY -- reversalDwell_ == 0
  // skips the dwell transition entirely (no intermediate zero write).
  dutyTick(m, bus, wireAddr, -0.5f, 100000);
  checkFloatEq(m.appliedDuty(), -0.5f,
               "sign flip forwarded immediately at reversalDwell=0 -- no dwell armed");
}

// sampleTime() (124-002, protocol-v5 §B2 prerequisite) -- 125-003: with the
// freshness gate deleted, sampleTime() now simply reports the nowUs of the
// most recent tick() call (see nezha_motor.h's own doc comment) -- this is
// the accessor the wire's `enc_left`/`enc_right` age fields read; ticket
// 004's App::WheelObserver will restore a real per-sample age.
void scenarioSampleTimeReflectsMostRecentTick() {
  beginScenario("sampleTime() reflects the most recent tick() call's own nowUs");
  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  const uint16_t wireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);

  Devices::MotorConfig cfg = baseNezhaConfig();
  Devices::NezhaMotor motor(plant, cfg);

  uint64_t nowUs = 0;
  scriptEncoderRequestCollect(bus, wireAddr, 0.0f);
  motor.requestSample();
  motor.tick(nowUs);
  checkTrue(motor.sampleTime() == nowUs, "sampleTime() == the tick's own nowUs");

  nowUs = 123456;
  scriptEncoderRequestCollect(bus, wireAddr, 5.0f);
  motor.requestSample();
  motor.tick(nowUs);
  checkTrue(motor.sampleTime() == nowUs, "sampleTime() advances to the next tick's own nowUs");

  checkUintEq(bus.errCount(Devices::kNezhaDeviceAddr), 0, "no script under-run across the run");
}

}  // namespace

int main() {
  scenarioReversalDwellWritesZeroThenHoldsThroughDeadline();
  scenarioOutputDeadbandBoostsSubDeadbandNonzeroDutyExactZeroStaysZero();
  scenarioStandstillGuardedResetGatesOnRestTicks();
  scenarioWedgeLatchAndSuspectDeriveAsBefore();
  scenarioRequestCollectPairingYieldsExpectedPositionVelocity();
  scenarioVelocityReadsZeroUntilTwoValidSamplesCollected();
  scenarioSetDutyTickWritesExactlyTheGivenDutyThroughShaping();
  scenarioNakedStopWriteIsRetriedNextTickNotLatched();
  scenarioApplyTravelCalibTakesEffectSameBootNoReflash();
  scenarioReconfigureGuardedWholeConfigReplacement();
  scenarioExplicitZeroWriteShapingIsPassThrough();
  scenarioSampleTimeReflectsMostRecentTick();

  if (g_failureCount == 0) {
    std::printf("OK: all devices motor scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the devices motor scenarios\n",
              g_failureCount);
  return 1;
}
