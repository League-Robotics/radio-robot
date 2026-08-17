// devices_motor_harness.cpp — off-hardware acceptance harness (originally
// ticket DB-004; restructured 2026-07-18 with the Motor-interface split):
// exercises Hardware::NezhaMotor's OWN write shaping (reversal dwell +
// output deadband, folded into the leaf's writeShapedDuty()), its
// request/collect encoder pairing, and duty passthrough through the real
// leaf against a TestSim::SimPlant (108-002), scripted deterministically
// via TestSim::ScriptedI2CHook (108-009) — AND the Hardware::MotorArmor
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
// outright by 128-015, see src/firm/motion/DESIGN.md's "wheel control
// generations" note -- / pending ticket 004's Core::WheelObserver, see
// nezha_motor.cpp's own file header). What survives: protocol/bus-hygiene/
// dwell/deadband/clamp scenarios (unchanged
// behavior) plus a NEW duty-passthrough scenario (scenario 5 below)
// replacing the deleted PID-chase coverage.
//
// Migrated by sprint 108 ticket 009
// (clasi/sprints/108-pure-i2cbus-clock-interfaces-and-a-real-simplant-
// simulator-sim-mode-tours/tickets/009-migrate-the-13-register-level-unit-
// tests-to-python-simplant-hook-tests-delete-c-harnesses.md) off the deleted
// src/firm/devices/i2c_bus_host.cpp scripted-FIFO Platform::I2CBus fake (ticket
// 001 reduced Platform::I2CBus to a pure interface and removed it).
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
// and src/firm/hardware/nezha/nezha_motor.cpp under -DHOST_BUILD, then runs the
// resulting binary via subprocess and asserts exit code 0.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include "hal/device_config.h"
#include "hal/device_types.h"
#include "hardware/generic/motor_armor.h"
#include "hardware/nezha/nezha_motor.h"
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
// A dependency-free Hal::Motor double for the MotorArmor DECORATOR
// scenarios (2026-07-18 restructure: MotorArmor composes a Motor& instead
// of being the leaf's base class). Test-settable position/velocity/
// appliedDuty; counts the reset verbs the armor's standstill guard
// dispatches. No I2C, no CODAL — devices/motor.h + device_config.h only.
class MockMotor : public Hal::Motor {
 public:
  // --- Motor faceplate (trivial forwarding/recording) ---
  void begin() override {}
  void requestSample() override {}
  void setDuty(float duty) override { lastDutyCmd = duty; }
  void setNeutral(Hal::Neutral) override {}
  // REVISION 1 (114-001, motor.h): trivial always-succeeds stand-in --
  // MockMotor has no boot-identity config of its own to actually reassign,
  // it only needs to satisfy the new pure virtual and let scenarios assert
  // it was called.
  bool reconfigure(const Hal::MotorConfig&) override {
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
Hal::MotorConfig defaultArmorConfig() {
  Hal::MotorConfig cfg;
  cfg.outputDeadband = 0.03f;   // [-1,1] fraction
  return cfg;
}

// --- Write-shaping scenarios (real NezhaMotor — the dwell/deadband gate
// lives in the leaf's own writeShapedDuty()) ------------------------------

// Forward declarations — defined with the NezhaMotor scenario helpers below.
void scriptEncoderRequestCollect(TestSim::ScriptedI2CHook& bus, uint16_t wireAddr,
                                  float positionMm);
Hal::MotorConfig baseNezhaConfig();

// Scripts one encoder request/collect cycle and drives one setDuty+tick
// pass; returns nothing — callers assert on appliedDuty() transitions
// (write-on-change/throttle make exact write-call sequences a raw-path
// concern; appliedDuty() is the shaped outcome).
void dutyTick(Hardware::NezhaMotor& motor, TestSim::ScriptedI2CHook& bus,
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
    const uint16_t wireAddr = static_cast<uint16_t>(Hardware::kNezhaDeviceAddr << 1);
    Hal::MotorConfig cfg = baseNezhaConfig();   // dwell/deadband = 100ms/0.03 (baseNezhaConfig()'s own explicit set)
    cfg.slewRate = 100.0f;                          // no slew clamping — isolates the dwell
    Hardware::NezhaMotor m(plant, cfg);

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
    const uint16_t wireAddr = static_cast<uint16_t>(Hardware::kNezhaDeviceAddr << 1);
    Hal::MotorConfig cfg = baseNezhaConfig();
    cfg.slewRate = 100.0f;
    Hardware::NezhaMotor m(plant, cfg);

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
    const uint16_t wireAddr = static_cast<uint16_t>(Hardware::kNezhaDeviceAddr << 1);
    Hal::MotorConfig cfg = baseNezhaConfig();
    cfg.slewRate = 100.0f;
    Hardware::NezhaMotor m(plant, cfg);
    dutyTick(m, bus, wireAddr, 0.0f, 50000);
    checkFloatEq(m.appliedDuty(), 0.0f, "exact duty==0.0f writes 0 immediately");
  }

  // (b) Sub-deadband nonzero, positive: boosted to +outputDeadband_ (0.03),
  //     not zeroed.
  {
    TestSim::SimPlant plant;
    TestSim::ScriptedI2CHook bus(plant);
    const uint16_t wireAddr = static_cast<uint16_t>(Hardware::kNezhaDeviceAddr << 1);
    Hal::MotorConfig cfg = baseNezhaConfig();   // outputDeadband = 0.03
    cfg.slewRate = 100.0f;
    Hardware::NezhaMotor m(plant, cfg);
    dutyTick(m, bus, wireAddr, 0.01f, 50000);   // 0 < 0.01 < 0.03 -- genuine nonzero, sub-deadband
    checkFloatEq(m.appliedDuty(), 0.03f,
                 "sub-deadband positive duty boosted to +outputDeadband_, not zeroed");
  }

  // (c) Sub-deadband nonzero, negative: boosted to -outputDeadband_
  //     (sign-preserving).
  {
    TestSim::SimPlant plant;
    TestSim::ScriptedI2CHook bus(plant);
    const uint16_t wireAddr = static_cast<uint16_t>(Hardware::kNezhaDeviceAddr << 1);
    Hal::MotorConfig cfg = baseNezhaConfig();
    cfg.slewRate = 100.0f;
    Hardware::NezhaMotor m(plant, cfg);
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
    const uint16_t wireAddr = static_cast<uint16_t>(Hardware::kNezhaDeviceAddr << 1);
    Hal::MotorConfig cfg = baseNezhaConfig();
    cfg.slewRate = 100.0f;
    Hardware::NezhaMotor m(plant, cfg);
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
    const uint16_t wireAddr = static_cast<uint16_t>(Hardware::kNezhaDeviceAddr << 1);
    Hal::MotorConfig cfg = baseNezhaConfig();   // reversalDwell = 100ms, outputDeadband = 0.03
    cfg.slewRate = 100.0f;
    Hardware::NezhaMotor m(plant, cfg);

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
    Hardware::MotorArmor armor(inner);
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
    Hardware::MotorArmor armor(inner);
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
    Hardware::MotorArmor idle(inner);
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
    Hardware::MotorArmor moving(inner);
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

// --- Hardware::NezhaMotor scenarios (real leaf, scripted Platform::I2CBus) ---

// Packs position [counts] into the little-endian int32 raw encoder
// reading NezhaMotor::collectEncoder() decodes. The leaf is COUNTS-NATIVE
// (1 count = 0.1 deg, the register's own unit; the old mm decode is
// deleted), and fwdSign=+1 in every scenario below, so raw == position
// exactly (no scaling, no rounding drift).
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
                                  float position) {  // [counts]
  bus.queueWrite(wireAddr, /*status=*/0);   // requestEncoder()'s 0x46 write
  bus.queueWrite(wireAddr, /*status=*/0);   // slack: a possible same-cycle duty write (0x60)

  int32_t raw = static_cast<int32_t>(std::lround(position));
  uint8_t data[4] = {
      static_cast<uint8_t>(raw & 0xFF),
      static_cast<uint8_t>((raw >> 8) & 0xFF),
      static_cast<uint8_t>((raw >> 16) & 0xFF),
      static_cast<uint8_t>((raw >> 24) & 0xFF),
  };
  bus.queueRead(wireAddr, data, 4, /*status=*/0);   // collectEncoder()'s 4-byte read
}

Hal::MotorConfig baseNezhaConfig() {
  Hal::MotorConfig cfg;
  cfg.port = 1;
  cfg.fwdSign = 1;
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
  const uint16_t wireAddr = static_cast<uint16_t>(Hardware::kNezhaDeviceAddr << 1);

  Hardware::NezhaMotor motor(plant, baseNezhaConfig());

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
  checkUintEq(bus.errCount(Hardware::kNezhaDeviceAddr), 0, "no script under-run across the pairing");
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
  const uint16_t wireAddr = static_cast<uint16_t>(Hardware::kNezhaDeviceAddr << 1);

  Hardware::NezhaMotor motor(plant, baseNezhaConfig());

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

  checkUintEq(bus.errCount(Hardware::kNezhaDeviceAddr), 0,
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
  const uint16_t wireAddr = static_cast<uint16_t>(Hardware::kNezhaDeviceAddr << 1);

  Hal::MotorConfig cfg = baseNezhaConfig();
  cfg.slewRate = 100.0f;   // no slew clamping -- isolates the passthrough

  Hardware::NezhaMotor motor(plant, cfg);

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

  checkUintEq(bus.errCount(Hardware::kNezhaDeviceAddr), 0, "no script under-run across the duty-passthrough sequence");
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
  const uint16_t wireAddr = static_cast<uint16_t>(Hardware::kNezhaDeviceAddr << 1);

  Hal::MotorConfig cfg = baseNezhaConfig();
  cfg.slewRate = 100.0f;   // no slew clamping -- isolates the write-status behavior
  Hardware::NezhaMotor motor(plant, cfg);

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
  checkUintEq(bus.errCount(Hardware::kNezhaDeviceAddr), 1,
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
  checkUintEq(bus.errCount(Hardware::kNezhaDeviceAddr), 1,
              "no new error -- the retry succeeded; errCount stays at the one earlier NAK");
}

// 7b. 129-001 (issue 07, the 2026-07-31 runaway): a stop write that
//     SUCCEEDS at the bus/status level (unlike scenario 7's NAK) but never
//     actually took physical effect -- the Nezha brick's latching behavior
//     the issue describes -- must still be RE-ISSUED on the next tick while
//     the wheel is measurably still moving, not permanently suppressed by
//     write-on-change just because lastWrittenPct_ already claims 0 was
//     sent. lastWrittenPct_ records the ATTEMPT, not the landing.
//
//     Modeled here as a scripted encoder that keeps reporting genuine
//     motion (velocity() well above kStopConfirmVelocity, mirrored below as
//     a plain literal -- see nezha_motor.h's own doc comment) across BOTH
//     the tick where the stop write lands successfully AND the following
//     tick, standing in for "the write was attempted and acked, but the
//     brick kept its prior nonzero speed." The oracle is bus.errCount():
//     the follow-up tick's 0x60 slot is deliberately left UNSCRIPTED, so a
//     write-on-change bypass that actually re-attempts the bus write (the
//     fixed behavior) trips a script-mismatch error; the pre-129-001 bug
//     (an unconditional `pct == lastWrittenPct_` suppression) would instead
//     skip the attempt entirely and leave errCount unchanged -- this test
//     fails under that old behavior, which is the point.
//
//     Deliberately does NOT use scriptEncoderRequestCollect()'s own
//     speculative "slack" 0x60 entry (unlike most other scenarios in this
//     file): an unconsumed slack write silently carries forward into the
//     NEXT tick's queue (ScriptedI2CHook's FIFOs don't expire unused
//     entries), which would hand the critical final write below a stray
//     already-queued success entry instead of the empty queue the oracle
//     depends on. Every write this scenario expects is queued exactly once,
//     by hand.
void scenarioDroppedStopWriteReassertsZeroWhileVelocityNonzero() {
  beginScenario("a stop write that lands successfully but doesn't physically take still "
                "re-asserts zero next tick while velocity() is nonzero");
  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  const uint16_t wireAddr = static_cast<uint16_t>(Hardware::kNezhaDeviceAddr << 1);

  Hal::MotorConfig cfg = baseNezhaConfig();
  cfg.slewRate = 100.0f;   // no slew clamping -- isolates the write-on-change behavior
  Hardware::NezhaMotor motor(plant, cfg);

  // kStopConfirmVelocity (nezha_motor.h) == 100 counts/s (the counts
  // rebake of the old 8 mm/s) -- 500 counts/s (25 counts every 50ms tick)
  // is comfortably above it, with margin for any measurement slop.
  const float kStepPosition = 25.0f;  // [counts] per 50ms tick == 500 counts/s
  uint64_t nowUs = 0;
  float pos = 0.0f;

  // Queues exactly one requestEncoder()/collectEncoder() pair (0x46 write +
  // 4-byte read), no speculative slack -- see this scenario's own header for
  // why. `expectDutyWrite` additionally queues exactly one MORE write slot
  // for a duty write this tick is expected to actually attempt.
  auto scriptCycle = [&](float position, bool expectDutyWrite) {  // [counts]
    bus.queueWrite(wireAddr, /*status=*/0);   // requestEncoder()'s 0x46 write
    int32_t raw = static_cast<int32_t>(std::lround(position));
    uint8_t data[4] = {
        static_cast<uint8_t>(raw & 0xFF),
        static_cast<uint8_t>((raw >> 8) & 0xFF),
        static_cast<uint8_t>((raw >> 16) & 0xFF),
        static_cast<uint8_t>((raw >> 24) & 0xFF),
    };
    bus.queueRead(wireAddr, data, 4, /*status=*/0);   // collectEncoder()'s 4-byte read
    if (expectDutyWrite) {
      bus.queueWrite(wireAddr, /*status=*/0);   // the expected 0x60 duty write
    }
  };

  // Prime: baseline anchor sample, motor never yet commanded (mode_ ==
  // None) -- no write attempted this tick.
  scriptCycle(pos, /*expectDutyWrite=*/false);
  motor.requestSample();
  motor.tick(nowUs);

  // Establish a genuine nonzero applied duty (the motor's very first-ever
  // commanded write -- always attempted, slew-exempt) -- the wheel starts
  // moving.
  nowUs += 50000;
  pos += kStepPosition;
  motor.setDuty(0.5f);
  scriptCycle(pos, /*expectDutyWrite=*/true);
  motor.requestSample();
  motor.tick(nowUs);
  checkFloatEq(motor.appliedDuty(), 0.5f, "nonzero duty established by a successful write");
  checkFloatEq(motor.velocity(), 500.0f, "setup: the wheel is genuinely moving");

  // Command a stop. pct changes (50 -> 0), a genuine value change, so this
  // write is attempted and SUCCEEDS regardless of the fix under test --
  // lastWrittenPct_ commits to 0 -- but the scripted encoder keeps
  // advancing, standing in for a write that was acked yet never physically
  // landed.
  nowUs += 50000;
  pos += kStepPosition;
  motor.setDuty(0.0f);
  scriptCycle(pos, /*expectDutyWrite=*/true);
  motor.requestSample();
  motor.tick(nowUs);
  checkFloatEq(motor.appliedDuty(), 0.0f,
               "the stop write itself succeeds -- lastWrittenPct_ now claims 0 was sent");
  checkFloatEq(motor.velocity(), 500.0f,
               "the wheel is STILL measurably moving despite the committed zero write -- "
               "the exact 2026-07-31 defect scenario");
  const uint32_t errsBeforeReassert = bus.errCount(Hardware::kNezhaDeviceAddr);

  // Next tick: the SAME target (0.0f) is commanded again while velocity()
  // is still nonzero. pct == lastWrittenPct_ (0 == 0) this time -- the
  // write-on-change guard would normally skip it. No duty write is queued
  // for this cycle, so a re-attempted write has nothing scripted to consume
  // and trips a script-mismatch error -- the oracle: a suppressed write
  // leaves errCount unchanged; a re-issued one increments it.
  nowUs += 50000;
  pos += kStepPosition;
  motor.setDuty(0.0f);
  scriptCycle(pos, /*expectDutyWrite=*/false);
  motor.requestSample();
  motor.tick(nowUs);

  checkFloatEq(motor.velocity(), 500.0f,
               "setup check: velocity() was still nonzero at the moment of this tick's write");
  checkUintEq(bus.errCount(Hardware::kNezhaDeviceAddr), errsBeforeReassert + 1,
              "the re-asserted stop write was actually attempted on the bus (stopNotTaken "
              "bypassed write-on-change) -- NOT silently suppressed because lastWrittenPct_ "
              "already claimed 0 was sent");
}

// (Scenario 8, applyTravelCalib(), is DELETED with the method itself —
// the leaf is counts-native and the mm decode no longer exists.)

// 9. reconfigure() -- REVISION 1 (114-001, motor.h): guarded, whole-config
//    replacement, proven through fwdSign (the one decode-visible field
//    with no other post-construction setter). Succeeds when the motor has
//    never been commanded (mode_ == Mode::None); fails and leaves config_
//    UNCHANGED when the motor is actively driving and not at rest;
//    succeeds again once the motor returns to rest.
void scenarioReconfigureGuardedWholeConfigReplacement() {
  beginScenario("reconfigure(): succeeds pre-command (whole config_ replace), fails while driving "
                "and not at rest, succeeds again once at rest");
  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  const uint16_t wireAddr = static_cast<uint16_t>(Hardware::kNezhaDeviceAddr << 1);

  Hal::MotorConfig cfg = baseNezhaConfig();   // port=1, fwdSign=1
  Hardware::NezhaMotor motor(plant, cfg);

  // --- Step 1: never commanded (mode_ == Mode::None) -- reconfigure()
  //     succeeds and replaces config_ WHOLESALE (fwdSign has no other
  //     post-construction setter -- this is the ONLY path that can change
  //     it). ---
  Hal::MotorConfig cfgA = baseNezhaConfig();
  cfgA.fwdSign = -1;
  bool ok1 = motor.reconfigure(cfgA);
  checkTrue(ok1, "reconfigure() succeeds on a never-yet-commanded motor (mode_ == Mode::None)");

  // raw == position (counts-native helper convention) -- at fwdSign=-1,
  // position=10 (raw=10) decodes to -10 counts, distinct from what the OLD
  // config (fwdSign=1) would have produced (+10) -- unambiguous proof the
  // replacement actually landed.
  scriptEncoderRequestCollect(bus, wireAddr, 10.0f);
  motor.requestSample();
  motor.tick(0);
  checkFloatEq(motor.position(), -10.0f,
               "fwdSign=-1 took effect (whole-config replace, not a partial merge)");

  // --- Step 2: drive the motor (setDuty() + a real landed write) --
  //     mode_ != Mode::None and appliedDuty() != 0 -- NOT at rest.
  //     reconfigure() must now refuse and leave config_ untouched.
  motor.setDuty(0.5f);
  scriptEncoderRequestCollect(bus, wireAddr, 10.0f);   // stationary (stale raw) -- unchanged raw
  motor.requestSample();
  motor.tick(50000);   // first write is slew-exempt -- lands immediately, nonzero
  checkTrue(motor.appliedDuty() != 0.0f, "setup: a real nonzero duty actually landed -- not at rest");

  Hal::MotorConfig cfgB = baseNezhaConfig();
  cfgB.fwdSign = 1;
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
  // cfgB's fwdSign=1, not cfgA's -1 -- the recovery reconfigure()
  // genuinely took effect.
  scriptEncoderRequestCollect(bus, wireAddr, 15.0f);   // raw=15
  motor.requestSample();
  motor.tick(150000);
  checkFloatEq(motor.position(), 15.0f,
               "position now decodes under cfgB's fwdSign=1 -- the recovery "
               "reconfigure() genuinely took effect");

  checkUintEq(bus.errCount(Hardware::kNezhaDeviceAddr), 0, "no script under-run across the reconfigure() sequence");
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
  const uint16_t wireAddr = static_cast<uint16_t>(Hardware::kNezhaDeviceAddr << 1);

  Hal::MotorConfig cfg = baseNezhaConfig();
  cfg.reversalDwell = 0.0f;    // [ms] explicit off, distinct from "unset"
  cfg.outputDeadband = 0.0f;   // [-1,1] explicit off, distinct from "unset"
  cfg.slewRate = 100.0f;       // no slew clamping -- isolates write shaping
  Hardware::NezhaMotor m(plant, cfg);

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

// 10b. SIGMA-DELTA duty quantizer (133-002). The brick takes an INTEGER
//      PERCENT, and at Core::DifferentialDrive::kDutyPerSpeed = 0.001182 one count is
//      8.46 mm/s -- 5.6% of a 150 mm/s command, and roughly twenty times
//      the left/right imbalance the wheel controller was being tuned to
//      close. Plain rounding cannot represent a command between two
//      counts at all, so no gain upstream can either.
//
//      Two properties, and the second is the safety one:
//        (a) a duty strictly between two counts is represented ACROSS
//            successive cycles -- the written percents straddle both
//            neighbours and their mean is the fraction actually asked
//            for, rather than every cycle truncating to the same count;
//        (b) the carry is DISCARDED on a commanded zero. A residual left
//            over from the last nonzero duty would round to +-1 on the
//            next command and, worse, could creep a wheel that was told
//            to stop. Sub-count fidelity across a stop is the correct
//            thing to lose.
void scenarioSigmaDeltaRepresentsSubCountDutyAndDropsCarryOnStop() {
  beginScenario("writeRawDuty()'s sigma-delta represents a sub-count duty over successive cycles, "
                "and discards its carry on a commanded zero (133-002)");
  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  const uint16_t wireAddr = static_cast<uint16_t>(Hardware::kNezhaDeviceAddr << 1);

  // (a) 12.3% -- strictly between counts 12 and 13. Plain lroundf() would
  //     write 12 on every single cycle forever, an error of 0.3 of a count
  //     held indefinitely in one direction.
  {
    Hal::MotorConfig cfg = baseNezhaConfig();
    cfg.slewRate = 100.0f;  // no slew clamping -- isolates the quantizer
    Hardware::NezhaMotor m(plant, cfg);

    const float duty = 0.123f;  // [-1,1] -> 12.3 percent
    const int kTicks = 20;      // two full periods of the residual pattern
    uint64_t nowUs = 50000;
    float sumPct = 0.0f;
    bool sawLow = false;
    bool sawHigh = false;
    for (int i = 0; i < kTicks; ++i) {
      dutyTick(m, bus, wireAddr, duty, nowUs);
      const float pct = m.appliedDuty() * 100.0f;  // [percent] the integer actually written
      sumPct += pct;
      if (pct < 12.5f) sawLow = true;
      if (pct > 12.5f) sawHigh = true;
      nowUs += 50000;
    }

    checkTrue(sawLow && sawHigh,
             "the written percent straddles BOTH neighbouring counts -- it is not truncating to "
             "one of them every cycle");
    checkFloatEq(sumPct / static_cast<float>(kTicks), 12.3f,
                "and the TIME-AVERAGED percent equals the fractional duty actually wanted, which "
                "is the sub-count resolution the integer wire field cannot otherwise carry",
                0.05f);
  }

  // (b) The carry is discarded on a commanded zero. Built as a
  //     discriminator, not an inspection: 12.6% rounds to 13 and leaves a
  //     -0.4 residual. Command zero, then command 12.6% again -- with the
  //     carry cleared that is 13 again; with the carry carried across the
  //     stop it would be 12.6 - 0.4 = 12.2, i.e. 12.
  {
    Hal::MotorConfig cfg = baseNezhaConfig();
    cfg.slewRate = 100.0f;
    Hardware::NezhaMotor m(plant, cfg);

    dutyTick(m, bus, wireAddr, 0.126f, 50000);
    checkFloatEq(m.appliedDuty(), 0.13f,
                "setup: 12.6% rounds up to count 13, leaving a -0.4 residual in the carry");

    dutyTick(m, bus, wireAddr, 0.0f, 100000);
    checkFloatEq(m.appliedDuty(), 0.0f,
                "a commanded zero writes EXACTLY count 0 -- never +-1 off the back of a residual, "
                "which would creep a stopped wheel");

    dutyTick(m, bus, wireAddr, 0.126f, 150000);
    checkFloatEq(m.appliedDuty(), 0.13f,
                "and the SAME command after the stop rounds identically to the first time: the "
                "carry was discarded at zero, not carried across it (a surviving -0.4 would have "
                "written count 12 here)");
  }

  // (c) A commanded zero stays exactly zero however long it is held --
  //     the carry cannot re-accumulate while the command is zero.
  {
    Hal::MotorConfig cfg = baseNezhaConfig();
    cfg.slewRate = 100.0f;
    Hardware::NezhaMotor m(plant, cfg);

    dutyTick(m, bus, wireAddr, 0.457f, 50000);  // build an arbitrary residual
    uint64_t nowUs = 100000;
    for (int i = 0; i < 20; ++i) {
      dutyTick(m, bus, wireAddr, 0.0f, nowUs);
      checkFloatEq(m.appliedDuty(), 0.0f,
                  "commanded zero writes exactly 0 on every tick it is held -- the sigma-delta "
                  "never accumulates a residual out of a zero command");
      nowUs += 50000;
    }
  }
}

// sampleTime() (124-002, protocol-v5 §B2 prerequisite) -- 125-003: with the
// freshness gate deleted, sampleTime() simply reported the nowUs of the most
// recent tick() call. 131-002 (issue A-commanded-zero-leaks-through-stage-
// b.md) narrowed that: sampleTime() now returns lastFreshUs_, which advances
// ONLY on a tick whose collect actually succeeded -- on the HEALTHY path
// this scenario exercises (every collect below succeeds), that is still
// indistinguishable from "the most recent tick() call's own nowUs," so this
// scenario's own assertions are unchanged; see
// scenarioSampleTimeHoldsLastSuccessfulCollectAcrossFailedCollects() below
// for the failed-collect contrast. This is the accessor the wire's
// `enc_left`/`enc_right` age fields read; ticket 004's Core::WheelObserver
// will restore a real per-sample estimator on top of it.
void scenarioSampleTimeReflectsMostRecentTick() {
  beginScenario("sampleTime() reflects the most recent tick() call's own nowUs (healthy path)");
  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  const uint16_t wireAddr = static_cast<uint16_t>(Hardware::kNezhaDeviceAddr << 1);

  Hal::MotorConfig cfg = baseNezhaConfig();
  Hardware::NezhaMotor motor(plant, cfg);

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

  checkUintEq(bus.errCount(Hardware::kNezhaDeviceAddr), 0, "no script under-run across the run");
}

// 12. NEW (131-002, issue A-commanded-zero-leaks-through-stage-b.md): a
//     failed encoder collect (connected_ false -- the scripted read below
//     NAKs, so collectEncoder()'s own `connected_ = pendingEncRequestOk_ &&
//     (readResult == kOk)` comes back false) must NOT advance sampleTime().
//     lastFreshUs_ holds the last SUCCESSFUL collect's timestamp across one
//     or more failing ticks in a row, then resumes advancing once a collect
//     succeeds again. Contrasts directly with
//     scenarioSampleTimeReflectsMostRecentTick() above, which covers the
//     healthy path where every collect succeeds. velocity_'s own diffing
//     behavior is untouched by this fix (not re-asserted here beyond the
//     existing scenarios above) -- this scenario is scoped to the
//     freshness timestamp only, per the ticket's own acceptance criterion.
void scenarioSampleTimeHoldsLastSuccessfulCollectAcrossFailedCollects() {
  beginScenario("sampleTime() holds the last successful collect's timestamp across failed "
                "collects (131-002)");
  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  const uint16_t wireAddr = static_cast<uint16_t>(Hardware::kNezhaDeviceAddr << 1);

  Hal::MotorConfig cfg = baseNezhaConfig();
  Hardware::NezhaMotor motor(plant, cfg);

  // Baseline: a genuinely successful collect at nowUs=0.
  scriptEncoderRequestCollect(bus, wireAddr, 0.0f);
  motor.requestSample();
  motor.tick(0);
  checkTrue(motor.connected(), "baseline collect succeeds");
  checkTrue(motor.sampleTime() == 0, "sampleTime() reflects the baseline successful collect");

  // A second successful collect advances sampleTime() normally -- confirms
  // the healthy reference behavior before introducing the failure below.
  scriptEncoderRequestCollect(bus, wireAddr, 5.0f);
  motor.requestSample();
  motor.tick(20000);
  checkTrue(motor.connected(), "second collect also succeeds");
  checkTrue(motor.sampleTime() == 20000, "sampleTime() advances on a second successful collect");

  // A failed collect: requestEncoder()'s own 0x46 write succeeds
  // (pendingEncRequestOk_ true), but the read collectEncoder() performs
  // NAKs -- connected_ becomes false. Must NOT advance sampleTime(), across
  // TWO consecutive failing ticks.
  uint8_t garbage[4] = {0, 0, 0, 0};
  bus.queueWrite(wireAddr, /*status=*/0);              // requestEncoder()'s 0x46 write succeeds
  bus.queueRead(wireAddr, garbage, 4, /*status=*/-5);  // collectEncoder()'s read NAKs
  motor.requestSample();
  motor.tick(40000);
  checkTrue(!motor.connected(), "the scripted read failure reports disconnected");
  checkTrue(motor.sampleTime() == 20000,
            "sampleTime() holds the LAST SUCCESSFUL collect's timestamp across a failed collect, "
            "not the failed attempt's own nowUs (40000)");

  bus.queueWrite(wireAddr, /*status=*/0);
  bus.queueRead(wireAddr, garbage, 4, /*status=*/-5);
  motor.requestSample();
  motor.tick(60000);
  checkTrue(!motor.connected(), "a second consecutive failed collect also reports disconnected");
  checkTrue(motor.sampleTime() == 20000,
            "sampleTime() still holds the last successful collect's timestamp across TWO "
            "consecutive failing ticks");

  // A subsequent successful collect resumes advancing sampleTime() normally.
  scriptEncoderRequestCollect(bus, wireAddr, 8.0f);
  motor.requestSample();
  motor.tick(80000);
  checkTrue(motor.connected(), "collect recovers");
  checkTrue(motor.sampleTime() == 80000,
            "sampleTime() resumes advancing once the collect succeeds again");
}

// 131-004 (position-rebaseline-destroys-the-pose.md secondary defect):
// rebaseline()/softRebaseline() re-anchors position() to 0.0f but must NOT
// zero velocity() -- this is a SOFTWARE-only re-anchor of the position
// baseline mid-motion, not a real stop, and velocity_ already holds THIS
// cycle's own genuinely-computed rate (tick() always runs before
// Core::RobotLoop::publishWheel() calls rebaseline() the same cycle).
// Before this ticket, softRebaseline() also wrote velocity_ = 0.0f, so a
// wheel reported 0 mm/s for 1-2 cycles purely because of the re-anchor --
// which Stage B would act on as a genuine stop
// (A-commanded-zero-leaks-through-stage-b.md).
void scenarioRebaselinePreservesVelocityAcrossTheBoundary() {
  beginScenario("rebaseline() preserves velocity() across the software re-anchor (131-004)");
  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  const uint16_t wireAddr = static_cast<uint16_t>(Hardware::kNezhaDeviceAddr << 1);

  Hardware::NezhaMotor motor(plant, baseNezhaConfig());

  // Two paired cycles establish a real, nonzero velocity -- mirrors
  // scenarioRequestCollectPairingYieldsExpectedPositionVelocity() above.
  scriptEncoderRequestCollect(bus, wireAddr, 0.0f);
  motor.requestSample();
  motor.tick(0);

  scriptEncoderRequestCollect(bus, wireAddr, 10.0f);
  motor.requestSample();
  motor.tick(20000);  // 20ms later

  checkFloatEq(motor.velocity(), 500.0f, "velocity established pre-rebaseline (10mm / 20ms)",
               1e-6f);

  // Simulates Core::RobotLoop::publishWheel() calling rebaseline() this same
  // cycle, once this tick's own already-collected position crosses the
  // rebaseline margin.
  motor.rebaseline();

  checkFloatEq(motor.position(), 0.0f, "rebaseline() re-anchors position() to 0", 1e-6f);
  checkTrue(motor.velocity() != 0.0f,
            "velocity() is NOT zeroed by rebaseline() -- it still holds the last genuinely "
            "computed value across the boundary cycle (was zeroed pre-131-004)");
  checkFloatEq(motor.velocity(), 500.0f,
               "velocity() is UNCHANGED by rebaseline(), not merely nonzero -- it holds exactly "
               "this cycle's own already-computed rate",
               1e-6f);

  // The first post-rebaseline tick's own collect is a fresh baseline-only
  // anchor off the rebaselined position (hasLastTick_ was reset to false
  // by rebaseline()) -- no velocity is (re-)computed THAT tick either,
  // mirroring the boot-anchor case
  // (scenarioVelocityReadsZeroUntilTwoValidSamplesCollected() above):
  // velocity() simply continues holding its last real value.
  scriptEncoderRequestCollect(bus, wireAddr, 0.5f);
  motor.requestSample();
  motor.tick(40000);
  checkFloatEq(motor.velocity(), 500.0f,
               "the first post-rebaseline tick is a baseline-only anchor -- velocity() still "
               "holds its last real value, not yet re-derived from the new baseline",
               1e-6f);

  // The SECOND post-rebaseline tick resumes computing a real difference
  // quotient off the new (rebaselined) baseline.
  scriptEncoderRequestCollect(bus, wireAddr, 1.5f);
  motor.requestSample();
  motor.tick(60000);
  checkFloatEq(motor.velocity(), 50.0f,
               "velocity() resumes a real difference quotient off the rebaselined baseline "
               "(1.0mm / 20ms) once a second post-rebaseline sample lands",
               1e-6f);
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
  scenarioDroppedStopWriteReassertsZeroWhileVelocityNonzero();
  scenarioReconfigureGuardedWholeConfigReplacement();
  scenarioExplicitZeroWriteShapingIsPassThrough();
  scenarioSigmaDeltaRepresentsSubCountDutyAndDropsCarryOnStop();
  scenarioSampleTimeReflectsMostRecentTick();
  scenarioSampleTimeHoldsLastSuccessfulCollectAcrossFailedCollects();
  scenarioRebaselinePreservesVelocityAcrossTheBoundary();

  if (g_failureCount == 0) {
    std::printf("OK: all devices motor scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the devices motor scenarios\n",
              g_failureCount);
  return 1;
}
