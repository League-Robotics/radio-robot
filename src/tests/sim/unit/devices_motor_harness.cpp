// devices_motor_harness.cpp — off-hardware acceptance harness (originally
// ticket DB-004; restructured 2026-07-18 with the Motor-interface split):
// exercises Devices::NezhaMotor's OWN write shaping (reversal dwell +
// output deadband, folded into the leaf's writeShapedDuty()), its
// request/collect encoder pairing, embedded PID, and PID-on/off dispatch
// through the real leaf against a TestSim::SimPlant (108-002), scripted
// deterministically via TestSim::ScriptedI2CHook (108-009) — AND the
// Devices::MotorArmor DECORATOR's observation/recovery policy
// (standstill-guarded resets, motion-qualified wedge reporting) through a
// dependency-free MockMotor inner double.
//
// Migrated by sprint 108 ticket 009
// (clasi/sprints/108-pure-i2cbus-clock-interfaces-and-a-real-simplant-
// simulator-sim-mode-tours/tickets/009-migrate-the-13-register-level-unit-
// tests-to-python-simplant-hook-tests-delete-c-harnesses.md) off the deleted
// src/firm/devices/i2c_bus_host.cpp scripted-FIFO Devices::I2CBus fake (ticket
// 001 reduced Devices::I2CBus to a pure interface and removed it). Every
// scenario below is otherwise UNCHANGED from the pre-migration harness --
// only the bus/scripting plumbing moved from the deleted concrete
// Devices::I2CBus onto TestSim::SimPlant + TestSim::ScriptedI2CHook.
//
// Modeled on src/tests/sim/unit/motor_policy_harness.cpp (the MockMotor-style
// armor scenarios) + src/tests/sim/unit/velocity_pid_harness.cpp (the PID
// convergence-scenario style), per device-bus-tickets.md's DB-004
// acceptance criteria. Unlike those pre-port harnesses, this one #includes
// ONLY src/firm/devices/ headers (isolation invariant) plus plain C/C++
// stdlib -- no messages/*.h, no com/i2c_bus.h.
//
// Plain C++ program, hand-rolled assertions -- prints a PASS/FAIL line per
// scenario and exits nonzero if any assertion failed. Run by the pytest
// wrapper in test_devices_motor.py, which compiles this file together with
// tests/_infra/sim/sim_plant.cpp, src/tests/sim/plant/{wheel,otos}_plant.cpp,
// src/firm/devices/velocity_pid.cpp, and src/firm/devices/nezha_motor.cpp under
// -DHOST_BUILD, then runs the resulting binary via subprocess and asserts
// exit code 0.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "devices/device_config.h"
#include "devices/device_types.h"
#include "devices/i2c_bus.h"
#include "devices/motor_armor.h"
#include "devices/nezha_motor.h"
#include "devices/velocity_pid.h"
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

void checkVecEq(const std::vector<float>& actual,
                 const std::vector<float>& expected, const std::string& what) {
  bool ok = actual.size() == expected.size();
  if (ok) {
    for (size_t i = 0; i < actual.size(); ++i) {
      if (std::fabs(actual[i] - expected[i]) > 1e-6f) { ok = false; break; }
    }
  }
  if (!ok) {
    fail(what + " — write-call sequence mismatch");
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
  void setVelocity(float velocity) override { lastVelocityCmd = velocity; }
  void setDuty(float duty) override { lastDutyCmd = duty; }
  void setNeutral(Devices::Neutral) override {}
  void setPidEnabled(bool) override {}
  void applyGains(const Devices::Gains& gains, Devices::Opt<float> = {}) override {
    gains_ = gains;
  }
  const Devices::Gains& gains() const override { return gains_; }
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
  float velocityTarget() const override { return lastVelocityCmd; }
  float appliedDuty() const override { return mockAppliedDuty_; }
  bool connected() const override { return true; }
  void resetPosition() override { ++resetPositionCalls; }   // bare = hard, immediate
  void rebaseline() override { ++rebaselineCalls; }

  // --- Test-driving surface ---
  void setMockPosition(float position) { mockPosition_ = position; }
  void setMockVelocity(float velocity) { mockVelocity_ = velocity; }
  void setMockAppliedDuty(float duty) { mockAppliedDuty_ = duty; }

  // --- Call recording (scenarios assert on these directly) ---
  int resetPositionCalls = 0;
  int rebaselineCalls = 0;
  int tickCalls = 0;
  int reconfigureCalls = 0;
  float lastVelocityCmd = 0.0f;
  float lastDutyCmd = 0.0f;

 private:
  float mockPosition_ = 0.0f;
  float mockVelocity_ = 0.0f;
  float mockAppliedDuty_ = 0.0f;
  Devices::Gains gains_{};
};

// Config for the MotorArmor decorator scenarios below (MockMotor inner --
// only outputDeadband is functionally relevant here, MotorArmor::
// reconfigure() reads it straight into its own motionThreshold_ motion-gate
// cache; MockMotor ignores config entirely, so reversalDwell is moot).
// Sprint 114 ticket 003: MotorConfig's write-shaping fields are now plain
// required floats -- no more ctor/reconfigure() ship-default substitution --
// so this sets the historical ship-default value (0.03) explicitly, matching
// what a real robot's config always carries.
Devices::MotorConfig defaultArmorConfig() {
  Devices::MotorConfig cfg;
  cfg.outputDeadband = 0.03f;   // [-1,1] fraction
  return cfg;
}

// --- Write-shaping scenarios (real NezhaMotor — the dwell/deadband gate
// moved INTO the leaf's own writeShapedDuty(), 2026-07-18 restructure) ----

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
//    other nonzero-duty reversal would -- the boost happens BEFORE the
//    dwell/sign-change check and falls straight into it unchanged, so a
//    tiny reversal is never a backdoor around wedge protection. Replaces
//    the pre-114-005 "sub-deadband duty is immediate/unclamped" scenario,
//    whose own premise (every sub-deadband duty, any sign, always writes 0)
//    is exactly the defect this ticket fixes.
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
// one slack entry per cycle is always sufficient (tick()'s dispatch issues
// at most one writeRawDuty() call, which itself writes to the bus at most
// once) and harmless when the duty write doesn't land that cycle (write-
// on-change/throttle skip) — the unused slack entry is simply drained by a
// later cycle's write instead, and this fake's queueWrite() never checks
// the written payload, only address+order, so which "logical" write
// consumes which slot does not matter.
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
  cfg.velFiltAlpha = 1.0f;   // no smoothing — velocity() reflects each tick's raw difference-quotient exactly
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
//    expected position, and velocity() reflects the difference quotient
//    across two paired cycles.
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

// 6. PID-on chases a velocity target: with a simple first-order plant
//    stand-in (measured velocity chases the last-applied duty, one-tick
//    actuation lag — mirrors velocity_pid_harness.cpp's own plant model),
//    driven entirely through NezhaMotor's real tick()/armor/PID path, the
//    tracking error shrinks substantially from the first cycle to the last.
void scenarioPidOnChasesVelocityTarget() {
  beginScenario("PID-on chases a velocity target");
  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  const uint16_t wireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);

  Devices::MotorConfig cfg = baseNezhaConfig();
  cfg.velGains = Devices::Gains{/*kp=*/0.01f, /*ki=*/0.05f, /*kff=*/0.002f,
                                 /*iMax=*/1.0f, /*kaw=*/2.0f};
  cfg.velDeadband = 5.0f;   // [mm/s] well below the target — not in deadband

  Devices::NezhaMotor motor(plant, cfg);
  checkTrue(motor.pidEnabled(), "PID is enabled by default");

  const float target = 300.0f;   // [mm/s]
  const uint32_t dtMs = 20;      // [ms] cycle cadence
  const float dtS = 0.02f;       // [s]

  motor.setVelocity(target);

  float position = 0.0f;
  float measuredVel = 0.0f;
  uint64_t nowUs = 0;

  // Prime cycle — establishes lastPosition_/lastTickUs_, no velocity yet.
  scriptEncoderRequestCollect(bus, wireAddr, position);
  motor.requestSample();
  motor.tick(nowUs);

  float firstError = std::fabs(target - motor.velocity());

  const int kTicks = 400;
  for (int i = 0; i < kTicks; ++i) {
    // Plant response to whatever was actually forwarded to the bus last
    // cycle (appliedDuty() — reflects armor's slew/throttle/dwell gating,
    // exactly like the real wheel would).
    float duty = motor.appliedDuty();
    measuredVel += (duty * 500.0f - measuredVel) * 0.1f;
    position += measuredVel * dtS;

    nowUs += static_cast<uint64_t>(dtMs) * 1000;
    scriptEncoderRequestCollect(bus, wireAddr, position);
    motor.requestSample();
    motor.tick(nowUs);
  }

  float lastError = std::fabs(target - motor.velocity());

  checkTrue(lastError < firstError,
            "tracking error shrinks from the first cycle to the last");
  checkTrue(lastError <= firstError * 0.5f,
            "converged error is well below the initial error (no blow-up)");
  checkUintEq(bus.errCount(Devices::kNezhaDeviceAddr), 0, "no script under-run across the chase");
}

// 7. PID-off feeds raw duty through the armor unchanged: with PID disabled,
//    setVelocity()'s staged target is ignored and setDuty()'s staged raw
//    duty reaches appliedDuty() via the SAME armoredWrite() gate PID-on
//    uses — including a reversal, proving the armor gates identically in
//    both modes ("armor applies in both modes").
void scenarioPidOffRoutesRawDutyThroughArmorUnchanged() {
  beginScenario("PID-off feeds raw duty through the armor unchanged");
  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  const uint16_t wireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);

  Devices::MotorConfig cfg = baseNezhaConfig();
  cfg.slewRate = 100.0f;   // no slew clamping — isolates armor gating (reversal dwell) from the independent slew-cap concern
  Devices::NezhaMotor motor(plant, cfg);

  motor.setPidEnabled(false);
  motor.setVelocity(999.0f);   // staged but must be ignored while PID is off
  motor.setDuty(0.5f);

  uint64_t nowUs = 0;
  const float stationaryPosition = 0.0f;   // stationary plant — isolates the write-path behavior

  // Several well-spaced (>40ms apart) cycles so slew/throttle both allow
  // the raw duty to fully land.
  for (int i = 0; i < 5; ++i) {
    scriptEncoderRequestCollect(bus, wireAddr, stationaryPosition);
    motor.requestSample();
    nowUs += 50000;   // 50ms — clears the 40ms write-rate throttle every cycle
    motor.tick(nowUs);
  }

  checkFloatEq(motor.appliedDuty(), 0.5f,
               "raw staged duty reaches the write path (PID target ignored while PID is off)");

  // Now command a reversal while still PID-off: armor must gate it exactly
  // as it would in PID-on mode (write 0, hold through the dwell, then
  // resume in the new direction).
  motor.setDuty(-0.5f);
  scriptEncoderRequestCollect(bus, wireAddr, stationaryPosition);
  motor.requestSample();
  nowUs += 50000;
  motor.tick(nowUs);   // sign flip — armor writes 0, arms the 100ms dwell
  checkFloatEq(motor.appliedDuty(), 0.0f,
               "reversal while PID-off is dwelled by the SAME armor gate (write 0 first)");

  // Still mid-dwell (deadline is dwell-arm-time + 100ms).
  scriptEncoderRequestCollect(bus, wireAddr, stationaryPosition);
  motor.requestSample();
  nowUs += 30000;   // 30ms later — dwell (100ms) not yet elapsed
  motor.tick(nowUs);
  checkFloatEq(motor.appliedDuty(), 0.0f, "still held at 0 mid-dwell");

  // Dwell elapses — resumes the new direction.
  scriptEncoderRequestCollect(bus, wireAddr, stationaryPosition);
  motor.requestSample();
  nowUs += 100000;   // well past the 100ms deadline
  motor.tick(nowUs);
  checkFloatEq(motor.appliedDuty(), -0.5f, "dwell elapsed — resumes the new-direction raw duty");

  checkUintEq(bus.errCount(Devices::kNezhaDeviceAddr), 0, "no script under-run across the PID-off sequence");
}

// 8. Fresh-sample gate (HARDWARE-CONFIRMED DB-004 fix): the Nezha brick's
//    encoder register refreshes far slower (~80ms) than the loop's own
//    cycle (~16ms). Scripts the SAME raw encoder value for
//    several consecutive request/collect cycles (simulating the brick not
//    having refreshed yet), then a jump to a fresh value, repeated across
//    several refresh windows -- while ALSO commanding a duty so
//    appliedDuty() is nonzero throughout ("normal driving"). Proves
//    velocity() computes the CORRECT speed (step / real elapsed time since
//    the LAST FRESH sample, not this tick's own dt) on every fresh sample,
//    is never rejected as a glitch, is never left starved at/near 0, and
//    that the wedge detector does not false-latch (neither wedged() nor
//    wedgeSuspect()) across the run -- reproducing and proving the fix for
//    the exact bring-up-image symptom (vel=0.000 always, glitch count
//    climbing, wedged=1) while the wheel was physically moving.
void scenarioFreshSampleGateSurvivesSlowBrickRefreshUnderFastFiberCycle() {
  beginScenario("fresh-sample gate survives slow brick refresh under fast fiber cycle");
  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  const uint16_t wireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);

  Devices::MotorConfig cfg = baseNezhaConfig();   // velFiltAlpha=1.0 -- velocity() reflects each fresh sample's raw difference-quotient exactly
  Devices::NezhaMotor motor(plant, cfg);
  // Wedge detection lives in the MotorArmor DECORATOR now (2026-07-18
  // restructure) -- wrap the leaf and tick through the armor so the
  // detector actually observes the run; the wedge assertions below read
  // the armor's latches.
  Devices::MotorArmor armored(motor);
  // Byte-for-byte-behavior-preserving rename (114-001 Revision 1): SAME cfg
  // the wrapped motor was constructed with -- motor is fresh (mode_ ==
  // Mode::None), so reconfigure() always succeeds here.
  (void)armored.reconfigure(cfg);

  // Drive a raw duty throughout (PID off, to keep the plant simple/
  // deterministic) so appliedDuty() is nonzero -- exercises the
  // wedgeSuspect() ("commanded but not moving") path alongside the raw
  // wedged() latch, matching the ticket's "normal driving" concern.
  motor.setPidEnabled(false);
  motor.setDuty(0.4f);

  const uint64_t kFiberCycleUs = 16000;    // [us] ~16ms fiber cycle (DB-007/DB-008)
  const int kStaleCyclesPerRefresh = 4;    // 4 stale + 1 fresh = 5 cycles per ~80ms brick refresh
  const float kStepPerRefresh = 40.0f;     // [mm] realistic per-refresh position step (500mm/s-class motion)

  // Prime cycle: boot-anchor at position 0. Scripts an extra write slack
  // entry (scriptEncoderRequestCollect already provides one) to absorb the
  // very first duty write this tick's mode dispatch may also issue.
  uint64_t nowUs = 0;
  float position = 0.0f;
  scriptEncoderRequestCollect(bus, wireAddr, position);
  motor.requestSample();
  armored.tick(nowUs);
  checkFloatEq(motor.position(), 0.0f, "primed position is 0");
  checkUintEq(motor.encGlitchCount(), 0, "boot anchor is not counted as a glitch");

  for (int refresh = 0; refresh < 6; ++refresh) {
    // Stale cycles: the brick has NOT refreshed -- same raw value.
    for (int i = 0; i < kStaleCyclesPerRefresh; ++i) {
      nowUs += kFiberCycleUs;
      scriptEncoderRequestCollect(bus, wireAddr, position);   // unchanged raw
      motor.requestSample();
      armored.tick(nowUs);
    }

    // Fresh cycle: the brick has refreshed -- position jumps by a realistic
    // step. freshElapsed is ALWAYS exactly (kStaleCyclesPerRefresh+1) fiber
    // cycles here (every window is the same length), so the expected
    // velocity is constant across every refresh window.
    position += kStepPerRefresh;
    nowUs += kFiberCycleUs;
    scriptEncoderRequestCollect(bus, wireAddr, position);
    motor.requestSample();
    armored.tick(nowUs);

    const float freshElapsed = static_cast<float>(kFiberCycleUs) *
                                static_cast<float>(kStaleCyclesPerRefresh + 1) / 1e6f;
    const float expectedVel = kStepPerRefresh / freshElapsed;   // 40mm / 0.08s = 500mm/s

    checkFloatEq(motor.position(), position, "position reflects the fresh sample, not a stale intermediate one");
    checkFloatEq(motor.velocity(), expectedVel,
                 "velocity == step / real elapsed time since the LAST FRESH sample (not per-tick)", 1.0f);
    checkTrue(motor.velocity() > 100.0f,
              "velocity is not starved to ~0 by the intervening stale cycles");
  }

  checkUintEq(motor.encGlitchCount(), 0,
              "no false glitches across repeated stale-then-fresh refresh windows");
  checkTrue(!armored.wedged(),
            "raw wedge latch does not false-trigger across normal stale-then-fresh cycling");
  checkTrue(!armored.wedgeSuspect(),
            "motion-qualified wedge-suspect does not false-trigger while genuinely driving");
  checkUintEq(bus.errCount(Devices::kNezhaDeviceAddr), 0, "no script under-run across the run");
}

// 9. Boot anchor: the first-ever fresh sample may report a large lifetime-
//    accumulated raw count (hardware-observed: ~-33526mm) -- this is
//    NORMAL (an already-running brick's accumulated register), not a
//    glitch. It must be anchored directly (no diff-from-0 "glitch"), and
//    stale/fresh cycling immediately after boot must not re-glitch or
//    loop on the anchor, nor false-latch the wedge detector.
void scenarioBootAnchorAcceptsLargeInitialPositionWithoutGlitchOrWedge() {
  beginScenario("boot anchor accepts a large initial position without glitch or wedge");
  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  const uint16_t wireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);

  Devices::MotorConfig cfg = baseNezhaConfig();
  Devices::NezhaMotor motor(plant, cfg);
  // Wedge detection lives in the MotorArmor DECORATOR now (2026-07-18
  // restructure) -- wrap and tick through the armor so the detector runs.
  Devices::MotorArmor armored(motor);
  // Byte-for-byte-behavior-preserving rename (114-001 Revision 1): SAME cfg
  // the wrapped motor was constructed with -- motor is fresh (mode_ ==
  // Mode::None), so reconfigure() always succeeds here.
  (void)armored.reconfigure(cfg);

  const float kBootPosition = -33526.0f;   // [mm] hardware-observed lifetime-accumulated boot value

  scriptEncoderRequestCollect(bus, wireAddr, kBootPosition);
  motor.requestSample();
  armored.tick(0);

  checkFloatEq(motor.position(), kBootPosition,
               "boot anchor lands directly on the first sample, no diff-from-0 glitch");
  checkFloatEq(motor.velocity(), 0.0f, "no spurious velocity computed on the boot anchor itself");
  checkUintEq(motor.encGlitchCount(), 0, "boot anchor is never counted as a glitch");
  checkTrue(!armored.wedged(), "boot anchor alone does not latch the wedge");

  // A handful of stale cycles right after boot (brick hasn't refreshed
  // yet) must hold the anchor, not loop or re-glitch on it.
  uint64_t nowUs = 0;
  float position = kBootPosition;
  for (int i = 0; i < 3; ++i) {
    nowUs += 16000;
    scriptEncoderRequestCollect(bus, wireAddr, position);   // still stale
    motor.requestSample();
    armored.tick(nowUs);
  }
  checkFloatEq(motor.position(), kBootPosition, "position holds through stale post-boot cycles");
  checkUintEq(motor.encGlitchCount(), 0, "stale post-boot cycles are not glitches (simply not fresh)");
  checkTrue(!armored.wedged(), "stale post-boot cycles alone do not latch the wedge");

  // First genuinely fresh post-boot sample: real, realistic motion off the
  // large boot anchor computes correctly and is not misclassified.
  // +30mm over the 64ms since the anchor = ~469 mm/s -- deliberately below
  // kMaxPlausibleSpeed (halved 1000 -> 600 in the 2026-07-18 tuning pass;
  // the prior +40mm/625mm/s now trips the gate and holds velocity at 0,
  // which is the gate working, not the boot-anchor behavior under test).
  nowUs += 16000;
  position += 30.0f;
  scriptEncoderRequestCollect(bus, wireAddr, position);
  motor.requestSample();
  armored.tick(nowUs);
  checkFloatEq(motor.position(), position, "first post-boot fresh sample lands correctly");
  checkTrue(motor.velocity() > 100.0f, "first post-boot fresh sample yields a real (non-zero) velocity");
  checkUintEq(motor.encGlitchCount(), 0, "genuine post-boot motion off a large anchor is not a glitch");
  checkTrue(!armored.wedged(), "no wedge latch through the boot+stale+fresh sequence");
  checkUintEq(bus.errCount(Devices::kNezhaDeviceAddr), 0, "no script under-run across the boot sequence");
}

// 10. C1 fix (103-002, 2026-07-13 code review): a NAK'd STOP write (pct==0)
//     must NOT be latched as "already written" -- write-on-change (nezha_
//     motor.cpp's writeRawDuty()) must retry the SAME value next tick
//     instead of permanently suppressing it, and appliedDuty() must keep
//     reporting the PREVIOUS (still physically applied) duty until a write
//     actually succeeds -- the exact scenario the review flagged: a failed
//     stop leaving the watchdog's "re-assert Neutral every cycle" (the
//     loop's stale-target gate) permanently defeated.
void scenarioNakedStopWriteIsRetriedNextTickNotLatched() {
  beginScenario("a NAK'd stop write is retried next tick, not permanently latched-as-written");
  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  const uint16_t wireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);

  Devices::MotorConfig cfg = baseNezhaConfig();
  cfg.slewRate = 100.0f;   // no slew clamping -- isolates the write-status behavior
  Devices::NezhaMotor motor(plant, cfg);
  motor.setPidEnabled(false);

  // Starts at 50ms (not 0): writeRawDuty()'s write-rate throttle compares
  // this tick's nowUs against lastWriteTimeUs_'s zero-init value, so a
  // first tick at nowUs==0 would itself read as "0us since the last write"
  // and be throttled away — every other scenario in this file sidesteps
  // this the same way (see scenarioPidOffRoutesRawDutyThroughArmorUnchanged).
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
  nowUs += 50000;   // clears the 40ms write-rate throttle (stop is exempt anyway)
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

// 11. applyGains() takes effect on the SAME instance, same boot, no
//     reflash/reconstruction (106-002/SUC-025): with gains initially at 0
//     (PID output stays pinned at 0 regardless of error), the plant never
//     moves; applyGains() with real gains, called mid-run on the SAME
//     NezhaMotor object, makes it start chasing the target on the very
//     next tick.
void scenarioApplyGainsTakesEffectSameBootNoReflash() {
  beginScenario("applyGains() changes subsequent PID output on the same boot, no reflash");
  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  const uint16_t wireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);

  Devices::MotorConfig cfg = baseNezhaConfig();
  cfg.velDeadband = 5.0f;   // well below the target -- not in deadband
  // cfg.velGains left at Devices::Gains{}'s all-zero default -- PID output
  // is inert (0) regardless of error.

  Devices::NezhaMotor motor(plant, cfg);
  checkFloatEq(motor.gains().kp, 0.0f, "gains() reflects the constructed (zero) kp before applyGains()");

  const float target = 300.0f;   // [mm/s]
  const uint32_t dtMs = 20;      // [ms] cycle cadence
  const float dtS = 0.02f;       // [s]
  motor.setVelocity(target);

  float position = 0.0f;
  float measuredVel = 0.0f;
  uint64_t nowUs = 0;

  scriptEncoderRequestCollect(bus, wireAddr, position);
  motor.requestSample();
  motor.tick(nowUs);

  // Phase 1: zero gains -- output stays pinned at 0 despite a large error.
  for (int i = 0; i < 20; ++i) {
    float duty = motor.appliedDuty();
    measuredVel += (duty * 500.0f - measuredVel) * 0.1f;
    position += measuredVel * dtS;
    nowUs += static_cast<uint64_t>(dtMs) * 1000;
    scriptEncoderRequestCollect(bus, wireAddr, position);
    motor.requestSample();
    motor.tick(nowUs);
  }
  checkFloatEq(motor.appliedDuty(), 0.0f, "zero gains -- applied duty stays 0 despite a large error");
  checkFloatEq(motor.velocity(), 0.0f, "zero gains -- plant never moves");
  float errorBeforeGainChange = std::fabs(target - motor.velocity());

  // Live gain-apply, no reconstruction -- the SAME motor instance.
  Devices::Gains newGains{/*kp=*/0.01f, /*ki=*/0.05f, /*kff=*/0.002f,
                           /*iMax=*/1.0f, /*kaw=*/2.0f};
  motor.applyGains(newGains);
  checkFloatEq(motor.gains().kp, 0.01f, "gains() reflects the newly-applied kp immediately");

  // Phase 2: same instance, same plant state, same target -- now converges.
  for (int i = 0; i < 400; ++i) {
    float duty = motor.appliedDuty();
    measuredVel += (duty * 500.0f - measuredVel) * 0.1f;
    position += measuredVel * dtS;
    nowUs += static_cast<uint64_t>(dtMs) * 1000;
    scriptEncoderRequestCollect(bus, wireAddr, position);
    motor.requestSample();
    motor.tick(nowUs);
  }
  float errorAfterGainChange = std::fabs(target - motor.velocity());

  checkTrue(errorAfterGainChange < errorBeforeGainChange,
            "after applyGains(), tracking error shrinks on the SAME instance -- no reflash/reconstruction needed");
  checkTrue(motor.appliedDuty() != 0.0f, "post-applyGains() duty is no longer pinned at 0");
  checkUintEq(bus.errCount(Devices::kNezhaDeviceAddr), 0, "no script under-run across the gain-change sequence");
}

// 12. applyGains(): travelCalib only updates config_.wheelTravelCalib when
//     explicitly PRESENT (Opt<float>{has=true}); a gains-only call (default
//     travelCalib, absent) leaves it unchanged -- proves the two parameters
//     are independently gated, matching RobotLoop's own per-side
//     travel_calib vs. both-sides kp/ki/kff/iMax/kaw application split
//     (config.proto's MotorConfigPatch.side comment).
void scenarioApplyGainsTravelCalibAppliesWhenPresentOtherwiseUnchanged() {
  beginScenario("applyGains(): travelCalib updates wheelTravelCalib only when explicitly present");
  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  const uint16_t wireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);

  Devices::MotorConfig cfg = baseNezhaConfig();   // wheelTravelCalib = 1.0, fwdSign = 1
  Devices::NezhaMotor motor(plant, cfg);

  // Prime cycle: anchor at raw=0.
  scriptEncoderRequestCollect(bus, wireAddr, 0.0f);
  motor.requestSample();
  motor.tick(0);

  // Gains-only applyGains() (default travelCalib -- absent) must leave
  // wheelTravelCalib unchanged: a raw step scripted for "10.0mm at
  // calib=1.0" still lands at 10.0mm.
  motor.applyGains(Devices::Gains{0.02f, 0.0f, 0.0f, 1.0f, 1.0f});
  scriptEncoderRequestCollect(bus, wireAddr, 10.0f);
  motor.requestSample();
  motor.tick(20000);
  checkFloatEq(motor.position(), 10.0f,
               "gains-only applyGains() call leaves wheelTravelCalib unchanged (still 1.0)");

  // applyGains() WITH travelCalib=2.0 present -- the SAME raw register
  // value (this helper's own positionMm*10 raw convention, still assuming
  // calib=1.0 to construct the raw bytes) now decodes to DOUBLE the mm,
  // proving the side-selected field landed live, same boot, no reflash.
  Devices::Opt<float> travelCalib;
  travelCalib.has = true;
  travelCalib.val = 2.0f;
  motor.applyGains(motor.gains(), travelCalib);
  scriptEncoderRequestCollect(bus, wireAddr, 15.0f);   // raw=150 -- at calib=2.0 this decodes to 30.0mm
  motor.requestSample();
  motor.tick(40000);
  checkFloatEq(motor.position(), 30.0f,
               "travelCalib=2.0 doubles the SAME raw-derived reading into mm -- confirms the applied change");

  checkUintEq(bus.errCount(Devices::kNezhaDeviceAddr), 0, "no script under-run across the travelCalib sequence");
}

// 13. reconfigure() -- REVISION 1 (114-001, motor.h): guarded, whole-config
//     replacement. Succeeds and fully replaces config_ (fwdSign/
//     wheelTravelCalib/velGains -- NOT just the narrow applyGains() surface)
//     when the motor has never been commanded (mode_ == Mode::None); fails
//     and leaves config_ UNCHANGED when the motor is actively driving and
//     not at rest; succeeds again once the motor returns to rest. This is
//     the exact mechanism that resolves ticket 001's own thrown exception
//     (SimHarness::configureMotor() previously reached only MotorArmor's
//     own cached motionThreshold_, never the wrapped NezhaMotor's config_ --
//     see sprint.md's Architecture Revision 1 / Decision 6).
void scenarioReconfigureGuardedWholeConfigReplacement() {
  beginScenario("reconfigure(): succeeds pre-command (whole config_ replace), fails while driving "
                "and not at rest, succeeds again once at rest");
  TestSim::SimPlant plant;
  TestSim::ScriptedI2CHook bus(plant);
  const uint16_t wireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);

  Devices::MotorConfig cfg = baseNezhaConfig();   // port=1, fwdSign=1, wheelTravelCalib=1.0
  Devices::NezhaMotor motor(plant, cfg);

  // --- Step 1: never commanded (mode_ == Mode::None) -- reconfigure()
  //     succeeds and replaces config_ WHOLESALE, not just the narrow
  //     applyGains() surface (fwdSign/wheelTravelCalib have no other
  //     runtime setter -- this is the ONLY path that can change them
  //     post-construction). ---
  Devices::MotorConfig cfgA = baseNezhaConfig();
  cfgA.fwdSign = -1;
  cfgA.wheelTravelCalib = 2.0f;
  cfgA.velGains.kp = 0.77f;
  bool ok1 = motor.reconfigure(cfgA);
  checkTrue(ok1, "reconfigure() succeeds on a never-yet-commanded motor (mode_ == Mode::None)");
  checkFloatEq(motor.gains().kp, 0.77f, "kp took effect -- config_ was replaced, not just cached");

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
  //
  //     Every scripted sample from here on repeats the SAME raw position
  //     (10.0mm, matching step 1's own boot-anchor raw) -- an unchanged raw
  //     count is a STALE sample (NezhaMotor's own freshness gate, see
  //     nezha_motor.cpp's tick() step 2), so filteredVelocity_ never moves
  //     off its 0.0f default. This deliberately keeps the "at rest" checks
  //     below hinging ONLY on appliedDuty() -- exactly what setDuty(0.0f)
  //     controls -- rather than on an incidental velocity spike from a
  //     verification-only encoder jump. ---
  motor.setDuty(0.5f);
  scriptEncoderRequestCollect(bus, wireAddr, 10.0f);   // stationary (stale raw) -- unchanged raw
  motor.requestSample();
  motor.tick(50000);   // first write is slew-exempt -- lands immediately, nonzero
  checkTrue(motor.appliedDuty() != 0.0f, "setup: a real nonzero duty actually landed -- not at rest");

  Devices::MotorConfig cfgB = baseNezhaConfig();
  cfgB.fwdSign = 1;
  cfgB.wheelTravelCalib = 1.0f;
  cfgB.velGains.kp = 0.99f;
  bool ok2 = motor.reconfigure(cfgB);
  checkTrue(!ok2, "reconfigure() refuses while the motor is actively driving and not at rest");
  checkFloatEq(motor.gains().kp, 0.77f, "kp UNCHANGED after the refused reconfigure() -- still cfgA's value");

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
  checkFloatEq(motor.gains().kp, 0.99f, "kp now reflects cfgB -- reconfigure() took effect this time");

  // A FRESH scripted sample (a genuinely new raw value) now decodes under
  // cfgB's fwdSign=1/wheelTravelCalib=1.0, not cfgA's (-1/2.0) -- the
  // recovery reconfigure() genuinely took effect. This is the last tick in
  // the sequence, so the resulting velocity spike (from the stale-anchored
  // boot position) has no further "at rest" check downstream to disturb.
  scriptEncoderRequestCollect(bus, wireAddr, 15.0f);   // raw=150
  motor.requestSample();
  motor.tick(150000);
  checkFloatEq(motor.position(), 15.0f,
               "position now decodes under cfgB's fwdSign=1/wheelTravelCalib=1.0 -- the recovery "
               "reconfigure() genuinely took effect");

  checkUintEq(bus.errCount(Devices::kNezhaDeviceAddr), 0, "no script under-run across the reconfigure() sequence");
}

// 13. Explicit all-zero write shaping (reversalDwell=0/outputDeadband=0) is
//     a pure pass-through -- proves the Opt<float> -> float collapse
//     (sprint 114 ticket 003) did not silently change the MEANING of an
//     explicit zero. A duty that would have been zeroed under the old ship
//     default (0.03) lands unmodified, and a sign flip forwards immediately
//     with no intermediate zero write -- matching writeShapedDuty()'s own
//     documented "reversalDwell_ == 0 skips the dwell transition entirely"
//     contract (nezha_motor.cpp).
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

// 14. Settle-not-hunt sweep (sprint 114 ticket 005 -- Design Rationale
//     Decision 4's own explicit "prove it empirically, don't just assert
//     it" requirement, sprint.md). Decision 4's structural argument for why
//     this boost is safe where the deleted App::Pilot min-speed floor
//     (112-004) was not: the boost sits INSIDE NezhaMotor's own velocity-PID
//     closed loop, so real measured velocity feeds back every tick, AND
//     (the load-bearing half of the argument, re-derived here) the OUTER
//     loop's own commanded target is itself proportional to a residual
//     ERROR that shrinks as real motion closes it (App::Pilot's
//     `omega += headingKp_ * thetaErr` / `distanceKp_ * (sRef - sMeas)`) --
//     a boosted write is never invisible, so the very next tick's target is
//     already smaller. This is NOT the same claim as "NezhaMotor's own
//     velocity PID can smoothly HOLD an arbitrary constant velocity below
//     the deadband floor forever" -- it structurally cannot (the plant's
//     achievable-velocity set excludes the open interval (0,
//     outputDeadband_/kff), so a controller asked to hold a FIXED point
//     inside that gap has no choice but to dither between 0 and the floor).
//     An earlier version of this sweep drove NezhaMotor with a fixed
//     constant target and found exactly that dither (up to ~190 sign
//     reversals per case) -- a REAL finding, but about a scenario the
//     production system never actually poses: App::Pilot never asks for a
//     constant sub-floor velocity forever, it asks for a velocity
//     proportional to a shrinking residual. This sweep instead drives an
//     explicit OUTER P loop (`target = kpOuter * residual`, `residual -=
//     measuredVel * dt`) around the REAL NezhaMotor + writeShapedDuty()
//     fix, mirroring App::Pilot's own error-driven-target shape (the same
//     shape, not the full Pilot/Executor/HeadingSource graph, which the sim
//     SYSTEM test in deadband_terminal_correction_harness.cpp exercises
//     end-to-end) -- the actual claim under test.
//
//     Sweeps small initial residuals near a real dwell-tolerance
//     neighborhood (bench_test_config.cpp's own heading_dwell_tol=3deg at
//     trackWidth/2=64mm ~= 3.3mm; distance_tol=6mm) AND the outer loop's own
//     gain (the sweep axis the ticket's own acceptance criteria call out:
//     "across the model-reference feedback's current gain"), matching the
//     order of magnitude of bench_test_config.cpp's own
//     distance_kp=2.5/heading_kp=2.5. Every swept case must settle --
//     converge the residual into a small tolerance band and STAY there --
//     with AT MOST ONE sign reversal (one overshoot past the target, then
//     settle), never a sustained oscillation.
void scenarioDeadbandBoostSettlesNotHuntsAcrossResidualSweep() {
  beginScenario("deadband boost settles (not hunts) across a residual-error/outer-gain sweep");

  const float kResiduals[] = {3.0f, 5.0f, 8.0f, 12.0f, 20.0f};   // [mm] initial residual error
  const float kOuterKps[] = {1.5f, 2.5f, 4.0f};                  // [1/s] outer P loop gain

  constexpr float kKff = 0.002f;       // [duty per mm/s] matches bench_test_config.cpp's own convention
  constexpr float kMotorKp = 0.01f;    // NezhaMotor's own embedded velocity-PID proportional gain
  constexpr float kSettleTol = 1.5f;   // [mm] small tolerance band around a fully-closed residual
  constexpr int kTicks = 300;          // 6s of virtual time at dtMs=20 -- generous convergence budget
  constexpr int kTailTicks = 50;       // last 1s of the run -- "settled and STAYS settled"
  constexpr uint32_t dtMs = 20;
  constexpr float dtS = 0.02f;

  for (float kpOuter : kOuterKps) {
    for (float e0 : kResiduals) {
      TestSim::SimPlant plant;
      TestSim::ScriptedI2CHook bus(plant);
      const uint16_t wireAddr = static_cast<uint16_t>(Devices::kNezhaDeviceAddr << 1);

      Devices::MotorConfig cfg = baseNezhaConfig();   // outputDeadband=0.03, reversalDwell=100ms
      cfg.velGains = Devices::Gains{/*kp=*/kMotorKp, /*ki=*/0.0f, /*kff=*/kKff, /*iMax=*/1.0f, /*kaw=*/2.0f};
      cfg.velDeadband = 0.5f;   // [mm/s] small -- keeps the PID's P+FF terms engaged for every swept target

      Devices::NezhaMotor motor(plant, cfg);

      float residual = e0;   // [mm] the outer loop's own error -- shrinks with real measured motion
      float position = 0.0f;
      float measuredVel = 0.0f;
      uint64_t nowUs = 0;

      // Prime cycle -- establishes lastPosition_/lastTickUs_, no velocity yet.
      scriptEncoderRequestCollect(bus, wireAddr, position);
      motor.requestSample();
      motor.tick(nowUs);

      std::vector<float> residuals;
      residuals.reserve(kTicks + 1);
      residuals.push_back(residual);

      for (int i = 0; i < kTicks; ++i) {
        // Outer P loop -- App::Pilot's own error-driven-target shape,
        // applied directly to the residual (NezhaMotor has no heading/
        // distance concept of its own; the sim SYSTEM test exercises the
        // REAL App::Pilot graph end to end).
        float target = kpOuter * residual;
        motor.setVelocity(target);

        // Plant responds to whatever ACTUALLY landed on the wire last cycle
        // (appliedDuty() -- the real, shaped write, reversal-dwell/write-
        // throttle/slew all included, exactly as scenarioPidOnChasesVelocityTarget
        // above does), same simple first-order stand-in.
        float duty = motor.appliedDuty();
        measuredVel += (duty * 500.0f - measuredVel) * 0.1f;
        position += measuredVel * dtS;
        residual -= measuredVel * dtS;   // the outer error closes with REAL motion, not the target

        nowUs += static_cast<uint64_t>(dtMs) * 1000;
        scriptEncoderRequestCollect(bus, wireAddr, position);
        motor.requestSample();
        motor.tick(nowUs);

        residuals.push_back(residual);
      }

      char label[128];
      std::snprintf(label, sizeof(label), "e0=%.1fmm kpOuter=%.2f/s", static_cast<double>(e0),
                    static_cast<double>(kpOuter));

      // Overshoot count: a genuine sign reversal of the residual, gated on
      // the PRIOR residual being outside the settle band -- a sample
      // hovering near 0 and dithering sign trivially is exactly what
      // settled-and-holding looks like, not a hunt.
      int reversals = 0;
      for (size_t i = 1; i < residuals.size(); ++i) {
        bool prevOutside = std::fabs(residuals[i - 1]) > kSettleTol;
        bool signFlip = (residuals[i] > 0.0f) != (residuals[i - 1] > 0.0f);
        if (prevOutside && signFlip) ++reversals;
      }
      checkTrue(reversals <= 1, std::string("bounded overshoot (<=1 sign reversal) -- ") + label +
                                     ": saw " + std::to_string(reversals) + " reversal(s)");

      // Settled and STAYS settled -- the tail of the run holds inside the
      // tolerance band, proving convergence rather than one lucky crossing.
      bool tailSettled = true;
      float worstTailResidual = 0.0f;
      for (size_t i = residuals.size() - kTailTicks; i < residuals.size(); ++i) {
        worstTailResidual = std::max(worstTailResidual, std::fabs(residuals[i]));
        if (std::fabs(residuals[i]) > kSettleTol) tailSettled = false;
      }
      char tailMsg[192];
      std::snprintf(tailMsg, sizeof(tailMsg),
                    "settles and stays within +/-%.1fmm -- %s (worst tail residual %.2fmm)",
                    static_cast<double>(kSettleTol), label, static_cast<double>(worstTailResidual));
      checkTrue(tailSettled, tailMsg);
    }
  }
}

}  // namespace

int main() {
  scenarioReversalDwellWritesZeroThenHoldsThroughDeadline();
  scenarioOutputDeadbandBoostsSubDeadbandNonzeroDutyExactZeroStaysZero();
  scenarioStandstillGuardedResetGatesOnRestTicks();
  scenarioWedgeLatchAndSuspectDeriveAsBefore();
  scenarioRequestCollectPairingYieldsExpectedPositionVelocity();
  scenarioPidOnChasesVelocityTarget();
  scenarioPidOffRoutesRawDutyThroughArmorUnchanged();
  scenarioFreshSampleGateSurvivesSlowBrickRefreshUnderFastFiberCycle();
  scenarioBootAnchorAcceptsLargeInitialPositionWithoutGlitchOrWedge();
  scenarioNakedStopWriteIsRetriedNextTickNotLatched();
  scenarioApplyGainsTakesEffectSameBootNoReflash();
  scenarioApplyGainsTravelCalibAppliesWhenPresentOtherwiseUnchanged();
  scenarioReconfigureGuardedWholeConfigReplacement();
  scenarioExplicitZeroWriteShapingIsPassThrough();
  scenarioDeadbandBoostSettlesNotHuntsAcrossResidualSweep();

  if (g_failureCount == 0) {
    std::printf("OK: all devices motor scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the devices motor scenarios\n",
              g_failureCount);
  return 1;
}
